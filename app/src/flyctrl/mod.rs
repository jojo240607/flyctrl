//! 飞控应用：多任务实时架构（采样 / 控制 / 遥测 / 监控 分离）。
//!
//! 模块划分：
//!   - `control`   硬实时控制律任务（见 `control.rs`）
//!   - `sensors`   传感器采样任务（见 `sensors_task.rs`）
//!   - `telemetry` 遥测下行任务（见 `telemetry.rs`）
//!   - `uplink`    上行接收任务（见 `uplink.rs`）：地面站 -> 飞控命令通道
//!   - `monitor`   系统监控/心跳任务（见 `monitor.rs`）
//!
//! 任务优先级档位（见 `crate::abi`）：
//!   - `control`   prio=4  硬实时(priv=1, RTOS_RT_HARD) 周期 4ms：取最新样本 → EKF → FDIR → PID → PWM
//!   - `sensors`   prio=5  软实时(priv=1)              周期 2ms：采 IMU/RC/Baro/Mag/GPS → 写共享帧
//!   - `uplink`    prio=10 (priv=1)                    轮询 1ms：usb0.read 增量解析 → 命令路由
//!   - `telemetry` prio=12 (priv=1)                    周期 20ms：从最新估计发 MAVLink(标准)
//!   - `monitor`   prio=14 (priv=1)                    周期 1000ms：心跳日志 + 看门狗
//!
//! 跨任务共享数据经 RTOS 互斥量（`crate::rtos_sync::Mutex`）保护：
//!   - SENSOR_FRAME / SENSOR_SEQ：最新传感器样本（sensors 单写、control/monitor 读，seqlock 无锁）
//!   - EST_STATE   / EST_MTX  ：最新估计状态 + 健康（control 写、telemetry/monitor 读）
//!   - usb0 下行   / USB_TX_MTX：telemetry(12)/uplink(10) 双写者串行化（RTOS TX ring 无锁）
//!
//! 注意：mag(QMC5883L@0x0D) 当前读取会在单总线 I2C 事务中卡死（见 memory：flyctrl/app
//! mag.read 卡死 bug），待 joc-base I2C 驱动修复前，sensors 任务对 mag 走「缺失降级」路径，
//! 不实际发起读事务，避免拖垮采样线程。

pub mod control;
pub mod alloc_task;
pub mod rate_task;
pub mod nav_task;
pub mod wq_tasks;
pub mod safety_task;
pub mod rt_stat;
#[cfg(feature = "hil")]
pub mod hil_shmem;
pub mod pace;
pub mod sensors_task;
pub mod telemetry;
pub mod uplink;

use flyctrl_core::vehicle::{ImuSample, PosSample, Quaternion, RcInput, VehicleState};
use flyctrl_core::fdir::Health;
use flyctrl_core::units::{Meter, MeterPerSecond, RadianPerSecond};

use rtos_app_sdk::abi::RTOS_PRIO_BH_HIGH;
use rtos_app_sdk::rtos::{spawn_rt, Mutex, Semaphore, RTOS_RT_HARD, RTOS_RT_NONE, RTOS_RT_SOFT};

/// [性能测量] 控制拍计数器（固定 VMA 0x2000F000，测试直读）。见 linker/app.ld。
#[link_section = ".app_ctrltick"]
pub static mut CTRL_TICKS: u32 = 0;
/// [PERF] 控制循环分段标记（0=帧读后 1=设定点后 2=step_hil后 3=PWM后 4=循环末）。
/// 测试挂写钩子取两次写之间的 retired 差，定位单拍耗时归属。
#[used]
#[link_section = ".app_ctrltick"]
pub static mut CTRL_PHASE: u32 = 0;

/* ===================== 共享数据 ===================== */

/// 最新传感器样本帧（sensors 写、control 读）。mag 缺失时为 None。
pub struct SensorFrame {
    pub imu: Option<ImuSample>,
    pub rc: RcInput,
    pub gps: Option<PosSample>,
    pub baro_alt: Option<f32>,
    /// 机体系三轴磁场（QMC5883L 读数；缺失降级为 None）。
    pub mag: Option<[f32; 3]>,
    /// ★design.md §5：`wq:sensors` 发布的 **IMU topic**（带时间戳），供 ekf/attitude/rate 消费：
    ///   · `gyro_vel`  = `vehicle_angular_velocity`（**控制器侧**：陷波→低通）→ 速率环用 ✓
    ///   · `accel_filt`= `vehicle_acceleration`（陷波→低通）→ 估计器用 ✓
    ///   · `gyro_raw`  = 未滤波陀螺（PX4：EKF2 用未滤波陀螺）→ 估计器用 ✓
    ///   · `ts_cyc`    = 采样**硬件时间戳**（DWT CYCCNT，design.md §7）
    pub gyro_vel: [f32; 3],
    pub accel_filt: [f32; 3],
    pub gyro_raw: [f32; 3],
    pub ts_cyc: u32,
    pub imu_ok: bool,
    pub gps_ok: bool,
    pub baro_ok: bool,
    pub mag_ok: bool,
    pub armed: bool,
}

impl SensorFrame {
    const fn empty() -> Self {
        SensorFrame {
            imu: None,
            rc: RcInput { roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.0, armed: false, mode: 0, fresh: false },
            gps: None,
            baro_alt: None,
            mag: None,
            gyro_vel: [0.0; 3],
            accel_filt: [0.0; 3],
            gyro_raw: [0.0; 3],
            ts_cyc: 0,
            imu_ok: false,
            gps_ok: false,
            baro_ok: false,
            mag_ok: false,
            armed: false,
        }
    }
}

/// 最新估计状态 + 健康（control 写、telemetry/monitor 读）。
///
/// `#[repr(C)]`：布局固定（与 [`VehicleState`] 同为外部内存读取寻址）。
#[repr(C)]
pub struct EstState {
    pub est: VehicleState,
    pub health: Health,
    pub armed: bool,
}

impl EstState {
    const fn empty() -> Self {
        EstState {
            est: VehicleState {
                time_boot_ms: 0,
                pos: [Meter(0.0), Meter(0.0), Meter(0.0)],
                vel: [MeterPerSecond(0.0), MeterPerSecond(0.0), MeterPerSecond(0.0)],
                att: Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
                omega: [RadianPerSecond(0.0), RadianPerSecond(0.0), RadianPerSecond(0.0)],
                airspeed: MeterPerSecond(0.0),
                accel_bias: [0.0; 3],
            },
            health: Health::Degraded,
            armed: false,
        }
    }
}

/// 最新估计状态 + 健康（control 写、telemetry/monitor 读）。
/// 注意：含 `health: Health` enum（非零判别式），若用 `EstState::empty()` 作初始化器会带
/// 非零字节、被 Rust 放进 `.data` 段；而 App 链接契约（XIP + 仅 .bss，见 app.ld）下 `.data`
/// 会落到 Flash，运行时写它即写 Flash → BusFault。故用 `#[link_section=".bss.est_state"]`
/// 强制进 .bss，并用 `zeroed()` 作全零初始化器（Health::Healthy=0 为合法变体，zeroed 安全）；
/// 真正的初值在 `spawn_flyctrl` 里运行时 `EST_STATE = EstState::empty()` 填充——写落 RAM 安全。
/// 段名用 `.rust_bss`（独立顶层段，非 `.bss.*` 子类）以便主链接脚本把它收进 APP_RAM，
/// 释放主 SRAM 给系统堆。
#[link_section = ".rust_bss"]
pub static mut EST_STATE: EstState = unsafe { core::mem::zeroed() };

/// 全局共享帧 + 互斥量（静态存储，启动时 init）。
/// 注意：与 `EST_STATE` 同理，`SensorFrame::empty()` 构造的 `Option<T>` 因无 niche，
/// 其 padding 字节可能非零，会被 Rust 放入 `.data` 段而落到 Flash；sensors 任务写
/// 它会触发 BusFault。故强制 `.rust_bss` + `zeroed()`（全零合法初值）。
#[link_section = ".rust_bss"]
pub static mut SENSOR_FRAME: SensorFrame = unsafe { core::mem::zeroed() };
/// 共享帧顺序计数器（seqlock 写标记）：sensors/uplink 写前 +1(奇)、写后 +1(偶)。
/// 读者（control，prio4 高于所有写者）【不校验】 seq：单次读取即原子（读过程不会被
/// 低优先级写者抢占），但可能读到"写者被抢占中途"的新老混合快照，下一拍自然恢复一致
/// （此即 HIL 注入期间实测踩坑的根因：绝不能 `continue` 忙等重试，否则低优先级写者
/// 饿死 → 整机卡死，详见 control.rs 读帧处注释）。seq 仅作可见性护栏（compiler_fence
/// 的发布/获取锚点）。若未来出现优先级高于 control 的写者，此护栏会静默失效，需重审。
/// 之所以不用 Mutex(二值信号量)：本 RTOS ABI 无真互斥量，二值信号量在 control(硬实时)
/// 与 sensors(相邻更低优先级) 临界区被抢占的场景下争用不安全，会导致调度器损坏。
#[link_section = ".rust_bss"]
pub static mut SENSOR_SEQ: u32 = 0;
/// ★design.md §7：**IMU 1kHz 样本环形**（生产者=采样，消费者=`rate`(latest)/`ekf`(排空)）。
#[link_section = ".rust_bss"]
pub static mut IMU_RING: flyctrl_core::imu_ring::ImuRing = flyctrl_core::imu_ring::ImuRing::new();

/// ★design.md §3：BMI088 **data-ready INT**（ISR → 任务）节拍事件 + 计数（诊断）。
#[link_section = ".rust_bss"]
pub static mut IMU_DRDY_SEM: rtos_app_sdk::abi::rtos_sem_t =
    rtos_app_sdk::abi::rtos_sem_t { count: 0, limit: 0, waitq: core::ptr::null_mut() };
#[link_section = ".rust_bss"]
pub static mut IMU_DRDY_CNT: u32 = 0;

/// DRDY ISR（EXTI line4 / IRQ10）：**只 `sem_give`**（绝不做阻塞 SPI ✗）。
extern "C" fn imu_drdy_isr(_ctx: *mut core::ffi::c_void) {
    unsafe {
        IMU_DRDY_CNT = IMU_DRDY_CNT.wrapping_add(1);
        // ★design.md §3：DRDY → L0 内采样 IMU（读→ImuRing+topic）
        crate::flyctrl::sensors_task::imu_sample_step();
    }
}

/// 装配 DRDY：打开 `exti_imu`（配 SYSCFG/EXTI）+ 挂 IRQ10 ISR（design.md §3）。
pub fn init_imu_drdy() {
    unsafe {
        if let Some(f) = rtos_app_sdk::abi::slot().sem_init {
            f(core::ptr::addr_of_mut!(IMU_DRDY_SEM), 0, 1);
        }
        IMU_DRDY_CNT = 0;
    }
    if let Some(mut d) = rtos_app_sdk::device::Device::open("exti_imu") {
        let _ = d.open_dev();
    } else {
        rtos_app_sdk::warn!(tag: "flyctrl", "exti_imu not available -> DRDY INT disabled");
        return;
    }
    let _ = rtos_app_sdk::irq::attach_and_enable(10, imu_drdy_isr, core::ptr::null_mut());
    rtos_app_sdk::info!(tag: "flyctrl", "IMU DRDY INT armed (GPIOE4/EXTI4/IRQ10)");
}
#[link_section = ".rust_bss"]
pub static mut EST_MTX: Mutex = Mutex::uninit();
/// usb0 下行写互斥：telemetry(prio12) 与 uplink(prio10) 共用同一 usb0 TX ring，
/// 而 RTOS 侧 `usb_stream_write`/`rb_write` 无锁（注释假设"caller task model 单生产者"），
/// 两个写者并发会把 MAVLink 帧在 ring 中交错损坏。故 app 层用此互斥串行化所有 usb0 写。
#[link_section = ".rust_bss"]
pub static mut USB_TX_MTX: Mutex = Mutex::uninit();

/// HIL 事件信号量：事件驱动闭环的同步原语（一输入一输出、不依赖 control 自身时钟）。
/// uplink 写完一帧 HIL_SENSOR 后 `give()`；control 阻塞 `wait()`，收到一帧执行一拍
/// `step_hil`（与 SIL 的"每物理步一拍"推模式 1:1 对齐）。初值 0，仅 HIL 编译期存在。
#[cfg(feature = "hil")]
#[link_section = ".rust_bss"]
pub static mut HIL_EVT: Semaphore = Semaphore::uninit();

/// ★C3 速率设定值共享（姿态层 control 写、速率层 rate_task 读）。
///
/// 优先级论证（同 SENSOR_FRAME ✓）：写者 control(prio=4) **高于**读者 rate_task(prio=5)
/// ⇒ 读者运行期间写者不会运行 ⇒ 读必为**完整写**，无需 seqlock 重试 ✓。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RateCmd {
    /// 期望机体角速率 (p, q, r)，rad/s（姿态层已按 `RATE_MAX_DPS` 限幅 ✓）。
    pub rates: [f32; 3],
    /// 集体推力（悬停油门基值）。
    pub thrust: f32,
    /// 非 0 = 有效（健康/解锁/初始化闸全开 ⟺ true ✓）；0 ⇒ 速率层零输出 ✓。
    pub valid: u32,
}
#[link_section = ".rust_bss"]
pub static mut RATE_CMD: RateCmd = unsafe { core::mem::zeroed() };

/// ★design.md §4：**`rate_ctrl` → `control_allocator`** 的交接 topic。
/// PX4 同构：`vehicle_torque_setpoint`（三轴力矩）+ `vehicle_thrust_setpoint`（归一推力）。
/// 优先级：写者 `rate`(prio=2) 高于读者 `alloc`(prio=3) ⇒ 读必为完整写 ✓。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RateTt {
    pub torque: [f32; 3],
    pub thrust: f32,
    /// 非 0 = 有效（健康/解锁闸全开）；0 ⇒ 分配器零输出 ✓。
    pub valid: u32,
    /// 发布序号（诊断：确认分配器每拍都拿到新值 ✓）。
    pub seq: u32,
}
#[link_section = ".rust_bss"]
pub static mut RATE_TT: RateTt = unsafe { core::mem::zeroed() };

/// ★C1 共享设定点（姿态层 `control` 写、EKF 层 `rate_task` 读）。
/// 优先级：写者 control(4) 高于读者 rate(5) ⇒ 读必为完整写 ✓（同 SENSOR_FRAME 论证 ✓）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct SetpointShared {
    pub sp: flyctrl_core::controller::Setpoint,
    /// 非 0 = 有效。
    pub valid: u32,
}
#[link_section = ".rust_bss"]
pub static mut SETPOINT: SetpointShared = unsafe { core::mem::zeroed() };

/// ★C1 共享 EKF 诊断（`rate_task` 写、`control` 读）：速度环 D 项 world_accel + mag 滤波器内部量。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct HilDiag {
    /// 估计世界系加速度（NED，m/s²）—— 姿态层速度环 D 项（PX4 `states.acceleration` ✓）。
    pub world_accel: [f32; 3],
    /// mag 滤波器内部量（`mag_i`/`mag_b`/标志/计数）—— 遥测诊断 DBG_MAGI ✓。
    pub mag_i: [f32; 3],
    pub mag_b: [f32; 3],
    pub yaw_aligned: u32,
    pub mag_disturbed: u32,
    pub mag_applied: u32,
    pub mag_skipped: u32,
    pub mag_hdg_innov_lpf: f32,
    pub last_mag_yaw_innov: f32,
    /// 估计偏航（rad，诊断 ✓）。
    pub yaw_rad: f32,
    /// 本拍 motor 指令（rate_task 写，供遥测 `set_actuator_cmd` / throttle ✓）。
    pub motor: [f32; 4],
    /// 本拍是否有效（门控 ✓）。
    pub gated: u32,
}
#[link_section = ".rust_bss"]
pub static mut HIL_DIAG: HilDiag = unsafe { core::mem::zeroed() };

/// ★P0-3b 共享**姿态设定点**（L3 `nav` 写、L2 `control` 读）。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct AttSp {
    pub q: [f32; 4],
    pub thrust: f32,
    pub valid: u32,
}
#[link_section = ".rust_bss"]
pub static mut ATT_SP: AttSp = unsafe { core::mem::zeroed() };

/* ===================== 任务栈 ===================== */

const STACK_CTRL: usize = 8192; // ★C1：控制任务已不再运行 EKF（迁至 1kHz rate_task）⇒ 仅姿态/速度外环 + 设定点构造，回降至 8KB（原 20KB 为 EKF 大矩阵；现释放给 rate_task ✓）。原注：控制律含 EKF+PID：EKF step 各更新函数有 4x400B 局部矩阵（a/ap/apat/krkt=1.6KB）+ propagate 1.2KB + 对象本身与调用链，峰值实测 >3KB。
const STACK_TELEM: usize = 4096; // 遥测 encode 3 个 MAVLink 帧(heartbeat/local_pos/sys_status)栈使用大，1024 疑似栈溢出导致 telem 卡住不写 usb0，提到 4096
const STACK_UPLINK: usize = 4096; // 上行 poll_read+feed+decode 栈使用大，实测 1024 栈溢出导致系统 fault，提到 4096
const STACK_SAFETY: usize = 2048; // L1 safety_monitor (tiny stack)
const STACK_ALLOC: usize = 4096; // ★design.md §4：L1 control_allocator（仅分配+PWM）
const STACK_RATE: usize = 4096; // ★P0-3：L1 薄速率环（仅 rate_step+混控+PWM，无 EKF）⇒ 4KB 充足 ✓
const STACK_EKF: usize = 14336; // ★P0-3：L2 EKF 任务（大矩阵：propagate 1.2KB + update 1.6KB + 调用链）⇒ 留足余量 ✓

#[link_section = ".app_stacks"]
static mut STACK_TELEM_BUF: [u8; STACK_TELEM] = [0u8; STACK_TELEM];
#[link_section = ".app_stacks"]
static mut STACK_UPLINK_BUF: [u8; STACK_UPLINK] = [0u8; STACK_UPLINK];
#[link_section = ".app_stacks"]
static mut STACK_ALLOC_BUF: [u8; STACK_ALLOC] = [0u8; STACK_ALLOC];
static mut STACK_RATE_BUF: [u8; STACK_RATE] = [0u8; STACK_RATE];
#[link_section = ".app_stacks"]
static mut STACK_SAFETY_BUF: [u8; STACK_SAFETY] = [0u8; STACK_SAFETY];
#[link_section = ".app_stacks"]
static mut STACK_EKF_BUF: [u8; STACK_EKF] = [0u8; STACK_EKF];

/* ===================== 启动 ===================== */

/// 构建 "pwmN"（N=0..3）设备名（含结尾 \0）。
///
/// 注意：不能用带非零初值的 `static NAMES`（`.rust_data`）——App 独立镜像的
/// `.data` 初值未被可靠打包进 app.bin（LMA 偏移超出 bin 长度），运行时自拷贝
/// 读到的全是 0，导致 `Device::open("pwm0")` 用空名字查表失败。改为直接返回
/// 字符串字面量（落在 `.rodata`，XIP 只读、无需拷贝，已被验证可靠）。
pub(crate) fn make_name(n: u8) -> &'static str {
    match n {
        0 => "pwm0",
        1 => "pwm1",
        2 => "pwm2",
        3 => "pwm3",
        _ => "",
    }
}

/// 初始化共享互斥量并创建四任务。由 lib.rs::rust_app_start 调用。
pub fn spawn_flyctrl() {
    // 运行时填充 EST_STATE（其 static 被强制进 .bss，初始化器已丢弃，必须此处填充）。
    unsafe { (*core::ptr::addr_of_mut!(EST_STATE)) = EstState::empty(); }

    // 运行时填充 G_PARAM_VALS（同 .rust_bss 初值被清零，必须用默认增益显式初始化，
    // 否则 PARAM_REQUEST_LIST 下发的参数值全 0）。
    uplink::init_param_defaults();

    // 初始化互斥量（天花板优先级取可能锁定者的最高 prio）
    unsafe {
        // SENSOR_FRAME 改用 seqlock（见 SENSOR_SEQ），不再需要 SENSOR_MTX。
        EST_MTX.init(RTOS_PRIO_BH_HIGH);    // control(4)/telem(12)
        rtos_app_sdk::info!(tag: "flyctrl", "EST_MTX init count={} (expect 1, 否则互斥未生效→telem 死等)",
                     EST_MTX.debug_count());
        // usb0 写者仅 telem(12)/uplink(10)，最高持锁者 prio=10。
        USB_TX_MTX.init(10);
        rtos_app_sdk::info!(tag: "flyctrl", "USB_TX_MTX init count={} (expect 1)",
                     USB_TX_MTX.debug_count());
    }
    // HIL 事件信号量：初值 0、上限 1。control 阻塞等待、uplink 注入后投递。
    #[cfg(feature = "hil")]
    unsafe {
        HIL_EVT.init();
        rtos_app_sdk::info!(tag: "flyctrl", "HIL_EVT init count={} (expect 0, 事件驱动闭环)",
                     HIL_EVT.debug_count());
    }

    // 日志消费者任务（低优先，drain 日志 ring → uart0）。必须最先创建，确保后续
    // 业务任务的 info! 日志能及时被输出（只写 ring，绝不阻塞业务任务）。
    // ★design.md §6：日志消费归 **L3 WorkItem**（"日志 事件驱动·可丢弃"）——
    //   不再创建独立 log 线程 ✓；改为打开 ring 并由 L3 `wq:log` item 周期 drain。
    rtos_app_sdk::log::activate_ring_consumer();

    // ★★§5.220：把"栈竞技场"范围交给 MAVLink 编码器的**悬垂缓冲守卫** ✓
    //   只按"地址 < SP"判会误伤静态/.bss 缓冲 ✗（它们天生在栈下方 ✓）⇒ 必须显式给范围 ✓
    // ★§5.220/§5.221：仅在 `buf-guard` feature 下接线 ✓（默认关 ⇒ 固件布局与崩溃构建一致 ✓）
    #[cfg(feature = "buf-guard")]
    {
        unsafe {
            let lo = STACK_RATE_BUF.as_ptr() as usize;
            let hi = STACK_UPLINK_BUF.as_ptr() as usize + STACK_UPLINK;
            flyctrl_core::comm::mavlink::GUARD_LO = lo;
            flyctrl_core::comm::mavlink::GUARD_HI = hi;
        }
    }

    // control：硬实时 prio=4, priv=1, RTOS_RT_HARD
    // ★design.md L2：`control` 已改为**工作队列 WorkItem**（见 `wq_tasks`）——不再建线程 ✓。
    // sensors：软实时 prio=5, priv=1
    // ★design.md：`sensors` 不再是线程 —— IMU 采样归 L0 ISR（#2c）、融合归 L2 WorkItem
    //   `wq:sensors`（见 `wq_tasks`）✓。
    // telemetry：prio=12, priv=1
    // telemetry：prio=12, priv=1
    // ★design.md：`telem` 不再是线程 —— 归 L3 WorkItem `wq:l3`（见 `wq_tasks`）✓。
    // uplink：prio=10, priv=1（轮询 usb0.read，低于 sensors 不挤占采样，高于 telemetry 优先处理命令）
    // uplink：prio=10, priv=1（轮询 usb0.read，低于 sensors 不挤占采样，高于 telemetry 优先处理命令）
    // ★design.md：`uplink` 不再是线程 —— 归 L3 WorkItem `wq:l3`（见 `wq_tasks`）✓。
    // ★design.md P2-3：EKF 改为**定时器驱动的队列项**（不再用线程）——见 `wq_tasks`。
    //   本任务仅做一次性装配（静态 `HilContext` + 建 `wq:ekf`/`wq:att` + 250Hz 定时器）后常驻。
    spawn_rt(
        "wqsetup",
        wq_tasks::setup_entry,
        5,
        unsafe { STACK_EKF_BUF.as_mut_ptr() },
        STACK_EKF,
        1,
        RTOS_RT_NONE,
        0,
        0,
    );

    // ★design.md L1 `rate`：**薄硬实时线程**（1kHz, prio 4 ≤ BH_HIGH ⇒ HARD ✓）
    //   速率环 + 控制分配 + PWM；EKF 已剥离到 L2 `ekf` ✓ ⇒ wcet 应≈0 违约 ✓。
    spawn_rt(
        "safety",
        safety_task::safety_entry,
        4,
        unsafe { STACK_SAFETY_BUF.as_mut_ptr() },
        STACK_SAFETY,
        1,
        RTOS_RT_HARD,
        2,
        1,
    );
    // ★design.md §4 L1 `control_allocator`：独立硬实时线程（1kHz, prio 3, HARD）。
    //   prio 3 位于 rate(2) 之下、safety(4) 之上 ⇒ 三条 L1 线程均**高于** L2 worker(5) ✓。
    spawn_rt(
        "alloc",
        alloc_task::alloc_entry,
        3,
        unsafe { STACK_ALLOC_BUF.as_mut_ptr() },
        STACK_ALLOC,
        1,
        RTOS_RT_HARD,
        1, // deadline：1kHz ⇒ 1ms
        1,
    );
    spawn_rt(
        "rate",
        rate_task::rate_entry,
        2, // ★修正：L1 最高（< ekf=3 ✓）
        unsafe { STACK_RATE_BUF.as_mut_ptr() },
        STACK_RATE,
        1,
        RTOS_RT_HARD,
        1, // deadline：1kHz ⇒ 1ms
        1, // wcet：1ms 预算
    );

    // ★design.md L3 `nav`：位置/速度外环（50Hz, prio 8, 非实时）—— 产出**姿态设定点** `ATT_SP` ✓
    // ★design.md：`nav` 不再是线程 —— 归 L3 WorkItem `wq:l3`（见 `wq_tasks`）✓。

    // ★design.md §3：DRDY INT 的 arm 移入 `wq_tasks::setup()`（须在 `sensors_init()` **之后**，
    //   否则 ISR 早期触发时静态未初始化 ⇒ 崩 ✗）。
    rtos_app_sdk::info!(tag: "flyctrl", "spawned 5 tasks: control/sensors/telem/uplink/rate (1kHz 速率环)");
}
