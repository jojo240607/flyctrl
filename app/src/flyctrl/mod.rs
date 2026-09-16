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
#[cfg(feature = "hil")]
pub mod hil_shmem;
pub mod sensors_task;
pub mod telemetry;
pub mod uplink;

use flyctrl_core::vehicle::{ImuSample, PosSample, Quaternion, RcInput, VehicleState};
use flyctrl_core::fdir::Health;
use flyctrl_core::units::{Meter, MeterPerSecond, RadianPerSecond};

use rtos_app_sdk::abi::RTOS_PRIO_BH_HIGH;
use rtos_app_sdk::rtos::{spawn_rt, Mutex, Semaphore, RTOS_RT_HARD, RTOS_RT_NONE};

/* ===================== 共享数据 ===================== */

/// 最新传感器样本帧（sensors 写、control 读）。mag 缺失时为 None。
pub struct SensorFrame {
    pub imu: Option<ImuSample>,
    pub rc: RcInput,
    pub gps: Option<PosSample>,
    pub baro_alt: Option<f32>,
    /// 机体系三轴磁场（QMC5883L 读数；缺失降级为 None）。
    pub mag: Option<[f32; 3]>,
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

/* ===================== 任务栈 ===================== */

const STACK_CTRL: usize = 8192; // 控制律含 EKF+PID：EKF step 各更新函数有 4x400B 局部矩阵（a/ap/apat/krkt=1.6KB）
                               // + propagate 1.2KB + 对象本身与调用链，峰值实测 >3KB；3072 时栈溢出→返回地址
                               // 被数据覆盖→UsageFault(UNDEFINSTR/INVSTATE)→USB EP0 失服→HIL 端口 SetCommState 超时。
                               // 8KB 含 GPS 更新路径余量充足（APP_RAM 余 ~103KB）。
const STACK_SENS: usize = 3584; // 采样含回放+帧拷贝：实测峰值 > 3072（原靠 monitor 缓冲垫着才不崩），提到 3584 自洽
const STACK_TELEM: usize = 4096; // 遥测 encode 3 个 MAVLink 帧(heartbeat/local_pos/sys_status)栈使用大，1024 疑似栈溢出导致 telem 卡住不写 usb0，提到 4096
const STACK_UPLINK: usize = 4096; // 上行 poll_read+feed+decode 栈使用大，实测 1024 栈溢出导致系统 fault，提到 4096

#[link_section = ".rust_bss"]
static mut STACK_CTRL_BUF: [u8; STACK_CTRL] = [0u8; STACK_CTRL];
#[link_section = ".rust_bss"]
static mut STACK_SENS_BUF: [u8; STACK_SENS] = [0u8; STACK_SENS];
#[link_section = ".rust_bss"]
static mut STACK_TELEM_BUF: [u8; STACK_TELEM] = [0u8; STACK_TELEM];
#[link_section = ".rust_bss"]
static mut STACK_UPLINK_BUF: [u8; STACK_UPLINK] = [0u8; STACK_UPLINK];

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
    rtos_app_sdk::log::spawn_log_task();

    // control：硬实时 prio=4, priv=1, RTOS_RT_HARD
    spawn_rt(
        "control",
        control::control_entry,
        RTOS_PRIO_BH_HIGH,
        unsafe { STACK_CTRL_BUF.as_mut_ptr() },
        STACK_CTRL,
        1,
        RTOS_RT_HARD,
        0,
        0,
    );
    // sensors：软实时 prio=5, priv=1
    spawn_rt(
        "sensors",
        sensors_task::sensors_entry,
        5,
        unsafe { STACK_SENS_BUF.as_mut_ptr() },
        STACK_SENS,
        1,
        RTOS_RT_NONE,
        0,
        0,
    );
    // telemetry：prio=12, priv=1
    spawn_rt(
        "telem",
        telemetry::telemetry_entry,
        rtos_app_sdk::abi::RTOS_PRIO_MAIN,
        unsafe { STACK_TELEM_BUF.as_mut_ptr() },
        STACK_TELEM,
        1,
        RTOS_RT_NONE,
        0,
        0,
    );
    // uplink：prio=10, priv=1（轮询 usb0.read，低于 sensors 不挤占采样，高于 telemetry 优先处理命令）
    spawn_rt(
        "uplink",
        uplink::uplink_task,
        10,
        unsafe { STACK_UPLINK_BUF.as_mut_ptr() },
        STACK_UPLINK,
        1,
        RTOS_RT_NONE,
        0,
        0,
    );

    rtos_app_sdk::info!(tag: "flyctrl", "spawned 4 tasks: control/sensors/telem/uplink (monitor merged into telem)");
}
