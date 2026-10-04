//! 控制律硬实时任务（核心，4ms 周期）。
//!
//! 流程：取最新传感器帧 → EKF → FDIR → PID → PWM。
//! 读 SENSOR_FRAME（经 seqlock，见 `SENSOR_SEQ`）、写 EST_STATE（经 EST_MTX）。

use core::ffi::c_void;

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::{Controller, PidController, Setpoint};
// ★**默认估计器改为 ESKF**（迁移计划步 3 ✓；全表验收 0/10 劣于 Legacy ✓，见 docs/c1-migration-plan.md ✓）
use flyctrl_core::estimator::select::AnyEstimator; // ★§5.272：`AnyEstimatorKind` 已随 Legacy 删除 ✓
use flyctrl_core::fdir::Health;
use flyctrl_core::hil::{HilContext, SimImu};
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, Second};
use flyctrl_core::vehicle::{Quaternion, RcInput, VehicleState};

use rtos_app_sdk::abi::RTOS_PRIO_BH_HIGH;
use rtos_app_sdk::info;
#[cfg(not(feature = "hil"))]
use rtos_app_sdk::rtos::delay_until;
// 控制循环用 tick_count 量实测周期（HIL/非 HIL 都要）
use rtos_app_sdk::rtos::tick_count;
use core::sync::atomic::Ordering;

/// 诊断开关：开启后会在启动前几圈打印大量 dbg 行，极易压垮开机瞬间的
/// 设备串口 TX 缓冲、导致同期的传感器任务日志被丢弃（误判传感器任务“死亡”）。
/// 正常验证时关闭。
const VERBOSE: bool = false;

// --- 非 HIL 飞行参数（演示/调参常量，后续可收敛到 core config） ---
/// 速率模式（STABILIZE/ALT_HOLD）：摇杆满偏 → 期望水平速度 (m/s)。
/// 实测期望→真值速度放大 ~1.84×（EKF 速度标定 vs 物理），×1.7 → 杆 0.4
/// 期望 0.68 → 实际 ~1.25m/s → 8 字半径 1.25×16/2π ≈ 3.2m。
#[cfg(not(feature = "hil"))]
const RATE_XY_GAIN: f32 = 1.7;
/// 速率模式：EKF 位置预测外推时间 (s)——用"速度外推的预测位置"补偿位置估计延迟。
#[cfg(not(feature = "hil"))]
const VEL_PRED_HORIZON: f32 = 0.25;
/// LAND / RTL 到位后：每拍垂向下降量 (m，D 向下为正，4ms 拍 → 5m/s 缓降)。
#[cfg(not(feature = "hil"))]
const LAND_DESCENT_PER_TICK: f32 = 0.02;
/// RTL：水平距原点小于该值 (m) 视为到位，开始缓降。
#[cfg(not(feature = "hil"))]
const RTL_ARRIVE_RADIUS: f32 = 1.0;
/// LOITER：RC 摇杆叠加的水平微调速度 (m/s 满偏)。
#[cfg(not(feature = "hil"))]
const LOITER_NUDGE_GAIN: f32 = 0.3;
/// 控制周期（RTOS tick = 1ms）。配合 `delay_until` 做**绝对节拍**：周期恒为 4ms，
/// 不随控制体执行时间 / 被占用时间漂移（相对 msleep(4) 实测被拉长到 ~6.1ms）。
#[cfg(not(feature = "hil"))]
const CONTROL_PERIOD_TICKS: u32 = 4;
#[cfg(feature = "hil")]
use crate::flyctrl::HIL_EVT;
use crate::flyctrl::{ATT_SP, EST_MTX, EST_STATE, HIL_DIAG, RATE_CMD, SENSOR_FRAME, SENSOR_SEQ, SETPOINT};

/// 上行指令解锁：地面站经 COMMAND_LONG(ARM/DISARM) 设置。
/// 与控制律内部 RC 解锁做逻辑或（任一为真即解锁）。
pub fn set_cmd_armed(arm: bool) {
    crate::flyctrl::uplink::G_CMD_ARMED.store(arm, Ordering::Relaxed);
}

/// 上行指令模式：地面站经 COMMAND_LONG(DO_SET_MODE) 设置。
/// telemetry 心跳 custom_mode 会读取此值反映当前模式。
pub fn set_cmd_mode(mode: u16) {
    crate::flyctrl::uplink::G_CMD_MODE.store(mode, Ordering::Relaxed);
}

/// 控制律硬实时任务入口。
/* ===================== ★design.md L2 `wq:attitude` WorkItem =====================
 * `control` 已从"独立线程"改为"L2 工作队列里的 WorkItem"：本文件导出
 * `ctrl_init()`（装配）+ `control_step()`（**一拍**，由队列 worker 调用）。
 * 持久状态放模块级静态（C-ABI 回调无法访问栈变量）。节拍由定时器提交驱动。
 * ============================================================================== */
use core::mem::MaybeUninit;
static mut CTRL_PID: MaybeUninit<PidController> = MaybeUninit::uninit();
// ⚠️本仓铁律：raw-bin 加载下 `.data` 初值不生效 ⇒ 含非零初值的静态必须进 `.rust_bss`
//   并在 `ctrl_init()` 里**运行期写入**（`name` 是 &str 指针、`first=true` 都非零 ✗）。
#[link_section = ".rust_bss"]
static mut CTRL_STAT: MaybeUninit<crate::flyctrl::rt_stat::RtStat> = MaybeUninit::uninit();
struct CtrlState {
    hold_alt: Meter,
    alt_locked: bool,
    last_est: Option<VehicleState>,
    seq: u32,
    first: bool,
    last_ticks: u32,
}
static mut CTRL_ST: CtrlState = unsafe { core::mem::zeroed() };
#[used]
static mut DBG_MOTOR: [f32; 4] = [0.0; 4];
#[used]
static mut DBG_MAGI: [f32; 16] = [0.0; 16];
#[used]
pub static mut DBG_RC: [f32; 10] = [0.0; 10];

/// design.md 4: CTRL_HEARTBEAT (increment each tick) -- for L1 safety_monitor.
pub static mut CTRL_HEARTBEAT: u32 = 0;

pub fn ctrl_init() {
    unsafe {
        CTRL_PID.write(PidController::from_config(&VehicleConfig::default_quad().ctrl_params()));
        CTRL_STAT.write(crate::flyctrl::rt_stat::RtStat::new("ctrl"));
        let s = &mut *core::ptr::addr_of_mut!(CTRL_ST);
        s.first = true;
        s.last_est = None;
    }
}

/// ★L2 `wq:attitude`：**一拍**（由工作队列 worker 调用）。
/// ★解锁直通用的**怠速油门**（PX4 `MOT_SPIN_ARMED` 同构）：仅让电机低速转动，
/// 远低于悬停（~0.5）⇒ 已解锁但无设定点时**不会起飞** ✓。
const IDLE_THRUST: f32 = 0.05;

pub fn control_step() {
    unsafe { CTRL_HEARTBEAT = CTRL_HEARTBEAT.wrapping_add(1); }
    let ctrl = unsafe { CTRL_PID.assume_init_mut() };
    let st = unsafe { CTRL_STAT.assume_init_mut() };
    let (mut hold_alt, mut alt_locked, mut last_est, mut seq, mut first, mut last_ticks) = unsafe {
        let s = &*core::ptr::addr_of!(CTRL_ST);
        (s.hold_alt, s.alt_locked, s.last_est, s.seq, s.first, s.last_ticks)
    };
        let t0 = st.tick();
        // 实测控制周期（RTOS tick = 1ms）：取「本轮与上轮的 tick 差」作真实 dt，
        // 拍率变化时估计/积分仍正确 ✓。
        //
        // ★**2026-09-21 更新**：旧注释称"实际周期 ~9.8ms"✗ —— 那是【模拟器优化前】的
        //   状态（属过时信息 ✗）。**当前实测**（`mcu_simulater` 的
        //   `zz_ctlprof::ctl_period_and_tick_cost` ✓）：控制拍 **249.7Hz**
        //   （周期均值 **4.0000ms**、抖动 std 0.84ms ✓）⇒ 固件已能跑满 4ms 预算 ✓。
        //   本"按差值取 dt"的写法仍保留 ✓（对拍率波动/过载是必要防御 ✓）。
        flyctrl_core::perf::probe(40); // 段40 起：循环顶（dt/ticks ✓）
        unsafe { crate::flyctrl::CTRL_TICKS = crate::flyctrl::CTRL_TICKS.wrapping_add(1); }
        let now_ticks = tick_count();
        let dt_ms = now_ticks.wrapping_sub(last_ticks).clamp(1, 50) as f32;
        last_ticks = now_ticks;
        let dt = Second(dt_ms / 1000.0);
        // ★C1：dt 已随 EKF 迁至 rate_task；本任务仅用 _dt 供姿态层 dt ✓。
        let _dt = dt;
        // 应用地面站参数（每周期原子读 G_PARAM_VALS -> pid 增益；PARAM_SET 即时生效）。
        flyctrl_core::perf::probe(41); // 段41 起：参数同步后
        crate::flyctrl::uplink::sync_gains_to_pid(ctrl);
        flyctrl_core::perf::probe(42); // 段42 起：sync_gains 完（含读帧前 ✓）
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: loop enter"); }

        // --- 取最新传感器帧（seqlock：control 优先级高于所有写者，读不被打断） ---
        // 写者：sensors(prio=5，非 HIL) / uplink(prio=10，HIL)。两者优先级均低于本任务
        // (control, prio=4)，因此读过程不可能被写者抢占 → 单次读即原子一致，无需重试。
        // 【关键】绝不能 `continue` 忙等重试：若赶上写者正处于写入中（SENSOR_SEQ 为奇，
        // 写者被本任务抢占在置奇与置偶之间），忙等会让低优先级写者永远得不到调度，
        // control 无限自旋 → 整机卡死（HIL 注入期间已实测复现：运行数秒后日志/下行全停）。
        // 正确处理：直接采用本拍快照（可能新老混合/略旧），下一 4ms 拍自然取得一致新帧。
        let (mut imu, rc, gps, baro_alt, mag, armed);
        unsafe {
            let f = &mut *core::ptr::addr_of_mut!(SENSOR_FRAME);
            imu = f.imu;
            // ★§5.136 读者侧一致性校验（对标 ArduPilot `check_gyro`/PX4 DataValidator 的
            //   量程校验）：seqlock 读者不校验 seq、**明确允许新老混合快照**（见上方注释的
            //   实时性约束：本任务优先级高于写者 ⇒ 重试在本拍必然读到同一撕裂态，无效）。
            //   撕裂帧会把 `imu` 的字节与相邻字段拼接 ⇒ 合成物理不可能的浮点值
            //   （实测陀螺 −57.6 / −3541 / **20165 rad/s**，且**超出驱动换算上限
            //   34.9 rad/s** ⇒ 可断言非驱动产物、而是帧撕裂）。此处做量程校验，
            //   不合格 ⇒ 当作无 IMU 帧（回退上一有效帧/SimImu ⇒ 不污染滤波与控制）✓
            if let Some(s) = imu.as_ref() {
                let g = [s.gyro[0].0, s.gyro[1].0, s.gyro[2].0];
                let a = [s.accel[0].0, s.accel[1].0, s.accel[2].0];
                let gmax = g.iter().fold(0.0f32, |m, v| if v.abs() > m { v.abs() } else { m });
                let amax = a.iter().fold(0.0f32, |m, v| if v.abs() > m { v.abs() } else { m });
                let ok = g.iter().chain(a.iter()).all(|v| v.is_finite())
                    && gmax <= 35.0   // 2000 dps 物理量程
                    && amax <= 200.0; // 比力上界
                if !ok {
                    imu = None;
                }
            }
            rc = f.rc;
            gps = f.gps;
            baro_alt = f.baro_alt;
            mag = f.mag;
            armed = f.armed;
            // ★C1：IMU 单次消费已移至 `rate_task`（按 `SENSOR_SEQ` 新帧判定 ✓）——
            //   本任务（姿态层）不再消费 IMU ✓。
            let _ = imu;
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        }
        // ★§5.136 临时诊断
        // 指令解锁：与地面站上行命令做逻辑或（RC 解锁 或 指令解锁 任一为真）。
        let cmd_armed = crate::flyctrl::uplink::G_CMD_ARMED.load(Ordering::Relaxed);
        let armed_eff = armed || cmd_armed;
        unsafe { crate::flyctrl::CTRL_PHASE = 0; }
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: sen-mtx got"); }

        // 地面站 RC 通道覆盖（RC_CHANNELS_OVERRIDE）：生效时以地面站通道优先于 sim RC。
        // 通道映射：ch1=roll, ch2=pitch, ch3=throttle, ch4=yaw（标准 MAVLink 约定）。
        // PWM 1000-2000us 归一化到 0.0-1.0。
        let (rc_ov, rc_ov_valid) = crate::flyctrl::uplink::get_rc_override();
        let rc = if rc_ov_valid {
            let norm = |pwm: u16| -> f32 {
                let v = (pwm as f32 - 1000.0) / 1000.0;
                v.clamp(0.0, 1.0)
            };
            // ★§5.152【中位归零（机械性 ✓）】：`RcInput` 的 roll/pitch/yaw 是**有符号**
            //   摇杆量（"右移/后拉为正" ✓，零=中位 ✓ 见 `vehicle.rs:351-363`），而 PWM 归一
            //   是 0..1（中位 **0.5** ✗）。此前直接传 `norm()` ⇒ ① 中位时残留 +0.5 的恒定
            //   指令 ✗ ② 前推 0.1 被当 0.6 用（放大 6× ✗）。实测：LOITER 前推摇杆时北向
            //   速度远超指令量级且方向不符（§5.145 台账项 ✓）。
            //   ⇒ 三轴按【中位 0 点 + 满舵 ±1】归一 ✓：`(norm − 0.5) × 2` ✓
            //   ⚠️ 油门保持 0..1（其语义本就是 0..1 ✓，中位 0.5 = 悬停油门 ✓ 见
            //     `vehicle.rs:358` ✓）
            // 内联（不用闭包 ✓ 减少一层间接 ⇒ 排除布局/优化相关疑点 ✓）
            let roll_s = (norm(rc_ov[0]) - 0.5) * 2.0;
            let pitch_s = (norm(rc_ov[1]) - 0.5) * 2.0;
            let yaw_s = (norm(rc_ov[3]) - 0.5) * 2.0;
            RcInput {
                throttle: norm(rc_ov[2]),
                roll: roll_s,
                pitch: pitch_s,
                yaw: yaw_s,
                // ★§5.145 修复（机械性 ✓）：override **只应覆盖摇杆 4 通道**（MAVLink
                //   `RC_CHANNELS_OVERRIDE` 语义 ✓），**不应改解锁/模式**——它们仍是 RC 链路
                //   的职责 ✓。此前用 `rc_ov[0] > 1500` 作解锁指示 ⇒ 前推摇杆(roll=1500)
                //   会**误判为失锁** ✗（实测：override 下北向速度恒 0，因未解锁 ⇒ 位置环
                //   不工作 ✗）。改为**继承 RC 链路的 armed/mode** ✓（与 `fresh: true` 只表示
                //   "摇杆数据新鲜" 一致 ✓）
                armed: rc.armed,
                mode: rc.mode,
                fresh: true,
            }
        } else {
            rc
        };

        // --- 期望状态：模式决定目标（原点定高 / RTL 回原点 / LAND 缓降） ---
        // custom_mode 用 ArduCopter 标准码（G_CMD_MODE 由上行 DO_SET_MODE/TAKEOFF/LAND/RTL 写入）。
        // HIL：设定点直接来自 PC 仿真器（SET_POSITION_TARGET_LOCAL_NED），RC 路径编译期关闭。
        // 设定点在共享单步**之前**构造：非 HIL 高度基准 / HIL 回退定高均用上一拍估计
        // `last_est`（EKF 位置 4ms 内变化远小于 1mm，与"本拍估计后构造"等价）。
        // ★分段探针：读帧【已完成】、设定点构造开始 ✓（把 0→1 一分为二 ✓，§5.88）
        unsafe { crate::flyctrl::CTRL_PHASE = 5; }
        // ★§5.183：把 ESKF 估计的世界系加速度注入 **D 项专用通道**（`set_world_accel` ✓）。
        //   与 `Setpoint.acc`（轨迹前馈）**分离** ✓：此前把它塞进 `Setpoint.acc` ⇒ 同一信号
        //   既当**轨迹前馈**又当 **D 项输入**（PX4 `_vel_dot` vs `_acc_sp` 是两个量 ✗）
        //   ⇒ 测得加速度被当**前馈正反馈**注入控制律（悬停时 world_accel≈0 故未暴露，
        //   但姿态估计偏差时它会直接进入倾角指令 ✗）。
        {
            // ★C1：world_accel（速度环 D 项）自 rate_task 的 EKF 共享诊断读回 ✓。
            let wa = unsafe { (*core::ptr::addr_of!(HIL_DIAG)).world_accel };
            ctrl.set_world_accel([
                MeterPerSecondSquared(wa[0]),
                MeterPerSecondSquared(wa[1]),
                MeterPerSecondSquared(wa[2]),
            ]);
        }
        let (setpoint, setpoint_valid) = {
            #[cfg(feature = "hil")]
            {
                use flyctrl_core::units::Radian;
                let sp = crate::flyctrl::uplink::hil_setpoint();
                if crate::flyctrl::uplink::hil_setpoint_valid() {
                    (
                        Setpoint {
                            pos: [Meter(sp.x), Meter(sp.y), Meter(sp.z)],
                            yaw: Radian(sp.yaw),
                            vel: [MeterPerSecond(sp.vx), MeterPerSecond(sp.vy), MeterPerSecond(sp.vz)],
                            acc: [MeterPerSecondSquared(sp.afx), MeterPerSecondSquared(sp.afy), MeterPerSecondSquared(sp.afz)],
                        },
                        true,
                    )
                } else {
                    // sim 尚未连接：保持当前位置定高，避免悬停指令冲击。
                    let hold_z = last_est.map(|e| e.pos[2].0).unwrap_or(0.0);
                    (
                        Setpoint {
                            pos: [Meter(0.0), Meter(0.0), Meter(hold_z)],
                            yaw: Radian(0.0),
                            vel: [MeterPerSecond(0.0); 3],
                            // ★§5.183：`Setpoint.acc` 归位为**轨迹前馈**（此处无轨迹 ⇒ 0 ✓）；
                            //   世界系加速度改走上方的 D 项专用通道（`set_world_accel` ✓）。
                            acc: [MeterPerSecondSquared(0.0); 3],
                        },
                        false,
                    )
                }
            }
            #[cfg(not(feature = "hil"))]
            {
                use flyctrl_core::units::Radian;
                use flyctrl_core::comm::mavlink::enums::{
                    COPTER_MODE_ALT_HOLD, COPTER_MODE_GUIDED, COPTER_MODE_LAND,
                    COPTER_MODE_LOITER, COPTER_MODE_RTL, COPTER_MODE_STABILIZE,
                };
                // 模式来源（**唯一真源** `flightmode::mode_from_rc_switch`）：
                // **RC 模式开关优先** —— 真机上飞行员必须能自己切模式，这也是
                // 地面站断链时唯一的手段；仅当 RC 链路不新鲜（失联）时才回退到
                // 地面站 `G_CMD_MODE`（MAVLink DO_SET_MODE）。
                //
                // 历史：本行原先**只**读 G_CMD_MODE ⇒ 固件里 `RcInput.mode`
                // （SBUS ch[5] 已解析）被彻底丢弃，而一期又关闭了 usb-link
                // ⇒ 模式恒为 0，M 场无法用遥控器切到 LOITER（定点）做抗风验收。
                let cmd_mode = if rc.fresh {
                    flyctrl_core::flightmode::mode_from_rc_switch(rc.mode).to_copter_mode()
                } else {
                    crate::flyctrl::uplink::G_CMD_MODE.load(Ordering::Relaxed)
                };
                // 模式 → 位置外环开关（每拍设置）：速率模式（STABILIZE/ALT_HOLD）旁路
                // 位置外环（期望速度=摇杆直通），位置模式（LOITER/GUIDED/RTL/LAND）
                // 启用位置跟踪（位置 P + 速度前馈）。
                let rate_mode = matches!(cmd_mode, COPTER_MODE_STABILIZE | COPTER_MODE_ALT_HOLD);
                ctrl.set_rate_mode_xy(rate_mode);
                let thr_off = (rc.throttle - 0.5) * 2.0;
                unsafe {
                    let d = core::ptr::addr_of_mut!(DBG_RC);
                    (*d)[0] = if armed_eff { 1.0 } else { 0.0 };
                    (*d)[1] = if rc.fresh { 1.0 } else { 0.0 };
                    (*d)[2] = rc.mode as f32;
                    (*d)[3] = rc.throttle;
                    (*d)[4] = rc.pitch;
                    (*d)[5] = rc.roll;
                    (*d)[6] = cmd_mode as f32;
                }
                // 当前估计位置（NED；无估计时回退 hold_alt 基准）。
                let cur = last_est
                    .map(|e| (e.pos[0].0, e.pos[1].0, e.pos[2].0))
                    .unwrap_or((0.0, 0.0, hold_alt.0));
                // 模式 → 期望目标（NED 位置 + 期望速度 + 是否速率模式）：
                let (tx, ty, tz, vx, vy, use_rc_vel) = match cmd_mode {
                    // LAND：当前位置水平保持，垂向每拍缓降趋向地面（D 向下，地面=0）。
                    COPTER_MODE_LAND => (
                        cur.0, cur.1,
                        (cur.2 - LAND_DESCENT_PER_TICK).max(0.0),
                        0.0, 0.0, false,
                    ),
                    // RTL：水平回原点 (0,0) 定高（起飞基准 hold_alt）；水平到位后缓降。
                    COPTER_MODE_RTL => {
                        let horiz = flyctrl_core::math::sqrt(cur.0 * cur.0 + cur.1 * cur.1);
                        let tz = if horiz < RTL_ARRIVE_RADIUS {
                            (cur.2 - LAND_DESCENT_PER_TICK).max(0.0)
                        } else {
                            hold_alt.0
                        };
                        (0.0, 0.0, tz, 0.0, 0.0, false)
                    }
                    // 定点：原点定高；LOITER 允许 RC 摇杆叠加水平微调速度
                    // （GUIDED 无目标通道时原点保持，后续接入 SET_POSITION_TARGET）。
                    COPTER_MODE_LOITER | COPTER_MODE_GUIDED => (
                        // ★§5.145 诊断：记录 LOITER 期望速度
                        {
                            unsafe {
                                let d = core::ptr::addr_of_mut!(DBG_RC);
                                (*d)[7] = rc.pitch * LOITER_NUDGE_GAIN;
                                (*d)[8] = -rc.roll * LOITER_NUDGE_GAIN;
                                (*d)[9] = 1.0;
                            }
                            0.0
                        }, 0.0, hold_alt.0,
                        rc.pitch * LOITER_NUDGE_GAIN,
                        -rc.roll * LOITER_NUDGE_GAIN,
                        false, // §5.157 排查：暂回 false（保留 ① 首拍修复 ✓）
                    ),
                    // STABILIZE / ALT_HOLD / 默认：速率模式（摇杆 → 期望速度）+ 定高，
                    // 大疆手感（推杆飞、松杆停）；pos 用速度外推预测位置补偿 EKF 延迟。
                    _ => (
                        0.0, 0.0, hold_alt.0 - thr_off * 2.0,
                        rc.pitch * RATE_XY_GAIN,
                        -rc.roll * RATE_XY_GAIN,
                        true,
                    ),
                };
                // 速率模式：速度外推的预测位置（补偿 EKF 位置估计延迟）；位置模式：绝对目标。
                // ★§5.165：外推时域可被旋钮覆盖（默认 -1 ⇒ 编译期值 ✓）
                let pred_h = {
                    let ov = unsafe {
                        core::ptr::read_volatile(core::ptr::addr_of!(
                            flyctrl_core::controller::pid::G_VEL_PRED))
                    };
                    if ov >= 0.0 { ov } else { VEL_PRED_HORIZON }
                };
                let pos = match (use_rc_vel, last_est) {
                    (true, Some(e)) => [
                        Meter(e.pos[0].0 + e.vel[0].0 * pred_h),
                        Meter(e.pos[1].0 + e.vel[1].0 * pred_h),
                        Meter(tz),
                    ],
                    (true, None) => [Meter(tx), Meter(ty), Meter(tz)],
                    (false, _) => [Meter(tx), Meter(ty), Meter(tz)],
                };
                (
                    Setpoint {
                        pos,
                        yaw: Radian(rc.yaw * 0.5),
                        vel: [
                            MeterPerSecond(vx),
                            MeterPerSecond(vy),
                            MeterPerSecond(0.0),
                        ],
                        acc: [MeterPerSecondSquared(0.0); 3],
                    },
                    // 非 HIL 模式的设定点恒有效：RC 油门/模式/已锁基准构成的 setpoint
                    // 是真实控制目标。若传 false，`hil_pos_inited` 永不置位（step_hil
                    // 位置闸要求 setpoint_valid），执行器恒零——正是联调 F 的
                    // 「非 HIL 无 PWM」阻塞根因（旧版临时放宽绕开的点）。
                    true,
                )
            }
        };

        unsafe { crate::flyctrl::CTRL_PHASE = 1; }
        // ★C1：本任务 = 姿态层（PX4 `mc_pos_control` + `mc_att_control`）。
        //   EKF/FDIR/初始化门控/速率层/混控/PWM 已迁至 1kHz `rate_task` ✓。
        //   本拍：读 rate_task 的估计 → 发布设定点 → 姿态 P → 发布速率设定值。
        flyctrl_core::perf::probe(43); // 段43 起：读帧+设定点完，进姿态层
        // 1) 读 1kHz rate_task 发布的估计状态（EKF 在 IMU 率更新 ✓）。
        //    优先级：写者 rate(5) 低于读者 control(4) ⇒ 读期间写者不运行 ✓
        //    （同 SENSOR_FRAME 论证；不加锁以避免与高频任务争用 EST_MTX ✓）。
        let (est, health) = unsafe {
            let s = &*core::ptr::addr_of!(EST_STATE);
            (s.est, s.health)
        };
        // 2) 发布设定点（供 rate_task 的 `ekf_hil` 初始化/门控 ✓）。
        unsafe {
            let s = &mut *core::ptr::addr_of_mut!(SETPOINT);
            s.sp = setpoint;
            s.valid = if setpoint_valid { 1 } else { 0 };
        }
        // 3) 姿态层（PX4 `mc_att_control`）：位置/速度外环 + 姿态 P → 速率设定值。
        // ★P0-3b：L2 姿态层只做**姿态 P**；期望姿态（q_des+thrust）由 L3 `nav` 外环产出 ✓。
        let rsp = unsafe {
            let a = &*core::ptr::addr_of!(ATT_SP);
            if a.valid != 0 {
                let q_des = Quaternion { w: a.q[0], x: a.q[1], y: a.q[2], z: a.q[3] };
                let rates = ctrl.attitude_rates_sp(q_des, &est, _dt);
                // ★★怠速下限（PX4 `MOT_SPIN_ARMED` 同构）：**已解锁**时油门不得低于 IDLE_THRUST，
                //   否则"外环尚未给出油门（未起飞）"⇒ thrust=0 ⇒ 执行器恒 0 ✗（实测
                //   `unlock_flight` 抓到 `armed=true` 而 `m_permille` 全 0 ✗）。上限仍由外环/限幅决定 ✓。
                let thr = if armed_eff && health != Health::Critical {
                    a.thrust.max(IDLE_THRUST)
                } else {
                    a.thrust
                };
                flyctrl_core::controller::RateSetpoint::new(rates, thr)
            } else if armed_eff && health != Health::Critical {
                // ★★★解锁直通（PX4 `MOT_SPIN_ARMED` 同构）：**已解锁但外环尚未给出有效
                //   姿态设定点**（`ATT_SP.valid=0`，如仅解锁未起飞 / 位置环未就绪）——
                //   此时若回 INVALID（thrust=0）⇒ 速率环零推力 ⇒ 执行器恒 0 ✗，与"已解锁
                //   ⇒ 电机应怠速运转"的语义不符（实测 `unlock_flight` 抓到 m_permille 全 0 ✗）。
                //   直通：**零角速率 + 怠速油门** ⇒ 保持水平且不起飞 ✓。
                flyctrl_core::controller::RateSetpoint::new([0.0, 0.0, 0.0], IDLE_THRUST)
            } else {
                flyctrl_core::controller::RateSetpoint::INVALID
            }
        };
        // 4) EKF 诊断（mag 内部量）自 rate_task 的共享区读回（遥测 DBG_MAGI ✓）。
        unsafe {
            let hd = &*core::ptr::addr_of!(HIL_DIAG);
            let d = core::ptr::addr_of_mut!(DBG_MAGI);
            (*d)[0] = hd.mag_i[0]; (*d)[1] = hd.mag_i[1]; (*d)[2] = hd.mag_i[2];
            (*d)[3] = hd.mag_b[0]; (*d)[4] = hd.mag_b[1]; (*d)[5] = hd.mag_b[2];
            (*d)[6] = hd.yaw_aligned as f32;
            (*d)[7] = hd.mag_disturbed as f32;
            (*d)[8] = hd.mag_applied as f32; (*d)[9] = hd.mag_skipped as f32;
            (*d)[10] = hd.mag_hdg_innov_lpf; (*d)[11] = hd.last_mag_yaw_innov;
            if let Some(mm) = mag {
                (*d)[12] = mm[0]; (*d)[13] = mm[1]; (*d)[14] = mm[2];
            }
            (*d)[15] = hd.yaw_rad;
        }
        // 5) 执行器指令（rate_task 产出，仅用于遥测/诊断回读 ✓）。
        let cmd_motor = unsafe { (*core::ptr::addr_of!(HIL_DIAG)).motor };
        last_est = Some(est);
        unsafe { crate::flyctrl::CTRL_PHASE = 2; }

        // 解锁瞬间锁定高度基准（armed_eff = RC 解锁 或 地面站 COMMAND_LONG 解锁）
        if armed_eff && !alt_locked {
            hold_alt = est.pos[2];
            alt_locked = true;
        } else if !armed_eff {
            alt_locked = false;
        }

        flyctrl_core::perf::probe(44); // 段44 起：姿态层完成
        // HIL：执行器回传已由 rate_task 完成（`set_actuator_cmd` ✓）。
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: pid ok"); }

        // 指令观测（静态，无栈开销；虚拟外设测试定位"无推力来源"用）
        unsafe {
            DBG_MOTOR = cmd_motor;
        }
        flyctrl_core::perf::probe(45); // 段45 起：发布速率设定值前
        // ★C3：发布**速率设定值**（PX4 `vehicle_rates_setpoint` 同构 ✓）——
        //   独立 `rate_task`(1kHz) 消费它 + 控制器侧陀螺跑速率层并驱动 PWM ✓。
        unsafe {
            let rcmd = &mut *core::ptr::addr_of_mut!(RATE_CMD);
            rcmd.rates = rsp.rates;
            rcmd.thrust = rsp.thrust;
            rcmd.valid = if health != Health::Critical && armed_eff { 1 } else { 0 };
        }
        unsafe { crate::flyctrl::CTRL_PHASE = 3; }
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: after publish"); }
        if VERBOSE && seq == 0 {
            let ec = unsafe { EST_MTX.debug_count() };
            info!(tag: "ctrl", "dbg: est-mtx count={} sensor-seq={}", ec, unsafe { SENSOR_SEQ });
        }

        flyctrl_core::perf::probe(46); // 段46 起：发布完（此后为遥测 + delay ✓）
        // --- 遥测诊断（`EST_STATE` 由 rate_task 写 ✓，本任务**不再写**） ---
        {
            // 油门百分比(0..100) 供 telemetry 经 VFR_HUD 下发。
            let throttle_avg = (cmd_motor[0] + cmd_motor[1] + cmd_motor[2] + cmd_motor[3]) / 4.0;
            crate::flyctrl::uplink::G_THROTTLE.store((throttle_avg.clamp(0.0, 1.0) * 100.0) as u8, Ordering::Relaxed);
        }
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: est-mtx got"); }

        st.sample(st.tick().wrapping_sub(t0), 672_000, 672_000); // 250Hz 名义
        if seq % 250 == 0 { st.report_and_reset(); } // ★P1-2 可观测
        seq = seq.wrapping_add(1);
        if first {
            first = false;
            info!(tag: "ctrl",
                  "first loop done; imu_ok={} armed={} crit={} alt={:.2}",
                  imu.is_some(), armed_eff, health == Health::Critical, est.pos[2].0);
        }
        if seq % 25 == 0 {
            // EKF 状态观察日志（10Hz）：姿态/位置/速度/有限性——调试/虚拟外设验证用，
            // 量产后可经 VERBOSE 门控关闭。
            let (r, p, y) = (est.att.roll(), est.att.pitch(), est.att.yaw());
            let fin = est.att.w.is_finite() && est.att.x.is_finite()
                && est.att.y.is_finite() && est.att.z.is_finite()
                && est.pos.iter().all(|v| v.0.is_finite())
                && est.vel.iter().all(|v| v.0.is_finite());
            info!(tag: "ctrl", "dbg est r={:.1} p={:.1} y={:.1}deg p=({:.2},{:.2},{:.2}) v=({:.2},{:.2},{:.2}) fin={} imu_ok={}",
                  r.to_degrees(), p.to_degrees(), y.to_degrees(),
                  est.pos[0].0, est.pos[1].0, est.pos[2].0,
                  est.vel[0].0, est.vel[1].0, est.vel[2].0, fin, imu.is_some());
        }
        if seq % 250 == 0 {
            // hb 心跳：gv/gpsd 验证 RMC 速度链路（gps_v 为 SENSOR_FRAME 中
            // PosSample.vel，Some 表示固件解析到了 $GNRMC 的速度字段）
            let gps_v = gps.and_then(|g| g.vel).map(|v| [v[0].0, v[1].0, v[2].0]);
            let (gv, gv_n) = match gps_v {
                Some(v) => (v, 1u8),
                None => ([0.0, 0.0, 0.0], 0u8),
            };
            // 精简行：日志缓冲 180B（SDK emit 截断点）——去掉冗余 gps_v/baro_h，
            // 行末 m=[...] 不再被截断（此前 19 参数 ≈186B 尾部被截，见 SDK 越界修复）。
            // ★全整数/定点（避开 `flt2dec` ✗ —— 它是单次日志极贵的主因 ✓，
            //   见 §5.76/§5.77：`log::emit` 22% + 浮点格式化 3% ✓）；
            //   ★保留 `"hb seq="` 子串（多处测试依赖 ✓，如 x_flyctrl_modes ✓）
            info!(tag: "ctrl", "hb seq={} armed={} crit={} alt_mm={} imu_ok={} mag={} gps={} gv_cms=({},{},{}) baro={} gpsd_mm={} gz_cms={} m_permille=[{},{},{},{}]",
                  seq, armed_eff, health == Health::Critical, (est.pos[2].0 * 1000.0) as i32,
                  imu.is_some(), mag.is_some(), gps.is_some(),
                  (gv[0] * 100.0) as i32, (gv[1] * 100.0) as i32, (gv[2] * 100.0) as i32,
                  baro_alt.is_some(),
                  (gps.map(|g| g.pos[2].0).unwrap_or(0.0) * 1000.0) as i32,
                  (est.vel[2].0 * 100.0) as i32,
                  (cmd_motor[0] * 1000.0) as i32, (cmd_motor[1] * 1000.0) as i32,
                  (cmd_motor[2] * 1000.0) as i32, (cmd_motor[3] * 1000.0) as i32);
        }
        // ★design.md P0-2：每 2s 打印 RT 违约计数（deadline/wcet/sched）——
        //   让 P0-1 声明的 deadline/wcet 违约**可见**（不改变行为，只观测 ✓）。
        if seq % 500 == 0 {
            let mut v = [0u32; 3];
            if let Some(f) = rtos_app_sdk::abi::slot().rt_violation {
                f(v.as_mut_ptr());
            }
            info!(tag: "ctrl", "rtviol dl={} wcet={} sched={}", v[0], v[1], v[2]);
        }

    unsafe {
        let s = &mut *core::ptr::addr_of_mut!(CTRL_ST);
        s.hold_alt = hold_alt; s.alt_locked = alt_locked; s.last_est = last_est;
        s.seq = seq; s.first = first; s.last_ticks = last_ticks;
    }
}
