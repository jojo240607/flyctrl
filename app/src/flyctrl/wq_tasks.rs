//! ★design.md **L2 软实时工作队列**（**1 个 worker 线程**承载多个 WorkItem）。
//!
//! 对齐 design.md / PX4：**一条队列 = 一个 worker 线程**，多个逻辑任务作为
//! WorkItem 挂进去（不是"每任务一个 worker"）。本文件承载：
//!   · `estimator`（EKF/FDIR，250Hz）
//!   · `attitude` （姿态层 `control`，250Hz）
//! 由 **250Hz 软件定时器** 把两个 item 提交到 **L2 队列**（`wq:l2`），
//! worker 按 EDF 取出、逐项测执行时间（`budget_cycles`）。
//!
//! L1 `rate`（1kHz）保持**独立硬实时线程**（design.md L1，不进队列）。
//! 状态放静态区（C-ABI 回调拿不到栈变量），由一次性 setup 任务原地初始化。

use core::ffi::c_void;
use core::mem::MaybeUninit;

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::PidController;

/// **控制器实现类型（按 feature 选择）** —— 与 `select.rs` 的 `InnerEst` 同一手法：
/// 两种类型都实现了 `Controller` ⇒ 构造与用法**一行都不用改** ✓
/// ⚠默认（无 feature）**逐位不变** ✓。
#[cfg(not(feature = "fcalg-ctrl"))]
pub type CtrlImpl = PidController;
#[cfg(feature = "fcalg-ctrl")]
pub type CtrlImpl = flyctrl_core::estimator::fcalg_ctrl_bridge::FcalgController;

/// ★**构造子随类型不同**（旧栈是 `PidController::default_quad()`，新栈是 `FcalgController::new()`）
/// ⇒ 用 cfg 工厂收口，调用点保持一行 ✓。默认路径**逐位不变** ✓。
#[cfg(not(feature = "fcalg-ctrl"))]
fn make_ctrl() -> CtrlImpl {
    PidController::default_quad()
}
#[cfg(feature = "fcalg-ctrl")]
fn make_ctrl() -> CtrlImpl {
    flyctrl_core::estimator::fcalg_ctrl_bridge::FcalgController::new()
}
use flyctrl_core::estimator::select::AnyEstimator;
use flyctrl_core::estimator::Estimator as _; // ★诊断：`accel_bias()` 是 trait 方法，须导入 ✓
use flyctrl_core::hil::{HilContext, SimImu};
use flyctrl_core::units::Second;

use rtos_app_sdk::abi::{rtos_work_t, slot};
use rtos_app_sdk::info;
use rtos_app_sdk::rtos::msleep;

use crate::flyctrl::{EST_STATE, HIL_DIAG, SENSOR_FRAME, SENSOR_SEQ, SETPOINT};

/// 队列：L2 一条（承载 estimator+attitude）；L3 一条（承载 nav/pos/comm/log）。
pub const Q_L2: u8 = 1;
pub const Q_L3: u8 = 2;

/// worker 栈（App 提供）。L2 需容纳 EKF（~4KB）⇒ 8KB；L3 轻量 ⇒ 2KB。
#[repr(C, align(8))]
struct WqStack<const N: usize>([u8; N]);
#[link_section = ".app_stacks"]
static mut L2_WQ_STACK: WqStack<12288> = WqStack([0; 12288]);
#[link_section = ".app_stacks"]
static mut L3_WQ_STACK: WqStack<2048> = WqStack([0; 2048]);

// ---- estimator item 状态 ----
static mut EKF_HIL: MaybeUninit<HilContext<AnyEstimator, CtrlImpl>> = MaybeUninit::uninit();
static mut EKF_IMU: MaybeUninit<SimImu> = MaybeUninit::uninit();
static mut EKF_LAST_SEQ: u32 = 0;
static mut EKF_LAST_TICKS: u32 = 0;
static mut EKF_ITEM: rtos_work_t = unsafe { core::mem::zeroed() };
// ---- attitude item ----
static mut ATT_ITEM: rtos_work_t = unsafe { core::mem::zeroed() };
static mut SENSORS_ITEM: rtos_work_t = unsafe { core::mem::zeroed() };
// ★L3
static mut NAV_ITEM: rtos_work_t = unsafe { core::mem::zeroed() };
static mut TELEM_ITEM: rtos_work_t = unsafe { core::mem::zeroed() };
static mut UPLINK_ITEM: rtos_work_t = unsafe { core::mem::zeroed() };
/// ★design.md §6：日志 WorkItem（L3「事件驱动·可丢弃」）—— 取代独立 log 线程 ✓。
static mut LOG_ITEM: rtos_work_t = unsafe { core::mem::zeroed() };

/// L2 WorkItem：estimator（EKF/FDIR 一拍）。
/// ★诊断：L2 各 item 的 exec（cycles）——定位"谁在吃 4ms"
pub static mut IT_EXEC_EKF: u32 = 0;
pub static mut IT_EXEC_ATT: u32 = 0;
pub static mut IT_EXEC_SEN: u32 = 0;

extern "C" fn estimator_work(_arg: *mut c_void) {
    let t_it = rtos_app_sdk::rtos::cycle_now();
    struct D; impl Drop for D { fn drop(&mut self) {} }
    let hil = unsafe { EKF_HIL.assume_init_mut() };
    let sim_imu = unsafe { EKF_IMU.assume_init_mut() };
    // ★design.md §7：排空 `IMU_RING`（1kHz 样本）→ 逐样本 `predict_delta`（不丢样本）。
    unsafe {
        let r = &mut *core::ptr::addr_of_mut!(crate::flyctrl::IMU_RING);
        let mut n = 0usize;
        while let Some(d) = r.pop() {
            if n < hil.imu_deltas.len() { hil.imu_deltas[n] = d; n += 1; }
        }
        hil.imu_deltas_len = n;
    }
    // ★design.md §7：积分用**实际 dt**（本拍与上拍 tick 差），不用名义 4ms——
    //   否则 worker 被延后时 item 成批补跑、EKF 每拍仍积 4ms ⇒ **过积分** ⇒ 漂移。
    let now = rtos_app_sdk::rtos::tick_count();
    let dt_ms = unsafe {
        let last = EKF_LAST_TICKS;
        EKF_LAST_TICKS = now;
        now.wrapping_sub(last)
    }.clamp(1, 50) as f32;
    hil.dt = Second(dt_ms / 1000.0);
    unsafe {
        let d = &mut *core::ptr::addr_of_mut!(crate::flyctrl::HIL_DIAG);
        d.n_ekf_calls += 1.0;
        d.sum_dt_ms += dt_ms as f32;
        // ★实测：上一次执行周期数（IT_EXEC_EKF 在 worker 末尾写入 ✓）+ 漏拍/降级（workq 自己的 ✓）
        d.it_exec_ekf_cyc = core::ptr::read_volatile(core::ptr::addr_of!(IT_EXEC_EKF)) as f32;
        d.ekf_miss = core::ptr::read_volatile(core::ptr::addr_of!(EKF_ITEM.miss_count)) as f32;
        d.ekf_degraded = core::ptr::read_volatile(core::ptr::addr_of!(EKF_ITEM.degraded)) as f32;
    }
    // ★2026-10-05 验证打印：**实际 tick 间隔** vs 样本代表的 4ms
    //   反推假设：workq 1ms 量化 ⇒ 实际 ≈4.4ms ✗ ⇒ ab_z 吸收 ~3.8% ⇒ 重力门关闭 ⇒ 发散 ✓
    {
        static mut DBG_DT_N: u32 = 0;
        let c = unsafe { DBG_DT_N };
        unsafe { DBG_DT_N = DBG_DT_N.wrapping_add(1) };
        if c % 250 == 0 || c < 20 {
            rtos_app_sdk::info!(tag: "dtdbg", "dt_ms={} n_deltas={}", dt_ms, hil.imu_deltas_len);
        }
    }
    let (imu, gps, baro_alt, mag, rc, armed, seq_now) = unsafe {
        let f = &*core::ptr::addr_of!(SENSOR_FRAME);
        (f.imu, f.gps, f.baro_alt, f.mag, f.rc, f.armed, SENSOR_SEQ)
    };
    // ★无自旋 seqlock 校验：读后再读一次 `SENSOR_SEQ`；若写者中途写过（值变）⇒
    //   本帧可能是撕裂的 ⇒ 当作无新帧（sample-and-hold），**不自旋**（避免把低优先写者饿死 ✗）。
    let seq_after = unsafe { SENSOR_SEQ };
    let fresh = unsafe {
        let last = EKF_LAST_SEQ;
        EKF_LAST_SEQ = seq_now;
        seq_now == seq_after && seq_now != last && (seq_now & 1) == 0
    };
    let imu_in = if fresh { imu } else { None };
    let (sp, sp_valid) = unsafe {
        let s = &*core::ptr::addr_of!(SETPOINT);
        (s.sp, s.valid != 0)
    };
    let (est, health, gated) = hil.ekf_hil(
        imu_in, gps, baro_alt, None, None, mag, &sp, sp_valid, armed, rc.fresh, sim_imu,
    );
    unsafe {
        let s = &mut *core::ptr::addr_of_mut!(EST_STATE);
        s.est = est;
        s.health = health;
        s.armed = armed;
    }
    // ★诊断（限流）：加计零偏 + 估计位置/速度 —— 定位 `x_env_motion` 的 est.y 发散源
    unsafe {
        static mut DBG_EKF_N: u32 = 0;
        let c = DBG_EKF_N;
        DBG_EKF_N = DBG_EKF_N.wrapping_add(1);
        if c % 250 == 0 || c < 20 {  // ★②：头 20 次全打（定位 NaN 起点）
            let ab = hil.est.accel_bias();
            let e = &hil.est.inner;
            // ★② 最大位置修正量及其来源路（kind: 1=gpsP 2=gpsV 3=baro 4=grav 5=mag ✓）
            let (mu_kind, mu_dp, gm0, gm1, gm2, gr) = unsafe {
                let d = core::ptr::addr_of!(flyctrl_core::estimator::eskf::ESKF_MAX_UPD);
                ((*d)[0], (*d)[1], (*d)[2], (*d)[3], (*d)[4], (*d)[5])
            };
            info!(tag: "ekfdbg", "maxUpd kind={} |dp|={:.2} est=({:.2},{:.2},{:.2}) ab=({:.3},{:.3},{:.3}) | step={} gpsP={}/{} gpsV={}/{} grav={}/{} mag={}/{}/{}",
                  mu_kind, mu_dp,
                  est.pos[0].0, est.pos[1].0, est.pos[2].0,
                  ab[0], ab[1], ab[2],
                  e.n_step, e.n_gps_pos, e.n_gps_pos_rejected,
                  e.n_gps_vel, e.n_gps_vel_rejected,
                  e.n_grav_applied, e.n_grav_gated,
                  e.n_mag, e.n_mag_rejected, e.n_mag_reanchored);
            info!(tag: "gpsmeas", "meas=({:.2},{:.2},{:.2}) |resid|={:.2}", gm0, gm1, gm2, gr);
            // ★拒收诊断：最后一次被拒的 (max_resid, |ν|, nis) + 最后一次成功的 nis
            let (rj0, rj1, rj2, rjn, ok0, ok1, ok2) = unsafe {
                let r = core::ptr::addr_of!(flyctrl_core::estimator::eskf::ESKF_LAST_REJ);
                let k = core::ptr::addr_of!(flyctrl_core::estimator::eskf::ESKF_LAST_OK);
                ((*r)[0], (*r)[1], (*r)[2], (*r)[3], (*k)[0], (*k)[1], (*k)[2])
            };
            info!(tag: "rejdbg", "rej maxres={:.2} |nu|={:.2} nis={:.2} n={} | ok maxres={:.2} |nu|={:.2} nis={:.2}",
                  rj0, rj1, rj2, rjn, ok0, ok1, ok2);
        }
    }
    unsafe { IT_EXEC_EKF = rtos_app_sdk::rtos::cycle_now().wrapping_sub(t_it); }
    // ★★★接线（fcalg-est）：旧栈 8 项磁场诊断里，**新栈只暴露 mag_i / mag_b / yaw_rad**。
    //   其余 6 项（yaw_aligned / mag_field_disturbed / mag_applied / mag_skipped /
    //   mag_hdg_innov_lpf / last_mag_yaw_innov）是**旧 ESKF 特定实现**的内部量；
    //   fcalg 的磁通路结构不同（yaw-only + 无冻结旋钮）⇒ **无对应**。
    //   ⇒ 按"选择 (b)"：**不编造替代值**，那 6 项保持 HIL_DIAG 的默认（0），并在此注明。
    #[cfg(not(feature = "fcalg-est"))]
    unsafe {
        let fl = hil.est.inner.filter();
        let wa = hil.est.inner.world_accel();
        let d = &mut *core::ptr::addr_of_mut!(HIL_DIAG);
        d.world_accel = wa;
        d.mag_i = fl.mag_i;
        d.mag_b = fl.mag_b;
        d.yaw_aligned = fl.yaw_aligned as u32;
        d.mag_disturbed = fl.mag_field_disturbed as u32;
        d.mag_applied = fl.mag_applied;
        d.mag_skipped = fl.mag_skipped;
        d.mag_hdg_innov_lpf = fl.mag_hdg_innov_lpf;
        d.last_mag_yaw_innov = fl.last_mag_yaw_innov;
        d.yaw_rad = fl.st.q.yaw();
        d.gated = gated as u32;
    #[cfg(feature = "fcalg-est")]
    unsafe {
        let d = &mut *core::ptr::addr_of_mut!(HIL_DIAG);
        d.world_accel = hil.est.inner.world_accel();
        d.mag_i = hil.est.inner.mag_i();
        d.mag_b = hil.est.inner.mag_b();
        d.yaw_rad = hil.est.inner.yaw_rad();
        // 旧栈的另外 6 项磁场诊断：新栈未暴露 ⇒ **保持默认 0**（不编造）
        d.yaw_aligned = 0;
        d.mag_disturbed = 0;
        d.mag_applied = 0.0;
        d.mag_skipped = 0.0;
        d.mag_hdg_innov_lpf = 0.0;
        d.last_mag_yaw_innov = 0.0;
        d.gated = gated as u32;
    }
    }
}

/// L2 WorkItem：attitude（姿态层一拍 = 原 `control` 的一次循环体）。
extern "C" fn attitude_work(_arg: *mut c_void) {
    let t = rtos_app_sdk::rtos::cycle_now();
    crate::flyctrl::control::control_step();
    unsafe { IT_EXEC_ATT = rtos_app_sdk::rtos::cycle_now().wrapping_sub(t); }
}

/// L2 WorkItem：sensors（慢传感器 + IMU 采样；design.md §5 `wq:sensors`）。
extern "C" fn sensors_work(_arg: *mut c_void) {
    // IMU 采样已迁 L0 ISR；本 item 只读慢传感器（气压/磁/GPS）+ 组装帧
    let t = rtos_app_sdk::rtos::cycle_now();
    crate::flyctrl::sensors_task::sensors_step();
    unsafe { IT_EXEC_SEN = rtos_app_sdk::rtos::cycle_now().wrapping_sub(t); }
}

/// ★L3 WorkItem：nav/pos（位置/速度外环，50Hz）→ ATT_SP。
extern "C" fn nav_work(_arg: *mut c_void) {
    crate::flyctrl::nav_task::nav_step();
}

/// L3 WorkItem：telemetry（MAVLink 下行，20ms）。
extern "C" fn telem_work(_arg: *mut c_void) {
    crate::flyctrl::telemetry::telemetry_step();
}

/// ★design.md §6 L3 WorkItem：日志 drain（一轮排空 + uart0 DMA TX ✓）。
extern "C" fn log_work(_arg: *mut c_void) {
    rtos_app_sdk::log::drain_once();
}

/// L3 WorkItem：uplink（MAVLink 上行/解析）。
extern "C" fn uplink_work(_arg: *mut c_void) {
    crate::flyctrl::uplink::uplink_step();
}

/// L3 WorkItem：uplink（usb0 非阻塞轮询 + MAVLink 上行）。




/// ★装配（在**大栈 setup 任务**里调用：构造 `HilContext` 瞬时值需要栈）。
pub fn setup() {
    unsafe {
        // estimator 静态状态
        let mut hil = HilContext::new(
            AnyEstimator::default_product(),
            make_ctrl(),
            Second(4.0 / 1000.0),
        );
        hil.est.set_observation_noise(0.25, 0.01, 0.09);
        flyctrl_core::estimator::eskf::G_ESKF_MAG_HDG_GATE = 2.0;
        // ★★★2026-10-04【根因修复 ⑦：磁强制 **yaw-only**（PX4 安全默认 ✓）】
        //   AUTO（=0.0）在"磁场干净 + yaw 已对齐"时会**自动启用 3D 磁融合** ✗（含 roll/pitch 观测），
        //   而重力辅助若被门控 ⇒ 3D 磁成 roll/pitch 唯一参考 ⇒ 每拍把姿态旋转数十度 ⇒ NaN ✗
        //   （ELFSYM 直读：`magq.before→after` 差数十度 ✗；`update_mag_yaw` 只写 `e.dtheta[2]` ✓）。
        //   PX4：3D 依赖重力锚定 roll/pitch、属高级可选；默认只用 yaw ✓。
        flyctrl_core::estimator::eskf::G_ESKF_MAG_YAW_ON = 2.0; // 2.0 = yaw-only ✓
        let k = core::ptr::read_volatile(core::ptr::addr_of!(flyctrl_core::estimator::eskf::G_ESKF_FREEZE_BIAS));
        if k >= 0.5 {
            hil.est.set_freeze_bias(true);
        }
        EKF_HIL.write(hil);
        EKF_IMU.write(SimImu::new());
        EKF_ITEM = rtos_work_t {
            next: core::ptr::null_mut(),
            fn_: Some(estimator_work),
            arg: core::ptr::null_mut(),
            budget_cycles: 134_400, // ★design.md §5: estimator 预算 800µs @168MHz
            deadline_cycles: 0,
            miss_count: 0,
            degraded: 0,
            period_cycles: 0, // 由 workq_add_periodic 声明
            next_run_cycles: 0,
            queued: 0,
            pnext: core::ptr::null_mut(),
        };
        UPLINK_ITEM = rtos_work_t {
            next: core::ptr::null_mut(),
            fn_: Some(uplink_work),
            arg: core::ptr::null_mut(),
            budget_cycles: 0,
            deadline_cycles: 0,
            miss_count: 0,
            degraded: 0,
            period_cycles: 0, // 由 workq_add_periodic 声明
            next_run_cycles: 0,
            queued: 0,
            pnext: core::ptr::null_mut(),
        };
        TELEM_ITEM = rtos_work_t {
            next: core::ptr::null_mut(),
            fn_: Some(telem_work),
            arg: core::ptr::null_mut(),
            budget_cycles: 0,
            deadline_cycles: 0,
            miss_count: 0,
            degraded: 0,
            period_cycles: 0, // 由 workq_add_periodic 声明
            next_run_cycles: 0,
            queued: 0,
            pnext: core::ptr::null_mut(),
        };
        NAV_ITEM = rtos_work_t {
            next: core::ptr::null_mut(),
            fn_: Some(nav_work),
            arg: core::ptr::null_mut(),
            budget_cycles: 200_000,
            deadline_cycles: 0,
            miss_count: 0,
            degraded: 0,
            period_cycles: 0, // 由 workq_add_periodic 声明
            next_run_cycles: 0,
            queued: 0,
            pnext: core::ptr::null_mut(),
        };
        LOG_ITEM = rtos_work_t {
            next: core::ptr::null_mut(),
            fn_: Some(log_work),
            arg: core::ptr::null_mut(),
            budget_cycles: 168_000, // ~1ms
            deadline_cycles: 0,
            miss_count: 0,
            degraded: 0,
            period_cycles: 0, // 由 workq_add_periodic 声明
            next_run_cycles: 0,
            queued: 0,
            pnext: core::ptr::null_mut(),
        };
        SENSORS_ITEM = rtos_work_t {
            next: core::ptr::null_mut(),
            fn_: Some(sensors_work),
            arg: core::ptr::null_mut(),
            budget_cycles: 50_400, // ★design.md §5: sensors 预算 300µs @168MHz
            deadline_cycles: 0,
            miss_count: 0,
            degraded: 0,
            period_cycles: 0, // 由 workq_add_periodic 声明
            next_run_cycles: 0,
            queued: 0,
            pnext: core::ptr::null_mut(),
        };
        ATT_ITEM = rtos_work_t {
            next: core::ptr::null_mut(),
            fn_: Some(attitude_work),
            arg: core::ptr::null_mut(),
            budget_cycles: 33_600, // ★design.md §5: attitude 预算 200µs @168MHz
            deadline_cycles: 0,
            miss_count: 0,
            degraded: 0,
            period_cycles: 0, // 由 workq_add_periodic 声明
            next_run_cycles: 0,
            queued: 0,
            pnext: core::ptr::null_mut(),
        };
    }
    // attitude item 的控制器静态态 + sensors item 的静态态
    crate::flyctrl::control::ctrl_init();
    crate::flyctrl::sensors_task::sensors_init();
    // ★须在 sensors_init 之后才 arm EXTI（否则 ISR 提前触发 → 静态未初始化 ✗）
    crate::flyctrl::init_imu_drdy();
    // ★design.md §5#4：**队列间带宽隔离** —— 给每条队列设 CPU 配额（cycles/突发）。
    //   L2(estimator+attitude+sensors) 上限 ~4ms；L3(nav/telem/uplink) 上限 ~1ms。
    if let Some(f) = slot().workq_set_quota {
        f(Q_L2, 300_000);  // ★design.md §5#4：须 ≥ 三项预算和(218.4k) + 裕量（原 672k < 688k 自相矛盾）
        f(Q_L3, 168_000);  // ~1ms
    }
    crate::flyctrl::nav_task::nav_init();
    crate::flyctrl::telemetry::telemetry_init();
    crate::flyctrl::uplink::uplink_init();

    if let Some(f) = slot().workq_create {
        f(Q_L2, b"wql2\0".as_ptr() as *const _, 5, // ★design.md §4：L2 必须低于全部 L1（rate2/alloc3/safety4）
          unsafe { core::ptr::addr_of_mut!(L2_WQ_STACK).cast::<u8>() }, 12288);
        f(Q_L3, b"wql3\0".as_ptr() as *const _, 10,
          unsafe { core::ptr::addr_of_mut!(L3_WQ_STACK).cast::<u8>() }, 2048);
    }
    unsafe {
        // ★design.md §5：【周期由 WorkItem 声明】—— 队列**自带调度器**按 EDF 派发，
        //   不再用外部硬件定时器各自 submit ✗（那样等于把频率写在定时器里、且 EDF 失效）。
        //   预算/周期/截止期全部落在 item 上 ⇒ §5#1「每个 WorkItem 声明」+ §5#3「队列内 EDF」✓。
        // ★周期单位是 **ms**：内核用【实测 cycles/ms】换算 ⇒ App 侧不写任何频率常量 ✓
        if let Some(f) = slot().workq_add_periodic {
            f(Q_L2, core::ptr::addr_of_mut!(ATT_ITEM), 4);      // attitude  250Hz
            f(Q_L2, core::ptr::addr_of_mut!(EKF_ITEM), 5);      // estimator 200Hz
            f(Q_L2, core::ptr::addr_of_mut!(SENSORS_ITEM), 5);  // sensors   200Hz
            f(Q_L3, core::ptr::addr_of_mut!(NAV_ITEM), 20);     // L3 50Hz
            f(Q_L3, core::ptr::addr_of_mut!(TELEM_ITEM), 20);
            f(Q_L3, core::ptr::addr_of_mut!(UPLINK_ITEM), 20);
            f(Q_L3, core::ptr::addr_of_mut!(LOG_ITEM), 20);     // 日志（L3 事件驱动·可丢弃）
        }
    }
    // ★诊断：打印内核实测 cycles/ms（仿真器声明 84MHz ⇒ 期望 ~84000；真机 168MHz ⇒ ~168000）
    let cpu_per_ms = slot().cycles_per_ms.map(|f| f()).unwrap_or(0);
    info!(tag: "wq", "calib: cycles/ms={} (仿真器期望 ~84000，真机 ~168000)", cpu_per_ms);
    info!(tag: "wq", "design.md §5: L2/L3 队列自带调度器（item 声明周期 + EDF）ready");
}

/// ★design.md §3：DRDY 到（ISR）。
/// ⚠️**当前不在此提交 `sensors` item** —— 1kHz 提交会把 L2 worker（单线程）压垮 ⇒
///   estimator/attitude 饿死 ✗。正确形态：**IMU 采样直接在 L0 ISR 内完成**（读→`ImuRing`），
///   `sensors` item（慢传感器）仍由 250Hz 定时器提交。该迁移为下一步（需拆 SENSOR_FRAME 写者）。
pub fn on_imu_drdy() {
    // 占位：IMU 采样迁入 ISR 后在此调用（见上）。
}

/// setup 任务入口：装配后常驻。
pub extern "C" fn setup_entry(_arg: *mut c_void) {
    setup();
    loop {
        msleep(1000);
    }
}
