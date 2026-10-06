//! ★design.md L2 `wq:estimator`：**EKF/FDIR 任务**（250Hz，软实时，带预算）。
//!
//! 分层依据（`docs/fc-task-architecture.md`）：
//!   · L1 `rate`(1kHz)  = 薄硬实时：陀螺 → 速率 PID → 混控 → PWM（**不含 EKF**）；
//!   · **L2 `ekf`(250Hz)** = 本任务：IMU 预处理 + ESKF + FDIR + 初始化门控 → 发 `EST_STATE`/`HIL_DIAG`；
//!   · L2 `control`(250Hz) = 姿态层：读 `EST_STATE` → 姿态 P → 发 `RATE_CMD`。
//!
//! 动机：本仓 ESKF 实测 ~2–3ms/次（1kHz 下打满 CPU ⇒ 饿死低优先级任务 ✗，已实测）。
//! 把它从 L1 速率环剥离 ⇒ 速率环不受 EKF 抖动影响，且 EKF 超预算时**只影响自己** ✓。

use core::ffi::c_void;

use flyctrl_core::controller::PidController;
use flyctrl_core::estimator::select::AnyEstimator;
use flyctrl_core::hil::{HilContext, SimImu};
use flyctrl_core::units::Second;

use rtos_app_sdk::rtos::{delay_until, tick_count};
use rtos_app_sdk::info;

use crate::flyctrl::rt_stat::RtStat;
use crate::flyctrl::{EST_STATE, HIL_DIAG, SENSOR_FRAME, SENSOR_SEQ, SETPOINT};

/// L2 估计器任务入口（250Hz）。
pub extern "C" fn ekf_entry(_arg: *mut c_void) {
    info!(tag: "ekf", "task started; period=4ms (250Hz, L2 estimator)");

    // EKF/FDIR 上下文（自 `rate_task` 迁入 ✓）。
    let mut hil = HilContext::new(
        AnyEstimator::default_product(),
        PidController::default_quad(),
        Second(4.0 / 1000.0),
    );
    // —— EKF 配置（真机路径口径 ✓，自 control/rate 原样迁入）——
    hil.est.set_observation_noise(0.25, 0.01, 0.09);
    unsafe {
        flyctrl_core::estimator::eskf::G_ESKF_MAG_HDG_GATE = 2.0;
        flyctrl_core::estimator::eskf::G_ESKF_MAG_YAW_ON = 0.0; // 0.0 = AUTO（一手默认 ✓）
    }
    {
        let k = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(flyctrl_core::estimator::eskf::G_ESKF_FREEZE_BIAS))
        };
        if k >= 0.5 {
            hil.est.set_freeze_bias(true);
        }
    }
    let mut sim_imu = SimImu::new();

    let mut wake_tick = tick_count();
    let mut last_seq: u32 = 0;
    let mut first = true;
    let mut st = RtStat::new("ekf");
    let mut it: u32 = 0;
    loop {
        let t0 = st.tick();
        // 当拍传感器帧（seqlock 计数；`seq` 变且为偶 ⇒ 新完整帧 ✓）。
        let (imu, gps, baro_alt, mag, rc, armed, seq_now) = unsafe {
            let f = &*core::ptr::addr_of!(SENSOR_FRAME);
            (f.imu, f.gps, f.baro_alt, f.mag, f.rc, f.armed, SENSOR_SEQ)
        };
        let fresh = seq_now != last_seq && (seq_now & 1) == 0;
        last_seq = seq_now;
        let imu_in = if fresh { imu } else { None };

        // 姿态层发布的设定点（仅供 EKF 初始化/门控 ✓）。
        let (sp, sp_valid) = unsafe {
            let s = &*core::ptr::addr_of!(SETPOINT);
            (s.sp, s.valid != 0)
        };

        // EKF + FDIR + 初始化门控（250Hz ✓）。
        let (est, health, gated) = hil.ekf_hil(
            imu_in, gps, baro_alt, None, None, mag, &sp, sp_valid, armed, rc.fresh, &mut sim_imu,
        );

        // 发布估计状态（control/遥测读 ✓）。写者 ekf(5) 与读者 control(6)/telem 不同优先级，
        // 允许撕裂由下一拍恢复（同 SENSOR_FRAME 论证 ✓）。
        unsafe {
            let s = &mut *core::ptr::addr_of_mut!(EST_STATE);
            s.est = est;
            s.health = health;
            s.armed = armed;
        }
        // 发布 EKF 诊断（world_accel 供姿态层速度环 D 项 ✓ + mag 内部量 ✓）。
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
        }

        st.sample(st.tick().wrapping_sub(t0));
        // ★2026-10-05 诊断出口（一次性）：判定"重力观测到底有没有在起作用" ✗
        //   判据 #7 已证：姿态残差永久冻结的直接原因是【重力观测被拒】✗ ⇒
        //   这里把采用数/被门掉数/丢帧数打出来，真链路跑一次即可判定 ✓
        if it == 250 {
            info!(tag: "diag",
                "grav_a={} grav_g={} imu_rej={} imu_out={} step={}",
                hil.est.inner.n_grav_applied, hil.est.inner.n_grav_gated,
                hil.n_imu_rejected, hil.n_imu_step_outlier, hil.est.inner.n_step);
        }
        it = it.wrapping_add(1);
        if it % 250 == 0 { st.report_and_reset(); } // ★P1-2 可观测
        if first {
            first = false;
            info!(tag: "ekf", "first loop done; gated={}", gated);
        }

        // 250Hz 节拍（4ms；1ms RTOS tick 网格 ► 满足 250Hz 名义，硬件定时器化见 P1-1）。
        delay_until(&mut wake_tick, 4);
    }
}
