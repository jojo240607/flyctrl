//! fcalg 桥接验收（feature `fcalg-est`）—— **通过 `Estimator` trait 驱动**，证明接线可用。
//!
//! 口径 (b)：只要求桥接正确 + 新栈满足自身契约，**不与旧栈逐位等价**。
#![cfg(feature = "fcalg-est")]

use std::sync::atomic::Ordering;

use flyctrl_core::estimator::fcalg_bridge::{FcalgEstimator, NOT_IMPLEMENTED_CALLS};
use flyctrl_core::estimator::trait_def::Estimator;
use flyctrl_core::units::{
    Meter, MeterPerSecond, MeterPerSecondSquared, RadianPerSecond, Second,
};
use flyctrl_core::vehicle::{ImuSample, PosSample, Quaternion};

const G: f32 = 9.806_65;

/// 静态悬停（真值恒停）下，经 trait 驱动必须收敛且不发散。
/// 比力取 `(0,0,−g)`（契约：静止水平时比力指向天，机体系 z 向下）。
#[test]
fn bridge_converges_on_static_hover() {
    // ★增量式快照：`NOT_IMPLEMENTED_CALLS` 是**进程级全局**，而同一测试二进制的用例
    //   **默认并行** —— 断言绝对值会被相邻用例污染（本会话第二次栽在这个模式上：
    //   上次是 L1 的 finite 计数器）。故只断言"本用例"没有增加它 ⇒ 并行安全。
    // 按**实例**断言（并行安全）：全局静态会被相邻用例并发增加，增量法也挡不住。
    let mut e = FcalgEstimator::new();
    e.set_initial_attitude(Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    e.set_initial_position([0.0; 3]);

    let dt = 0.005f32;
    let acc = [0.0f32, 0.0, -G];
    for k in 0..600 {
        // 逐样本 predict（固件环形路径）
        e.predict_delta([0.0; 3], [acc[0] * dt, acc[1] * dt, acc[2] * dt], dt, dt);
        // 气压：局部高度 0（契约 alt == −p_z）
        e.update_alt(0.0);
        if k % 10 == 0 {
            // GPS 位置 + 速度（`vel` 是 Option ⇒ 桥接分两路）
            e.update_fusion(
                Some(PosSample { pos: [Meter(0.0); 3], vel: Some([MeterPerSecond(0.0); 3]) }),
                None,
            );
            e.update_mag(None); // 无磁 ⇒ 不得有副作用
        }
    }
    // 单帧路径也过一遍（`step` 是另一条入口，必须同样可用）
    let imu = ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(-G),
        ],
        gyro: [RadianPerSecond(0.0); 3],
    };
    let s = e.step(Second(dt), imu, None, None);

    assert!(s.att.w.abs() > 0.999, "静态悬停下姿态应≈单位四元数: {:?}", s.att);
    eprintln!(
        "[hov] 终态 pos=({:.4},{:.4},{:.4}) vel=({:.4},{:.4},{:.4}) qw={:.6} rej={} ba=({:.5},{:.5},{:.5})",
        s.pos[0].0, s.pos[1].0, s.pos[2].0, s.vel[0].0, s.vel[1].0, s.vel[2].0, s.att.w,
        e.fuse_rejects, s.accel_bias[0], s.accel_bias[1], s.accel_bias[2]
    );
    // 界**有依据**：GPS 位置 σ=0.5 m（参数表 obs.sigma_gps_p）⇒ 取 2σ = 1.0 m。
    // 实测终值为 0.0000（见上面的 [hov] 行），远在界内 —— 这个界不是"调大到绿"。
    for k in 0..3 {
        assert!(s.pos[k].0.abs() < 1.0, "位置须在 2σ(GPS) 内: {:?}", s.pos);
        assert!(s.vel[k].0.abs() < 1.0, "速度须在 2σ 内: {:?}", s.vel);
    }
    assert!(s.accel_bias.iter().all(|x| x.is_finite()));
    assert!(
        e.accel_bias().iter().all(|x| x.is_finite()),
        "accel_bias 通路必须可用"
    );
    assert_eq!(e.not_impl_calls, 0, "本用例只走已实现通路 ⇒ 实例计数必须为 0");
}

/// **"绝不静默"在接线层的判据**：三处"无对应"被调用时必须**计数**，
/// 而不是悄悄 no-op（否则将来会有人以为群延迟补偿/VIO/RTK 在起作用）。
#[test]
fn unimplemented_entry_points_are_counted_not_silent() {
    let mut e = FcalgEstimator::new();
    e.set_accel_lag_s(0.015);
    e.update_vio(None);
    e.update_rtk(None);
    assert_eq!(e.not_impl_calls, 3, "三处无对应必须各计一次（绝不静默 no-op）");
    // 全局静态只作聚合，**不断言其绝对值**（并行下不可靠 —— 本会话两次教训）
    assert!(NOT_IMPLEMENTED_CALLS.load(Ordering::Relaxed) >= 3);
}

/// `reset` 必须回到可用初态（接线后会被调用，不得留下脏状态）。
#[test]
fn reset_returns_to_usable_initial_state() {
    let mut e = FcalgEstimator::new();
    e.set_initial_position([100.0, -50.0, 20.0]);
    e.reset();
    let s = e.state();
    for k in 0..3 {
        assert_eq!(s.pos[k].0, 0.0, "reset 后位置必须归零");
    }
    assert!(s.att.w.abs() > 0.999, "reset 后姿态必须为单位");
}

/// ★**闭合判据**：桥接的诊断计数器必须与**实际调用次数**一致
/// （app 侧的诊断日志读的正是这些数 —— 漏计会静默地误导）。
#[test]
fn diagnostic_counters_close_with_actual_calls() {
    let mut e = FcalgEstimator::new();
    e.set_initial_attitude(Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    let dt = 0.005f32;
    let f_b = [0.0f32, 0.0, -G];
    let n_pred = 100;
    let n_fuse = 10;
    for k in 0..n_pred {
        e.predict_delta([0.0; 3], [f_b[0] * dt, f_b[1] * dt, f_b[2] * dt], dt, dt);
        if k % (n_pred / n_fuse) == 0 {
            e.update_fusion(
                Some(PosSample { pos: [Meter(0.0); 3], vel: Some([MeterPerSecond(0.0); 3]) }),
                None,
            );
        }
    }
    assert_eq!(e.n_step, n_pred as u64, "步数计数必须闭合");
    assert_eq!(e.n_gps_pos, n_fuse, "GPS 位接受数必须闭合");
    assert_eq!(e.n_gps_vel, n_fuse, "GPS 速接受数必须闭合");
    assert_eq!(e.n_gps_pos_rejected, 0);
    assert_eq!(e.n_gps_vel_rejected, 0);
    // 重力：每拍都试 ⇒ 应用 + 门控必须恰好等于步数
    assert_eq!(
        e.n_grav_applied + e.n_grav_gated,
        n_pred as u32,
        "重力路每拍必计数（applied={} gated={}）",
        e.n_grav_applied,
        e.n_grav_gated
    );
    assert!(e.n_grav_applied > 0, "静止悬停的比力应通过量级门");
    // 无对应项必须恒 0（登记为"不假装有数"）
    assert_eq!(e.n_mag_reanchored, 0);
    // 磁路未喂 ⇒ 两计数都应为 0（不得凭空增加）
    assert_eq!((e.n_mag, e.n_mag_rejected), (0, 0));

    // 喂一个荒谬 GPS ⇒ 拒收计数必须 +1，而接受数不变（闭合仍然成立）
    let before = (e.n_gps_pos, e.n_gps_pos_rejected);
    let bogus = PosSample {
        pos: [Meter(1e5), Meter(-1e5), Meter(1e5)],
        vel: None,
    };
    e.update_fusion(Some(bogus), None);
    assert_eq!(e.n_gps_pos, before.0, "被拒不得计入接受数");
    assert_eq!(e.n_gps_pos_rejected, before.1 + 1, "被拒必须计入拒收数");
}

/// ★`world_accel()` 的**定义性判据**：静止悬停（水平 + 比力 (0,0,−g)）时世界系加速度必须 ≈ 0
/// （推力恰好抵消重力）。这也是它当初存在的理由：给 HIL 诊断一个"净加速度"读数。
#[test]
fn world_accel_is_zero_at_static_hover() {
    let mut e = FcalgEstimator::new();
    e.set_initial_attitude(Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    let dt = 0.005f32;
    let f_b = [0.0f32, 0.0, -G];
    for _ in 0..50 {
        e.predict_delta([0.0; 3], [f_b[0] * dt, f_b[1] * dt, f_b[2] * dt], dt, dt);
    }
    let wa = e.world_accel();
    for a in 0..3 {
        assert!(
            wa[a].abs() < 0.05,
            "静止悬停下世界系加速度应 ≈ 0: {wa:?}（推力未抵消重力？）"
        );
    }
    // 对照：比力**大于** G（推得比悬停更用力）⇒ 净加速度**向上**（NED 里 z 为**负**）。
    // ★首版写成 `-G + 1.0`（= −8.8，**比悬停更轻**）却断言向上 ✗ —— 模块给的 +1.0
    //   才是对的（推得轻 ⇒ 向下加速）。**又一次是我的直觉错、模块对**（同 NED 的 p_z 那次）。
    let f_up = [0.0f32, 0.0, -G - 1.0];
    e.predict_delta([0.0; 3], [f_up[0] * dt, f_up[1] * dt, f_up[2] * dt], dt, dt);
    let wa2 = e.world_accel();
    assert!(wa2[2] < -0.5, "净上行比力应给出向上的（NED −z）加速度: {wa2:?}");
}
