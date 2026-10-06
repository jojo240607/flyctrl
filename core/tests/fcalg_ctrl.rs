//! fcalg **控制器**桥接验收（feature `fcalg-ctrl`）—— 只判**能判的**；
//! 判不了的（倾角符号约定）在模块头**显式登记**，不假装已验证。
#![cfg(feature = "fcalg-ctrl")]

use flyctrl_core::controller::trait_def::{Controller, Setpoint};
use flyctrl_core::estimator::fcalg_ctrl_bridge::FcalgController;
use flyctrl_core::units::{Meter, Radian, Second};
use flyctrl_core::vehicle::VehicleState;

const DT: f32 = 0.004;

/// 物理健全性：水平估计 + 悬停设定点 ⇒ **四桨等推力**（净加速度为 0 ⇒ 推力比 = 1）。
#[test]
fn hover_level_gives_equal_motor_thrust() {
    let mut c = FcalgController::new();
    let est = VehicleState::zero();
    let sp = Setpoint::hover([Meter(0.0); 3], Radian(0.0));
    let cmd = c.control(Second(DT), &sp, &est);
    for i in 0..4 {
        assert!(
            (cmd.motor[i] - cmd.motor[0]).abs() < 1e-6,
            "零力矩下四桨必须等推力: {:?}",
            cmd.motor
        );
        assert!(cmd.motor[i] > 0.5, "悬停推力不得为零: {:?}", cmd.motor);
    }
    // 速率设定值必须有效且为零（已达期望姿态）
    let rs = c.rate_setpoint();
    assert!(rs.valid);
    for a in 0..3 {
        assert!(rs.rates[a].abs() < 1e-5, "水平+水平设定 ⇒ 速率设定应为 0: {:?}", rs.rates);
    }
}

/// 零加速度设定 + 偏航设定 ⇒ 期望姿态退化为**纯偏航**（有判据的那一条构造性质）。
#[test]
fn zero_accel_setpoint_is_pure_yaw() {
    let mut c = FcalgController::new();
    let est = VehicleState::zero(); // 水平
    let sp = Setpoint::hover([Meter(0.0); 3], Radian(0.5));
    let _ = c.control(Second(DT), &sp, &est);
    let r = c.rate_setpoint().rates;
    assert!(r[0].abs() < 1e-4 && r[1].abs() < 1e-4, "纯偏航设定不得产生 roll/pitch 速率: {r:?}");
    assert!((r[2] - c.kp_att * 0.5).abs() < 1e-3, "yaw 速率须 = kp·误差: {} vs {}", r[2], c.kp_att * 0.5);
}

/// 姿态环：估计有 roll 偏时，速率设定必须 = kp·(姿态误差)，**方向为纠偏方向**。
#[test]
fn rate_setpoint_is_kp_times_attitude_error_with_correct_sign() {
    let mut c = FcalgController::new();
    let mut est = VehicleState::zero();
    let q = fcalg::quat::Quat::from_euler_zyx([0.1, 0.0, 0.0]); // 估计 roll = +0.1
    est.att = flyctrl_core::vehicle::Quaternion { w: q.w, x: q.x, y: q.y, z: q.z };
    let sp = Setpoint::hover([Meter(0.0); 3], Radian(0.0));
    let _ = c.control(Second(DT), &sp, &est);
    let r = c.rate_setpoint().rates;
    // 正 roll 需要**负** roll 速率来纠偏（右乘误差态约定）
    assert!((r[0] + c.kp_att * 0.1).abs() < 1e-3, "roll 速率须 = −kp·误差: {}", r[0]);
}

/// 两处"无对应"必须**计数**（绝不静默 no-op）—— 与估计器桥接同一条纪律。
#[test]
fn unimplemented_methods_are_counted() {
    let mut c = FcalgController::new();
    assert_eq!(c.not_impl_calls, 0);
    c.touch_unimplemented();
    assert_eq!(c.not_impl_calls, 2, "set_measured_airspeed_vec / set_world_accel 各计一次");
}

/// `reset` 必须让速率设定值回到 INVALID（接线后会被调用）。
#[test]
fn reset_invalidates_rate_setpoint() {
    let mut c = FcalgController::new();
    let est = VehicleState::zero();
    let sp = Setpoint::hover([Meter(0.0); 3], Radian(0.0));
    let _ = c.control(Second(DT), &sp, &est);
    assert!(c.rate_setpoint().valid);
    c.reset();
    assert!(!c.rate_setpoint().valid, "reset 后速率设定必须无效");
}
