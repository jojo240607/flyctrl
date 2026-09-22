//! ★**证明 ESKF 适配器的每条通路确实在运行**（`docs/c1-migration-plan.md` 步 2 ✓）
//!
//! 纪律（本会话头号 ✓）：**不断言"实现存在"，而断言"通路被走到"** ✓。
//! 每条计数都必须非零 —— 含【门关闭】与【显式拒绝】两类 ✓。

use flyctrl_core::estimator::eskf_estimator::EskfEstimator;
use flyctrl_core::estimator::trait_def::Estimator;
use flyctrl_core::units::{
    Airspeed, Meter, MeterPerSecond, MeterPerSecondSquared, Radian, RadianPerSecond, Second,
};
use flyctrl_core::vehicle::{
    AirspeedSample, ImuSample, PosSample, Quaternion, RtkSample, VioSample,
};

fn imu_hover() -> ImuSample {
    ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(-9.81),
        ],
        gyro: [RadianPerSecond(0.0); 3],
    }
}

#[test]
fn every_adapter_path_is_exercised() {
    let q0 = flyctrl_core::vehicle::Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0));
    let mut e = EskfEstimator::new(q0, [0.0; 3], [0.0; 3], 5.0, [0.2, 0.0, 0.4]);
    let dt = Second(0.004);

    // 1) 悬停拍 ⇒ 重力辅助【应用】✓（比力 ≈ 纯重力 ⇒ 门开 ✓）
    for _ in 0..20 {
        let _ = e.step(dt, imu_hover(), None, None);
    }

    // 2) ★大加速度拍 ⇒ 重力辅助【被加速度门拒绝】✓
    //    物理：向上冲击 ⇒ 机体系比力 ≈ 4.5g ⇒ dev > 0.25g ⇒ 门关 ✓（C1 版 A11 的机制 ✓）
    let imu_hi = ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(-3.5 * 9.81),
        ],
        gyro: [RadianPerSecond(0.0); 3],
    };
    let _ = e.step(dt, imu_hi, None, None);

    // 3) GPS 位置 + Doppler 速度 ✓
    let _ = e.step(
        dt,
        imu_hover(),
        Some(PosSample {
            pos: [Meter(1.0), Meter(2.0), Meter(-3.0)],
            vel: Some([
                MeterPerSecond(0.1),
                MeterPerSecond(0.0),
                MeterPerSecond(0.0),
            ]),
        }),
        None,
    );

    // 4) 气压：先给一个【远离估计】的高度（应被 NIS 门拒 ✓），再给一个【合理】的 ✓
    e.update_alt(10.0); // 初值 p=0 ⇒ 新息大 ⇒ 预期被拒 ✓（拒绝是对的 ✓）
    e.update_alt(0.05); // 合理 ⇒ 预期被接受 ✓

    // 5) 磁（首个 ⇒ `reset_mag_states` 代数反解 ✓）再给一个 ✓
    e.update_mag(Some([0.2, 0.0, 0.4]));
    e.update_mag(Some([0.2, 0.0, 0.4]));

    // 6) ★无等价观测 ⇒ **显式拒绝**（绝不静默 ✗）✓
    e.update_vio(Some(VioSample {
        pos: Some([Meter(0.0); 3]),
        vel: None,
    }));
    e.update_rtk(Some(RtkSample {
        pos: [Meter(0.0); 3],
    }));
    let _ = e.step(
        dt,
        imu_hover(),
        None,
        Some(AirspeedSample {
            speed: Airspeed(5.0),
            timestamp_s: 0.0,
        }),
    );

    // 7) 初值设置与复位 ✓
    e.set_initial_attitude(flyctrl_core::vehicle::Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0)));
    e.set_initial_position([1.0, 2.0, 3.0]);
    let _ = e.state();
    let _ = e.accel_bias();

    // ★逐计数断言（每条通路都必须在运行 ✓）
    // ★口径（A11 教训推广 ✓）：每通路断言【已应用 + 被拒绝 > 0】，
    //   而不是只断言"已应用" ✗ —— 否则"拒绝"会被误读成"通路没走到" ✗。
    let checks: &[(&str, u32)] = &[
        ("step", e.n_step),
        ("重力辅助(应用+门拒)", e.n_grav_applied + e.n_grav_gated),
        ("磁(应用+门拒)", e.n_mag + e.n_mag_rejected),
        ("重力辅助已应用", e.n_grav_applied),
        ("★重力辅助被门拒绝", e.n_grav_gated),
        ("气压(已应用+被拒)", e.n_baro + e.n_baro_rejected),
        ("磁", e.n_mag),
        ("GPS位置(已应用+被拒)", e.n_gps_pos + e.n_gps_pos_rejected),
        ("GPS速度(已应用+被拒)", e.n_gps_vel + e.n_gps_vel_rejected),
        ("★VIO显式拒绝", e.n_vio_refused),
        ("★RTK显式拒绝", e.n_rtk_refused),
        ("★空速显式拒绝", e.n_airspeed_refused),
        ("磁首样本触发", if e.filter().mag_i == [0.0; 3] { 0 } else { 1 }),
    ];
    println!("\n[ESKF 适配器通路] 逐计数 ✓");
    for (name, n) in checks {
        println!("  {name:>22}: {n}");
        assert!(*n > 0, "通路【{name}】计数为 0 ✗ ⇒ 该通路未被走到 ⇒ 判据可能空洞 ✗");
    }
}
