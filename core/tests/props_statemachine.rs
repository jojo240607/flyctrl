//! M7.2 类型级安全网测试。
//!
//! 验证：
//! 1. 解锁前必须持有健康许可（`ArmPermit`）——不健康时 `request_arm` 返回 None，
//!    且类型系统强制 `arm` 必须消费令牌（编译期不可绕过）。
//! 2. 失控保护（Failsafe）单向：Armed → Failsafe 后，只能经显式 reset 回到
//!    Disarmed，不可能再回到 Armed（类型级无此流转）。
//! 3. 校准后不能直解解锁：Calibrating 只暴露 `finish`（回到 Disarmed）。

use flyctrl_core::fdir::Fdir;
use flyctrl_core::state::{Armed, Disarmed, Failsafe, Fcs};
use flyctrl_core::units::Second;
use flyctrl_core::vehicle::{ImuSample, MeterPerSecondSquared, RadianPerSecond};

#[test]
fn prop_arm_requires_health_permit() {
    let fcs = Fcs::<Disarmed>::new();

    // 不健康：拿不到许可。
    let none = fcs.request_arm(false);
    assert!(none.is_none(), "不健康时不得发放解锁许可");

    // 健康：拿到许可，可解锁。
    let permit = fcs.request_arm(true);
    assert!(permit.is_some(), "健康时应发放解锁许可");
    let armed = fcs.arm(permit.unwrap());
    let _: Fcs<Armed> = armed;
}

#[test]
fn prop_failsafe_one_way_type() {
    // 构造一个已上锁状态。
    let fcs = Fcs::<Disarmed>::new();
    let permit = fcs.request_arm(true).unwrap();
    let armed = fcs.arm(permit);

    // 失控进入 Failsafe（类型变为 Fcs<Failsafe>）。
    let fs: Fcs<Failsafe> = armed.failsafe();
    // Failsafe 仅暴露 reset -> Disarmed，没有回到 Armed 的路径。
    let back: Fcs<Disarmed> = fs.reset();
    // 若想重新解锁仍需健康许可（类型级保证不可能"绕过"）。
    let p2 = back.request_arm(false);
    assert!(p2.is_none(), "复位后仍需健康检查才能再次解锁");
}

#[test]
fn prop_calibration_cannot_arm_directly() {
    // Disarmed -> Calibrating -> (finish) -> Disarmed。
    // Calibrating 类型不提供 arm，编译期保证"校准中不可解锁"。
    let fcs = Fcs::<Disarmed>::new();
    let cal = fcs.start_calibration();
    let back: Fcs<Disarmed> = cal.finish();
    let _ = back;
}

#[test]
fn prop_fdir_health_gates_permit() {
    // 真实串接：FDIR 判定健康 -> 才能发解锁许可。
    let mut fdir = Fdir::new();
    // 正常 IMU（加速度 z 轴约 -g，且每拍有微小变化）。
    let mut imu = ImuSample {
        accel: [MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(-9.81)],
        gyro: [RadianPerSecond(0.1), RadianPerSecond(-0.05), RadianPerSecond(0.02)],
    };
    let mut healthy = false;
    for step in 0..50 {
        imu.accel[0] = MeterPerSecondSquared((step as f32) * 1e-4); // 让 IMU 不冻结
        let h = fdir.update(&imu, true, true, true);
        if h == flyctrl_core::fdir::Health::Nominal {
            healthy = true;
        }
        let fcs = Fcs::<Disarmed>::new();
        let permit = fcs.request_arm(healthy);
        if step >= 5 {
            assert!(permit.is_some(), "健康后必须能解锁");
        }
    }
    let _ = Second::ZERO;
}
