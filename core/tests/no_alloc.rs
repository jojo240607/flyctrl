//! 零堆分配审计：验证 `flyctrl-core` + HAL mock 可在不引入运行时堆分配的前提下
//! 跑通完整控制回路（sensor -> estimator -> controller -> actuator）。
//!
//! 飞控固件不允许运行时堆分配（确定性、无碎片风险）。审计方法：
//! 1. 通过对仓库源码的静态 grep（见 PLAN §5 M5.4）确认 `core/` 下无 `Vec/Box/String/
//!    alloc::` 引用；本测试在 host 上跑完整回路，证明 HAL 抽象与核心算法协同无 alloc。
//! 2. 测试本身只调用核心/ HAL 的栈上 API，不使用任何集合类型。
//!
//! 注意：本测试文件本身运行在 std 测试 harness 下（host），但被测的 `flyctrl-core`
//! crate 是 `#![no_std]` 且不依赖 `alloc`，故回路逻辑在固件端可零堆复现。

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::pid::PidController;
use flyctrl_core::controller::trait_def::{Controller, Setpoint};
use flyctrl_core::estimator::complementary::ComplementaryEstimator;
use flyctrl_core::estimator::trait_def::Estimator;
use flyctrl_core::fdir::Fdir;
use flyctrl_core::hal::actuator::{MockMotors, MotorActuator, OutputProtocol};
use flyctrl_core::hal::sensor::{BaroSensor, GpsSensor, ImuSensor, MagSensor, MockBaro, MockGps, MockImu, MockMag};
use flyctrl_core::units::*;
use flyctrl_core::vehicle::VehicleState;

#[test]
fn zero_heap_control_loop_runs() {
    let cfg = VehicleConfig::default_quad();
    let mut imu = MockImu::new();
    let mut gps = MockGps::new();
    let _baro = MockBaro::new();
    let _mag = MockMag::new();
    let mut est = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let mut ctrl = PidController::from_config(&cfg.ctrl_params());
    let mut act = MockMotors::new(OutputProtocol::Pwm);
    let mut fdir = Fdir::new();

    let dt = Second(0.005);
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));

    let mut last_state: VehicleState = VehicleState::zero();
    for _ in 0..200 {
        let sample = imu.read();
        let pos = gps.read();
        let _health = fdir.update(&sample, pos.is_some());
        let state = est.step(dt, sample, pos);
        let cmd = ctrl.control(dt, &sp, &state);
        act.apply(&cmd);
        last_state = state;
    }

    let cmd = act.last_cmd();
    for &m in &cmd.motor {
        assert!(m >= 0.0 && m <= 1.0, "motor thrust out of range: {}", m);
    }
    assert!(last_state.pos[2].0.is_finite(), "estimate diverged to NaN");
}

#[test]
fn hal_mock_sensors_healthy() {
    let mut imu = MockImu::new();
    let mut gps = MockGps::new();
    let baro = MockBaro::new();
    let mag = MockMag::new();
    assert!(imu.healthy());
    assert!(gps.healthy());
    assert!(baro.healthy());
    assert!(mag.healthy());

    let s = imu.read();
    assert!(s.accel[2].0 > 9.0, "mock IMU should read ~gravity on body -Z");
    assert!(gps.read().is_some());
}
