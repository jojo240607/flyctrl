//! M7.1 关键不变量属性测试（host 端）。
//!
//! 用确定性 LCG 随机化生成大量输入，穷举验证四类核心不变量：
//! - 四元数单位范数在估计/合成后维持
//! - 电机指令恒有界 [0,1]
//! - EKF 协方差矩阵恒对称半正定
//! - 整链（传感器→估计→控制）无 NaN 发散
//!
//! 不引入 proptest 依赖（保持零外部依赖、可复现），自实现小型 LCG。

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::pid::PidController;
use flyctrl_core::controller::Controller;
use flyctrl_core::estimator::ekf::EkfEstimator;
use flyctrl_core::estimator::Estimator;
use flyctrl_core::fdir::Fdir;
use flyctrl_core::invariants::{
    actuator_bounded, cov_psd, quat_is_unit, state_finite, QUAT_NORM_TOL,
};
use flyctrl_core::units::*;
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, PosSample, Quaternion, VehicleState};

/// 确定性 LCG（线性同余），用于可复现的随机化输入。
struct Lcg(u64);
impl Lcg {
    fn new(seed: u64) -> Self {
        Lcg(seed.max(1))
    }
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.0
    }
    /// [0,1) 浮点
    fn f(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32
    }
    /// [-mag, mag] 浮点
    fn spread(&mut self, mag: f32) -> f32 {
        (self.f() * 2.0 - 1.0) * mag
    }
}

fn random_att(rng: &mut Lcg) -> Quaternion {
    let mut q = Quaternion {
        w: rng.spread(1.0),
        x: rng.spread(1.0),
        y: rng.spread(1.0),
        z: rng.spread(1.0),
    };
    let n = (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z).sqrt();
    if n > 1e-6 {
        q.w /= n;
        q.x /= n;
        q.y /= n;
        q.z /= n;
    } else {
        q = Quaternion::IDENTITY;
    }
    q
}

fn random_state(rng: &mut Lcg) -> VehicleState {
    VehicleState {
        pos: [
            Meter(rng.spread(50.0)),
            Meter(rng.spread(50.0)),
            Meter(rng.spread(50.0)),
        ],
        vel: [
            MeterPerSecond(rng.spread(10.0)),
            MeterPerSecond(rng.spread(10.0)),
            MeterPerSecond(rng.spread(10.0)),
        ],
        omega: [
            RadianPerSecond(rng.spread(3.0)),
            RadianPerSecond(rng.spread(3.0)),
            RadianPerSecond(rng.spread(3.0)),
        ],
        att: random_att(rng),
        airspeed: MeterPerSecond(rng.spread(10.0)),
    }
}

fn random_imu(rng: &mut Lcg) -> ImuSample {
    ImuSample {
        accel: [
            MeterPerSecondSquared(rng.spread(20.0)),
            MeterPerSecondSquared(rng.spread(20.0)),
            MeterPerSecondSquared(rng.spread(20.0)),
        ],
        gyro: [
            RadianPerSecond(rng.spread(3.0)),
            RadianPerSecond(rng.spread(3.0)),
            RadianPerSecond(rng.spread(3.0)),
        ],
    }
}

#[test]
fn prop_quat_unit_after_random_init() {
    let mut rng = Lcg::new(0x1234_5678);
    for _ in 0..2000 {
        let q = random_att(&mut rng);
        assert!(
            quat_is_unit(q),
            "随机初始化的四元数应单位范数，违反容差 {}",
            QUAT_NORM_TOL
        );
    }
}

#[test]
fn prop_actuator_bounded_under_disturbance() {
    let mut rng = Lcg::new(0x9abc_def0);
    let cfg = VehicleConfig::default_quad();
    let mut ekf = EkfEstimator::default_quad();
    let mut pid = PidController::from_config(&cfg.ctrl_params());
    let sp = flyctrl_core::controller::Setpoint::hover(
        [Meter(0.0), Meter(0.0), Meter(-5.0)],
        Radian(0.0),
    );

    for _ in 0..5000 {
        let s = random_state(&mut rng);
        ekf.reset();
        // 喂几拍含噪声的 IMU，估计当前状态。
        let mut est = s;
        for _ in 0..4 {
            let mut imu = random_imu(&mut rng);
            imu.accel[2] = MeterPerSecondSquared(-9.81 + rng.spread(0.5));
            let z = PosSample {
                pos: [est.pos[0], est.pos[1], est.pos[2]],
            };
            est = ekf.step(Second(0.01), imu, Some(z), None);
        }
        let cmd = pid.control(Second(0.01), &sp, &est);
        assert!(
            actuator_bounded(&cmd),
            "任意扰动下电机指令必须落在 [0,1]，得到 {:?}",
            cmd.motor
        );
    }
}

#[test]
fn prop_ekf_cov_psd_under_noise() {
    let mut rng = Lcg::new(0x55aa_55aa);
    let mut ekf = EkfEstimator::default_quad();

    for _ in 0..1000 {
        ekf.reset();
        let mut s = random_state(&mut rng);
        // 连续推进，并叠加随机 GPS 噪声，检验协方差始终 PSD。
        for _ in 0..8 {
            let mut imu = random_imu(&mut rng);
            imu.accel[2] = MeterPerSecondSquared(-9.81 + rng.spread(0.5));
            let noisy_pos = [
                s.pos[0] + Meter(rng.spread(0.5)),
                s.pos[1] + Meter(rng.spread(0.5)),
                s.pos[2] + Meter(rng.spread(0.5)),
            ];
            let z = PosSample { pos: noisy_pos };
            s = ekf.step(Second(0.01), imu, Some(z), None);
        }
        let p = ekf.cov();
        assert!(
            cov_psd(p, 9),
            "EKF 协方差在噪声观测下必须保持对称半正定"
        );
    }
}

#[test]
fn prop_no_nan_over_full_chain() {
    let mut rng = Lcg::new(0xc0ffee);
    let cfg = VehicleConfig::default_quad();
    let mut ekf = EkfEstimator::default_quad();
    let mut pid = PidController::from_config(&cfg.ctrl_params());
    let sp = flyctrl_core::controller::Setpoint::hover(
        [Meter(0.0), Meter(0.0), Meter(-5.0)],
        Radian(0.0),
    );

    for _ in 0..3000 {
        ekf.reset();
        let mut s = random_state(&mut rng);
        for _ in 0..6 {
            let mut imu = random_imu(&mut rng);
            imu.accel[2] = MeterPerSecondSquared(-9.81 + rng.spread(1.0));
            let z = PosSample {
                pos: [s.pos[0], s.pos[1], s.pos[2]],
            };
            s = ekf.step(Second(0.01), imu, Some(z), None);
        }
        // 估计状态有限。
        assert!(state_finite(&s), "估计状态不得含 NaN/Inf");
        let cmd = pid.control(Second(0.01), &sp, &s);
        // 控制指令有限且有界。
        assert!(actuator_bounded(&cmd));
    }
}

#[test]
fn prop_fdir_critical_is_one_way() {
    // 失控保护必须单向且安全：IMU 冻结（连续卡死）进入 Critical 后，
    // 在故障未清除（仍喂冻结帧）时应保持在 Critical，不会自动恢复为 Nominal。
    let mut fdir = Fdir::new();
    let frozen = ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
        ],
        gyro: [RadianPerSecond(0.0); 3],
    };
    let mut saw_critical = false;
    for _ in 0..100 {
        let h = fdir.update(&frozen, true, true, true); // 位置可用但 IMU 冻结
        if h == flyctrl_core::fdir::Health::Critical {
            saw_critical = true;
        }
        if saw_critical {
            assert!(
                h == flyctrl_core::fdir::Health::Critical,
                "失控保护必须单向：进入 Critical 后不得自动回到 Nominal"
            );
        }
    }
    assert!(saw_critical, "冻结 IMU 应被检测为 Critical");
}

#[test]
fn prop_actuator_saturated_still_bounded() {
    // 即便设定点极端（远超出可达包络），限幅必须保证指令恒在 [0,1]。
    let cfg = VehicleConfig::default_quad();
    let mut pid = PidController::from_config(&cfg.ctrl_params());
    pid.reset();
    let sp = flyctrl_core::controller::Setpoint::hover(
        [Meter(1000.0), Meter(1000.0), Meter(-1000.0)],
        Radian(3.14),
    );
    let est = VehicleState::zero();
    for _ in 0..200 {
        let cmd = pid.control(Second(0.01), &sp, &est);
        assert!(actuator_bounded(&cmd), "极端设定点下仍需限幅到 [0,1]");
    }
    let _ = ActuatorCmd::zero();
}
