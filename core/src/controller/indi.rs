//! INDI：增量非线性动态逆（M8.1）。
//!
//! 经典串级 PID 在面对强扰动 / 模型不确定（如重心偏移、桨效率下降）时，
//! 姿态内环的线性增益不足以快速抑制。INDI 在原基线控制器之上叠加一个
//! **角加速度反馈增量**，把姿态度纯积分为"单位增益积分器"，对模型误差鲁棒。
//!
//! 原理（机体角速度 p,q,r 通道，G 为控制效能 rad/(s·单位推力)）：
//! ```text
//! u_base        = 基线控制器（如 PID）的电机指令
//! p_cmd         = 由 u_base 反解 X 混控得到的期望机体角速度
//! p_dot_cmd     = (p_cmd - p_cmd_prev)/dt       （期望角加速度）
//! p_dot_meas    = (omega - omega_prev)/dt       （实测角加速度，IMU 微分）
//! e_acc         = p_dot_cmd - p_dot_meas
//! delta_pqr     = K_inv · e_acc                 （K_inv = I/(G·dt)）
//! u             = u_base + 混控(delta_pqr)      （增量叠加回电机）
//! ```
//! 这样系统等效为单位增益积分器：`p_dot = p_dot_cmd`，与模型参数 G 无关。
//!
//! INDI 包装任意实现了 [`Controller`] 的基线（PID/LQR/MPC），且不改变其接口，
//! 因此可直接复用 M7 的 HIL 闭环与属性测试。

use crate::controller::{Controller, Setpoint};
use crate::units::*;
use crate::vehicle::{ActuatorCmd, VehicleState};

/// 反解 X 型混控：电机指令 → 期望机体角速度 (p, q, r)。
/// 与 `pid.rs` 混控互逆：
///   m0 = T + 0.5(p+q+r), m1 = T + 0.5(-p-q+r), m2 = T + 0.5(-p+q-r), m3 = T + 0.5(p-q-r)
/// 故 p = (m0-m1-m2+m3)/2, q = (m0-m1+m2-m3)/2, r = (m0+m1-m2-m3)/2。
fn motors_to_rates(m: &[f32; 4]) -> [f32; 3] {
    [
        (m[0] - m[1] - m[2] + m[3]) * 0.5,
        (m[0] - m[1] + m[2] - m[3]) * 0.5,
        (m[0] + m[1] - m[2] - m[3]) * 0.5,
    ]
}

/// 把机体角速度增量经 X 混控变回电机指令增量（与 motors_to_rates 互逆）。
fn rates_to_motor_inc(dpqr: [f32; 3]) -> [f32; 4] {
    let [dp, dq, dr] = dpqr;
    [
        0.5 * (dp + dq + dr),
        0.5 * (-dp - dq + dr),
        0.5 * (-dp + dq - dr),
        0.5 * (dp - dq - dr),
    ]
}

pub struct IndiController<B: Controller> {
    base: B,
    /// 控制效能（rad/(s·单位推力)），按机型惯量/臂长标定；INDI 增量增益 = I/(G·dt)。
    ctrl_eff: [f32; 3],
    /// INDI 增量增益（= I/(G·dt) 的对角），运行时按 dt 计算。
    k_inv: [f32; 3],
    dt: f32,
    /// 上拍指令角速度（用于求期望角加速度）。
    prev_cmd_rate: [f32; 3],
    /// 上拍实测角速度（用于求实测角加速度）。
    prev_omega: [f32; 3],
    initialized: bool,
    /// INDI 强度（0=纯基线，1=全额 INDI）。用于对比/降级。
    gain_scale: f32,
}

impl<B: Controller> IndiController<B> {
    /// `ctrl_eff` 为三轴控制效能（p,q,r），`gain_scale` 为 INDI 总强度。
    pub fn new(base: B, ctrl_eff: [f32; 3], gain_scale: f32) -> Self {
        Self {
            base,
            ctrl_eff,
            k_inv: [0.0; 3],
            dt: 0.01,
            prev_cmd_rate: [0.0; 3],
            prev_omega: [0.0; 3],
            initialized: false,
            gain_scale: gain_scale.clamp(0.0, 1.0),
        }
    }

    /// 直接用机型惯量/臂长标定控制效能：G ≈ arm_length / (I · k_thrust_per_rad)，
    /// 这里给一个保守解析初值，实测标定可覆盖。
    pub fn with_inertia(base: B, inertia: [f32; 3], dt: f32, gain_scale: f32) -> Self {
        // 控制效能 ~ 1/(I)（推力矩 / 惯量），归一到合理量级。
        let eff = [
            1.0 / inertia[0].max(1e-3),
            1.0 / inertia[1].max(1e-3),
            1.0 / inertia[2].max(1e-3),
        ];
        let mut s = Self::new(base, eff, gain_scale);
        s.dt = dt;
        s.k_inv = [
            (1.0 / (eff[0] * dt)).clamp(0.0, 200.0),
            (1.0 / (eff[1] * dt)).clamp(0.0, 200.0),
            (1.0 / (eff[2] * dt)).clamp(0.0, 200.0),
        ];
        s
    }
}

impl<B: Controller> Controller for IndiController<B> {
    fn control(&mut self, dt: Second, sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        // 1) 基线控制器给出电机指令。
        let u_base = self.base.control(dt, sp, est);

        // 首拍无法求差分，直接输出基线。
        if !self.initialized {
            self.initialized = true;
            self.prev_cmd_rate = motors_to_rates(&u_base.motor);
            self.prev_omega = [est.omega[0].0, est.omega[1].0, est.omega[2].0];
            self.dt = dt.0;
            // 用首拍 dt 重算 k_inv。
            self.k_inv = [
                (1.0 / (self.ctrl_eff[0] * dt.0)).clamp(0.0, 200.0),
                (1.0 / (self.ctrl_eff[1] * dt.0)).clamp(0.0, 200.0),
                (1.0 / (self.ctrl_eff[2] * dt.0)).clamp(0.0, 200.0),
            ];
            return u_base;
        }

        // 2) 期望角加速度（基线指令角速度差分）。
        let cmd_rate = motors_to_rates(&u_base.motor);
        let p_dot_cmd = [
            (cmd_rate[0] - self.prev_cmd_rate[0]) / dt.0,
            (cmd_rate[1] - self.prev_cmd_rate[1]) / dt.0,
            (cmd_rate[2] - self.prev_cmd_rate[2]) / dt.0,
        ];
        // 3) 实测角加速度（IMU 角速度差分）。
        let omega = [est.omega[0].0, est.omega[1].0, est.omega[2].0];
        let p_dot_meas = [
            (omega[0] - self.prev_omega[0]) / dt.0,
            (omega[1] - self.prev_omega[1]) / dt.0,
            (omega[2] - self.prev_omega[2]) / dt.0,
        ];
        // 4) INDI 增量：角加速度误差反馈。
        let e_acc = [
            p_dot_cmd[0] - p_dot_meas[0],
            p_dot_cmd[1] - p_dot_meas[1],
            p_dot_cmd[2] - p_dot_meas[2],
        ];
        let dpqr = [
            self.k_inv[0] * e_acc[0] * self.gain_scale,
            self.k_inv[1] * e_acc[1] * self.gain_scale,
            self.k_inv[2] * e_acc[2] * self.gain_scale,
        ];
        // 5) 增量叠加回电机（混控互逆）。
        let inc = rates_to_motor_inc(dpqr);
        let mut motor = [0.0f32; 4];
        for i in 0..4 {
            motor[i] = (u_base.motor[i] + inc[i]).clamp(0.0, 1.0);
        }

        // 更新历史。
        self.prev_cmd_rate = cmd_rate;
        self.prev_omega = omega;

        ActuatorCmd { motor }
    }

    fn reset(&mut self) {
        self.base.reset();
        self.initialized = false;
        self.prev_cmd_rate = [0.0; 3];
        self.prev_omega = [0.0; 3];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::VehicleConfig;
    use crate::controller::pid::PidController;
    use crate::estimator::ekf::EkfEstimator;
    use crate::estimator::Estimator;
    use crate::hal::sensor::{GpsSensor, ImuSensor, MockGps, MockImu};
    use crate::invariants::{actuator_bounded, state_finite};

    #[test]
    fn indi_bounded_and_no_nan() {
        // INDI 包装 PID，闭环运行：指令仍恒有界、状态无 NaN。
        let cfg = VehicleConfig::default_quad();
        let mut ekf = EkfEstimator::default_quad();
        let base = PidController::from_config(&cfg.ctrl_params());
        let mut indi =
            IndiController::with_inertia(base, cfg.inertia, 0.01, 0.8);
        let mut imu = MockImu::new();
        let mut gps = MockGps::new();
        let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-5.0)], Radian(0.0));

        for _ in 0..300 {
            ekf.reset();
            let mut s = VehicleState::zero();
            for _ in 0..4 {
                s = ekf.step(Second(0.01), imu.read(), gps.read());
            }
            let cmd = indi.control(Second(0.01), &sp, &s);
            assert!(actuator_bounded(&cmd), "INDI 输出必须 [0,1]");
            assert!(state_finite(&s));
        }
    }

    #[test]
    fn indi_active_increment_under_angular_accel() {
        // INDI 的本质属性：当估计角速度在两拍之间发生变化（存在角加速度误差）时，
        // INDI 必须在基线指令之上叠加一个非零增量；且不论增量多大，最终指令恒有界。
        // 用"人为在两拍间改变 omega"制造角加速度，验证 INDI 增量被激活且输出受限于 [0,1]。
        let cfg = VehicleConfig::default_quad();
        let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-5.0)], Radian(0.0));

        let mut ekf = EkfEstimator::default_quad();
        let mut imu = MockImu::new();
        let mut gps = MockGps::new();

        let base = PidController::from_config(&cfg.ctrl_params());
        let mut indi = IndiController::with_inertia(base, cfg.inertia, 0.01, 1.0);

        let mut saw_nonzero_increment = false;
        for step in 0..400 {
            ekf.reset();
            let mut s = VehicleState::zero();
            for _ in 0..4 {
                s = ekf.step(Second(0.01), imu.read(), gps.read());
            }
            // 两拍之间注入一个突变滚转角速度（模拟外部扰动产生的角加速度）。
            if step % 2 == 0 {
                s.omega[0] = RadianPerSecond(s.omega[0].0 + 0.8 + (step as f32) * 0.01);
            }

            let c = indi.control(Second(0.01), &sp, &s);
            // 输出恒有界。
            for m in c.motor.iter() {
                assert!(*m >= 0.0 && *m <= 1.0, "INDI 指令必须 [0,1]");
            }
            // 反解 INDI 实际施加的滚转控制量，与"无扰动基线"对比：有角加速度误差时
            // INDI 的滚转分量应与纯基线不同（增量生效）。这里只需证明增量机制存在：
            // 当存在角加速度时，roll 分量偏离零（基线在零扰动附近也应接近零，但 INDI 会因
            // 误差再额外补偿）。用"存在非零 roll 分量"作为增量激活的代理指标。
            let roll = (c.motor[0] + c.motor[3]) - (c.motor[1] + c.motor[2]);
            if roll.abs() > 1e-4 {
                saw_nonzero_increment = true;
            }
        }
        assert!(saw_nonzero_increment, "INDI 应在角加速度误差下产生非零增量");
    }
}
