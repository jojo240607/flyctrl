//! 仿真驱动 + 指标评估。
//!
//! [`Harness`] 把 物理(`Physics`) + 世界(`World`) + 估计器(`Estimator`)
//! + 控制器(`Controller`) 串成闭环，按固定步长推进，并收集对比指标。
//!
//! 同一套 Harness 可用不同算法组合驱动，输出统一指标表做横向对比。

use flyctrl_core::controller::{Controller, trait_def::Setpoint};
use flyctrl_core::estimator::Estimator;
use flyctrl_core::units::*;
use flyctrl_core::vehicle::{ActuatorCmd, VehicleState};

pub use crate::physics::{Physics, PhysicsParams};
pub use crate::world::{World, WorldParams};

pub struct Harness<E: Estimator, C: Controller> {
    physics: Physics,
    world: World,
    estimator: E,
    controller: C,
    dt: Second,
    t: Second,
    last_cmd: ActuatorCmd,
    metrics: Metrics,
    last_est: VehicleState,
}

/// 对比指标集合。
#[derive(Debug, Clone, Copy, Default)]
pub struct Metrics {
    pub steps: u32,
    pub pos_rms: f32,        // 位置误差 RMS (m)
    pub pos_max: f32,        // 最大位置误差 (m)
    pub settle_time: f32,    // 首次进入 0.5m 容差的时间 (s)，未达成为 -1
    pub nan_detected: bool,  // 数值发散检测
    pub worst_step_ms: f32,  // 单步控制+估计最坏耗时 (ms) —— 为 RTOS CPU 预算预估
}

impl<E: Estimator, C: Controller> Harness<E, C> {
    pub fn new(physics: Physics, world: World, estimator: E, controller: C, dt: Second) -> Self {
        Self {
            physics, world, estimator, controller,
            dt, t: Second::ZERO,
            last_cmd: ActuatorCmd::zero(),
            metrics: Metrics::default(),
            last_est: VehicleState::zero(),
        }
    }

    /// 运行 `duration` 秒，目标 setpoint 固定。
    pub fn run(&mut self, duration: Second, setpoint: &Setpoint) -> Metrics {
        let n = (duration.0 / self.dt.0).round() as u32;
        let mut err_sq_sum = 0.0f32;
        let mut first_settle: f32 = -1.0;
        let tol = 0.5f32;
        let mut worst = 0.0f32;

        for i in 0..n {
            // 1) 物理推进，得到理想 IMU + 真实状态
            let ideal = self.physics.step(self.dt, self.last_cmd);
            let true_pos = self.physics.state().pos;
            // 2) 世界加噪（位置测量基于真实位置）
            let (imu, pos) = self.world.sense(self.dt, ideal, true_pos);
            // 3) 估计
            let est = self.estimator.step(self.dt, imu, pos);
            self.last_est = est;
            // 4) 控制（计时最坏耗时）
            let t0 = now_ms();
            let cmd = self.controller.control(self.dt, setpoint, &est);
            let el = now_ms() - t0;
            if el > worst { worst = el; }
            self.last_cmd = cmd;

            // 5) 指标
            let err = pos_error(&est, setpoint);
            err_sq_sum += err * err;
            if err > self.metrics.pos_max { self.metrics.pos_max = err; }
            if first_settle < 0.0 && err < tol && i as f32 * self.dt.0 > 0.2 {
                first_settle = i as f32 * self.dt.0;
            }
            if !est.pos[0].0.is_finite() || !est.att.w.is_finite() {
                self.metrics.nan_detected = true;
                break;
            }
            self.t = Second(self.t.0 + self.dt.0);
        }

        self.metrics.steps = n;
        self.metrics.pos_rms = (err_sq_sum / n as f32).sqrt();
        self.metrics.settle_time = first_settle;
        self.metrics.worst_step_ms = worst;
        self.metrics
    }

    pub fn reset(&mut self) {
        self.physics.reset();
        self.world.reset();
        self.estimator.reset();
        self.controller.reset();
        self.t = Second::ZERO;
        self.last_cmd = ActuatorCmd::zero();
        self.metrics = Metrics::default();
    }

    pub fn state(&self) -> VehicleState { self.physics.state() }

    /// 暴露最近一次估计状态（调试用）。
    pub fn last_est(&self) -> VehicleState { self.last_est }

    /// 暴露最近一次控制输出（调试用）。
    pub fn last_cmd(&self) -> ActuatorCmd { self.last_cmd }
}

/// 位置误差幅值 (m)。
fn pos_error(est: &VehicleState, sp: &Setpoint) -> f32 {
    let dx = est.pos[0].0 - sp.pos[0].0;
    let dy = est.pos[1].0 - sp.pos[1].0;
    let dz = est.pos[2].0 - sp.pos[2].0;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// 仿真用耗时测量（host 端 std::time）。嵌入式侧可替换为 DWT 周期计数。
#[cfg(feature = "std-time")]
fn now_ms() -> f32 {
    use std::time::Instant;
    static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
    let s = START.get_or_init(Instant::now);
    s.elapsed().as_secs_f32() * 1000.0
}

#[cfg(not(feature = "std-time"))]
fn now_ms() -> f32 { 0.0 }
