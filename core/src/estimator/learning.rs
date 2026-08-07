//! 学习增强估计（M8.3 接口骨架）。
//!
//! 目标：在标准估计算法（EKF/互补）之上叠加一个**残差补偿模型**，用数据驱动
//! 方式修正系统性的估计偏差（如传感器温漂、气动扰动引起的位置/速度残差）。
//!
//! 本里程碑交付**类型安全的接口骨架 + 基线实现**，可被后续训练好的模型直接接入：
//! - [`ResidualModel`]：残差补偿模型接口（输入：当前估计 + 原始观测，输出：对
//!   位置/速度的修正量）。`no_std`、有界、确定。
//! - [`NullResidual`]：零补偿基线（什么都不改），保证接口随时可编译、可跑。
//! - [`LearningEstimator<E>`]：包装任意 [`Estimator`]，把其输出经 `ResidualModel`
//!   修正后返回。
//!
//! 后续可插入真实模型（查表 / 线性加权 / 小网络），只要实现 `ResidualModel` 即可
//! 零改动接入整个飞控闭环（与 M7 HIL 同一份代码路径）。

use crate::estimator::Estimator;
use crate::units::*;
use crate::vehicle::{ImuSample, PosSample, VehicleState};

/// 残差补偿模型接口。
///
/// `estimate` 为基底估计器的当前输出，`imu`/`gps` 为当拍原始观测。
/// 返回对 (pos, vel) 的修正量（NED 米 / 米每秒）。
pub trait ResidualModel {
    /// 计算残差修正；默认零补偿（基类行为）。
    fn correct(&self, estimate: &VehicleState, _imu: &ImuSample, _gps: &Option<PosSample>) -> ([f32; 3], [f32; 3]) {
        ([0.0; 3], [0.0; 3])
    }

    /// 在线更新钩子（预留：真实模型可据新样本增量学习）。基类无操作。
    fn adapt(&mut self, _estimate: &VehicleState, _imu: &ImuSample, _gps: &Option<PosSample>) {}
}

/// 零补偿基线：不改变任何估计（接口默认实现一致）。
#[derive(Debug, Clone, Copy, Default)]
pub struct NullResidual;

impl ResidualModel for NullResidual {}

/// 恒定偏置残差（测试/标定用，或作为"已学习好的查表模型"最简形态）。
#[derive(Debug, Clone, Copy, Default)]
pub struct BiasResidual {
    pub pos_bias: [f32; 3],
    pub vel_bias: [f32; 3],
}

impl ResidualModel for BiasResidual {
    fn correct(&self, _estimate: &VehicleState, _imu: &ImuSample, _gps: &Option<PosSample>) -> ([f32; 3], [f32; 3]) {
        (self.pos_bias, self.vel_bias)
    }
}

/// 学习增强估计包装器：基底估计器输出 + 残差模型修正。
pub struct LearningEstimator<E: Estimator, R: ResidualModel> {
    base: E,
    residual: R,
}

impl<E: Estimator, R: ResidualModel> LearningEstimator<E, R> {
    pub fn new(base: E, residual: R) -> Self {
        Self { base, residual }
    }
}

impl<E: Estimator, R: ResidualModel> Estimator for LearningEstimator<E, R> {
    fn step(&mut self, dt: Second, imu: ImuSample, gps: Option<PosSample>) -> VehicleState {
        let mut st = self.base.step(dt, imu, gps);
        let (dp, dv) = self.residual.correct(&st, &imu, &gps);
        st.pos[0] = Meter(st.pos[0].0 + dp[0]);
        st.pos[1] = Meter(st.pos[1].0 + dp[1]);
        st.pos[2] = Meter(st.pos[2].0 + dp[2]);
        st.vel[0] = MeterPerSecond(st.vel[0].0 + dv[0]);
        st.vel[1] = MeterPerSecond(st.vel[1].0 + dv[1]);
        st.vel[2] = MeterPerSecond(st.vel[2].0 + dv[2]);
        self.residual.adapt(&st, &imu, &gps);
        st
    }

    fn reset(&mut self) {
        self.base.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::ekf::EkfEstimator;

    #[test]
    fn null_residual_passthrough() {
        // 零补偿应完全不改基底估计。
        let mut ekf = EkfEstimator::default_quad();
        let mut le = LearningEstimator::new(EkfEstimator::default_quad(), NullResidual);
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(-9.81)],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let gps = Some(PosSample { pos: [Meter(1.0), Meter(2.0), Meter(-5.0)] });
        let a = ekf.step(Second(0.01), imu, gps);
        let b = le.step(Second(0.01), imu, gps);
        assert_eq!(a.pos[0].0, b.pos[0].0);
        assert_eq!(a.vel[1].0, b.vel[1].0);
    }

    #[test]
    fn bias_residual_corrects() {
        // 已知偏置应被精确加回。
        let bias = BiasResidual { pos_bias: [0.1, -0.2, 0.3], vel_bias: [0.0; 3] };
        let mut le = LearningEstimator::new(EkfEstimator::default_quad(), bias);
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(-9.81)],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let gps = Some(PosSample { pos: [Meter(1.0), Meter(2.0), Meter(-5.0)] });
        let st = le.step(Second(0.01), imu, gps);
        // 基底估计应约等于 gps 位置，加偏置后应偏离 0.1/-0.2/0.3。
        assert!((st.pos[0].0 - 1.1).abs() < 0.5, "pos[0] 应含 +0.1 偏置修正，得到 {}", st.pos[0].0);
    }
}
