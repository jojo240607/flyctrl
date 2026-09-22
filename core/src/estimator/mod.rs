//! 状态估计模块。
//!
//! 所有估计算法实现 [`Estimator`] trait，仿真/飞控核心只依赖 trait，
//! 从而可在互补滤波、EKF 等实现间自由替换并做横向对比。

pub mod trait_def;
pub mod complementary;
pub mod ekf;
pub mod learning;
pub mod eskf_estimator;
pub mod select; // ★估计器选择层（迁移计划步 3 ✓，默认 ESKF ✓） // ★ESKF 的 Estimator 适配器（迁移计划步 1 ✓）
pub mod eskf; // ★默认估计方案：误差状态 EKF（C1+C2；见 docs/c1-design.md / c2-design.md）

pub use trait_def::Estimator;
pub use complementary::ComplementaryEstimator;
pub use ekf::EkfEstimator;
pub use learning::{LearningEstimator, NullResidual, ResidualModel, BiasResidual};
