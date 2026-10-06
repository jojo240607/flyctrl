//! # fcalg —— 重建的飞控算法核心
//!
//! 与旧 `flyctrl-core` 零耦合；按 `CONTRACT.md` 的阶梯逐层重建，
//! 每层一个模块 + 一个**参考无关**的验收测试（契约 §6）。
//!
//! 当前只到 **L0（单位/帧/四元数契约）+ L1（有限性纪律）**。

pub mod biquad;
pub mod finite;
pub mod imu_filter;
pub mod math;
pub mod quat;
pub mod units;

pub use finite::{gate, gate_all, violations, violations_total, Stage, Violation};
pub use imu_filter::{ImuFilter, ImuFiltered};
pub use quat::{specific_force_at_rest, Quat, GRAVITY_NED};
pub use units::{Meters, Mps, Radians, Rps, Seconds};
