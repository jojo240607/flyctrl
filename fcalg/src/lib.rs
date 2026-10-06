//! # fcalg —— 重建的飞控算法核心
//!
//! 与旧 `flyctrl-core` 零耦合；按 `CONTRACT.md` 的阶梯逐层重建，
//! 每层一个模块 + 一个**参考无关**的验收测试（契约 §6）。
//!
//! 当前只到 **L0（单位/帧/四元数契约）+ L1（有限性纪律）**。

pub mod align;
pub mod biquad;
pub mod covariance;
pub mod error_state;
pub mod finite;
pub mod imu_delta;
pub mod imu_filter;
pub mod math;
pub mod propagate;
pub mod quat;
pub mod transition;
pub mod units;
pub mod update;

pub use align::{align_static, AlignConfig, AlignError, AlignResult};
pub use covariance::{is_positive_definite, is_symmetric_exact, propagate_covariance, Cov, CovError};
pub use error_state::{boxminus, boxplus, I_ATT, I_BA, I_BG, I_MAGB, I_MAGI, I_POS, I_VEL, N};
pub use finite::{gate, gate_all, violations, violations_total, Stage, Violation};
pub use imu_delta::{DeltaBuilder, ImuDelta, Reject};
pub use imu_filter::{ImuFilter, ImuFiltered};
pub use propagate::{propagate, State};
pub use quat::{specific_force_at_rest, Quat, GRAVITY_NED};
pub use transition::{skew, transition_matrix};
pub use units::{Meters, Mps, Radians, Rps, Seconds};
pub use update::{update, UpdateError, UpdateOut};
