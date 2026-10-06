//! # fcalg —— 重建的飞控算法核心
//!
//! 与旧 `flyctrl-core` 零耦合；按 `CONTRACT.md` 的阶梯逐层重建，
//! 每层一个模块 + 一个**参考无关**的验收测试（契约 §6）。
//!
//! 当前只到 **L0（单位/帧/四元数契约）+ L1（有限性纪律）**。

pub mod align;
pub mod attitude;
pub mod biquad;
pub mod covariance;
pub mod error_state;
pub mod finite;
pub mod gate;
pub mod imu_delta;
pub mod imu_filter;
pub mod math;
pub mod mixer;
pub mod observe;
pub mod propagate;
pub mod rate;
pub mod quat;
pub mod sim;
pub mod transition;
pub mod units;
pub mod update;

pub use align::{align_static, AlignConfig, AlignError, AlignResult};
pub use attitude::attitude_rate_setpoint;
pub use covariance::{is_positive_definite, is_symmetric_exact, propagate_covariance, Cov, CovError};
pub use error_state::{boxminus, boxplus, I_ATT, I_BA, I_BG, I_MAGB, I_MAGI, I_POS, I_VEL, N};
pub use finite::{gate, gate_all, violations, violations_total, Stage, Violation};
pub use gate::{channel_indices, reflate_diag, Channel, ChannelGuard};
pub use imu_delta::{DeltaBuilder, ImuDelta, Reject};
pub use imu_filter::{ImuFilter, ImuFiltered};
pub use mixer::{x4_mix, MixError, MotorCmd, X4SIGNS};
pub use propagate::{propagate, State};
pub use rate::{rate_p_step, RateGains};
pub use observe::{baro, gps_pos, gps_vel, Obs, ObsParams, NO_INFO};
pub use quat::{specific_force_at_rest, Quat, GRAVITY_NED};
pub use sim::{
    baro_from_truth, gps_pos_from_truth, gps_vel_from_truth, imu_from_truth, mag_yaw_from_truth,
};
pub use transition::{skew, transition_matrix};
pub use units::{Meters, Mps, Radians, Rps, Seconds};
pub use update::{update, UpdateError, UpdateOut};
