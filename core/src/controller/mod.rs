//! 控制律模块。
//!
//! 所有控制算法实现 [`Controller`] trait：给定期望状态 + 估计状态，输出执行器指令。
//! 通过 trait 抽象，PID / LQR / MPC 等实现可插拔，仿真后端统一驱动做对比。

pub mod trait_def;
pub mod pid;
pub mod lqr;
pub mod mpc;
pub mod indi;

pub use trait_def::{Controller, Setpoint};
pub use pid::PidController;
pub use lqr::LqrController;
pub use mpc::MpcController;
pub use indi::IndiController;
