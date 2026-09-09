//! 真实磁力计（指南针）驱动集合。
//!
//! 后续外接不同型号磁力计时，在此目录新增子模块（如 `ak8963.rs`），
//! 每个型号实现 `flyctrl_core::hal::sensor::MagSensor` trait，互不影响。

pub mod qmc5883;

pub use qmc5883::MagQmc5883;
