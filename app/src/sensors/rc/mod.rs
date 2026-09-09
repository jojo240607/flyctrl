//! 真实遥控接收机（RC）驱动集合。
//!
//! 后续外接不同协议遥控接收机时，在此目录新增子模块（如 `ppm.rs`），
//! 每个型号实现 `flyctrl_core::hal::sensor::RcReceiver` trait，互不影响。

pub mod sbus;

pub use sbus::RcSbus;
