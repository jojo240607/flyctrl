//! 真实 GPS（全球定位系统）驱动集合。
//!
//! 后续外接不同型号 GPS 模块时，在此目录新增子模块（如 `m8n.rs`），
//! 每个型号实现 `flyctrl_core::hal::sensor::GpsSensor` trait，互不影响。

pub mod ublox;

pub use ublox::GpsUblox;
