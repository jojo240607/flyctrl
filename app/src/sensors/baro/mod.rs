//! 真实气压计（高度计）驱动集合。
//!
//! 后续外接不同型号气压计时，在此目录新增子模块（如 `bmp388.rs`），
//! 每个型号实现 `flyctrl_core::hal::sensor::BaroSensor` trait，互不影响。

pub mod bmp280;

pub use bmp280::BaroBmp280;
