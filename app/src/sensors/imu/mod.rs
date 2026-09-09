//! 真实 IMU（惯性测量单元）驱动集合。
//!
//! 后续外接不同型号 IMU 时，在此目录新增子模块（如 `icm20948.rs`），
//! 每个型号实现 `flyctrl_core::hal::sensor::ImuSensor` trait，互不影响。

pub mod mpu6050;

pub use mpu6050::ImuMpu6050;
