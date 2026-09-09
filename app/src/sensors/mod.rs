//! Rust 传感器驱动层：用 RTOS 总线（i2c0/spi0/uart1/uart2）组合具体传感器。
//!
//! 分层：
//! - `sim/`  模拟（回放）传感器集合：从全局 `PLAYBACK` 数据集读取，无需外接硬件即可调试。
//!   回放数据集（`dataset.rs` `dataset_data.rs`）也放在此目录，仅模拟驱动使用。
//! - `imu/` `baro/` `mag/` `gps/` `rc/`  真实硬件驱动，按物理类型分目录，
//!   每类下每种型号一个子模块（如 `imu/mpu6050.rs`、`gps/ublox.rs`），
//!   后续接不同型号传感器时新增子模块即可，互不影响。
//! - `stack.rs`  统一封装：虚拟/真实编译期切换（`cfg(feature = "real-sensors")`），
//!   算法/控制/遥测层只依赖 trait，不感知底层来源。

pub mod baro;
pub mod gps;
pub mod imu;
pub mod mag;
pub mod rc;
pub mod sim;
pub mod stack;

pub use baro::BaroBmp280;
pub use gps::GpsUblox;
pub use imu::ImuMpu6050;
pub use mag::MagQmc5883;
pub use rc::RcSbus;
pub use sim::SimImu;
