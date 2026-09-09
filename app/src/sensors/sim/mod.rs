//! 虚拟（模拟）传感器驱动集合：从全局回放数据集 `PLAYBACK` 读取真实形态数据，
//! 伪装成真实硬件传感器，使飞控在无需外接设备时也能闭环调试。
//!
//! - `dataset.rs` `dataset_data.rs`  回放数据集（仅模拟驱动使用的支撑数据）
//! - 每个类一个文件（`imu.rs` / `baro.rs` / `gps.rs` / `rc.rs` / `mag.rs`），
//!   与 `../imu` `../baro` `../gps` `../rc` `../mag` 下的真实驱动一一对应，
//!   实现同一组 `flyctrl_core::hal::sensor` trait。

pub mod baro;
pub mod dataset;
pub mod gps;
pub mod imu;
pub mod mag;
pub mod rc;
pub mod sim_imu;

pub use baro::VirtualBaro;
pub use dataset::Playback;
pub use gps::VirtualGps;
pub use imu::VirtualImu;
pub use mag::VirtualMag;
pub use rc::VirtualRc;
pub use sim_imu::SimImu;
