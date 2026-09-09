//! 虚拟传感器回放数据集。
//!
//! 本模块定义回放所需的帧格式、采样率常量，以及一个全局回放状态机 `Playback`。
//! 真实数据（或真实形态的合成片段）放在 `dataset_data.rs`，由 `tools/gen_dataset.py`
//! 生成（默认生成一段真实形态机动片段；若提供 ArduPilot `.bin` / PX4 `.ulg` / CSV
//! 真实日志，则抽取四类通道生成同格式数组替换之）。板上固件通过 `include!` 把数组
//! 编译进 Flash 只读段，循环回放无限长。
//!
//! 帧字段契约（与各虚拟驱动一一对应）：
//! - `imu_accel` [m/s^2]：机体 x/y/z 加速度（含重力 ~9.81）
//! - `imu_gyro`  [rad/s]：机体 x/y/z 角速度
//! - `baro_alt`  [m]：气压计相对高度
//! - `gps`       [lat(deg), lon(deg), alt(m)]：GPS 经纬高
//! - `rc`        [throttle, roll, pitch, yaw, aux1]，量纲与 RcInput 一致（-1..1 / 0..1）

#![allow(dead_code)]

/// 单帧回放数据。`#[repr(C)]` 保证布局稳定，便于 `dataset_data.rs` 字面量对齐。
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Frame {
    pub imu_accel: [f32; 3],
    pub imu_gyro: [f32; 3],
    pub baro_alt: f32,
    pub gps: [f32; 3],
    pub rc: [f32; 5],
}

/// 回放采样率（Hz）。控制律按自身周期读取，每帧被读取 `控制频率/采样率` 次。
pub const DATASET_HZ: u32 = 50;

/// GPS 原点（度），与 `tools/gen_dataset.py` 中的 LAT0/LON0 保持一致，
/// 用于把数据集里的经纬度差换算成 NED 水平分量（米）。
pub const GPS_ORIGIN: (f32, f32) = (37.4275, -122.1697);
pub const METERS_PER_DEG_LAT: f32 = 111320.0;
pub const METERS_PER_DEG_LON: f32 = 111320.0 * 0.792; // cos(37.4275°)

/// 回放帧数组：由 `tools/gen_dataset.py` 生成到 `dataset_data.rs`。
/// 若该文件不存在，请用 `python tools/gen_dataset.py` 生成（默认内置真实形态片段）。
include!("dataset_data.rs");

/// 全局回放状态机：所有虚拟驱动共享同一个读指针，保证四类数据在时间上同步。
pub struct Playback {
    idx: usize,
    /// 累积时间（秒），用于按 dt 推进索引。
    acc: f32,
}

impl Playback {
    pub const fn new() -> Self {
        Playback { idx: 0, acc: 0.0 }
    }

    /// 推进 `dt` 秒，返回当前应回放的帧索引（循环）。
    pub fn advance(&mut self, dt: f32) -> usize {
        let step = 1.0 / (DATASET_HZ as f32);
        self.acc += dt;
        while self.acc >= step {
            self.acc -= step;
            self.idx = (self.idx + 1) % DATASET_FRAMES.len();
        }
        self.idx
    }

    /// 读取当前帧（不推进）。
    pub fn current(&self) -> Frame {
        DATASET_FRAMES[self.idx]
    }

    pub fn len(&self) -> usize {
        DATASET_FRAMES.len()
    }

    pub fn is_empty(&self) -> bool {
        DATASET_FRAMES.is_empty()
    }
}

/// 全局唯一回放实例（静态单例，置于 .rust_data，运行时由 `spawn_flyctrl` 之前构造）。
#[link_section = ".rust_data"]
pub static mut PLAYBACK: Playback = Playback::new();
