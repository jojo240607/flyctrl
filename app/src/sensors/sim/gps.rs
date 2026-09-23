//! 虚拟 GPS 驱动：从全局 `PLAYBACK` 读取回放数据集，伪装成真实 GPS（NED 局部坐标）。
//!
//! 与 `gps::ublox::GpsUblox` 实现同一 `GpsSensor` trait，使 `sensors_task`
//! 只需切换数据源即可，无需改动采集逻辑。

use flyctrl_core::hal::sensor::GpsSensor;
use flyctrl_core::units::Meter;
use flyctrl_core::vehicle::PosSample;

use crate::sensors::sim::dataset::{
    Frame, GPS_ORIGIN, METERS_PER_DEG_LAT, METERS_PER_DEG_LON, PLAYBACK,
};

/// ★GPS 高度基准（懒初始化 = 首个帧的高度 ✓）—— 使垂直与气压/发射点同基准 ✓
static mut ALT_ORIGIN: f32 = 0.0;

pub struct VirtualGps;

impl VirtualGps {
    pub fn new() -> Option<Self> {
        Some(Self)
    }
}

impl GpsSensor for VirtualGps {
    fn read(&mut self) -> Option<PosSample> {
        let f: Frame = unsafe { PLAYBACK.current() };
        // 数据集 gps = [lat, lon, alt(m)]；PosSample.pos 为 NED [x,y,z]（z 向下为正）。
        let lat = f.gps[0];
        let lon = f.gps[1];
        let alt = f.gps[2];
        // 相对起点的局部 NED（米）：经纬度差 × 米/度近似。
        let north = (lat - GPS_ORIGIN.0) * METERS_PER_DEG_LAT;
        let east = (lon - GPS_ORIGIN.1) * METERS_PER_DEG_LON;
        // ★垂直也取【相对起点】✓（2026-09-23，§5.46 ✓）—— 与上面两行的水平处理自洽 ✓
        //   原实现用【绝对】`-alt` ✗ ⇒ 与气压的 `update_alt(alt − baro_ref)`（相对 ✓）
        //   相差约 10.5m ⇒ 滤波在垂直方向被对拉 ⇒ 高度偏差 4.665m ✗（实测 ✓）
        //   基准取【首个 GPS 帧的高度】✓（懒初始化 ⇒ 不依赖 .data 初值 ✓）
        unsafe {
            if ALT_ORIGIN == 0.0 {
                ALT_ORIGIN = alt;
            }
        }
        let z_rel = unsafe { ALT_ORIGIN } - alt; // NED：向上为负，且发射点处 ≈ 0 ✓
        Some(PosSample::pos_only([
            Meter(north),
            Meter(east),
            Meter(z_rel),
        ]))
    }

    fn healthy(&self) -> bool {
        true
    }
}
