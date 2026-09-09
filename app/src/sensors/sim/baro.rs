//! 虚拟气压计驱动：从全局 `PLAYBACK` 读取回放数据集，伪装成真实气压高度计。
//!
//! 与 `baro::bmp280::BaroBmp280` 实现同一 `BaroSensor` trait，使 `sensors_task`
//! 只需切换数据源即可，无需改动采集逻辑。

use flyctrl_core::hal::sensor::BaroSensor;
use flyctrl_core::units::Meter;

use crate::sensors::sim::dataset::{Frame, PLAYBACK};

pub struct VirtualBaro;

impl VirtualBaro {
    pub fn new() -> Option<Self> {
        Some(Self)
    }
}

impl BaroSensor for VirtualBaro {
    fn read_altitude(&mut self) -> Meter {
        let f: Frame = unsafe { PLAYBACK.current() };
        Meter(f.baro_alt)
    }

    fn healthy(&self) -> bool {
        true
    }
}
