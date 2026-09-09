//! 虚拟磁力计驱动：当前数据集不含磁力计通道，返回零向量（机体坐标系）。
//!
//! 仅用于保持 `MagSensor` 接口完整，使 `SensorStack` 在虚拟模式下也能持有 mag 源；
//! 真实磁力计接入后由 `mag::qmc5883::MagQmc5883` 替换，调用方（`control` 的 `use_mag`）决定是否启用。

use flyctrl_core::hal::sensor::MagSensor;

pub struct VirtualMag;

impl VirtualMag {
    pub fn new() -> Option<Self> {
        Some(Self)
    }
}

impl MagSensor for VirtualMag {
    fn read(&mut self) -> [f32; 3] {
        [0.0; 3]
    }

    fn healthy(&self) -> bool {
        true
    }
}
