//! QMC5883L（I2C 从机 0x0D）：三轴磁力计。
//!
//! 经 RTOS `i2c0` 总线（I2C1）组合；缺失/无响应时构造返回 `None` 由上层降级。

use flyctrl_core::hal::sensor::MagSensor;

use rtos_app_sdk::device::{Device, i2c_write_read};

pub struct MagQmc5883 {
    bus: Device,
    addr: u16,
    healthy: bool,
}

impl MagQmc5883 {
    const DATA_X_L: u8 = 0x00;
    /// ★§5.132：raw 计数 → Gauss 的换算（±2G 量程，与虚拟外设模型同约定：
    /// raw = G×32768/2 ⇒ 16384 LSB/G）。实芯片 ±2G 标称 12000 LSB/G——对方向型
    /// 融合无影响（只用归一化方向）。检查驱动时**必须确认物理单位**再返回。
    const LSB_PER_GAUSS: f32 = 16384.0;

    pub fn new(bus_name: &str, addr: u16) -> Option<Self> {
        Device::open(bus_name).map(|bus| {
            // [I2C DMA] 总线切换 STREAM_MODE_DMA（失败保持 POLL，不阻塞启动）。
            let mode = rtos_app_sdk::ioctl::STREAM_MODE_DMA;
            let _ = bus.ioctl(
                rtos_app_sdk::ioctl::STREAM_IOCTL_SET_MODE,
                &mode as *const u32 as *mut core::ffi::c_void,
            );
            Self { bus, addr, healthy: true }
        })
    }

    pub fn read_raw(&self) -> Option<[f32; 3]> {
        let mut raw = [0u8; 6];
        if i2c_write_read(&self.bus, self.addr, Self::DATA_X_L, &mut raw) != 0 {
            return None;
        }
        let v = |o: usize| i16::from_le_bytes([raw[o], raw[o + 1]]) as f32;
        Some([v(0), v(2), v(4)])
    }
}

impl MagSensor for MagQmc5883 {
    fn read(&mut self) -> [f32; 3] {
        match self.read_raw() {
            // ★§5.132 修复：raw 计数 → 物理单位（Gauss）。
            //   此前 `read()` 直接把原始 i16 当物理值返回 ✗（遗漏换算）⇒ ESKF 收到
            //   16384 倍大的磁矢量 ⇒ 绝大样本被 NIS 门拒；少数落到小数值的样本
            //   通过门控后以【错误方向】污染姿态 ⇒ M 场长悬停偶发 12°/43° 单拍
            //   姿态跳变 + 真机翻滚（x_hover_demo 根因，逐拍探针实证 mag_m=8191/6553）。
            //   量程 ±2G（照虚拟外设 qmc5883 模型约定：raw = G×32768/2 ⇒ LSB/G=16384；
            //   实测 0.2G → 3276 ✓）。真芯片 ±2G 标称 12000 LSB/G——增益差 1.37×
            //   对【方向型】磁融合无影响（ESKF 只用归一化方向 + 在线估 mag_I ✓）。
            Some(m) => [m[0] / Self::LSB_PER_GAUSS, m[1] / Self::LSB_PER_GAUSS, m[2] / Self::LSB_PER_GAUSS],
            None => {
                self.healthy = false;
                [0.0; 3]
            }
        }
    }

    fn healthy(&self) -> bool {
        self.healthy
    }
}
