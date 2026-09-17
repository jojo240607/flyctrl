//! BMP280（I2C 从机 0x76）：气压/温度 → 相对高度（向上为正，与 `hil::step_hil`
//! 气压观测约定一致）。
//!
//! 经 RTOS `i2c0` 总线（I2C1）组合；缺失/无响应时构造返回 `None` 由上层降级。

use flyctrl_core::hal::sensor::BaroSensor;
use flyctrl_core::units::Meter;

use rtos_app_sdk::device::{Device, i2c_write_read};

pub struct BaroBmp280 {
    bus: Device,
    addr: u16,
    healthy: bool,
}

impl BaroBmp280 {
    const PRESS_MSB: u8 = 0xF7;

    pub fn new(bus_name: &str, addr: u16) -> Option<Self> {
        let bus = Device::open(bus_name)?;
        // 简化：假设传感器已配置为正常模式（CTRL_MEAS 由 board/初始化完成）。
        // [I2C DMA] 总线切换 STREAM_MODE_DMA：驱动层事务经 DMA 引擎搬运（固件
        // SET_MODE + 模拟器 I2cDma 事件 → DMA1 流）；失败则保持 POLL（不阻塞启动）。
        let mode = rtos_app_sdk::ioctl::STREAM_MODE_DMA;
        let _ = bus.ioctl(
            rtos_app_sdk::ioctl::STREAM_IOCTL_SET_MODE,
            &mode as *const u32 as *mut core::ffi::c_void,
        );
        Some(Self { bus, addr, healthy: true })
    }

    /// 读 6 字节原始压力/温度，粗略转高度（占位线性近似，真实需校准系数）。
    pub fn read_altitude_raw(&self) -> Option<Meter> {
        let mut raw = [0u8; 6];
        if i2c_write_read(&self.bus, self.addr, Self::PRESS_MSB, &mut raw) != 0 {
            return None;
        }
        let p = (((raw[0] as u32) << 16) | ((raw[1] as u32) << 8) | (raw[2] as u32)) >> 4;
        let p_pa = p as f32; // 占位：未做校准
        // 气压→高度（ISA 近似，海平面 101325 Pa）
        let h = 44330.0 * (1.0 - libm::powf(p_pa / 101325.0, 0.1903));
        Some(Meter(h)) // 向上为正：step_hil 经 update_alt 内部取负转 D 向下状态
    }
}

impl BaroSensor for BaroBmp280 {
    fn read_altitude(&mut self) -> Meter {
        match self.read_altitude_raw() {
            Some(h) => h,
            None => {
                self.healthy = false;
                Meter(0.0)
            }
        }
    }

    fn healthy(&self) -> bool {
        self.healthy
    }
}
