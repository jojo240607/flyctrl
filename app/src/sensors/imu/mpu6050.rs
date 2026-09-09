//! MPU6050（I2C 从机 0x68）：加速度 + 陀螺仪。
//!
//! 经 RTOS `i2c0` 总线（I2C1）组合；缺失/无响应时构造返回 `None` 由上层降级。

use core::ffi::c_void;

use flyctrl_core::hal::sensor::ImuSensor;
use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::ImuSample;

use rtos_app_sdk::device::{Device, I2cXfer, i2c_write_read};
use rtos_app_sdk::ioctl;

pub struct ImuMpu6050 {
    bus: Device,
    addr: u16,
    healthy: bool,
}

impl ImuMpu6050 {
    const ACCEL_XOUT_H: u8 = 0x3B;
    const PWR_MGMT_1: u8 = 0x6B;

    pub fn new(bus_name: &str, addr: u16) -> Option<Self> {
        let bus = Device::open(bus_name)?;
        let s = Self { bus, addr, healthy: true };
        // 唤醒（清零 PWR_MGMT_1 的 SLEEP 位）
        let mut tx = [Self::PWR_MGMT_1, 0x00];
        let mut w = I2cXfer { addr, buf: tx.as_mut_ptr(), len: 2, result: 0 };
        if s.bus.ioctl(ioctl::I2C_IOCTL_MASTER_WRITE, &mut w as *mut I2cXfer as *mut c_void) != 0
            || w.result != 0
        {
            return None;
        }
        Some(s)
    }

    /// 读 6 轴（14 字节：accel3×2 + 温度2 + gyro3×2），解析为 `ImuSample`。
    pub fn read_raw(&self) -> Option<ImuSample> {
        let mut raw = [0u8; 14];
        if i2c_write_read(&self.bus, self.addr, Self::ACCEL_XOUT_H, &mut raw) != 0 {
            return None;
        }
        let g = |o: usize| i16::from_be_bytes([raw[o], raw[o + 1]]) as f32;
        // 量程：accel ±2g (LSB/16384)，gyro ±250°/s (LSB/131)；°/s → rad/s。
        Some(ImuSample {
            accel: [
                MeterPerSecondSquared(g(0) / 16384.0 * 9.81),
                MeterPerSecondSquared(g(2) / 16384.0 * 9.81),
                MeterPerSecondSquared(g(4) / 16384.0 * 9.81),
            ],
            gyro: [
                RadianPerSecond(g(8) / 131.0 * core::f32::consts::PI / 180.0),
                RadianPerSecond(g(10) / 131.0 * core::f32::consts::PI / 180.0),
                RadianPerSecond(g(12) / 131.0 * core::f32::consts::PI / 180.0),
            ],
        })
    }
}

impl ImuSensor for ImuMpu6050 {
    fn read(&mut self) -> ImuSample {
        match self.read_raw() {
            Some(s) => s,
            None => {
                self.healthy = false;
                ImuSample {
                    accel: [MeterPerSecondSquared(0.0); 3],
                    gyro: [RadianPerSecond(0.0); 3],
                }
            }
        }
    }

    fn healthy(&self) -> bool {
        self.healthy
    }
}
