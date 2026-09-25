//! BMI088（SPI 双片选：accel + gyro 两芯片）加速度 + 陀螺仪。
//!
//! 经 RTOS `bmi088` 设备（joc-base drv/bmi088.c，SPI2 + bmi_accel_cs/bmi_gyro_cs）
//! 访问；缺失/无响应时构造返回 `None` 由上层降级。
//!
//! 换算在驱动侧完成（`BMI088_IOCTL_GET_SI` 直接返回 SI：accel m/s²、gyro rad/s，
//! 与固件换算常量一致：accel ±3g=10920 LSB/g、gyro ±2000dps=16.4 LSB/dps）。

use core::ffi::c_void;

use flyctrl_core::hal::sensor::ImuSensor;
use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::ImuSample;

use rtos_app_sdk::device::Device;
use rtos_app_sdk::ioctl::{self, Bmi088Si, Bmi088Who};

pub struct ImuBmi088 {
    bus: Device,
    healthy: bool,
}

impl ImuBmi088 {
    pub fn new(bus_name: &str) -> Option<Self> {
        let bus = Device::open(bus_name)?;
        let mut s = Self { bus, healthy: true };
        // 探测：回读双芯片 WHO_AM_I（ACCEL=0x1E、GYRO=0x0F）。无响应（断线/
        // 未上电）时降级创建（healthy=false），后续 read 返回零值 → FDIR 降级。
        let mut who = Bmi088Who { accel: 0, gyro: 0 };
        let rc = s.bus.ioctl(
            ioctl::BMI088_IOCTL_GET_WHO,
            &mut who as *mut Bmi088Who as *mut c_void,
        );
        if rc != 0 || (who.accel != 0x1E && who.gyro != 0x0F) {
            s.healthy = false;
        }
        Some(s)
    }

    /// 读 6 轴 SI（驱动换算：accel m/s²、gyro rad/s）。
    pub fn read_raw(&self) -> Option<ImuSample> {
        let mut si = Bmi088Si {
            accel: [0.0; 3],
            gyro: [0.0; 3],
        };
        let rc = self.bus.ioctl(
            ioctl::BMI088_IOCTL_GET_SI,
            &mut si as *mut Bmi088Si as *mut c_void,
        );
        if rc != 0 {
            return None;
        }
        if si.accel.iter().any(|v| !v.is_finite()) || si.gyro.iter().any(|v| !v.is_finite()) {
            return None;
        }
        // ★§5.136 输入合理性校验（驱动层，仅真传感器路径）：传输链偶发投递垃圾样本
        //   实测（逐拍探针）：陀螺单点 −57.6 / −3541 / **20165 rad/s**、加计异常——
        //   它们会污染【控制器速率阻尼项】att_kd·ω ⇒ 混控瞬时饱和（推力骤减+力矩尖峰）
        //   ⇒ 位置环被反复踢 ⇒ LOITER 增长型振荡（而 ESKF 自身门控使估计仍准）。
        //   判据：陀螺单轴 ≤ 35 rad/s（BMI088 2000 dps 物理量程 ✓）；比力模长
        //   1..200 m/s²（悬停 ≈g=9.8，自由落体 ≈0 允许到 1；>200 为非物理）。
        //   不合格 ⇒ 返回 None ⇒ `step_hil` 采样保持上一有效帧 ✓（不污染滤波/控制）
        let gn22 = si.gyro[0] * si.gyro[0] + si.gyro[1] * si.gyro[1] + si.gyro[2] * si.gyro[2];
        if gn22 > 35.0 * 35.0 {
            return None;
        }
        let an2 = si.accel[0] * si.accel[0] + si.accel[1] * si.accel[1] + si.accel[2] * si.accel[2];
        if !(1.0..=(200.0 * 200.0)).contains(&an2) {
            return None;
        }
        Some(ImuSample {
            accel: [
                MeterPerSecondSquared(si.accel[0]),
                MeterPerSecondSquared(si.accel[1]),
                MeterPerSecondSquared(si.accel[2]),
            ],
            gyro: [
                RadianPerSecond(si.gyro[0]),
                RadianPerSecond(si.gyro[1]),
                RadianPerSecond(si.gyro[2]),
            ],
        })
    }
}

impl ImuSensor for ImuBmi088 {
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
