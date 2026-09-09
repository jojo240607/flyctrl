//! 虚拟 IMU 驱动：从全局 `PLAYBACK` 读取回放数据集，伪装成真实 IMU（accel+gyro）。
//!
//! 与 `imu::mpu6050::ImuMpu6050` 实现同一 `ImuSensor` trait，使 `sensors_task`
//! 只需切换数据源即可，无需改动采集逻辑。

use flyctrl_core::hal::sensor::ImuSensor;
use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::ImuSample;

use crate::sensors::sim::dataset::{Frame, PLAYBACK};

/// IMU 加速度计测量的是"比力"（含重力），而 `ImuSample.accel` 语义为机体加速度（不含重力）。
/// 数据集里的 `imu_accel` 为含重力值，这里减去近水平的重力分量得到比力。
const GRAVITY: f32 = 9.81;

pub struct VirtualImu;

impl VirtualImu {
    pub fn new() -> Option<Self> {
        Some(Self)
    }
}

impl ImuSensor for VirtualImu {
    fn read(&mut self) -> ImuSample {
        let f: Frame = unsafe { PLAYBACK.current() };
        let ax = f.imu_accel[0];
        let ay = f.imu_accel[1];
        let az = f.imu_accel[2] - GRAVITY; // 去除重力 -> 比力（机体加速度）
        ImuSample {
            accel: [
                MeterPerSecondSquared(ax),
                MeterPerSecondSquared(ay),
                MeterPerSecondSquared(az),
            ],
            gyro: [
                RadianPerSecond(f.imu_gyro[0]),
                RadianPerSecond(f.imu_gyro[1]),
                RadianPerSecond(f.imu_gyro[2]),
            ],
        }
    }

    fn healthy(&self) -> bool {
        true
    }
}
