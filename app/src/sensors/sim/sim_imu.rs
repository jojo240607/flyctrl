//! 占位 IMU 源：总线/传感器不可用时启用，平滑正弦激励，避免 FDIR 误判冻结。
//!
//! 坐标系约定与飞控一致（FRD 前-右-下）：水平悬停时加速度计比力应为
//! `(0,0,-9.81)`（比力 = 实际加速度 - 重力；悬停 a=0，FRD 下重力矢量 `(0,0,+9.81)`，
//! 故比力取反）。此前误写成 `+9.81`（与真实悬停比力反向），HIL 注入饥饿回退时
//! EKF 把 `a_world = R·a_body + g_vec` 算成垂向 2g 加速度，把高度估计拖偏。

use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::ImuSample;

pub struct SimImu {
    t: f32,
}

impl SimImu {
    pub fn new() -> Self {
        Self { t: 0.0 }
    }

    pub fn next(&mut self, dt: f32) -> ImuSample {
        self.t += dt;
        ImuSample {
            accel: [
                MeterPerSecondSquared(0.05 * libm::sinf(self.t)),
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(-9.81 + 0.05 * libm::cosf(self.t)),
            ],
            gyro: [RadianPerSecond(0.0); 3],
        }
    }
}
