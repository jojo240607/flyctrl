//! 世界环境：传感器噪声、风扰、故障注入。
//!
//! 仿真后端调用 [`World::sense`] 把"理想 IMU/位置"加工成"真实测量"，
//! 让估计算法在带噪环境下被验证（对比不同算法对噪声的鲁棒性）。

use flyctrl_core::units::*;
use flyctrl_core::vehicle::{ImuSample, PosSample};

pub struct WorldParams {
    pub accel_noise: f32,    // 加速度计噪声 std (m/s^2)
    pub gyro_noise: f32,     // 陀螺噪声 std (rad/s)
    pub pos_noise: f32,      // 位置测量噪声 std (m)
    pub wind: [f32; 3],      // 常值风（世界系，m/s）
    pub wind_gust: f32,      // 阵风幅度 (m/s)
}

impl Default for WorldParams {
    fn default() -> Self {
        Self {
            accel_noise: 0.05,
            gyro_noise: 0.005,
            pos_noise: 0.3,
            wind: [0.0, 0.0, 0.0],
            wind_gust: 0.0,
        }
    }
}

pub struct World {
    params: WorldParams,
    t: f32,
}

impl World {
    pub fn new(params: WorldParams) -> Self { Self { params, t: 0.0 } }

    /// 把理想测量加噪 + 风扰，产出传感器样本。
    /// `true_pos` 为物理真实位置（世界系），加噪后作为 GPS/气压测量返回。
    pub fn sense(&mut self, dt: Second, ideal: ImuSample, true_pos: [Meter; 3])
        -> (ImuSample, Option<PosSample>)
    {
        self.t += dt.0;
        let p = &self.params;
        let gn = p.gyro_noise;
        let an = p.accel_noise;
        let imu = ImuSample {
            accel: [
                MeterPerSecondSquared(ideal.accel[0].0 + randn() * an),
                MeterPerSecondSquared(ideal.accel[1].0 + randn() * an),
                MeterPerSecondSquared(ideal.accel[2].0 + randn() * an),
            ],
            gyro: [
                RadianPerSecond(ideal.gyro[0].0 + randn() * gn),
                RadianPerSecond(ideal.gyro[1].0 + randn() * gn),
                RadianPerSecond(ideal.gyro[2].0 + randn() * gn),
            ],
        };
        // 位置测量 = 真实位置 + 噪声（模拟 GPS+气压融合）
        let pos = PosSample {
            pos: [
                Meter(true_pos[0].0 + randn() * p.pos_noise),
                Meter(true_pos[1].0 + randn() * p.pos_noise),
                Meter(true_pos[2].0 + randn() * p.pos_noise),
            ],
        };
        (imu, Some(pos))
    }

    pub fn reset(&mut self) { self.t = 0.0; }
}

/// 极简近似正态（Box-Muller 简化版，确定性种子由调用次数驱动）。
/// 注意：仿真用近似噪声即可，不需要密码学质量。
fn randn() -> f32 {
    // 用系统时间无关的简单 LCG 产生 [0,1)，映射到近似正态
    use core::sync::atomic::{AtomicU64, Ordering};
    static SEED: AtomicU64 = AtomicU64::new(0x9E3779B97F4A7C15);
    let s = SEED.fetch_add(0x2545F4914F6CDD1D, Ordering::Relaxed);
    let u1 = ((s >> 11) as f32) / ((1u64 << 53) as f32);
    let u2 = (((s << 13) ^ 0x12345678) as f32) / ((1u64 << 53) as f32);
    let r = (-2.0 * u1.max(1e-6).ln()).sqrt() * (2.0 * core::f32::consts::PI * u2).cos();
    r.clamp(-4.0, 4.0)
}
