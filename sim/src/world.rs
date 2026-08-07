//! 世界环境：传感器噪声、风扰、故障注入。
//!
//! 仿真后端调用 [`World::sense`] 把"理想 IMU/位置"加工成"真实测量"，
//! 让估计算法在带噪环境下被验证（对比不同算法对噪声的鲁棒性）。

use flyctrl_core::units::*;
use flyctrl_core::vehicle::{ImuSample, PosSample};

/// 传感器故障类型。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FaultKind {
    /// 无故障（仅噪声）。
    None,
    /// IMU 恒定偏置漂移（加速度计/陀螺整体加常数）。
    ImuDrift,
    /// IMU 卡死（冻结在故障触发时刻的值，不再更新）。
    ImuStuck,
    /// GPS/位置测量丢帧（持续窗口内返回 None，位置测量不可用）。
    GpsDropout,
}

impl FaultKind {
    pub fn name(self) -> &'static str {
        match self {
            FaultKind::None => "None",
            FaultKind::ImuDrift => "ImuDrift",
            FaultKind::ImuStuck => "ImuStuck",
            FaultKind::GpsDropout => "GpsDropout",
        }
    }
    pub fn parse(s: &str) -> FaultKind {
        match s.to_lowercase().as_str() {
            "imudrift" | "drift" => FaultKind::ImuDrift,
            "imustuck" | "stuck" => FaultKind::ImuStuck,
            "gpsdropout" | "gps" | "dropout" => FaultKind::GpsDropout,
            _ => FaultKind::None,
        }
    }
}

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
            gyro_noise: 0.0003,
            pos_noise: 0.3,
            wind: [0.0, 0.0, 0.0],
            wind_gust: 0.0,
        }
    }
}

pub struct World {
    params: WorldParams,
    t: f32,
    /// 激活的故障类型。
    fault: FaultKind,
    /// 故障窗口 [t0, t1)（秒）。
    fault_window: (f32, f32),
    /// IMU 卡死时冻结的样本。
    stuck_imu: Option<ImuSample>,
}

impl World {
    pub fn new(params: WorldParams) -> Self {
        Self {
            params, t: 0.0,
            fault: FaultKind::None,
            fault_window: (0.0, 0.0),
            stuck_imu: None,
        }
    }

    /// 配置传感器故障：类型 + 触发窗口 [t0,t1)（秒）。
    /// 漂移偏置在触发瞬间内部确定性生成（±固定量级）。
    pub fn set_fault(&mut self, fault: FaultKind, t0: f32, t1: f32) {
        self.fault = fault;
        self.fault_window = (t0, t1);
    }

    /// 当前是否处于故障激活窗口内。
    fn fault_active(&self) -> bool {
        self.fault != FaultKind::None
            && self.t >= self.fault_window.0
            && self.t < self.fault_window.1
    }

    /// 把理想测量加噪 + 风扰 + 故障注入，产出传感器样本。
    /// `true_pos` 为物理真实位置（世界系），加噪后作为 GPS/气压测量返回。
    pub fn sense(&mut self, dt: Second, ideal: ImuSample, true_pos: [Meter; 3])
        -> (ImuSample, Option<PosSample>)
    {
        self.t += dt.0;
        let p = &self.params;
        let gn = p.gyro_noise;
        let an = p.accel_noise;
        let active = self.fault_active();

        let mut imu = ImuSample {
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

        match (self.fault, active) {
            (FaultKind::ImuDrift, true) => {
                // 恒定偏置：触发窗口内叠加固定偏移（确定性量级）。
                imu.accel[0] = MeterPerSecondSquared(imu.accel[0].0 + 0.4);
                imu.accel[2] = MeterPerSecondSquared(imu.accel[2].0 - 0.4);
                imu.gyro[2] = RadianPerSecond(imu.gyro[2].0 + 0.03);
            }
            (FaultKind::ImuStuck, true) => {
                // 卡死：冻结触发时刻的样本。首次进入时记录。
                match self.stuck_imu {
                    Some(s) => imu = s,
                    None => {
                        self.stuck_imu = Some(imu);
                    }
                }
            }
            _ => {}
        }
        if !active {
            self.stuck_imu = None; // 窗口外重置卡死记录
        }

        // 位置测量 = 真实位置 + 噪声（模拟 GPS+气压融合）
        let pos = if active && self.fault == FaultKind::GpsDropout {
            None // 丢帧：位置测量不可用
        } else {
            Some(PosSample {
                pos: [
                    Meter(true_pos[0].0 + randn() * p.pos_noise),
                    Meter(true_pos[1].0 + randn() * p.pos_noise),
                    Meter(true_pos[2].0 + randn() * p.pos_noise),
                ],
            })
        };
        (imu, pos)
    }

    pub fn reset(&mut self) {
        self.t = 0.0;
        self.stuck_imu = None;
    }
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
