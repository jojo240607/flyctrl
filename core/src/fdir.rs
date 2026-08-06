//! FDIR 雏形（Fault Detection, Isolation, and Recovery）。
//!
//! 轻量级传感器健康监控：检测 IMU 饱和/冻结（卡死）与 GPS/位置测量丢失（dropout），
//! 输出健康等级与建议的降级模式。控制器/估计器可据此切换策略（如 GPS 丢失时
//! 退化为仅姿态 + 高度保持，或 IMU 冻结时冻结估计）。
//!
//! 设计目标：纯函数式、无堆、确定性强，便于在 MCU 上常驻运行。当前为算法基线，
//! 真实硬件接入时再叠加投票/时序一致性等更强判据。

use crate::vehicle::ImuSample;

/// 传感器健康等级。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Health {
    /// 正常。
    Nominal,
    /// 降级：某传感器部分失效，仍可控（如 GPS dropout，仅高度/姿态保持）。
    Degraded,
    /// 危险：关键传感器失效（如 IMU 冻结），应进入安全模式（缓慢降落/保持）。
    Critical,
}

/// FDIR 监控状态（含简单滑动窗口统计量与超时计时）。
pub struct Fdir {
    /// 连续 GPS 丢失计数（步）。
    gps_lost_steps: u32,
    /// GPS dropout 判定阈值（步）。
    gps_timeout: u32,
    /// 上一拍 IMU 加速度读数（用于检测冻结）。
    last_acc: [f32; 3],
    /// IMU 连续不变计数（冻结检测）。
    imu_stale_steps: u32,
    /// 冻结判定阈值（步）：连续 N 拍 IMU 完全一致视为卡死。
    imu_stale_timeout: u32,
    /// 当前健康等级。
    health: Health,
}

impl Fdir {
    pub fn new() -> Self {
        Self {
            gps_lost_steps: 0,
            gps_timeout: 40,        // 40*5ms = 200ms 无 GPS 即降级
            last_acc: [0.0; 3],
            imu_stale_steps: 0,
            imu_stale_timeout: 20,  // 连续 20 拍（100ms）IMU 不变判冻结
            health: Health::Nominal,
        }
    }

    /// 单步更新：喂入 IMU 与 GPS 可用性（pos 为 None 表示本拍无位置测量）。
    /// 返回当前健康等级。
    pub fn update(&mut self, imu: &ImuSample, pos_available: bool) -> Health {
        // --- IMU 冻结检测：加速度三轴连续不变 ---
        let frozen = imu.accel[0].0 == self.last_acc[0]
            && imu.accel[1].0 == self.last_acc[1]
            && imu.accel[2].0 == self.last_acc[2];
        if frozen {
            self.imu_stale_steps += 1;
        } else {
            self.imu_stale_steps = 0;
        }
        self.last_acc = [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0];

        // --- GPS/位置测量 dropout 检测 ---
        if pos_available {
            self.gps_lost_steps = 0;
        } else {
            self.gps_lost_steps += 1;
        }

        // --- 健康等级裁决 ---
        if self.imu_stale_steps >= self.imu_stale_timeout {
            self.health = Health::Critical; // IMU 冻结 -> 危险
        } else if self.gps_lost_steps >= self.gps_timeout {
            self.health = Health::Degraded; // GPS 丢失 -> 降级（仍可姿态保持）
        } else {
            self.health = Health::Nominal;
        }
        self.health
    }

    pub fn health(&self) -> Health { self.health }

    /// 是否建议降级（GPS 丢失）：估计器可据此忽略位置测量、仅做姿态/高度保持。
    pub fn degraded(&self) -> bool { self.health != Health::Nominal }

    /// 是否进入安全模式（IMU 冻结等致命故障）。
    pub fn critical(&self) -> bool { self.health == Health::Critical }
}

impl Default for Fdir {
    fn default() -> Self { Self::new() }
}
