//! FDIR（Fault Detection, Isolation, and Recovery）。
//!
//! 轻量级多源传感器健康监控：检测 IMU 冻结（卡死）、GPS/位置测量丢失（dropout）、
//! 气压计（高度源）与磁力计（航向源）失效，输出健康等级与降级建议。
//! 控制器/估计器可据此切换策略（如 GPS 丢失时退化为仅姿态 + 高度保持；
//! IMU 冻结时冻结估计并进入安全降落）。
//!
//! 设计目标：纯函数式、无堆、确定性强，便于在 MCU 上常驻运行。真实硬件接入时
//! 可再叠加投票/时序一致性等更强判据。

use crate::vehicle::{ImuSample, Ned};

/// 传感器健康等级。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Health {
    /// 正常。
    Nominal,
    /// 降级：某非关键传感器部分失效，仍可控（如 GPS dropout，仅高度/姿态保持）。
    Degraded,
    /// 危险：关键传感器失效（如 IMU 冻结），应进入安全模式（缓慢降落/保持）。
    Critical,
}

/// FDIR 健康原因位（便于 SYS_STATUS / 地面站上报具体失效源）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HealthFlags {
    /// IMU 冻结（致命）。
    pub imu_frozen: bool,
    /// GPS/位置测量丢失。
    pub gps_lost: bool,
    /// 气压计（高度源）失效。
    pub baro_lost: bool,
    /// 磁力计（航向源）失效。
    pub mag_lost: bool,
}

impl HealthFlags {
    pub fn all_ok() -> Self {
        Self { imu_frozen: false, gps_lost: false, baro_lost: false, mag_lost: false }
    }
    /// 是否仍具备高度估计能力（IMU + 气压计至少其一可用）。
    pub fn has_altitude(&self) -> bool { !self.imu_frozen && !self.baro_lost }
    /// 是否仍具备航向估计能力（IMU + 磁力计至少其一可用）。
    pub fn has_heading(&self) -> bool { !self.imu_frozen && !self.mag_lost }
    /// 是否仍具备水平位置估计能力（需 GPS 且 IMU 正常）。
    pub fn has_position(&self) -> bool { !self.imu_frozen && !self.gps_lost }
}

/// FDIR 监控状态（含简单滑动窗口统计量与超时计时）。
pub struct Fdir {
    /// 连续 GPS 丢失计数（步）。
    gps_lost_steps: u32,
    /// GPS dropout 判定阈值（步）。
    gps_timeout: u32,
    /// 连续气压计丢失计数（步）。
    baro_lost_steps: u32,
    baro_timeout: u32,
    /// 连续磁力计丢失计数（步）。
    mag_lost_steps: u32,
    mag_timeout: u32,
    /// 上一拍 IMU 加速度读数（用于检测冻结）。
    last_acc: [f32; 3],
    /// IMU 连续不变计数（冻结检测）。
    imu_stale_steps: u32,
    /// 冻结判定阈值（步）：连续 N 拍 IMU 完全一致视为卡死。
    imu_stale_timeout: u32,
    /// 当前健康等级。
    health: Health,
    /// 当前健康原因位。
    flags: HealthFlags,
}

impl Fdir {
    pub fn new() -> Self {
        Self {
            gps_lost_steps: 0,
            gps_timeout: 40,        // 40*5ms = 200ms 无 GPS 即降级
            baro_lost_steps: 0,
            baro_timeout: 80,       // 400ms 无气压计 → 标记 baro_lost
            mag_lost_steps: 0,
            mag_timeout: 80,
            last_acc: [0.0; 3],
            imu_stale_steps: 0,
            imu_stale_timeout: 20,  // 连续 20 拍（100ms）IMU 不变判冻结
            health: Health::Nominal,
            flags: HealthFlags::all_ok(),
        }
    }

    /// 单步更新：喂入 IMU、位置(GPS)可用性、气压计可用性、磁力计可用性。
    /// 返回当前健康等级。
    pub fn update(
        &mut self,
        imu: &ImuSample,
        pos_available: bool,
        baro_available: bool,
        mag_available: bool,
    ) -> Health {
        // --- IMU 冻结检测：加速度三轴连续不变 ---
        // 注意：稳定悬停时机体加速度确实长时间恒定（含约 9.81 m/s² 的重力比力），
        // 这**不是**故障。真正的 IMU 卡死通常输出恒定且明显偏离静力学重力的异常值
        // （如全 0 或噪声断流），故冻结判据要求：(a) 连续不变；(b) 加速度范数明显
        // 偏离"合理静态重力区间"（[6, 14] m/s²，覆盖失重/过载/断流等异常）。
        let a = [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0];
        let norm = libm::sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        let frozen = (norm < 6.0 || norm > 14.0)
            && a[0] == self.last_acc[0]
            && a[1] == self.last_acc[1]
            && a[2] == self.last_acc[2];
        if frozen {
            self.imu_stale_steps += 1;
        } else {
            self.imu_stale_steps = 0;
        }
        self.last_acc = a;

        // --- 各传感器 dropout 检测（滑动窗口计数）---
        self.gps_lost_steps = if pos_available { 0 } else { self.gps_lost_steps + 1 };
        self.baro_lost_steps = if baro_available { 0 } else { self.baro_lost_steps + 1 };
        self.mag_lost_steps = if mag_available { 0 } else { self.mag_lost_steps + 1 };

        // --- 健康原因位裁决 ---
        let flags = HealthFlags {
            imu_frozen: self.imu_stale_steps >= self.imu_stale_timeout,
            gps_lost: self.gps_lost_steps >= self.gps_timeout,
            baro_lost: self.baro_lost_steps >= self.baro_timeout,
            mag_lost: self.mag_lost_steps >= self.mag_timeout,
        };
        self.flags = flags;

        // --- 健康等级裁决 ---
        // 关键传感器（IMU）冻结 → 危险（必须安全模式）。
        // 仅 GPS 丢失但高度/航向源仍在 → 降级（可姿态/高度保持并 RTL）。
        // 气压计 + 磁力计同时失效（无高度/航向但 GPS 在）→ 降级（无自主能力，需 manual/land）。
        if flags.imu_frozen {
            self.health = Health::Critical;
        } else if flags.gps_lost {
            self.health = Health::Degraded;
        } else if flags.baro_lost || flags.mag_lost {
            self.health = Health::Degraded;
        } else {
            self.health = Health::Nominal;
        }
        self.health
    }

    pub fn health(&self) -> Health { self.health }

    /// 健康原因位（供 SYS_STATUS / 地面站精细上报）。
    pub fn flags(&self) -> HealthFlags { self.flags }

    /// 各传感器 dropout 判定阈值（步）：返回 (gps, baro, mag, imu_stale)。
    /// 供调用方估算"全部源恢复后回到 Nominal 的最大步数"等上界。
    pub fn timeouts(&self) -> (u32, u32, u32, u32) {
        (self.gps_timeout, self.baro_timeout, self.mag_timeout, self.imu_stale_timeout)
    }

    /// 是否建议降级（GPS 丢失）：估计器可据此忽略位置测量、仅做姿态/高度保持。
    pub fn degraded(&self) -> bool { self.health != Health::Nominal }

    /// 是否进入安全模式（IMU 冻结等致命故障）。
    pub fn critical(&self) -> bool { self.health == Health::Critical }
}

impl Default for Fdir {
    fn default() -> Self { Self::new() }
}

/// RTL 返航点记忆：首次获得可靠定位时锁定 home（起飞点），供 RTL 模式回飞。
///
/// 纯数据 + 纯函数，无堆；对应真实飞控的"记录起飞点 / home position"。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtlHome {
    /// home 的 NED 位置（起飞点；N/E 为水平偏差，D 为向下深度，单位 m）。
    pub pos: Ned,
    /// 是否已锁定 home（首次定位成功后置位）。
    pub locked: bool,
}

impl RtlHome {
    pub fn new() -> Self {
        Self { pos: Ned::origin(), locked: false }
    }

    /// 尝试锁定 home：仅在尚未锁定时记录当前位置（第一次可靠定位）。
    /// 返回是否本次发生了锁定。
    pub fn try_lock(&mut self, current: Ned) -> bool {
        if !self.locked {
            self.pos = current;
            self.locked = true;
            true
        } else {
            false
        }
    }

    /// 当前位置相对 home 的水平距离（m）；未锁定时返回 +inf 哨兵 1e9。
    pub fn horizontal_distance(&self, current: Ned) -> f32 {
        if !self.locked { return 1e9; }
        let dx = current.0[0].0 - self.pos.0[0].0;
        let dy = current.0[1].0 - self.pos.0[1].0;
        libm::sqrtf(dx * dx + dy * dy)
    }
}

impl Default for RtlHome {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::{ImuSample, MeterPerSecondSquared, RadianPerSecond};

    fn sample(accel: [f32; 3]) -> ImuSample {
        ImuSample {
            accel: [
                MeterPerSecondSquared(accel[0]),
                MeterPerSecondSquared(accel[1]),
                MeterPerSecondSquared(accel[2]),
            ],
            gyro: [RadianPerSecond(0.0); 3],
        }
    }

    #[test]
    fn nominal_when_all_sources_ok() {
        let mut f = Fdir::new();
        let h = f.update(&sample([0.0, 0.0, 9.8]), true, true, true);
        assert_eq!(h, Health::Nominal);
        assert_eq!(f.flags(), HealthFlags::all_ok());
    }

    #[test]
    fn degraded_on_gps_loss_not_critical() {
        let mut f = Fdir::new();
        f.update(&sample([0.0, 0.0, 9.8]), true, true, true); // 先建立基线
        // 连续 GPS 丢失超过阈值；IMU 注入微小变化避免误判冻结（真实 IMU 有噪声）
        let mut h = Health::Nominal;
        for i in 0..f.gps_timeout + 1 {
            let jitter = (i as f32) * 1e-3;
            h = f.update(&sample([0.1 + jitter, 0.0, 9.8]), false, true, true);
        }
        assert_eq!(h, Health::Degraded);
        assert!(f.flags().gps_lost);
        assert!(!f.critical()); // GPS 丢失不是致命
    }

    #[test]
    fn critical_on_imu_freeze() {
        let mut f = Fdir::new();
        f.update(&sample([0.0, 0.0, 9.8]), true, true, true);
        // IMU 冻结（每次读数完全相同）且范数明显偏离静态重力区间 [6,14] → 危险。
        // 用全 0 读数模拟断流/卡死（norm=0 < 6）。
        let mut h = Health::Nominal;
        for _ in 0..f.imu_stale_timeout + 1 {
            h = f.update(&sample([0.0, 0.0, 0.0]), true, true, true);
        }
        assert_eq!(h, Health::Critical);
        assert!(f.flags().imu_frozen);
        assert!(f.critical());
    }

    #[test]
    fn baro_loss_marks_degraded() {
        let mut f = Fdir::new();
        f.update(&sample([0.0, 0.0, 9.8]), true, true, true);
        let mut h = Health::Nominal;
        for i in 0..f.baro_timeout + 1 {
            let jitter = (i as f32) * 1e-3;
            h = f.update(&sample([0.1 + jitter, 0.0, 9.8]), true, false, true);
        }
        assert_eq!(h, Health::Degraded);
        assert!(f.flags().baro_lost);
    }

    #[test]
    fn rtl_home_locks_once() {
        let mut home = RtlHome::new();
        assert!(!home.locked);
        assert!(home.try_lock(Ned::origin()));
        assert!(home.locked);
        // 第二次尝试不改变已锁定的 home
        assert!(!home.try_lock(Ned::new(50.0, 0.0, -10.0)));
        assert_eq!(home.pos, Ned::origin());
        // 水平距离计算
        let d = home.horizontal_distance(Ned::new(3.0, 4.0, -1.0));
        assert!((d - 5.0).abs() < 1e-3);
    }

    #[test]
    fn rtl_home_unlocked_distance_is_sentinel() {
        let home = RtlHome::new();
        assert_eq!(home.horizontal_distance(Ned::new(3.0, 4.0, 0.0)), 1e9);
    }
}
