//! L10 · 门控与恢复
//! # 旧栈缺的那一环
//! 连续拒收/融合超时后**真的重灌方差**。旧栈没有这一环 ⇒ 一旦方差塌到地板，
//! 增益≈0 ⇒ 观测再也拉不动状态 ⇒ **永久失去可观测性**（实测：Pzz 塌到 1e-6 后
//! 垂直通道永不恢复，估计一路跑飞）。
//! # 与旧栈"静默钳位"的区别（同一段代码的两种命运，必须写清）
//! | | 旧栈方差钳位 | 本层重灌 |
//! |---|---|---|
//! | 方向 | **压到地板**（销毁可观测性） | **抬升**（恢复可观测性） |
//! | 触发 | **无条件、静默** | **阈值触发 + 计数**（显式） |
//! | 非有限 | `!(NaN>1e-6)` 为真 ⇒ 被悄悄变成地板 | **一律 `Err`，绝不参与**（那才是掩盖） |
//! # 「每回合只触发一次」
//! 连续拒收达阈值即重灌，并计一次 `recoveries`；此后继续拒收**不再重复重灌**
//! （否则每次拒收都把方差抬满 ⇒ 退化成"永远不收敛"）。收到一次有效融合即 `accepted()`
//! 复位，下一回合重新计数。
use crate::covariance::Cov;
use crate::error_state::{I_ATT, I_MAGB, I_MAGI, I_POS, I_VEL};
use crate::finite::{gate_all, Stage, Violation};
/// 观测通道。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Baro,
    GpsPos,
    GpsVel,
    MagYaw,
}
/// 通道 → 它**能观测到**的误差态索引（重灌只该抬这些）。
pub fn channel_indices(ch: Channel) -> &'static [usize] {
    match ch {
        Channel::Baro => &[I_POS + 2],
        Channel::GpsPos => &[I_POS, I_POS + 1, I_POS + 2],
        Channel::GpsVel => &[I_VEL, I_VEL + 1, I_VEL + 2],
        // 航向观测耦合姿态、磁惯性系、磁机体系三块
        Channel::MagYaw => &[
            I_ATT,
            I_ATT + 1,
            I_ATT + 2,
            I_MAGI,
            I_MAGI + 1,
            I_MAGI + 2,
            I_MAGB,
            I_MAGB + 1,
            I_MAGB + 2,
        ],
    }
}
/// 单通道的连续拒收计数与重灌计数。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelGuard {
    max_rejects: u32,
    rejects: u32,
    recoveries: u32,
}
impl ChannelGuard {
    pub fn new(max_rejects: u32) -> Self {
        Self { max_rejects: max_rejects.max(1), rejects: 0, recoveries: 0 }
    }
    /// 收到一次有效融合 ⇒ 本回合结束，计数复位。
    pub fn accepted(&mut self) {
        self.rejects = 0;
    }
    /// 记一次拒收；返回**是否应触发重灌**（每回合恰一次）。
    pub fn rejected(&mut self) -> bool {
        self.rejects += 1;
        if self.rejects == self.max_rejects {
            self.recoveries += 1;
            true
        } else {
            false
        }
    }
    pub fn rejects(&self) -> u32 {
        self.rejects
    }
    pub fn recoveries(&self) -> u32 {
        self.recoveries
    }
}
/// 把指定对角**抬升**到不低于 `floor`（**只升不降**），返回被抬升的项数。
/// 非有限 ⇒ `Err`（契约 §4：绝不参与重灌 —— 那是掩盖）。
pub fn reflate_diag(p: &mut Cov, idx: &[usize], floor: f32) -> Result<u32, Violation> {
    if !(floor.is_finite() && floor > 0.0) {
        return Err(if floor.is_nan() { Violation::Nan } else { Violation::Inf });
    }
    let mut n = 0;
    for &i in idx {
        let v = p[i][i];
        gate_all(Stage::L10Gate, &[v])?;
        if v < floor {
            p[i][i] = floor;
            n += 1;
        }
    }
    Ok(n)
}
