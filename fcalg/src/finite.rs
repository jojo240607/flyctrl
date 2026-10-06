//! L1 · 有限性纪律 —— 永不静默（契约 §4）。所有模块的共同底座。
//!
//! 为何独立成层：旧框架最大的可定位性缺陷是**静默兜底** ——
//! 方差钳位写的是 `if !(x > 1e-6) { x = 1e-6 }`，而 `!(NaN > 1e-6)` **也为真**
//! ⇒ NaN 被悄悄变成"看起来正常的地板值" ⇒ 既丢证据、又制造虚假的正常。

use core::sync::atomic::{AtomicU32, Ordering};

/// 阶段编号（按模块固定分配；`Unnamed` 保留）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum Stage {
    Unnamed = 0,
    L2ImuFilter = 1,
    L3ImuDelta = 2,
    L4Align = 3,
    L5Propagate = 4,
    L6Transition = 5,
    L7Covariance = 6,
    L8Update = 7,
    L9ObsBaro = 8,
    L9ObsGps = 9,
    L9ObsMag = 10,
    L10Gate = 11,
    L11CtrlAtt = 12,
    L11CtrlRate = 13,
    L11Mixer = 14,
    L13Calib = 15,
}

pub const N_STAGES: usize = 16;

const ZERO: AtomicU32 = AtomicU32::new(0);
static VIOLATIONS: [AtomicU32; N_STAGES] = [ZERO; N_STAGES];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Violation {
    Nan,
    Inf,
}

/// 契约 §4 的唯一把关口：有限 ⇒ `Ok(v)`；否则计数并返回 `Err`。
#[inline]
pub fn gate(stage: Stage, v: f32) -> Result<f32, Violation> {
    if v.is_finite() {
        Ok(v)
    } else {
        VIOLATIONS[stage as usize].fetch_add(1, Ordering::Relaxed);
        Err(if v.is_nan() { Violation::Nan } else { Violation::Inf })
    }
}

/// 逐分量把关（向量/矩阵/四元数）。
#[inline]
pub fn gate_all<const N: usize>(stage: Stage, v: &[f32; N]) -> Result<(), Violation> {
    for x in v {
        gate(stage, *x)?;
    }
    Ok(())
}

/// 某阶段累计违规数。
#[inline]
pub fn violations(stage: Stage) -> u32 {
    VIOLATIONS[stage as usize].load(Ordering::Relaxed)
}

/// 全阶段累计违规数（集成验收的直接判据）。
pub fn violations_total() -> u32 {
    VIOLATIONS.iter().map(|c| c.load(Ordering::Relaxed)).sum()
}

pub fn reset() {
    for c in VIOLATIONS.iter() {
        c.store(0, Ordering::Relaxed);
    }
}
