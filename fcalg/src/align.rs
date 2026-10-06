//! L4 · 静止对齐 / 初始化
//!
//! # 模块契约
//!
//! **输入**：静止期平均比力 `a_body`（m/s²，机体系 FRD）+ 平均陀螺 `gyro`（rad/s，机体系）；
//! **输出**：`AlignResult { q, gyro_bias }` —— 初姿（机体→世界）与陀螺零偏。
//!
//! # 三条硬约束（旧栈正是在这里出的事）
//!
//! 1. **入口硬门：非有限禁入。** 任何 NaN/±Inf ⇒ 立刻返回 `Err`，
//!    **绝不产出姿态**（旧栈是拿着垃圾比力算出垃圾姿态，之后一路 NaN 到滤波器里 ✗）；
//! 2. **量级判据**：`| ‖a_body‖ − g | / g ≤ tol` 才认为是"静止且只有重力"。
//!    不满足 ⇒ 显式 `NotAtRest`，**不得**用"比力大于某个值就放行"这种松判据
//!    （旧栈 `raw_accel=1.11` 放行 `an>1.0` ⇒ tilt alignment 垃圾 ⇒ roll 翻 π ✗）；
//! 3. **yaw 不可观**：无外部航向参考时，对齐**不得发明 yaw** ——
//!    取 yaw = 0（等价于取最小旋转），并在输出里如实反映。yaw 由磁/外部观测后续给出。
//!
//! # 反解式（由 `CONTRACT.md` §2 的比力关系导出，本文件是它的唯一实现处）
//!
//! 由 `Rᵀ ẑ = (−sinθ, cosθ·sinφ, cosθ·cosφ)`（ZYX）与 `f_b = −g·Rᵀ ẑ` 得
//! `f_b = g·(sinθ, −cosθ·sinφ, −cosθ·cosφ)`，故
//! `θ = asin(a_x/g)`，`φ = atan2(−a_y, −a_z)`。

use crate::finite::{gate_all, Stage, Violation};
use crate::math;
use crate::quat::{gate_quat, Quat, GRAVITY_NED};

/// 对齐配置。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlignConfig {
    /// 比力量级的**相对**容差（如 0.06 = 6%）。
    pub g_tol_frac: f32,
}

impl Default for AlignConfig {
    fn default() -> Self {
        Self { g_tol_frac: 0.06 }
    }
}

/// 对齐结果。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AlignResult {
    /// 初姿（机体→世界）；yaw 恒为 0（不可观）。
    pub q: Quat,
    /// 陀螺零偏（= 静止期平均陀螺，机体系）。
    pub gyro_bias: [f32; 3],
}

/// 显式失败原因（不静默）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlignError {
    /// 输入非有限 —— **硬门**，绝不产出姿态。
    NonFinite(Violation),
    /// 比力量级不满足 ≈g。
    NotAtRest { mag: f32, expected: f32 },
}

/// 静止对齐：由平均比力求初姿（yaw 取 0），由平均陀螺求零偏。
pub fn align_static(
    a_body: [f32; 3],
    gyro: [f32; 3],
    cfg: AlignConfig,
) -> Result<AlignResult, AlignError> {
    // ① 入口硬门（契约 §4）
    let fin = gate_all(Stage::L4Align, &a_body).and_then(|_| gate_all(Stage::L4Align, &gyro));
    if let Err(v) = fin {
        return Err(AlignError::NonFinite(v));
    }

    // ② 量级判据：必须"只有重力"。用**标称 g**（契约固定），而不是测量模长 ——
    //    否则任何模长都"自洽"，判据就失去意义。
    let g = GRAVITY_NED[2];
    let mag = math::sqrt(a_body[0] * a_body[0] + a_body[1] * a_body[1] + a_body[2] * a_body[2]);
    if !(mag.is_finite() && mag > 0.0) || ((mag - g).abs() / g) > cfg.g_tol_frac {
        return Err(AlignError::NotAtRest { mag, expected: g });
    }

    // ③ 反解 tilt（yaw = 0）
    let theta = math::asin((a_body[0] / g).clamp(-1.0, 1.0));
    let phi = math::atan2(-a_body[1], -a_body[2]);
    let q = Quat::from_euler_zyx([phi, theta, 0.0])
        .normalize()
        .ok_or(AlignError::NonFinite(Violation::Nan))?;

    // 后置：产物也必须有限（不得让坏系数/退化走到下游）
    gate_quat(Stage::L4Align, q).map_err(AlignError::NonFinite)?;

    Ok(AlignResult { q, gyro_bias: gyro })
}
