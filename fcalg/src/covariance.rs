//! L7 · 协方差传播 `P' = F·P·Fᵀ + Q`（21×21）
//! # 设计决定（显式）
//! 1. **只算上三角再镜像** ⇒ 结果**逐位对称**。理由：下游 Cholesky/Joseph 形式都假设对称，
//!    让不对称性累积没有意义（旧栈也是这么做的，这里把它写成契约而非实现细节）。
//! 2. **不做任何静默兜底**。旧栈的方差钳位写的是 `if !(x > 1e-6) { x = 1e-6 }`，
//!    而 `!(NaN > 1e-6)` **也为真** ⇒ NaN 被悄悄变成"看起来正常的地板值"
//!    ⇒ 既销毁证据、又制造虚假的可观测性。本层改为：**非有限 ⇒ `Err`；非正定 ⇒ `Err`**，
//!    由上层显式决定如何处理（并计数）。**绝不静默修正**。
//! 3. 性能：本实现是直白的 O(N³)，是拍内热点。**优化必须走"同一组判据"**
//!    （稀疏化/ILP 只允许改耗时，不允许改语义 —— 本文件的 6 条判据就是那条线）。
use crate::error_state::N;
use crate::finite::{gate_all, Stage, Violation};
/// 协方差矩阵。
pub type Cov = [[f32; N]; N];
/// 显式失败原因。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CovError {
    /// 输入或中间量非有限（**不得静默钳位**）。
    NonFinite(Violation),
    /// 结果非正定（方差已失去物理意义）。
    NotPositiveDefinite,
}
/// 逐位对称（构造上应恒真；用于判据）。
pub fn is_symmetric_exact(a: &Cov) -> bool {
    for i in 0..N {
        for j in 0..N {
            if a[i][j] != a[j][i] {
                return false;
            }
        }
    }
    true
}
/// Cholesky 判正定（非有限或非正主元 ⇒ false）。不修改输入、不静默修正。
pub fn is_positive_definite(a: &Cov) -> bool {
    let mut l = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..=i {
            let mut s = a[i][j];
            if !s.is_finite() {
                return false;
            }
            for k in 0..j {
                s -= l[i][k] * l[j][k];
            }
            if i == j {
                if !(s > 0.0) {
                    return false;
                }
                l[i][i] = s.sqrt();
            } else {
                if !(l[j][j] > 0.0) {
                    return false;
                }
                l[i][j] = s / l[j][j];
            }
        }
    }
    true
}
fn gate_cov(a: &Cov) -> Result<(), Violation> {
    for r in a.iter() {
        gate_all(Stage::L7Covariance, r)?;
    }
    Ok(())
}
/// 协方差预测。失败时**不返回任何部分结果**（调用方状态不受影响）。
pub fn propagate_covariance(p: &Cov, f: &[[f32; N]; N], q: &Cov) -> Result<Cov, CovError> {
    // ① 入口门（契约 §4）：非有限一律显式拒绝，**不做任何钳位**
    gate_cov(p).map_err(CovError::NonFinite)?;
    gate_cov(q).map_err(CovError::NonFinite)?;
    for r in f.iter() {
        gate_all(Stage::L7Covariance, r).map_err(CovError::NonFinite)?;
    }
    // ② FP = F·P
    let mut fp = [[0.0f32; N]; N];
    for i in 0..N {
        for k in 0..N {
            let fik = f[i][k];
            if fik == 0.0 {
                continue;
            }
            for j in 0..N {
                fp[i][j] += fik * p[k][j];
            }
        }
    }
    // ③ 上三角：out = FP·Fᵀ + Q，随后镜像 ⇒ 逐位对称
    let mut out = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..=i {
            let mut s = q[i][j];
            for k in 0..N {
                let fjk = f[j][k];
                if fjk == 0.0 {
                    continue;
                }
                s += fp[i][k] * fjk;
            }
            out[i][j] = s;
            out[j][i] = s;
        }
    }
    // ④ 后置：非有限 ⇒ 拒绝（**不得钳位**）
    gate_cov(&out).map_err(CovError::NonFinite)?;
    // ⑤ 后置：必须正定（方差失去物理意义 ⇒ 显式失败，交由上层决定重灌/复位）
    if !is_positive_definite(&out) {
        return Err(CovError::NotPositiveDefinite);
    }
    Ok(out)
}
