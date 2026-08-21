//! 关键不变量（形式化验证核心）。
//!
//! 飞控的"安全属性"用纯函数式断言表达，供属性测试（`core/tests/props_*.rs`）
//! 在 host 端穷举/随机化验证，并可在 MCU 端常驻运行（轻量、有界、无堆）。
//!
//! 覆盖四类不变量：
//! 1. 估计协方差正定（EKF P 矩阵对称半正定、对角元非负）。
//! 2. 控制输出有界（电机指令恒在 [0,1]）。
//! 3. 姿态四元数单位范数（|q|≈1，避免积分漂移导致姿态崩坏）。
//! 4. 估计/控制状态无 NaN（数值发散的早期拦截）。
//!
//! 所有函数返回 `bool`，且 `false` 即"违反不变量"，便于在断言与运行时监控复用。

use crate::math::sqrt;
use crate::vehicle::{ActuatorCmd, Quaternion, VehicleState};

/// 四元数单位范数容差（积分误差允许上限）。
pub const QUAT_NORM_TOL: f32 = 1e-3;

/// 协方差对角元下限（数值安全，避免完全退化到 0 以下）。
pub const COV_DIAG_MIN: f32 = 0.0;

/// 检查四元数是否单位范数（在容差内）。
/// 任意合法姿态四元数必须满足 `|q| == 1`，否则旋转/姿态估计失效。
pub fn quat_is_unit(q: Quaternion) -> bool {
    let n = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    (n - 1.0).abs() <= QUAT_NORM_TOL && n.is_finite()
}

/// 检查控制指令有界：四个电机推力恒在 [0,1]。
/// 越界意味着混控/限幅链失效，可能导致电机饱和或反向。
pub fn actuator_bounded(cmd: &ActuatorCmd) -> bool {
    for &m in &cmd.motor {
        if !m.is_finite() || m < 0.0 || m > 1.0 {
            return false;
        }
    }
    true
}

/// 检查估计状态无 NaN/Inf（数值发散的早期信号）。
/// 任一分量非有限即视为发散，应触发 FDIR 安全模式。
pub fn state_finite(s: &VehicleState) -> bool {
    s.pos.iter().all(|v| v.0.is_finite())
        && s.vel.iter().all(|v| v.0.is_finite())
        && s.omega.iter().all(|v| v.0.is_finite())
        && s.att.w.is_finite()
        && s.att.x.is_finite()
        && s.att.y.is_finite()
        && s.att.z.is_finite()
}

/// 检查协方差矩阵对称半正定（通过 Jacobi 旋转求所有特征值 ≥ 0）。
///
/// EKF 的 P 矩阵理论上必须对称半正定；数值误差可能破坏该性质，
/// 导致卡尔曼增益出现负方差（不可信估计）。本函数用固定迭代次数的
/// Jacobi 特征值分解（无堆、有界）校验最小特征值 ≥ `COV_DIAG_MIN`。
///
/// `p` 为 `n*n` 行主序方阵；`n` 必须 ≤ 12（满足当前 10 维 EKF + 余量）。
pub fn cov_psd(p: &[f32], n: usize) -> bool {
    if n == 0 || n > 12 || p.len() != n * n {
        return false;
    }
    // 先检查对称性与对角元非负（快速失败）。
    let sym_tol = 1e-4;
    for i in 0..n {
        if p[i * n + i] < COV_DIAG_MIN {
            return false;
        }
        for j in (i + 1)..n {
            if (p[i * n + j] - p[j * n + i]).abs() > sym_tol {
                return false;
            }
        }
    }
    // Jacobi 旋转求最小特征值。
    let mut a = [0.0f32; 144]; // 12*12 上限
    a[..n * n].copy_from_slice(&p[..n * n]);
    let mut min_ev = f32::INFINITY;
    // 固定迭代次数（n 维矩阵充分收敛的上界）。
    let iters = n * n * 4;
    for _ in 0..iters {
        // 找非对角最大元。
        let mut p_ = 0usize;
        let mut q_ = 0usize;
        let mut max = 0.0f32;
        for i in 0..n {
            for j in (i + 1)..n {
                let v = a[i * n + j].abs();
                if v > max {
                    max = v;
                    p_ = i;
                    q_ = j;
                }
            }
        }
        if max <= 1e-9 {
            break;
        }
        let app = a[p_ * n + p_];
        let aqq = a[q_ * n + q_];
        let apq = a[p_ * n + q_];
        let phi = 0.5 * (aqq - app) / apq;
        let t = if phi >= 0.0 {
            1.0 / (phi + sqrt(phi * phi + 1.0))
        } else {
            1.0 / (phi - sqrt(phi * phi + 1.0))
        };
        let c = 1.0 / sqrt(t * t + 1.0);
        let s = t * c;
        // 旋转 p,q 行列。
        for k in 0..n {
            let akp = a[k * n + p_];
            let akq = a[k * n + q_];
            a[k * n + p_] = c * akp - s * akq;
            a[k * n + q_] = s * akp + c * akq;
        }
        for k in 0..n {
            let apk = a[p_ * n + k];
            let aqk = a[q_ * n + k];
            a[p_ * n + k] = c * apk - s * aqk;
            a[q_ * n + k] = s * apk + c * aqk;
        }
    }
    // 对角元即特征值（近似）。
    for i in 0..n {
        let ev = a[i * n + i];
        if ev < min_ev {
            min_ev = ev;
        }
    }
    min_ev >= COV_DIAG_MIN - 1e-4
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vehicle::Quaternion;

    #[test]
    fn quat_identity_is_unit() {
        assert!(quat_is_unit(Quaternion::IDENTITY));
    }

    #[test]
    fn actuator_zero_bounded() {
        assert!(actuator_bounded(&ActuatorCmd::zero()));
    }

    #[test]
    fn actuator_overrange_detected() {
        let bad = ActuatorCmd { motor: [1.5, 0.0, 0.0, 0.0] };
        assert!(!actuator_bounded(&bad));
    }

    #[test]
    fn cov_identity_psd() {
        let mut p = [0.0f32; 9];
        for i in 0..3 {
            p[i * 3 + i] = 1.0;
        }
        assert!(cov_psd(&p, 3));
    }

    #[test]
    fn cov_negative_diag_not_psd() {
        let mut p = [0.0f32; 9];
        for i in 0..3 {
            p[i * 3 + i] = 1.0;
        }
        p[0] = -1.0; // 负对角元
        assert!(!cov_psd(&p, 3));
    }
}
