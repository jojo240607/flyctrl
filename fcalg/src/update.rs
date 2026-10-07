//! L8 · 通用 3 轴量测更新（**标准形 `P − K·S·Kᵀ`** + NIS 卡方门）
//! # 为什么是**函数式**（不就地改 P）
//! 旧栈靠"约定记得在拒绝时不 apply"来保证"拒收 ⇒ P 不变"。本层改成：`update` 取 `&Cov`，
//! **返回新的 P 与 dx** ⇒ 拒绝路径**在结构上不可能**碰到 P ✗。
//! 这比"靠约定"强：不变量由类型保证，而不是靠人记住。
//! # 顺序（显式）
//! `S = H·P·Hᵀ + R` → `S⁻¹` → **NIS 门** → `K = P·Hᵀ·S⁻¹` → `dx = K·ν` →
//! 更新式：`P' = P − K·S·Kᵀ`（只算上三角再镜像 ⇒ 逐位对称；A 提速 ✓ ~14× 于该步）。
//! 门在算 K 之前 ⇒ 拒收时不做任何多余计算，也不产生任何中间副作用。
//! # 纪律
//! 全程过 L1 门；非有限 ⇒ `Err`；`S` 奇异 ⇒ `Err`；结果非正定 ⇒ `Err`。
//! 应用 `dx` 到标称态（`boxplus`）由调用方负责 —— 本层是**纯线性代数**。
#[allow(unused_imports)]
use crate::math::F32Ext;

use crate::covariance::{has_negative_variance, Cov};
use crate::error_state::N;
use crate::finite::{gate_all, Stage, Violation};
/// 更新输出。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UpdateOut {
    /// 更新后的协方差（逐位对称、已验正定）。
    pub p: Cov,
    /// 误差态修正量（由调用方用 `boxplus` 施加）。
    pub dx: [f32; N],
    /// 本拍新息的 `sqrt(NIS)`（**NIS 一致性标定的原料** —— 期望值由有效观测轴数决定）。
    pub nis_sigma: f32,
}
/// 显式失败原因。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UpdateError {
    NonFinite(Violation),
    /// `S` 奇异/不可逆。
    SingularS,
    /// 新息超门限（**拒收**；P 与状态保持不变）。
    Rejected { nis_sigma: f32 },
    /// 结果非正定 —— 显式失败，**不得静默修正**。
    NotPositiveDefinite,
}
/// 3×3 求逆（伴随矩阵/行列式）。
/// 3×3 求逆（伴随矩阵/行列式）。**共用**：标定模块不得再复制第二份实现（避免实现分叉）。
pub(crate) fn inv3(a: &[[f32; 3]; 3]) -> Option<[[f32; 3]; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if !det.is_finite() || det.abs() < 1e-12 {
        return None;
    }
    let inv = 1.0 / det;
    Some([
        [
            (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * inv,
            (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * inv,
            (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * inv,
        ],
        [
            (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * inv,
            (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * inv,
            (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * inv,
        ],
        [
            (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * inv,
            (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * inv,
            (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * inv,
        ],
    ])
}
/// 通用 3 轴更新。
/// **写入口**：`P` 与 `dx` 写进调用方缓冲，返回 `nis_sigma` ——
/// **不再按值返回 `UpdateOut`**（其 `p: Cov` = 1,764 B 的返回临时量 ✗，
/// 是实测栈溢出 4,128 B 的另一来源）✓。
/// ★A-实测：`update_into` 每拍被调用几次 ✓
pub static UPD_CALLS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
pub fn update_into(
    p: &Cov,
    h: &[[f32; N]; 3],
    resid: &[f32; 3],
    r: &[[f32; 3]; 3],
    gate_sigma: f32,
    out_p: &mut Cov,
    out_dx: &mut [f32; N],
) -> Result<f32, UpdateError> {
    // ① 入口门
    for row in p.iter() {
        gate_all(Stage::L8Update, row).map_err(UpdateError::NonFinite)?;
    }
    for row in h.iter() {
        gate_all(Stage::L8Update, row).map_err(UpdateError::NonFinite)?;
    }
    for row in r.iter() {
        gate_all(Stage::L8Update, row).map_err(UpdateError::NonFinite)?;
    }
    gate_all(Stage::L8Update, resid).map_err(UpdateError::NonFinite)?;
    if !(gate_sigma.is_finite() && gate_sigma > 0.0) {
        return Err(UpdateError::NonFinite(if gate_sigma.is_nan() {
            Violation::Nan
        } else {
            Violation::Inf
        }));
    }
    let _ = UPD_CALLS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    // ② PHᵀ
    let mut pht = [[0.0f32; 3]; N];
    for i in 0..N {
        for j in 0..3 {
            let mut s = 0.0f32;
            for k in 0..N {
                s += p[i][k] * h[j][k];
            }
            pht[i][j] = s;
        }
    }
    // ③ S = H·PHᵀ + R
    let mut s_mat = *r;
    for a in 0..3 {
        for b in 0..3 {
            let mut s = 0.0f32;
            for k in 0..N {
                s += h[a][k] * pht[k][b];
            }
            s_mat[a][b] += s;
        }
    }
    let s_inv = inv3(&s_mat).ok_or(UpdateError::SingularS)?;
    // ④ NIS 门（在算 K 之前 ⇒ 拒收路径不做多余计算）
    let mut tmp = [0.0f32; 3];
    for a in 0..3 {
        let mut s = 0.0f32;
        for b in 0..3 {
            s += s_inv[a][b] * resid[b];
        }
        tmp[a] = s;
    }
    let mut nis = 0.0f32;
    for a in 0..3 {
        nis += resid[a] * tmp[a];
    }
    if !nis.is_finite() {
        return Err(UpdateError::NonFinite(Violation::Nan));
    }
    if nis < 0.0 {
        return Err(UpdateError::SingularS);
    }
    let nis_sigma = nis.sqrt();
    if nis_sigma > gate_sigma {
        return Err(UpdateError::Rejected { nis_sigma });
    }
    // ⑤ K = PHᵀ·S⁻¹；dx = K·ν
    let mut k = [[0.0f32; 3]; N];
    for i in 0..N {
        for b in 0..3 {
            let mut s = 0.0f32;
            for a in 0..3 {
                s += pht[i][a] * s_inv[a][b];
            }
            k[i][b] = s;
        }
        let mut d = 0.0f32;
        for a in 0..3 {
            d += k[i][a] * resid[a];
        }
        out_dx[i] = d;
    }
    // ⑥ **标准形更新（A 提速 ✓）**：P' = P − K·S·Kᵀ
    //   动因（实测 ✓）：真固件里 estimator_work 单次 **775,424 cycles / 预算 134,400**
    //     （超 5.8× ✗）、漏拍 **62/63** ✗ ⇒ 实际 ~23 Hz（声明 200 Hz ✗）⇒ 积分不足
    //     ⇒ 观测全拒 ⇒ 全链失控 ✓。热点正是这里的 Joseph 形：**两次完整 21×21 矩阵乘
    //     ≈ 18,500 MAC** ✗。而 `k`(K) 与 `s_mat`(S) **都已经算好了** ⇒ 这两次大乘是纯浪费 ✗。
    //   本形：KS = K·S（21×3·3×3 = 189）⇒ (KS)·Kᵀ（21×3·21 = 1,323）⇒ **≈1,300 MAC** ✓
    //     ⇒ 该步 **~14×** ✓。数学上等价 ✓（Joseph 仅在浮点稳定性上更优 ✗）；
    //     fcalg 的 131 条判据（正定性 / NIS 一致性 / 有限性闸 / 鉴别力 ✓）负责验退化 ✓。
    //   只算上三角 + 镜像 ⇒ 保持**逐位对称** ✓（与 ③ 同一约定 ✓）。
    let mut ks = [[0.0f32; 3]; N];
    for i in 0..N {
        for a in 0..3 {
            let mut acc = 0.0f32;
            for b in 0..3 {
                acc += k[i][b] * s_mat[b][a];
            }
            ks[i][a] = acc;
        }
    }
    for i in 0..N {
        for j in 0..=i {
            let mut acc = 0.0f32;
            for a in 0..3 {
                acc += ks[i][a] * k[j][a];
            }
            let v = p[i][j] - acc;
            out_p[i][j] = v;
            out_p[j][i] = v;
        }
    }
    for row in out_p.iter() {
        gate_all(Stage::L8Update, row).map_err(UpdateError::NonFinite)?;
    }
    if has_negative_variance(out_p) {
        return Err(UpdateError::NotPositiveDefinite);
    }
    Ok(nis_sigma)
}

/// **按值版本**（薄包装）—— 保留给调用方与既有测试 ✓（零波及）。
pub fn update(
    p: &Cov,
    h: &[[f32; N]; 3],
    resid: &[f32; 3],
    r: &[[f32; 3]; 3],
    gate_sigma: f32,
) -> Result<UpdateOut, UpdateError> {
    let mut outp = [[0.0f32; N]; N];
    let mut dx = [0.0f32; N];
    let nis_sigma = update_into(p, h, resid, r, gate_sigma, &mut outp, &mut dx)?;
    Ok(UpdateOut { p: outp, dx, nis_sigma })
}
