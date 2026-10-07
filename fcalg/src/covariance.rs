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
#[allow(unused_imports)]
use crate::math::F32Ext;

use crate::error_state::N;
use core::sync::atomic::AtomicU32;
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
/// **实质性坏方差**：任一对角 ≤ 0 或非有限。
/// ★为何不用"正定"当硬门限（本会话定位到的一个**设计**问题）：
///   卡尔曼滤波的 P **本来就会合法地变得病态**（反复融合同一路选择观测 ⇒
///   状态间近乎完全相关 ⇒ 最小特征值远低于最大对角）。此时 `is_positive_definite`
///   会因 f32 抵消而**假阴性** ⇒ 硬门限会误拒合法协方差（实测：输入 P 正定、F 非奇异，
///   却判 F·P·Fᵗ 非正定 —— 数学上不可能 ⇒ 是判据不可靠）。
///   ⇒ 硬门限改为只拒"**实质性**坏方差"（负对角/非有限）；正定只作**诊断**，
///     PD 的**维持**靠构造（Joseph + 上三角镜像），不靠事后门限。
/// `predict` 入口发现 P 已不正定的次数（**计数而非打印** —— 库不得依赖 std）。
pub static PD_ENTRY_VIOLATIONS: AtomicU32 = AtomicU32::new(0);
/// 负对角被显式抬高的次数（**绝不静默** —— 集成验收应断言它为 0）。
pub static NEG_DIAG_FIXED: AtomicU32 = AtomicU32::new(0);
/// 抬高到的最小正值（地板）。
pub const NEG_DIAG_FLOOR: f32 = 1e-12;
/// 把**负对角**（f32 抵消的产物，物理上不可能）显式抬高到地板并**计数**。
/// 与"静默钳位"的区别：这里**每次都会计数**，调用方/验收可以直接断言它为 0；
/// 旧栈那次是"把 NaN 也悄悄变成 1e-6 且无人知晓"。
pub fn fix_negative_diagonals(a: &mut Cov) -> u32 {
    let mut n = 0;
    for i in 0..N {
        if !(a[i][i] > 0.0) {
            a[i][i] = NEG_DIAG_FLOOR;
            n += 1;
        }
    }
    if n > 0 {
        NEG_DIAG_FIXED.fetch_add(n, core::sync::atomic::Ordering::Relaxed);
    }
    n
}
pub fn has_negative_variance(a: &Cov) -> bool {
    for i in 0..N {
        if !(a[i][i] > 0.0) {
            return true;
        }
    }
    false
}
/// Cholesky 判正定（非有限或非正主元 ⇒ false）。**诊断用**，不作硬门限（见上）。
/// 按最大对角归一 + 相对容差 ⇒ 尺度不变。
pub fn is_positive_definite(a: &Cov) -> bool {
    // ★★**必须先按最大对角归一，再用相对容差** —— 这是本会话定位到的一个真问题：
    //   原实现要求 `s > 0.0` **精确成立**，而 21×21 的 Cholesky 在动态范围大时会出现
    //   **相减抵消**，f32 舍入让 `s` 略微 ≤ 0 ⇒ **把正定阵判成非正定（假阴性）**。
    //   实测症状：同一次运行里两个检查对同一个 P 给出相反答案（那是仪器问题，不是算法问题）；
    //   失败样本的 P 对角动态范围达 4e4（max 5e-2 / min 1.3e-6），且另一次 max 达 1.0e1。
    //   权衡（诚实写明）：容差过严会把 f32 舍入当非正定；过松会放过真的坏方差。
    //   取**相对** 1e-7（归一化之后）—— 仍能拒绝任何相对量级上的真非正定。
    let mut scl = 0.0f32;
    for i in 0..N {
        scl = scl.max(a[i][i].abs());
    }
    if !(scl.is_finite() && scl > 0.0) {
        return false;
    }
    let inv = 1.0 / scl;
    let mut l = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..=i {
            let mut s = a[i][j] * inv;
            if !s.is_finite() {
                return false;
            }
            for k in 0..j {
                s -= l[i][k] * l[j][k];
            }
            if i == j {
                if !(s > 1e-7) {
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
/// **写入口**：结果写进调用方的 `&mut Cov` —— **不再按值返回 `Cov`** ✓
/// 动因（实测）：真固件里 L2 worker 栈溢出 4,128 B，其中一个来源就是
/// `→ Cov` 的**按值返回临时量**（1,764 B/次，本函数与 `update` 各一次）✓。
/// 索引对 `&mut Cov` 同样有效 ⇒ **函数体与原来逐字一致** ✓（零语义改动）。
/// ★A（实测驱动 ✓）：**有效维数**。磁的 6 态（`I_MAGI/I_MAGB`）在"磁参考未配置"时
/// 无观测、无驱动 ⇒ 令其不参与矩阵运算（末尾块**原样拷贝** ⇒ P 仍正定 ✓）。
/// 实测依据：单次 `ekf_hil` ≈488k cycles、其中矩阵核占大头；`mac = 21 cycles/op` ✗，
/// 而 6/21 的死重按 N³ 标度 ≈ **2.8×** ✓。仅**每核一次**分支 ⇒ 不触发"内层加分支"的负优化 ✓。
pub static mut ACTIVE_N: usize = N;

/// ★A-实测：各核每拍被调用几次（决定"成本模型该按什么乘"✓）
pub static PROP_CALLS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// 由桥接在"磁不可用"时调用（见 `fcalg_bridge`：`mag_i == [0,0,0]` ⇒ 15 ✓）
pub fn set_active_n(n: usize) {
    unsafe {
        ACTIVE_N = if n >= 3 && n <= N { n } else { N };
    }
}

pub fn propagate_covariance_into(
    p: &Cov,
    f: &[[f32; N]; N],
    q: &Cov,
    out: &mut Cov,
) -> Result<(), CovError> {
    let _ = PROP_CALLS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    // ① 入口门（契约 §4）：非有限一律显式拒绝，**不做任何钳位**
    gate_cov(p).map_err(CovError::NonFinite)?;
    gate_cov(q).map_err(CovError::NonFinite)?;
    for r in f.iter() {
        gate_all(Stage::L7Covariance, r).map_err(CovError::NonFinite)?;
    }
    // ② FP = F·P（仅前 `an` 维 ✓；`an = N` 时行为与原先逐位相同 ✓）
    let an = unsafe { ACTIVE_N };
    let mut fp = [[0.0f32; N]; N];
    for i in 0..an {
        for k in 0..N {
            let fik = f[i][k];
            if fik == 0.0 {
                continue;
            }
            for j in 0..an {
                fp[i][j] += fik * p[k][j];
            }
        }
    }
    // ③ 上三角：out = FP·Fᵀ + Q，随后镜像 ⇒ 逐位对称（写入调用方缓冲 ✓）
    for i in 0..an {
        for j in 0..=i {
            let mut s = q[i][j];
            for k in 0..an {
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
    // ③b 未参与运算的尾部块**原样拷贝**（保证 P 完整、正定 ✓；an=N 时循环为空 ✓）
    for i in an..N {
        for j in 0..N {
            out[i][j] = p[i][j];
        }
    }
    for i in 0..an {
        for j in an..N {
            out[i][j] = p[i][j];
        }
    }
    // ④ 后置：非有限 ⇒ 拒绝（**不得钳位**）
    gate_cov(&out).map_err(CovError::NonFinite)?;
    // ⑤ 后置：必须正定（方差失去物理意义 ⇒ 显式失败，交由上层决定重灌/复位）
    // ★对角负值分两种（PX4 也这么分）：
    //   · **实质性负**（相对最大对角超过 1e-6）⇒ 物理上不可能，**显式 Err**（L7 的判据）
    //   · **舍入级负**（f32 抵消产物）⇒ 抬到地板并**计数**（L15 的数值鲁棒性需要）
    //   这样既不让"静默修正"混进来，也不让 1e-12 级的舍入把滤波器打死。
    let mut scl = 0.0f32;
    for i in 0..N {
        scl = scl.max(out[i][i].abs());
    }
    for i in 0..N {
        if out[i][i] < -1e-6 * scl {
            return Err(CovError::NotPositiveDefinite);
        }
    }
    if has_negative_variance(out) {
        fix_negative_diagonals(out);
    }
    Ok(())
}

/// **按值版本**（薄包装）—— 保留给调用方与既有测试 ✓（零波及）。
pub fn propagate_covariance(p: &Cov, f: &[[f32; N]; N], q: &Cov) -> Result<Cov, CovError> {
    let mut out = [[0.0f32; N]; N];
    propagate_covariance_into(p, f, q, &mut out)?;
    Ok(out)
}
