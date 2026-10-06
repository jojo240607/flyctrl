//! L6 · 误差状态（21 维）与 boxplus / boxminus
//! 误差态定义（**右乘**，与 L5 传播的右乘一致）：
//!   `q_true = q_nom ⊗ exp(δθ)`，`v_true = v_nom + δv`，`p_true = p_nom + δp`，其余为加法。
//! 这两个算子是 FD 裁判的前提：没有它们就无法把"解析 F"与"实际传播"对上。
use crate::math;
use crate::propagate::State;
use crate::quat::Quat;
/// 误差状态维数。
pub const N: usize = 21;
pub const I_ATT: usize = 0;
pub const I_VEL: usize = 3;
pub const I_POS: usize = 6;
pub const I_BG: usize = 9;
pub const I_BA: usize = 12;
pub const I_MAGI: usize = 15;
pub const I_MAGB: usize = 18;
/// 旋转向量 ⇒ 四元数（与 L5 同式；集中在此以免两处实现分叉）。
#[inline]
pub fn exp_rot(theta: [f32; 3]) -> Quat {
    let a = math::sqrt(theta[0] * theta[0] + theta[1] * theta[1] + theta[2] * theta[2]);
    if a < 1e-8 {
        return Quat { w: 1.0, x: 0.5 * theta[0], y: 0.5 * theta[1], z: 0.5 * theta[2] };
    }
    let inv = 1.0 / a;
    Quat::from_axis_angle([theta[0] * inv, theta[1] * inv, theta[2] * inv], a)
}
/// 四元数 ⇒ 旋转向量（`log`）。**取短弧**（`w<0` 先整体取反），否则 FD 差分不连续。
#[inline]
pub fn log_rot(mut q: Quat) -> [f32; 3] {
    if q.w < 0.0 {
        q = Quat { w: -q.w, x: -q.x, y: -q.y, z: -q.z };
    }
    let v = [q.x, q.y, q.z];
    let nv = math::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if nv < 1e-8 {
        return [2.0 * v[0], 2.0 * v[1], 2.0 * v[2]];
    }
    let ang = 2.0 * math::atan2(nv, q.w);
    let k = ang / nv;
    [k * v[0], k * v[1], k * v[2]]
}
/// `x ⊞ δ`：把误差加到标称态上（右乘姿态、其余加法）。
pub fn boxplus(x: &State, d: &[f32; N]) -> Option<State> {
    let mut s = *x;
    s.q = x.q.mul(exp_rot([d[I_ATT], d[I_ATT + 1], d[I_ATT + 2]])).normalize()?;
    for k in 0..3 {
        s.v[k] = x.v[k] + d[I_VEL + k];
        s.p[k] = x.p[k] + d[I_POS + k];
        s.bg[k] = x.bg[k] + d[I_BG + k];
        s.ba[k] = x.ba[k] + d[I_BA + k];
        s.mag_i[k] = x.mag_i[k] + d[I_MAGI + k];
        s.mag_b[k] = x.mag_b[k] + d[I_MAGB + k];
    }
    Some(s)
}
/// `y ⊟ x`：求 δ 使 `x ⊞ δ = y`（姿态部分用 `exp(δθ) = conj(x.q) ⊗ y.q`）。
pub fn boxminus(x: &State, y: &State) -> Option<[f32; N]> {
    let mut d = [0.0f32; N];
    let dth = log_rot(x.q.conj().mul(y.q).normalize()?);
    for k in 0..3 {
        d[I_ATT + k] = dth[k];
        d[I_VEL + k] = y.v[k] - x.v[k];
        d[I_POS + k] = y.p[k] - x.p[k];
        d[I_BG + k] = y.bg[k] - x.bg[k];
        d[I_BA + k] = y.ba[k] - x.ba[k];
        d[I_MAGI + k] = y.mag_i[k] - x.mag_i[k];
        d[I_MAGB + k] = y.mag_b[k] - x.mag_b[k];
    }
    Some(d)
}
