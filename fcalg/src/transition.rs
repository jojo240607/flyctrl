//! L6 · 误差状态转移矩阵 F（21×21）
//! 连续时间误差动力学（**右乘**姿态误差 δθ，与 L5 的传播约定一致）：
//!   `δθ̇ = −[ω×]·δθ − δbg`
//!   `δv̇ = −R(q)·[f_b×]·δθ − R(q)·δba`
//!   `δṗ = δv`；零偏与磁两态为常量（导数为 0）
//! 离散：`F = I + A·dt`（一阶；判据由 FD 裁判给出，见 `tests/l6_transition.rs`）。
//! 变量含义：`ω = Δang/dt_ang − bg`（机体，零偏已扣）、`f_b = Δvel/dt_vel − ba`（机体）。
#[allow(unused_imports)]
use crate::math::F32Ext;

use crate::error_state::{I_ATT, I_BA, I_BG, I_POS, I_VEL, N};
use crate::quat::Quat;
/// 反对称阵 `[v×]`（满足 `[v×]u = v × u`）。
#[inline]
pub fn skew(v: [f32; 3]) -> [[f32; 3]; 3] {
    [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
}
/// `R(q)` 的行（`R·x` 的第 i 行 = 该行与 x 点积）。用 `rotate` 的基向量像构造，保证与 L0 同一实现。
fn rot_matrix(q: Quat) -> [[f32; 3]; 3] {
    let c0 = q.rotate([1.0, 0.0, 0.0]);
    let c1 = q.rotate([0.0, 1.0, 0.0]);
    let c2 = q.rotate([0.0, 0.0, 1.0]);
    [[c0[0], c1[0], c2[0]], [c0[1], c1[1], c2[1]], [c0[2], c1[2], c2[2]]]
}
/// 误差状态转移矩阵 `F = I + A·dt`。
pub fn transition_matrix(q: Quat, omega_body: [f32; 3], f_b: [f32; 3], dt: f32) -> [[f32; N]; N] {
    let mut a = [[0.0f32; N]; N];
    let r = rot_matrix(q);
    let sw = skew(omega_body);
    let sf = skew(f_b);
    // δθ̇ = −[ω×]δθ − δbg
    for i in 0..3 {
        for j in 0..3 {
            a[I_ATT + i][I_ATT + j] = -sw[i][j];
            a[I_ATT + i][I_BG + j] = if i == j { -1.0 } else { 0.0 };
        }
    }
    // δv̇ = −R[f×]δθ − R δba
    for i in 0..3 {
        for j in 0..3 {
            let mut s = 0.0f32;
            for k in 0..3 {
                s += r[i][k] * sf[k][j];
            }
            a[I_VEL + i][I_ATT + j] = -s;
            a[I_VEL + i][I_BA + j] = -r[i][j];
        }
    }
    // δṗ = δv
    for i in 0..3 {
        a[I_POS + i][I_VEL + i] = 1.0;
    }
    let mut f = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..N {
            f[i][j] = if i == j { 1.0 } else { 0.0 } + a[i][j] * dt;
        }
    }
    f
}
