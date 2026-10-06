//! L6 验收 —— 核心是**有限差分裁判**：拿解析 F 对"实际传播"的数值导数。
//! 这是独立于实现的判据：它判 F 写得对不对，而不是判它"和上次一样不一样"。
use fcalg::error_state::{boxminus, boxplus, I_ATT, I_BA, I_BG, N};
use fcalg::imu_delta::ImuDelta;
use fcalg::propagate::{propagate, State};
use fcalg::quat::{Quat, GRAVITY_NED};
use fcalg::transition::transition_matrix;
const DT: f32 = 0.01;
fn sample() -> (State, ImuDelta) {
    let mut st = State::level();
    st.q = Quat::from_euler_zyx([0.4, -0.55, 1.1]);
    st.bg = [0.02, -0.01, 0.03];
    st.ba = [0.05, -0.04, 0.02];
    let w = [0.7f32, -0.4, 0.25];
    let f = [0.3f32, -0.2, -9.7];
    let d = ImuDelta {
        delta_ang: [w[0] * DT, w[1] * DT, w[2] * DT],
        delta_vel: [f[0] * DT, f[1] * DT, f[2] * DT],
        dt_ang: DT,
        dt_vel: DT,
        ts_ticks: 0,
    };
    (st, d)
}
fn step(st: &State, d: &ImuDelta) -> State {
    let mut s = *st;
    propagate(&mut s, d, GRAVITY_NED).unwrap();
    s
}
/// **有限差分裁判**：F 的每一列必须等于"在误差方向扰动 ε ⇒ 传播 ⇒ 取差 / ε"。
#[test]
fn f_matches_finite_difference_all_columns() {
    let (x0, d) = sample();
    let xn = step(&x0, &d);
    let w_body = [d.delta_ang[0] / DT - x0.bg[0], d.delta_ang[1] / DT - x0.bg[1], d.delta_ang[2] / DT - x0.bg[2]];
    let f_b = [d.delta_vel[0] / DT - x0.ba[0], d.delta_vel[1] / DT - x0.ba[1], d.delta_vel[2] / DT - x0.ba[2]];
    let f = transition_matrix(x0.q, w_body, f_b, DT);
    let eps = 1e-4f32;
    let mut worst = 0.0f32;
    for j in 0..N {
        let mut dx = [0.0f32; N];
        dx[j] = eps;
        let xp = boxplus(&x0, &dx).unwrap();
        let xpn = step(&xp, &d);
        let col = boxminus(&xn, &xpn).unwrap();
        for i in 0..N {
            let fd = col[i] / eps;
            worst = worst.max((fd - f[i][j]).abs());
        }
    }
    assert!(worst < 5e-3, "解析 F 与数值导数必须一致: 最大偏差 {worst}");
}
/// 符号约定的**判别性**检验：若把 δv-δθ 块的符号写反，FD 裁判必须失败。
/// （旧栈正是靠这类判据发现"符号写反"的 —— 见 `f_vel_att_sign_vs_finite_difference`。）
#[test]
fn fd_judge_discriminates_the_sign() {
    let (x0, d) = sample();
    let xn = step(&x0, &d);
    let w_body = [d.delta_ang[0] / DT - x0.bg[0], d.delta_ang[1] / DT - x0.bg[1], d.delta_ang[2] / DT - x0.bg[2]];
    let f_b = [d.delta_vel[0] / DT - x0.ba[0], d.delta_vel[1] / DT - x0.ba[1], d.delta_vel[2] / DT - x0.ba[2]];
    let f = transition_matrix(x0.q, w_body, f_b, DT);
    let eps = 1e-4f32;
    let j = I_ATT; // 对 δθ 的一列
    let mut dx = [0.0f32; N];
    dx[j] = eps;
    let xpn = step(&boxplus(&x0, &dx).unwrap(), &d);
    let col = boxminus(&xn, &xpn).unwrap();
    let mut worst_ok = 0.0f32;
    let mut worst_flipped = 0.0f32;
    for i in 0..N {
        let fd = col[i] / eps;
        worst_ok = worst_ok.max((fd - f[i][j]).abs());
        worst_flipped = worst_flipped.max((fd + f[i][j]).abs());
    }
    assert!(worst_ok < 5e-3, "F 本身须匹配（偏差 {worst_ok}）");
    assert!(worst_flipped > 0.5, "取反后必须明显不匹配（否则判据无判别力）");
}
/// 结构：F 必须只在声明的块上非单位（多余耦合 = 实现错）。
#[test]
fn f_structure_only_declared_blocks() {
    let (x0, _) = sample();
    let f = transition_matrix(x0.q, [0.5, -0.3, 0.2], [0.1, 0.2, -9.8], DT);
    let allowed = |i: usize, j: usize| -> bool {
        let blk = |a: usize, b: usize| i >= a && i < a + 3 && j >= b && j < b + 3;
        (i == j) || blk(I_ATT, I_ATT) || blk(I_ATT, I_BG) || blk(3, I_ATT) || blk(3, I_BA) || (i >= 6 && i < 9 && j >= 3 && j < 6)
    };
    for i in 0..N {
        for j in 0..N {
            if !allowed(i, j) {
                assert_eq!(f[i][j], 0.0, "F[{i}][{j}] 出现未声明耦合");
            }
        }
    }
}
/// 极限：`dt = 0` ⇒ `F = I`。
#[test]
fn zero_dt_gives_identity() {
    let (x0, _) = sample();
    let f = transition_matrix(x0.q, [1.0, 2.0, 3.0], [4.0, 5.0, 6.0], 0.0);
    for i in 0..N {
        for j in 0..N {
            assert_eq!(f[i][j], if i == j { 1.0 } else { 0.0 });
        }
    }
}
/// 定义式：姿态块必须是 `−[ω×]`（而非 `+`），且 φ-零偏耦合为 `−I`。
#[test]
fn attitude_block_signs_are_definitional() {
    let (x0, _) = sample();
    let w = [0.3f32, -0.5, 0.8];
    let f = transition_matrix(x0.q, w, [0.0; 3], DT);
    // 注意：这是 **F = I + A·dt**，所以姿态块 = I + (−[w×])·dt（**对角线上有 1** ✓）。
    let a_block = [
        [0.0, w[2] * DT, -w[1] * DT],
        [-w[2] * DT, 0.0, w[0] * DT],
        [w[1] * DT, -w[0] * DT, 0.0],
    ];
    for i in 0..3 {
        for j in 0..3 {
            let one = if i == j { 1.0 } else { 0.0 };
            assert!(
                (f[I_ATT + i][I_ATT + j] - (one + a_block[i][j])).abs() < 1e-6,
                "姿态块符号错 @[{i}][{j}]: {} vs {}",
                f[I_ATT + i][I_ATT + j],
                one + a_block[i][j]
            );
            let want = if i == j { -DT } else { 0.0 };
            assert!((f[I_ATT + i][I_BG + j] - want).abs() < 1e-6, "φ-bg 块须为 −I·dt");
        }
    }
}
/// 不变量：`boxminus(x, x ⊞ δ) == δ`（往返），且对姿态是**短弧**。
#[test]
fn boxplus_boxminus_round_trip() {
    let (x0, _) = sample();
    let mut d = [0.0f32; N];
    for (k, v) in d.iter_mut().enumerate() {
        *v = 0.01 * (k as f32 + 1.0);
    }
    d[I_ATT] = 0.2;
    d[I_ATT + 1] = -0.15;
    d[I_ATT + 2] = 0.05;
    let y = boxplus(&x0, &d).unwrap();
    let back = boxminus(&x0, &y).unwrap();
    for k in 0..N {
        assert!((back[k] - d[k]).abs() < 1e-5, "δ 往返失败 @{k}: {} vs {}", back[k], d[k]);
    }
    let _ = I_BA;
}
