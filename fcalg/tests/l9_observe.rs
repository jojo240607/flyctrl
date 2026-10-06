//! L9 验收 —— 每路 H 都过**有限差分裁判**（判 H 与量测模型自洽）、
//! 「无信息」编码为对角大 R（非对角是错的）、以及观测只作用在它该作用的轴上。
use fcalg::covariance::Cov;
use fcalg::error_state::{boxplus, I_POS, I_VEL, N};
use fcalg::observe::{baro, gps_pos, gps_vel, Obs, ObsParams, NO_INFO};
use fcalg::propagate::State;
use fcalg::quat::Quat;
use fcalg::update::update;
fn base_state() -> State {
    let mut st = State::level();
    st.q = Quat::from_euler_zyx([0.2, -0.3, 0.7]).normalize().unwrap();
    st.p = [3.0, -1.5, -2.0];
    st.v = [0.5, 0.25, -0.1];
    st
}
fn pd_cov() -> Cov {
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..N {
            p[i][j] = if i == j { 0.05 } else { 1e-5 };
        }
    }
    p
}
/// 对给定观测做 H 的有限差分裁判：`H[:,j] == (m(x ⊞ εe_j) − m(x))/ε`。
fn fd_judge<F: Fn(&State) -> [f32; 3]>(o: &Obs, m: F, cols: &[usize]) {
    let x = base_state();
    let m0 = m(&x);
    let eps = 1e-3f32;
    for &j in cols {
        let mut d = [0.0f32; N];
        d[j] = eps;
        let xp = boxplus(&x, &d).unwrap();
        let m1 = m(&xp);
        for a in 0..3 {
            let fd = (m1[a] - m0[a]) / eps;
            assert!((fd - o.h[a][j]).abs() < 1e-3, "H 与数值导数须一致 @[{a}][{j}]: {fd} vs {}", o.h[a][j]);
        }
    }
}
/// baro 的 H 必须与模型 `alt == −p_z` 自洽（**符号**是关键）。
#[test]
fn baro_h_matches_altitude_model() {
    let st = base_state();
    let o = baro(-st.p[2] + 0.4, &st, &ObsParams::default());
    fd_judge(&o, |x| [-x.p[2], 0.0, 0.0], &[I_POS + 2]);
    assert!((o.resid[0] - 0.4).abs() < 1e-6, "新息须 = 量测 − 预测: {}", o.resid[0]);
}
/// GPS 位置的 H 必须与模型 `p` 自洽。
#[test]
fn gps_pos_h_matches_position_model() {
    let st = base_state();
    let o = gps_pos([4.0, -2.0, -1.0], &st, &ObsParams::default());
    fd_judge(&o, |x| x.p, &[I_POS, I_POS + 1, I_POS + 2]);
}
/// GPS 速度的 H 必须与模型 `v` 自洽。
#[test]
fn gps_vel_h_matches_velocity_model() {
    let st = base_state();
    let o = gps_vel([0.6, 0.3, -0.2], &st, &ObsParams::default());
    fd_judge(&o, |x| x.v, &[I_VEL, I_VEL + 1, I_VEL + 2]);
}
/// **硬契约（行为性判据）**：GPS 速度**垂直分量不得被修正** —— RMC 只给水平 Doppler，
/// 垂直分量恒 0 是"没测"而不是"测到 0"。旧栈正是把恒 0 当有效观测 ⇒ 真爬升时估计被死压到 0
/// ⇒ 气压残差增长 ⇒ 门控死锁 ⇒ 失控爬升。故判据必须是**行为**（残差再大也不动），
/// 而不是"H 行是否全零"（本路用 `R[2][2]=NO_INFO` 表达无信息，H 保持自然的选择矩阵）。
#[test]
fn gps_vel_vertical_is_no_information() {
    let st = base_state();
    let o = gps_vel([0.0, 0.0, 99.0], &st, &ObsParams::default());
    assert_eq!(o.r[2][2], NO_INFO, "垂直轴 R 必须为「无信息」");
    let out = update(&pd_cov(), &o.h, &o.resid, &o.r, 1e9).unwrap();
    assert!(
        out.dx[I_VEL + 2].abs() < 1e-2,
        "垂直速度不得被修正（残差 99 m/s 也不得动它）: {}",
        out.dx[I_VEL + 2]
    );
    assert!(out.dx[I_VEL].abs() > 1e-3, "水平轴必须仍被正常修正");
}
/// 「无信息」编码必须是**对角**大 R（非对角在物理上不是"无观测"）。
#[test]
fn no_info_encoding_is_diagonal() {
    let st = base_state();
    let o = baro(0.0, &st, &ObsParams::default());
    assert_eq!((o.r[0][1], o.r[0][2], o.r[1][0], o.r[1][2], o.r[2][0], o.r[2][1]), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    assert_eq!(o.r[1][1], NO_INFO);
}
/// 行为不变量：只用气压观测 ⇒ 只有位置 z 的方差下降，速度与姿态基本不动。
#[test]
fn baro_only_affects_altitude_variance() {
    let st = base_state();
    let p = pd_cov();
    let o = baro(-st.p[2] + 0.05, &st, &ObsParams::default());
    let out = update(&p, &o.h, &o.resid, &o.r, 6.0).unwrap();
    assert!(out.p[I_POS + 2][I_POS + 2] < p[I_POS + 2][I_POS + 2], "高度方差必须下降");
    assert!((out.p[0][0] - p[0][0]).abs() < 1e-4, "姿态方差不应被气压影响");
    for k in 0..3 {
        assert!((out.dx[I_VEL + k]).abs() < 1e-2, "气压观测不应明显推动速度");
    }
}
/// 保守观测（R 极大）⇒ 状态几乎不动（信息量为零的极限）。
#[test]
fn conservative_observation_barely_moves_state() {
    let st = base_state();
    let prm = ObsParams { sigma_baro: 1e4, sigma_gps_p: 1e4, sigma_gps_v: 1e4, sigma_mag: 1e4 };
    let o = baro(-st.p[2] + 100.0, &st, &prm);
    let out = update(&pd_cov(), &o.h, &o.resid, &o.r, 1e9).unwrap();
    assert!(out.dx[I_POS + 2].abs() < 1e-3, "几乎无信息 ⇒ 修正必须极小: {}", out.dx[I_POS + 2]);
}
