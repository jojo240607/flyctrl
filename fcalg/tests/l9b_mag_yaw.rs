//! L9b 验收 —— 有限差分裁判（**三个块**：姿态 / mag_i / mag_b）+ yaw-only 的**行为**判据。
//! 行为判据是这一层的灵魂：**倾角与幅值的变化不得产生航向新息**（那正是 yaw-only 的定义），
//! 而**纯航向误差必须如实反映**。
use fcalg::error_state::{boxplus, I_ATT, I_MAGB, I_MAGI, N};
use fcalg::observe::{mag_yaw, ObsParams, NO_INFO};
use fcalg::propagate::State;
use fcalg::quat::Quat;
fn base() -> State {
    let mut st = State::level();
    st.q = Quat::from_euler_zyx([0.15, -0.25, 0.9]).normalize().unwrap();
    st.mag_i = [0.25, 0.05, 0.42];
    st.mag_b = [0.01, -0.02, 0.03];
    st
}
/// 由真值场 + 零偏构造一个"测得的机体磁场"。
fn meas_from(st: &State, mag_i: [f32; 3]) -> [f32; 3] {
    let m = st.q.conj().rotate(mag_i);
    [m[0] + st.mag_b[0], m[1] + st.mag_b[1], m[2] + st.mag_b[2]]
}
fn nu_of(x: &State, m: [f32; 3]) -> f32 {
    let mb = [m[0] - x.mag_b[0], m[1] - x.mag_b[1], m[2] - x.mag_b[2]];
    let mw = x.q.rotate(mb);
    let dz = mw[1].atan2(mw[0]) - x.mag_i[1].atan2(x.mag_i[0]);
    dz.sin().atan2(dz.cos())
}
/// **有限差分裁判**：因本观测以 ν 定义，故 `H ≡ −∂ν/∂δ`，判据写成
/// `H[j] == −(ν(x⊞εe_j) − ν(x⊖εe_j))/(2ε)`。
/// ⚠**必须用中心差分**：前向差分在 ε=1e-3 下的截断误差 ≈ (ε/2)|f″| ⇒ 与这些块的量级同阶
/// （实测绝对差 0.0003~0.0072、**双号**、与 |值| 同阶 ⇒ 那是**判据的噪声，不是 H 的错**）。
/// 中心差分把截断降到 O(ε²) ⇒ 可把容差收到 1e-3。
/// 教训：判据本身的数值品质也要够格，否则会把“仪器噪声”当成“被测对象的错”。
#[test]
fn h_matches_finite_difference_for_all_three_blocks() {
    let x = base();
    let m = meas_from(&x, x.mag_i);
    let o = mag_yaw(m, &x, 0.3);
    let eps = 1e-3f32;
    let n0 = nu_of(&x, m);
    let cols = [
        I_ATT,
        I_ATT + 1,
        I_ATT + 2,
        I_MAGI,
        I_MAGI + 1,
        I_MAGI + 2,
        I_MAGB,
        I_MAGB + 1,
        I_MAGB + 2,
    ];
    for &j in cols.iter() {
        let mut dp = [0.0f32; N];
        dp[j] = eps;
        let mut dm = [0.0f32; N];
        dm[j] = -eps;
        let np = nu_of(&boxplus(&x, &dp).unwrap(), m);
        let nm = nu_of(&boxplus(&x, &dm).unwrap(), m);
        let fd = -(np - nm) / (2.0 * eps);
        eprintln!(
            "[fd] col={j:2} fd={fd:12.6} mine={:12.6} diff={:12.6} ratio={:.6}",
            o.h[0][j],
            fd - o.h[0][j],
            if o.h[0][j].abs() > 1e-9 { fd / o.h[0][j] } else { f32::NAN }
        );
        assert!(
            (fd - o.h[0][j]).abs() < 1e-3,
            "H 与 −∂ν/∂δ 须一致 @{j}: {fd} vs {}",
            o.h[0][j]
        );
    }
}
/// 行为：**纯航向误差必须如实反映**（绕世界竖直轴转 δψ ⇒ 新息 ≈ δψ）。
#[test]
fn pure_heading_error_shows_up_as_innovation() {
    let x = base();
    for dpsi in [-0.3f32, -0.05, 0.05, 0.3] {
        let yawed = Quat::from_euler_zyx([0.0, 0.0, dpsi]);
        let mag_i_true = yawed.rotate(x.mag_i);
        let m = meas_from(&x, mag_i_true);
        let o = mag_yaw(m, &x, 0.3);
        assert!(
            (o.resid[0] - dpsi).abs() < 2e-3,
            "纯航向误差 {dpsi} 须如实反映: {}",
            o.resid[0]
        );
    }
}
/// 行为：**倾角变化不得产生航向新息**（改 mag_i 的竖直分量 ⇒ ν ≈ 0）。
/// 这条就是 yaw-only 的定义 —— 3D 融合会在这里产生新息，从而把姿态拉偏。
#[test]
fn inclination_change_does_not_produce_heading_innovation() {
    let x = base();
    for dz in [-0.2f32, 0.0, 0.35] {
        let mag_i_true = [x.mag_i[0], x.mag_i[1], x.mag_i[2] + dz];
        let m = meas_from(&x, mag_i_true);
        let o = mag_yaw(m, &x, 0.3);
        assert!(o.resid[0].abs() < 2e-3, "倾角变化 {dz} 不得入航向新息: {}", o.resid[0]);
    }
}
/// 行为：**幅值缩放不得产生航向新息**（航向与场强无关）。
#[test]
fn magnitude_scaling_does_not_produce_heading_innovation() {
    let x = base();
    for k in [1.5f32, 3.0, 0.4] {
        let mag_i_true = [x.mag_i[0] * k, x.mag_i[1] * k, x.mag_i[2] * k];
        let m = meas_from(&x, mag_i_true);
        let o = mag_yaw(m, &x, 0.3);
        assert!(o.resid[0].abs() < 2e-3, "幅值 ×{k} 不得入航向新息: {}", o.resid[0]);
    }
}
/// 「无信息」编码：磁航向只用 0 号轴，另两轴必须是对角 NO_INFO。
#[test]
fn mag_yaw_uses_only_first_axis() {
    let x = base();
    let o = mag_yaw(meas_from(&x, x.mag_i), &x, 0.5);
    assert_eq!(o.r[1][1], NO_INFO);
    assert_eq!(o.r[2][2], NO_INFO);
    assert_eq!((o.r[0][1], o.r[1][0]), (0.0, 0.0));
    let _ = ObsParams::default();
}
