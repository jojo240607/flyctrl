//! L8 验收 —— 有限差分裁判（**非平凡 H**：涉及姿态）、可解析合成输入（一维卡尔曼闭式解）、
//! 不变量（对称/正定/信息不增）、以及"拒收在结构上不碰 P"。
use fcalg::covariance::{is_positive_definite, is_symmetric_exact, Cov};
use fcalg::error_state::{boxplus, I_ATT, I_POS, N};
use fcalg::finite::{reset, violations, Stage, Violation};
use fcalg::propagate::State;
use fcalg::quat::Quat;
use fcalg::transition::skew;
use fcalg::update::{update, UpdateError};
fn base_state() -> State {
    let mut st = State::level();
    st.q = Quat::from_euler_zyx([0.25, -0.35, 0.8]).normalize().unwrap();
    st
}
fn pd_cov() -> Cov {
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..N {
            p[i][j] = if i == j { 0.02 } else { 1e-5 };
        }
    }
    p
}
/// **非平凡量测模型**：世界"下方"在机体系的表示 `v_b = R(q)ᵀ·ẑ`（涉及姿态）。
fn model_down_body(x: &State) -> [f32; 3] {
    x.q.conj().rotate([0.0, 0.0, 1.0])
}
/// 该模型的解析 H：因误差态是**右乘**（`q_true = q ⊗ exp(δθ)`），
/// `R_trueᵀ ẑ = exp(−[δθ×])·v ≈ v + [v×]δθ` ⇒ 姿态块 = `[v×]`，其余为 0。
fn h_down_body(x: &State) -> [[f32; N]; 3] {
    let v = model_down_body(x);
    let sw = skew(v);
    let mut h = [[0.0f32; N]; 3];
    for a in 0..3 {
        for b in 0..3 {
            h[a][I_ATT + b] = sw[a][b];
        }
    }
    h
}
/// **有限差分裁判（非平凡 H）**：`H[:,j] == (model(x ⊞ εe_j) − model(x))/ε`。
/// 这一条同时验证了"H 的姿态块符号/约定与 L5/L6 的**右乘**误差态自洽"。
#[test]
fn h_matches_finite_difference_with_attitude_coupling() {
    let x = base_state();
    let h = h_down_body(&x);
    let eps = 1e-3f32;
    let m0 = model_down_body(&x);
    for j in 0..3 {
        let mut d = [0.0f32; N];
        d[I_ATT + j] = eps;
        let xp = boxplus(&x, &d).unwrap();
        let m1 = model_down_body(&xp);
        for a in 0..3 {
            let fd = (m1[a] - m0[a]) / eps;
            assert!(
                (fd - h[a][I_ATT + j]).abs() < 5e-3,
                "H 与数值导数须一致 @[{a}][{j}]: {fd} vs {}",
                h[a][I_ATT + j]
            );
        }
    }
}
/// 可解析合成输入：一维（选择矩阵 H、对角 P/R）时后验必须等于卡尔曼闭式解。
#[test]
fn selection_matrix_matches_scalar_kalman_closed_form() {
    let mut p = pd_cov();
    p[I_POS][I_POS] = 4.0; // 只让 x 分量显著，便于与标量式对照
    let mut h = [[0.0f32; N]; 3];
    h[0][I_POS] = 1.0;
    let rr = 1.0f32;
    // 另两轴用**对角**极大 R = “无信息”。
    // ⚠不可写成巨大**非对角**元 —— 那在物理上是“噪声完全相关”，不是“无观测”，
    //   且会让 S 病态到 f32 下不可逆（本用例初版就是这么写错的，被模块正确拒绝）。
    let r = [[rr, 0.0, 0.0], [0.0, 1e12, 0.0], [0.0, 0.0, 1e12]];
    let meas = 2.5f32;
    let resid = [meas - 0.0, 0.0, 0.0];
    let out = update(&p, &h, &resid, &r, 6.0).unwrap();
    let pv = p[I_POS][I_POS];
    let k = pv / (pv + rr);
    let expect_x = k * meas;
    assert!((out.dx[I_POS] - expect_x).abs() < 1e-3, "dx 须符闭式解: {} vs {}", out.dx[I_POS], expect_x);
    let expect_p = pv * rr / (pv + rr);
    assert!(
        (out.p[I_POS][I_POS] - expect_p).abs() < 1e-2,
        "后验方差须符闭式解: {} vs {}",
        out.p[I_POS][I_POS],
        expect_p
    );
}
/// 不变量：零残差 ⇒ `dx == 0`，但 P 仍必须变小（信息仍被吸收）。
#[test]
fn zero_residual_moves_nothing_but_still_adds_information() {
    let x = base_state();
    let p = pd_cov();
    let h = h_down_body(&x);
    let r = [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]];
    let out = update(&p, &h, &[0.0; 3], &r, 6.0).unwrap();
    for i in 0..N {
        assert_eq!(out.dx[i], 0.0, "零残差不得产生修正 @{i}");
    }
    assert!(out.p[I_ATT][I_ATT] < p[I_ATT][I_ATT], "观测必须降低方差");
}
/// 不变量：更新不得增大任何对角方差（信息只能增加）。
#[test]
fn update_never_increases_variance() {
    let x = base_state();
    let p = pd_cov();
    let h = h_down_body(&x);
    let r = [[1e-3, 0.0, 0.0], [0.0, 1e-3, 0.0], [0.0, 0.0, 1e-3]];
    let out = update(&p, &h, &[0.0; 3], &r, 6.0).unwrap();
    for i in 0..N {
        assert!(
            out.p[i][i] <= p[i][i] + 1e-6,
            "更新不得增大方差 @{i}: {} > {}",
            out.p[i][i],
            p[i][i]
        );
    }
}
/// 不变量：结果逐位对称且正定（Joseph 形式的意义所在）。
#[test]
fn joseph_result_is_symmetric_and_positive_definite() {
    let x = base_state();
    let p = pd_cov();
    let h = h_down_body(&x);
    let r = [[0.05, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.05]];
    let out = update(&p, &h, &[0.02, -0.01, 0.03], &r, 6.0).unwrap();
    assert!(is_symmetric_exact(&out.p));
    assert!(is_positive_definite(&out.p));
}
/// 门控：新息远超门限 ⇒ **拒收**（`Rejected`），且 P 逐位不变（函数式 ⇒ 结构上不可能被改）。
#[test]
fn nis_gate_rejects_and_p_is_untouched() {
    let x = base_state();
    let p = pd_cov();
    let before = p;
    let h = h_down_body(&x);
    let r = [[1e-4, 0.0, 0.0], [0.0, 1e-4, 0.0], [0.0, 0.0, 1e-4]];
    let res = update(&p, &h, &[5.0, 5.0, 5.0], &r, 3.0);
    match res {
        Err(UpdateError::Rejected { nis_sigma }) => assert!(nis_sigma > 3.0),
        other => panic!("必须被门限拒收，实际 {other:?}"),
    }
    assert_eq!(p, before, "拒收时 P 必须逐位不变");
}
/// 纪律：非有限 ⇒ `Err` 且计数（绝不静默钳位）。
#[test]
fn non_finite_inputs_are_rejected_and_counted() {
    reset();
    let x = base_state();
    let mut h = h_down_body(&x);
    h[0][I_ATT] = f32::NAN;
    let r = [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]];
    assert_eq!(
        update(&pd_cov(), &h, &[0.0; 3], &r, 6.0),
        Err(UpdateError::NonFinite(Violation::Nan))
    );
    assert!(violations(Stage::L8Update) > 0);
}
