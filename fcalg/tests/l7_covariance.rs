//! L7 验收 —— 只用契约 §6 允许的判据：不变量（对称/正定）、极限退化（F=I、Q=0）、
//! 定义式（Q 的可加性）、以及"非有限/非正定必须显式拒绝，绝不静默钳位"。
use fcalg::covariance::{is_positive_definite, is_symmetric_exact, propagate_covariance, Cov, CovError};
use fcalg::error_state::N;
use fcalg::finite::{reset, violations, Stage, Violation};
use fcalg::quat::Quat;
use fcalg::transition::transition_matrix;
fn eye() -> [[f32; N]; N] {
    let mut m = [[0.0f32; N]; N];
    for i in 0..N {
        m[i][i] = 1.0;
    }
    m
}
/// 严格对角占优 ⇒ 必然正定的输入（用于把"正定"这一维隔离出来）。
fn pd_cov(scale: f32) -> Cov {
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..N {
            p[i][j] = if i == j { 0.01 * (i as f32 + 1.0) * scale } else { 1e-4 * scale };
        }
    }
    p
}
fn zeros() -> Cov {
    [[0.0f32; N]; N]
}
/// 非平凡的 F（来自 L6，含转动耦合）—— 跨模块：F 必须保持 P 正定。
fn f_real() -> [[f32; N]; N] {
    let q = Quat::from_euler_zyx([0.3, -0.4, 0.9]).normalize().unwrap();
    transition_matrix(q, [0.7, -0.3, 0.2], [0.2, -0.1, -9.7], 0.005)
}
/// 极限：`F = I` ⇒ `P' = P + Q`。
#[test]
fn identity_f_gives_p_plus_q() {
    let p = pd_cov(1.0);
    let q = pd_cov(0.5);
    let out = propagate_covariance(&p, &eye(), &q).unwrap();
    for i in 0..N {
        for j in 0..N {
            assert!((out[i][j] - (p[i][j] + q[i][j])).abs() < 1e-6, "F=I 时须为 P+Q @[{i}][{j}]");
        }
    }
}
/// 定义式：Q 可加 —— `P'(p,f,Q) == P'(p,f,0) + Q`（参考无关，且能抓到 Q 位置写错）。
#[test]
fn q_term_is_exactly_additive() {
    let p = pd_cov(1.0);
    let q = pd_cov(0.5);
    let f = f_real();
    let with_q = propagate_covariance(&p, &f, &q).unwrap();
    let without = propagate_covariance(&p, &f, &zeros()).unwrap();
    for i in 0..N {
        for j in 0..N {
            let want = without[i][j] + q[i][j];
            assert!((with_q[i][j] - want).abs() < 1e-5, "Q 必须可加 @[{i}][{j}]");
        }
    }
}
/// 不变量：结果必须**逐位对称**（上三角+镜像的构造保证）。
#[test]
fn result_is_exactly_symmetric() {
    let out = propagate_covariance(&pd_cov(1.0), &f_real(), &pd_cov(0.25)).unwrap();
    assert!(is_symmetric_exact(&out), "协方差结果必须逐位对称");
}
/// 不变量：正定输入 ⇒ 正定输出（F 非奇异、Q 正定）。
#[test]
fn positivity_is_preserved() {
    let p = pd_cov(1.0);
    assert!(is_positive_definite(&p), "测试输入本身须正定");
    let out = propagate_covariance(&p, &f_real(), &pd_cov(0.1)).unwrap();
    assert!(is_positive_definite(&out), "传播后必须仍正定");
}
/// 极限：`Q = 0` 且 `F = I` ⇒ 原地不动。
#[test]
fn zero_q_and_identity_is_noop() {
    let p = pd_cov(1.0);
    let out = propagate_covariance(&p, &eye(), &zeros()).unwrap();
    for i in 0..N {
        for j in 0..N {
            assert_eq!(out[i][j], p[i][j]);
        }
    }
}
/// **针对旧栈缺陷**：非有限输入必须**显式拒绝**，绝不能像旧钳位那样变成 `1e-6`。
#[test]
fn non_finite_is_rejected_never_floored() {
    reset();
    let mut p = pd_cov(1.0);
    p[3][3] = f32::NAN;
    assert_eq!(
        propagate_covariance(&p, &eye(), &pd_cov(0.1)),
        Err(CovError::NonFinite(Violation::Nan))
    );
    assert!(violations(Stage::L7Covariance) > 0, "违规必须被计数");
    // 反证：返回的绝不是"带地板值的 Ok"
    assert!(propagate_covariance(&p, &eye(), &pd_cov(0.1)).is_err());
}
/// **针对旧栈缺陷**：非正定（负方差）必须显式 `Err`，不得静默修正后继续。
#[test]
fn non_positive_definite_is_rejected_explicitly() {
    let mut bad = pd_cov(1.0);
    bad[5][5] = -1.0;
    assert!(!is_positive_definite(&bad), "负方差必须被判为非正定");
    assert_eq!(
        propagate_covariance(&bad, &eye(), &pd_cov(0.1)),
        Err(CovError::NotPositiveDefinite)
    );
}
