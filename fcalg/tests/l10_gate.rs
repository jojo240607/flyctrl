//! L10 验收 —— 核心是**行为**判据：方差塌陷后，**重灌必须真的让观测重新拉得动状态**。
//! 这正是旧栈缺那一环的直接后果（Pzz 塌到 1e-6 ⇒ 增益≈0 ⇒ 永久失去可观测性）。
use fcalg::covariance::Cov;
use fcalg::error_state::{I_ATT, I_POS, I_VEL, N};
use fcalg::finite::{reset, violations, Stage, Violation};
use fcalg::gate::{channel_indices, reflate_diag, Channel, ChannelGuard};
use fcalg::observe::{baro, ObsParams};
use fcalg::propagate::State;
use fcalg::update::{update, UpdateError};
/// 人为造出"高空差 + 方差塌到地板"的旧栈式困境。
fn collapsed_state() -> (State, Cov) {
    let mut st = State::level();
    st.p[2] = -50.0; // 估计高度 50 m
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        p[i][i] = 0.05;
    }
    p[I_POS + 2][I_POS + 2] = 1e-6; // 旧栈的"地板值"
    (st, p)
}
/// **行为判据**：不重灌 ⇒ 观测拉不动（修正≈0）；重灌 ⇒ 立刻拉得动。
#[test]
fn recovery_makes_re_anchoring_possible() {
    let (st, mut p) = collapsed_state();
    let prm = ObsParams::default(); // sigma_baro = 0.3 ⇒ R = 0.09
    let o = baro(0.0, &st, &prm); // 真实高度 0 m，估计 −p_z = 50 m
    let before = update(&p, &o.h, &o.resid, &o.r, 1e9).unwrap();
    let dx_frozen = before.dx[I_POS + 2].abs();
    assert!(
        dx_frozen < 1e-3,
        "未重灌时必须拉不动（增益≈0）: |dx|={dx_frozen}"
    );
    // 重灌：把该通道能观测的方差抬到下界
    let n = reflate_diag(&mut p, channel_indices(Channel::Baro), 1.0).unwrap();
    assert_eq!(n, 1, "应抬升 1 项（高度方差）");
    let after = update(&p, &o.h, &o.resid, &o.r, 1e9).unwrap();
    let dx_ok = after.dx[I_POS + 2].abs();
    assert!(
        dx_ok > 1e-2 && dx_ok > 100.0 * dx_frozen,
        "重灌后必须重新拉得动: |dx|={dx_ok}（重灌前 {dx_frozen}）"
    );
    // 修正方向：NED 下 z 向下为正 ⇒ "高 50 m" 是 `p_z = −50`，真值高度 0 ⇒ `p_z = 0`
    // ⇒ 修正应使 `p_z` **增大**（−50 → 0）⇒ `dx > 0`。
    //（首版我写成 `dx < 0`，是拿"向下"的直觉套 NED —— 契约 §2 把这一层约定写明就是为了避免这个。）
    assert!(after.dx[I_POS + 2] > 0.0, "修正方向必须朝真值（NED: −50 → 0 ⇒ 正）: {}", after.dx[I_POS + 2]);
}
/// 重灌**只升不降**：已高于下界的项不动、低于的抬上来，并如实计数。
#[test]
fn reflate_only_raises_and_counts() {
    let (_, mut p) = collapsed_state();
    p[I_POS][I_POS] = 5.0; // 已高于下界
    let n = reflate_diag(&mut p, channel_indices(Channel::GpsPos), 1.0).unwrap();
    assert_eq!(n, 2, "位置三轴中只有两个低于下界");
    assert_eq!(p[I_POS][I_POS], 5.0, "高于下界的项不得被改动");
    assert_eq!(p[I_POS + 2][I_POS + 2], 1.0);
    assert_eq!(p[I_POS + 1][I_POS + 1], 1.0);
}
/// 非有限方差 ⇒ **显式 Err + 计数**（绝不静默变地板 —— 与旧栈的分界线）。
#[test]
fn reflate_rejects_non_finite() {
    reset();
    let (_, mut p) = collapsed_state();
    p[I_POS + 2][I_POS + 2] = f32::NAN;
    assert_eq!(
        reflate_diag(&mut p, channel_indices(Channel::Baro), 1.0),
        Err(Violation::Nan)
    );
    assert!(violations(Stage::L10Gate) > 0);
    assert!(p[I_POS + 2][I_POS + 2].is_nan(), "不得被静默改写");
}
/// 健康时不重灌（有效融合会复位计数）。
#[test]
fn no_recovery_while_healthy() {
    let mut g = ChannelGuard::new(10);
    for _ in 0..100 {
        assert!(!g.rejected(), "未达阈值不得重灌");
        if g.rejects() >= 5 {
            g.accepted();
        }
    }
    assert_eq!(g.recoveries(), 0, "有效融合持续复位 ⇒ 一次也不该重灌");
}
/// 阈值语义：第 N 次拒收触发，**每回合恰一次**；`accepted()` 后重新计数。
#[test]
fn recovery_triggers_once_per_episode() {
    let mut g = ChannelGuard::new(10);
    for i in 1..10 {
        assert!(!g.rejected(), "第 {i} 次拒收（未达 10）不得触发");
    }
    assert!(g.rejected(), "第 10 次必须触发");
    for i in 11..40 {
        assert!(!g.rejected(), "第 {i} 次不得重复触发（否则退化成永不收敛）");
    }
    assert_eq!(g.recoveries(), 1);
    g.accepted();
    assert_eq!(g.rejects(), 0);
    for _ in 0..9 {
        let _ = g.rejected();
    }
    assert!(g.rejected(), "新回合的第 10 次必须再次触发");
    assert_eq!(g.recoveries(), 2);
}
/// 结构不变量：通道→索引映射必须正确（复制粘贴极易出错）。
#[test]
fn channel_indices_are_correct() {
    assert_eq!(channel_indices(Channel::Baro), &[I_POS + 2]);
    assert_eq!(channel_indices(Channel::GpsPos), &[I_POS, I_POS + 1, I_POS + 2]);
    assert_eq!(channel_indices(Channel::GpsVel), &[I_VEL, I_VEL + 1, I_VEL + 2]);
    let m = channel_indices(Channel::MagYaw);
    assert!(m.contains(&I_ATT) && m.len() == 9);
    assert!(!m.contains(&I_POS), "磁航向不该重灌位置方差");
    // 与 L8 的 Err 语义对偶：拒收走 Rejected，而不是静默
    let (st, p) = collapsed_state();
    let prm = ObsParams::default();
    let o = baro(0.0, &st, &prm);
    assert!(matches!(
        update(&p, &o.h, &o.resid, &o.r, 1e-3),
        Err(UpdateError::Rejected { .. })
    ));
}
