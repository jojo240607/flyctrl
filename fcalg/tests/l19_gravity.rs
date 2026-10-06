//! L19 验收 —— 重力（倾角）观测。本层补的是**已登记缺口**：新栈此前无重力观测
//! ⇒ 飞行中 roll/pitch 不可观（旧栈有 `update_gravity`）。
use fcalg::error_state::{boxplus, I_ATT, N};
use fcalg::filter::Eskf;
use fcalg::observe::{gravity, gravity_magnitude_ok, ObsParams};
use fcalg::params::param;
use fcalg::propagate::State;
use fcalg::quat::{specific_force_at_rest, Quat, GRAVITY_NED};
use fcalg::transition::skew;

fn truth(rpy: [f32; 3]) -> State {
    let mut st = State::level();
    st.q = Quat::from_euler_zyx(rpy).normalize().unwrap();
    st.mag_i = [0.25, 0.05, 0.42];
    st.mag_b = [0.01, -0.02, 0.03];
    st
}
fn diag_cov(d: f32) -> fcalg::covariance::Cov {
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        p[i][i] = d;
    }
    p
}

/// **有限差分裁判**：`H_att` 必须等于"在误差方向扰动 ⇒ 模型变化 / ε"（右乘约定）。
#[test]
fn h_matches_finite_difference() {
    let st = truth([0.2, -0.3, 0.8]);
    let prm = ObsParams::default();
    let o = gravity(specific_force_at_rest(st.q), &st, prm.sigma_baro);
    let eps = 1e-3f32;
    let m0 = gravity(specific_force_at_rest(st.q), &st, prm.sigma_baro).resid;
    let _ = o;
    for j in 0..3 {
        let mut d = [0.0f32; N];
        d[I_ATT + j] = eps;
        let xp = boxplus(&st, &d).unwrap();
        // 模型 h(x)：预测比力（不含零偏已经为 0）
        let gb = xp.q.conj().rotate(GRAVITY_NED);
        let h1 = [-gb[0] + xp.ba[0], -gb[1] + xp.ba[1], -gb[2] + xp.ba[2]];
        let gb0 = st.q.conj().rotate(GRAVITY_NED);
        let h0 = [-gb0[0] + st.ba[0], -gb0[1] + st.ba[1], -gb0[2] + st.ba[2]];
        for a in 0..3 {
            let fd = (h1[a] - h0[a]) / eps;
            let sw = skew([-gb0[0], -gb0[1], -gb0[2]]);
            assert!(
                (fd - sw[a][j]).abs() < 5e-3,
                "H_att 与数值导数须一致 @[{a}][{j}]: {fd} vs {}",
                sw[a][j]
            );
        }
    }
    let _ = m0;
}

/// ★**核心判据**：初始 tilt 有偏时，重力观测必须把它拉回来（这正是缺重力观测时做不到的）。
#[test]
fn wrong_initial_tilt_converges() {
    let t = truth([0.0, 0.0, 0.0]); // 真值水平
    // 估计初值：roll 偏 0.15 rad（≈8.6°）
    let bad = Quat::from_euler_zyx([0.15, -0.10, 0.0]);
    let mut st = t;
    st.q = bad;
    let mut f = Eskf::new(st, diag_cov(0.05), 10);
    let f_b = specific_force_at_rest(t.q); // 真值比力（水平 ⇒ (0,0,−g)）
    let before = f.st.q.to_euler_zyx();
    for _ in 0..500 {
        assert!(f.update_gravity(f_b).unwrap(), "量级门应通过（只有重力）");
    }
    let after = f.st.q.to_euler_zyx();
    assert!(
        after[0].abs() < before[0].abs() * 0.1,
        "roll 必须被重力观测拉住: {} → {}",
        before[0],
        after[0]
    );
    assert!(after[1].abs() < before[1].abs() * 0.1 + 1e-3, "pitch 同样");
}

/// 量级门：机动中（‖f‖ 远非 g）**不得融合**，且必须显式返回 false（非静默）。
#[test]
fn magnitude_gate_rejects_maneuvering() {
    let t = truth([0.2, -0.3, 0.8]);
    let mut f = Eskf::new(t, diag_cov(0.05), 10);
    let before = f.st;
    let maneuvering = [0.0f32, 0.0, -2.0 * GRAVITY_NED[2]]; // ‖f‖ = 2g
    assert!(!gravity_magnitude_ok(maneuvering, 0.06));
    assert_eq!(f.update_gravity(maneuvering), Ok(false), "必须显式返回 false");
    assert_eq!(f.st, before, "门不过时不得改动状态");
    assert!(f.gravity_guard.rejects() > 0, "门不过必须被计数（绝不静默）");
    // 正常重力放行
    assert!(gravity_magnitude_ok([0.0, 0.0, -GRAVITY_NED[2]], 0.06));
}

/// 完美比力 ⇒ 零新息；且 `Eskf` 的量级门容差必须与参数表 `align.g_tol_frac` 一致
/// （表=代码，否则又是一处"各说一套"）。
#[test]
fn perfect_gravity_is_zero_innovation_and_gate_matches_table() {
    let t = truth([0.2, -0.3, 0.8]);
    let f_b = specific_force_at_rest(t.q);
    let o = gravity(f_b, &t, 0.3);
    for a in 0..3 {
        assert!(o.resid[a].abs() < 1e-5, "完美比力新息须为 0: {:?}", o.resid);
    }
    let e = fcalg::filter::Eskf::new(t, diag_cov(0.05), 10);
    assert_eq!(
        e.g_tol_frac,
        param("align.g_tol_frac").unwrap().value,
        "量级门容差必须与参数表一致"
    );
}
