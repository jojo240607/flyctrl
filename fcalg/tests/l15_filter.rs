//! L15 验收 —— **端到端**（合成真值驱动整条链，不经仿真器）：
//! 完美观测保持真值、纯预测=解析自由落体、从错误初值收敛、坏观测被拒且无副作用、
//! 长黑障后**重灌真的重新锚定**（L10 的端到端语义）。
use fcalg::covariance::Cov;
use fcalg::error_state::{I_POS, N};
use fcalg::filter::{Eskf, FilterError};
use fcalg::gate::Channel;
use fcalg::observe::{baro, gps_pos, gps_vel, mag_yaw, ObsParams};
use fcalg::propagate::State;
use fcalg::quat::{specific_force_at_rest, Quat, GRAVITY_NED};
use fcalg::sim::{
    baro_from_truth, gps_pos_from_truth, gps_vel_from_truth, imu_from_truth, mag_yaw_from_truth,
};
const DT: f32 = 0.005;
fn diag_cov(d: f32) -> Cov {
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        p[i][i] = d;
    }
    p
}
fn truth() -> State {
    let mut st = State::level();
    st.q = Quat::from_euler_zyx([0.2, -0.3, 0.8]).normalize().unwrap();
    st.mag_i = [0.25, 0.05, 0.42];
    st.mag_b = [0.01, -0.02, 0.03];
    st
}
fn fuse_all(f: &mut Eskf, t: &State, prm: &ObsParams) {
    let _ = f.fuse(&baro_from_truth(t, prm), Channel::Baro, 1e6, 1.0);
    let _ = f.fuse(&gps_pos_from_truth(t, prm), Channel::GpsPos, 1e6, 1.0);
    let _ = f.fuse(&gps_vel_from_truth(t, prm), Channel::GpsVel, 1e6, 1.0);
    let _ = f.fuse(&mag_yaw_from_truth(t, 0.3), Channel::MagYaw, 1e6, 1.0);
}
/// **端到端主判据**：四路完美观测 ⇒ 估值必须一直贴着真值，不漂不炸。
#[test]
fn perfect_observations_hold_truth() {
    let t = truth();
    let prm = ObsParams::default();
    let mut f = Eskf::new(t, diag_cov(0.05), 10);
    let f_b = specific_force_at_rest(t.q);
    for _ in 0..3000 {
        let d = imu_from_truth(&f.st, [0.0; 3], f_b, DT, DT);
        if let Err(e) = f.predict(&d, GRAVITY_NED) {
            let mut dmin = f32::INFINITY; let mut dmax = f32::NEG_INFINITY;
            for i in 0..N { let v = f.p[i][i]; dmin = dmin.min(v); dmax = dmax.max(v); }
            let p_pd_before = fcalg::covariance::is_positive_definite(&f.p);
            panic!("predict 失败 {e:?} | P 对角 min={dmin:e} max={dmax:e} | **predict 前 P 是否已不正定**={p_pd_before}");
        }
        fuse_all(&mut f, &t, &prm);
    }
    assert!(f.healthy(), "滤波器必须保持有限");
    let dot = (f.st.q.w * t.q.w + f.st.q.x * t.q.x + f.st.q.y * t.q.y + f.st.q.z * t.q.z).abs();
    assert!(dot > 1.0 - 1e-5, "姿态须贴真值: dot={dot}");
    for k in 0..3 {
        assert!(f.st.v[k].abs() < 1e-2, "速度须贴真值: {:?}", f.st.v);
        assert!(f.st.p[k].abs() < 5e-2, "位置须贴真值: {:?}", f.st.p);
    }
}
/// 纯预测（无观测）= 解析自由落体：`v_z = g·t`。
#[test]
fn dead_reckoning_matches_free_fall() {
    let mut f = Eskf::new(State::level(), diag_cov(0.05), 10);
    let n = 200;
    for _ in 0..n {
        let d = imu_from_truth(&f.st, [0.0; 3], [0.0; 3], DT, DT);
        if let Err(e) = f.predict(&d, GRAVITY_NED) {
            let mut dmin = f32::INFINITY; let mut dmax = f32::NEG_INFINITY;
            for i in 0..N { let v = f.p[i][i]; dmin = dmin.min(v); dmax = dmax.max(v); }
            let p_pd_before = fcalg::covariance::is_positive_definite(&f.p);
            panic!("predict 失败 {e:?} | P 对角 min={dmin:e} max={dmax:e} | **predict 前 P 是否已不正定**={p_pd_before}");
        }
    }
    let t = DT * n as f32;
    assert!(
        (f.st.v[2] - GRAVITY_NED[2] * t).abs() < 1e-2,
        "自由落体须与解析一致: {} vs {}",
        f.st.v[2],
        GRAVITY_NED[2] * t
    );
}
/// 从**错误初值**收敛（位置偏 20 m）。
#[test]
fn converges_from_wrong_initial_state() {
    let t = truth();
    let prm = ObsParams::default();
    let mut st0 = t;
    st0.p[0] += 20.0;
    st0.p[2] -= 15.0;
    let mut f = Eskf::new(st0, diag_cov(1.0), 10);
    let f_b = specific_force_at_rest(t.q);
    for _ in 0..2000 {
        let d = imu_from_truth(&f.st, [0.0; 3], f_b, DT, DT);
        if let Err(e) = f.predict(&d, GRAVITY_NED) {
            let mut dmin = f32::INFINITY; let mut dmax = f32::NEG_INFINITY;
            for i in 0..N { let v = f.p[i][i]; dmin = dmin.min(v); dmax = dmax.max(v); }
            let p_pd_before = fcalg::covariance::is_positive_definite(&f.p);
            panic!("predict 失败 {e:?} | P 对角 min={dmin:e} max={dmax:e} | **predict 前 P 是否已不正定**={p_pd_before}");
        }
        let _ = f.fuse(&gps_pos_from_truth(&t, &prm), Channel::GpsPos, 1e6, 1.0);
        let _ = f.fuse(&baro_from_truth(&t, &prm), Channel::Baro, 1e6, 1.0);
    }
    assert!(
        (f.st.p[0]).abs() < 0.5 && (f.st.p[2]).abs() < 0.5,
        "必须从错误初值收敛: {:?}",
        f.st.p
    );
}
/// 坏观测被门限拒收，且**对状态没有任何副作用**（L8 的函数式提交）。
#[test]
fn bad_observation_is_rejected_without_side_effect() {
    let t = truth();
    let prm = ObsParams::default();
    let mut f = Eskf::new(t, diag_cov(0.05), 10);
    let before_st = f.st;
    let before_p = f.p;
    let mut bad = gps_pos(t.p, &t, &prm);
    bad.resid = [1e6, -1e6, 1e6]; // 荒谬新息
    match f.fuse(&bad, Channel::GpsPos, 3.0, 1.0) {
        Err(FilterError::Update(_)) => {}
        other => panic!("必须被拒收: {other:?}"),
    }
    assert_eq!(f.st, before_st, "拒收不得改动状态");
    assert_eq!(f.p, before_p, "拒收不得改动 P");
    // 对照：正常观测仍能正常融合（门控不是把通道关死）
    f.fuse(&gps_pos_from_truth(&t, &prm), Channel::GpsPos, 1e6, 1.0)
        .unwrap();
}
/// **长黑障后重灌真的重新锚定**：Pzz 先塌到地板，连续拒收触发重灌，随后观测必须能拉回。
#[test]
fn blackout_then_recovery_re_anchors() {
    let t = truth();
    let prm = ObsParams::default();
    let mut st0 = t;
    st0.p[2] = -50.0; // 高度偏 50 m
    let mut p0 = diag_cov(0.05);
    p0[I_POS + 2][I_POS + 2] = 1e-6; // 旧栈式"地板值"
    let mut f = Eskf::new(st0, p0, 10);
    // ① 黑障：连续喂"坏"气压观测（大残差）⇒ 被拒 10 次后触发重灌
    for _ in 0..12 {
        let mut bad = baro(-st0.p[2], &f.st, &prm);
        bad.resid = [500.0, 0.0, 0.0];
        let _ = f.fuse(&bad, Channel::Baro, 3.0, 1.0);
    }
    assert!(f.guards[0].recoveries() >= 1, "连续拒收必须触发重灌");
    // ② 恢复：喂正确气压观测 ⇒ 必须能拉回来（若未重灌，增益≈0 会卡死）
    let mut last = f.st.p[2].abs();
    for _ in 0..300 {
        let _ = f.fuse(&baro_from_truth(&t, &prm), Channel::Baro, 1e6, 1.0);
        last = f.st.p[2].abs();
    }
    assert!(last < 1.0, "重灌后必须能重新锚定高度: |p_z|={last}");
}
