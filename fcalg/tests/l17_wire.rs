//! L17 验收 —— 适配层的两条关键判据 + 输出映射。
//! 口径 (b)：**不要求与旧栈逐位等价**，只要求适配层正确、且新栈满足自身契约。
use fcalg::covariance::Cov;
use fcalg::error_state::N;
use fcalg::filter::Eskf;
use fcalg::observe::ObsParams;
use fcalg::propagate::State;
use fcalg::quat::Quat;
use fcalg::wire::{fuse_fw, to_fw, FwObservation};
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
/// 由真值造一帧固件形状的观测（**测量值**，不是 Obs 对象）。
fn fw_from_truth(t: &State) -> FwObservation {
    let m = t.q.conj().rotate(t.mag_i);
    FwObservation {
        baro_alt: Some(-t.p[2]),
        gps_pos: Some(t.p),
        gps_vel: Some(t.v),
        mag_body: Some([m[0] + t.mag_b[0], m[1] + t.mag_b[1], m[2] + t.mag_b[2]]),
    }
}
/// **★陷阱判据（本层存在的理由）**：状态偏离真值时，喂测量值**必须真的修正状态**。
/// 若适配层错误地接 `Obs` 对象，新息会恒为 0、状态一动不动 —— 这条会红。
#[test]
fn adapter_closes_the_observation_trap() {
    let t = truth();
    let prm = ObsParams::default();
    let mut st0 = t;
    st0.p = [20.0, -5.0, -15.0]; // 明显偏离
    let mut f = Eskf::new(st0, diag_cov(1.0), 10);
    let before = f.st.p;
    let r = fuse_fw(&mut f, &fw_from_truth(&t), &prm, 1e6, 1.0);
    assert!(r[1].unwrap().is_ok(), "GPS 位必须被接受");
    // 必须动了，且朝真值方向
    for k in 0..3 {
        assert!(
            (f.st.p[k] - before[k]).abs() > 1e-3,
            "适配层必须让测量真的修正状态 @{k}（否则就是接错成 Obs 的静默失效）"
        );
        assert!(
            f.st.p[k].abs() < before[k].abs(),
            "修正方向必须朝真值 @{k}: {} → {}",
            before[k],
            f.st.p[k]
        );
    }
}
/// 状态=真值时，完美测量 ⇒ 状态基本不动（新息≈0）。
#[test]
fn perfect_measurements_hold_state() {
    let t = truth();
    let prm = ObsParams::default();
    let mut f = Eskf::new(t, diag_cov(0.05), 10);
    let before = f.st;
    let r = fuse_fw(&mut f, &fw_from_truth(&t), &prm, 1e6, 1.0);
    for x in r.iter() {
        assert!(x.map(|v| v.is_ok()).unwrap_or(false), "四路都应被接受");
    }
    let dot = (f.st.q.w * before.q.w + f.st.q.x * before.q.x + f.st.q.y * before.q.y + f.st.q.z * before.q.z).abs();
    assert!(dot > 1.0 - 1e-6, "姿态不应被动");
    for k in 0..3 {
        assert!((f.st.p[k] - before.p[k]).abs() < 1e-4, "位置不应被动 @{k}");
    }
}
/// `None` 的观测不得被融合（分工要显式）。
#[test]
fn absent_observations_are_not_fused() {
    let t = truth();
    let prm = ObsParams::default();
    let mut f = Eskf::new(t, diag_cov(0.05), 10);
    let mut o = fw_from_truth(&t);
    o.baro_alt = None;
    o.mag_body = None;
    let r = fuse_fw(&mut f, &o, &prm, 1e6, 1.0);
    assert!(r[0].is_none() && r[3].is_none(), "无观测的通道必须返回 None");
    assert!(r[1].is_some() && r[2].is_some());
}
/// 输出映射：NED 位置/速度、wxyz 姿态、机体角速度、加计零偏逐字段对应。
#[test]
fn output_mapping_is_field_wise_correct() {
    let mut st = truth();
    st.p = [1.5, -2.5, 0.75];
    st.v = [-0.5, 0.25, 0.1];
    st.ba = [0.01, -0.02, 0.03];
    let w = [0.1f32, -0.2, 0.3];
    let e = to_fw(&st, w);
    assert_eq!(e.pos, [1.5, -2.5, 0.75]);
    assert_eq!(e.vel, [-0.5, 0.25, 0.1]);
    assert_eq!(e.att_wxyz, [st.q.w, st.q.x, st.q.y, st.q.z]);
    assert_eq!(e.omega, w);
    assert_eq!(e.accel_bias, [0.01, -0.02, 0.03]);
}
