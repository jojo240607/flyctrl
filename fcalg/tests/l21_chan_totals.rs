//! L21 —— 按通道的接受/拒收**总数**（诊断门面的原材料）+ **闭合判据**。
//! 闭合判据（`acc + rej == attempts`）正是当初抓出 L3 `Stale` 永久卡死的同款判据。
use fcalg::covariance::Cov;
use fcalg::error_state::N;
use fcalg::filter::Eskf;
use fcalg::gate::Channel;
use fcalg::observe::{baro, gps_pos, ObsParams};
use fcalg::propagate::State;
use fcalg::quat::Quat;
fn diag_cov(d: f32) -> Cov {
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        p[i][i] = d;
    }
    p
}
fn truth() -> State {
    let mut st = State::level();
    st.q = Quat::from_euler_zyx([0.1, -0.2, 0.3]).normalize().unwrap();
    st
}
/// 闭合：每个通道的 `接受 + 拒收` 必须等于**尝试数**（`nis_n`）—— 计数器不得漏项。
#[test]
fn per_channel_counters_close() {
    let t = truth();
    let prm = ObsParams::default();
    let mut f = Eskf::new(t, diag_cov(0.2), 10);
    for k in 0..300 {
        // 每 3 拍喂一个**荒谬** GPS 位置（必被门限拒）⇒ 制造确定性的接受/拒收混合
        let pp = if k % 3 == 0 { [1e5, -1e5, 1e5] } else { [0.0; 3] };
        let ob = gps_pos(pp, &f.st, &prm);
        let _ = f.fuse(&ob, Channel::GpsPos, 0.0, 1.0); // 0.0 ⇒ 用 dof 导出门限
        let ob = baro(0.0, &f.st, &prm);
        let _ = f.fuse(&ob, Channel::Baro, 0.0, 1.0);
    }
    for i in 0..4 {
        assert_eq!(
            f.chan_acc[i] as u64 + f.chan_rej[i] as u64,
            f.nis_n[i] as u64,
            "通道 {i}: 接受 {} + 拒收 {} 必须等于尝试 {}",
            f.chan_acc[i],
            f.chan_rej[i],
            f.nis_n[i]
        );
    }
    // 本场景必须真的产生过拒收（否则闭合判据是空跑）
    assert!(f.chan_rej[1] > 0, "荒谬 GPS 必须被拒过: {}", f.chan_rej[1]);
    assert!(f.chan_acc[1] > 0, "正常 GPS 必须被接受过: {}", f.chan_acc[1]);
    assert!(f.chan_acc[0] > 0 && f.chan_rej[0] == 0, "气压全正常 ⇒ 不应有拒收");
}
/// 与 `guards[i].rejects()` 的**语义区分**必须真实存在（前者是连续、后者是总数）：
/// 接受一次后连续计数清零，而**总数**只增不减。
#[test]
fn totals_are_monotonic_and_differ_from_consecutive() {
    let t = truth();
    let prm = ObsParams::default();
    let mut f = Eskf::new(t, diag_cov(0.2), 10);
    // 先连续拒两次
    for _ in 0..2 {
        let ob = gps_pos([1e5, 1e5, 1e5], &f.st, &prm);
        let _ = f.fuse(&ob, Channel::GpsPos, 0.0, 1.0);
    }
    let rej_total = f.chan_rej[1];
    assert!(rej_total >= 2, "连续两次荒谬必被拒: {rej_total}");
    // 再接受一次 ⇒ `guards.rejects()`（连续）清零，而**总数**保持
    let ob = gps_pos([0.0; 3], &f.st, &prm);
    let _ = f.fuse(&ob, Channel::GpsPos, 0.0, 1.0);
    assert_eq!(f.guards[1].rejects(), 0, "连续计数应在接受后清零（用于触发重灌）");
    assert_eq!(f.chan_rej[1], rej_total, "**总数**不得被清零（诊断语义）");
}
