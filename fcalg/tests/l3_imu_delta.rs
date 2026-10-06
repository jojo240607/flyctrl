//! L3 验收 —— 只用契约 §6 允许的判据：
//! 可解析合成输入（戳间隔 → dt）、极限/退化（零 dt / 停顿 / 回绕 / 非有限）、以及计数不变量。

use fcalg::finite::{reset, violations, Stage, Violation};
use fcalg::imu_delta::{DeltaBuilder, Reject};

const TPS: u32 = 1_000_000; // 1 MHz tick ⇒ 1 tick = 1 µs
const MAX_DT: f32 = 0.02;

fn b() -> DeltaBuilder {
    DeltaBuilder::new(TPS, MAX_DT)
}

/// 可解析合成输入：戳间隔 → dt 必须**精确等于**该间隔（不是名义周期）。
/// 同时：Δang/Δvel = 速率 × dt（定义式）。
#[test]
fn dt_from_adjacent_timestamps_and_delta_is_rate_times_dt() {
    let mut d = b();
    assert_eq!(d.push(1000, [0.0; 3], [0.0; 3]), Err(Reject::First));
    assert!(d.primed());
    assert_eq!(d.dropped(), 0, "首样本是初始化，不得计为丢弃");

    let g = [0.5f32, -0.25, 0.125];
    let a = [0.1f32, -0.2, -9.8];
    let out = d.push(2000, g, a).unwrap();
    assert!((out.dt_ang - 1e-3).abs() < 1e-9, "dt 须 = 戳差 1ms: {}", out.dt_ang);
    assert_eq!(out.dt_ang, out.dt_vel, "双口径必须同给");
    for k in 0..3 {
        assert_eq!(out.delta_ang[k], g[k] * out.dt_ang, "Δang 须 = 速率 × dt");
        assert_eq!(out.delta_vel[k], a[k] * out.dt_vel, "Δvel 须 = 比力 × dt");
    }
}

/// 核心性质：**不规则间隔必须被如实跟踪**（这正是"用名义周期"会错的地方）。
#[test]
fn irregular_intervals_are_tracked_not_nominal() {
    let mut d = b();
    let _ = d.push(0, [0.0; 3], [0.0; 3]);
    // 期望值**由步长导出**（1 tick = 1 µs），不手写常数以免算错
    let steps = [1000u32, 3000, 3500, 9500];
    let mut ts = 0u32;
    for step in steps {
        ts += step;
        let expect = step as f32 / TPS as f32;
        let out = d.push(ts, [1.0, 0.0, 0.0], [0.0, 0.0, -9.8]).unwrap();
        assert!(
            (out.dt_ang - expect).abs() < 1e-9,
            "dt 必须跟随戳差: 期望 {expect} 实测 {}",
            out.dt_ang
        );
    }
    assert_eq!(d.accepted(), 4);
    assert_eq!(d.dropped(), 0);
}

/// 极限/退化：零 dt 与停顿必须被拒绝并计数。
#[test]
fn zero_dt_and_stale_are_rejected_and_counted() {
    let mut d = b();
    let _ = d.push(1000, [0.0; 3], [0.0; 3]);
    assert_eq!(d.push(1000, [0.0; 3], [0.0; 3]), Err(Reject::ZeroDt));
    assert_eq!(d.dropped(), 1);
    // 停顿：dt = 30ms > max 20ms
    assert_eq!(d.push(1000 + 30_000, [0.0; 3], [0.0; 3]), Err(Reject::Stale));
    assert_eq!(d.dropped(), 2);
    assert_eq!(d.accepted(), 0);
}

/// `Stale` 的语义：**丢增量、不丢时间基** —— 自动重锚（计数 `resyncs`），
/// 于是后续样本立刻恢复正常 dt，而不是永久卡死（旧设计实测会卡死：accepted 恒 0）。
#[test]
fn stale_resyncs_instead_of_deadlocking() {
    let mut d = b();
    let _ = d.push(1000, [0.0; 3], [0.0; 3]);
    let _ = d.push(2000, [0.0; 3], [0.0; 3]).unwrap(); // last = 2000
    assert_eq!(d.push(100_000, [0.0; 3], [0.0; 3]), Err(Reject::Stale));
    assert_eq!(d.resyncs(), 1, "停顿必须被记为一次重锚");
    // 重锚后：立刻恢复正常（若未重锚，这一拍也会因 dt=已累积的巨差而被拒 ⇒ 永久卡死）
    let out = d.push(101_000, [0.0; 3], [0.0; 3]).unwrap();
    assert!((out.dt_ang - 1e-3).abs() < 1e-9, "重锚后须恢复: {}", out.dt_ang);
}

/// 回绕（不可分辨）：**不自动重锚**，必须由上层显式 `rebuild` —— 上层才知道真的回绕了。
#[test]
fn wrap_requires_explicit_rebuild() {
    let mut d = b();
    let _ = d.push(u32::MAX - 10_000, [0.0; 3], [0.0; 3]);
    let _ = d.push(u32::MAX - 9_000, [0.0; 3], [0.0; 3]).unwrap();
    assert_eq!(d.push(100, [0.0; 3], [0.0; 3]), Err(Reject::WrapAmbiguous));
    let before = d.resyncs();
    d.rebuild(100); // 上层确认回绕 ⇒ 显式重锚
    assert_eq!(d.resyncs(), before + 1);
    let out = d.push(1100, [0.0; 3], [0.0; 3]).unwrap();
    assert!((out.dt_ang - 1e-3).abs() < 1e-9);
}

/// 契约 §4：非有限输入 ⇒ 拒绝，且**时间基不被推进**。
#[test]
fn non_finite_does_not_advance_time_base() {
    reset();
    let mut d = b();
    let _ = d.push(1000, [0.0; 3], [0.0; 3]);
    assert_eq!(
        d.push(2000, [f32::NAN, 0.0, 0.0], [0.0, 0.0, 0.0]),
        Err(Reject::NonFinite(Violation::Nan))
    );
    assert!(violations(Stage::L3ImuDelta) > 0, "违规必须被计数");
    let out = d.push(2000, [0.0; 3], [0.0; 3]).unwrap();
    assert!((out.dt_ang - 1e-3).abs() < 1e-9, "坏样本不得推进时间基: {}", out.dt_ang);
}

/// 极限：32 位回绕（与戳倒退不可分辨）必须**显式拒绝**，不做静默修补。
#[test]
fn wrap_is_rejected_not_silently_accepted() {
    let mut d = b();
    let _ = d.push(u32::MAX - 10_000, [0.0; 3], [0.0; 3]);
    let _ = d.push(u32::MAX - 9_000, [0.0; 3], [0.0; 3]).unwrap();
    assert_eq!(d.push(100, [0.0; 3], [0.0; 3]), Err(Reject::WrapAmbiguous));
    assert_eq!(d.dropped(), 1);
}

/// 不变量：`accepted + dropped` 必须等于"除首样本外的投入数"。
#[test]
fn counters_add_up() {
    let mut d = b();
    let _ = d.push(0, [0.0; 3], [0.0; 3]); // First
    let inputs = 40;
    let mut ts = 0u32;
    for i in 0..inputs {
        ts += if i % 7 == 0 { 40_000 } else { 1_000 }; // 每 7 个插一次超阈停顿
        let _ = d.push(ts, [0.0; 3], [0.0; 3]);
    }
    assert_eq!(d.accepted() + d.dropped() as u64, inputs, "计数必须闭合");
    assert!(d.dropped() > 0 && d.accepted() > 0);
}
