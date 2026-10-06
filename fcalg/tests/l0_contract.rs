//! L0 验收 —— 只用契约 §6 允许的判据（不变量 / 极限退化 / 可解析合成输入）。
//! 不出现"把实现输出记为期望值"的断言。

use fcalg::quat::{specific_force_at_rest, Quat, GRAVITY_NED};
use fcalg::units::{meters_per_second, radians_from_rate, Meters, Mps, Radians, Rps, Seconds};

fn approx(a: f32, b: f32, tol: f32) -> bool {
    (a - b).abs() <= tol
}

/// 不变量：单位四元数不改变向量。
#[test]
fn identity_rotation_is_noop() {
    let v = [1.0, -2.0, 3.0];
    let r = Quat::IDENTITY.rotate(v);
    for i in 0..3 {
        assert!(approx(r[i], v[i], 1e-6), "单位旋转必须无操作: {r:?}");
    }
}

/// 不变量：旋转保模。
#[test]
fn rotate_preserves_norm() {
    let q = Quat::from_euler_zyx([0.3, -0.7, 1.9]);
    let v: [f32; 3] = [2.5, 0.0, -1.5];
    let n0 = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let r = q.rotate(v);
    let n1 = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
    assert!(approx(n0, n1, 1e-5), "旋转必须保模 {n0} vs {n1}");
}

/// 不变量：与共轭复合 ⇒ 往返恒等。
#[test]
fn conj_round_trips() {
    let q = Quat::from_euler_zyx([-0.4, 0.9, -2.2]).normalize().unwrap();
    let v = [0.7, -1.1, 0.4];
    let back = q.conj().rotate(q.rotate(v));
    for i in 0..3 {
        assert!(approx(back[i], v[i], 1e-5), "往返必须恒等: {back:?}");
    }
}

/// 不变量：复合 = 依次旋转（`a∘b` 先 `b` 后 `a`）。
#[test]
fn compose_equals_sequential_rotate() {
    let a = Quat::from_euler_zyx([0.2, 0.1, -0.3]).normalize().unwrap();
    let b = Quat::from_euler_zyx([-0.5, 0.25, 0.8]).normalize().unwrap();
    let v = [1.3, 0.6, -0.9];
    let once = a.mul(b).rotate(v);
    let twice = a.rotate(b.rotate(v));
    for i in 0..3 {
        assert!(approx(once[i], twice[i], 1e-5), "复合语义不一致: {once:?} vs {twice:?}");
    }
}

/// 契约自洽（§2）：静止水平时比力必须 = (0,0,−g)。
#[test]
fn level_attitude_specific_force_is_minus_g_z() {
    let f = specific_force_at_rest(Quat::IDENTITY);
    assert!(approx(f[0], 0.0, 1e-6) && approx(f[1], 0.0, 1e-6), "水平比力水平分量须为 0: {f:?}");
    assert!(approx(f[2], -GRAVITY_NED[2], 1e-5), "水平比力 z 须 = −g: {f:?}");
}

/// 契约自洽（§2）：纯偏航不改变比力。
#[test]
fn yaw_only_keeps_specific_force() {
    for yaw in [-2.0f32, 0.0, 0.5, 3.0] {
        let q = Quat::from_euler_zyx([0.0, 0.0, yaw]).normalize().unwrap();
        let f = specific_force_at_rest(q);
        assert!(
            approx(f[0], 0.0, 1e-5)
                && approx(f[1], 0.0, 1e-5)
                && approx(f[2], -GRAVITY_NED[2], 1e-5),
            "纯偏航不得改变比力 (yaw={yaw}): {f:?}"
        );
    }
}

/// 可解析合成输入：欧拉角往返（避开万向锁）。
#[test]
fn euler_round_trip() {
    for rpy in [[0.0f32, 0.0, 0.0], [0.35, -0.6, 1.2], [-1.1, 0.9, -2.7]] {
        let q = Quat::from_euler_zyx(rpy);
        let back = q.to_euler_zyx();
        for i in 0..3 {
            assert!(approx(back[i], rpy[i], 1e-4), "欧拉往返失败 {rpy:?} → {back:?}");
        }
    }
}

/// 极限/退化：非有限或零模 ⇒ 必须显式失败（不得静默替换，契约 §4）。
#[test]
fn normalize_rejects_degenerate() {
    assert!(Quat { w: f32::NAN, x: 0.0, y: 0.0, z: 0.0 }.normalize().is_none());
    assert!(Quat { w: f32::INFINITY, x: 0.0, y: 0.0, z: 0.0 }.normalize().is_none());
    assert!(Quat { w: 0.0, x: 0.0, y: 0.0, z: 0.0 }.normalize().is_none());
}

/// 极限：显式单位换算。
#[test]
fn unit_arithmetic_identities() {
    let d = Meters::new(10.0);
    let t = Seconds::new(2.0);
    assert_eq!(meters_per_second(d, t), Mps::new(5.0));
    assert_eq!(radians_from_rate(Rps::new(3.0), t), Radians::new(6.0));
    assert_eq!(Meters::new(6.0) / Meters::new(2.0), 3.0);
    assert_eq!((Meters::new(1.0) + Meters::new(2.0)).value(), 3.0);
}
