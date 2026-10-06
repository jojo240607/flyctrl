//! L5 验收 —— 只用契约 §6 允许的判据：解析闭式解对照（小角/大角/常加速度）、
//! 极限退化、不变量、跨模块往返（L0→L4→L5）、以及"失败时不半更新"。
use fcalg::align::{align_static, AlignConfig};
use fcalg::finite::{reset, violations, Stage, Violation};
use fcalg::imu_delta::ImuDelta;
use fcalg::propagate::{propagate, State};
use fcalg::quat::{specific_force_at_rest, Quat, GRAVITY_NED};
const G: f32 = GRAVITY_NED[2];
fn delta(ang: [f32; 3], vel: [f32; 3], dt: f32) -> ImuDelta {
    ImuDelta { delta_ang: ang, delta_vel: vel, dt_ang: dt, dt_vel: dt, ts_ticks: 0 }
}
/// 解析闭式解（小角）：常角速率 ⇒ 姿态必须等于 `from_axis_angle(轴, 总角)`。
#[test]
fn attitude_matches_closed_form_small_angle() {
    let dt = 0.001f32;
    let w = [0.3f32, -0.5, 0.2];
    let n = 100;
    let mut st = State::level();
    for _ in 0..n {
        let d = delta([w[0] * dt, w[1] * dt, w[2] * dt], [0.0; 3], dt);
        propagate(&mut st, &d, [0.0; 3]).unwrap();
    }
    let total = [w[0] * dt * n as f32, w[1] * dt * n as f32, w[2] * dt * n as f32];
    let a = (total[0] * total[0] + total[1] * total[1] + total[2] * total[2]).sqrt();
    let expect = Quat::from_axis_angle([total[0] / a, total[1] / a, total[2] / a], a);
    let dot = (st.q.w * expect.w + st.q.x * expect.x + st.q.y * expect.y + st.q.z * expect.z).abs();
    assert!(dot > 1.0 - 1e-5, "小角须与闭式解一致: dot={dot}");
}
/// 解析闭式解（**大角**，>2 rad）：证明不是小角近似。
#[test]
fn attitude_matches_closed_form_large_angle() {
    let dt = 0.001f32;
    let w = [1.0f32, 0.5, -0.25];
    let n = 2000;
    let mut st = State::level();
    for _ in 0..n {
        let d = delta([w[0] * dt, w[1] * dt, w[2] * dt], [0.0; 3], dt);
        propagate(&mut st, &d, [0.0; 3]).unwrap();
    }
    let total = [w[0] * dt * n as f32, w[1] * dt * n as f32, w[2] * dt * n as f32];
    let a = (total[0] * total[0] + total[1] * total[1] + total[2] * total[2]).sqrt();
    assert!(a > 2.0, "本用例必须是大角（总角 {a}）");
    let expect = Quat::from_axis_angle([total[0] / a, total[1] / a, total[2] / a], a);
    let dot = (st.q.w * expect.w + st.q.x * expect.x + st.q.y * expect.y + st.q.z * expect.z).abs();
    assert!(dot > 1.0 - 1e-4, "大角须与闭式解一致: dot={dot}");
}
/// 极限退化：比力恰好抵消重力 ⇒ 状态一动不动。
#[test]
fn thrust_against_gravity_leaves_state_unchanged() {
    let dt = 0.005f32;
    let mut st = State::level();
    let d = delta([0.0; 3], [0.0, 0.0, -G * dt], dt);
    for _ in 0..5000 {
        propagate(&mut st, &d, GRAVITY_NED).unwrap();
    }
    assert_eq!(st.q, Quat::IDENTITY, "无角速率 ⇒ 姿态不变");
    for k in 0..3 {
        assert!(st.v[k].abs() < 1e-3, "净加速度 0 ⇒ 速度不变: {:?}", st.v);
        assert!(st.p[k].abs() < 1e-2, "净加速度 0 ⇒ 位置不变: {:?}", st.p);
    }
}
/// 解析闭式解（自由落体）：比力为 0 ⇒ `v = g·t`、`p = ½g·t²`（梯形对常加速度精确）。
#[test]
fn free_fall_is_analytic() {
    let dt = 0.005f32;
    let n = 400;
    let mut st = State::level();
    for _ in 0..n {
        let d = delta([0.0; 3], [0.0; 3], dt);
        propagate(&mut st, &d, GRAVITY_NED).unwrap();
    }
    let t = dt * n as f32;
    assert!((st.v[2] - G * t).abs() < 1e-2, "v_z 须 = g·t: {} vs {}", st.v[2], G * t);
    let pf = 0.5 * G * t * t;
    assert!((st.p[2] - pf).abs() < 1e-1, "p_z 须 = ½g·t²: {} vs {}", st.p[2], pf);
    assert!(st.v[0].abs() < 1e-4 && st.v[1].abs() < 1e-4, "水平不得有速度");
}
/// 解析闭式解（净上行加速度）：水平姿态下 `f_b=(0,0,−g+a)` ⇒ `v_z = a·t`。
#[test]
fn net_accel_is_analytic() {
    let dt = 0.005f32;
    let a = 2.5f32;
    let n = 200;
    let mut st = State::level();
    for _ in 0..n {
        let d = delta([0.0; 3], [0.0, 0.0, (-G + a) * dt], dt);
        propagate(&mut st, &d, GRAVITY_NED).unwrap();
    }
    let t = dt * n as f32;
    assert!((st.v[2] - a * t).abs() < 1e-2, "v_z 须 = a·t: {} vs {}", st.v[2], a * t);
}
/// 定义式：陀螺零偏恰好等于角速率 ⇒ 姿态不变。
#[test]
fn gyro_bias_fully_compensates() {
    let dt = 0.002f32;
    let w = [0.4f32, -0.3, 0.7];
    let mut st = State::level();
    st.bg = w;
    for _ in 0..1000 {
        let d = delta([w[0] * dt, w[1] * dt, w[2] * dt], [0.0; 3], dt);
        propagate(&mut st, &d, [0.0; 3]).unwrap();
    }
    assert!(st.q.w.abs() > 1.0 - 1e-6, "零偏抵消后姿态不得变化: {:?}", st.q);
}
/// 不变量：长时间传播后四元数仍须单位模。
#[test]
fn quaternion_stays_unit_norm() {
    let dt = 0.001f32;
    let mut st = State::level();
    let d = delta([0.002f32, -0.001, 0.003], [0.01, -0.02, -0.05], dt);
    for _ in 0..20_000 {
        propagate(&mut st, &d, GRAVITY_NED).unwrap();
    }
    assert!((st.q.norm() - 1.0).abs() < 1e-6, "四元数模须保持 1: {}", st.q.norm());
    assert!(st.q.is_finite());
}
/// 契约 §3 的用处：双 dt 各司其职 —— 姿态用 `dt_ang`，速度用 `dt_vel`。
#[test]
fn dual_dt_are_used_separately() {
    let (dt_a, dt_v) = (0.01f32, 0.02f32);
    let mut st = State::level();
    st.bg = [1.0, 0.0, 0.0];
    let d = ImuDelta {
        delta_ang: [0.1, 0.0, 0.0],
        delta_vel: [0.0, 0.0, -G * dt_v],
        dt_ang: dt_a,
        dt_vel: dt_v,
        ts_ticks: 0,
    };
    propagate(&mut st, &d, GRAVITY_NED).unwrap();
    let roll = st.q.to_euler_zyx()[0];
    assert!((roll - 0.09).abs() < 1e-5, "姿态须用 dt_ang: roll={roll}");
    assert!(st.v[2].abs() < 1e-4, "速度须用 dt_vel: v={:?}", st.v);
}
/// 纪律（契约 §4）：非有限输入 ⇒ `Err`，且状态**逐位不变**。
#[test]
fn failure_leaves_state_bitwise_unchanged() {
    reset();
    let mut st = State::level();
    let ok = delta([0.01, 0.0, 0.0], [0.0; 3], 0.001);
    propagate(&mut st, &ok, GRAVITY_NED).unwrap();
    let before = st;
    let bad = delta([f32::NAN, 0.0, 0.0], [0.0; 3], 0.001);
    assert_eq!(propagate(&mut st, &bad, GRAVITY_NED), Err(Violation::Nan));
    assert_eq!(st, before, "失败时状态必须逐位不变");
    assert!(violations(Stage::L5Propagate) > 0);
}
/// 跨模块判据（L0→L4→L5）：由已知姿态经契约正向式生成静止比力，用 L4 对齐求初姿，
/// 再传播 —— 姿态不得漂、速度位置必须停在零。
#[test]
fn static_stream_from_known_attitude_does_not_drift() {
    for rpy in [[0.3f32, -0.5, 0.0], [-0.8, 0.6, 0.0]] {
        let q_true = Quat::from_euler_zyx(rpy);
        let f_b = specific_force_at_rest(q_true);
        let aligned = align_static(f_b, [0.0; 3], AlignConfig::default()).unwrap();
        let mut st = State::level();
        st.q = aligned.q;
        let dt = 0.005f32;
        let d = delta([0.0; 3], [f_b[0] * dt, f_b[1] * dt, f_b[2] * dt], dt);
        for _ in 0..5000 {
            propagate(&mut st, &d, GRAVITY_NED).unwrap();
        }
        let dot = (st.q.w * aligned.q.w
            + st.q.x * aligned.q.x
            + st.q.y * aligned.q.y
            + st.q.z * aligned.q.z)
            .abs();
        assert!(dot > 1.0 - 1e-6, "静止流下姿态不得漂: dot={dot}");
        for k in 0..3 {
            assert!(st.v[k].abs() < 1e-3, "静止流下速度须为零: {:?}", st.v);
            assert!(st.p[k].abs() < 1e-2, "静止流下位置须为零: {:?}", st.p);
        }
    }
}
