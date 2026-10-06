//! L12 验收 —— 真值注入的两条强判据：
//! ① **完美观测 ⇒ 四路新息恒为 0**（一次性验证四路的符号与帧约定）
//! ② **生成 → 传播**与**闭式指数解**一致（生成式是传播式的严格逆）
use fcalg::error_state::{boxplus, I_ATT, I_BG, I_POS, I_VEL, N};
use fcalg::observe::ObsParams;
use fcalg::propagate::{propagate, State};
use fcalg::quat::{specific_force_at_rest, Quat, GRAVITY_NED};
use fcalg::sim::{
    baro_from_truth, gps_pos_from_truth, gps_vel_from_truth, imu_from_truth, mag_yaw_from_truth,
};
fn truth() -> State {
    let mut st = State::level();
    st.q = Quat::from_euler_zyx([0.25, -0.35, 1.1]).normalize().unwrap();
    st.v = [1.5, -0.7, 0.3];
    st.p = [12.0, -4.5, -30.0];
    st.bg = [0.01, -0.02, 0.005];
    st.ba = [0.03, -0.01, 0.02];
    st.mag_i = [0.25, 0.05, 0.42];
    st.mag_b = [0.01, -0.02, 0.03];
    st
}
/// **完美观测 ⇒ 四路新息恒为 0**（含磁航向 —— 它耦合姿态与磁两态）。
#[test]
fn perfect_observations_give_zero_innovation() {
    let st = truth();
    let prm = ObsParams::default();
    let chans = [
        ("baro", baro_from_truth(&st, &prm)),
        ("gps_pos", gps_pos_from_truth(&st, &prm)),
        ("gps_vel", gps_vel_from_truth(&st, &prm)),
        ("mag_yaw", mag_yaw_from_truth(&st, 0.3)),
    ];
    for (name, o) in chans.iter() {
        for a in 0..3 {
            assert!(
                o.resid[a].abs() < 1e-5,
                "{name} 的第 {a} 轴新息必须为 0（符号/帧约定错？）: {}",
                o.resid[a]
            );
        }
    }
}
/// 生成式是传播式的**严格逆**：常角速率下，传播结果必须等于**闭式指数解**。
#[test]
fn generate_then_propagate_matches_exponential_closed_form() {
    let st0 = truth();
    let mut st = st0;
    let w = [0.6f32, -0.4, 0.3];
    let f_b = [0.2f32, -0.1, -9.7];
    let dt = 0.002f32;
    let n = 1500;
    for _ in 0..n {
        let d = imu_from_truth(&st, w, f_b, dt, dt);
        propagate(&mut st, &d, GRAVITY_NED).unwrap();
    }
    // 姿态：闭式解 q0 ⊗ exp(ω·T)
    let t = dt * n as f32;
    let ang = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt() * t;
    let axis = {
        let m = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
        [w[0] / m, w[1] / m, w[2] / m]
    };
    let dq = Quat::from_axis_angle(axis, ang);
    let q_expect = st0.q.mul(dq).normalize().unwrap();
    let dot = (st.q.w * q_expect.w + st.q.x * q_expect.x + st.q.y * q_expect.y + st.q.z * q_expect.z).abs();
    assert!(dot > 1.0 - 1e-4, "姿态须与闭式解一致: dot={dot}");
    // 速度：v0 + (R(q)·f_b + g)·T —— 姿态在变 ⇒ 只能用**增量式**核对首末，
    // 故这一条只断言"有限且在合理量级"（严格的解析核对见 L5 的常姿态用例）
    assert!(st.v.iter().all(|x| x.is_finite() && x.abs() < 1e4));
    assert!(st.p.iter().all(|x| x.is_finite() && x.abs() < 1e6));
}
/// **静止真值全程不动**：由真值生成的比力恰好抵消重力，一路传播后姿态不漂、速度位置停在零。
#[test]
fn static_truth_stays_static_end_to_end() {
    let st0 = truth();
    let mut st = State::level();
    st.q = st0.q;
    let f_b = specific_force_at_rest(st0.q);
    let dt = 0.005f32;
    for _ in 0..4000 {
        let d = imu_from_truth(&st, [0.0; 3], f_b, dt, dt);
        propagate(&mut st, &d, GRAVITY_NED).unwrap();
    }
    let dot = (st.q.w * st0.q.w + st.q.x * st0.q.x + st.q.y * st0.q.y + st.q.z * st0.q.z).abs();
    assert!(dot > 1.0 - 1e-6, "静止真值下姿态不漂: dot={dot}");
    for k in 0..3 {
        assert!(st.v[k].abs() < 1e-3, "静止真值下速度须为零: {:?}", st.v);
        assert!(st.p[k].abs() < 1e-2, "静止真值下位置须为零: {:?}", st.p);
    }
    // 生成器自身的一致性：真值状态下各观测新息仍为 0
    let prm = ObsParams::default();
    assert!(baro_from_truth(&st, &prm).resid[0].abs() < 1e-5);
    assert!(gps_pos_from_truth(&st, &prm).resid[0].abs() < 1e-9);
}
/// 退化防护：真值注入也要过有限性纪律（不产出 NaN 输入）。
#[test]
fn generated_inputs_are_finite() {
    let st = truth();
    let d = imu_from_truth(&st, [0.1, 0.2, 0.3], [0.0, 0.0, -9.8], 0.01, 0.01);
    assert!(d.delta_ang.iter().all(|x| x.is_finite()));
    assert!(d.delta_vel.iter().all(|x| x.is_finite()));
    // boxplus 的熵检查：真值 + 零误差 = 真值
    let same = boxplus(&st, &[0.0; N]).unwrap();
    assert_eq!(same, st);
    let _ = (I_ATT, I_BG, I_POS, I_VEL);
}
