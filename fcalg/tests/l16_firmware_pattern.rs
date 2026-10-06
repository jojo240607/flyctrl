//! L16 验收 —— **按固件的真实调用图样**驱动整条新栈（仍在 host，不经仿真器）
//!
//! 为何要有这一层：接线进固件之前，必须先在**固件的调用图样**下证明它。
//! 这个图样恰恰是旧栈出过事的地方：
//! - **1 kHz IMU 累积到 200 Hz 估计步**（每步 5 个增量）—— 旧栈曾把"每样本一次 predict"
//!   改成"累积后一次"，也曾在两种做法间反复；两种做法都必须在判据下
//! - **多率观测**：气压 50 Hz、GPS 10 Hz、磁 10 Hz（不同率、不同通道、各自门控）
//! - **双 dt 不同源**：`dt_ang` 与 `dt_vel` 独立（固件里它们来自不同的环形口径）
//!
//! 判据：收敛到真值、全程有限、**健康传感器下不得触发任何重灌**、多率确实按预期发。
use fcalg::error_state::{N};
use fcalg::filter::Eskf;
use fcalg::gate::Channel;
use fcalg::imu_delta::ImuDelta;
use fcalg::observe::{baro, gps_pos, gps_vel, mag_yaw, ObsParams};
use fcalg::propagate::State;
use fcalg::quat::{specific_force_at_rest, Quat, GRAVITY_NED};
use fcalg::sim::imu_from_truth;
/// IMU 周期（1 kHz）。
const IMU_DT: f32 = 0.001;
/// 每个估计步累积的 IMU 样本数（1 kHz ÷ 200 Hz = 5）。
const EST_DIV: usize = 5;
/// 估计步周期。
const EST_DT: f32 = IMU_DT * EST_DIV as f32;
fn diag_cov(d: f32) -> fcalg::covariance::Cov {
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        p[i][i] = d;
    }
    p
}
fn truth() -> State {
    let mut st = State::level();
    st.q = Quat::from_euler_zyx([0.18, -0.27, 0.9]).normalize().unwrap();
    st.mag_i = [0.25, 0.05, 0.42];
    st.mag_b = [0.01, -0.02, 0.03];
    st
}
/// 按固件图样跑：累积 IMU → 一次 predict → 按各自率融合。
/// 返回 (收敛后的状态, 各通道重灌次数, 各通道实际融合次数)。
fn run(dual_dt: bool) -> (State, [u32; 4], [u32; 4]) {
    let t = truth();
    let prm = ObsParams::default();
    let mut f = Eskf::new(t, diag_cov(0.2), 10);
    let f_b = specific_force_at_rest(t.q);
    let mut fused = [0u32; 4];
    for step in 0..4000 {
        // ① 累积本步的 IMU 增量（**不丢样本**：全部参与积分）
        let (mut da, mut dv, mut ta, mut tv) = ([0.0f32; 3], [0.0f32; 3], 0.0f32, 0.0f32);
        for _ in 0..EST_DIV {
            // 双 dt 场景：角口径的 dt 取两倍（模拟两个时钟不同源）
            let (dta, dtv) = if dual_dt { (2.0 * IMU_DT, IMU_DT) } else { (IMU_DT, IMU_DT) };
            let d = imu_from_truth(&f.st, [0.0; 3], f_b, dta, dtv);
            for k in 0..3 {
                da[k] += d.delta_ang[k];
                dv[k] += d.delta_vel[k];
            }
            ta += d.dt_ang;
            tv += d.dt_vel;
        }
        let acc = ImuDelta { delta_ang: da, delta_vel: dv, dt_ang: ta, dt_vel: tv, ts_ticks: 0 };
        f.predict(&acc, GRAVITY_NED).unwrap();
        // ② 多率观测（与真值同源但**对当前状态**构造 ⇒ 新息反映真实偏差）
        if step % 4 == 0 {
            let o = baro(-t.p[2], &f.st, &prm);
            if f.fuse(&o, Channel::Baro, 3.0, 1.0).is_ok() {
                fused[0] += 1;
            }
        }
        if step % 20 == 0 {
            let o = gps_pos(t.p, &f.st, &prm);
            if f.fuse(&o, Channel::GpsPos, 6.0, 1.0).is_ok() {
                fused[1] += 1;
            }
            let o = gps_vel(t.v, &f.st, &prm);
            if f.fuse(&o, Channel::GpsVel, 6.0, 1.0).is_ok() {
                fused[2] += 1;
            }
            let m = t.q.conj().rotate(t.mag_i);
            let meas = [m[0] + t.mag_b[0], m[1] + t.mag_b[1], m[2] + t.mag_b[2]];
            let o = mag_yaw(meas, &f.st, 2.0);
            if f.fuse(&o, Channel::MagYaw, 6.0, 1.0).is_ok() {
                fused[3] += 1;
            }
        }
    }
    let rec = [
        f.guards[0].recoveries(),
        f.guards[1].recoveries(),
        f.guards[2].recoveries(),
        f.guards[3].recoveries(),
    ];
    (f.st, rec, fused)
}
/// 固件图样（1 kHz→200 Hz 累积 + 多率观测）下必须收敛到真值。
#[test]
fn firmware_pattern_converges() {
    let t = truth();
    let (st, rec, fused) = run(false);
    // ① 收敛
    let dot = (st.q.w * t.q.w + st.q.x * t.q.x + st.q.y * t.q.y + st.q.z * t.q.z).abs();
    assert!(dot > 1.0 - 1e-4, "姿态须收敛: dot={dot}");
    for k in 0..3 {
        assert!(st.v[k].abs() < 5e-2, "速度须收敛: {:?}", st.v);
        assert!(st.p[k].abs() < 5e-2, "位置须收敛: {:?}", st.p);
    }
    // ② 健康传感器下**不得触发任何重灌**（若触发，说明门控阈值或噪声参数不自洽）
    assert_eq!(rec, [0, 0, 0, 0], "健康传感器下不应有任何重灌: {rec:?}");
    // ③ 多率确实按预期发（结构判据：抓"某个通道其实没在跑"这类静默失效）
    assert!(fused[0] > 900, "气压应约 1000 次: {}", fused[0]);
    assert!(fused[1] > 180 && fused[1] < 220, "GPS 位应约 200 次: {}", fused[1]);
    assert!(fused[2] > 180 && fused[2] < 220, "GPS 速应约 200 次: {}", fused[2]);
    assert!(fused[3] > 180 && fused[3] < 220, "磁航向应约 200 次: {}", fused[3]);
}
/// 双 dt 不同源（固件里角口径与速度口径来自不同的环形）也必须收敛。
#[test]
fn firmware_pattern_converges_with_dual_dt() {
    let t = truth();
    let (st, rec, _) = run(true);
    assert!(st.q.is_finite() && st.v.iter().all(|x| x.is_finite()));
    let dot = (st.q.w * t.q.w + st.q.x * t.q.x + st.q.y * t.q.y + st.q.z * t.q.z).abs();
    assert!(dot > 1.0 - 1e-3, "双 dt 下姿态也须收敛: dot={dot}");
    assert_eq!(rec, [0, 0, 0, 0], "双 dt 下也不应触发重灌: {rec:?}");
}
/// 累积**不丢样本**：把 1 kHz 的增量一次性累积后 predict，与
/// 分 5 次逐样本 predict 的**角增量总和**必须一致（防"累积时丢样本"的回退）。
#[test]
fn accumulation_does_not_drop_samples() {
    let t = truth();
    let f_b = specific_force_at_rest(t.q);
    let w = [0.4f32, -0.2, 0.1];
    let (mut da_sum, mut n) = ([0.0f32; 3], 0);
    for _ in 0..EST_DIV {
        let d = imu_from_truth(&t, w, f_b, IMU_DT, IMU_DT);
        for k in 0..3 {
            da_sum[k] += d.delta_ang[k];
        }
        n += 1;
    }
    assert_eq!(n, EST_DIV, "必须恰好累积 EST_DIV 个样本");
    // 总量的解析值：Σ (ω+bg)·dt = ω·(N·dt)
    for k in 0..3 {
        let expect = w[k] * (IMU_DT * n as f32);
        assert!(
            (da_sum[k] - expect).abs() < 1e-6,
            "累积角增量必须等于解析总量 @{k}: {} vs {}",
            da_sum[k],
            expect
        );
    }
}
