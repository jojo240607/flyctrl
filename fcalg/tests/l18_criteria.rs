//! L18 · （乙）H 场数值判据 —— **新栈自己的**判据，且界**由参数表导出**（不是魔数）
//!
//! # 判据的构造方式（这是本层唯一要紧的事）
//! 界 = `k × σ_obs`，其中 `σ_obs` **从 `params` 表读**（`obs.sigma_*`）。
//! ⇒ 改参数表，判据自动跟随；不存在"某处写死 0.5 而别处改了 σ"的分叉。
//!
//! # 为何 `k = 1`
//! H 场是**合成、无噪声**的（契约 §0：算法验收不经仿真器 ⇒ 观测就是真值）。
//! 无噪声时估计误差只来自线性化/离散化/数值 ⇒ 应**远低于**观测噪声量级。
//! 取 `k = 1` 是保守的松界（实测值远在界内，见打印行）；**不是"调到刚好能过"**。
//!
//! # 明确不覆盖（登记而非编数）
//! **姿态误差**没有表内 σ 可用（`obs.sigma_mag` 是**门限**不是误差 σ）
//! ⇒ 本轮**不给姿态判据**，登记为待办（需么给磁航向一个真正的误差 σ，
//! 要么从位置/加速度噪声推导倾角误差）。**不编一个看起来合理的魔数。**

use fcalg::error_state::N;
use fcalg::filter::Eskf;
use fcalg::gate::Channel;
use fcalg::imu_delta::ImuDelta;
use fcalg::observe::{baro, gps_pos, gps_vel, ObsParams};
use fcalg::params::param;
use fcalg::propagate::State;
use fcalg::quat::{specific_force_at_rest, Quat, GRAVITY_NED};

const DT: f32 = 0.005;

fn diag_cov(d: f32) -> fcalg::covariance::Cov {
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        p[i][i] = d;
    }
    p
}

fn truth() -> State {
    let mut st = State::level();
    st.q = Quat::from_euler_zyx([0.15, -0.22, 0.7]).normalize().unwrap();
    st.mag_i = [0.25, 0.05, 0.42];
    st.mag_b = [0.01, -0.02, 0.03];
    st
}

/// H 场式驱动（合成真值、无噪声），返回最后 1000 拍的 RMS 误差：
/// `(位置, 速度, 高度)`。
fn rms_errors() -> (f32, f32, f32) {
    let t = truth();
    let prm = ObsParams::default();
    let mut f = Eskf::new(t, diag_cov(0.2), 10);
    let f_b = specific_force_at_rest(t.q);
    let (mut sp, mut sv, mut sa, mut n) = (0.0f64, 0.0f64, 0.0f64, 0u32);
    for k in 0..4000 {
        let d = ImuDelta {
            delta_ang: [0.0; 3],
            delta_vel: [f_b[0] * DT, f_b[1] * DT, f_b[2] * DT],
            dt_ang: DT,
            dt_vel: DT,
            ts_ticks: 0,
        };
        f.predict(&d, GRAVITY_NED).unwrap();
        // 观测取自真值、对**当前状态**构造（见 L17 的陷阱说明）
        let ob = baro(-t.p[2], &f.st, &prm);
        let _ = f.fuse(&ob, Channel::Baro, 1e6, 1.0);
        if k % 10 == 0 {
            let op = gps_pos(t.p, &f.st, &prm);
            let _ = f.fuse(&op, Channel::GpsPos, 1e6, 1.0);
            let ov = gps_vel(t.v, &f.st, &prm);
            let _ = f.fuse(&ov, Channel::GpsVel, 1e6, 1.0);
        }
        if k >= 3000 {
            let mut ep = 0.0f64;
            let mut ev = 0.0f64;
            for a in 0..3 {
                let dp = (f.st.p[a] - t.p[a]) as f64;
                let dv = (f.st.v[a] - t.v[a]) as f64;
                ep += dp * dp;
                ev += dv * dv;
            }
            sp += ep;
            sv += ev;
            sa += ((f.st.p[2] - t.p[2]) as f64) * ((f.st.p[2] - t.p[2]) as f64);
            n += 1;
        }
    }
    let nn = n as f64;
    (
        (sp / nn).sqrt() as f32,
        (sv / nn).sqrt() as f32,
        (sa / nn).sqrt() as f32,
    )
}

/// **（乙）主判据**：H 场式驱动下的 RMS 误差必须 ≤ 从参数表导出的界（`k=1`）。
#[test]
fn h_field_errors_are_below_table_derived_bounds() {
    let (ep, ev, ea) = rms_errors();
    // ★界**从参数表读** —— 不在这里写魔数
    let b_pos = param("obs.sigma_gps_p").expect("参数表须有 obs.sigma_gps_p").value;
    let b_vel = param("obs.sigma_gps_v").expect("参数表须有 obs.sigma_gps_v").value;
    let b_alt = param("obs.sigma_baro").expect("参数表须有 obs.sigma_baro").value;
    // 基线记录（回归时可对比；不是判据本身）
    eprintln!(
        "[crit] RMS(位置)={ep:.6} ≤ {b_pos} | RMS(速度)={ev:.6} ≤ {b_vel} | RMS(高度)={ea:.6} ≤ {b_alt}"
    );
    assert!(
        ep <= b_pos,
        "位置 RMS 必须 ≤ 表内 σ_gps_p（{b_pos}）: 实测 {ep}"
    );
    assert!(
        ev <= b_vel,
        "速度 RMS 必须 ≤ 表内 σ_gps_v（{b_vel}）: 实测 {ev}"
    );
    assert!(
        ea <= b_alt,
        "高度 RMS 必须 ≤ 表内 σ_baro（{b_alt}）: 实测 {ea}"
    );
}

/// 判据的**自洽性**：界必须是"参数表的函数"，表变则界变（防有人改成魔数）。
#[test]
fn bounds_are_functions_of_the_param_table() {
    // 直接证明三条界都来自 params 表（而不是本文件里的常量）
    for name in ["obs.sigma_gps_p", "obs.sigma_gps_v", "obs.sigma_baro"] {
        let m = param(name).unwrap();
        assert!(m.value.is_finite() && m.value > 0.0, "{name} 须为有效 σ");
        assert!(
            !m.source.is_debt(),
            "{name} 作为判据的依据不得是「本重建选定」欠债（否则界没有独立依据）"
        );
    }
}

/// **P 不得塌到退化**（本会话踩过的坑：Q=0 ⇒ 协方差被反复收缩 ⇒ predict 某天突然 Err）。
/// 下界**由 Q 导出**（`0.1 × q_i × dt`），不是魔数 —— Q 变则下界跟着变。
#[test]
fn covariance_does_not_collapse_when_process_noise_is_nonzero() {
    let t = truth();
    let prm = ObsParams::default();
    let mut f = Eskf::new(t, diag_cov(0.2), 10);
    let f_b = specific_force_at_rest(t.q);
    // 长时间"完美观测"驱动（最容易把 P 收死的场景）
    for k in 0..1500 {
        let d = ImuDelta {
            delta_ang: [0.0; 3],
            delta_vel: [f_b[0] * DT, f_b[1] * DT, f_b[2] * DT],
            dt_ang: DT,
            dt_vel: DT,
            ts_ticks: 0,
        };
        f.predict(&d, GRAVITY_NED).expect("有 Q 之后 predict 不应再失败");
        let ob = baro(-t.p[2], &f.st, &prm);
        let _ = f.fuse(&ob, Channel::Baro, 1e6, 1.0);
        if k % 10 == 0 {
            let op = gps_pos(t.p, &f.st, &prm);
            let _ = f.fuse(&op, Channel::GpsPos, 1e6, 1.0);
            let ov = gps_vel(t.v, &f.st, &prm);
            let _ = f.fuse(&ov, Channel::GpsVel, 1e6, 1.0);
        }
    }
    let qn = fcalg::filter::ProcessNoise::default();
    let mut worst = f32::INFINITY;
    let mut worst_i = 0usize;
    for i in 0..N {
        let lo = 0.1 * qn.coeff(i) * DT; // 下界由 Q 导出
        if f.p[i][i] < lo {
            worst = f.p[i][i];
            worst_i = i;
        }
    }
    assert!(
        worst.is_infinite(),
        "状态 {worst_i} 的方差塌到 {worst}（Q 未起作用？下界由 q.coeff×dt 导出）"
    );
}
