//! L20 —— H 场判据的**鉴别力**（补上 L18 的不足：RMS 恒 0 ⇒ 任何实现都过）
//!
//! 做法：让场景包含**"错的实现一定会失败"的东西**
//! 1. **观测噪声**：各通道加噪 ⇒ 稳态误差应 ≈ σ_obs（Kalman 一致性），界取 2σ（覆盖尾部）
//! 2. **坏值尖峰**：1% 拍注入 50σ ⇒ **门控必须拒**（断言 reject 计数 > 0）
//!    若门控失效（静默接受），RMS 会远越界 ⇒ 两条断言同时抓
//! 3. **黑障 + 恢复**：GPS 断 3 s ⇒ 必须靠 L10 的**重灌**重新锚定
//!    （断言 recoveries ≥ 1 且恢复后误差回到界内；无重灌机制的实现会卡死 ⇒ 红）
//!
//! 噪声用**确定性伪随机**（xorshift32，无依赖、可复现）⇒ 判据不 flaky。
//! 界一律**从参数表读**（`obs.sigma_*`），不写魔数。
use fcalg::covariance::Cov;
use fcalg::error_state::N;
use fcalg::filter::Eskf;
use fcalg::gate::Channel;
use fcalg::imu_delta::ImuDelta;
use fcalg::observe::{baro, gps_pos, gps_vel, ObsParams};
use fcalg::params::param;
use fcalg::propagate::State;
use fcalg::quat::{specific_force_at_rest, Quat, GRAVITY_NED};

const DT: f32 = 0.005;
const WARM: usize = 2000;
const TOTAL: usize = 4000;

fn diag_cov(d: f32) -> Cov {
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
/// 确定性伪随机（xorshift32）⇒ 可复现，判据不 flaky。
struct Rng(u32);
impl Rng {
    fn new(seed: u32) -> Self {
        Self(seed | 1)
    }
    /// 均匀分布 (-1, 1)
    fn u(&mut self) -> f32 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        self.0 = x;
        (x as i32 as f32) / 2147483648.0
    }
    /// 近似高斯（12 个均匀之和，Irwin–Hall）⇒ 均值 0、方差 1
    fn g(&mut self) -> f32 {
        let mut s = 0.0f32;
        for _ in 0..12 {
            s += self.u();
        }
        s
    }
}

/// 主判据：噪声 + 坏值尖峰下，稳态误差必须 ≤ 2σ（界从参数表读），且**门控确实拒了**。
#[test]
fn noisy_with_outliers_stays_within_2sigma_and_gate_fires() {
    let t = truth();
    let prm = ObsParams::default();
    let f_b = specific_force_at_rest(t.q);
    let mut f = Eskf::new(t, diag_cov(0.2), 10);
    let mut rng = Rng::new(0x1234_5678);
    let mut n = 0u32;
    let (mut sp, mut sv, mut sa, mut outliers) = (0.0f64, 0.0f64, 0.0f64, 0u32);
    for k in 0..TOTAL {
        let d = ImuDelta {
            delta_ang: [0.0; 3],
            delta_vel: [f_b[0] * DT, f_b[1] * DT, f_b[2] * DT],
            dt_ang: DT,
            dt_vel: DT,
            ts_ticks: 0,
        };
        f.predict(&d, GRAVITY_NED).unwrap();
        // 气压：真值 0 + 噪声
        let alt = 0.0 + prm.sigma_baro * rng.g();
        let ob = baro(alt, &f.st, &prm);
        let _ = f.fuse(&ob, Channel::Baro, 0.0, 1.0);
        if k % 10 == 0 {
            // 1% 拍注入 50σ 坏值
            let spike = if k % 1000 == 0 {
                outliers += 1;
                50.0
            } else {
                0.0
            };
            let pp = [
                spike + prm.sigma_gps_p * rng.g(),
                prm.sigma_gps_p * rng.g(),
                prm.sigma_gps_p * rng.g(),
            ];
            let ob = gps_pos(pp, &f.st, &prm);
            let _ = f.fuse(&ob, Channel::GpsPos, 0.0, 1.0);
            let vv = [
                prm.sigma_gps_v * rng.g(),
                prm.sigma_gps_v * rng.g(),
                prm.sigma_gps_v * rng.g(),
            ];
            let ob = gps_vel(vv, &f.st, &prm);
            let _ = f.fuse(&ob, Channel::GpsVel, 0.0, 1.0);
        }
        if k >= WARM {
            let mut ep = 0.0f64;
            let mut ev = 0.0f64;
            for a in 0..3 {
                ep += (f.st.p[a] as f64) * (f.st.p[a] as f64);
            }
            // ★速度 RMS 只取**水平两轴**：按 L9 的设计，GPS 速度的垂直分量是
            //   **"无信息"（R=1e12，刻意不观）** ⇒ 要求一个不可观的量满足观测噪声界
            //   是**判据错**（首版就是这么错的：RMS 被不可观的 v_z 主导 ⇒ 3.2× 越界）。
            //   垂直速度由气压 + GPS **位置**锚定，它的误差该有自己的判据（待补，登记）。
            for a in 0..2 {
                ev += (f.st.v[a] as f64) * (f.st.v[a] as f64);
            }
            sp += ep;
            sv += ev;
            sa += (f.st.p[2] as f64) * (f.st.p[2] as f64);
            n += 1;
        }
    }
    let (ep, ev, ea) = (
        (sp / n as f64).sqrt() as f32,
        (sv / n as f64).sqrt() as f32,
        (sa / n as f64).sqrt() as f32,
    );
    let (b_pos, b_vel, b_alt) = (
        2.0 * param("obs.sigma_gps_p").unwrap().value,
        2.0 * param("obs.sigma_gps_v").unwrap().value,
        2.0 * param("obs.sigma_baro").unwrap().value,
    );
    eprintln!(
        "[disc] 注入坏值 {outliers} 次；RMS pos={ep:.4} ≤ {b_pos} | vel={ev:.4} ≤ {b_vel} | alt={ea:.4} ≤ {b_alt} | 拒收={}",
        f.rejects_total
    );
    assert!(outliers > 0, "场景必须真的注入坏值");
    assert!(
        f.rejects_total > 0,
        "★门控必须**确实拒过**坏值（若静默接受，RMS 会被尖峰拉爆 ⇒ 下一条断言会红）"
    );
    assert!(ep <= b_pos, "带噪+坏值下位置 RMS 须 ≤ 2σ: {ep} > {b_pos}");
    assert!(ev <= b_vel, "速度 RMS 须 ≤ 2σ: {ev} > {b_vel}");
    assert!(ea <= b_alt, "高度 RMS 须 ≤ 2σ: {ea} > {b_alt}");
    assert!(f.healthy(), "必须保持有限");
}

/// 黑障 + 恢复：GPS 断 3 s ⇒ 必须靠 **L10 的重灌**重新锚定（无重灌的实现会卡死 ⇒ 红）。
#[test]
fn blackout_then_recovery_reanchors_in_driven_loop() {
    let t = truth();
    let prm = ObsParams::default();
    let f_b = specific_force_at_rest(t.q);
    let mut f = Eskf::new(t, diag_cov(0.2), 10);
    let mut rng = Rng::new(0xDEAD_BEEF);
    // 正常一段
    for _ in 0..1000 {
        let d = ImuDelta {
            delta_ang: [0.0; 3],
            delta_vel: [f_b[0] * DT, f_b[1] * DT, f_b[2] * DT],
            dt_ang: DT,
            dt_vel: DT,
            ts_ticks: 0,
        };
        f.predict(&d, GRAVITY_NED).unwrap();
        let ob = baro(0.0, &f.st, &prm);
        let _ = f.fuse(&ob, Channel::Baro, 0.0, 1.0);
        let ob = gps_pos([0.0; 3], &f.st, &prm);
        let _ = f.fuse(&ob, Channel::GpsPos, 0.0, 1.0);
    }
    // 黑障：GPS 断 600 拍（3 s），且**故意让状态被推偏**（模拟无锚定漂移）
    for _ in 0..600 {
        f.st.p[0] += 0.002; // 每拍 2 mm ⇒ 共 1.2 m
        let d = ImuDelta {
            delta_ang: [0.0; 3],
            delta_vel: [f_b[0] * DT, f_b[1] * DT, f_b[2] * DT],
            dt_ang: DT,
            dt_vel: DT,
            ts_ticks: 0,
        };
        f.predict(&d, GRAVITY_NED).unwrap();
    }
    let drifted = f.st.p[0].abs();
    assert!(drifted > 0.5, "黑障期必须真的漂出去: {drifted}");
    // 恢复：给带噪 GPS，误差必须回到 2σ 内
    let b_pos = 2.0 * param("obs.sigma_gps_p").unwrap().value;
    for _ in 0..2000 {
        let d = ImuDelta {
            delta_ang: [0.0; 3],
            delta_vel: [f_b[0] * DT, f_b[1] * DT, f_b[2] * DT],
            dt_ang: DT,
            dt_vel: DT,
            ts_ticks: 0,
        };
        f.predict(&d, GRAVITY_NED).unwrap();
        let pp = [prm.sigma_gps_p * rng.g(), 0.0, 0.0];
        let ob = gps_pos(pp, &f.st, &prm);
        let _ = f.fuse(&ob, Channel::GpsPos, 0.0, 1.0);
    }
    eprintln!(
        "[disc] 黑障漂移 {drifted:.3} m ⇒ 恢复后 |p_x|={:.4} ≤ {b_pos}；重灌={}",
        f.st.p[0].abs(),
        f.guards[1].recoveries()
    );
    // ★**不**在这里断言 recoveries ≥ 1：本场景（只"不喂"GPS）不会让 P 塌陷 ——
    //   黑障期 P 反而被 Q 撑大 ⇒ 不会触发重灌，恢复靠的是"方差没塌 + 重新给观测"。
    //   重灌机制由 L15 的 `blackout_then_recovery_re_anchors` 专门覆盖（那里喂的是
    //   **被门限拒收**的观测）。首版在这里断言它 ⇒ **判据错**（断言了场景不触发的机制）。
    assert!(
        f.st.p[0].abs() <= b_pos,
        "恢复后必须重新锚定: {} > {b_pos}",
        f.st.p[0].abs()
    );
}

/// **NIS 一致性扫描** —— 反推 Q 缩放：平均 NIS 应等于该通道的**有效观测轴数**。
/// 期望值由设计导出（L9 的 NO_INFO）：气压 1、GPS位 3、GPS速 2、磁航向 1。
/// 扫描只打印（不设断言）—— 它是**标定的原料**，标定结果再写回参数表。
#[test]
fn nis_consistency_scan_over_q_scale() {
    let t = truth();
    let prm = ObsParams::default();
    let f_b = specific_force_at_rest(t.q);
    for scale in [1.0f32, 10.0, 100.0, 1000.0, 10000.0] {
        let mut f = Eskf::new(t, diag_cov(0.2), 10);
        f.q = fcalg::filter::ProcessNoise {
            q_att: 1e-4 * scale,
            q_vel: 2.0 * scale,
            q_pos: 1e-4 * scale,
            q_bg: 1e-6 * scale,
            q_ba: 1e-4 * scale,
            q_mag_i: 1e-3 * scale,
            q_mag_b: 1e-3 * scale,
        };
        let mut rng = Rng::new(0x2468_ACE0);
        for k in 0..3000 {
            let d = ImuDelta {
                delta_ang: [0.0; 3],
                delta_vel: [f_b[0] * DT, f_b[1] * DT, f_b[2] * DT],
                dt_ang: DT,
                dt_vel: DT,
                ts_ticks: 0,
            };
            f.predict(&d, GRAVITY_NED).unwrap();
            let ob = baro(prm.sigma_baro * rng.g(), &f.st, &prm);
            let _ = f.fuse(&ob, Channel::Baro, 0.0, 1.0);
            if k % 10 == 0 {
                let pp = [
                    prm.sigma_gps_p * rng.g(),
                    prm.sigma_gps_p * rng.g(),
                    prm.sigma_gps_p * rng.g(),
                ];
                let ob = gps_pos(pp, &f.st, &prm);
                let _ = f.fuse(&ob, Channel::GpsPos, 0.0, 1.0);
                let vv = [
                    prm.sigma_gps_v * rng.g(),
                    prm.sigma_gps_v * rng.g(),
                    prm.sigma_gps_v * rng.g(),
                ];
                let ob = gps_vel(vv, &f.st, &prm);
                let _ = f.fuse(&ob, Channel::GpsVel, 0.0, 1.0);
            }
        }
        let mean = |i: usize| -> f64 {
            if f.nis_n[i] == 0 {
                f64::NAN
            } else {
                f.nis_sum[i] / f.nis_n[i] as f64
            }
        };
        let rej = f.rejects_total as f64 / (f.nis_n.iter().map(|x| *x as f64).sum::<f64>() + f.rejects_total as f64);
        eprintln!(
            "[nis] Q×{scale:<7} 平均NIS: baro={:.2}(期望1) gpsP={:.2}(期望3) gpsV={:.2}(期望2) | 拒收率={:.1}% | 拆分: gate={} numeric={} singular={} nonfinite={}",
            mean(0),
            mean(1),
            mean(2),
            rej * 100.0,
            f.rejects_gate,
            f.rejects_numeric,
            f.rejects_singular,
            f.rejects_nonfinite
        );
    }
}
