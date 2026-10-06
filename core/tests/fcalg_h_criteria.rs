//! H 场数值判据（(乙) 口径）—— **界由参数表导出**，不是魔数
//!
//! 运行：`cargo test -p flyctrl-core --features fcalg-est --test fcalg_h_criteria`
//!
//! 流程**按固件真实顺序**：静止对齐得到姿态 → `set_initial_attitude` → 逐拍 predict + 观测。
//! （fcalg 目前**没有重力观测** ⇒ 飞行中 tilt 不可观 ⇒ 依赖初始对齐；
//!   见 `fcalg::observe` 模块头"已登记的缺口"。本判据正是在这个前提下立的。）
//!
//! 界 = `k × σ_obs`，`σ` **从 `fcalg::params` 读**；`k = 1` 的依据是
//! "H 场（合成、无噪声）下误差只来自线性化/离散化/数值 ⇒ 应远低于观测噪声量级"。
#![cfg(feature = "fcalg-est")]

use flyctrl_core::estimator::fcalg_bridge::{reexport, FcalgEstimator};
use flyctrl_core::estimator::trait_def::Estimator;
use flyctrl_core::units::{Meter, MeterPerSecond, Second};
use flyctrl_core::vehicle::{PosSample, Quaternion};

const DT: f32 = 0.005;
/// 稳态窗口起点（前段用于收敛）。
const WARM: usize = 2000;
const TOTAL: usize = 3000;

/// (乙) 主判据：H 场式驱动下，稳态窗口的 RMS 误差必须 ≤ 表内 σ（k=1）。
#[test]
fn h_field_estimator_errors_within_table_sigma() {
    let q_true = reexport::Quat::from_euler_zyx([0.15, -0.22, 0.7]).normalize().unwrap();
    let f_b = reexport::specific_force_at_rest(q_true);

    let mut e = FcalgEstimator::new();
    e.set_initial_attitude(Quaternion { w: q_true.w, x: q_true.x, y: q_true.y, z: q_true.z });
    e.set_initial_position([0.0; 3]);

    let (mut sp, mut sv, mut sa, mut n) = (0.0f64, 0.0f64, 0.0f64, 0u32);
    for k in 0..TOTAL {
        e.predict_delta([0.0; 3], [f_b[0] * DT, f_b[1] * DT, f_b[2] * DT], DT, DT);
        e.update_alt(0.0);
        if k % 10 == 0 {
            e.update_fusion(
                Some(PosSample { pos: [Meter(0.0); 3], vel: Some([MeterPerSecond(0.0); 3]) }),
                None,
            );
        }
        if k >= WARM {
            let s = e.state();
            let mut ep = 0.0f64;
            let mut ev = 0.0f64;
            for a in 0..3 {
                ep += (s.pos[a].0 as f64) * (s.pos[a].0 as f64);
                ev += (s.vel[a].0 as f64) * (s.vel[a].0 as f64);
            }
            sp += ep;
            sv += ev;
            sa += (s.pos[2].0 as f64) * (s.pos[2].0 as f64);
            n += 1;
        }
    }
    let (ep, ev, ea) = (
        (sp / n as f64).sqrt() as f32,
        (sv / n as f64).sqrt() as f32,
        (sa / n as f64).sqrt() as f32,
    );
    // ★界从参数表读（改表则判据跟随）
    let b_pos = reexport::param("obs.sigma_gps_p").unwrap().value;
    let b_vel = reexport::param("obs.sigma_gps_v").unwrap().value;
    let b_alt = reexport::param("obs.sigma_baro").unwrap().value;
    eprintln!("[hcrit] RMS pos={ep:.6} ≤ {b_pos} | vel={ev:.6} ≤ {b_vel} | alt={ea:.6} ≤ {b_alt}");
    assert!(ep <= b_pos, "位置 RMS 须 ≤ 表内 σ_gps_p({b_pos}): {ep}");
    assert!(ev <= b_vel, "速度 RMS 须 ≤ 表内 σ_gps_v({b_vel}): {ev}");
    assert!(ea <= b_alt, "高度 RMS 须 ≤ 表内 σ_baro({b_alt}): {ea}");
    assert_eq!(e.not_impl_calls, 0, "本流程只走已实现通路");
}

/// 判据自洽：这三条界必须来自参数表、且**出处不是「本重建选定」欠债**
/// （否则"界"没有独立依据 —— 与 fcalg 侧同一条纪律）。
#[test]
fn h_field_bounds_have_independent_provenance() {
    for name in ["obs.sigma_gps_p", "obs.sigma_gps_v", "obs.sigma_baro"] {
        let m = reexport::param(name).unwrap();
        assert!(m.value.is_finite() && m.value > 0.0);
        assert!(
            !m.source.is_debt(),
            "{name} 作为 H 场判据的依据不得是欠债"
        );
        assert!(
            matches!(m.source, reexport::Source::Primary(_) | reexport::Source::Derived(_)),
            "{name} 须为 Primary/Derived，实测为 {:?}",
            m.source
        );
    }
}
