//! L4 验收 —— 只用契约 §6 允许的判据：
//! 可解析合成输入（正解 → 反解闭合）、跨模块契约往返、极限/退化、以及"坏输入不产出姿态"。

use fcalg::align::{align_static, AlignConfig, AlignError};
use fcalg::finite::{reset, violations, Stage, Violation};
use fcalg::quat::{specific_force_at_rest, Quat, GRAVITY_NED};

const G: f32 = GRAVITY_NED[2];

/// 按契约 §2 的定义式**正向生成**比力（与被测实现无关）。
fn gen(rpy: [f32; 3]) -> [f32; 3] {
    let theta = rpy[1];
    let phi = rpy[0];
    [
        G * theta.sin(),
        -G * theta.cos() * phi.sin(),
        -G * theta.cos() * phi.cos(),
    ]
}

/// 极限：水平 ⇒ 单位四元数，零偏 = 平均陀螺。
#[test]
fn level_gives_identity_and_bias_is_average() {
    let gyro = [0.01f32, -0.02, 0.003];
    let r = align_static([0.0, 0.0, -G], gyro, AlignConfig::default()).unwrap();
    assert_eq!(r.q, Quat::IDENTITY, "水平比力必须给出单位姿态");
    assert_eq!(r.gyro_bias, gyro, "零偏必须等于静止期平均陀螺");
}

/// 可解析合成输入：正解 → 反解必须闭合（覆盖 tilt 网格）。
#[test]
fn synthetic_tilt_round_trips() {
    for phi in [-1.2f32, -0.3, 0.0, 0.4, 1.1] {
        for theta in [-1.0f32, -0.2, 0.0, 0.25, 0.9] {
            let a = gen([phi, theta, 0.0]);
            let r = align_static(a, [0.0; 3], AlignConfig::default())
                .unwrap_or_else(|e| panic!("({phi},{theta}) 应可对齐: {e:?}"));
            let e = r.q.to_euler_zyx();
            assert!(
                (e[0] - phi).abs() < 1e-4 && (e[1] - theta).abs() < 1e-4,
                "反解须闭合: 期望 ({phi},{theta}) 实测 ({},{})",
                e[0],
                e[1]
            );
        }
    }
}

/// 契约 §2 的跨模块往返：把输入比力经"对齐 ⇒ 契约正向式"回到原值。
#[test]
fn contract_round_trip_through_specific_force() {
    for rpy in [[0.5f32, -0.7, 0.0], [-1.0, 0.3, 0.0], [0.0, 0.0, 0.0]] {
        let a = gen(rpy);
        let r = align_static(a, [0.0; 3], AlignConfig::default()).unwrap();
        let back = specific_force_at_rest(r.q);
        for k in 0..3 {
            assert!(
                (back[k] - a[k]).abs() < 1e-3,
                "比力往返须闭合: {a:?} vs {back:?}"
            );
        }
    }
}

/// 硬性质：yaw 不可观 ⇒ 对齐**不得发明 yaw**（结果 yaw 恒为 0，与输入无关）。
/// 注：判据用容差而非精确 0 —— 性质是"保持在初值 0 的数值噪声内"，
/// 而 `atan2` 的分子是两项相消，f32 下会留下末位残差。
#[test]
fn yaw_is_never_invented() {
    for (phi, theta) in [(0.4f32, -0.5f32), (-0.9, 0.8), (0.0, 0.0)] {
        let r = align_static(gen([phi, theta, 0.0]), [0.0; 3], AlignConfig::default()).unwrap();
        let yaw = r.q.to_euler_zyx()[2];
        assert!(yaw.abs() < 1e-6, "对齐不得发明 yaw（实测 {yaw}）");
    }
}

/// 硬门：非有限输入 ⇒ 拒绝，**且不产出姿态**，违规被计数（契约 §4）。
#[test]
fn non_finite_is_rejected_without_producing_attitude() {
    reset();
    let r = align_static([0.0, f32::NAN, -G], [0.0; 3], AlignConfig::default());
    assert_eq!(r, Err(AlignError::NonFinite(Violation::Nan)));
    let r2 = align_static([0.0, 0.0, -G], [f32::INFINITY, 0.0, 0.0], AlignConfig::default());
    assert_eq!(r2, Err(AlignError::NonFinite(Violation::Inf)));
    assert!(violations(Stage::L4Align) >= 2, "违规必须按阶段计数");
}

/// 量级判据：偏离 ≈g 必须显式拒绝（含零比力这种退化）。
/// 旧栈就是被"比力大于某值就放行"这种松判据放过去的 ⇒ 这里必须是相对容差。
#[test]
fn magnitude_gate_rejects_off_nominal() {
    let cfg = AlignConfig { g_tol_frac: 0.06 };
    // 5% 偏小 ⇒ 放行
    assert!(align_static([0.0, 0.0, -G * 0.95], [0.0; 3], cfg).is_ok());
    // 8% 偏大 ⇒ 拒绝
    assert!(matches!(
        align_static([0.0, 0.0, -G * 1.08], [0.0; 3], cfg),
        Err(AlignError::NotAtRest { .. })
    ));
    // 零比力（退化）⇒ 拒绝，不得除零产生 NaN 姿态
    assert!(matches!(
        align_static([0.0, 0.0, 0.0], [0.0; 3], cfg),
        Err(AlignError::NotAtRest { .. })
    ));
    // 远超重力（明显在机动）⇒ 拒绝
    assert!(matches!(
        align_static([0.0, 0.0, -G * 2.5], [0.0; 3], cfg),
        Err(AlignError::NotAtRest { .. })
    ));
}
