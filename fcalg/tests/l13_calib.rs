//! L13 验收 —— 严格逆（生成↔反解成对）、退化（奇异必须显式拒绝）、定义式（纯零偏/纯尺度）、
//! 以及有限性纪律。
use fcalg::calib::{CalibError, SensorCalib};
use fcalg::finite::{reset, violations, Stage, Violation};
fn shear_m() -> [[f32; 3]; 3] {
    [[1.02, 0.01, -0.02], [0.0, 0.98, 0.015], [0.03, -0.01, 1.05]]
}
/// 极限：恒等标定是**无操作**。
#[test]
fn identity_is_noop() {
    let c = SensorCalib::identity();
    for x in [[1.0f32, -2.0, 3.0], [0.0, 0.0, -9.8]] {
        assert_eq!(c.apply(x).unwrap(), x);
        assert_eq!(c.raw_from_true(x), x);
    }
}
/// **严格逆**：`apply(raw_from_true(x)) == x`（尺度+安装+零偏同时非平凡）。
#[test]
fn apply_is_exact_inverse_of_generate() {
    let c = SensorCalib::new(shear_m(), [0.01, -0.02, 0.03]).unwrap();
    for x in [
        [1.5f32, -2.0, 9.7],
        [0.0, 0.0, -9.80665],
        [-0.3, 0.4, 0.05],
    ] {
        let back = c.apply(c.raw_from_true(x)).unwrap();
        for i in 0..3 {
            assert!(
                (back[i] - x[i]).abs() < 1e-4,
                "反解必须是生成的严格逆 @{i}: {} vs {}",
                back[i],
                x[i]
            );
        }
    }
}
/// 定义式：纯零偏 ⇒ 输出 = 输入 − b；纯尺度 ⇒ 输出 = 输入 / s。
#[test]
fn pure_bias_and_pure_scale_are_definitional() {
    let b = [0.05f32, -0.03, 0.02];
    let cb = SensorCalib::new(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        b,
    )
    .unwrap();
    let x = [1.0f32, 2.0, 3.0];
    let out = cb.apply(x).unwrap();
    for i in 0..3 {
        assert!((out[i] - (x[i] - b[i])).abs() < 1e-6, "纯零偏反解 @{i}");
    }
    let s = [1.1f32, 0.95, 1.03];
    let cs = SensorCalib::new(
        [[s[0], 0.0, 0.0], [0.0, s[1], 0.0], [0.0, 0.0, s[2]]],
        [0.0; 3],
    )
    .unwrap();
    let out2 = cs.apply(x).unwrap();
    for i in 0..3 {
        assert!((out2[i] - x[i] / s[i]).abs() < 1e-5, "纯尺度反解 @{i}");
    }
    // `scale()` 必须如实报告尺度
    for j in 0..3 {
        assert!((cs.scale()[j] - s[j]).abs() < 1e-6);
    }
    assert_eq!(cb.bias(), b);
}
/// 安装误差：一个纯旋转型的错装矩阵必须被反解回去（方向复原）。
#[test]
fn misalignment_rotation_is_undone() {
    let a = 0.035f32; // ≈2°
    let m = [
        [1.0, -a, 0.0],
        [a, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];
    let c = SensorCalib::new(m, [0.0; 3]).unwrap();
    let x = [0.3f32, -1.2, 9.8];
    let back = c.apply(c.raw_from_true(x)).unwrap();
    for i in 0..3 {
        assert!((back[i] - x[i]).abs() < 1e-4, "错装必须被复原 @{i}: {}", back[i]);
    }
}
/// 退化：`M` 奇异必须**构造时显式拒绝**，而不是运行时放大误差。
#[test]
fn singular_matrix_is_rejected_at_construction() {
    let m = [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [0.0, 0.0, 1.0]];
    assert_eq!(SensorCalib::new(m, [0.0; 3]), Err(CalibError::Singular));
    // 零矩阵（极端退化）
    assert_eq!(SensorCalib::new([[0.0; 3]; 3], [0.0; 3]), Err(CalibError::Singular));
}
/// 纪律：非有限 ⇒ `Err` + 计数（构造与反解两处都要把关）。
#[test]
fn non_finite_is_rejected_and_counted() {
    reset();
    let mut m = shear_m();
    m[0][1] = f32::NAN;
    assert_eq!(
        SensorCalib::new(m, [0.0; 3]),
        Err(CalibError::NonFinite(Violation::Nan))
    );
    let c = SensorCalib::identity();
    assert_eq!(
        c.apply([0.0, f32::INFINITY, 0.0]),
        Err(CalibError::NonFinite(Violation::Inf))
    );
    assert!(violations(Stage::L13Calib) >= 2, "违规必须计数");
}
