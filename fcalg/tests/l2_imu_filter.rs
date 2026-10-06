//! L2 验收 —— 只用契约 §6 允许的判据：
//! 解析频响、直流单位增益、解析群延迟、极限/退化、以及"状态不被污染"这条不变量。

use fcalg::biquad::{Biquad, Coeffs};
use fcalg::finite::{reset, violations, Stage, Violation};
use fcalg::imu_filter::ImuFilter;

const FS: f32 = 1000.0;

fn mag((re, im): (f32, f32)) -> f32 {
    (re * re + im * im).sqrt()
}

/// 解析频响：低通直流增益必须恰为 1（定义性性质，与实现无关）。
#[test]
fn lowpass_dc_gain_is_unity() {
    let c1 = Coeffs::lowpass1(FS, 20.0);
    assert!((mag(c1.response(FS, 0.0)) - 1.0).abs() < 1e-5, "一阶低通直流增益须为 1");
    let c2 = Coeffs::lowpass2(FS, 20.0, 0.7071);
    assert!((mag(c2.response(FS, 0.0)) - 1.0).abs() < 1e-4, "二阶低通直流增益须为 1");
}

/// 解析频响：陷波在 f0 处增益必须为 0，且远离 f0 时接近 1。
#[test]
fn notch_kills_f0_and_passes_elsewhere() {
    let c = Coeffs::notch(FS, 40.0, 2.0);
    assert!(mag(c.response(FS, 40.0)) < 1e-4, "陷波中心增益须为 0");
    assert!(mag(c.response(FS, 5.0)) > 0.95, "远离中心须基本通过");
    assert!(mag(c.response(FS, 200.0)) > 0.95, "高端也须基本通过");
}

/// 解析频响：低通高频必须衰减。
#[test]
fn lowpass_attenuates_high_frequency() {
    let c = Coeffs::lowpass2(FS, 20.0, 0.7071);
    assert!(mag(c.response(FS, 200.0)) < 0.05, "高于截止一个十倍频程须强衰减");
    assert!(mag(c.response(FS, 0.5)) > 0.99, "远低于截止须几乎无衰减");
}

/// 群延迟解析对账：一阶低通的冲激响应一阶矩 `Σn·h[n]/Σh[n]` 必须等于 `(1−a)/a`。
/// 同时对照物理量 `1/(2π·fc)`（松容差，验证系数取值符合物理意图）。
#[test]
fn lowpass1_group_delay_matches_analytic() {
    let c = Coeffs::lowpass1(FS, 20.0);
    let a = c.b0;
    let mut f = Biquad::new(c);
    let n = 200_000;
    let (mut s0, mut s1) = (0.0f64, 0.0f64);
    for i in 0..n {
        let x = if i == 0 { 1.0f32 } else { 0.0 };
        let y = f.step(x) as f64;
        s0 += y;
        s1 += i as f64 * y;
    }
    let measured = s1 / s0;
    let analytic = (1.0 - a as f64) / a as f64;
    assert!(
        (measured - analytic).abs() / analytic < 0.01,
        "群延迟须符解析式: 实测 {measured:.4} vs 解析 {analytic:.4} 样本"
    );
    let physical = (measured / FS as f64) as f32;
    let tau = 1.0 / (2.0 * core::f32::consts::PI * 20.0);
    assert!(
        (physical - tau).abs() / tau < 0.10,
        "群延迟须符合物理量 1/(2πfc)={tau:.5}s vs {physical:.5}s"
    );
}

/// 解析裁判（时域动态）：二阶低通的**过冲**必须符合 `Mp = exp(−πζ/√(1−ζ²))`，`ζ = 1/(2Q)`。
/// 并验证极限：`Q = 0.5`（ζ = 1，临界阻尼）⇒ 无过冲。
/// 注：取 `fc/fs = 0.01` 使双线性变换的频域弯折足够小，模拟原型的解析式可直接对照。
#[test]
fn lowpass2_step_overshoot_matches_analytic() {
    let fc = 10.0f32;
    // Butterworth Q：ζ = 0.7071 ⇒ 解析过冲 ≈ 4.32%
    let q = 0.7071f32;
    let mut f = Biquad::new(Coeffs::lowpass2(FS, fc, q));
    let mut peak = f32::MIN;
    for _ in 0..20_000 {
        peak = peak.max(f.step(1.0));
    }
    let zeta = 1.0 / (2.0 * q);
    let mp = (-core::f32::consts::PI * zeta / (1.0 - zeta * zeta).sqrt()).exp();
    assert!(peak > 0.99, "必须收敛到 1: 峰值 {peak}");
    assert!(
        (peak - 1.0 - mp).abs() < 0.02,
        "过冲须符解析式: 实测 {:.4} vs 解析 {:.4}",
        peak - 1.0,
        mp
    );

    // 临界阻尼：Q = 0.5 ⇒ ζ = 1 ⇒ 无过冲
    let mut g = Biquad::new(Coeffs::lowpass2(FS, fc, 0.5));
    let mut peak2 = f32::MIN;
    for _ in 0..20_000 {
        peak2 = peak2.max(g.step(1.0));
    }
    assert!(peak2 > 0.99, "必须收敛到 1: 峰值 {peak2}");
    assert!(peak2 <= 1.001, "ζ=1 不得过冲: 峰值 {peak2}");
}

/// 极限：陷波对**恰好 f0** 的正弦的稳态增益必须极小（比解析频响更严的时域判据）。
#[test]
fn notch_removes_sine_at_f0_in_time_domain() {
    let mut f = Biquad::new(Coeffs::notch(FS, 40.0, 2.0));
    let w = 2.0 * core::f32::consts::PI * 40.0 / FS;
    let (mut acc_out, mut acc_in, mut cnt) = (0.0f64, 0.0f64, 0);
    for i in 0..4000 {
        let x = (w * i as f32).sin();
        let y = f.step(x);
        if i >= 2000 {
            acc_in += (x as f64) * (x as f64);
            acc_out += (y as f64) * (y as f64);
            cnt += 1;
        }
    }
    let atten_db = 10.0 * (acc_in / acc_out.max(1e-30)).log10();
    assert!(cnt > 0 && atten_db > 40.0, "f0 处正弦须被强抑制: {atten_db:.1} dB");
}

/// 不变量（契约 §4）：非有限输入 ⇒ `Err`，且**滤波器状态不被污染**。
/// 判据：与"从未见过 NaN"的同参滤波器在同一有效序列上输出逐位相同。
#[test]
fn non_finite_input_does_not_pollute_state() {
    reset();
    let mut dirty = ImuFilter::new(FS, 40.0, 2.0, 20.0, 0.7071);
    let mut clean = ImuFilter::new(FS, 40.0, 2.0, 20.0, 0.7071);

    let warm = |i: usize| {
        (
            [0.01 * i as f32, -0.02 * i as f32, 0.003 * i as f32],
            [0.1, -0.2, -9.8 + 0.01 * i as f32],
        )
    };
    for i in 0..50 {
        let (g, a) = warm(i);
        dirty.step(g, a).unwrap();
        clean.step(g, a).unwrap();
    }

    let (g, a) = ([0.5, 0.5, 0.5], [0.0, 0.0, f32::NAN]);
    assert_eq!(dirty.step(g, a), Err(Violation::Nan));
    assert!(violations(Stage::L2ImuFilter) > 0, "违规必须被计数");
    assert!(dirty.states_finite(), "状态不得被 NaN 污染");

    for i in 50..120 {
        let (g, a) = warm(i);
        let d = dirty.step(g, a).unwrap();
        let c = clean.step(g, a).unwrap();
        assert_eq!(d, c, "拒绝坏输入后必须与干净滤波器逐位一致（第 {i} 拍）");
    }
}

/// 极限：常数输入 ⇒ 稳态输出等于输入（直流单位增益的时域形式）。
#[test]
fn constant_input_converges_to_itself() {
    let mut f = ImuFilter::new(FS, 40.0, 2.0, 20.0, 0.7071);
    let g = [0.3, -0.4, 0.05];
    let a = [0.2, -1.5, -9.81];
    let mut out = f.step(g, a).unwrap();
    for _ in 0..20_000 {
        out = f.step(g, a).unwrap();
    }
    for k in 0..3 {
        assert!((out.gyro[k] - g[k]).abs() < 1e-4, "陀螺路直流须单位增益: {out:?}");
        assert!((out.accel[k] - a[k]).abs() < 1e-3, "比力路直流须单位增益: {out:?}");
    }
}
