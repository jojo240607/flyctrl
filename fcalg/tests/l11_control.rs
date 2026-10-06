//! L11 验收 —— 合成状态直接驱动（**不依赖估计器**，契约 §0）：
//! 姿态环短弧与解析一致、速率环对一阶被控对象的闭式响应、混控的单位增益/
//! 推力保持/**饱和不反向**/满秩单射。
use fcalg::attitude::attitude_rate_setpoint;
use fcalg::mixer::{x4_mix, X4SIGNS};
use fcalg::quat::Quat;
use fcalg::rate::{rate_p_step, RateGains};
/// 姿态环小角：`ω_sp` 必须**精确**等于 `kp·log_rot(q_sp)`（约定判据），
/// 并在**极小角**下退化到 `kp·欧拉角`（解析极限 ✓ —— 只有极小角才成立：
/// `from_euler_zyx` 的等价旋转向量与欧拉角之差是二阶项 θ²/2，
/// 首版用 θ≈0.06 断言 1e-4 容差，被这一项（~0.0018）拦下 —— 是我期望值不对，不是模块错）。
#[test]
fn attitude_small_angle_matches_analytic() {
    let kp = 4.0f32;
    let d = [0.05f32, -0.03, 0.02];
    let q_sp = Quat::from_euler_zyx(d);
    let w = attitude_rate_setpoint(Quat::IDENTITY, q_sp, kp);
    // ① 精确：约定判据（机体系统对误差的对数 × kp）
    let exact = fcalg::error_state::log_rot(q_sp);
    for i in 0..3 {
        assert!((w[i] - kp * exact[i]).abs() < 1e-6, "约定须精确 @{i}: {} vs {}", w[i], kp * exact[i]);
    }
    // ② 解析极限：极小角下 = kp·欧拉角
    let tiny = [5e-4f32, -3e-4, 2e-4];
    let w2 = attitude_rate_setpoint(Quat::IDENTITY, Quat::from_euler_zyx(tiny), kp);
    for i in 0..3 {
        assert!((w2[i] - kp * tiny[i]).abs() < 1e-5, "极小角须退化到解析 @{i}: {}", w2[i]);
    }
}
/// 姿态环**短弧**：180° 附近不得出现方向翻转或幅值爆炸（旧栈吃过 "roll 翻 π"）。
#[test]
fn attitude_shortest_arc_near_pi() {
    let kp = 1.0f32;
    let mut prev = 0.0f32;
    for deg in [150.0f32, 170.0, 179.0, 181.0, 190.0, 210.0] {
        let ang = deg.to_radians();
        let q_sp = Quat::from_axis_angle([0.0, 0.0, 1.0], ang);
        let w = attitude_rate_setpoint(Quat::IDENTITY, q_sp, kp);
        // 短弧：误差幅值必须 ≤ π（否则就是走了长弧）
        let mag = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
        assert!(mag <= std::f32::consts::PI + 1e-3, "{deg}° 误差幅值越界: {mag}");
        // 连续变化：相邻角度之间不得跳变（>0.5 rad 视为跳变）
        if prev > 0.0 {
            assert!((mag - prev).abs() < 0.5, "{deg}° 附近出现跳变: {prev} → {mag}");
        }
        prev = mag;
        // z 分量符号必须与短弧方向一致（绕 z 正转 ⇒ 正）
        if deg < 180.0 {
            assert!(w[2] > 0.0, "{deg}° 绕 z 应取正向短弧");
        } else {
            assert!(w[2] < 0.0, "{deg}° 应取反向短弧（不得翻转成 +π）");
        }
    }
}
/// 速率环闭式响应：一阶被控对象 `J·ω̇ = k·u` + P 控制 ⇒ `ω = sp·(1 − e^(−t/τc))`。
#[test]
fn rate_loop_matches_first_order_closed_form() {
    let (j, k, kp) = (0.01f32, 1.0f32, 20.0f32);
    let sp = 2.0f32;
    let g = RateGains { kp, ki: 0.0 };
    let dt = 1e-4f32;
    let tau_c = j / (k * kp);
    let mut w = 0.0f32;
    for n in 0..20000 {
        let u = rate_p_step(&g, sp, w).unwrap();
        w += (k * u / j) * dt;
        if n % 5000 == 4999 {
            let t = dt * (n + 1) as f32;
            let want = sp * (1.0 - (-t / tau_c).exp());
            assert!((w - want).abs() < 0.02, "t={t}: 实测 {w} vs 闭式解 {want}");
        }
    }
}
/// 混控单位增益：零力矩 ⇒ 四电机等于请求推力；均值恒等于推力。
#[test]
fn mixer_unit_gain_and_thrust_preserved() {
    for thrust in [0.0f32, 0.25, 0.5, 0.75, 1.0] {
        let c = x4_mix(thrust, [0.0; 3]).unwrap();
        for m in 0..4 {
            assert_eq!(c.0[m], thrust, "零力矩下每桨都应等于推力");
        }
        assert_eq!(c.mean(), thrust);
    }
    // 小幅力矩：均值必须仍等于推力（推力被保留）
    let c = x4_mix(0.5, [0.05, -0.03, 0.02]).unwrap();
    assert!((c.mean() - 0.5).abs() < 1e-6, "小幅力矩下推力必须精确保留");
}
/// **饱和不反向**：力矩再大，实际产生的力矩符号也不得与请求相反。
#[test]
fn mixer_never_reverses_torque_on_saturation() {
    for &thrust in &[0.0f32, 0.1, 0.5, 0.9, 1.0] {
        for &tq in &[0.2f32, 0.6, 1.5, 4.0] {
            for axis in 0..3 {
                let mut req = [0.0f32; 3];
                req[axis] = tq;
                let c = x4_mix(thrust, req).unwrap();
                let got = c.torque();
                assert!(
                    got[axis] >= 0.0,
                    "轴{axis} 请求 +{tq}（thrust={thrust}）却产生反向力矩 {}",
                    got[axis]
                );
                for m in 0..4 {
                    assert!((0.0..=1.0).contains(&c.0[m]), "指令必须落在 [0,1]: {}", c.0[m]);
                }
            }
        }
    }
}
/// 结构不变量：三行符号两两正交、各两正两负 ⇒ 与推力行合并满秩（映射单射）。
#[test]
fn mixer_sign_table_is_full_rank() {
    for a in 0..3 {
        let pos = X4SIGNS[a].iter().filter(|v| **v > 0.0).count();
        let neg = X4SIGNS[a].iter().filter(|v| **v < 0.0).count();
        assert_eq!((pos, neg), (2, 2), "第 {a} 行必须两正两负");
        for b in (a + 1)..3 {
            let dot: f32 = (0..4).map(|m| X4SIGNS[a][m] * X4SIGNS[b][m]).sum();
            assert_eq!(dot, 0.0, "第 {a} 行与第 {b} 行必须正交");
        }
    }
    // 单射：不同的力矩指令必须给出不同的电机向量
    let a = x4_mix(0.5, [0.1, 0.0, 0.0]).unwrap();
    let b = x4_mix(0.5, [0.05, 0.04, 0.02]).unwrap();
    assert_ne!(a, b, "不同力矩不得映射到同一电机指令");
}
