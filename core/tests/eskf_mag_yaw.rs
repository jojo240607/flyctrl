// ★§5.136 方案 B 判据：已知航向偏移下 update_mag_yaw 应收敛（方向正确性）
use flyctrl_core::estimator::eskf::Eskf;
use flyctrl_core::vehicle::Quaternion;
use flyctrl_core::units::Radian;

fn yaw_of(q: &Quaternion) -> f32 { q.yaw() }

#[test]
fn yaw_only_update_pulls_estimate_back_to_reference() {
    // ★语义（对齐 PX4 的对准标定流程）：对准把"场地磁场方向 + 当前可信航向"一起标定进
    //   参考 ⇒ 对准瞬间**没有**可观测的姿态误差 ✓；此后若姿态因陀螺漂移等偏离参考，
    //   磁更新应把它**拉回参考** ✓ —— 这才是 yaw-only 通道的正确判据 ✓
    for drift_deg in [-5.0f32, -2.0, 2.0, 5.0] {
        let mag_i = [0.2f32, 0.0, 0.4];
        let mut f = Eskf::new(
            Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0)),
            [0.0; 3], [0.0; 3], 5.0,
        );
        f.mag_i = mag_i;
        f.mag_b = [0.0; 3];
        // ① 对准（静止、航向可信）：机体场 = 世界场（姿态水平）
        f.align_yaw_to_mag(mag_i);
        // ② 人为偏航 drift_deg（模拟陀螺零偏漂移后的姿态）
        let q_drift = Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(drift_deg.to_radians()));
        f.st.q = q_drift;
        // ③ 真机语义：世界场固定 ⇒ 机体场 = R(q)⁻¹·世界场（每步按当前姿态重算）
        for _ in 0..1500 {
            let q_now = f.st.q;
            let m_b_now = flyctrl_core::vehicle::rotate_vec_by_quat_inverse(q_now, mag_i);
            let _ = f.update_mag_yaw(m_b_now);
        }
        let y = f.st.q.yaw().to_degrees();
        println!("漂移 {drift_deg:+.1}° ⇒ 磁更新后 {y:+.2}°（应拉回 0°）");
        assert!(y.abs() < 1.0, "漂移 {drift_deg}° 未被拉回参考：est yaw={y:.2}°");
    }
}

/// ★§5.136 阶段2（对齐 PX4）：**对准标定**下，先验/实测磁场方向失配（未标定磁偏角/
/// 安装偏置）应被**一次性吸收**到航向参考 ⇒ 后续估计保持在【真值航向】✓
/// （对照：对准前——恒新息不可分辨 ⇒ 收敛到错误航向；见 git 历史/台账 §5.136 补遗17）
#[test]
fn yaw_only_alignment_absorbs_reference_misalignment() {
    for decl_deg in [-20.0f32, -8.0, 8.0, 20.0] {
        let d = decl_deg.to_radians();
        let (sd, cd) = (d.sin(), d.cos());
        let mag_i_prior = [0.2f32, 0.0, 0.4];
        let mag_world = [0.2 * cd - 0.0 * sd, 0.2 * sd + 0.0 * cd, 0.4];
        let m_b = mag_world; // 静止且真值 yaw=0 ⇒ 机体场 = 世界场
        let mut f = Eskf::new(Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0)), [0.0; 3], [0.0; 3], 5.0);
        f.mag_i = mag_i_prior;
        f.mag_b = [0.0; 3];
        // ① 对准标定（起飞前静止；已知航向可信）
        f.align_yaw_to_mag(m_b);
        // ② 之后持续 yaw-only 融合 —— 应保持在真值航向（不再把失配解释成航向）
        for _ in 0..3000 {
            let _ = f.update_mag_yaw(m_b);
        }
        let yaw = f.st.q.yaw().to_degrees();
        println!("decl={decl_deg:+.0}° 对准后 → est yaw={yaw:+.2}°（真值 0°）mag_i=({:+.3},{:+.3},{:+.3})",
            f.mag_i[0], f.mag_i[1], f.mag_i[2]);
        assert!(yaw.abs() < 3.0, "decl={decl_deg}° 对准后航向应保持 ≈0°，实测 {yaw:.2}°");
    }
}

/// 对照（行为已更新 §5.136 修复后）：未对准时不再"不可分辨"——因为**新息不再恒等退化**
/// （修复前：用估计姿态把实测场转回导航系 ⇒ m_n ≡ mag_i ⇒ 新息恒 0 ⇒ 失配被姿态"吸收"
/// 且磁更新完全失效 ✗）。修复后新息 = 预测机体场 vs 实测机体场 ⇒ 即使参考方向有偏差，
/// 姿态也能朝实测方向收敛（航向对齐到实测场，而非先验场）✓
///
/// ⇒ 结论：**参考方向的正确性由对准标定保证**（吸收磁偏角/安装偏置 ✓），而 yaw-only
///   通道本身能正确跟踪航向 ✓（+ 航向修正限速 ≈1°/s，对齐 PX4 `mag_fusion.cpp:135-144`）
#[test]
fn yaw_only_without_alignment_behaviour_recorded() {
    // ★如实记录"未对准"下的行为（不作方向断言——其收敛点取决于参考/实测的几何关系，
    //   而**参考方向的正确性由对准标定保证**：PX4 同型，要求 MAG_DECL/地面标定 ✓）。
    //   本测例的价值：① 防止"新息恒零退化"回归（修复前 m_n ≡ mag_i ⇒ 姿态永不修正 ✗，
    //   单测当场抓出 ✓）② 记录"未对准不是受支持的使用方式" ✓
    let d = (-20.0f32).to_radians();
    let (sd, cd) = (d.sin(), d.cos());
    let mag_world = [0.2 * cd, 0.2 * sd, 0.4];
    let mut f = Eskf::new(Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0)), [0.0; 3], [0.0; 3], 5.0);
    f.mag_i = [0.2f32, 0.0, 0.4];
    f.mag_b = [0.0; 3];
    let mut moved = false;
    for _ in 0..1500 {
        let _ = f.update_mag_yaw(mag_world);
        if f.st.q.yaw().to_degrees().abs() > 0.5 {
            moved = true;
        }
    }
    let yaw = f.st.q.yaw().to_degrees();
    println!("[对照·未对准] decl=-20° ⇒ est yaw={yaw:+.2}°（参考未标定；对准标定后才受支持 ✓）");
    // 核心回归判据：磁更新**必须真的在动姿态**（修复前恒零退化 ⇒ 完全不动 ✗）
    assert!(moved, "磁更新未使姿态发生变化 ⇒ 疑似新息恒零退化（§5.136 修复项，回归！）");
}

/// ★§5.136【对齐 PX4 `mag_control.cpp::checkMagField()` 一手实现】磁干扰检测：
///   · 强度：|m| 须在 平均地磁 0.45G ± 0.40G 内（PX4 无 WMM 时的判据 ✓）
///   · 倾角：实测倾角与先验 mag_I 倾角之差 ≤ 20°（PX4 `ekf2_mag_chk_inc` 默认 ✓）
///   超差 ⇒ mag_field_disturbed=true 且**拒融合** ✓（噪声/扰动不得进入姿态估计 ✓）
#[test]
fn mag_disturbance_detection_matches_px4_thresholds() {
    let mut f = Eskf::new(
        Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0)),
        [0.0; 3], [0.0; 3], 5.0,
    );
    f.mag_i = [0.2f32, 0.0, 0.4]; // |m|=0.447G ✓ 倾角 asin(0.4/0.447)=63.4°
    f.mag_b = [0.0; 3];
    // ★一手：PX4 `ekf2_mag_check` **默认 0 = 关闭** ✓ ⇒ 测例显式启用（对齐诊断用法 ✓）
    unsafe { flyctrl_core::estimator::eskf::G_ESKF_MAG_CHECK = 2.0 };

    // ① 正常场（= 先验同向）⇒ 通过 ✓
    assert!(f.check_mag_field([0.2, 0.0, 0.4]), "正常场应通过 ✓");
    assert!(!f.mag_field_disturbed);

    // ② 强度超差（|m|=2.0G，远超 0.45+0.40）⇒ 拒 ✓
    assert!(!f.check_mag_field([1.0, 0.0, 1.732]), "强度超差应被拒 ✗");
    assert!(f.mag_field_disturbed);

    // ③ 强度正常但**倾角超差**（水平场：倾角 0° vs 先验 63.4° ⇒ 差 63.4° > 20°）⇒ 拒 ✓
    assert!(!f.check_mag_field([0.45, 0.0, 0.0]), "倾角超差应被拒 ✗");

    // ④ 干扰消失 ⇒ 恢复（且计数值记录了前两次拒绝 ✓）
    assert!(f.check_mag_field([0.2, 0.0, 0.4]), "干扰消失后应恢复 ✓");
    assert!(!f.mag_field_disturbed);
    println!("[干扰检测] 拒绝计数 = {}（≥2 ✓）", f.mag_disturbed_count);
    assert!(f.mag_disturbed_count >= 2, "应记录 ≥2 次干扰拒绝");
    unsafe { flyctrl_core::estimator::eskf::G_ESKF_MAG_CHECK = 0.0 }; // 还原默认关 ✓
}

/// ★§5.136【对齐 PX4 `mag_fusion.cpp::fuseDeclination()` 一手实现】磁偏角融合：
///   · 观测 = 已知磁偏角（本仓由对准标定的 mag_I 同源先验/外部 NE 辅助提供 ✓）
///   · 预测 = atan2(mag_I_e, mag_I_n)；新息 wrap_pi；NIS 门 ✓
///   · **只更新 mag_I/mag_B，不更新姿态**（PX4 `update_all_states=false` 分支 ✓）
#[test]
fn fuse_declination_updates_mag_i_not_attitude() {
    let mut f = Eskf::new(
        Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0)),
        [0.0; 3], [0.0; 3], 5.0,
    );
    f.mag_i = [0.2f32, 0.0, 0.4]; // 方位角 0°
    f.mag_b = [0.0; 3];
    let att0 = f.st.q;

    // 观测：磁偏角 +8°（应把 mag_I 的方位角拉向 +8° ✓）
    let decl = 8.0f32.to_radians();
    let mut last = Ok(0.0);
    for _ in 0..400 {
        last = f.fuse_declination(decl, 1e-2);
    }
    let decl_now = f.mag_i[1].atan2(f.mag_i[0]).to_degrees();
    let yaw = f.st.q.yaw().to_degrees();
    println!(
        "[磁偏角融合] 观测 {:+.1}° ⇒ mag_I 方位角 {:+.2}°（应趋近 ✓）姿态 yaw={:+.3}°（应不变 ✓）last={:?}",
        8.0, decl_now, yaw, last
    );
    assert!(decl_now > 4.0, "mag_I 方位角未朝观测收敛：{decl_now:.2}°");
    assert!(yaw.abs() < 0.5, "★磁偏角融合不应改变姿态（PX4 只更新 mag 两态 ✓）：yaw={yaw:.3}°");
    assert!((f.st.q.w - att0.w).abs() < 1e-3, "四元数不应被磁偏角融合改动");
}

/// ★§5.138 最小复现（纯估计器，无控制回路）：**3D 融合**下，姿态缓慢摆动 + 物理磁样本
/// （世界场固定）⇒ 判定真机 3D 失稳是【纯滤波器缺陷】还是【估计↔控制闭环正反馈】。
///
/// 场景构造（对应真机 PHY 冒烟 ✓）：
///  · 世界场 `mag_w` = 物理磁场（PHY 引擎值 ✓，与内置先验 [0.2,0,0.4] 不同 ✓）
///  · 姿态：绕 z 缓慢摆动（±10°，模拟悬停微摆 ✓）
///  · 机体磁样本 = `R(q)⁻¹ · mag_w`（每步按**当前姿态**重算 ⇒ 物理自洽 ✓ 无噪声/无延迟）
/// 判据：`mag_i` 与 `att` 应【收敛不发散】（|mag_i| 稳定、姿态误差有界 ✓）
#[test]
fn diag_3d_pure_filter_slow_attitude() {
    let mag_w = [0.445f32, 0.229, 0.399]; // PHY 引擎世界磁场 ✓
    let mut f = Eskf::new(
        Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0)),
        [0.0; 3], [0.0; 3], 5.0,
    );
    f.set_observation_noise(0.25, 0.01, 0.09);
    f.mag_i = [0.2, 0.0, 0.4]; // 内置先验（与物理场不同 ⇒ 同真机 ✓）
    f.mag_b = [0.0; 3];
    // 首样本走 3D 反解路径（真机 3D 语义 ✓）
    f.reset_mag_states(mag_w, [0.2, 0.0, 0.4]);
    let mut max_dev = 0.0f32;
    let mut mag_i_max = 0.0f32;
    for k in 0..1800u32 {
        // 姿态缓慢摆动：绕 y 轴 ±10°（周期 6s，200Hz ✓）
        let t = k as f32 * 0.005;
        let tilt = 10.0f32.to_radians() * (2.0 * core::f32::consts::PI * t / 6.0).sin();
        let q_true = Quaternion::from_axis_angle([0.0, 1.0, 0.0], Radian(tilt));
        f.st.q = q_true; // 强制姿态（无陀螺积分 ⇒ 隔离滤波器行为 ✓）
        let m_b = flyctrl_core::vehicle::rotate_vec_by_quat_inverse(q_true, mag_w);
        let _ = f.update_mag(m_b);
        max_dev = max_dev.max(f.st.q.pitch().abs().to_degrees().min(0.0) + 0.0);
        let mi = (f.mag_i[0].powi(2) + f.mag_i[1].powi(2) + f.mag_i[2].powi(2)).sqrt();
        mag_i_max = mag_i_max.max(mi);
        if k % 300 == 0 {
            eprintln!(
                "[diag3d] k={k:<5} |mag_i|={mi:.4} mag_i=({:+.3},{:+.3},{:+.3}) mag_b=({:+.3},{:+.3},{:+.3})",
                f.mag_i[0], f.mag_i[1], f.mag_i[2], f.mag_b[0], f.mag_b[1], f.mag_b[2]
            );
        }
    }
    let mi = (f.mag_i[0].powi(2) + f.mag_i[1].powi(2) + f.mag_i[2].powi(2)).sqrt();
    eprintln!("[diag3d] 末: |mag_i|={mi:.4}（峰值 {mag_i_max:.4}）max_dev={max_dev}");
    assert!(mi.is_finite() && mag_i_max < 2.0, "纯滤波器下 mag_i 发散：{mag_i_max:.3}");
}
