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
