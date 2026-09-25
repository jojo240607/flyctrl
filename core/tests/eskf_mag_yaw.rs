// ★§5.136 方案 B 判据：已知航向偏移下 update_mag_yaw 应收敛（方向正确性）
use flyctrl_core::estimator::eskf::Eskf;
use flyctrl_core::vehicle::Quaternion;
use flyctrl_core::units::Radian;

fn yaw_of(q: &Quaternion) -> f32 { q.yaw() }

#[test]
fn yaw_only_update_drives_estimate_toward_measurement() {
    for truth_deg in [-10.0f32, -3.0, 3.0, 10.0] {
        // 估计从 yaw=0 起；实测磁场 = 世界场[0.2,0,0.4] 经真实姿态（yaw=truth）转到机体
        let q_true = Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(truth_deg.to_radians()));
        let mag_i = [0.2f32, 0.0, 0.4];
        // m_b = R(q_true)^T · mag_i
        let m_b = flyctrl_core::vehicle::rotate_vec_by_quat_inverse(q_true, mag_i);
        let mut f = Eskf::new(Quaternion::from_axis_angle([0.0,0.0,1.0], Radian(0.0)), [0.0;3], [0.0;3], 5.0);
        f.mag_i = mag_i; f.mag_b = [0.0;3];
        let y0 = yaw_of(&f.st.q);
        for _ in 0..200 { let _ = f.update_mag_yaw(m_b); }
        let y1 = yaw_of(&f.st.q);
        println!("truth={truth_deg:+.1}° yaw: {:.2}° → {:.2}°", y0.to_degrees(), y1.to_degrees());
        // 收敛方向：y1 应朝 truth 靠近（且幅度 > 50%）
        let err0 = (truth_deg.to_radians() - y0).abs();
        let err1 = (truth_deg.to_radians() - y1).abs();
        assert!(err1 < err0 * 0.5, "truth={truth_deg}° 未收敛：{:.2}°→{:.2}°（err {:.3}→{:.3}）",
            y0.to_degrees(), y1.to_degrees(), err0, err1);
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

/// 对照（文档化）：**未对准**时恒新息不可分辨 ⇒ 估计收敛到错误航向（补偿量 ≈ 失配角）。
/// 保留该测例以防"静默退回未对准路径"（若将来路径变化，此处应显式更新并说明）。
#[test]
fn yaw_only_without_alignment_is_unresolvable_documented() {
    let d = (-20.0f32).to_radians();
    let (sd, cd) = (d.sin(), d.cos());
    let mag_world = [0.2 * cd - 0.0 * sd, 0.2 * sd + 0.0 * cd, 0.4];
    let mut f = Eskf::new(Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0)), [0.0; 3], [0.0; 3], 5.0);
    f.mag_i = [0.2f32, 0.0, 0.4];
    f.mag_b = [0.0; 3];
    for _ in 0..3000 {
        let _ = f.update_mag_yaw(mag_world);
    }
    let yaw = f.st.q.yaw().to_degrees();
    println!("[对照] 未对准 decl=-20° ⇒ est yaw={yaw:+.2}°（≈失配角的补偿 ⇒ 故必须对准 ✓）");
    assert!((yaw - 20.0).abs() < 6.0, "未对准路径行为已变：{yaw:.2}°（此前 ≈+19°）");
}
