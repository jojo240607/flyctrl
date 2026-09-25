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

/// ★§5.136【已知限制】先验/实测磁场**方向失配**（未标定的磁偏角/安装角）下的行为记录：
/// **恒定航向新息在"参考偏置"与"真实姿态 yaw 误差"之间本质不可分** ⇒ 本实现里姿态
/// 以远高于学习的增益把失配"解释掉" ⇒ 估计收敛到【错误航向】（补偿量 ≈ 失配角）。
///
/// 实测：decl=−20° ⇒ est yaw ≈ +19°（而 mag_i 不动）。
/// ⇒ 对齐 PX4 需要【第二个航向源】做交叉校验（PX4：陀螺积分航向/GSF 航向估计器 +
///    MAG_DECL 配置先验），而非从磁单独学习（§5.136 补遗 17）。
/// 本测例把该行为**固化**（防静默变化），并作为"引入第二航向源后应转为收敛"的靶子。
#[test]
fn yaw_only_reference_misalignment_is_documented_limitation() {
    let d = (-20.0f32).to_radians();
    let (sd, cd) = (d.sin(), d.cos());
    let mag_i_prior = [0.2f32, 0.0, 0.4];
    let mag_world = [0.2 * cd - 0.0 * sd, 0.2 * sd + 0.0 * cd, 0.4];
    let m_b = mag_world; // 静止且真值 yaw=0 ⇒ 机体场 = 世界场
    let mut f = Eskf::new(Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0)), [0.0; 3], [0.0; 3], 5.0);
    f.mag_i = mag_i_prior;
    f.mag_b = [0.0; 3];
    for _ in 0..3000 {
        let _ = f.update_mag_yaw(m_b);
    }
    let yaw = f.st.q.yaw().to_degrees();
    println!("[已知限制] decl=-20° ⇒ est yaw={yaw:+.2}°（真值 0°；补偿量 ≈ 失配角 ✓ 见文档）");
    // 固化当前行为：误差量级 ≈ 失配角（同号、±6° 容差）
    assert!(
        (yaw - 20.0).abs() < 6.0,
        "行为已变化：decl=-20° 下 est yaw={yaw:.2}°（此前 ≈+19°，即把失配解释成航向）         —— 若已引入第二航向源，请把本测例改为断言 |yaw| < 3° ✓"
    );
}
