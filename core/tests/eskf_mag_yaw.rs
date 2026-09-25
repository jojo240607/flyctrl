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
