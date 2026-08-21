//! 姿态内环与电机混控（多旋翼控制器共享内核，P3-A3 提取）。
//!
//! 多个控制器（PID / TECS / …）共用同一姿态内环与 X 型四旋翼混控，
//! 保证不同外环算法之间的"内环行为完全一致"，便于公平对比与复用。
//! 此模块代码由 `pid.rs` 原内环/混控原样提取（行为不变，仅消除重复）。

use crate::vehicle::Quaternion;

/// 姿态内环输出：期望机体角速度 + 姿态误差旋转向量（供调试）。
pub struct AttitudeOut {
    /// 期望机体角速度 (p_cmd, q_cmd, r_cmd)。
    pub rates: [f32; 3],
    /// 姿态误差旋转向量（机体系，≈ 2·sign(w)·(x,y,z)，调试用）。
    pub err: [f32; 3],
}

/// 姿态内环：四元数姿态误差 -> 期望机体角速度（PD，无欧拉角奇点）。
///
/// q_err = q_est⁻¹ ⊗ q_des（机体坐标系下的误差旋转）；误差旋转向量 ≈ 2·sign(w)·(x,y,z)；
/// 期望机体角速度 = Kp_att·误差向量 - Kd_att·当前角速度（阻尼）。
pub fn attitude_rates(
    est_att: Quaternion,
    q_des: Quaternion,
    att_kp: f32,
    att_kd: f32,
    omega: [f32; 3],
) -> AttitudeOut {
    let q_err = crate::vehicle::quat_mul(crate::vehicle::quat_conj(est_att), q_des);
    let sgn = if q_err.w < 0.0 { -2.0 } else { 2.0 };
    let ex_b = sgn * q_err.x;
    let ey_b = sgn * q_err.y;
    let ez_b = sgn * q_err.z;
    AttitudeOut {
        rates: [
            att_kp * ex_b - att_kd * omega[0],
            att_kp * ey_b - att_kd * omega[1],
            att_kp * ez_b - att_kd * omega[2],
        ],
        err: [ex_b, ey_b, ez_b],
    }
}

/// X 型四旋翼混控：总推力 + 三轴机体角速度 -> 4 路归一化油门（未限幅）。
///
/// 布局 0=前右 1=后左 2=前左 3=后右；spin 0,1 CCW / 2,3 CW：
///   τx = l(m0+m3-m1-m2)  τy = l(m0+m2-m1-m3)  τz = k(m0+m1-m2-m3)
/// 解得 m0 = T + 0.5(p+q+r)，…
/// 符号修正（open_loop_torque_sign_probe 实测，2026-08-21）：yaw 项取 +r_cmd，
/// roll/pitch 保持原符号（见 pid.rs 混控注释）。
pub fn x4_mix(des_thrust: f32, pqr: [f32; 3]) -> [f32; 4] {
    let [p_cmd, q_cmd, r_cmd] = pqr;
    [
        des_thrust + 0.5 * (p_cmd + q_cmd + r_cmd),
        des_thrust + 0.5 * (-p_cmd - q_cmd + r_cmd),
        des_thrust + 0.5 * (-p_cmd + q_cmd - r_cmd),
        des_thrust + 0.5 * (p_cmd - q_cmd - r_cmd),
    ]
}
