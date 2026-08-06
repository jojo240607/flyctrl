//! PID 控制器（工程基线，串级：位置外环 -> 姿态内环）。
//!
//! 结构：
//!   外环：位置误差 -> 期望速度（限幅）        (P)
//!   中环：速度误差 -> 期望世界系加速度 -> 期望姿态（四元数）  (P)
//!   内环：四元数姿态误差 -> 期望机体角速度 -> 混控            (PD，无欧拉角奇点)
//! 最终把总推力 + 三轴机体角速度映射到 4 个电机（X 型混控）。
//!
//! 这是与 PX4/APM 同级的工程基线。后续 LQR/MPC 实现将复用同一 `Controller`
//! 接口，在同一仿真场景下直接 PK。

use crate::units::*;
use crate::vehicle::{ActuatorCmd, Quaternion, VehicleState};
use crate::controller::{Controller, trait_def::Setpoint};

pub struct PidController {
    // 位置外环 P：位置误差 -> 期望速度（世界系）
    kp_xy: f32,
    kp_z: f32,
    // 速度中环 P：速度误差 -> 期望加速度（世界系），再映射为倾角/推力
    kv_xy: f32,
    kv_z: f32,
    // 最大速度/倾角限制（防饱和、保稳定）
    vmax_xy: f32,
    vmax_z: f32,
    tilt_max: f32,
    // 姿态内环：四元数误差 -> 机体角速度 的 P（比例）与 D（角速度阻尼）增益
    att_kp: f32,
    att_kd: f32,
    // 推力基值（悬停油门）与重力（用于倾角->加速度映射）
    hover_thrust: f32,
    gravity: f32,
}

impl PidController {
    /// 典型 450mm X 四旋翼参数（后续可移到机型配置）。
    /// 标准串级：pos_err -> 期望速度(限幅) -> vel_err -> 期望加速度 -> 期望姿态(四元数) -> 角速度。
    pub fn default_quad() -> Self {
        Self {
            kp_xy: 0.5,
            kp_z: 0.5,
            kv_xy: 0.8,
            kv_z: 0.8,
            vmax_xy: 2.0,
            vmax_z: 2.0,
            tilt_max: 0.35,
            att_kp: 3.0,
            att_kd: 0.3,
            hover_thrust: 0.5,
            gravity: 9.81,
        }
    }
}

impl Controller for PidController {
    fn control(&mut self, dt: Second, sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        let dt = dt.0;
        let g = self.gravity;

        // --- 外环：位置误差 -> 期望速度（限幅，避免饱和） ---
        let ex = sp.pos[0].0 - est.pos[0].0;
        let ey = sp.pos[1].0 - est.pos[1].0;
        let ez = sp.pos[2].0 - est.pos[2].0;
        let des_vx = clampf(self.kp_xy * ex, -self.vmax_xy, self.vmax_xy);
        let des_vy = clampf(self.kp_xy * ey, -self.vmax_xy, self.vmax_xy);
        let des_vz = clampf(self.kp_z * ez, -self.vmax_z, self.vmax_z);

        // --- 中环：速度误差 -> 期望世界系加速度 ---
        let acc_n = self.kv_xy * (des_vx - est.vel[0].0); // 北向
        let acc_e = self.kv_xy * (des_vy - est.vel[1].0); // 东向
        let acc_d = self.kv_z * (des_vz - est.vel[2].0); // 下垂方向（NED）

        // 高度推力：悬停 + 垂直加速度项（acc_d>0 表示要向下加速，减推力）
        let des_thrust = clampf(
            self.hover_thrust - acc_d / g,
            0.1, 1.0,
        );

        // 期望机体倾角（小角）：北向加速度 -> 俯仰，东向加速度 -> 横滚。
        // 采用四元数误差内环（见下），这里把世界系期望加速度转换为期望姿态四元数。
        let tilt_n = clampf(acc_n / g, -self.tilt_max, self.tilt_max);
        let tilt_e = clampf(acc_e / g, -self.tilt_max, self.tilt_max);

        // 期望姿态四元数：由（roll=tilt_e, pitch=-tilt_n, yaw=sp.yaw）构成。
        // 推导见前：北向加速需 -pitch，东向加速需 +roll。
        let yaw = sp.yaw;
        let q_des = Quaternion::from_euler(Radian(tilt_e), Radian(-tilt_n), yaw);

        // --- 内环：四元数姿态误差 -> 期望机体角速度（标准鲁棒写法，无欧拉角奇点） ---
        // q_err = q_est^-1 ⊗ q_des（机体坐标系下的误差旋转）
        let q_err = crate::vehicle::quat_mul(
            crate::vehicle::quat_conj(est.att), q_des);
        // 误差旋转向量 ≈ 2·sign(w)·(x,y,z)
        let sgn = if q_err.w < 0.0 { -2.0 } else { 2.0 };
        let ex_b = sgn * q_err.x;
        let ey_b = sgn * q_err.y;
        let ez_b = sgn * q_err.z;
        // 期望机体角速度 = Kp_att·误差向量 - Kd_att·当前角速度（阻尼）
        let p_cmd = self.att_kp * ex_b - self.att_kd * est.omega[0].0;
        let q_cmd = self.att_kp * ey_b - self.att_kd * est.omega[1].0;
        let r_cmd = self.att_kp * ez_b - self.att_kd * est.omega[2].0;

        // --- 混控：X 型四旋翼（0=前右 1=后左 2=前左 3=后右） ---
        // 与 physics 约定一致：CCW(0,1) 更高 -> +yaw(r_cmd)；
        //   前(0,2)高 -> +pitch(q_cmd)；右(0,3)高 -> +roll(p_cmd)
        let m0 = des_thrust + 0.5 * (p_cmd + q_cmd + r_cmd);
        let m1 = des_thrust + 0.5 * (-p_cmd - q_cmd + r_cmd);
        let m2 = des_thrust + 0.5 * (-p_cmd + q_cmd - r_cmd);
        let m3 = des_thrust + 0.5 * (p_cmd - q_cmd - r_cmd);

        ActuatorCmd {
            motor: [
                m0.clamp(0.0, 1.0),
                m1.clamp(0.0, 1.0),
                m2.clamp(0.0, 1.0),
                m3.clamp(0.0, 1.0),
            ],
        }
    }

    fn reset(&mut self) {
        // 四元数误差内环无状态积分，无需复位
    }
}

#[inline]
fn clampf(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo { lo } else if v > hi { hi } else { v }
}
