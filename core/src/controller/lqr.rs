//! LQR 姿态/位置控制器（no_std, 无堆分配）。
//!
//! 结构：级联全状态反馈（cascaded LQR）。
//! - 外环：位置/速度误差 -> 期望加速度 -> 期望姿态角（小角几何）+ 期望推力。
//! - 内环：姿态角误差 + 角速度 -> 三轴力矩指令（全状态反馈 `u = -K x`）。
//!
//! 增益以连续时间 LQR 直觉选取（位置环欠阻尼小、姿态环快）。 mixer 与 PID
//! 共用修正后的符号约定（`p_cmd>0 -> +roll`, `q_cmd>0 -> +pitch`）。

use crate::controller::trait_def::{ActuatorCmd, Controller, Setpoint};
use crate::math::atan2;
use crate::units::*;
use crate::vehicle::{VehicleState};

#[inline]
fn clampf(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

// 小角欧拉提取（Z-Y-X：yaw, pitch, roll）。用于闭环反馈。
fn quat_to_rpy(w: f32, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    let roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch = atan2(2.0 * (w * y - x * z), 1.0 - 2.0 * (y * y + z * z));
    let yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (z * z + x * x));
    (roll, pitch, yaw)
}

pub struct LqrController {
    g: f32,
    // 位置环增益
    kp_pos: f32,
    kd_vel: f32,
    // 姿态环增益（全状态反馈）
    k_phi: f32,
    k_theta: f32,
    k_psi: f32,
    k_p: f32,
    k_q: f32,
    k_r: f32,
    tilt_max: f32,
}

impl LqrController {
    pub fn new() -> Self {
        LqrController {
            g: 9.81,
            kp_pos: 0.35,
            kd_vel: 0.9,
            k_phi: 4.5,
            k_theta: 4.5,
            k_psi: 1.5,
            k_p: 1.2,
            k_q: 1.2,
            k_r: 0.6,
            tilt_max: 0.35,
        }
    }

    pub fn default_quad() -> Self {
        Self::new()
    }
}

impl Controller for LqrController {
    fn control(&mut self, _dt: Second, sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        // 位置/速度误差（世界系 NED）
        let ex = sp.pos[0].0 - est.pos[0].0;
        let ey = sp.pos[1].0 - est.pos[1].0;
        let ez = sp.pos[2].0 - est.pos[2].0;
        let evx = sp.vel[0].0 - est.vel[0].0;
        let evy = sp.vel[1].0 - est.vel[1].0;
        let evz = sp.vel[2].0 - est.vel[2].0;

        // 期望加速度（世界系 NED）
        let ax = self.kp_pos * ex + self.kd_vel * evx;
        let ay = self.kp_pos * ey + self.kd_vel * evy;
        let az = self.kp_pos * ez + self.kd_vel * evz;

        // 期望推力：悬停基准 0.5（归一化），由垂直加速度误差调制。
        // 世界系 z 向下为正，需要下降(az>0)时减小推力。
        let des_thrust = clampf(0.5 - az / self.g, 0.1, 1.0);

        // 期望姿态角（小角几何）：phi 对应 east(accel y), theta 对应 north(accel x)
        let phi_des = clampf(ay / self.g, -self.tilt_max, self.tilt_max);
        let theta_des = clampf(-ax / self.g, -self.tilt_max, self.tilt_max);
        let psi_des = sp.yaw.0;

        let (phi, theta, psi) = quat_to_rpy(est.att.w, est.att.x, est.att.y, est.att.z);

        let e_phi = phi_des - phi;
        let e_theta = theta_des - theta;
        let e_psi = psi_des - psi;

        let p_cmd = self.k_phi * e_phi - self.k_p * est.omega[0].0;
        let q_cmd = self.k_theta * e_theta - self.k_q * est.omega[1].0;
        let r_cmd = self.k_psi * e_psi - self.k_r * est.omega[2].0;

        // mixer（+roll 右, +pitch 前上）
        let m0 = des_thrust + 0.5 * (p_cmd + q_cmd + r_cmd);
        let m1 = des_thrust + 0.5 * (-p_cmd - q_cmd + r_cmd);
        let m2 = des_thrust + 0.5 * (-p_cmd + q_cmd - r_cmd);
        let m3 = des_thrust + 0.5 * (p_cmd - q_cmd - r_cmd);

        ActuatorCmd {
            motor: [
                clampf(m0, 0.0, 1.0),
                clampf(m1, 0.0, 1.0),
                clampf(m2, 0.0, 1.0),
                clampf(m3, 0.0, 1.0),
            ],
        }
    }

    fn reset(&mut self) {}
}
