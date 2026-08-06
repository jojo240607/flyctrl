//! PID 控制器（工程基线，串级：位置外环 -> 姿态内环）。
//!
//! 结构：
//!   外环：位置误差 -> 期望速度 -> 期望姿态（倾角）  (P)
//!   内环：姿态/角速度误差 -> 力矩指令               (PID)
//! 最终把总推力 + 三轴力矩映射到 4 个电机（X 型混控）。
//!
//! 这是与 PX4/APM 同级的工程基线。后续 LQR/MPC 实现将复用同一 `Controller`
//! 接口，在同一仿真场景下直接 PK。

use crate::units::*;
use crate::vehicle::{ActuatorCmd, Quaternion, VehicleState};
use crate::controller::{Controller, trait_def::Setpoint};

#[derive(Clone, Copy)]
struct Pid {
    kp: f32, ki: f32, kd: f32,
    i: f32,        // 积分项
    prev: f32,     // 上次误差（微分）
    i_limit: f32,  // 抗积分饱和
}

impl Pid {
    fn new(kp: f32, ki: f32, kd: f32, i_limit: f32) -> Self {
        Self { kp, ki, kd, i: 0.0, prev: 0.0, i_limit }
    }
    fn update(&mut self, err: f32, dt: f32) -> f32 {
        if dt <= 0.0 { return 0.0; }
        self.i += err * dt;
        self.i = self.i.clamp(-self.i_limit, self.i_limit);
        let d = (err - self.prev) / dt;
        self.prev = err;
        self.kp * err + self.ki * self.i + self.kd * d
    }
    fn reset(&mut self) { self.i = 0.0; self.prev = 0.0; }
}

pub struct PidController {
    // 位置外环 P（产生期望倾角）
    pos_xy_p: f32,
    pos_z_p: f32,
    // 姿态内环 PID
    roll: Pid,
    pitch: Pid,
    yaw: Pid,
    // 推力基值（悬停油门）
    hover_thrust: f32,
}

impl PidController {
    /// 典型 450mm X 四旋翼参数（后续可移到机型配置）。
    pub fn default_quad() -> Self {
        Self {
            pos_xy_p: 0.8,
            pos_z_p: 1.2,
            roll: Pid::new(0.15, 0.05, 0.02, 0.5),
            pitch: Pid::new(0.15, 0.05, 0.02, 0.5),
            yaw: Pid::new(0.2, 0.05, 0.0, 0.5),
            hover_thrust: 0.5,
        }
    }
}

impl Controller for PidController {
    fn control(&mut self, dt: Second, sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        let dt = dt.0;

        // --- 外环：位置误差 -> 期望倾角（小角近似） ---
        let ex = sp.pos[0].0 - est.pos[0].0;
        let ey = sp.pos[1].0 - est.pos[1].0;
        let ez = sp.pos[2].0 - est.pos[2].0;

        // 期望机体倾角（北东系误差 -> 机体 roll/pitch 目标）。这里用简化映射：
        // 向东误差 -> 俯仰前倾；向北误差 -> 横滚（按 yaw 旋转到机体）。
        let yaw = quaternion_yaw(est.att);
        let (sy, cy) = crate::math::sin_cos(yaw);
        // 把世界系误差旋到机体水平系
        let ex_body = ex * cy + ey * sy;
        let ey_body = -ex * sy + ey * cy;
        let des_pitch = (self.pos_xy_p * ex_body).clamp(-0.5, 0.5); // 前倾向北
        let des_roll = (-self.pos_xy_p * ey_body).clamp(-0.5, 0.5); // 右倾向东
        let des_thrust = (self.hover_thrust - self.pos_z_p * ez).clamp(0.1, 1.0);

        // --- 内环：当前姿态（从四元数取 roll/pitch） ---
        let (roll, pitch) = quaternion_to_rp(est.att);
        let yaw_now = yaw;

        let roll_cmd = self.roll.update(des_roll - roll, dt);
        let pitch_cmd = self.pitch.update(des_pitch - pitch, dt);
        let yaw_cmd = self.yaw.update(Radian(sp.yaw.0 - yaw_now).wrapped().0, dt);

        // --- 混控：X 型四旋翼 ---
        // 电机映射（同 vehicle.rs 注释）：0=前右 1=后左 2=前左 3=后右
        // 推力基 + 俯仰(前后差) + 横滚(左右差) + 偏航(对角差)
        let m0 = des_thrust + pitch_cmd * 0.5 + roll_cmd * 0.5 - yaw_cmd * 0.5;
        let m1 = des_thrust + pitch_cmd * 0.5 - roll_cmd * 0.5 + yaw_cmd * 0.5;
        let m2 = des_thrust - pitch_cmd * 0.5 - roll_cmd * 0.5 - yaw_cmd * 0.5;
        let m3 = des_thrust - pitch_cmd * 0.5 + roll_cmd * 0.5 + yaw_cmd * 0.5;

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
        self.roll.reset();
        self.pitch.reset();
        self.yaw.reset();
    }
}

/// 从四元数取 yaw（绕世界 Z 的偏航角）。
fn quaternion_yaw(q: Quaternion) -> f32 {
    // yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    let num = 2.0 * (q.w * q.z + q.x * q.y);
    let den = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    crate::math::atan2(num, den)
}

/// 简化：从四元数取 roll/pitch（小角/标准公式，假设无大初始偏航影响）。
fn quaternion_to_rp(q: Quaternion) -> (f32, f32) {
    let roll = crate::math::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    let pitch = crate::math::clamp(crate::math::asin(2.0 * (q.w * q.y - q.z * q.x)), -1.57, 1.57);
    (roll, pitch)
}
