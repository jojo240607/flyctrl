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
    // 调试快照：最近一次内环计算的姿态误差向量与期望机体角速度
    dbg_err: [f32; 3],
    dbg_pqr: [f32; 3],
    dbg_omega: [f32; 3],
}

impl PidController {
    /// 读取最近一次内环调试快照（误差向量、期望机体角速度、机体角速度）。
    pub fn dbg_last(&self) -> ([f32; 3], [f32; 3], [f32; 3]) {
        (self.dbg_err, self.dbg_pqr, self.dbg_omega)
    }

    /// 从地面站参数表（[KpXY, KpZ, KvXY, KvZ, HoverThrust]）应用增益。
    /// 仅覆盖这 5 个字段，其余（vmax/tilt/att_kd/gravity）保持出厂默认，
    /// 避免地面站误改导致控制律发散。调用方需保证 `g` 长度 ≥ 5。
    pub fn apply_gains(&mut self, g: &[f32]) {
        if g.len() < 5 { return; }
        self.kp_xy = g[0];
        self.kp_z = g[1];
        self.kv_xy = g[2];
        self.kv_z = g[3];
        self.hover_thrust = g[4];
    }

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
            dbg_err: [0.0; 3],
            dbg_pqr: [0.0; 3],
            dbg_omega: [0.0; 3],
        }
    }

    /// 从机型配置构造。
    pub fn from_config(c: &crate::config::CtrlParams) -> Self {
        let mut s = Self::default_quad();
        s.tilt_max = c.tilt_max;
        s.hover_thrust = c.hover_thrust;
        s.gravity = c.gravity;
        s.vmax_xy = c.vmax_xy;
        s.vmax_z = c.vmax_z;
        s.att_kp = c.att_kp;
        s.att_kd = c.att_kd;
        s.kp_xy = c.kp_xy;
        s.kv_xy = c.kv_xy;
        s
    }
}

impl Controller for PidController {
    fn control(&mut self, _dt: Second, sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        let g = self.gravity;

        // --- 外环：位置误差 -> 期望速度（限幅，避免饱和） ---
        // 加入设定点速度前馈：轨迹跟踪时直接把 sp.vel 叠加到期望速度，
        // 减少相位滞后（square/circle 场景 RMS 显著下降）。
        let ex = sp.pos[0].0 - est.pos[0].0;
        let ey = sp.pos[1].0 - est.pos[1].0;
        let ez = sp.pos[2].0 - est.pos[2].0;
        let des_vx = clampf(self.kp_xy * ex + sp.vel[0].0, -self.vmax_xy, self.vmax_xy);
        let des_vy = clampf(self.kp_xy * ey + sp.vel[1].0, -self.vmax_xy, self.vmax_xy);
        let des_vz = clampf(self.kp_z * ez + sp.vel[2].0, -self.vmax_z, self.vmax_z);

        // --- 中环：速度误差 -> 期望世界系加速度 ---
        let acc_n = self.kv_xy * (des_vx - est.vel[0].0); // 北向
        let acc_e = self.kv_xy * (des_vy - est.vel[1].0); // 东向
        let acc_d = self.kv_z * (des_vz - est.vel[2].0); // 下垂方向（NED）

        // 高度推力：悬停 + 垂直加速度项（acc_d>0 表示要向下加速，减推力）。
        // 期望机体倾角（小角）：北向加速度 -> 俯仰，东向加速度 -> 横滚。
        // 采用四元数误差内环（见下），这里把世界系期望加速度转换为期望姿态四元数。
        let tilt_n = clampf(acc_n / g, -self.tilt_max, self.tilt_max);
        let tilt_e = clampf(acc_e / g, -self.tilt_max, self.tilt_max);

        // 关键：机体倾斜后推力竖直分量 = T·cos(φ)，必须按 1/cos(φ) 放大总推力，
        // 否则一倾斜就掉高 -> 高度环进一步减推力 -> 死亡螺旋翻滚。
        let tilt_mag = libm::sqrtf(tilt_n * tilt_n + tilt_e * tilt_e);
        let cos_tilt = if tilt_mag < 1.55 {
            libm::cosf(tilt_mag).max(0.2)
        } else {
            0.2
        };
        let des_thrust = clampf(
            (self.hover_thrust - acc_d / g) / cos_tilt,
            0.1, 1.0,
        );

        // 期望姿态四元数：由（roll=tilt_e, pitch=-tilt_n, yaw=sp.yaw）构成。
        // 飞控机体(经 X-180 实为前-左-下)：推力沿机体 -Z_body。绕 +Y 正转(+pitch) 把推力
        // 旋到 -X(南)，故北向(+X)加速需 -pitch；东向(+Y)由 +roll(绕+X)正确产生东向推力。
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
        self.dbg_err = [ex_b, ey_b, ez_b];
        self.dbg_pqr = [p_cmd, q_cmd, r_cmd];
        self.dbg_omega = [est.omega[0].0, est.omega[1].0, est.omega[2].0];

        // --- 混控：X 型四旋翼（0=前右 1=后左 2=前左 3=后右） ---
        // 控制器命令 (p_cmd,q_cmd,r_cmd) 在飞控机体轴；quat_up_to_ned 用 X-180 翻转，
        // 故飞控机体 -> 引擎机体的力矩向量变换为 (τx, τy, τz)_eng = (τx, -τy, -τz)_fc。
        // 即期望引擎机体力矩 = (p_cmd, -q_cmd, -r_cmd)。由引擎机体电机力矩公式反解
        // （m0前右/m1后左/m2前左/m3后右，spin 0,1 CCW / 2,3 CW）：
        //   τx = l(m0+m3-m1-m2)  τy = l(m1+m3-m0-m2)  τz = k(m0+m1-m2-m3)
        // 代入 (p_cmd,-q_cmd,-r_cmd) 解得如下（K=0.5 吸收臂长/反扭矩系数）：
        //   +X(roll) ：m0,m3 增 / m1,m2 减
        //   +Y(pitch)：m1,m3 增 / m0,m2 减
        //   +Z(yaw)  ：CCW(0,1) 增 / CW(2,3) 减
        let m0 = des_thrust + 0.5 * (p_cmd + q_cmd - r_cmd);
        let m1 = des_thrust + 0.5 * (-p_cmd - q_cmd - r_cmd);
        let m2 = des_thrust + 0.5 * (-p_cmd + q_cmd + r_cmd);
        let m3 = des_thrust + 0.5 * (p_cmd - q_cmd + r_cmd);

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
