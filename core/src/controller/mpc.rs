//! MPC 位置控制器（短视界，no_std 无堆分配）。
//!
//! 思路：把位置环建模为各轴独立的离散双积分器，做有限视界（N 步）滚动优化，
//! 在 **倾角/推力约束** 下最小化跟踪误差 + 控制代价；再复用 LQR 的姿态内环 +
//! X 型混控把（期望加速度 -> 期望姿态 + 推力）落到电机。
//!
//! 与 LQR 的区别：LQR 是无限视界固定增益（无约束），MPC 在每一拍重新求解带
//! 约束的开环序列并只执行首步（receding horizon），对饱和/大阶跃/轨迹拐角的
//! 抗约束能力更强。求解器用投影梯度（少量迭代），计算量小、确定、无矩阵求逆。
//!
//! 坐标系：世界 NED（北-X，东-Y，下-Z）。

use crate::controller::trait_def::{ActuatorCmd, Controller, Setpoint};
use crate::math::atan2;
use crate::units::Second;
use crate::vehicle::VehicleState;

/// 每个轴的 MPC 求解器参数与状态缓冲（per-instance 固定容量数组，no_std 安全）。
struct AxisMpc2 {
    n: usize,
    q_pos: f32,
    q_vel: f32,
    r: f32,
    umax: f32,
    u: [f32; 32],
}

impl AxisMpc2 {
    fn new(n: usize, q_pos: f32, q_vel: f32, r: f32, umax: f32) -> Self {
        Self { n, q_pos, q_vel, r, umax, u: [0.0; 32] }
    }

    /// 给定当前 (x,v) 与参考 (xr, vr)，求解控制序列并取首步加速度指令。
    /// dt 为控制周期；iters 为投影梯度迭代次数。约束：|u| <= umax。
    fn solve(&mut self, x0: f32, v0: f32, xr: f32, vr: f32, dt: f32, iters: usize) -> f32 {
        let n = self.n;
        // 初始控制序列：简单 PD 前馈，夹在 umax
        for k in 0..n {
            let u0 = 0.4 * (xr - x0) + 0.8 * (vr - v0);
            self.u[k] = clamp(u0, -self.umax, self.umax);
        }

        // 投影梯度下降（伴随法反向传播梯度）
        let alpha = 0.5;
        for _ in 0..iters {
            let mut sx = [0.0f32; 33];
            let mut sv = [0.0f32; 33];
            sx[0] = x0;
            sv[0] = v0;
            for k in 0..n {
                sx[k + 1] = sx[k] + dt * sv[k] + 0.5 * dt * dt * self.u[k];
                sv[k + 1] = sv[k] + dt * self.u[k];
            }
            let mut lx = 0.0f32;
            let mut lv = 0.0f32;
            for k in (0..n).rev() {
                let dcost_dx = self.q_pos * (sx[k] - xr);
                let dcost_dv = self.q_vel * (sv[k] - vr);
                let dcost_du = self.r * self.u[k];
                let gx = dcost_dx + lx;
                let gv = dcost_dv + lv + dt * lx;
                let gu = dcost_du + 0.5 * dt * dt * lx + dt * lv;
                lx = gx;
                lv = gv;
                self.u[k] = clamp(self.u[k] - alpha * gu, -self.umax, self.umax);
            }
        }
        self.u[0]
    }
}

pub struct MpcController {
    g: f32,
    tilt_max: f32,
    hover_thrust: f32,
    // 姿态内环增益（全状态反馈，与 LQR 同）
    k_phi: f32,
    k_theta: f32,
    k_psi: f32,
    k_p: f32,
    k_q: f32,
    k_r: f32,
    // 三轴 MPC 求解器
    mx: AxisMpc2,
    my: AxisMpc2,
    mz: AxisMpc2,
    n: usize,
    iters: usize,
}

impl MpcController {
    pub fn new() -> Self {
        let n = 12;
        let tilt = 0.35;
        let g = 9.81;
        Self {
            g,
            tilt_max: tilt,
            hover_thrust: 0.5,
            k_phi: 4.5,
            k_theta: 4.5,
            k_psi: 1.5,
            k_p: 1.2,
            k_q: 1.2,
            k_r: 0.6,
            // 水平轴加速度上限 = g*tan(tilt) ≈ g*tilt
            mx: AxisMpc2::new(n, 1.0, 0.6, 0.05, g * tilt),
            my: AxisMpc2::new(n, 1.0, 0.6, 0.05, g * tilt),
            // 垂直轴：推力约束 -> 加速度上限取 g（合理上界）
            mz: AxisMpc2::new(n, 1.0, 0.6, 0.05, g),
            n,
            iters: 6,
        }
    }

    pub fn default_quad() -> Self {
        Self::new()
    }

    pub fn from_config(c: &crate::config::CtrlParams) -> Self {
        let mut s = Self::new();
        s.g = c.gravity;
        s.tilt_max = c.tilt_max;
        s.hover_thrust = c.hover_thrust;
        let umax_h = c.gravity * c.tilt_max;
        s.mx = AxisMpc2::new(s.n, 1.0, 0.6, 0.05, umax_h);
        s.my = AxisMpc2::new(s.n, 1.0, 0.6, 0.05, umax_h);
        s.mz = AxisMpc2::new(s.n, 1.0, 0.6, 0.05, c.gravity);
        s
    }
}

fn quat_to_rpy(w: f32, x: f32, y: f32, z: f32) -> (f32, f32, f32) {
    let roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch = atan2(2.0 * (w * y - x * z), 1.0 - 2.0 * (y * y + z * z));
    let yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (z * z + x * x));
    (roll, pitch, yaw)
}

impl Controller for MpcController {
    fn control(&mut self, dt: Second, sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        let dt = dt.0;

        // 三轴独立 MPC 求解期望加速度（含设定点速度前馈：ref vel = sp.vel）
        let ax = self.mx.solve(est.pos[0].0, est.vel[0].0, sp.pos[0].0, sp.vel[0].0, dt, self.iters);
        let ay = self.my.solve(est.pos[1].0, est.vel[1].0, sp.pos[1].0, sp.vel[1].0, dt, self.iters);
        let az = self.mz.solve(est.pos[2].0, est.vel[2].0, sp.pos[2].0, sp.vel[2].0, dt, self.iters);

        // 期望推力（世界系 z 向下为正，需要下降 az>0 减小推力）
        let des_thrust = clamp(self.hover_thrust - az / self.g, 0.1, 1.0);

        // 期望姿态（小角几何）
        let phi_des = clamp(ay / self.g, -self.tilt_max, self.tilt_max);
        let theta_des = clamp(-ax / self.g, -self.tilt_max, self.tilt_max);
        let psi_des = sp.yaw.0;

        let (phi, theta, psi) = quat_to_rpy(est.att.w, est.att.x, est.att.y, est.att.z);
        let e_phi = phi_des - phi;
        let e_theta = theta_des - theta;
        let e_psi = psi_des - psi;

        let p_cmd = self.k_phi * e_phi - self.k_p * est.omega[0].0;
        let q_cmd = self.k_theta * e_theta - self.k_q * est.omega[1].0;
        let r_cmd = self.k_psi * e_psi - self.k_r * est.omega[2].0;

        let m0 = des_thrust + 0.5 * (p_cmd + q_cmd + r_cmd);
        let m1 = des_thrust + 0.5 * (-p_cmd - q_cmd + r_cmd);
        let m2 = des_thrust + 0.5 * (-p_cmd + q_cmd - r_cmd);
        let m3 = des_thrust + 0.5 * (p_cmd - q_cmd - r_cmd);

        ActuatorCmd {
            motor: [
                clamp(m0, 0.0, 1.0),
                clamp(m1, 0.0, 1.0),
                clamp(m2, 0.0, 1.0),
                clamp(m3, 0.0, 1.0),
            ],
        }
    }

    fn reset(&mut self) {}
}

#[inline]
fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo { lo } else if x > hi { hi } else { x }
}
