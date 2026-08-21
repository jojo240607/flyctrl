//! TECS（总能量控制系统）控制器：能量保持 + 空速拖拽前馈（P3-A3）。
//!
//! 与工程基线 [`super::pid::PidController`] 相比，TECS 的关键差异在两处：
//!
//! 1. **垂直通道改为对"总能量高度"（能量比高度）做 PI**：
//!    ```text
//!    h_eq = h + v_h²/(2·g)      // 势能高度 + 动能高度（比能量 E = g·h + ½v_h² 的等效高度）
//!    ```
//!    油门响应的是"动能+势能之和"的误差，而非单纯高度误差。速度机动时（加速增
//!    动能、减速放动能）允许势能↔动能自然交换，油门不再为追逐高度而额外注能，
//!    因此风扰/机动下**总能量保持**更好、垂向过冲与能量泵送更小。
//!
//! 2. **水平加速度叠加空速拖拽前馈**：
//!    ```text
//!    a_drag = drag_k · |v_rel| · v_rel      （v_rel = v_ground - wind，相对空速矢量）
//!    ```
//!    用空速计测得的气流矢量 v_rel 直接预补偿机体气动型阻（0.5·ρ·Cd·|v_rel|·v_rel），
//!    逆风/高速前飞时提前倾角，减小位置/速度环滞后与偏移。方向取 +v̂_rel（与阻力相反），
//!    稳态顶风（v_ground→0、v_rel→-wind）时持续顶风倾角抵消风载。
//!
//!    注意：方向基准必须用**相对空速矢量 v_rel**（经 `set_measured_airspeed_vec`
//!    注入，见 trait_def），而非地速方向——逆风时两者方向相反；只用地速会给出
//!    与阻力同向的错误前馈（曾导致逆风被吹回更甚）。
//!
//! 3. **姿态内环复用 [`super::attitude`]**：`attitude_rates` + `x4_mix` 与 PID 完全
//!    一致，保证不同外环算法"内环行为完全一致"，便于公平对比。
//!
//! 结构（串级，与 PID 同构但垂直环能量化）：
//!   水平外环：位置误差 -> 期望速度（限幅）                    (P)
//!   水平中环：速度误差 + 空速拖拽前馈 -> 期望倾角              (P + 前馈)
//!   垂直环  ：总能量高度误差 -> 期望垂直速度 -> 期望加速度 -> 油门 (PI，条件积分)
//!   内环    ：四元数姿态误差 -> 期望机体角速度 -> 混控          (PD，复用 attitude.rs)

use crate::units::*;
use crate::vehicle::{ActuatorCmd, Quaternion, VehicleState};
use crate::controller::{Controller, trait_def::Setpoint};

pub struct TecsController {
    // 水平外环 P：位置误差 -> 期望速度（世界系）
    kp_xy: f32,
    vmax_xy: f32,
    // 水平中环 P：速度误差 -> 期望水平加速度
    kv_xy: f32,
    // 垂直总能量环：能量高度误差 -> 期望垂直速度（PI，条件积分回算）
    kp_z: f32,
    kv_z: f32,
    ki_z: f32,
    iz: f32,
    vmax_z: f32,
    // 空速拖拽前馈系数：a_drag = drag_k·|v_rel|·v_rel（m/s² per m/s²），0=关闭前馈
    drag_k: f32,
    /// P3-A3：空速计测得的相对空速矢量（NED 水平，m/s）= v_ground - wind。
    /// 由宿主每周期经 `Controller::set_measured_airspeed_vec` 注入；只做前馈方向，
    /// 与 EKF 的 `est.airspeed`（地速幅值，速度融合用）相互独立。
    meas_v_rel: [f32; 2],
    // 姿态内环：四元数误差 -> 机体角速度（PD，复用 attitude.rs）
    att_kp: f32,
    att_kd: f32,
    // 通用
    hover_thrust: f32,
    gravity: f32,
    tilt_max: f32,
    // 垂直 EMA 低通（同 PID，滤 EKF 高频噪声，vel_lpf_tau=0 不过滤）
    vel_lpf_tau: f32,
    filt_vd: f32,
    filt_d: f32,
    filt_init: bool,
    // 调试快照
    dbg_err: [f32; 3],
    dbg_pqr: [f32; 3],
    dbg_omega: [f32; 3],
    dbg_airspeed: f32, // 相对空速幅值（m/s，注入的 meas_v_rel）
    dbg_drag_a: f32,   // 拖拽前馈加速度幅值（m/s²）
    dbg_h_eq: f32,     // 估计总能量高度（向下语义）
    dbg_e_eq: f32,     // 能量高度误差（向下正）
    dbg_des_thr: f32,
}

impl TecsController {
    /// 典型 450mm X 四旋翼（与 PID 基线同增益，保证公平对比）。
    pub fn default_quad() -> Self {
        Self {
            kp_xy: 0.5,
            vmax_xy: 2.0,
            kv_xy: 0.8,
            kp_z: 0.5,
            kv_z: 1.5,
            ki_z: 0.3,
            iz: 0.0,
            vmax_z: 2.0,
            // 0.5·ρ·Cd_h/m ≈ 0.5·1.225·0.18/1.2 ≈ 0.092（default_quad 机体水平型阻）
            drag_k: 0.09,
            meas_v_rel: [0.0; 2],
            att_kp: 3.0,
            att_kd: 0.3,
            hover_thrust: 0.5,
            gravity: 9.81,
            tilt_max: 0.35,
            vel_lpf_tau: 0.15,
            filt_vd: 0.0,
            filt_d: 0.0,
            filt_init: false,
            dbg_err: [0.0; 3],
            dbg_pqr: [0.0; 3],
            dbg_omega: [0.0; 3],
            dbg_airspeed: 0.0,
            dbg_drag_a: 0.0,
            dbg_h_eq: 0.0,
            dbg_e_eq: 0.0,
            dbg_des_thr: 0.0,
        }
    }

    /// 从机型配置构造（姿态环/位置环/垂向环增益与 PID 一致，外加 drag_fwd）。
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
        s.vel_lpf_tau = c.vel_lpf_tau;
        s.drag_k = c.drag_fwd;
        s
    }

    /// 空速拖拽前馈系数读写（供对照测试关闭/调整前馈）。
    pub fn set_drag_fwd(&mut self, k: f32) { self.drag_k = k; }
    pub fn drag_fwd(&self) -> f32 { self.drag_k }

    /// 调试：最近一次（估计能量高度[向下], 能量高度误差[向下正]）。
    pub fn debug_energy(&self) -> (f32, f32) { (self.dbg_h_eq, self.dbg_e_eq) }
    /// 调试：最近一次（真空速 m/s, 拖拽前馈加速度幅值 m/s²）。
    pub fn debug_drag(&self) -> (f32, f32) { (self.dbg_airspeed, self.dbg_drag_a) }
    /// 调试：最近一次内环快照（姿态误差向量、期望机体角速度、机体角速度）。
    pub fn dbg_last(&self) -> ([f32; 3], [f32; 3], [f32; 3]) {
        (self.dbg_err, self.dbg_pqr, self.dbg_omega)
    }
}

impl Controller for TecsController {
    fn control(&mut self, _dt: Second, sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        let g = self.gravity;
        let dt = _dt.0;

        // --- 垂直 EMA 低通（同 PID，滤 IMU 高频噪声） ---
        let (est_d, est_vd) = if self.vel_lpf_tau > 0.0 {
            let alpha = (dt / (self.vel_lpf_tau + dt)).clamp(0.0, 1.0);
            if !self.filt_init {
                self.filt_d = est.pos[2].0;
                self.filt_vd = est.vel[2].0;
                self.filt_init = true;
            } else {
                self.filt_d += alpha * (est.pos[2].0 - self.filt_d);
                self.filt_vd += alpha * (est.vel[2].0 - self.filt_vd);
            }
            (self.filt_d, self.filt_vd)
        } else {
            (est.pos[2].0, est.vel[2].0)
        };

        // --- 水平外环：位置误差 -> 期望速度（限幅，含设定点速度前馈） ---
        let ex = sp.pos[0].0 - est.pos[0].0;
        let ey = sp.pos[1].0 - est.pos[1].0;
        let des_vx = clampf(self.kp_xy * ex + sp.vel[0].0, -self.vmax_xy, self.vmax_xy);
        let des_vy = clampf(self.kp_xy * ey + sp.vel[1].0, -self.vmax_xy, self.vmax_xy);

        // --- 水平中环：速度误差 -> 期望水平加速度（+设定点加速度前馈） ---
        let mut acc_n = self.kv_xy * (des_vx - est.vel[0].0) + sp.acc[0].0;
        let mut acc_e = self.kv_xy * (des_vy - est.vel[1].0) + sp.acc[1].0;

        // --- 空速拖拽前馈：a_drag = drag_k·|v_rel|·v_rel ---
        // 用注入的相对空速矢量（meas_v_rel = v_ground - wind）直接预补偿气动型阻
        // （0.5·ρ·Cd·|v_rel|·v_rel，沿 -v_rel 方向）。符号取 +v̂_rel：与阻力相反；
        // 稳态顶风（地速→0、v_rel→-wind）时持续顶风倾角抵消风载。
        // 0.5 m/s 以下不激活，避免近悬停数值噪声。
        let vr = self.meas_v_rel;
        let v_rel_mag = libm::sqrtf(vr[0] * vr[0] + vr[1] * vr[1]);
        let mut drag_a = [0.0f32; 2];
        if v_rel_mag > 0.5 {
            let k = self.drag_k * v_rel_mag; // k·|v_rel|
            drag_a[0] = k * vr[0];
            drag_a[1] = k * vr[1];
        }
        acc_n += drag_a[0];
        acc_e += drag_a[1];
        self.dbg_airspeed = v_rel_mag;
        self.dbg_drag_a = libm::sqrtf(drag_a[0] * drag_a[0] + drag_a[1] * drag_a[1]);

        // --- 垂直环：总能量高度 PI（NED 下 d_eq = d - v_h²/(2g)） ---
        // 比能量 E = g·h + ½v_h²（h=-d 向上），等效高度 h_eq = E/g = h + v_h²/(2g)，
        // 转 NED（d=-h）即 d_eq = d - v_h²/(2g)。期望动能高度用设定点水平速度；
        // 实际动能高度用地速（机体世界系动能，与风无关）。
        let vh_est = libm::sqrtf(est.vel[0].0 * est.vel[0].0 + est.vel[1].0 * est.vel[1].0);
        let vh_sp = libm::sqrtf(sp.vel[0].0 * sp.vel[0].0 + sp.vel[1].0 * sp.vel[1].0);
        let sp_d_eq = sp.pos[2].0 - (vh_sp * vh_sp) / (2.0 * g);
        let est_d_eq = est_d - (vh_est * vh_est) / (2.0 * g);
        let ez = sp_d_eq - est_d_eq; // 能量高度误差（向下正）

        // PI + 条件积分回算（同 PID，防 windup）
        let pre_iz = self.kp_z * ez + sp.vel[2].0;
        let iz_tent = clampf(self.iz + self.ki_z * ez * dt, -2.0, 2.0);
        let des_vz_tent = pre_iz + iz_tent;
        let mut iz_final = iz_tent;
        if des_vz_tent > self.vmax_z {
            iz_final = clampf(self.vmax_z - pre_iz, -2.0, 2.0);
        } else if des_vz_tent < -self.vmax_z {
            iz_final = clampf(-self.vmax_z - pre_iz, -2.0, 2.0);
        }
        self.iz = iz_final;
        let des_vz = clampf(pre_iz + self.iz, -self.vmax_z, self.vmax_z);
        let acc_d = self.kv_z * (des_vz - est_vd) + sp.acc[2].0;

        self.dbg_h_eq = est_d_eq;
        self.dbg_e_eq = ez;

        // --- 期望姿态 + 油门（与 PID 相同映射） ---
        let tilt_n = clampf(acc_n / g, -self.tilt_max, self.tilt_max);
        let tilt_e = clampf(acc_e / g, -self.tilt_max, self.tilt_max);
        let tilt_mag = libm::sqrtf(tilt_n * tilt_n + tilt_e * tilt_e);
        let cos_tilt = if tilt_mag < 1.55 {
            libm::cosf(tilt_mag).max(0.2)
        } else {
            0.2
        };
        let des_thrust = clampf((self.hover_thrust - acc_d / g) / cos_tilt, 0.1, 1.0);
        self.dbg_des_thr = des_thrust;

        // 期望姿态四元数（同 PID：roll=+tilt_e, pitch=-tilt_n, yaw=sp.yaw）。
        let q_des = Quaternion::from_euler(Radian(tilt_e), Radian(-tilt_n), sp.yaw);

        // --- 内环：复用 attitude.rs（与 PID 完全一致） ---
        let att_out = super::attitude::attitude_rates(
            est.att,
            q_des,
            self.att_kp,
            self.att_kd,
            [est.omega[0].0, est.omega[1].0, est.omega[2].0],
        );
        self.dbg_err = att_out.err;
        self.dbg_pqr = att_out.rates;
        self.dbg_omega = [est.omega[0].0, est.omega[1].0, est.omega[2].0];

        // --- 混控：复用 attitude.rs（X 型四旋翼） ---
        let motors = super::attitude::x4_mix(des_thrust, att_out.rates);

        ActuatorCmd {
            motor: [
                motors[0].clamp(0.0, 1.0),
                motors[1].clamp(0.0, 1.0),
                motors[2].clamp(0.0, 1.0),
                motors[3].clamp(0.0, 1.0),
            ],
        }
    }

    fn reset(&mut self) {
        self.iz = 0.0;
        self.filt_vd = 0.0;
        self.filt_d = 0.0;
        self.filt_init = false;
    }

    fn set_measured_airspeed_vec(&mut self, v: [f32; 2]) {
        self.meas_v_rel = v;
    }
}

#[inline]
fn clampf(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}
