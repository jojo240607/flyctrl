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
    // 垂向位置积分项（消除传感器噪声下的稳态下沉）：iz 为积分累积，ki_z 为积分增益
    ki_z: f32,
    iz: f32,
    // 阶段 11-A：EKF 估计的垂直速度/位置一阶低通（EMA）状态，滤除 IMU 高频噪声。
    // 噪声经 EKF 估计后直接驱动油门会导致悬停发散；LPF 时间常数由 vel_lpf_tau 控制（0=不过滤）。
    vel_lpf_tau: f32,
    filt_vd: f32,   // 滤波后的垂直速度（NED，向下正）
    filt_d: f32,    // 滤波后的垂直位置（NED，向下正）
    filt_init: bool, // 首帧直接赋值避免启动瞬态
    // 调试快照：最近一次内环计算的姿态误差向量与期望机体角速度
    dbg_err: [f32; 3],
    dbg_pqr: [f32; 3],
    dbg_omega: [f32; 3],
    // 阶段 11-A 诊断：控制律内部步计数（仅用于一次性 stderr 诊断，固定步后停止）
    dbg_step: u32,
    // 阶段 11-A 诊断：最近一次控制律内部量（供 host 侧打印，绕开 no_std 无 eprintln）
    dbg_raw_d: f32,
    dbg_raw_vd: f32,
    dbg_filt_d: f32,
    dbg_filt_vd: f32,
    dbg_ez: f32,
    dbg_izv: f32,
    dbg_des_vz: f32,
    dbg_acc_d: f32,
    dbg_des_thr: f32,
}

impl PidController {
    /// 读取最近一次内环调试快照（误差向量、期望机体角速度、机体角速度）。
    pub fn dbg_last(&self) -> ([f32; 3], [f32; 3], [f32; 3]) {
        (self.dbg_err, self.dbg_pqr, self.dbg_omega)
    }

    /// 调试：返回垂向位置积分累积值。
    pub fn debug_iz(&self) -> f32 {
        self.iz
    }

    /// 阶段 11-A 诊断：返回最近一次控制律内部量元组
    /// (raw_d, raw_vd, filt_d, filt_vd, ez, iz, des_vz, acc_d, des_thr)。
    pub fn debug_pid_internal(&self) -> (f32, f32, f32, f32, f32, f32, f32, f32, f32) {
        (
            self.dbg_raw_d,
            self.dbg_raw_vd,
            self.dbg_filt_d,
            self.dbg_filt_vd,
            self.dbg_ez,
            self.dbg_izv,
            self.dbg_des_vz,
            self.dbg_acc_d,
            self.dbg_des_thr,
        )
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
            kv_z: 1.5,
            vmax_xy: 2.0,
            vmax_z: 2.0,
            tilt_max: 0.35,
            att_kp: 3.0,
            att_kd: 0.3,
            hover_thrust: 0.5,
            gravity: 9.81,
            ki_z: 0.3, // 原 0.6：积分零点从 ωz=1.2 降到 0.6 rad/s（低于增益穿越 ωc≈0.68 rad/s），
                      // 使相位裕度从 19.7° 提升到 36.0°（GM 保持 inf）；实测垂直抗扰峰值偏差
                      // 反而更小（0.094m vs 0.131m），稳态偏差均≈0，未牺牲抗风性能
            iz: 0.0,
            vel_lpf_tau: 0.15,
            filt_vd: 0.0,
            filt_d: 0.0,
            filt_init: false,
            dbg_err: [0.0; 3],
            dbg_pqr: [0.0; 3],
            dbg_omega: [0.0; 3],
            dbg_step: 0,
            dbg_raw_d: 0.0,
            dbg_raw_vd: 0.0,
            dbg_filt_d: 0.0,
            dbg_filt_vd: 0.0,
            dbg_ez: 0.0,
            dbg_izv: 0.0,
            dbg_des_vz: 0.0,
            dbg_acc_d: 0.0,
            dbg_des_thr: 0.0,
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
        s.vel_lpf_tau = c.vel_lpf_tau;
        s
    }
}

impl Controller for PidController {
    fn control(&mut self, _dt: Second, sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        let g = self.gravity;
        let dt = _dt.0;

        // --- 阶段 11-A：垂直通道 EMA 低通（滤除 IMU 高频噪声） ---
        // IMU 抖动经 EKF 估计后直接驱动油门，会导致悬停向上发散（见 PLAN 阶段 11-A）。
        // 对垂直位置/速度估计做一阶低通，时间常数 vel_lpf_tau（0=不过滤，保持历史行为）。
        // 首帧直接赋值，避免启动瞬态。
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

        // --- 外环：位置误差 -> 期望速度（限幅，避免饱和） ---
        // 加入设定点速度前馈：轨迹跟踪时直接把 sp.vel 叠加到期望速度，
        // 减少相位滞后（square/circle 场景 RMS 显著下降）。
        let ex = sp.pos[0].0 - est.pos[0].0;
        let ey = sp.pos[1].0 - est.pos[1].0;
        let ez = sp.pos[2].0 - est_d;
        // 垂向位置积分（抗稳态下沉）：iz 累积位置误差，作为期望速度的积分分量。
        // 标准 PI 配条件积分（clamping 抗 windup，PLAN 阶段 11-A）：
        // 用回算（back-calculation）抗积分饱和：当 des_vz 将饱和时，把 iz 直接置为
        // “恰好使 des_vz 抵达饱和边界”的值（并夹在 ±2 内），既不继续 windup、也不反向。
        // 注意：下行饱和（需最大爬升）时 iz 应取正值（与 pre_iz 反向，二者相加恰为 -vmax_z），
        // 旧实现把 clamp 上下界写反且夹了错误变量，导致 iz 朝错误方向 windup 到限幅之外、
        // 抵消 PD 爬升指令、悬停无法恢复。
        let pre_iz = self.kp_z * ez + sp.vel[2].0; // P 项 + 速度前馈（不含积分）
        let iz_tent = clampf(self.iz + self.ki_z * ez * dt, -2.0, 2.0);
        let des_vz_tent = pre_iz + iz_tent;
        let mut iz_final = iz_tent;
        if des_vz_tent > self.vmax_z {
            // 向上饱和：iz 回算到使 des_vz 恰为 +vmax_z 的值
            iz_final = clampf(self.vmax_z - pre_iz, -2.0, 2.0);
        } else if des_vz_tent < -self.vmax_z {
            // 向下饱和（需最大爬升）：iz 回算到使 des_vz 恰为 -vmax_z 的值（正值，正确方向）
            iz_final = clampf(-self.vmax_z - pre_iz, -2.0, 2.0);
        }
        self.iz = iz_final;
        let des_vx = clampf(self.kp_xy * ex + sp.vel[0].0, -self.vmax_xy, self.vmax_xy);
        let des_vy = clampf(self.kp_xy * ey + sp.vel[1].0, -self.vmax_xy, self.vmax_xy);
        let des_vz = clampf(pre_iz + self.iz, -self.vmax_z, self.vmax_z);

        // --- 中环：速度误差 -> 期望世界系加速度 ---
        // P3-A1 轨迹跟踪：在速度误差 P 项之上叠加设定点加速度前馈 `sp.acc`，
        // 使转弯/机动时控制器直接按期望加速度预倾（而非等位置/速度误差积累），
        // 减小轨迹跟踪相位滞后。
        let acc_n = self.kv_xy * (des_vx - est.vel[0].0) + sp.acc[0].0; // 北向
        let acc_e = self.kv_xy * (des_vy - est.vel[1].0) + sp.acc[1].0; // 东向
        let acc_d = self.kv_z * (des_vz - est_vd) + sp.acc[2].0; // 下垂方向（NED），用滤波后垂直速度

        // 阶段 11-A 诊断：把控制律内部量存进调试字段，供 host 侧打印（绕开 no_std 无 eprintln）。
        self.dbg_raw_d = est.pos[2].0;
        self.dbg_raw_vd = est.vel[2].0;
        self.dbg_filt_d = est_d;
        self.dbg_filt_vd = est_vd;
        self.dbg_ez = ez;
        self.dbg_izv = self.iz;
        self.dbg_des_vz = des_vz;
        self.dbg_acc_d = acc_d;
        // des_thrust 在下方计算，此处先留 0，计算后回填

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
        self.dbg_des_thr = des_thrust;

        // 期望姿态四元数：由（roll=+tilt_e, pitch=-tilt_n, yaw=sp.yaw）构成。
        // 飞控机体(经 X-180 实为前-左-下)：推力沿机体 -Z_body。绕 +Y 正转(+pitch) 把推力
        // 旋到 -X(南)，故北向(+X)加速需 -pitch；东向(+Y)则需 +roll（绕 +X 正转把 -Z 旋到 +Y，
        // 实测见下）。
        let yaw = sp.yaw;
        // 期望 roll 取 +tilt_e（实测 2026-08-21）：NED 中绕 +X(前向) 正转(右滚)把机体 -Z
        // 旋到 +Y(东) -> 东向推力，故东向加速度需 +roll；旧代码用 -tilt_e 恰好反向，
        // 导致东向速度指令产生西向推力、东向持续漂移发散（Hover 逐秒诊断 y: 0→-65m）。
        let q_des = Quaternion::from_euler(Radian(tilt_e), Radian(-tilt_n), yaw);

        // --- 内环：四元数姿态误差 -> 期望机体角速度（标准鲁棒写法，无欧拉角奇点） ---
        // 复用共享姿态内环 `attitude::attitude_rates`（P3-A3 提取，与 TECS 完全一致）。
        // 含：q_err = q_est^-1 ⊗ q_des、误差旋转向量 ≈ 2·sign(w)·(x,y,z)、
        //     期望机体角速度 = Kp_att·误差向量 - Kd_att·当前角速度（阻尼）。
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

        // --- 混控：X 型四旋翼（0=前右 1=后左 2=前左 3=后右） ---
        // 复用共享混控 `attitude::x4_mix`（P3-A3 提取，与 TECS 完全一致）。
        // 布局与符号（含 yaw 取 +r_cmd 的符号修正）见 attitude.rs 混控注释；
        // 控制器命令 (p_cmd,q_cmd,r_cmd) 定义在飞控机体轴（NED/FRD：前-X 右-Y 下-Z）。
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
        // 四元数误差内环无状态积分；清除垂向位置积分项防 windup 残留
        self.iz = 0.0;
        // 阶段 11-A：重置 EMA 滤波状态，避免跨任务/重启残留
        self.filt_vd = 0.0;
        self.filt_d = 0.0;
        self.filt_init = false;
        self.dbg_step = 0;
    }
}

#[inline]
fn clampf(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo { lo } else if v > hi { hi } else { v }
}
