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


/// [联调诊断] PID 内部量观测（静态，无栈开销；测试直读，定位后移除）。
#[used]
pub static mut DBG_PID: [f32; 12] = [0.0; 12];
/// [联调诊断] clamp 前原始值 / clamp 后推力（独立 static 强制求值顺序）。
#[used]
pub static mut DBG_PRE: f32 = 0.0;
#[used]
pub static mut DBG_THR: f32 = 0.0;

/// [联调诊断] **倾角指令观测**：`[acc_n, acc_e, tilt_n, tilt_e]`（clamp 前后各两个）。
///
/// 用途（H2 专项）：判断倾角指令是否**顶满 `tilt_max`**。
/// `tilt = clamp(acc/g, ±tilt_max)` —— 一旦顶满，倾角指令打满 ⇒ 姿态环被推到极限
/// ⇒ 电机饱和 ⇒ 0.3~0.7Hz 极限环（见 `docs/stage4-outer-loop-findings.md` P14）。
/// 不补这个观测点，就只能看到“电机饱和”这个下游现象，看不到根因那一步。
#[used]
pub static mut DBG_TILT: [f32; 4] = [0.0; 4];

/// [标定] `ki_xy`（**水平位置积分增益**）运行时覆盖。
///
/// ⚠️ 哨兵与 `G_MAG_ALPHA` 同规：**< 0（默认 -1）= 用编译期值**。
/// 理由：`0` 是本旋钮的**有效取值**（= 关闭水平积分，即既有行为），
/// 不能拿 0 当"未设置"，否则表达不了"显式关闭"。
#[used]
pub static mut G_KI_XY: f32 = -1.0;

/// [标定] `tilt_max`（倾角指令上限，rad）运行时覆盖。哨兵同规：**<0 = 用编译期值**。
///
/// 为何可调：H 场能力曲线显示 **5.4 m/s（蒲福 3 级上限）处漂移悬崖**（0.85→5.59m）。
/// 物理解释：该风速下抗风稳态倾角需 ≈15.3°(pitch)+2.5°(roll)，叠加阵风摆动后逼近
/// `tilt_max=20°` ⇒ **位置环饱和、失去纠偏能力**。本旋钮用于验证该解释：
/// 若漂移随 `tilt_max` 单调改善，则"倾角权限"就是那个工程杠杆（代价：需推力余量）。
#[used]
pub static mut G_TILT_MAX: f32 = -1.0;

/// [标定] 水平积分**上限**（m/s）运行时覆盖。哨兵同为 **<0 = 用编译期默认 2.0**。
/// 为何必须可调：该夹子直接决定残余稳态偏移 —— 当所需稳态速度指令超过它时，
/// 偏移被夹死为 `(des_v_need - 上限)/kp_xy`，此时**再加 `ki_xy` 也无用**。
#[used]
pub static mut G_I_XY_MAX: f32 = -1.0;

/// [标定] 水平一阶低通时间常数（s）运行时覆盖。哨兵同规：**<0 = 用编译期值**。
/// 0 是有效取值（= 关闭低通，即既有行为），故哨兵取 <0。
#[used]
pub static mut G_VEL_LPF_H_TAU: f32 = -1.0;

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
    // 速率模式（水平）：位置外环旁路，期望速度 = sp.vel（摇杆直通，推杆飞/松杆停）。
    // 默认 false（位置模式）；非 HIL 演示固件开启（大疆手感），HIL 保持位置模式。
    rate_mode_xy: bool,
    // 姿态内环：四元数误差 -> 机体角速度 的 P（比例）与 D（角速度阻尼）增益
    att_kp: f32,
    att_kd: f32,
    // 推力基值（悬停油门）与重力（用于倾角->加速度映射）
    hover_thrust: f32,
    gravity: f32,
    // 垂向位置积分项（消除传感器噪声下的稳态下沉）：iz 为积分累积，ki_z 为积分增益
    ki_z: f32,
    iz: f32,
    // 水平位置积分项（抗**恒定扰动**下的稳态偏移，如侧风）：与垂向 iz **对称**。
    //
    // 为何需要：水平外环原为 **P-only** ⇒ 恒风下必须靠位置误差换稳态速度指令
    // （`des_v = kp_xy·e`）⇒ 稳态偏移 `e = des_v/kp_xy`。实测 B3 风（5.4m/s、
    // 位置环口径）漂移 **6.64m**，与 `2.0/0.3 = 6.67m` 吻合到 0.5%；仓库自己的
    // 注释也记着这个特征（`mission.rs`："kp_xy=0.3、2m/s 巡航约 6.7m"）。
    // 垂向早有 `iz` 正是为此（"抗稳态下沉"），水平此前没有 —— 这是能力缺口。
    //
    // **默认 0 = 关闭**（保持既有行为不变），取值由扫描数据定，不由实现者拍。
    ki_xy: f32,
    /// 水平位置积分累积（NED 北/东）。抗饱和与垂向同用**回算**（back-calculation）。
    i_xy: [f32; 2],
    // 阶段 11-A：EKF 估计的垂直速度/位置一阶低通（EMA）状态，滤除 IMU 高频噪声。
    // 噪声经 EKF 估计后直接驱动油门会导致悬停发散；LPF 时间常数由 vel_lpf_tau 控制（0=不过滤）。
    vel_lpf_tau: f32,
    /// 速率环（内环 omega）一阶低通时间常数（s）。omega = EKF 的 gyro-bias，其噪声
    /// 直达内环 D 项会自激（x_hover_noise 实测 w_y ±1.4、电机差动饱和）。0=不过滤。
    rate_lpf_tau: f32,
    filt_w: [f32; 3], // 滤波后的机体角速度
    rate_filt_init: bool,
    filt_vd: f32,   // 滤波后的垂直速度（NED，向下正）
    filt_d: f32,    // 滤波后的垂直位置（NED，向下正）
    /// 水平一阶低通时间常数（s）。**0 = 关闭（既有行为）**。
    ///
    /// 为何需要：水平位置/速度此前**没有任何低通**就直驱倾角指令，而垂向一直有
    /// （`vel_lpf_tau`，注释写明"噪声经 EKF 估计后直接驱动油门会导致悬停发散"）。
    /// 后果实测（H 场，位置环，60s）：**零均值扰动下位置无界游走** ——
    /// 无恒风时峰值≡末态（9.36m 且仍在增长）；有恒风时因速度估计有确定偏置
    /// 反而有界（0.85m）。即文档 §4.2 早已写下的"估计器输出的速度噪声直接进了
    /// 位置外环…水平通道没有等效处理"。
    vel_lpf_h_tau: f32,
    /// 水平低通状态 `[pos_n, pos_e, vel_n, vel_e]`；`filt_h_init` 首帧直接赋值。
    filt_h: [f32; 4],
    filt_h_init: bool,
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

    /// 切换水平速率模式（位置外环旁路，期望速度 = sp.vel）。
    /// 非 HIL 演示固件开启（大疆手感）；HIL 轨迹模式保持位置环。
    pub fn set_rate_mode_xy(&mut self, on: bool) {
        self.rate_mode_xy = on;
    }

    /// 典型 450mm X 四旋翼参数（后续可移到机型配置）。
    /// 标准串级：pos_err -> 期望速度(限幅) -> vel_err -> 期望加速度 -> 期望姿态(四元数) -> 角速度。
    pub fn default_quad() -> Self {
        Self {
            kp_xy: 0.5,
            kp_z: 0.5,
            kv_xy: 0.8,
            kv_z: 1.5,
            vmax_xy: 3.5, // 速率模式（摇杆→速度）上限：满杆 3.0m/s 不被 clamp（8 字半径 ~3m）
            vmax_z: 2.0,
            // 倾角指令上限：**25°**（原 20°/0.35rad）。
            //
            // 依据（H 场实测，位置环 60s）：20° 时 5.4m/s（蒲福 3 级上限）下倾角指令
            // 顶满（末|pitch|=20.99°）、位置环失去纠偏 ⇒ 漂移 5.59m；抬到 25° 解除饱和
            // ⇒ 3.45m（改善 38%），且 30°/40° 无进一步收益 ⇒ **25° 已足够**。
            // 代价：所需推力 ∝1/cosθ ⇒ 25° 比 20° 仅多 3.7%。
            // 回归守卫：`wind_turb_scan::tilt_max_attribution_scan`。
            tilt_max: 0.4363, // 25°
            rate_mode_xy: false,
            att_kp: 3.0,
            att_kd: 0.3,
            hover_thrust: 0.5,
            gravity: 9.81,
            ki_z: 0.3, // 原 0.6：积分零点从 ωz=1.2 降到 0.6 rad/s（低于增益穿越 ωc≈0.68 rad/s），
                      // 使相位裕度从 19.7° 提升到 36.0°（GM 保持 inf）；实测垂直抗扰峰值偏差
                      // 反而更小（0.094m vs 0.131m），稳态偏差均≈0，未牺牲抗风性能
            iz: 0.0,
            ki_xy: 0.0, // 默认关：既有行为逐位不变；由 wind_turb_scan 扫出后再定
            i_xy: [0.0; 2],
            vel_lpf_tau: 0.15,
            rate_lpf_tau: 0.02, // 50Hz：抑噪为主，相位滞后小
            filt_w: [0.0; 3],
            rate_filt_init: false,
            filt_vd: 0.0,
            filt_d: 0.0,
            vel_lpf_h_tau: 0.0, // 默认关：既有行为逐位不变；取值由 A/B 扫描定
            filt_h: [0.0; 4],
            filt_h_init: false,
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
        // 标定旋钮（易失读：由外部写入；0 是有效值故哨兵取 <0）
        {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_KI_XY)) };
            if ov >= 0.0 {
                self.ki_xy = ov;
            }
            let ot = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_VEL_LPF_H_TAU)) };
            if ot >= 0.0 {
                self.vel_lpf_h_tau = ot;
            }
        }
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
        // 水平低通：**与垂向完全同形**（位置+速度都滤、首帧直接赋值）。
        let (est_n, est_e, est_vn, est_ve) = if self.vel_lpf_h_tau > 0.0 {
            let alpha = (dt / (self.vel_lpf_h_tau + dt)).clamp(0.0, 1.0);
            if !self.filt_h_init {
                self.filt_h = [est.pos[0].0, est.pos[1].0, est.vel[0].0, est.vel[1].0];
                self.filt_h_init = true;
            } else {
                self.filt_h[0] += alpha * (est.pos[0].0 - self.filt_h[0]);
                self.filt_h[1] += alpha * (est.pos[1].0 - self.filt_h[1]);
                self.filt_h[2] += alpha * (est.vel[0].0 - self.filt_h[2]);
                self.filt_h[3] += alpha * (est.vel[1].0 - self.filt_h[3]);
            }
            (self.filt_h[0], self.filt_h[1], self.filt_h[2], self.filt_h[3])
        } else {
            (est.pos[0].0, est.pos[1].0, est.vel[0].0, est.vel[1].0)
        };

        // --- 外环：位置误差 -> 期望速度（限幅，避免饱和） ---
        // 加入设定点速度前馈：轨迹跟踪时直接把 sp.vel 叠加到期望速度，
        // 减少相位滞后（square/circle 场景 RMS 显著下降）。
        let ex = sp.pos[0].0 - est_n;
        let ey = sp.pos[1].0 - est_e;
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
        // 水平位置积分（抗恒定扰动下的稳态偏移，如侧风）—— 与垂向 iz 对称：
        // 同样用**回算**抗饱和（饱和时把积分置为"恰使 des_v 抵达边界"的值）。
        // 积分上限 ±2.0 m/s，与垂向同口径。
        // 积分上限：运行时旋钮（哨兵 <0 = 用编译期默认 2.0）。为何要可调：
        // B3 风下所需稳态速度指令 ≈ 2.26 m/s > 2.0 ⇒ **积分撞上限**，
        // 残余稳态偏移 = (des_v_need - I_XY_MAX)/kp_xy 被这个夹子决定。
        let i_xy_max = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_I_XY_MAX)) };
            if ov >= 0.0 {
                ov
            } else {
                2.0
            }
        };
        let pre_ix = self.kp_xy * ex + sp.vel[0].0; // P 项 + 速度前馈（不含积分）
        let pre_iy = self.kp_xy * ey + sp.vel[1].0;
        let mut ix_final = clampf(self.i_xy[0] + self.ki_xy * ex * dt, -i_xy_max, i_xy_max);
        let mut iy_final = clampf(self.i_xy[1] + self.ki_xy * ey * dt, -i_xy_max, i_xy_max);
        if !self.rate_mode_xy {
            let dx = pre_ix + ix_final;
            if dx > self.vmax_xy {
                ix_final = clampf(self.vmax_xy - pre_ix, -i_xy_max, i_xy_max);
            } else if dx < -self.vmax_xy {
                ix_final = clampf(-self.vmax_xy - pre_ix, -i_xy_max, i_xy_max);
            }
            let dy = pre_iy + iy_final;
            if dy > self.vmax_xy {
                iy_final = clampf(self.vmax_xy - pre_iy, -i_xy_max, i_xy_max);
            } else if dy < -self.vmax_xy {
                iy_final = clampf(-self.vmax_xy - pre_iy, -i_xy_max, i_xy_max);
            }
        }
        self.i_xy = [ix_final, iy_final];
        // 速率模式：位置外环旁路，期望速度 = sp.vel（摇杆直通）；否则 P + I + 速度前馈
        let des_vx = if self.rate_mode_xy {
            clampf(sp.vel[0].0, -self.vmax_xy, self.vmax_xy)
        } else {
            clampf(pre_ix + ix_final, -self.vmax_xy, self.vmax_xy)
        };
        let des_vy = if self.rate_mode_xy {
            clampf(sp.vel[1].0, -self.vmax_xy, self.vmax_xy)
        } else {
            clampf(pre_iy + iy_final, -self.vmax_xy, self.vmax_xy)
        };
        let des_vz = clampf(pre_iz + self.iz, -self.vmax_z, self.vmax_z);

        // --- 中环：速度误差 -> 期望世界系加速度 ---
        // P3-A1 轨迹跟踪：在速度误差 P 项之上叠加设定点加速度前馈 `sp.acc`，
        // 使转弯/机动时控制器直接按期望加速度预倾（而非等位置/速度误差积累），
        // 减小轨迹跟踪相位滞后。
        let acc_n = self.kv_xy * (des_vx - est_vn) + sp.acc[0].0; // 北向
        let acc_e = self.kv_xy * (des_vy - est_ve) + sp.acc[1].0; // 东向
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
        // 倾角上限旋钮（易失读；哨兵 <0 = 用编译期值）
        let tilt_max_eff = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_TILT_MAX)) };
            if ov > 0.0 { ov } else { self.tilt_max }
        };
        let tilt_n = clampf(acc_n / g, -tilt_max_eff, tilt_max_eff);
        let tilt_e = clampf(acc_e / g, -tilt_max_eff, tilt_max_eff);
        // ⚠️ **有条件写**（重要）：每拍无条件写一个 16B 静态会把控制任务推过 4ms 预算
        // ——实测同一固件仅加这条 store，`x_hover_noise` 就从 20.00°/28.64°（有界极限环）
        // 变成 141.90°/87.38°（40s 后发散）。固件里已有 `DBG_PID`(48B)/`DBG_MOTOR`(16B)
        // 等多个每拍诊断量，本条是压垮的那一根。
        // 而 H2 专项真正要问的是"**倾角是否顶满**"，所以只在**顶满时**记录 ⇒ 正常情况
        // 几乎零成本（一个可预测分支），且保留关键信息。
        if tilt_n.abs() >= tilt_max_eff - 1e-6 || tilt_e.abs() >= tilt_max_eff - 1e-6 {
            unsafe { DBG_TILT = [acc_n, acc_e, tilt_n, tilt_e]; }
        }

        // 关键：机体倾斜后推力竖直分量 = T·cos(φ)，必须按 1/cos(φ) 放大总推力，
        // 否则一倾斜就掉高 -> 高度环进一步减推力 -> 死亡螺旋翻滚。
        let tilt_mag = crate::math::sqrt(tilt_n * tilt_n + tilt_e * tilt_e);
        let cos_tilt = if tilt_mag < 1.55 {
            crate::math::cos(tilt_mag).max(0.2)
        } else {
            0.2
        };
        let dbg_pre = (self.hover_thrust - acc_d / g) / cos_tilt;
        unsafe { DBG_PRE = dbg_pre; }
        let des_thrust = clampf(dbg_pre, 0.1, 1.0);
        unsafe { DBG_THR = des_thrust; }
        self.dbg_des_thr = des_thrust;
        // [联调诊断] 记录内部量（含 gravity 与 clamp 前原始值）
        unsafe {
            DBG_PID = [
                self.hover_thrust, acc_d, des_vz, ez, des_thrust, cos_tilt,
                self.kv_z, est_vd, g,
                (self.hover_thrust - acc_d / g) / cos_tilt,
                self.kp_z, self.ki_z,
            ];
        }

        // 期望姿态四元数：由（roll=+tilt_e, pitch=-tilt_n, yaw=sp.yaw）构成。
        // 飞控机体(经 X-180 实为前-左-下)：推力沿机体 -Z_body。绕 +Y 正转(+pitch) 把推力
        // 旋到 -X(南)，故北向(+X)加速需 -pitch；东向(+Y)则需 +roll（绕 +X 正转把 -Z 旋到 +Y，
        // 实测见下）。
        let yaw = sp.yaw;
        // 期望 roll 取 +tilt_e（实测 2026-08-21）：NED 中绕 +X(前向) 正转(右滚)把机体 -Z
        // 旋到 +Y(东) -> 东向推力，故东向加速度需 +roll；旧代码用 -tilt_e 恰好反向，
        // 导致东向速度指令产生西向推力、东向持续漂移发散（Hover 逐秒诊断 y: 0→-65m）。
        // ⚠️ **坐标系修正（2026-09-21，阶段 5 的切向偏航实验暴露）**：
        // `tilt_n`/`tilt_e` 由**世界系**水平加速度 `acc_n`/`acc_e` 经 `/g` 得到，
        // 而 `from_euler(roll, pitch, yaw)` 里的 roll/pitch 是**机体系**欧拉角 ——
        // 二者只有在 **`yaw = 0`**（机体系与世界系对齐）时才相同。
        //
        // 旧代码直接把 `tilt_e`/`-tilt_n` 当 roll/pitch ⇒ **偏航非零时期望倾角方向错**
        // ⇒ 位置误差纠不回来 ⇒ 发散。实测（阶段 5 分解实验，圆轨迹 r=2m ω=1rad/s）：
        //   yaw=0（偏航固定）：跟踪 err_max **1.305m** ✓
        //   切向偏航（1 rad/s）：跟踪 err_max **22.221m** ✗（17× 劣化）
        //
        // 推导：期望推力水平分量在世界系为 `(tilt_n, tilt_e)`；转到机体系即左乘
        // `Rz(-yaw)`（忽略一阶以下的高阶耦合）：
        //   机体系前向 =  tilt_n·cosψ + tilt_e·sinψ
        //   机体系右向 = -tilt_n·sinψ + tilt_e·cosψ
        // 而本实现的欧拉映射是 `roll = 机体系右向`、`pitch = -机体系前向`
        // （见下方 445~447 行的符号推导）。`yaw=0` 时退化为旧行为 ⇒ 既有判据不变 ✓。
        let (sy, cy) = crate::math::sin_cos(yaw.0);
        let tilt_fwd = tilt_n * cy + tilt_e * sy;
        let tilt_right = -tilt_n * sy + tilt_e * cy;
        let q_des = Quaternion::from_euler(Radian(tilt_right), Radian(-tilt_fwd), yaw);

        self.control_attitude(_dt, q_des, des_thrust, est)
    }

    fn reset(&mut self) {
        // 四元数误差内环无状态积分；清除垂向位置积分项防 windup 残留
        self.iz = 0.0;
        // 阶段 11-A：重置 EMA 滤波状态，避免跨任务/重启残留
        self.filt_vd = 0.0;
        self.filt_d = 0.0;
        self.filt_w = [0.0; 3];
        self.rate_filt_init = false;
        self.filt_init = false;
        self.dbg_step = 0;
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

/// 姿态内环的独立入口（固有方法，不属于 `Controller` trait）。
impl PidController {
    /// **姿态内环**：期望姿态 + 总推力 → 执行器指令（**不含**位置/速度外环）。
    ///
    /// 从 [`control`](Controller::control) 尾部提取，使姿态内环**可独立驱动**。
    /// 动机（阶段 2）：姿态环原先**没有外部入口** —— `q_des` 只由外环在
    /// `control()` 内部生成（`tilt = clamp(acc/g, ±tilt_max)`），而阶段 2 的核心
    /// 需求是“先用**真值姿态**验证控制器本身，再接入估计姿态”。
    ///
    /// `control()` 内部调用本方法，**行为逐位不变**（纯提取，无逻辑改动）。
    ///
    /// 链路：`omega` 一阶低通 → `attitude_rates`（四元数误差 P-D → 期望机体角速率）
    /// → `x4_mix` → 限幅。注意此处**无独立速率 PID**：角速率指令直接进混控。
    pub fn control_attitude(
        &mut self,
        _dt: Second,
        q_des: Quaternion,
        des_thrust: f32,
        est: &VehicleState,
    ) -> ActuatorCmd {
        let dt = _dt.0;
        // --- 内环：四元数姿态误差 -> 期望机体角速度（标准鲁棒写法，无欧拉角奇点） ---
        // 复用共享姿态内环 `attitude::attitude_rates`（P3-A3 提取，与 TECS 完全一致）。
        // 含：q_err = q_est^-1 ⊗ q_des、误差旋转向量 ≈ 2·sign(w)·(x,y,z)、
        //     期望机体角速度 = Kp_att·误差向量 - Kd_att·当前角速度（阻尼）。
        // 速率环低通：omega 直接来自 EKF(gyro-bias)，噪声直达内环 D 项会自激。
        // 一阶低通 rate_lpf_tau（0=不过滤）。首帧直接赋值避免启动瞬态。
        let omega_f = if self.rate_lpf_tau > 0.0 {
            let a = (dt / (self.rate_lpf_tau + dt)).clamp(0.0, 1.0);
            if !self.rate_filt_init {
                self.filt_w = [est.omega[0].0, est.omega[1].0, est.omega[2].0];
                self.rate_filt_init = true;
            } else {
                for k in 0..3 {
                    self.filt_w[k] += a * (est.omega[k].0 - self.filt_w[k]);
                }
            }
            self.filt_w
        } else {
            [est.omega[0].0, est.omega[1].0, est.omega[2].0]
        };
        let att_out = super::attitude::attitude_rates(
            est.att,
            q_des,
            self.att_kp,
            self.att_kd,
            omega_f,
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
                clampf(motors[0], 0.0, 1.0),
                clampf(motors[1], 0.0, 1.0),
                clampf(motors[2], 0.0, 1.0),
                clampf(motors[3], 0.0, 1.0),
            ],
        }
    }
}
