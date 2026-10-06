//! `fcalg` **控制器**桥接（feature `fcalg-ctrl`）—— 姿态环 / 速率环 / 混控
//!
//! 原则同估计器桥接：**薄**，只做字段搬运与显式登记，不含算法逻辑。
//!
//! # 与固件分层的关系（如实说明）
//! 固件的 `Controller::control` 是**姿态层**（产出 `RateSetpoint`，由 1kHz `rate_task`
//! 消费）；本桥接在同一函数里把**速率层 + 混控**也跑一遍，使返回值 `ActuatorCmd`
//! 可独立使用（旧栈的结构也是"姿态层产出设定值 + 速率层消费"）。
//!
//! # ★已登记（本桥接**不实现**，且**必须计数**，绝不静默）
//! `set_measured_airspeed_vec` / `set_world_accel` —— 新栈无对应通道。
//!
//! # ★★判据现状（更新，见 `desired_attitude` 的文档）
//! ✅ **推力方向约定已获判据**（`thrust_direction_matches_required_accel`）——
//!    这条完全确定符号，并覆盖"加速北向 ⇒ 机头下俯"。
//! ⚠**仍登记**：倾角**如何在 roll/pitch 之间分配**是**约定选择**（多解），
//!    需一手参照（PX4 `quat_from_axis_angle` 的半角分配）才能定。
//!    ⇒ **在该项取证之前不要把本桥接用于真机。**

use core::sync::atomic::{AtomicU32, Ordering};

use fcalg::attitude::attitude_rate_setpoint;
use fcalg::mixer::x4_mix;
use fcalg::quat::Quat;

use crate::controller::trait_def::{Controller, RateSetpoint, Setpoint};
use crate::units::{MeterPerSecondSquared, Second};
use crate::vehicle::{ActuatorCmd, VehicleState};

/// 被调用但**本桥接不实现**的方法次数（绝不静默）。
pub static CTRL_NOT_IMPLEMENTED_CALLS: AtomicU32 = AtomicU32::new(0);

/// 期望加速度 → 期望姿态（yaw 保持设定值）。
///
/// 构造：`z_b_des = −normalize(a_des − g_ned)`（机体系 z 向下；推力沿 −z_b）
/// ⇒ 先求"把世界 ẑ 转到 z_b_des"的最短旋转 `q_tilt`，再叠加偏航：`q_des = q_yaw ∘ q_tilt`。
/// # 判据现状（更新）
/// ✅ **方向约定已有判据**：判据为"期望姿态下机体 −z 轴（推力方向）在世界系
///    必须指向 `a_des − g_ned`"—— 这条**完全确定符号**，并覆盖"加速北向 ⇒ 机头下俯"。
/// ✅ 零加速度 ⇒ 退化为纯偏航（有判据）。
/// ⚠**仍登记的约定选择**：倾角**如何在 roll/pitch 之间分配**（多个解都满足方向约束）。
///    PX4 用"半角分配"（`quat_from_axis_angle` 的 tilt 均分）；本实现未对该分配方式取证。
///    ⇒ 需要一手参照才能定；**在此之前不要把本桥接用于真机**。
pub fn desired_attitude(a_des: [f32; 3], yaw: f32) -> Quat {
    const G: f32 = 9.806_65;
    let f = [a_des[0], a_des[1], a_des[2] - G]; // = a_des − g_ned（g_ned = (0,0,+G)）
    let n = ffi_sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
    if !(n > 1e-6) {
        return Quat::from_euler_zyx([0.0, 0.0, yaw]);
    }
    let zb = [-f[0] / n, -f[1] / n, -f[2] / n]; // 期望的机体 z 轴（世界系）
    // 最短旋转：世界 ẑ → zb
    let (w0, x0, y0, z0) = (0.0f32, 0.0f32, 0.0f32, 1.0f32); // 世界 ẑ 作为四元数轴
    let d = z0 * zb[2] + x0 * zb[0] + y0 * zb[1] + w0 * 0.0;
    let q_tilt = if d > 0.999_999 {
        Quat::IDENTITY
    } else if d < -0.999_999 {
        // 反向：绕任意垂直轴 180°
        Quat { w: 0.0, x: 1.0, y: 0.0, z: 0.0 }
    } else {
        let c = [y0 * zb[2] - z0 * zb[1], z0 * zb[0] - x0 * zb[2], x0 * zb[1] - y0 * zb[0]];
        Quat { w: 1.0 + d, x: c[0], y: c[1], z: c[2] }
    };
    let q_yaw = Quat::from_euler_zyx([0.0, 0.0, yaw]);
    q_yaw.mul(q_tilt).normalize().unwrap_or(Quat::IDENTITY)
}

#[inline]
fn ffi_sqrt(x: f32) -> f32 {
    // 走 fcalg 的数学入口，避免在固件上引入 std
    fcalg::math::sqrt(x)
}

/// 新栈控制器适配器。
pub struct FcalgController {
    /// 位置环 P（1/s）。
    pub kp_pos: f32,
    /// 速度环 P（1/s）。
    pub kp_vel: f32,
    /// 姿态环 P（1/s）。
    pub kp_att: f32,
    /// 速率环 P。
    pub kp_rate: f32,
    /// 最近一次速率设定值。
    pub rate_sp: RateSetpoint,
    /// 本实例被调用但未实现的次数。
    pub not_impl_calls: u32,
}

impl Default for FcalgController {
    fn default() -> Self {
        Self::new()
    }
}

impl FcalgController {
    pub fn new() -> Self {
        Self {
            kp_pos: 1.0,
            kp_vel: 2.0,
            kp_att: 4.0,
            kp_rate: 0.15,
            rate_sp: RateSetpoint::INVALID,
            not_impl_calls: 0,
        }
    }
}

impl Controller for FcalgController {
    fn control(&mut self, _dt: Second, sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        // ① 位置环：v_sp = sp.vel + kp_pos·(sp.pos − pos)
        let mut v_sp = [0.0f32; 3];
        for a in 0..3 {
            v_sp[a] = sp.vel[a].0 + self.kp_pos * (sp.pos[a].0 - est.pos[a].0);
        }
        // ② 速度环：a_des = acc_ff + kp_vel·(v_sp − v)
        let mut a_des = [0.0f32; 3];
        for a in 0..3 {
            a_des[a] = sp.acc[a].0 + self.kp_vel * (v_sp[a] - est.vel[a].0);
        }
        // ③ 集体推力比：|a_des − g_ned| / g（悬停 = 1.0；混控域 [0,1]）
        const G: f32 = 9.806_65;
        let f = [a_des[0], a_des[1], a_des[2] - G];
        let n = ffi_sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
        let thrust = (n / G).clamp(0.0, 1.0);
        // ④ 期望姿态（yaw 用设定值）→ ⑤ 姿态误差 → 机体速率设定值
        let q_des = desired_attitude(a_des, sp.yaw.0);
        let q_est = Quat { w: est.att.w, x: est.att.x, y: est.att.y, z: est.att.z };
        let rates = attitude_rate_setpoint(q_est, q_des, self.kp_att);
        self.rate_sp = RateSetpoint::new(rates, thrust);
        // ⑥ 速率层 P（用实测机体角速度）+ 混控
        let mut torque = [0.0f32; 3];
        for a in 0..3 {
            torque[a] = self.kp_rate * (rates[a] - est.omega[a].0);
        }
        match x4_mix(thrust, torque) {
            Ok(m) => {
                let mut c = ActuatorCmd::zero();
                c.motor = m.0; // 用 zero() 再改字段 ⇒ 不依赖 ActuatorCmd 还有哪些字段
                c
            }
            Err(_) => ActuatorCmd::zero(),
        }
    }

    fn rate_setpoint(&self) -> RateSetpoint {
        self.rate_sp
    }

    fn reset(&mut self) {
        self.rate_sp = RateSetpoint::INVALID;
    }

    // ── 两处"无对应"：显式登记并计数（绝不静默 no-op）──────────────
    fn set_measured_airspeed_vec(&mut self, _v: [f32; 2]) {
        self.not_impl_calls = self.not_impl_calls.wrapping_add(1);
        CTRL_NOT_IMPLEMENTED_CALLS.fetch_add(1, Ordering::Relaxed);
    }
    fn set_world_accel(&mut self, _a: [MeterPerSecondSquared; 3]) {
        self.not_impl_calls = self.not_impl_calls.wrapping_add(1);
        CTRL_NOT_IMPLEMENTED_CALLS.fetch_add(1, Ordering::Relaxed);
    }
}

impl FcalgController {
    /// 供测试：显式触发未实现通路（避免测试去拼 trait 对象）。
    pub fn touch_unimplemented(&mut self) {
        self.set_measured_airspeed_vec([0.0; 2]);
        self.set_world_accel([MeterPerSecondSquared(0.0); 3]);
    }
    /// 便于测试的字段搬运入口（与 trait 的 `control` 同源）。
    pub fn rates_for(&self, sp: &Setpoint, est: &VehicleState) -> [f32; 3] {
        let mut c = FcalgController::new();
        c.kp_pos = self.kp_pos;
        c.kp_vel = self.kp_vel;
        c.kp_att = self.kp_att;
        c.kp_rate = self.kp_rate;
        let _ = c.control(Second(0.004), sp, est);
        c.rate_sp.rates
    }
}

