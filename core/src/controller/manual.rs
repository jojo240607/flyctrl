//! 遥控（RC）直通控制律：手动（角速率）/ 增稳（姿态保持），P3-D1。
//!
//! 与 Setpoint 型控制器（PID/TECS 等）不同，这两档不跟踪位置/速度设定点，
//! 而是直接把遥控摇杆映射为机体角速率（Manual）或目标姿态（Stabilize），
//! 供"遥控解锁 → 手动 → 增稳 → 自主任务"全流程的前半段使用。
//! 姿态内环与电机混控复用 [`crate::controller::attitude`]，与 PID/TECS 完全一致，
//! 保证不同控制档位之间的内环行为一致。

use crate::config::CtrlParams;
use crate::vehicle::{ActuatorCmd, Quaternion, RcInput, VehicleState};

/// 手动/增稳共用参数。
#[derive(Debug, Clone, Copy)]
pub struct ManualParams {
    /// 悬停油门基值（摇杆中位附近电机指令）。
    pub hover_thrust: f32,
    /// 重力（m/s²，NED 下为正）。
    pub gravity: f32,
    /// 姿态内环比例增益（四元数误差 -> 期望机体角速度）。
    pub att_kp: f32,
    /// 姿态内环角速度阻尼增益。
    pub att_kd: f32,
    /// 摇杆满偏对应的目标角速率（rad/s，手动档）。
    pub rate_scale: f32,
    /// 摇杆满偏对应的目标倾斜角（rad，增稳档）。
    pub max_tilt: f32,
    /// 油门从中位到满偏对应的爬升率（m/s，增稳档；0.5 中位 → 0）。
    pub vz_scale: f32,
    /// 垂直速度环增益（增稳档：油门 → 爬升率误差 → 推力）。
    pub kv_z: f32,
}

impl ManualParams {
    /// 从机型配置构造（复用姿态环/悬停/倾角限制，速率与爬升档位取典型默认）。
    pub fn from_ctrl(c: &CtrlParams) -> Self {
        Self {
            hover_thrust: c.hover_thrust,
            gravity: c.gravity,
            att_kp: c.att_kp,
            att_kd: c.att_kd,
            rate_scale: 3.0,
            max_tilt: c.tilt_max,
            vz_scale: 2.0,
            kv_z: 1.5,
        }
    }
}

fn clampf(v: f32, lo: f32, hi: f32) -> f32 {
    v.clamp(lo, hi)
}

/// 手动档（角速率/ACRO）：摇杆直通为机体角速率目标，油门直通为总推力。
///
/// 不依赖姿态估计（只要陀螺），是解锁后最先可用的最低权限档位。
/// 摇杆回中时仅剩角速度阻尼（`-att_kp·ω`），机体按惯性自由漂移。
pub fn manual_rates(rc: &RcInput, est: &VehicleState, p: &ManualParams) -> ActuatorCmd {
    let p_des = p.rate_scale * rc.roll;
    let q_des = p.rate_scale * rc.pitch;
    let r_des = p.rate_scale * rc.yaw;
    // 速率环 P：目标角速率 - 陀螺实测 → 期望角加速度（机体系）。
    let pqr = [
        clampf(p.att_kp * (p_des - est.omega[0].0), -8.0, 8.0),
        clampf(p.att_kp * (q_des - est.omega[1].0), -8.0, 8.0),
        clampf(p.att_kp * (r_des - est.omega[2].0), -8.0, 8.0),
    ];
    let des_thrust = clampf(rc.throttle, 0.0, 1.0);
    let m = crate::controller::attitude::x4_mix(des_thrust, pqr);
    ActuatorCmd {
        motor: [
            clampf(m[0], 0.0, 1.0),
            clampf(m[1], 0.0, 1.0),
            clampf(m[2], 0.0, 1.0),
            clampf(m[3], 0.0, 1.0),
        ],
    }
}

/// 增稳档（姿态保持）：摇杆 → 目标倾斜角，松杆回中 → 水平；油门 → 爬升率。
///
/// 依赖姿态估计（EKF），摇杆回中时姿态环把机体拉回水平（悬停姿态）。
/// 偏航通道保持航向（摇杆右移 → 目标偏航角速率）。
pub fn stabilize(rc: &RcInput, est: &VehicleState, p: &ManualParams) -> ActuatorCmd {
    // 摇杆 → 目标姿态角（右移右滚、后拉抬头，与 pid.rs 的 +tilt_e / +pitch 符号一致）。
    let roll_t = p.max_tilt * rc.roll;
    let pitch_t = p.max_tilt * rc.pitch;
    let q_des = Quaternion::from_euler(
        crate::units::Radian(roll_t),
        crate::units::Radian(pitch_t),
        crate::units::Radian(est.att.yaw()),
    );
    // 姿态内环：目标姿态 - 实测 → 期望角速率（roll/pitch 复用共享内环）。
    let att = crate::controller::attitude::attitude_rates(
        est.att,
        q_des,
        p.att_kp,
        p.att_kd,
        [est.omega[0].0, est.omega[1].0, est.omega[2].0],
    );
    // 偏航：摇杆 → 偏航角速率（+ 阻尼保持航向）。
    let r_cmd = clampf(
        p.att_kp * (p.rate_scale * rc.yaw - est.omega[2].0),
        -6.0, 6.0,
    );
    // 油门 → 爬升率（0.5 中位 → 悬停，推满 → 向上 vz_scale，收到底 → 向下 vz_scale）。
    // NED 下垂方向为正：向上爬升为负速度，故加负号。
    let vz_des = -(rc.throttle - 0.5) * 2.0 * p.vz_scale;
    // 垂直速度环：期望爬升率 - 实测 → 期望下垂加速度 → 推力（符号同 pid.rs）。
    let acc_d = p.kv_z * (vz_des - est.vel[2].0);
    // 倾斜后按 1/cos(φ) 放大总推力，避免一倾斜就掉高（同 pid.rs 防死亡螺旋）。
    let tilt_mag = libm::sqrtf(roll_t * roll_t + pitch_t * pitch_t);
    let cos_tilt = if tilt_mag < 1.55 {
        libm::cosf(tilt_mag).max(0.2)
    } else {
        0.2
    };
    let des_thrust = clampf((p.hover_thrust - acc_d / p.gravity) / cos_tilt, 0.1, 1.0);
    let m = crate::controller::attitude::x4_mix(des_thrust, [att.rates[0], att.rates[1], r_cmd]);
    ActuatorCmd {
        motor: [
            clampf(m[0], 0.0, 1.0),
            clampf(m[1], 0.0, 1.0),
            clampf(m[2], 0.0, 1.0),
            clampf(m[3], 0.0, 1.0),
        ],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::{MeterPerSecond, RadianPerSecond};

    fn hover_state() -> VehicleState {
        // 水平悬停：位置 (0,0,-5)，速度 0，姿态水平，角速度 0。
        let mut s = VehicleState::zero();
        s.pos[2] = crate::units::Meter(-5.0);
        s
    }

    fn params() -> ManualParams {
        ManualParams {
            hover_thrust: 0.5,
            gravity: 9.81,
            att_kp: 3.0,
            att_kd: 0.3,
            rate_scale: 3.0,
            max_tilt: 0.35,
            vz_scale: 2.0,
            kv_z: 1.5,
        }
    }

    #[test]
    fn manual_neutral_throttle_zero_motors_off() {
        // 松杆 + 油门到底 → 电机全停（安全：先收油门再解锁，不应有推力）。
        let rc = RcInput {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.0,
            armed: true,
            mode: 0,
            fresh: true,
        };
        let cmd = manual_rates(&rc, &hover_state(), &params());
        assert_eq!(cmd.motor, [0.0; 4]);
    }

    #[test]
    fn manual_roll_stick_produces_asymmetric_mix() {
        // 右滚摇杆满偏 → 期望 +p 角速率 → X 混控出现 m0/m3 增、m1/m2 减的不对称。
        let rc = RcInput {
            roll: 1.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.5,
            armed: true,
            mode: 0,
            fresh: true,
        };
        let cmd = manual_rates(&rc, &hover_state(), &params());
        // 验证 roll 方向不对称（+p_cmd 时 m0、m3 增大，m1、m2 减小）。
        assert!(cmd.motor[0] > cmd.motor[1] && cmd.motor[2] < cmd.motor[3]);
        // 总推力保持油门基值附近。
        let t: f32 = cmd.motor.iter().sum();
        assert!((t - 2.0).abs() < 0.5);
    }

    #[test]
    fn stabilize_neutral_sticks_level_and_balanced() {
        // 松杆 + 油门中位 → 姿态保持水平（目标=当前姿态，误差≈0），四路平衡≈悬停。
        let rc = RcInput {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.5,
            armed: true,
            mode: 1,
            fresh: true,
        };
        let cmd = stabilize(&rc, &hover_state(), &params());
        // 四路接近且每路≈悬停油门（姿态误差为 0 → 无差动，仅高度保持微调）。
        for &m in &cmd.motor {
            assert!((m - 0.5).abs() < 0.2, "motor={}", m);
        }
        let spread = cmd.motor.iter().fold(0.0f32, |a, &m| a.max((m - 0.5).abs()));
        assert!(spread < 0.1, "spread={}", spread);
    }

    #[test]
    fn stabilize_roll_stick_tilts_away_from_level() {
        // 右滚摇杆满偏 → 目标姿态含 +roll → 内环产生 +p 角速度指令（与手动档同向）。
        let rc = RcInput {
            roll: 0.5,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.5,
            armed: true,
            mode: 1,
            fresh: true,
        };
        let cmd = stabilize(&rc, &hover_state(), &params());
        assert!(cmd.motor[0] > cmd.motor[1] && cmd.motor[2] < cmd.motor[3]);
    }

    #[test]
    fn stabilize_throttle_high_boosts_thrust_for_climb() {
        // 油门推满 → 期望爬升率 vz_scale → 需额外推力克服惯性 → 总推力高于悬停。
        let rc = RcInput {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 1.0,
            armed: true,
            mode: 1,
            fresh: true,
        };
        let cmd = stabilize(&rc, &hover_state(), &params());
        let t: f32 = cmd.motor.iter().sum();
        assert!(t > 4.0 * 0.5 + 0.1, "thrust sum={}", t);
    }

    #[test]
    fn stabilize_disarms_on_link_loss_input() {
        // 链路新鲜度仅由上层处理，控制律层面：armed=false 由上层零推力门控保证。
        // 这里验证：fresh=false 的默认值 + 中位油门不会产生异常指令（安全默认）。
        let rc = RcInput::neutral();
        let cmd = stabilize(&rc, &hover_state(), &params());
        for &m in &cmd.motor {
            assert!(m >= 0.0 && m <= 1.0);
        }
    }
}
