//! 控制律统一接口。
//!
//! `setpoint` 为期望状态（轨迹/姿态目标），`estimate` 为当前估计状态，
//! 返回 [`ActuatorCmd`]（各电机归一化推力）。核心只依赖 trait，不关心具体算法。

use crate::units::*;
pub use crate::vehicle::{ActuatorCmd, VehicleState};

#[derive(Debug, Clone, Copy)]
pub struct Setpoint {
    pub pos: [Meter; 3],       // 期望 NED 位置（D 向下为正，悬停通常为负高度）
    pub yaw: Radian,           // 期望偏航
    pub vel: [MeterPerSecond; 3], // 期望速度（可为零）
    /// 期望加速度（NED，m/s²，可为零）。轨迹跟踪前馈：速度中环把 `kv*(des_v - v)`
    /// 与 `acc` 前馈相加得期望世界系加速度 → 期望倾角（转弯/机动预倾，P3-A1）。
    pub acc: [MeterPerSecondSquared; 3],
}

impl Setpoint {
    pub fn hover(pos: [Meter; 3], yaw: Radian) -> Self {
        Self {
            pos,
            yaw,
            vel: [MeterPerSecond::ZERO; 3],
            acc: [MeterPerSecondSquared::ZERO; 3],
        }
    }
}

pub trait Controller {
    /// 计算控制输出。dt 为控制周期。
    fn control(&mut self, dt: Second, setpoint: &Setpoint, estimate: &VehicleState) -> ActuatorCmd;

    /// P3-A3：注入空速计测得的相对空速矢量（NED 水平，m/s）= v_ground - wind。
    ///
    /// 供 TECS 等需要"真空速方向"的控制律做气动拖拽前馈（`a_ff = k·|v_rel|·v_rel`）。
    /// 与 EKF 的 `est.airspeed`（地速幅值，用于速度融合）相互独立：这里的测量矢量
    /// 只做前馈，不污染速度估计。默认空实现——不需要该通道的控制器（PID/LQR/INDI/MPC）
    /// 零改动。每个控制周期由宿主在调用 `control` 之前写入。
    fn set_measured_airspeed_vec(&mut self, _v: [f32; 2]) {}

    /// ★§5.183：注入【测量/估计的世界系加速度】（NED，m/s²），供速度环 **D 项**。
    ///
    /// 与轨迹前馈 `Setpoint.acc` **语义分离**（PX4 `_vel_dot = states.acceleration` vs
    /// `_acc_sp = setpoint.acceleration` ✓）：原实现把估计的世界系加速度塞进 `Setpoint.acc`
    /// ✗ ⇒ 同一信号既当**轨迹前馈**又当 **D 项输入** ⇒ 语义冲突（有真前馈时 D 项拿到的是
    /// 轨迹加速度而非测量加速度；且测得的加速度被当作前馈**正反馈**注入控制律）。
    /// 默认空实现——不需要该通道的控制器（LQR/INDI/MPC）零改动。宿主在 `control` 前写入。
    fn set_world_accel(&mut self, _a: [MeterPerSecondSquared; 3]) {}

    /// 复位内环积分器等状态。
    fn reset(&mut self);

    /// ★C3：最近一次由控制律产出的**速率设定值**（PX4 `vehicle_rates_setpoint` 同构 ✓）。
    ///
    /// 默认 `INVALID`（非串级控制器不暴露 ✓）；`PidController` 返回姿态层输出 ✓。
    /// app 侧独立高频 `rate_task` 在 1kHz 消费它 + **新鲜陀螺**跑速率层 ✓。
    fn rate_setpoint(&self) -> RateSetpoint {
        RateSetpoint::INVALID
    }
}

/// 速率层设定值：**姿态层 → 速率层**的接口（PX4 `vehicle_rates_setpoint` 同构 ✓）。
///
/// ★C3 串级：姿态层（`mc_att_control` 同构）产出本结构；速率层（`mc_rate_control` 同构）
/// 按**自身周期**（可远高于姿态层）消费它，与实测陀螺一起算力矩→混控。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RateSetpoint {
    /// 期望机体角速率 (p, q, r)，rad/s（姿态层已按 `RATE_MAX_DPS` 限幅 ✓）。
    pub rates: [f32; 3],
    /// 集体推力（悬停油门基值；量纲同 `ActuatorCmd` 之前的 `des_thrust` ✓）。
    pub thrust: f32,
    /// 是否有效：false ⇒ 速率层输出零（健康/解锁闸由宿主置 false ✓）。
    pub valid: bool,
}

impl RateSetpoint {
    /// 无效设定值（速率层据 `valid` 输出零 ✓）。
    pub const INVALID: Self = Self { rates: [0.0; 3], thrust: 0.0, valid: false };
    /// 由姿态层产出：有效设定值 ✓。
    pub fn new(rates: [f32; 3], thrust: f32) -> Self {
        Self { rates, thrust, valid: true }
    }
}
