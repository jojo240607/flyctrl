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

    /// 复位内环积分器等状态。
    fn reset(&mut self);
}
