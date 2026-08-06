//! 控制律统一接口。
//!
//! `setpoint` 为期望状态（轨迹/姿态目标），`estimate` 为当前估计状态，
//! 返回 [`ActuatorCmd`]（各电机归一化推力）。核心只依赖 trait，不关心具体算法。

use crate::units::*;
use crate::vehicle::{ActuatorCmd, VehicleState};

pub struct Setpoint {
    pub pos: [Meter; 3],       // 期望 NED 位置（D 向下为正，悬停通常为负高度）
    pub yaw: Radian,           // 期望偏航
    pub vel: [MeterPerSecond; 3], // 期望速度（可为零）
}

impl Setpoint {
    pub fn hover(pos: [Meter; 3], yaw: Radian) -> Self {
        Self { pos, yaw, vel: [MeterPerSecond::ZERO; 3] }
    }
}

pub trait Controller {
    /// 计算控制输出。dt 为控制周期。
    fn control(&mut self, dt: Second, setpoint: &Setpoint, estimate: &VehicleState) -> ActuatorCmd;

    /// 复位内环积分器等状态。
    fn reset(&mut self);
}
