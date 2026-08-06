//! 估计器统一接口。
//!
//! 输入为 IMU + 位置测量（可选 GPS/气压），输出为滤波后的 [`VehicleState`]。
//! `step` 必须是确定性、无堆分配、有界执行时间的，以满足硬实时要求。

use crate::units::Second;
use crate::vehicle::{ImuSample, PosSample, VehicleState};

pub trait Estimator {
    /// 推进一个采样周期，返回当前估计状态。
    fn step(&mut self, dt: Second, imu: ImuSample, pos: Option<PosSample>) -> VehicleState;

    /// 复位到初始/零状态。
    fn reset(&mut self);
}
