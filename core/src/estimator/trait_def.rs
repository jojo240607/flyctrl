//! 估计器统一接口。
//!
//! 输入为 IMU + 位置测量（可选 GPS/气压），输出为滤波后的 [`VehicleState`]。
//! `step` 必须是确定性、无堆分配、有界执行时间的，以满足硬实时要求。

use crate::units::Second;
use crate::vehicle::{AirspeedSample, ImuSample, PosSample, Quaternion, RtkSample, VehicleState, VioSample};

pub trait Estimator {
    /// 推进一个采样周期，返回当前估计状态。
    /// - imu：必选（加速度计 + 陀螺）
    /// - pos：可选（高度计 / GPS 位置）
    /// - airspeed：可选（空速计，约束水平速度幅值）
    fn step(
        &mut self,
        dt: Second,
        imu: ImuSample,
        pos: Option<PosSample>,
        airspeed: Option<AirspeedSample>,
    ) -> VehicleState;

    /// 注入 VIO（视觉里程计）测量：高频位置/速度（短期准、长期漂移）。
    ///
    /// EKF 用中等位置噪声 + 较小速度噪声融合，填补 GPS 帧间 / 失锁时的估计空白。
    /// 默认 no-op（无 VIO 的估计器不受影响），需融合的估计器（如 [`EkfEstimator`]）
    /// 覆写实现具体更新。
    fn update_vio(&mut self, _vio: Option<VioSample>) {}

    /// 注入 RTK-GPS 测量：厘米级高精度位置（NED）。
    ///
    /// EKF 用极小位置观测噪声融合，把位置协方差压到厘米量级、抑制 VIO 长期漂移
    /// （RTK 提供绝对参考）。默认 no-op（无 RTK 的估计器不受影响）。
    fn update_rtk(&mut self, _rtk: Option<RtkSample>) {}

    /// 复位到初始/零状态。
    fn reset(&mut self);

    /// HIL/共享单步：设置初始姿态四元数（首帧 IMU 重力向量 tilt alignment 后调用）。
    /// 默认 no-op（不支持显式姿态初始化的估计器不受影响）。
    fn set_initial_attitude(&mut self, _q: Quaternion) {}

    /// HIL/共享单步：设置初始位置估计（NED，m），使首拍位置误差≈0。
    /// 默认 no-op（不支持显式位置初始化的估计器不受影响）。
    fn set_initial_position(&mut self, _ned: [f32; 3]) {}

    /// 注入气压高度观测（m，向上为正），锚定垂直通道（predict-then-correct）。
    /// 默认 no-op（无气压融合的估计器不受影响）。
    fn update_alt(&mut self, _alt: f32) {}

    /// 当前估计状态（不推进），供诊断读取（已含所有已融合观测）。
    fn state(&self) -> VehicleState;

    /// 当前估计的加计零偏（机体系，m/s²）。默认实现返回零，
    /// 仅 EKF 等显式估计零偏的估计器会返回真实值。
    fn accel_bias(&self) -> [f32; 3] {
        [0.0; 3]
    }
}
