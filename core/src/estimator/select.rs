//! **估计器选择层**（迁移计划步 3 ✓）—— 让产品路径可切、默认 ESKF，并保留 Legacy 回退。
//!
//! 为何用**枚举**而非 trait 对象 ✓：本 crate 为 `no_std` 且**禁止堆分配** ✗
//! ⇒ `Box<dyn Estimator>` 不可用 ⇒ 用枚举 + 逐方法委托 ✓（见 `no_alloc.rs` 测试 ✓）。

use crate::estimator::eskf_estimator::EskfEstimator;
use crate::estimator::ekf::EkfEstimator;
use crate::estimator::trait_def::Estimator;
use crate::units::Second;
use crate::vehicle::{AirspeedSample, ImuSample, PosSample, Quaternion, RtkSample, VehicleState, VioSample};

/// 运行期可选的两个估计器 ✓。
pub enum AnyEstimator {
    /// ★**默认**：误差状态 EKF（C1+C2 ✓，见 `eskf.rs`）
    Eskf(EskfEstimator),
    /// 回退/对照：既有固定-α 锚定 EKF ✓
    Legacy(EkfEstimator),
}

impl AnyEstimator {
    /// ★**产品默认**：ESKF ✓（迁移目标 ✓）
    pub fn default_product() -> Self {
        AnyEstimator::Eskf(EskfEstimator::default_quad())
    }
    /// 回退/对照用 ✓
    pub fn legacy() -> Self {
        AnyEstimator::Legacy(EkfEstimator::default_quad())
    }
    /// 诊断：当前的实现名 ✓
    pub fn kind(&self) -> &'static str {
        match self {
            AnyEstimator::Eskf(_) => "eskf",
            AnyEstimator::Legacy(_) => "legacy",
        }
    }
}

/// 逐方法委托 ✓（机械但必须显式 ✗ 不得遗漏 ⇒ 漏一个就编译不过 ✓）
impl Estimator for AnyEstimator {
    fn step(
        &mut self,
        dt: Second,
        imu: ImuSample,
        pos: Option<PosSample>,
        airspeed: Option<AirspeedSample>,
    ) -> VehicleState {
        match self {
            AnyEstimator::Eskf(e) => e.step(dt, imu, pos, airspeed),
            AnyEstimator::Legacy(e) => e.step(dt, imu, pos, airspeed),
        }
    }
    fn update_vio(&mut self, vio: Option<VioSample>) {
        match self {
            AnyEstimator::Eskf(e) => e.update_vio(vio),
            AnyEstimator::Legacy(e) => e.update_vio(vio),
        }
    }
    fn update_rtk(&mut self, rtk: Option<RtkSample>) {
        match self {
            AnyEstimator::Eskf(e) => e.update_rtk(rtk),
            AnyEstimator::Legacy(e) => e.update_rtk(rtk),
        }
    }
    fn reset(&mut self) {
        match self {
            AnyEstimator::Eskf(e) => e.reset(),
            AnyEstimator::Legacy(e) => e.reset(),
        }
    }
    fn set_initial_attitude(&mut self, q: Quaternion) {
        match self {
            AnyEstimator::Eskf(e) => e.set_initial_attitude(q),
            AnyEstimator::Legacy(e) => e.set_initial_attitude(q),
        }
    }
    fn set_initial_position(&mut self, ned: [f32; 3]) {
        match self {
            AnyEstimator::Eskf(e) => e.set_initial_position(ned),
            AnyEstimator::Legacy(e) => e.set_initial_position(ned),
        }
    }
    fn update_alt(&mut self, alt: f32) {
        match self {
            AnyEstimator::Eskf(e) => e.update_alt(alt),
            AnyEstimator::Legacy(e) => e.update_alt(alt),
        }
    }
    fn update_mag(&mut self, mag: Option<[f32; 3]>) {
        match self {
            AnyEstimator::Eskf(e) => e.update_mag(mag),
            AnyEstimator::Legacy(e) => e.update_mag(mag),
        }
    }
    fn state(&self) -> VehicleState {
        match self {
            AnyEstimator::Eskf(e) => e.state(),
            AnyEstimator::Legacy(e) => e.state(),
        }
    }
    fn accel_bias(&self) -> [f32; 3] {
        match self {
            AnyEstimator::Eskf(e) => e.accel_bias(),
            AnyEstimator::Legacy(e) => e.accel_bias(),
        }
    }
}
