//! **估计器选择层**（迁移计划步 3 ✓）—— 让产品路径可切、默认 ESKF，并保留 Legacy 回退。
//!
//! 为何用**枚举**而非 trait 对象 ✓：本 crate 为 `no_std` 且**禁止堆分配** ✗
//! ⇒ `Box<dyn Estimator>` 不可用 ⇒ 用枚举 + 逐方法委托 ✓（见 `no_alloc.rs` 测试 ✓）。

use crate::estimator::eskf_estimator::EskfEstimator;
use crate::estimator::ekf::EkfEstimator;
use crate::estimator::trait_def::Estimator;
use crate::units::Second;
use crate::vehicle::{AirspeedSample, ImuSample, PosSample, Quaternion, RtkSample, VehicleState, VioSample};

/// 运行期可选的两个估计器 ✓（含"被拒绝通路"的可见计数 ✓）。
pub struct AnyEstimator {
    /// 实际实现 ✓
    pub inner: AnyEstimatorKind,
    /// ★ESKF 上被拒绝的 Legacy 专有通路次数 ✓（= 0 表示无此类注入 ✓）
    n_world_accel_refused: u32,
}

/// 实现选择 ✓
pub enum AnyEstimatorKind {
    /// ★**默认**：误差状态 EKF（C1+C2 ✓，见 `eskf.rs`）
    Eskf(EskfEstimator),
    /// 回退/对照：既有固定-α 锚定 EKF ✓
    Legacy(EkfEstimator),
}

impl Default for AnyEstimator {
    fn default() -> Self {
        Self::default_product()
    }
}

impl AnyEstimator {
    /// ★**产品默认**：ESKF ✓（迁移目标 ✓）
    pub fn default_product() -> Self {
        AnyEstimator {
            inner: AnyEstimatorKind::Eskf(EskfEstimator::default_quad()),
            n_world_accel_refused: 0,
        }
    }
    /// 回退/对照用 ✓
    pub fn legacy() -> Self {
        AnyEstimator {
            inner: AnyEstimatorKind::Legacy(EkfEstimator::default_quad()),
            n_world_accel_refused: 0,
        }
    }
    /// ★Legacy 专有的 oracle 注入（`set_world_accel` ✓，用于量化补偿收益上界 ✓）。
    ///
    /// ESKF **无等价入口** ✗（其世界加速度经速度/位置观测与重力门隐含处理 ✓）
    /// ⇒ 对 ESKF **显式拒绝并计数** ✓（绝不静默 no-op ✗）。
    pub fn set_world_accel(&mut self, a: [f32; 3]) {
        let refused = match &mut self.inner {
            AnyEstimatorKind::Legacy(e) => {
                e.set_world_accel(a);
                false
            }
            AnyEstimatorKind::Eskf(_) => true, // ★无等价入口 ⇒ 显式拒绝 ✓
        };
        if refused {
            self.n_world_accel_refused = self.n_world_accel_refused.wrapping_add(1);
        }
    }

    /// ★该通路在 ESKF 上被拒绝的次数 ✓（"拒绝也是可见的" ✓）
    pub fn world_accel_refused(&self) -> u32 {
        self.n_world_accel_refused
    }

    /// Legacy（带 `att_alpha` 锚定增益旋钮 ✓ —— **仅 Legacy 适用** ✗，ESKF 无此概念 ✓）
    pub fn legacy_with_alpha(a: f32) -> Self {
        AnyEstimator {
            inner: AnyEstimatorKind::Legacy(EkfEstimator::new(a, 0.5, 0.05, 1e-5, 5e-4, 0.5, 0.3, 0.3)),
            n_world_accel_refused: 0,
        }
    }

    /// 诊断：当前的实现名 ✓
    pub fn kind(&self) -> &'static str {
        match &self.inner {
            AnyEstimatorKind::Eskf(_) => "eskf",
            AnyEstimatorKind::Legacy(_) => "legacy",
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
        match &mut self.inner {
            AnyEstimatorKind::Eskf(e) => e.step(dt, imu, pos, airspeed),
            AnyEstimatorKind::Legacy(e) => e.step(dt, imu, pos, airspeed),
        }
    }
    fn update_vio(&mut self, vio: Option<VioSample>) {
        match &mut self.inner {
            AnyEstimatorKind::Eskf(e) => e.update_vio(vio),
            AnyEstimatorKind::Legacy(e) => e.update_vio(vio),
        }
    }
    fn update_rtk(&mut self, rtk: Option<RtkSample>) {
        match &mut self.inner {
            AnyEstimatorKind::Eskf(e) => e.update_rtk(rtk),
            AnyEstimatorKind::Legacy(e) => e.update_rtk(rtk),
        }
    }
    fn reset(&mut self) {
        match &mut self.inner {
            AnyEstimatorKind::Eskf(e) => e.reset(),
            AnyEstimatorKind::Legacy(e) => e.reset(),
        }
    }
    fn set_initial_attitude(&mut self, q: Quaternion) {
        match &mut self.inner {
            AnyEstimatorKind::Eskf(e) => e.set_initial_attitude(q),
            AnyEstimatorKind::Legacy(e) => e.set_initial_attitude(q),
        }
    }
    fn set_initial_position(&mut self, ned: [f32; 3]) {
        match &mut self.inner {
            AnyEstimatorKind::Eskf(e) => e.set_initial_position(ned),
            AnyEstimatorKind::Legacy(e) => e.set_initial_position(ned),
        }
    }
    fn update_alt(&mut self, alt: f32) {
        match &mut self.inner {
            AnyEstimatorKind::Eskf(e) => e.update_alt(alt),
            AnyEstimatorKind::Legacy(e) => e.update_alt(alt),
        }
    }
    fn update_mag(&mut self, mag: Option<[f32; 3]>) {
        match &mut self.inner {
            AnyEstimatorKind::Eskf(e) => e.update_mag(mag),
            AnyEstimatorKind::Legacy(e) => e.update_mag(mag),
        }
    }
    fn state(&self) -> VehicleState {
        match &self.inner {
            AnyEstimatorKind::Eskf(e) => e.state(),
            AnyEstimatorKind::Legacy(e) => e.state(),
        }
    }
    fn accel_bias(&self) -> [f32; 3] {
        match &self.inner {
            AnyEstimatorKind::Eskf(e) => e.accel_bias(),
            AnyEstimatorKind::Legacy(e) => e.accel_bias(),
        }
    }
}
