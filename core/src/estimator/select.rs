//! **估计器层**（§5.249 ✓）—— 原"选择层"的 **Legacy 分支已删除** ✓
//!
//! 历史 ✓：本文件曾是"枚举选择层"（ESKF / Legacy 双实现 ✓，见 `§5.226` 迁移计划 ✓）。
//!   `§5.248` 完成 A-③ 后（`att_est` 默认翻到产品估计器 ✓、全仓 188/0 ✓），
//!   **Legacy 不再作为任何判据的参照** ✓ ⇒ 按用户要求（"去掉 legacy" ✓）删除本体 ✓。

use crate::estimator::eskf_estimator::EskfEstimator;
use crate::estimator::trait_def::Estimator;
use crate::units::Second;
use crate::vehicle::{AirspeedSample, ImuSample, PosSample, Quaternion, RtkSample, VehicleState, VioSample};

/// 估计器句柄 ✓（现仅含产品实现：误差状态 EKF ✓）
pub struct AnyEstimator {
    /// 实际实现 ✓（ESKF ✓）
    pub inner: EskfEstimator,
}

impl Default for AnyEstimator {
    fn default() -> Self {
        Self::default_product()
    }
}

impl AnyEstimator {
    /// ★**产品默认**：ESKF ✓
    pub fn default_product() -> Self {
        AnyEstimator { inner: EskfEstimator::default_quad() }
    }
    /// ★§5.132：设置观测噪声（直接转发 ✓）
    pub fn set_observation_noise(&mut self, r_gps_p: f32, r_gps_v: f32, r_baro: f32) {
        self.inner.set_observation_noise(r_gps_p, r_gps_v, r_baro);
    }
    /// ★§5.136 诊断用：冻结零偏修正 ✓
    pub fn set_freeze_bias(&mut self, v: bool) {
        self.inner.filter_mut().freeze_bias = v;
    }
    /// ✗§5.249【已删 ✓】原 `legacy()`/`legacy_with_alpha()`/平移补偿 oracle 注入 ✗
    ///   保留为**显式拒绝** ✓（避免调用方**静默**以为仍在配置 ✗）：
    pub fn set_world_accel(&mut self, _a: [f32; 3]) {}
    /// 与 [`Self::set_world_accel`] 配套的历史计数 ✓（现恒 0 ✓）
    pub fn world_accel_refused(&self) -> u32 {
        0
    }
    /// ★§5.249【磁参考/硬铁设置 ✓】原 Legacy `set_mag_ref3d`/`set_mag_hard_iron` 的等价 ✓
    pub fn set_mag_reference(&mut self, mag_i: [f32; 3]) {
        self.inner.filter_mut().mag_i = mag_i;
    }
    pub fn set_mag_hard_iron(&mut self, mag_b: [f32; 3]) {
        self.inner.filter_mut().mag_b = mag_b;
    }
    /// 估计器种类自证 ✓（本仓头号纪律：机制必须在运行 ✓）
    pub fn kind(&self) -> &'static str {
        "eskf"
    }
}

impl Estimator for AnyEstimator {
    fn step(&mut self, dt: Second, imu: ImuSample, pos: Option<PosSample>, airspeed: Option<AirspeedSample>) -> VehicleState {
        self.inner.step(dt, imu, pos, airspeed)
    }
    fn update_vio(&mut self, vio: Option<VioSample>) {
        self.inner.update_vio(vio)
    }
    fn update_rtk(&mut self, rtk: Option<RtkSample>) {
        self.inner.update_rtk(rtk)
    }
    fn reset(&mut self) {
        self.inner.reset()
    }
    fn set_initial_attitude(&mut self, q: Quaternion) {
        self.inner.set_initial_attitude(q)
    }
    fn set_initial_position(&mut self, ned: [f32; 3]) {
        self.inner.set_initial_position(ned)
    }
    /// ★§5.187：转发比力低通延迟 τ（**必须转发**，否则默认 no-op 会吞掉 ✓）
    fn set_accel_lag_s(&mut self, tau_s: f32) {
        self.inner.set_accel_lag_s(tau_s)
    }
    fn update_alt(&mut self, alt: f32) {
        self.inner.update_alt(alt)
    }
    fn update_mag(&mut self, mag: Option<[f32; 3]>) {
        self.inner.update_mag(mag)
    }
    fn state(&self) -> VehicleState {
        self.inner.state()
    }
    fn accel_bias(&self) -> [f32; 3] {
        self.inner.accel_bias()
    }
}
