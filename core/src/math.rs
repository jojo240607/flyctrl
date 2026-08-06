//! no_std 数学函数 shim。
//!
//! 核心 crate 禁用 std，故 `f32` 的 `.sin_cos()`/`.asin()` 等方法不可用。
//! 这里统一转发到 `libm`，控制层与估计层都从本模块取，避免在多处散落
//! `libm::` 调用。后续若移植到带硬件 FPU 的固件，可在此切换为内建指令。

pub fn sin(x: f32) -> f32 { libm::sinf(x) }
pub fn cos(x: f32) -> f32 { libm::cosf(x) }
pub fn sin_cos(x: f32) -> (f32, f32) { (libm::sinf(x), libm::cosf(x)) }
pub fn asin(x: f32) -> f32 { libm::asinf(x) }
pub fn atan2(y: f32, x: f32) -> f32 { libm::atan2f(y, x) }
pub fn sqrt(x: f32) -> f32 { libm::sqrtf(x) }
pub fn ln(x: f32) -> f32 { libm::logf(x) }
pub fn exp(x: f32) -> f32 { libm::expf(x) }
pub fn abs(x: f32) -> f32 { libm::fabsf(x) }
pub fn round(x: f32) -> f32 { libm::roundf(x) }
pub fn clamp(x: f32, lo: f32, hi: f32) -> f32 { if x < lo { lo } else if x > hi { hi } else { x } }
