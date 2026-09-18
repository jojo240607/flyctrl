//! no_std 数学函数 shim。
//!
//! 核心 crate 禁用 std，故 `f32` 的 `.sin_cos()`/`.asin()` 等方法不可用。
//! 这里统一转发到 `libm`，控制层与估计层都从本模块取，避免在多处散落
//! `libm::` 调用（原先 fdir/vehicle/mission/manual/tecs/pid 里有十几处直接调用，
//! 现全部收敛到本模块）。
//!
//! 【已知开销】libm 0.2 的 `sinf`/`cosf`/`atan2f`/`logf`/`expf` 内部走 f64 通用
//! 路径（调用链里可见 `scalbn::<f64>`），会把 double 软件运行时拉进固件；F407 的
//! FPU 只支持单精度。试过换 `micromath` 2.1——**它内部同样使用 f64**，无改善，
//! 且 mcu_simulater 用 vendor/ 目录（只读），新增 crate 成本高，故不采用。
//! 后续若要认真消除 double，需手写纯 f32 的多项式近似。

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
