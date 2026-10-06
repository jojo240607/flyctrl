//! L0 · 数学 shim —— 全模块唯一的数学入口（**按目标平台分流**）
//!
//! 分流照搬旧栈 `flyctrl-core/src/math.rs` 的既有约定：
//! - `target_os = "none"`（固件）：`fpmath` —— **纯 f32 实现**，不会把 double 软件运行时拉进固件；
//! - host（SIL / 测试）：`libm` —— 刻意**与固件同一套 f32 实现**，避免"host 绿而固件不一致"。
//!
//! 于是"数学实现"与"算法约定"不纠缠：要换后端只改本文件。

#[cfg(target_os = "none")]
mod backend {
    pub fn sin(x: f32) -> f32 {
        fpmath::sin(x)
    }
    pub fn cos(x: f32) -> f32 {
        fpmath::cos(x)
    }
    pub fn asin(x: f32) -> f32 {
        fpmath::asin(x)
    }
    pub fn atan2(y: f32, x: f32) -> f32 {
        fpmath::atan2(y, x)
    }
    pub fn sqrt(x: f32) -> f32 {
        fpmath::sqrt(x)
    }
    pub fn exp(x: f32) -> f32 {
        fpmath::exp(x)
    }
}

#[cfg(not(target_os = "none"))]
mod backend {
    pub fn sin(x: f32) -> f32 {
        libm::sinf(x)
    }
    pub fn cos(x: f32) -> f32 {
        libm::cosf(x)
    }
    pub fn asin(x: f32) -> f32 {
        libm::asinf(x)
    }
    pub fn atan2(y: f32, x: f32) -> f32 {
        libm::atan2f(y, x)
    }
    pub fn sqrt(x: f32) -> f32 {
        libm::sqrtf(x)
    }
    pub fn exp(x: f32) -> f32 {
        libm::expf(x)
    }
}

pub use backend::*;

/// `f32` 的软件数学扩展 —— **为 `no_std` 目标补齐固有方法在 core 中缺失的那些**。
/// host 上固有方法优先 ⇒ **主机行为零变化**；固件上由本 trait 提供。
/// 用扩展 trait 而不是逐个改调用点：既有代码在两种目标下都能编，且不必重审每处语义。
pub trait F32Ext {
    fn sqrt(self) -> f32;
    fn atan2(self, y: f32) -> f32;
}
impl F32Ext for f32 {
    #[inline]
    fn sqrt(self) -> f32 {
        crate::math::sqrt(self)
    }
    #[inline]
    fn atan2(self, y: f32) -> f32 {
        crate::math::atan2(self, y)
    }
}
