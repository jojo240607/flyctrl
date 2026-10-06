//! L0 · 单位（SI，量纲类型显式）
//!
//! 契约 §1：不做隐式数值转换。混合量纲必须显式转换，
//! 使"把角速度当角度用"这类错误在编译期暴露。

#[allow(unused_imports)]
use crate::math::F32Ext;

macro_rules! quantity {
    ($name:ident, $doc:literal) => {
        #[doc = $doc]
        #[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
        pub struct $name(pub f32);

        impl $name {
            pub const ZERO: Self = Self(0.0);
            /// 由裸数值构造。**只在模块边界（驱动/序列化）使用**。
            #[inline]
            pub const fn new(v: f32) -> Self {
                Self(v)
            }
            /// 取出裸数值。**只在模块边界使用**。
            #[inline]
            pub const fn value(self) -> f32 {
                self.0
            }
            #[inline]
            pub fn is_finite(self) -> bool {
                self.0.is_finite()
            }
        }
        impl core::ops::Add for $name {
            type Output = Self;
            #[inline]
            fn add(self, r: Self) -> Self {
                Self(self.0 + r.0)
            }
        }
        impl core::ops::Sub for $name {
            type Output = Self;
            #[inline]
            fn sub(self, r: Self) -> Self {
                Self(self.0 - r.0)
            }
        }
        impl core::ops::Neg for $name {
            type Output = Self;
            #[inline]
            fn neg(self) -> Self {
                Self(-self.0)
            }
        }
        /// 同量纲比值 ⇒ 纯数。
        impl core::ops::Div for $name {
            type Output = f32;
            #[inline]
            fn div(self, r: Self) -> f32 {
                self.0 / r.0
            }
        }
        /// 乘纯数。
        impl core::ops::Mul<f32> for $name {
            type Output = Self;
            #[inline]
            fn mul(self, r: f32) -> Self {
                Self(self.0 * r)
            }
        }
    };
}

quantity!(Meters, "长度/位置（m）");
quantity!(Mps, "速度（m/s）");
quantity!(Radians, "角度（rad）");
quantity!(Rps, "角速度（rad/s）");
quantity!(Seconds, "时间（s）");

/// 显式换算：位置 / 时间 ⇒ 速度。
#[inline]
pub fn meters_per_second(d: Meters, t: Seconds) -> Mps {
    Mps(d.0 / t.0)
}

/// 显式换算：角速度 × 时间 ⇒ 角度。
#[inline]
pub fn radians_from_rate(w: Rps, t: Seconds) -> Radians {
    Radians(w.0 * t.0)
}
