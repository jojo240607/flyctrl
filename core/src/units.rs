//! 类型安全的 SI 单位系统。
//!
//! 用新类型包装 `f32`，让"把角度当弧度传给控制器"这类错误在编译期暴露。
//! 飞控是硬实时系统，单位运算走 `f32` 足矣，且避免 `f64` 在 Cortex-M4 上的
//! 软件浮点开销（F4 无 FPU 双精度）。

use core::ops::{Add, Sub, Mul, Div};

macro_rules! unit {
    ($name:ident, $desc:literal) => {
        #[doc = $desc]
        #[derive(Debug, Clone, Copy, PartialEq)]
        pub struct $name(pub f32);

        impl $name {
            pub const ZERO: Self = Self(0.0);
            pub fn as_f32(self) -> f32 { self.0 }
            pub fn abs(self) -> Self { Self(self.0.abs()) }
        }

        impl Add for $name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self { Self(self.0 + rhs.0) }
        }
        impl Sub for $name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self { Self(self.0 - rhs.0) }
        }
    };
}

unit!(Meter, "长度 (m)");
unit!(Second, "时间 (s)");
unit!(Radian, "角度 (rad)");
unit!(MeterPerSecond, "线速度 (m/s)");
unit!(Airspeed, "空速 (m/s)，皮托管/差分气压测得的总压-静压差换算");
unit!(RadianPerSecond, "角速度 (rad/s)");
unit!(MeterPerSecondSquared, "线加速度 (m/s^2)");
unit!(Newton, "力 (N)");
unit!(NewtonMeter, "力矩 (N·m)");

impl Radian {
    /// 归一化到 (-π, π]
    pub fn wrapped(self) -> Self {
        let mut v = self.0;
        // 用除以 2π 取余再移位，避免循环
        const TAU: f32 = core::f32::consts::PI * 2.0;
        v = v - TAU * crate::math::round(v / TAU);
        Radian(v)
    }
}

// 标量乘法 / 除法
impl Mul<f32> for MeterPerSecond { type Output = Self; fn mul(self, s: f32) -> Self { Self(self.0 * s) } }
impl Mul<f32> for RadianPerSecond { type Output = Self; fn mul(self, s: f32) -> Self { Self(self.0 * s) } }
impl Div<f32> for MeterPerSecond { type Output = Self; fn div(self, s: f32) -> Self { Self(self.0 / s) } }
impl Div<f32> for RadianPerSecond { type Output = Self; fn div(self, s: f32) -> Self { Self(self.0 / s) } }
