//! L2 · 双二次滤波器（陷波 / 低通）
//!
//! 统一用 **DF2T**（Direct Form II Transposed）：每轴只有 2 个状态 f32，数值性质好。
//!
//! 契约要点：
//! - 系数由 `(fs, f, Q)` 解析算出，**不缓存在滤波器实例里** —— 采样率一变必须重建；
//! - 直流增益可解析验证（低通 = 1；陷波在 f0 处 = 0）；
//! - 状态独立（每轴一个实例，见 `imu_filter`）。

use crate::math;

/// 归一化后的系数（约定 `a0 = 1`）：
/// `H(z) = (b0 + b1·z⁻¹ + b2·z⁻²) / (1 + a1·z⁻¹ + a2·z⁻²)`
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coeffs {
    pub b0: f32,
    pub b1: f32,
    pub b2: f32,
    pub a1: f32,
    pub a2: f32,
}

impl Coeffs {
    /// 一阶低通（单极点），表达为 biquad（`b2 = a2 = 0`）。
    ///
    /// `a = 1 − e^(−2π·fc/fs)` ⇒ `y[n] = a·x[n] + (1−a)·y[n−1]`，直流增益**恰为 1**。
    pub fn lowpass1(fs: f32, fc: f32) -> Self {
        let a = 1.0 - math::exp(-2.0 * core::f32::consts::PI * fc / fs);
        Self { b0: a, b1: 0.0, b2: 0.0, a1: -(1.0 - a), a2: 0.0 }
    }

    /// 二阶低通（RBJ cookbook）。`Q = 0.7071` 时阶跃响应无过冲。
    pub fn lowpass2(fs: f32, fc: f32, q: f32) -> Self {
        let w0 = 2.0 * core::f32::consts::PI * fc / fs;
        let (sw, cw) = (math::sin(w0), math::cos(w0));
        let alpha = sw / (2.0 * q);
        let b1 = 1.0 - cw;
        Self::norm(
            1.0 + alpha,
            b1 * 0.5,
            b1,
            b1 * 0.5,
            -2.0 * cw,
            1.0 - alpha,
        )
    }

    /// 陷波（RBJ cookbook）：在 `f0` 处增益**恰为 0**。
    pub fn notch(fs: f32, f0: f32, q: f32) -> Self {
        let w0 = 2.0 * core::f32::consts::PI * f0 / fs;
        let (sw, cw) = (math::sin(w0), math::cos(w0));
        let alpha = sw / (2.0 * q);
        Self::norm(1.0 + alpha, 1.0, -2.0 * cw, 1.0, -2.0 * cw, 1.0 - alpha)
    }

    fn norm(a0: f32, b0: f32, b1: f32, b2: f32, a1: f32, a2: f32) -> Self {
        Self { b0: b0 / a0, b1: b1 / a0, b2: b2 / a0, a1: a1 / a0, a2: a2 / a0 }
    }

    /// 复数频率响应（用于参考无关的频响判据；只依赖系数定义式）。
    pub fn response(&self, fs: f32, f: f32) -> (f32, f32) {
        let w = 2.0 * core::f32::consts::PI * f / fs;
        let (s1, c1) = (math::sin(w), math::cos(w));
        let (s2, c2) = (math::sin(2.0 * w), math::cos(2.0 * w));
        let (nr, ni) = (
            self.b0 + self.b1 * c1 + self.b2 * c2,
            -(self.b1 * s1 + self.b2 * s2),
        );
        let (dr, di) = (
            1.0 + self.a1 * c1 + self.a2 * c2,
            -(self.a1 * s1 + self.a2 * s2),
        );
        let d2 = dr * dr + di * di;
        ((nr * dr + ni * di) / d2, (ni * dr - nr * di) / d2)
    }
}

/// DF2T 双二次滤波器实例（每轴一个）。
#[derive(Debug, Clone, Copy, Default)]
pub struct Biquad {
    pub c: Coeffs,
    z1: f32,
    z2: f32,
}

impl Biquad {
    pub fn new(c: Coeffs) -> Self {
        Self { c, z1: 0.0, z2: 0.0 }
    }

    pub fn reset(&mut self) {
        self.z1 = 0.0;
        self.z2 = 0.0;
    }

    #[inline]
    pub fn step(&mut self, x: f32) -> f32 {
        let y = self.c.b0 * x + self.z1;
        self.z1 = self.c.b1 * x - self.c.a1 * y + self.z2;
        self.z2 = self.c.b2 * x - self.c.a2 * y;
        y
    }

    pub fn is_finite(&self) -> bool {
        self.z1.is_finite() && self.z2.is_finite()
    }
}

impl Default for Coeffs {
    fn default() -> Self {
        Self { b0: 1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0 }
    }
}
