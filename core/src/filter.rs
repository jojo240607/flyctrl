//! no_std 双二阶（Biquad）滤波器。
//!
//! 输入滤波方案（第 1 步）：在共享单步 [`crate::hil::HilContext::step_hil`] 的 IMU
//! 数据入口，用**陷波**抑制机体固定频率振动（如旋翼/机架 40Hz），用**低通**衰减
//! 传感器宽带白噪声。二者直流增益均为 1（慢变信号/重力比力不丢失），低频相位≈0。
//! SIL 与 MCU 复用同一份实现与状态演化 → 两侧滤波行为逐位一致（同输入测试仍成立）。
//!
//! 系数按 RBJ Audio EQ Cookbook 计算并**预先归一化**（除以 a0），每样本 5 次乘加
//! （直接 I 型），可在 Cortex-M4 上实时执行。
//!
//! # 稳定性
//! RBJ 系数保证极点在单位圆内（`|a1|<2`、`|a2|<1`），对有界输入输出有界。

use core::f32::consts::PI;

use crate::math;

/// 直接 I 型双二阶滤波器（归一化系数）。
///
/// 差分方程（直接 I 型）：
/// ```text
/// y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Biquad {
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Biquad {
    /// 恒等（无滤波）实例：用于测试/对比基线。
    pub fn passthrough() -> Self {
        Self {
            b0: 1.0, b1: 0.0, b2: 0.0,
            a1: 0.0, a2: 0.0,
            x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0,
        }
    }

    /// 陷波滤波器：在 `f0` 处形成窄带凹陷，抑制固定频率振动。
    ///
    /// - `f0`：中心频率（Hz），如机体振动 40Hz；
    /// - `fs`：采样率（Hz），= 1/控制周期（`HilContext` 用 `1/dt`）；
    /// - `q`：品质因数，越大凹陷越窄越尖锐（带宽 = f0/q）。
    ///
    /// 直流与远离 `f0` 的频率增益≈1 → 对慢变比力/角速度几乎无失真。
    pub fn notch(f0: f32, fs: f32, q: f32) -> Self {
        let w0 = 2.0 * PI * f0 / fs;
        let cw = math::cos(w0);
        let alpha = math::sin(w0) / (2.0 * q);
        let a0 = 1.0 + alpha;
        // RBJ notch：b0=b2=1、b1=-2cos；a1=-2cos、a2=1-alpha。
        Self {
            b0: 1.0 / a0,
            b1: (-2.0 * cw) / a0,
            b2: 1.0 / a0,
            a1: (-2.0 * cw) / a0,
            a2: (1.0 - alpha) / a0,
            x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0,
        }
    }

    /// 低通滤波器：衰减高于 `f0` 的频率，抑制传感器宽带白噪声与高频振动。
    ///
    /// - `f0`：截止频率（Hz）；`q=1/√2` 即 Butterworth 最平坦。
    ///
    /// 直流增益 1；`f0` 远高于控制带宽（如 10Hz 姿态环）时附加相位滞后可忽略。
    pub fn low_pass(f0: f32, fs: f32, q: f32) -> Self {
        let w0 = 2.0 * PI * f0 / fs;
        let cw = math::cos(w0);
        let alpha = math::sin(w0) / (2.0 * q);
        let a0 = 1.0 + alpha;
        let c1 = 1.0 - cw;
        // RBJ low-pass：b0=b2=(1-cos)/2、b1=1-cos；a1=-2cos、a2=1-alpha。
        Self {
            b0: (c1 / 2.0) / a0,
            b1: c1 / a0,
            b2: (c1 / 2.0) / a0,
            a1: (-2.0 * cw) / a0,
            a2: (1.0 - alpha) / a0,
            x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0,
        }
    }

    /// 处理一个样本，返回滤波后输出。
    pub fn process(&mut self, x: f32) -> f32 {
        let y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1 - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::eprintln;

    use super::*;

    const FS: f32 = 250.0; // 与 HilContext dt=0.004 对应

    fn amp_after(bf: &mut Biquad, freq: f32, settle: usize, measure: usize) -> f32 {
        // 喂入正弦，settle 拍后测量稳态输出振幅（峰值-谷值/2）。
        let mut out = [0.0f32; 3];
        for n in 0..(settle + measure) {
            let x = (2.0 * PI * freq * n as f32 / FS).sin();
            let y = bf.process(x);
            if n >= settle {
                let i = n - settle;
                out[0] = out[0].min(y);
                out[1] = out[1].max(y);
                out[2] += y;
            }
        }
        let mean = out[2] / measure as f32;
        let peak = out[1] - mean;
        let valley = mean - out[0];
        peak.max(valley)
    }

    #[test]
    fn notch_rejects_center_freq_keeps_dc() {
        // 陷波 @40Hz：中心 40Hz 正弦应被大幅抑制（<10%），低频（0.5Hz，远离凹陷带）
        // 应近乎全通。低频测量须覆盖≥数个整周期才能测到稳态振幅（0.5Hz 每周期 500 拍）。
        let mut nf = Biquad::notch(40.0, FS, 5.0);
        let amp_40 = amp_after(&mut nf, 40.0, 200, 200);
        eprintln!("DIAG notch amp@40Hz = {:.4}（输入振幅 1.0）", amp_40);
        assert!(amp_40 < 0.1, "陷波中心 40Hz 振幅应 <0.1，实测 {amp_40:.4}");

        let mut nf = Biquad::notch(40.0, FS, 5.0);
        let dc = amp_after(&mut nf, 0.5, 1500, 2000); // 稳态后 4 个整周期
        eprintln!("DIAG notch amp@0.5Hz = {:.4}", dc);
        assert!(dc > 0.9, "陷波对低频应近乎全通，实测 {dc:.4}");
    }

    #[test]
    fn lowpass_passes_dc_attenuates_high_freq() {
        // 低通 fc=30Hz：低频（0.5Hz，通带内）≈ 全通，100Hz 应显著衰减。
        let mut lp = Biquad::low_pass(30.0, FS, 0.7071);
        let dc = amp_after(&mut lp, 0.5, 1500, 2000);
        eprintln!("DIAG lp amp@0.5Hz = {:.4}", dc);
        assert!(dc > 0.9, "低通对低频应近乎全通，实测 {dc:.4}");

        let mut lp = Biquad::low_pass(30.0, FS, 0.7071);
        let hi = amp_after(&mut lp, 100.0, 200, 200);
        eprintln!("DIAG lp amp@100Hz = {:.4}", hi);
        assert!(hi < 0.35, "低通 100Hz 应显著衰减，实测 {hi:.4}");
    }

    #[test]
    fn constant_input_stays_constant() {
        // 常值输入（比力/零角速度）经滤波仍为同常值：不漂移、不引入偏置。
        // 前 200 拍为 IIR 瞬态收敛期（从零初值建立稳态），之后测稳态偏差。
        let mut nf = Biquad::notch(40.0, FS, 5.0);
        let mut lp = Biquad::low_pass(30.0, FS, 0.7071);
        for _ in 0..200 {
            let _ = lp.process(nf.process(-9.81));
        }
        let mut max_dev = 0.0f32;
        for _ in 0..500 {
            let y = lp.process(nf.process(-9.81));
            max_dev = max_dev.max((y - (-9.81)).abs());
        }
        eprintln!("DIAG constant drift = {:.6} m/s²", max_dev);
        assert!(max_dev < 1e-3, "常值输入滤波后应保持常值，最大偏差 {max_dev:.6}");
    }

    #[test]
    fn noise_input_stays_bounded() {
        // 噪声输入（±1 范围内随机）→ 输出恒有界（稳定性自检）。
        let mut nf = Biquad::notch(40.0, FS, 5.0);
        let mut lp = Biquad::low_pass(30.0, FS, 0.7071);
        let mut y = 0.0f32;
        let mut max_abs = 0.0f32;
        for n in 0..2000 {
            // 确定性伪随机（避免测试依赖 std rng）
            let x = ((n as f32) * 12.9898).sin() * 43758.5453;
            let x = x - x.floor() - 0.5; // [-0.5, 0.5)
            y = lp.process(nf.process(x * 2.0));
            max_abs = max_abs.max(y.abs());
            assert!(y.is_finite(), "NaN at n={n}");
        }
        eprintln!("DIAG noise max|y| = {:.3}", max_abs);
        assert!(max_abs <= 2.0, "滤波输出应有界，max|y|={max_abs:.3}");
    }
}
