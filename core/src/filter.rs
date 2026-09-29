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

    /// 该滤波器在 `freq_hz` 处的**群延迟**（秒）——用于自动标定“滤波器引入的等价时延”。
    ///
    /// 定义：`τ(ω) = −dφ/dω`（离散角频率 ω = 2π f/fs，rad/sample）⇒ 秒：`τ_s = τ/fs` ✓。
    /// 用途（§5.187）：比力低通引入的相位滞后补偿量（`update_gravity` 的姿态回退 τ）——
    ///   由**实际系数**导出 ⇒ 一旦低通/陷波参数改变，τ 自动跟随 ✓（不再硬编码 ✗）。
    /// 实现：对相位`φ(ω)=arg H(e^{jω})`做**中心差分**（含 ±2π 分支归一），对任意双二阶通用 ✓。
    pub fn group_delay_s(&self, freq_hz: f32, fs: f32) -> f32 {
        let w = 2.0 * PI * freq_hz / fs;
        let dw = 1e-4f32;
        let phase = |w: f32| -> f32 {
            let (c1, s1) = (math::cos(w), math::sin(w));
            let (c2, s2) = (math::cos(2.0 * w), math::sin(2.0 * w));
            // H(z) = (b0 + b1 z⁻¹ + b2 z⁻²)/(1 + a1 z⁻¹ + a2 z⁻²)，z = e^{jω}
            let nr = self.b0 + self.b1 * c1 + self.b2 * c2;
            let ni = -(self.b1 * s1 + self.b2 * s2);
            let dr = 1.0 + self.a1 * c1 + self.a2 * c2;
            let di = -(self.a1 * s1 + self.a2 * s2);
            math::atan2(ni, nr) - math::atan2(di, dr)
        };
        let mut p1 = phase(w - dw);
        let mut p2 = phase(w + dw);
        while p2 - p1 > PI {
            p2 -= 2.0 * PI;
        }
        while p1 - p2 > PI {
            p1 -= 2.0 * PI;
        }
        let dphi_dw = (p2 - p1) / (2.0 * dw);
        -dphi_dw / fs
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

    /// ★§5.187：群延迟估计（比力低通延迟 τ 自动标定的基础 ✓）。
    ///
    /// 判据：20Hz Butterworth 低通在飞行频段（≈DC~5Hz）的群延迟应≈解析值
    /// `1/(2π f0 q)` = 1/(2π·20·0.7071) ≈ **11.26ms**（容差 1.5ms）。
    /// 同时验证：40Hz 陷波在低频贡献≈0（不污染低通主导的 τ ✓）。
    #[test]
    fn group_delay_matches_analytic_lowpass() {
        let lp = Biquad::low_pass(20.0, FS, 0.7071);
        let analytic = 1.0 / (2.0 * PI * 20.0 * 0.7071);
        eprintln!("DIAG analytic τ = {:.3}ms", analytic * 1e3);
        for f in [0.5f32, 1.0, 2.0, 4.0] {
            let t = lp.group_delay_s(f, FS);
            eprintln!("DIAG lowpass τ({f}Hz) = {:.3}ms", t * 1e3);
            assert!(
                (t - analytic).abs() < 1.5e-3,
                "20Hz 低通 τ({f}Hz)={:.3}ms 应≈解析 {:.3}ms",
                t * 1e3,
                analytic * 1e3
            );
        }
        // 陷波在低频的群延迟应较小（<2.5ms）——但**非零**（~1.8ms ⇒ 必须计入总 τ ✓）
        let nf = Biquad::notch(40.0, FS, 2.0);
        for f in [0.5f32, 1.0, 4.0] {
            let t = nf.group_delay_s(f, FS).abs();
            eprintln!("DIAG notch τ({f}Hz) = {:.3}ms", t * 1e3);
            assert!(t < 2.5e-3, "40Hz 陷波低频群延迟应 <2.5ms，实测 {:.3}ms", t * 1e3);
        }
        // 恒等滤波器群延迟 = 0
        assert_eq!(Biquad::passthrough().group_delay_s(2.0, FS), 0.0);
    }
}
