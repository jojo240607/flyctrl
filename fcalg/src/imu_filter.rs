//! L2 · IMU 预处理（原始 → 供控制/估计使用的两路）
//!
//! # 模块契约
//!
//! **输入**：原始陀螺 `gyro`（rad/s，机体系 x=前/y=右/z=下）与原始比力 `accel`（m/s²，机体系），
//! 以及采样率 `fs`（Hz，构造时给定）。
//!
//! **输出**：
//! - `gyro` = **仅陷波**（抑制机架/旋翼振动带）——
//!   刻意**不做低通**：角速度是速率环的阻尼反馈，低通会引入相位滞后，反而降低阻尼裕度；
//! - `accel` = **陷波 + 低通**（抑制振动与白噪声），供姿态重力锚定与估计使用。
//!
//! 两路都同时供控制器与估计器取用（契约允许二者相同，**不允许隐式分叉**；
//! 若将来要"估计器用原始陀螺"之类的分化，必须改契约并在此加显式字段）。
//!
//! **前置/后置**：
//! - 每轴**独立**滤波器实例（跨轴复用会把三轴样本串进同一状态）；
//! - 输入与输出都过 L1 有限性门（`Stage::L2ImuFilter`）；
//! - **非有限输入 ⇒ 返回 `Err` 且滤波器状态不被污染**（不得把 NaN 写进状态）。
//!
//! **预算量级**：3 轴 ×(1 陷波) + 3 轴 ×(1 陷波 + 1 低通) ≈ 9 个 biquad/样本。

#[allow(unused_imports)]
use crate::math::F32Ext;

use crate::biquad::{Biquad, Coeffs};
use crate::finite::{gate_all, Stage, Violation};

/// 预处理输出（机体系，单位同输入）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuFiltered {
    /// 陷波后陀螺（rad/s）—— 控制器速率环与估计器共用。
    pub gyro: [f32; 3],
    /// 陷波 + 低通后比力（m/s²）—— 姿态重力锚定/估计使用。
    pub accel: [f32; 3],
}

/// IMU 预处理链（6 个轴，各自独立实例）。
pub struct ImuFilter {
    notch_g: [Biquad; 3],
    notch_a: [Biquad; 3],
    lp_a: [Biquad; 3],
}

impl ImuFilter {
    /// 构造。
    ///
    /// - `notch_f0`：陷波中心（机架/旋翼振动带，按实测选）；
    /// - `accel_lp_fc`：比力低通截止（抑制白噪声）；
    /// - `q`：陷波 Q；`lp_q = 0.7071` 则阶跃无过冲。
    pub fn new(fs: f32, notch_f0: f32, notch_q: f32, accel_lp_fc: f32, lp_q: f32) -> Self {
        let g = Coeffs::notch(fs, notch_f0, notch_q);
        let lp = Coeffs::lowpass2(fs, accel_lp_fc, lp_q);
        Self {
            notch_g: [Biquad::new(g); 3],
            notch_a: [Biquad::new(g); 3],
            lp_a: [Biquad::new(lp); 3],
        }
    }

    pub fn reset(&mut self) {
        for b in self.notch_g.iter_mut().chain(self.notch_a.iter_mut()).chain(self.lp_a.iter_mut()) {
            b.reset();
        }
    }

    /// 一拍。**先把关输入再进滤波器** —— 保证非有限值不会污染状态。
    pub fn step(&mut self, gyro: [f32; 3], accel: [f32; 3]) -> Result<ImuFiltered, Violation> {
        gate_all(Stage::L2ImuFilter, &gyro)?;
        gate_all(Stage::L2ImuFilter, &accel)?;

        let mut out = ImuFiltered { gyro: [0.0; 3], accel: [0.0; 3] };
        for k in 0..3 {
            out.gyro[k] = self.notch_g[k].step(gyro[k]);
            let a = self.notch_a[k].step(accel[k]);
            out.accel[k] = self.lp_a[k].step(a);
        }

        // 后置：输出也必须有限（系数非法/状态退化会在此暴露，而不是悄悄流向下游）
        gate_all(Stage::L2ImuFilter, &out.gyro)?;
        gate_all(Stage::L2ImuFilter, &out.accel)?;
        Ok(out)
    }

    /// 内部状态是否有限（诊断/集成验收用）。
    pub fn states_finite(&self) -> bool {
        self.notch_g.iter().all(|b| b.is_finite())
            && self.notch_a.iter().all(|b| b.is_finite())
            && self.lp_a.iter().all(|b| b.is_finite())
    }
}
