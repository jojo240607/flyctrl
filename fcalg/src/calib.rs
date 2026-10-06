//! L13 · 传感器标定（陀螺 / 加计的**尺度-安装矩阵 + 零偏**）
//! # 职责边界（明确划清，避免两处都做同一件事）
//! - **本模块只管**：`raw = M·true + b`（`M` = 尺度+安装误差，`b` = 零偏，均机体系）
//!   的**反解** `true = M⁻¹·(raw − b)`。
//! - **磁不在这里**：磁的零偏/尺度由 L5 的 `mag_i`/`mag_b` **在线标定状态**承担
//!   （L9b 的观测模型已含 `mag_b`）。在此再做一次就是两处重复、约定必然分叉。
//! - **接口约定**：输入是**原始量**，输出是**校准后的 SI 量**（rad/s、m/s²）。
//! # 为什么在构造时就求逆
//! `M` 奇异 ⇒ 反解会**放大**而不是校正 ⇒ 必须在构造时**显式拒绝**，
//! 而不是等到运行时把噪声放大成"貌似合理的姿态"。
use crate::finite::{gate_all, Stage, Violation};
use crate::update::inv3;
/// 标定参数（机体系）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorCalib {
    m: [[f32; 3]; 3],
    m_inv: [[f32; 3]; 3],
    b: [f32; 3],
}
/// 显式失败原因。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibError {
    NonFinite(Violation),
    /// `M` 奇异/近奇异 ⇒ 反解会放大误差，拒绝。
    Singular,
}
impl SensorCalib {
    /// 由前向参数构造（`raw = M·true + b`）。构造时求逆并校验。
    pub fn new(m: [[f32; 3]; 3], b: [f32; 3]) -> Result<Self, CalibError> {
        for r in m.iter() {
            gate_all(Stage::L13Calib, r).map_err(CalibError::NonFinite)?;
        }
        gate_all(Stage::L13Calib, &b).map_err(CalibError::NonFinite)?;
        let m_inv = inv3(&m).ok_or(CalibError::Singular)?;
        for r in m_inv.iter() {
            gate_all(Stage::L13Calib, r).map_err(CalibError::NonFinite)?;
        }
        Ok(Self { m, m_inv, b })
    }
    /// 恒等标定（不校正）。
    pub fn identity() -> Self {
        Self {
            m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            m_inv: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            b: [0.0; 3],
        }
    }
    /// 反解：`true = M⁻¹·(raw − b)`。
    pub fn apply(&self, raw: [f32; 3]) -> Result<[f32; 3], CalibError> {
        gate_all(Stage::L13Calib, &raw).map_err(CalibError::NonFinite)?;
        let d = [raw[0] - self.b[0], raw[1] - self.b[1], raw[2] - self.b[2]];
        let mut out = [0.0f32; 3];
        for i in 0..3 {
            out[i] = self.m_inv[i][0] * d[0] + self.m_inv[i][1] * d[1] + self.m_inv[i][2] * d[2];
        }
        gate_all(Stage::L13Calib, &out).map_err(CalibError::NonFinite)?;
        Ok(out)
    }
    /// 正向生成：`raw = M·true + b`（**供真值注入/测试**用；严格逆见 `apply`）。
    pub fn raw_from_true(&self, truth: [f32; 3]) -> [f32; 3] {
        let mut out = [0.0f32; 3];
        for i in 0..3 {
            out[i] = self.m[i][0] * truth[0] + self.m[i][1] * truth[1] + self.m[i][2] * truth[2];
            out[i] += self.b[i];
        }
        out
    }
    /// 各轴**尺度**（`M` 各列的模）—— 便于判据与可观测（1.0 = 无尺度误差）。
    pub fn scale(&self) -> [f32; 3] {
        let mut s = [0.0f32; 3];
        for j in 0..3 {
            s[j] = (self.m[0][j] * self.m[0][j] + self.m[1][j] * self.m[1][j] + self.m[2][j] * self.m[2][j]).sqrt();
        }
        s
    }
    /// 零偏（原始量单位）。
    pub fn bias(&self) -> [f32; 3] {
        self.b
    }
}
