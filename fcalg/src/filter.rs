//! L15 · 滤波器编排（把 L2–L14 接成一条**可跑的估计器**）
//! # 这一层的职责只有一个
//! **顺序与提交语义**：predict → 各路 update → 成功则 `boxplus` 提交，失败则交门控/重灌。
//! 算法本体的每一条约定都已在前面的层里各自验证过，这里**不重复实现任何约定**。
//! # 提交语义（显式）
//! - `update` 成功 ⇒ 先提交 P，再 `boxplus` 施加 dx，最后 `guards.accepted()` 复位本通道；
//! - `update` 失败（含被门限拒收）⇒ **不动 P/状态**，`guards.rejected()` 计数；
//!   若达阈值则**重灌**该通道可观测的方差对角（L10）—— 使它能重新锚定；
//! - `boxplus` 若因退化失败 ⇒ 视为本拍不提交（不产生半个状态）。
//! # 与旧栈的对比
//! 旧栈把这些散在任务函数里（"记得在拒绝时不 apply"是**约定**）；
//! 这里"拒收不动状态"由**函数式 update + 显式提交点**保证（L8 设计）。
#[allow(unused_imports)]
use crate::math::F32Ext;

use crate::covariance::{propagate_covariance, Cov, CovError};
use crate::error_state::{boxplus, I_ATT, I_BA, I_BG, I_MAGB, I_MAGI, I_POS, I_VEL, N};
use crate::finite::{gate, Stage, Violation};
use crate::gate::{channel_indices, reflate_diag, Channel, ChannelGuard};
use crate::imu_delta::ImuDelta;
use crate::observe::Obs;
use crate::propagate::{propagate, State};
use crate::transition::transition_matrix;
use crate::update::{update, UpdateError};
/// **过程噪声**（Q 的对角系数；按 dt 积分为方差）。
/// 出处：旧栈 `flyctrl core/src/estimator/eskf.rs::predict` 的 Q 构造
/// （`q[ATT]=qa·dt`、`q[VEL]=2·dt`、`q[POS]=1e-4·dt`、`q[BG]=1e-6·dt`、
///  `q[BA]=1e-4·dt`、`q[MAGI]=q[MAGB]=1e-3·dt`）—— **一手物证**。
/// ⚠caveat（写进出处而非藏起来）：那些值是在**旧 R** 下标定的 ⇒ 与本重建的 R 未必匹配，
///   属"继承来的起点"，接真传感器后须按 NIS 一致性重标。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ProcessNoise {
    pub q_att: f32,
    pub q_vel: f32,
    pub q_pos: f32,
    pub q_bg: f32,
    pub q_ba: f32,
    pub q_mag_i: f32,
    pub q_mag_b: f32,
}

impl Default for ProcessNoise {
    fn default() -> Self {
        Self {
            q_att: 1e-4,
            q_vel: 2.0,
            q_pos: 1e-4,
            q_bg: 1e-6,
            q_ba: 1e-4,
            q_mag_i: 1e-3,
            q_mag_b: 1e-3,
        }
    }
}

impl ProcessNoise {
    /// 按 dt 展开成 Q 矩阵（对角）。
    /// ★**Q=0 是个陷阱**（本会话实测踩到）：没有过程噪声 ⇒ 协方差被观测反复收缩到退化
    ///   ⇒ `propagate_covariance` 判非正定（**正确行为**）⇒ 表现为"某天 predict 突然 Err"。
    pub fn matrix(&self, dt: f32) -> Cov {
        let mut q = [[0.0f32; N]; N];
        for i in 0..3 {
            q[I_ATT + i][I_ATT + i] = self.q_att * dt;
            q[I_VEL + i][I_VEL + i] = self.q_vel * dt;
            q[I_POS + i][I_POS + i] = self.q_pos * dt;
            q[I_BG + i][I_BG + i] = self.q_bg * dt;
            q[I_BA + i][I_BA + i] = self.q_ba * dt;
            q[I_MAGI + i][I_MAGI + i] = self.q_mag_i * dt;
            q[I_MAGB + i][I_MAGB + i] = self.q_mag_b * dt;
        }
        q
    }
    /// 第 i 个状态的过程噪声系数（供"P 不得塌陷"判据导出下界）。
    pub fn coeff(&self, i: usize) -> f32 {
        if i < I_VEL {
            self.q_att
        } else if i < I_POS {
            self.q_vel
        } else if i < I_BG {
            self.q_pos
        } else if i < I_BA {
            self.q_bg
        } else if i < I_MAGI {
            self.q_ba
        } else if i < I_MAGB {
            self.q_mag_i
        } else {
            self.q_mag_b
        }
    }
}

/// 编排层错误。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FilterError {
    Prop(CovError),
    State(Violation),
    Update(UpdateError),
}
/// 通道在守卫数组中的下标。
fn ch_idx(ch: Channel) -> usize {
    match ch {
        Channel::Baro => 0,
        Channel::GpsPos => 1,
        Channel::GpsVel => 2,
        Channel::MagYaw => 3,
    }
}
/// 最小 ESKF 编排器。
pub struct Eskf {
    pub st: State,
    pub p: Cov,
    /// 过程噪声（**必须非零** —— 见 `ProcessNoise::matrix` 的注释）。
    pub q: ProcessNoise,
    /// 重力通道的独立守卫（与四路外部观测分开 —— 它是 IMU 驱动的，不共用 `guards`）。
    pub gravity_guard: ChannelGuard,
    /// 重力观测的单轴 σ（m/s²）。
    pub sigma_gravity: f32,
    /// 量级门容差（相对；与静止对齐共用 `align.g_tol_frac`）。
    pub g_tol_frac: f32,
    pub guards: [ChannelGuard; 4],
}
impl Eskf {
    pub fn new(st: State, p: Cov, max_consecutive_rejects: u32) -> Self {
        Self {
            st,
            p,
            q: ProcessNoise::default(),
            guards: [ChannelGuard::new(max_consecutive_rejects); 4],
            gravity_guard: ChannelGuard::new(max_consecutive_rejects),
            sigma_gravity: 0.3,
            g_tol_frac: 0.06,
        }
    }
    /// 预测：先推协方差（用解析 F），再推标称态。任一失败 ⇒ `Err`（调用方不得提交）。
    pub fn predict(&mut self, d: &ImuDelta, g_ned: [f32; 3]) -> Result<(), FilterError> {
        // F 需要与 propagate 完全相同的 ω 与 f_b（此处只做取值，不重复约定推导）
        let w = [
            d.delta_ang[0] / d.dt_ang - self.st.bg[0],
            d.delta_ang[1] / d.dt_ang - self.st.bg[1],
            d.delta_ang[2] / d.dt_ang - self.st.bg[2],
        ];
        let f_b = [
            d.delta_vel[0] / d.dt_vel - self.st.ba[0],
            d.delta_vel[1] / d.dt_vel - self.st.ba[1],
            d.delta_vel[2] / d.dt_vel - self.st.ba[2],
        ];
        if !crate::covariance::is_positive_definite(&self.p) {
            crate::covariance::PD_ENTRY_VIOLATIONS
                .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }
        let f = transition_matrix(self.st.q, w, f_b, d.dt_vel);
        let q = self.q.matrix(d.dt_vel);
        let p_new = propagate_covariance(&self.p, &f, &q).map_err(FilterError::Prop)?;
        self.p = p_new;
        propagate(&mut self.st, d, g_ned).map_err(FilterError::State)
    }
    /// 融合一路观测。失败时**不动状态与 P**（但会推进门控/必要时重灌）。
    pub fn fuse(
        &mut self,
        o: &Obs,
        ch: Channel,
        gate_sigma: f32,
        reflate_floor: f32,
    ) -> Result<(), FilterError> {
        match update(&self.p, &o.h, &o.resid, &o.r, gate_sigma) {
            Ok(out) => {
                let st_new = boxplus(&self.st, &out.dx).ok_or(FilterError::State(Violation::Nan))?;
                self.p = out.p;
                self.st = st_new;
                self.guards[ch_idx(ch)].accepted();
                Ok(())
            }
            Err(e) => {
                if self.guards[ch_idx(ch)].rejected() {
                    // 达阈值 ⇒ 重灌该通道可观测的方差（L10）。重灌失败（非有限）也**不掩盖**：
                    // 直接放弃本拍，让调用方看到 Err。
                    let _ = reflate_diag(&mut self.p, channel_indices(ch), reflate_floor);
                }
                Err(FilterError::Update(e))
            }
        }
    }
    /// **重力（倾角）观测**：旧栈 `update_gravity` 的对应物。
    ///
    /// 调用时机：紧接 `predict` 之后（IMU 驱动，与外部观测不同类）。
    /// - 量级门不过 ⇒ **不融合**，`gravity_guard` 计数，返回 `Ok(false)`（**显式**，非静默）；
    /// - 门过但融合失败 ⇒ `Err`（交给上层的门控/重灌）。
    pub fn update_gravity(&mut self, f_b_meas: [f32; 3]) -> Result<bool, FilterError> {
        if !crate::observe::gravity_magnitude_ok(f_b_meas, self.g_tol_frac) {
            // 只计数、不重灌（"正在机动"不是通道故障，不该抬方差）
            let _ = self.gravity_guard.rejected();
            return Ok(false);
        }
        let o = crate::observe::gravity(f_b_meas, &self.st, self.sigma_gravity);
        match update(&self.p, &o.h, &o.resid, &o.r, 1e9) {
            Ok(out) => {
                let st_new = boxplus(&self.st, &out.dx).ok_or(FilterError::State(Violation::Nan))?;
                self.p = out.p;
                self.st = st_new;
                self.gravity_guard.accepted();
                Ok(true)
            }
            Err(e) => Err(FilterError::Update(e)),
        }
    }

    /// 方便的自检：状态与协方差是否仍然有限（集成验收用）。
    pub fn healthy(&self) -> bool {
        let qf = self.st.q.is_finite();
        let vf = self.st.v.iter().all(|x| x.is_finite());
        let pf = self.st.p.iter().all(|x| x.is_finite());
        let cf = self.p.iter().all(|r| r.iter().all(|x| x.is_finite()));
        let _ = (I_ATT, I_POS, I_VEL);
        qf && vf && pf && cf && gate(Stage::L5Propagate, 1.0).is_ok()
    }
}
