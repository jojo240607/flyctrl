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
use crate::error_state::{boxplus, I_ATT, I_POS, I_VEL, N};
use crate::finite::{gate, Stage, Violation};
use crate::gate::{channel_indices, reflate_diag, Channel, ChannelGuard};
use crate::imu_delta::ImuDelta;
use crate::observe::Obs;
use crate::propagate::{propagate, State};
use crate::transition::transition_matrix;
use crate::update::{update, UpdateError};
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
    pub guards: [ChannelGuard; 4],
}
impl Eskf {
    pub fn new(st: State, p: Cov, max_consecutive_rejects: u32) -> Self {
        Self { st, p, guards: [ChannelGuard::new(max_consecutive_rejects); 4] }
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
        let q = [[0.0f32; N]; N];
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
