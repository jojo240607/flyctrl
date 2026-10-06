//! L5 · 标称状态传播（21 态误差状态滤波器的标称部分）
//!
//! # 状态划分（误差状态维数 N = 21）
//! | 索引 | 分量 | 帧 | 说明 |
//! |---|---|---|---|
//! | 0..3   | 姿态误差 | 机体系 | 标称姿态用四元数 `q` 承载 |
//! | 3..6   | 速度     | NED | |
//! | 6..9   | 位置     | NED | |
//! | 9..12  | 陀螺零偏 | 机体系 | 常量 |
//! | 12..15 | 加计零偏 | 机体系 | 常量 |
//! | 15..18 | 磁惯性系 | NED | 常量（在线标定的地球磁场） |
//! | 18..21 | 磁机体系 | 机体系 | 常量（在线标定的机体磁偏置） |
//!
//! # 传播约定（显式，不靠注释猜）
//! - **姿态**：`q ← q ⊗ exp((Δang/dt_ang − bg)·dt_ang)` —— **右乘**（机体系角增量）；
//! - **速度**：`v ← v + (R(q_old)·f_b + g_ned)·dt_vel`，`f_b = Δvel/dt_vel − ba`
//!   （姿态→速度耦合用**更新前**姿态：标称传播对该耦合是一阶的，这是 ESKF 固有性质，如实写明）；
//! - **位置**：`p ← p + ½(v_old + v_new)·dt_vel` —— **梯形**，对常加速度**精确**；
//! - **双 dt 各司其职**：姿态用 `dt_ang`，速度/位置用 `dt_vel`（契约 §3 要求双口径，这就是它的用处）；
//! - 零偏与磁两态是常量，传播中不变。
//!
//! # 纪律
//! 入口过 L1 有限性门；**任何非有限输入 ⇒ `Err` 且状态逐位不变**（不得留下半传播状态）。

#[allow(unused_imports)]
use crate::math::F32Ext;

use crate::finite::{gate, gate_all, Stage, Violation};
use crate::imu_delta::ImuDelta;
use crate::math;
use crate::quat::Quat;

/// 标称状态（21 态滤波器中的"名义值"部分）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub q: Quat,
    pub v: [f32; 3],
    pub p: [f32; 3],
    pub bg: [f32; 3],
    pub ba: [f32; 3],
    pub mag_i: [f32; 3],
    pub mag_b: [f32; 3],
}

impl State {
    /// 水平静止初值（姿态单位、速度位置为零、零偏与磁两态为零）。
    pub fn level() -> Self {
        Self {
            q: Quat::IDENTITY,
            v: [0.0; 3],
            p: [0.0; 3],
            bg: [0.0; 3],
            ba: [0.0; 3],
            mag_i: [0.0; 3],
            mag_b: [0.0; 3],
        }
    }
}

/// 旋转向量 ⇒ 四元数（`exp`）；小角走一阶分支（避免除零）。
#[inline]
fn exp_rot(theta: [f32; 3]) -> Quat {
    let a = math::sqrt(theta[0] * theta[0] + theta[1] * theta[1] + theta[2] * theta[2]);
    if a < 1e-8 {
        return Quat { w: 1.0, x: 0.5 * theta[0], y: 0.5 * theta[1], z: 0.5 * theta[2] };
    }
    let inv = 1.0 / a;
    Quat::from_axis_angle([theta[0] * inv, theta[1] * inv, theta[2] * inv], a)
}

/// 推进一拍。失败时**状态逐位不变**。
pub fn propagate(st: &mut State, d: &ImuDelta, g_ned: [f32; 3]) -> Result<(), Violation> {
    // ① 入口门（契约 §4）—— 全部检查通过后才动状态
    gate_all(Stage::L5Propagate, &d.delta_ang)?;
    gate_all(Stage::L5Propagate, &d.delta_vel)?;
    gate_all(Stage::L5Propagate, &g_ned)?;
    gate(Stage::L5Propagate, d.dt_ang)?;
    gate(Stage::L5Propagate, d.dt_vel)?;
    gate_all(Stage::L5Propagate, &[st.q.w, st.q.x, st.q.y, st.q.z])?;
    // ② 姿态：右乘（机体系）
    let mut th = [0.0f32; 3];
    for k in 0..3 {
        th[k] = d.delta_ang[k] - st.bg[k] * d.dt_ang;
    }
    let q_new = st.q.mul(exp_rot(th)).normalize().ok_or(Violation::Nan)?;
    // ③ 速度：比力（机体系）经更新前姿态转世界 + 重力
    let mut f_b = [0.0f32; 3];
    for k in 0..3 {
        f_b[k] = d.delta_vel[k] / d.dt_vel - st.ba[k];
    }
    let f_w = st.q.rotate(f_b);
    let v_old = st.v;
    let mut v_new = [0.0f32; 3];
    for k in 0..3 {
        v_new[k] = v_old[k] + (f_w[k] + g_ned[k]) * d.dt_vel;
    }
    // ④ 位置：梯形（常加速度下精确）
    let mut p_new = [0.0f32; 3];
    for k in 0..3 {
        p_new[k] = st.p[k] + 0.5 * (v_old[k] + v_new[k]) * d.dt_vel;
    }
    // ⑤ 提交（以上已无失败点）
    st.q = q_new;
    st.v = v_new;
    st.p = p_new;
    Ok(())
}
