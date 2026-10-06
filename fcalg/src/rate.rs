//! L11b · 速率环（内环）：比例项 + 积分项（增量式抗饱和）
//! # 约定
//! `u = kp·(sp − meas) + i`，`i` 为积分状态；积分**只在未饱和时**累积（抗饱和），
//! 且由调用方显式传入"是否饱和"—— 本模块**不猜**。
//! # 为何只给 P/I 而把 D 留给调用方
//! 微分通常作用在**测量值**上（避免设定值阶跃产生微分冲击）。把 D 的取法留给调用方
//! 是显式决定，而不是在这里悄悄选一种。
#[allow(unused_imports)]
use crate::math::F32Ext;

use crate::finite::{gate, Stage, Violation};
/// 速率环增益。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RateGains {
    pub kp: f32,
    pub ki: f32,
}
/// 速率环状态（积分器，每轴一个）。
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RateIntegrator {
    pub i: f32,
}
/// P 步（无积分）：`u = kp·(sp − meas)`。失败 ⇒ `Err`（不产出垃圾）。
pub fn rate_p_step(g: &RateGains, sp: f32, meas: f32) -> Result<f32, Violation> {
    let sp = gate(Stage::L11CtrlRate, sp)?;
    let meas = gate(Stage::L11CtrlRate, meas)?;
    let u = g.kp * (sp - meas);
    gate(Stage::L11CtrlRate, u)
}
impl RateIntegrator {
    /// 未饱和时累积积分；饱和时**冻结**（抗饱和）。返回更新后的积分量。
    pub fn step(&mut self, g: &RateGains, sp: f32, meas: f32, dt: f32, saturated: bool) -> Result<f32, Violation> {
        let sp = gate(Stage::L11CtrlRate, sp)?;
        let meas = gate(Stage::L11CtrlRate, meas)?;
        let dt = gate(Stage::L11CtrlRate, dt)?;
        let i = gate(Stage::L11CtrlRate, self.i)?;
        if !saturated && g.ki != 0.0 {
            self.i = i + g.ki * (sp - meas) * dt;
        }
        gate(Stage::L11CtrlRate, self.i)
    }
}
