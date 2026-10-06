//! L11a · 姿态环（外环）：姿态误差 → 机体角速率设定值
//! # 约定（与 L0/L5/L6 的**右乘**误差态一致）
//! 设当前姿态 `q`、设定姿态 `q_sp`（均为机体→世界）。则把 `q` 转到 `q_sp` 所需的
//! **机体系**增量旋转为 `exp(δθ) = q⁻¹ ⊗ q_sp` ⇒ `δθ = log(q⁻¹ ⊗ q_sp)`，
//! 速率设定值 `ω_sp = kp · δθ`。
//! # 为什么必须"取短弧"
//! `log` 必须取短弧（`w<0` 先整体取反）—— 否则在 180° 附近会出现 **±π 跳变**，
//! 表现为姿态指令瞬间反向（旧栈实测过 "roll 翻 π" 这类现象）。
//! 复用 `error_state::log_rot`，保证与滤波器侧**同一实现**（约定不可能分叉）。
#[allow(unused_imports)]
use crate::math::F32Ext;

use crate::error_state::log_rot;
use crate::quat::Quat;
/// 姿态误差 → 机体角速率设定值（rad/s）。
pub fn attitude_rate_setpoint(q: Quat, q_sp: Quat, kp: f32) -> [f32; 3] {
    let dq = match q.conj().mul(q_sp).normalize() {
        Some(v) => v,
        None => return [0.0; 3], // 退化 ⇒ 零设定值（不产出垃圾）
    };
    let d = log_rot(dq);
    [kp * d[0], kp * d[1], kp * d[2]]
}
