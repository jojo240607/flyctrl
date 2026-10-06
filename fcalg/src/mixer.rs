//! L11c · 混控（X 型四旋翼）
//! # 约定（**符号表写死在这里**，不在别处重复）
//! 电机序：`0=前左(FL) 1=前右(FR) 2=后右(RR) 3=后左(RL)`。
//! - **推力**：四桨均分 `1/4`；
//! - **滚转**（右滚为正）：左侧升、右侧降 ⇒ `[+1, −1, −1, +1]`；
//! - **俯仰**（抬头为正）：后侧升、前侧降 ⇒ `[−1, −1, +1, +1]`；
//! - **偏航**（右偏为正）：**X 型是【对角线】同向旋转** —— `FL/RR` 一组、`FR/RL` 一组
//!   （**不是**"前两个一组"！）。设 `FL(0)`、`RR(2)` 为顺时针(CW)，右偏需增大 CW 桨的反扭矩
//!   ⇒ `[+1, −1, +1, −1]`。
//!   ★首版误写成 `[+1,+1,−1,−1]`（把"前/后"当成了旋转分组）⇒ 偏航行恰好是俯仰行的**负**
//!     ⇒ 俯仰与偏航根本不能独立指令 ✗。是"三行两两正交"这条**结构判据**把它拓出来的。
//! 三行**两两正交**且各含两个 +、两个 −；与全 1 的推力行合并 ⇒ 4×4 满秩 ⇒ 映射单射。
//! # 饱和处理（显式规则，非"夹一下了事"）
//! 记 `d[i] = 力矩贡献`、`thrust` 为请求推力均值：
//! 1. 先求不改 d 也能满足 `[0,1]` 的推力可行区间 `[t_lo, t_hi]`；
//! 2. 若可行 ⇒ 把 `thrust` **夹进该区间**（**力矩不被改动**，均值尽量保持）；
//! 3. 若不可行（力矩本身放不下）⇒ 把 d **等比缩小**到恰好放得下（`thrust` 取 0.5），
//!    **保证不产生反向力矩**（这是比"直接夹"重要的性质）。
use crate::error_state::N;
/// 三行符号表（滚转/俯仰/偏航）。
pub const X4SIGNS: [[f32; 4]; 3] = [
    [1.0, -1.0, -1.0, 1.0],
    [-1.0, -1.0, 1.0, 1.0],
    [1.0, -1.0, 1.0, -1.0],
];
/// 电机指令（归一化 0..1）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorCmd(pub [f32; 4]);
/// 显式失败原因。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MixError {
    NonFinite,
}
/// 混控：`thrust`（0..1）+ 力矩（归一化）→ 四电机指令。
pub fn x4_mix(thrust: f32, torque: [f32; 3]) -> Result<MotorCmd, MixError> {
    if !(thrust.is_finite()
        && torque[0].is_finite()
        && torque[1].is_finite()
        && torque[2].is_finite())
    {
        return Err(MixError::NonFinite);
    }
    let _ = N;
    let mut d = [0.0f32; 4];
    for m in 0..4 {
        for a in 0..3 {
            d[m] += X4SIGNS[a][m] * torque[a];
        }
    }
    let lo_d = d.iter().cloned().fold(f32::INFINITY, f32::min);
    let hi_d = d.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    let t_lo = (0.0f32).max(-lo_d);
    let t_hi = (1.0f32).min(1.0 - hi_d);
    let mut out = [0.0f32; 4];
    if t_lo <= t_hi {
        // 可行：力矩**原样保留**，只把推力夹进可行区间（均值尽量保持）
        let t = thrust.clamp(t_lo, t_hi);
        for m in 0..4 {
            out[m] = t + d[m];
        }
    } else {
        // 不可行：等比缩小 d 使跨度恰好放得下，保证**不反向**
        let span = hi_d - lo_d;
        let k = if span > 0.0 { 1.0 / span } else { 0.0 };
        let t = 0.5f32;
        for m in 0..4 {
            out[m] = (t + k * d[m]).clamp(0.0, 1.0);
        }
    }
    Ok(MotorCmd(out))
}
impl MotorCmd {
    /// 均值（= 实际总推力/4）。
    pub fn mean(&self) -> f32 {
        (self.0[0] + self.0[1] + self.0[2] + self.0[3]) * 0.25
    }
    /// 实际产生的各轴力矩（用同一符号表反算）。
    pub fn torque(&self) -> [f32; 3] {
        let mut t = [0.0f32; 3];
        for a in 0..3 {
            for m in 0..4 {
                t[a] += X4SIGNS[a][m] * self.0[m];
            }
            t[a] *= 0.25;
        }
        t
    }
}
