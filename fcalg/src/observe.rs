//! L9 · 观测模型（baro / GPS 位置 / GPS 速度）
//! # 每路的输出统一为 `(H, ν, R)`，交给 L8 的通用更新
//! # 「无信息」的编码（**一条硬教训**）
//! 不参与观测的轴必须写成 **H 行全零 + R 对角大值**。
//! ⚠**不可**写成巨大的**非对角** R —— 那在物理上是"噪声完全相关"，不是"无观测"，
//!   而且会让 S 病态到 f32 下不可逆（L8 的用例初版正是这么写错的，被模块正确拒绝）。
//! # 各路的契约要点
//! - **baro**：模型 `alt == −p_z`（契约 §2：NED，z 向下）⇒ `H = −1 @ I_POS+2`；
//! - **GPS 位置**：`H = I @ I_POS`；
//! - **GPS 速度**：`H = I @ I_VEL`，但 **垂直分量无信息** ——
//!   RMC 只给水平 Doppler（speed+course），垂直分量恒为 0 是"没测"而非"测到 0"。
//!   旧栈吃过这个亏：把恒 0 的垂直速度当有效观测 ⇒ 真爬升时估计被死压到 0
//!   ⇒ 气压残差增长 ⇒ 门控死锁 ⇒ 失控爬升。故垂直 R 必须为「无信息」。
//! # 待定（**不硬塞**）
//! 磁航向（L9b）有一个必须先决定的语义分叉：**yaw-only 还是 3D 融合**。
//! 仓内证据倾向 yaw-only（3D 磁融合曾把姿态拉飞）。先把决定写进契约，再实现。
use crate::error_state::{I_POS, I_VEL, N};
use crate::propagate::State;
/// 「无信息」的对角 R（必须对角）。
pub const NO_INFO: f32 = 1e12;
/// 观测三元组。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Obs {
    pub h: [[f32; N]; 3],
    /// 新息 `ν = z − h(x)`。
    pub resid: [f32; 3],
    pub r: [[f32; 3]; 3],
}
/// 观测噪声参数（σ，单位与量测同）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ObsParams {
    pub sigma_baro: f32,
    pub sigma_gps_p: f32,
    pub sigma_gps_v: f32,
}
impl Default for ObsParams {
    fn default() -> Self {
        Self { sigma_baro: 0.3, sigma_gps_p: 0.5, sigma_gps_v: 0.1 }
    }
}
fn diag(a: f32, b: f32, c: f32) -> [[f32; 3]; 3] {
    [[a, 0.0, 0.0], [0.0, b, 0.0], [0.0, 0.0, c]]
}
/// 气压高度（标量观测，占据 0 号轴；另两轴无信息）。
pub fn baro(alt_meas: f32, st: &State, prm: &ObsParams) -> Obs {
    let mut h = [[0.0f32; N]; 3];
    h[0][I_POS + 2] = -1.0; // 模型：alt == −p_z
    let resid = [alt_meas - (-st.p[2]), 0.0, 0.0];
    let s = prm.sigma_baro * prm.sigma_baro;
    Obs { h, resid, r: diag(s, NO_INFO, NO_INFO) }
}
/// GPS 位置（NED，三轴全有效）。
pub fn gps_pos(meas: [f32; 3], st: &State, prm: &ObsParams) -> Obs {
    let mut h = [[0.0f32; N]; 3];
    for a in 0..3 {
        h[a][I_POS + a] = 1.0;
    }
    let resid = [meas[0] - st.p[0], meas[1] - st.p[1], meas[2] - st.p[2]];
    let s = prm.sigma_gps_p * prm.sigma_gps_p;
    Obs { h, resid, r: diag(s, s, s) }
}
/// GPS 速度（NED）：**水平有效、垂直无信息**（RMC 只给水平 Doppler）。
pub fn gps_vel(meas: [f32; 3], st: &State, prm: &ObsParams) -> Obs {
    let mut h = [[0.0f32; N]; 3];
    for a in 0..3 {
        h[a][I_VEL + a] = 1.0;
    }
    let resid = [meas[0] - st.v[0], meas[1] - st.v[1], meas[2] - st.v[2]];
    let s = prm.sigma_gps_v * prm.sigma_gps_v;
    Obs { h, resid, r: diag(s, s, NO_INFO) }
}
