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
//! （磁航向已定，见 §磁航向。）
//!
//! # 磁航向（L9b）—— **决定**：yaw-only，且"yaw-only"的含义见下
//! 1. **只把磁场的【航向角】作为观测量**（不用倾角、不用幅值）。
//!    这是旧栈 yaw-only 的真实所指（它禁掉的是 3D 方向融合；3D 融合曾把姿态拉飞）。
//! 2. ⚠**这不等于"H 只含 yaw 分量"**：用机体系测得的场去算航向时，残差对 roll/pitch
//!    仍有**物理耦合**（转动姿态会改变机体看到的方向）。故 H 必须是**精确导数**，
//!    由有限差分裁判守住 —— 不允许为了"看起来更 yaw-only"而人为把 H 截断
//!    （那会让 H 与模型不一致，判据必然失败）。
//! 3. 若将来要**禁止** roll/pitch 被磁修正，那是**换模型**（把误差投影到世界竖直轴上），
//!    必须同时改模型与判据，不能只改 H。
#[allow(unused_imports)]
use crate::math::F32Ext;

use crate::error_state::{I_ATT, I_MAGB, I_MAGI, I_POS, I_VEL, N};
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
    /// 磁航向的门限 σ（出处见参数表 obs.sigma_mag）。
    pub sigma_mag: f32,
}
impl Default for ObsParams {
    fn default() -> Self {
        Self { sigma_baro: 0.3, sigma_gps_p: 0.5, sigma_gps_v: 0.1, sigma_mag: 2.0 }
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
/// `R(q)` 的行（与 L6 同一实现方式：对基向量取像）。
fn rot_matrix(q: crate::quat::Quat) -> [[f32; 3]; 3] {
    let c0 = q.rotate([1.0, 0.0, 0.0]);
    let c1 = q.rotate([0.0, 1.0, 0.0]);
    let c2 = q.rotate([0.0, 0.0, 1.0]);
    [[c0[0], c1[0], c2[0]], [c0[1], c1[1], c2[1]], [c0[2], c1[2], c2[2]]]
}
fn skew3(v: [f32; 3]) -> [[f32; 3]; 3] {
    [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
}
/// 磁航向（**yaw-only**，标量观测占据 0 号轴）。
/// # 模型（显式，含符号约定）
/// `m_b = meas − mag_b`（机体零偏已扣）、`m_w = R(q)·m_b`（转 NED），
/// `ν = wrap( atan2(m_w[1], m_w[0]) − atan2(mag_i[1], mag_i[0]) )`。
/// ⚠**本观测直接以 ν 定义**（不是 `z − h(x)` 形式）⇒ 更新所需的
/// `H ≡ −∂ν/∂δ`（使 `K·ν` 减小 ν）。故各块取号如下：
/// - `∂ν/∂δθ = g·(−R·[m_b×])` ⇒ `H_att = +g·R·[m_b×]`
/// - `∂ν/∂mag_b = g·(−R)`     ⇒ `H_magb = +g·R`
/// - `∂ν/∂mag_i = −g_i`       ⇒ `H_magi = +g_i`，`g_i = [−mi_e/hi², mi_n/hi²]`
/// 其中 `g = [−m_wy/h², m_wx/h²]` 是 `atan2` 的导数，`h² = m_wx²+m_wy²`。
/// 倾角/幅值不进观测（只用航向）—— 这是 yaw-only 的定义；roll/pitch 的物理耦合保留（见模块头 §磁航向）。
pub fn mag_yaw(mag_body_meas: [f32; 3], st: &State, sigma: f32) -> Obs {
    let m_b = [
        mag_body_meas[0] - st.mag_b[0],
        mag_body_meas[1] - st.mag_b[1],
        mag_body_meas[2] - st.mag_b[2],
    ];
    let m_w = st.q.rotate(m_b);
    let dz = m_w[1].atan2(m_w[0]) - st.mag_i[1].atan2(st.mag_i[0]);
    let nu = crate::math::sin(dz).atan2(crate::math::cos(dz));
    let h2 = m_w[0] * m_w[0] + m_w[1] * m_w[1];
    let hi2 = st.mag_i[0] * st.mag_i[0] + st.mag_i[1] * st.mag_i[1];
    let mut h = [[0.0f32; N]; 3];
    if h2 > 1e-12 && hi2 > 1e-12 {
        let g = [-m_w[1] / h2, m_w[0] / h2];
        let r = rot_matrix(st.q);
        let sm = skew3(m_b);
        for j in 0..3 {
            let mut s = 0.0f32;
            for a in 0..2 {
                for k in 0..3 {
                    s += g[a] * r[a][k] * sm[k][j];
                }
            }
            h[0][I_ATT + j] = s;
            h[0][I_MAGB + j] = g[0] * r[0][j] + g[1] * r[1][j];
        }
        // ∂ν/∂mag_i = −∂ψ/∂mag_i，ψ = atan2(mi_e, mi_n)
        //   ∂ψ/∂(mi_n,mi_e) = [−mi_e/hi², +mi_n/hi²]
        //   ⇒ H_magi = −∂ν/∂δ = −[−mi_e/hi², +mi_n/hi²] = [−mi_e/hi², **+mi_n/hi²**]
        // ★两列首版**均写反**（FD 实测：col15 −0.766 vs +0.769；col16 +3.843 vs −3.846；
        //   量级均差 <0.1%，全是**符号**）—— 逐列对齐而非继续猜。
        h[0][I_MAGI] = -st.mag_i[1] / hi2;
        h[0][I_MAGI + 1] = st.mag_i[0] / hi2;
    }
    let s = sigma * sigma;
    Obs { h, resid: [nu, 0.0, 0.0], r: diag(s, NO_INFO, NO_INFO) }
}
