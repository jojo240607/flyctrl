//! L12 · 真值注入（合成输入生成器）—— 让整条链**在 host 上可端到端测**（契约 §0）
//! # 为什么需要它
//! 契约 §0 要求算法层验收**不经过仿真器**。做法就是：由**真值状态**生成"完美无偏"的
//! 传感器输入，再喂回算法。这样既是"可解析合成输入"（判据类型 4），
//! 又能产生一条极强的**跨模块一致性判据**：
//! > **完美观测 ⇒ 新息必须恒为 0**（四路一起）
//! 它一次性验证四路观测的**符号与帧约定** —— 这类错误此前只能靠逐个 FD 裁判去逮。
//! # 生成式（都是各模块模型式的**严格逆**）
//! - IMU：`Δang = (ω_body + bg)·dt_ang`、`Δvel = (f_b + ba)·dt_vel`
//!   （因为 `propagate` 内部算的是 `Δang/dt − bg` 与 `Δvel/dt − ba`）
//! - baro：模型 `alt == −p_z` ⇒ `alt = −p_z`
//! - GPS 位置 / 速度：直接取 `p` / `v`
//! - 磁航向：`meas = R(q)ᵀ·mag_i + mag_b`（⇒ 零偏扣除后转世界恰为 `mag_i`）
use crate::imu_delta::ImuDelta;
use crate::observe::{baro, gps_pos, gps_vel, mag_yaw, Obs, ObsParams};
use crate::propagate::State;
/// 由真值生成一拍 IMU 增量（**无偏**：不含噪声、零偏按真值补偿回去）。
pub fn imu_from_truth(
    st: &State,
    omega_body: [f32; 3],
    f_b: [f32; 3],
    dt_ang: f32,
    dt_vel: f32,
) -> ImuDelta {
    let mut d = ImuDelta {
        delta_ang: [0.0; 3],
        delta_vel: [0.0; 3],
        dt_ang,
        dt_vel,
        ts_ticks: 0,
    };
    for k in 0..3 {
        d.delta_ang[k] = (omega_body[k] + st.bg[k]) * dt_ang;
        d.delta_vel[k] = (f_b[k] + st.ba[k]) * dt_vel;
    }
    d
}
/// 由真值生成气压高度观测（模型 `alt == −p_z` 的逆）。
pub fn baro_from_truth(st: &State, prm: &ObsParams) -> Obs {
    baro(-st.p[2], st, prm)
}
/// 由真值生成 GPS 位置观测。
pub fn gps_pos_from_truth(st: &State, prm: &ObsParams) -> Obs {
    gps_pos(st.p, st, prm)
}
/// 由真值生成 GPS 速度观测。
pub fn gps_vel_from_truth(st: &State, prm: &ObsParams) -> Obs {
    gps_vel(st.v, st, prm)
}
/// 由真值生成磁航向观测：`meas = R(q)ᵀ·mag_i + mag_b`。
pub fn mag_yaw_from_truth(st: &State, sigma: f32) -> Obs {
    let m = st.q.conj().rotate(st.mag_i);
    let meas = [
        m[0] + st.mag_b[0],
        m[1] + st.mag_b[1],
        m[2] + st.mag_b[2],
    ];
    mag_yaw(meas, st, sigma)
}
