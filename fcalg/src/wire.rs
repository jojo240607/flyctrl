//! L17 · 固件适配层 —— `fcalg` ⇄ `flyctrl` 的**显式映射**（本层不含任何逻辑）
//!
//! # 接线验收口径 = **(b)：定性一致 + 新栈满足自身契约**
//! 不要求与旧栈逐位等价。理由：旧栈 ESKF 带**已知缺陷**（方差静默吞 NaN、无重灌机制），
//! 逐位等价等于要求新栈**复现那些缺陷** —— 那与本次重建的目的相反。
//!
//! # 本层唯一的职责
//! 把"固件手上有什么"映射成 `fcalg` 的类型、把结果映射回固件侧的形状。
//! 不做单位换算之外的任何计算 —— 一旦这里出现"逻辑"，它就成了第三个实现处。
//!
//! # ★语义陷阱（我自己在 L15 栽过一次 ⇒ 这里**由构造闭合**）
//! `fcalg` 的 [`Obs`](crate::observe::Obs) 是"**对当前状态构造**"的（内部含新息 ν）；
//! 而固件的 `update_*` 习惯是"**喂一个测量值**"。
//! 二者混用 ⇒ 新息恒为 0 ⇒ **什么都不发生，且不报错**（静默失效）。
//! 故本层入口**一律接测量值**（[`FwObservation`]），由本层用**当前状态**构造 `Obs`
//! ⇒ 该陷阱在本层**不可能**发生。
//!
//! # 单位与帧（与 `CONTRACT.md` 一致，此处只是搬运）
//! `baro_alt` 为**已减基准**的局部高度，按契约 `alt == −p_z`；
//! `gps_*` 为 **NED**（m / m/s）；`mag_body` 为**机体系**原始值（含零偏）。

use crate::filter::{Eskf, FilterError};
use crate::gate::Channel;
use crate::observe::{baro, gps_pos, gps_vel, mag_yaw, ObsParams};
use crate::propagate::State;

/// 固件手上一拍的观测输入（字段与 `hil.rs`/`wq_tasks.rs` 一一对应；`None` = 本拍无该观测）。
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct FwObservation {
    /// 已减基准的局部高度（契约：`alt == −p_z`）。
    pub baro_alt: Option<f32>,
    /// GPS 位置（NED，m）。
    pub gps_pos: Option<[f32; 3]>,
    /// GPS 速度（NED，m/s）。
    pub gps_vel: Option<[f32; 3]>,
    /// 磁力计（**机体系**原始值，含零偏）。
    pub mag_body: Option<[f32; 3]>,
}

/// 映射回固件侧的估计输出（语义与 `VehicleState` 对齐；字段序不是 ABI，仅语义）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FwEstimate {
    /// NED 位置（m）。
    pub pos: [f32; 3],
    /// NED 速度（m/s）。
    pub vel: [f32; 3],
    /// 姿态四元数（机体→世界，**w,x,y,z**）。
    pub att_wxyz: [f32; 4],
    /// 机体角速度（rad/s）—— 由**控制器侧链路**提供（滤波器不产生角速度）。
    pub omega: [f32; 3],
    /// 加计零偏（机体系，m/s²）。
    pub accel_bias: [f32; 3],
}

/// 各通道本拍的融合结果（`None` = 本拍无该观测）。
pub type FwFuseResult = [Option<Result<(), FilterError>>; 4];

fn ch_slot(ch: Channel) -> usize {
    match ch {
        Channel::Baro => 0,
        Channel::GpsPos => 1,
        Channel::GpsVel => 2,
        Channel::MagYaw => 3,
    }
}

fn put(r: &mut FwFuseResult, ch: Channel, v: Result<(), FilterError>) {
    r[ch_slot(ch)] = Some(v);
}

/// 按固件图样融合本拍全部可用观测。**入口接测量值**，本层用当前状态构造 `Obs`。
pub fn fuse_fw(
    f: &mut Eskf,
    o: &FwObservation,
    prm: &ObsParams,
    gate_sigma: f32,
    reflate_floor: f32,
) -> FwFuseResult {
    let mut out: FwFuseResult = [None; 4];
    if let Some(alt) = o.baro_alt {
        let ob = baro(alt, &f.st, prm);
        put(&mut out, Channel::Baro, f.fuse(&ob, Channel::Baro, gate_sigma, reflate_floor));
    }
    if let Some(p) = o.gps_pos {
        let ob = gps_pos(p, &f.st, prm);
        put(&mut out, Channel::GpsPos, f.fuse(&ob, Channel::GpsPos, gate_sigma, reflate_floor));
    }
    if let Some(v) = o.gps_vel {
        let ob = gps_vel(v, &f.st, prm);
        put(&mut out, Channel::GpsVel, f.fuse(&ob, Channel::GpsVel, gate_sigma, reflate_floor));
    }
    if let Some(m) = o.mag_body {
        let ob = mag_yaw(m, &f.st, prm.sigma_mag);
        put(&mut out, Channel::MagYaw, f.fuse(&ob, Channel::MagYaw, gate_sigma, reflate_floor));
    }
    out
}

/// 估计状态 ⇒ 固件侧形状。`omega_body` 由调用方给（控制器侧链路）。
pub fn to_fw(st: &State, omega_body: [f32; 3]) -> FwEstimate {
    FwEstimate {
        pos: st.p,
        vel: st.v,
        att_wxyz: [st.q.w, st.q.x, st.q.y, st.q.z],
        omega: omega_body,
        accel_bias: st.ba,
    }
}
