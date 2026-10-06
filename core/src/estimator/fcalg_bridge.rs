//! `fcalg` 桥接（feature `fcalg-est`）—— 把重建的新栈接到 [`Estimator`] trait 上
//!
//! # 原则：**薄**
//! 除了字段搬运与三处"显式未实现"的登记，本文件**不含任何算法逻辑** ——
//! 一旦这里出现逻辑，它就变成第三个实现处（旧栈"约定散落"的起点）。
//!
//! # 验收口径 = (b)
//! **定性一致 + 新栈满足自身契约**；**不要求与旧栈逐位等价**
//! （旧栈 ESKF 带已知缺陷：方差静默吞 NaN、无重灌机制 —— 逐位等价等于要求复现它们）。
//!
//! # ★三处"无对应"必须**显式登记并计数**（契约 §4 / 仓里"绝不静默"）
//! `set_accel_lag_s`（新栈不在滤波器内做群延迟补偿）、`update_vio`、`update_rtk`
//! （新栈无对应观测模型）—— 三者都计入 [`NOT_IMPLEMENTED_CALLS`]，
//! 集成验收可直接断言它，**而不是让人误以为它们在起作用**。
//!
//! # ★接口新增点
//! `omega`（机体角速度）**不在滤波器状态里**（它是控制器侧的量）⇒ 桥接自持一份，
//! 由 `step`/`predict_delta` 的输入更新。这一点在 L17 映射表里已登记。

use core::sync::atomic::{AtomicU32, Ordering};

use fcalg::error_state::N;
use fcalg::filter::Eskf;
use fcalg::gate::Channel;
use fcalg::imu_delta::ImuDelta;
use fcalg::observe::{baro, gps_pos, gps_vel, mag_yaw, Obs, ObsParams};
use fcalg::propagate::State;
use fcalg::quat::Quat;
use fcalg::wire::to_fw;

use crate::estimator::trait_def::Estimator;
use crate::units::{Meter, MeterPerSecond, RadianPerSecond, Second};
use crate::vehicle::{
    AirspeedSample, ImuSample, PosSample, Quaternion, VehicleState,
};

/// 被调用但**本桥接不实现**的方法次数（**绝不静默**：验收可断言它）。
pub static NOT_IMPLEMENTED_CALLS: AtomicU32 = AtomicU32::new(0);

/// 初始协方差（对角）。量级取自参数表（`obs.*` / `align.g_tol_frac` 的同一套语义）。
const P_INIT: f32 = 0.5;

fn diag_cov(d: f32) -> fcalg::covariance::Cov {
    let mut p = [[0.0f32; N]; N];
    for i in 0..N {
        p[i][i] = d;
    }
    p
}

/// 新栈估计器的 `Estimator` 适配器。
pub struct FcalgEstimator {
    f: Eskf,
    prm: ObsParams,
    /// NIS 门限（σ）。取自参数表 `gate.nis_sigma` 的当前值。
    gate_sigma: f32,
    /// 重灌地板。取自参数表 `gate.reflate_floor`。
    reflate_floor: f32,
    /// 控制器侧滤波陀螺（见模块头"接口新增点"）。
    omega_body: [f32; 3],
    /// 融合被拒的次数（诊断；不等于"重灌"，重灌计数在 `f.guards` 里）。
    pub fuse_rejects: u32,
    /// **本实例**被调用但未实现的次数（按实例计数 ⇒ 并行安全；
    /// 全局 `NOT_IMPLEMENTED_CALLS` 仅作跨实例聚合，不作为单测断言对象）。
    pub not_impl_calls: u32,
}

impl Default for FcalgEstimator {
    fn default() -> Self {
        Self::new()
    }
}

impl FcalgEstimator {
    pub fn new() -> Self {
        let prm = ObsParams::default();
        Self {
            f: Eskf::new(State::level(), diag_cov(P_INIT), 10),
            gate_sigma: 3.0,
            reflate_floor: 1.0,
            prm,
            omega_body: [0.0; 3],
            fuse_rejects: 0,
            not_impl_calls: 0,
        }
    }

    fn fuse_track(&mut self, o: &Obs, ch: Channel) {
        if self.f.fuse(o, ch, self.gate_sigma, self.reflate_floor).is_err() {
            self.fuse_rejects = self.fuse_rejects.wrapping_add(1);
        }
    }

    /// 位置/速度观测（`PosSample.vel` 是 `Option` ⇒ **必须分两路**，旧栈也是分开的）。
    fn fuse_pos(&mut self, pos: Option<PosSample>) {
        let Some(p) = pos else { return };
        let pm = [p.pos[0].0, p.pos[1].0, p.pos[2].0];
        let o = gps_pos(pm, &self.f.st, &self.prm);
        self.fuse_track(&o, Channel::GpsPos);
        if let Some(v) = p.vel {
            let vm = [v[0].0, v[1].0, v[2].0];
            let o = gps_vel(vm, &self.f.st, &self.prm);
            self.fuse_track(&o, Channel::GpsVel);
        }
    }
}

impl Estimator for FcalgEstimator {
    /// 单帧路径：`ImuDelta = gyro·dt / accel·dt` → predict → 融合位置。
    fn step(
        &mut self,
        dt: Second,
        imu: ImuSample,
        pos: Option<PosSample>,
        _airspeed: Option<AirspeedSample>,
    ) -> VehicleState {
        let dtf = dt.0;
        let g = [imu.gyro[0].0, imu.gyro[1].0, imu.gyro[2].0];
        let a = [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0];
        self.omega_body = g;
        let d = ImuDelta {
            delta_ang: [g[0] * dtf, g[1] * dtf, g[2] * dtf],
            delta_vel: [a[0] * dtf, a[1] * dtf, a[2] * dtf],
            dt_ang: dtf,
            dt_vel: dtf,
            ts_ticks: 0,
        };
        let _ = self.f.predict(&d, fcalg::GRAVITY_NED);
        self.fuse_pos(pos);
        self.state()
    }

    /// 逐样本 predict（环形路径）：直通，字段同名同义。
    fn predict_delta(&mut self, delta_ang: [f32; 3], delta_vel: [f32; 3], dt_ang: f32, dt_vel: f32) {
        if dt_ang > 0.0 {
            // 记下控制器侧角速率（供 `state().omega`）
            let inv = 1.0 / dt_ang;
            self.omega_body = [delta_ang[0] * inv, delta_ang[1] * inv, delta_ang[2] * inv];
        }
        let d = ImuDelta { delta_ang, delta_vel, dt_ang, dt_vel, ts_ticks: 0 };
        let _ = self.f.predict(&d, fcalg::GRAVITY_NED);
    }

    fn update_fusion(
        &mut self,
        pos: Option<PosSample>,
        _airspeed: Option<AirspeedSample>,
    ) -> VehicleState {
        self.fuse_pos(pos);
        self.state()
    }

    /// 气压高度：`hil.rs` 已减 `baro_ref` ⇒ 与契约 `alt == −p_z` 一致。
    fn update_alt(&mut self, alt: f32) {
        let o = baro(alt, &self.f.st, &self.prm);
        self.fuse_track(&o, Channel::Baro);
    }

    /// 磁航向（yaw-only；语义与参数表 `obs.sigma_mag` 对齐）。
    fn update_mag(&mut self, mag: Option<[f32; 3]>) {
        if let Some(m) = mag {
            let o = mag_yaw(m, &self.f.st, self.prm.sigma_mag);
            self.fuse_track(&o, Channel::MagYaw);
        }
    }

    fn set_initial_attitude(&mut self, q: Quaternion) {
        if let Some(n) = (Quat { w: q.w, x: q.x, y: q.y, z: q.z }).normalize() {
            self.f.st.q = n;
        }
    }

    fn set_initial_position(&mut self, ned: [f32; 3]) {
        self.f.st.p = ned;
    }

    fn reset(&mut self) {
        self.f = Eskf::new(State::level(), diag_cov(P_INIT), 10);
        self.omega_body = [0.0; 3];
        self.fuse_rejects = 0;
        self.not_impl_calls = 0;
    }

    fn state(&self) -> VehicleState {
        let e = to_fw(&self.f.st, self.omega_body);
        let mut vs = VehicleState::zero();
        for k in 0..3 {
            vs.pos[k] = Meter(e.pos[k]);
            vs.vel[k] = MeterPerSecond(e.vel[k]);
            vs.omega[k] = RadianPerSecond(e.omega[k]);
        }
        vs.att = Quaternion {
            w: e.att_wxyz[0],
            x: e.att_wxyz[1],
            y: e.att_wxyz[2],
            z: e.att_wxyz[3],
        };
        vs.accel_bias = e.accel_bias;
        vs
    }

    fn accel_bias(&self) -> [f32; 3] {
        self.f.st.ba
    }

    // ── 三处"无对应"：显式登记并计数（绝不静默 no-op）──────────────────────
    fn set_accel_lag_s(&mut self, _tau_s: f32) {
        self.not_impl_calls = self.not_impl_calls.wrapping_add(1);
        NOT_IMPLEMENTED_CALLS.fetch_add(1, Ordering::Relaxed);
    }
    fn update_vio(&mut self, _vio: Option<crate::vehicle::VioSample>) {
        self.not_impl_calls = self.not_impl_calls.wrapping_add(1);
        NOT_IMPLEMENTED_CALLS.fetch_add(1, Ordering::Relaxed);
    }
    fn update_rtk(&mut self, _rtk: Option<crate::vehicle::RtkSample>) {
        self.not_impl_calls = self.not_impl_calls.wrapping_add(1);
        NOT_IMPLEMENTED_CALLS.fetch_add(1, Ordering::Relaxed);
    }
}
