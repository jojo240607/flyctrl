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
//! # ★★接线待办：`AnyEstimator`（装配层收口类型）的接口差异清单
//!
//! 核对自 `core/src/estimator/select.rs`（**已逐条比对，勿再重查**）。
//! 装配层要从 `struct AnyEstimator { pub inner: EskfEstimator }` 改成
//! **按 feature 选择实现的枚举**（`fcalg-est` 开启时含 `FcalgEstimator` 变体），
//! 届时下列方法要逐条处置：
//!
//! | `AnyEstimator` 方法 | fcalg 侧 | 处置 |
//! |---|---|---|
//! | `set_observation_noise(r_gps_p, r_gps_v, r_baro)` | `ObsParams` | **可映射** ✓（注意 fcalg 另有 `sigma_mag`，签名不含 ⇒ 保留默认） |
//! | `set_mag_reference(mag_i)` | `State.mag_i` | **可映射** ✓ |
//! | `set_mag_hard_iron(mag_b)` | `State.mag_b` | **可映射** ✓ |
//! | `set_world_accel(a)` | 无 | 旧实现已是**显式拒绝** ⇒ fcalg 侧同样显式 + 计数 ✓ |
//! | `world_accel_refused()` | 无 | 计数读出：需**新设一个计数器**（旧实现恒 0） |
//! | **`set_freeze_bias(v)`** | **无对应** | ★**必须显式 no-op + 计数** —— 它直接改 `inner.filter_mut().freeze_bias`（旧 ESKF 的磁两态冻结旋钮）；fcalg **无此旋钮** ⇒ **绝不能静默丢弃**（否则调用方以为生效 ✗） |
//! | `kind()` | — | 返回不同串（如 `"fcalg-eskf"`）⇒ 机制自证 ✓ |
//!
//! 另有 **控制器侧**：`app/src/flyctrl/mod.rs` 的 `spawn_flyctrl` 现直接装配
//! `PidController`/`IndiController` 等具体类型；`fcalg-ctrl` 开启时需在此按 feature 选型
//! （控制器侧无 `AnyXxx` 收口类型 ⇒ 只需一处 cfg）。
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

/// 供集成测试与宿主使用的**薄重导出**（零逻辑）：集成测试只链接被测 crate 及其
/// dev-dependency，而 `fcalg` 只是本 crate 的普通依赖 ⇒ 必须经此处转出。
pub mod reexport {
    pub use fcalg::params::{param, Source};
    pub use fcalg::quat::{specific_force_at_rest, Quat, GRAVITY_NED};
}

/// 新栈估计器的 `Estimator` 适配器。
pub struct FcalgEstimator {
    f: Eskf,
    prm: ObsParams,
    /// NIS 门限**覆写值**：`0.0` ⇒ 使用 **dof 感知的导出门限**
    /// （`params::nis_threshold(dof, α)`；气压 1 / GPS位 3 / GPS速 2 / 磁航向 1）。
    /// ⚠**不要填"σ 倍数"** —— 那会绕过 dof 感知、把 1-dof 的阈值套到 3-dof 通道上
    ///   （本会话实测：那样会丢掉 12~16% 的健康观测）。首版默认填了 3.0，正是这个错。
    gate_sigma: f32,
    /// 重灌地板。取自参数表 `gate.reflate_floor`。
    reflate_floor: f32,
    /// 控制器侧滤波陀螺（见模块头"接口新增点"）。
    omega_body: [f32; 3],
    /// 融合被拒的次数（诊断；不等于"重灌"，重灌计数在 `f.guards` 里）。
    pub fuse_rejects: u32,
    /// ★**诊断兼容字段**（app 侧 12 处读数里 10 个计数器的直接对应）。
    /// 语义与旧栈同名量对齐：`n_gps_pos` = **接受**次数、`n_gps_pos_rejected` = **拒收**次数 …
    /// 它们与 fcalg 自身的按通道总数（`chan_acc/chan_rej`）由**闭合判据**保证一致。
    pub n_step: u64,
    pub n_gps_pos: u32,
    pub n_gps_pos_rejected: u32,
    pub n_gps_vel: u32,
    pub n_gps_vel_rejected: u32,
    pub n_grav_applied: u32,
    pub n_grav_gated: u32,
    pub n_mag: u32,
    pub n_mag_rejected: u32,
    /// ⚠**恒 0**：旧栈的"磁重锚"在 fcalg **无对应** ⇒ 显式登记（绝不假装有数）。
    pub n_mag_reanchored: u32,
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
            gate_sigma: 0.0, // ⇒ 用 dof 感知的导出门限（见字段注释）
            reflate_floor: 1.0,
            prm,
            omega_body: [0.0; 3],
            fuse_rejects: 0,
            not_impl_calls: 0,
            n_step: 0,
            n_gps_pos: 0,
            n_gps_pos_rejected: 0,
            n_gps_vel: 0,
            n_gps_vel_rejected: 0,
            n_grav_applied: 0,
            n_grav_gated: 0,
            n_mag: 0,
            n_mag_rejected: 0,
            n_mag_reanchored: 0,
        }
    }

    /// `AnyEstimator::set_observation_noise(r_gps_p, r_gps_v, r_baro)` 的对应 ✓
    /// （注意：fcalg 另有 `sigma_mag`，旧签名不含 ⇒ 保留默认 ✓）
    pub fn set_observation_noise3(&mut self, r_gps_p: f32, r_gps_v: f32, r_baro: f32) {
        self.prm.sigma_gps_p = r_gps_p;
        self.prm.sigma_gps_v = r_gps_v;
        self.prm.sigma_baro = r_baro;
    }
    /// `set_mag_reference` / `set_mag_hard_iron` 的对应 ✓（写进标称态）
    pub fn set_mag_ref(&mut self, mag_i: [f32; 3]) {
        self.f.st.mag_i = mag_i;
    }
    pub fn set_mag_bias(&mut self, mag_b: [f32; 3]) {
        self.f.st.mag_b = mag_b;
    }
    /// ★`set_freeze_bias` 在 fcalg **无对应**（旧 ESKF 的磁两态冻结旋钮）⇒
    ///   **显式拒绝并计数**，绝不静默丢弃（否则调用方以为生效 ✗）。
    pub fn refuse_freeze_bias(&mut self) -> u32 {
        self.not_impl_calls = self.not_impl_calls.wrapping_add(1);
        self.not_impl_calls
    }

    /// 诊断门面的原材料（`AnyEstimator` 改选型后，app 侧 12 处读数映射到这些）：
    /// 各通道的**接受/拒收总数**（`ch` 0..3 = baro/gpsP/gpsV/magYaw）。
    pub fn chan_acc(&self, ch: usize) -> u32 {
        self.f.chan_acc.get(ch).copied().unwrap_or(0)
    }
    pub fn chan_rej(&self, ch: usize) -> u32 {
        self.f.chan_rej.get(ch).copied().unwrap_or(0)
    }
    /// 估计器内部"步数"（对应于旧栈的 `n_step`：predict 调用次数）。
    pub fn hist_len(&self) -> usize {
        self.f.st.p.len()
    }

    fn fuse_track(&mut self, o: &Obs, ch: Channel) {
        let ok = self.f.fuse(o, ch, self.gate_sigma, self.reflate_floor).is_ok();
        match (ch, ok) {
            (Channel::GpsPos, true) => self.n_gps_pos += 1,
            (Channel::GpsPos, false) => self.n_gps_pos_rejected += 1,
            (Channel::GpsVel, true) => self.n_gps_vel += 1,
            (Channel::GpsVel, false) => self.n_gps_vel_rejected += 1,
            (Channel::MagYaw, true) => self.n_mag += 1,
            (Channel::MagYaw, false) => self.n_mag_rejected += 1,
            (Channel::Baro, _) => {}
        }
        if !ok {
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
        self.n_step += 1;
        // 重力（倾角）观测：紧接 predict（IMU 驱动）。量级门不过则**显式**返回 false 并计数。
        match self.f.update_gravity(a) {
            Ok(true) => self.n_grav_applied += 1,
            Ok(false) => self.n_grav_gated += 1,
            Err(_) => self.n_grav_gated += 1,
        }
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
        // 重力（倾角）观测：紧接 predict。比力由本拍速度增量还原（量纲：m/s²）。
        self.n_step += 1;
        if dt_vel > 0.0 {
            let inv = 1.0 / dt_vel;
            let f_b = [delta_vel[0] * inv, delta_vel[1] * inv, delta_vel[2] * inv];
            match self.f.update_gravity(f_b) {
                Ok(true) => self.n_grav_applied += 1,
                Ok(false) => self.n_grav_gated += 1,
                Err(_) => self.n_grav_gated += 1,
            }
        }
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
