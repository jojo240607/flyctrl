//! **ESKF（误差状态 EKF，`eskf.rs`）的 `Estimator` 适配器** ✓ —— 让它可作产品路径的默认估计器。
//!
//! 命名说明 ✓：本文件与 `eskf.rs` 同族（C1+C2 = ESKF 的两项扩展 ✓），故统一用 `eskf` 前缀 ✓。
//!
//! 设计纪律（见 `docs/c1-migration-plan.md` ✓）：
//! - **不改 `eskf.rs` 的算法** ✗ —— 本文件只做接口搬运 ✓
//! - **绝不静默 no-op** ✗ —— 无等价实现的通路【显式拒绝】并计数 ✓
//! - **每条通路都有计数** ✓（"先证明机制确实在运行" ✓）

use crate::estimator::eskf::Eskf;
use crate::estimator::trait_def::Estimator;
use crate::units::Second;
use crate::vehicle::{
    AirspeedSample, ImuSample, PosSample, Quaternion, RtkSample, VehicleState, VioSample,
};

/// ★★★**全局诊断计数**（供 M 场经 ELF 符号读取 ✓ —— 回答"固件上 ESKF 实际收到了
/// 哪些观测、各多少次" ✗✓，即"先证明机制在运行"✓）。
///
/// 索引约定 ✓（与 `EskfEstimator` 的实例计数同义）：
///  0 n_step · 1 n_grav_applied · 2 **n_grav_gated** · 3 n_baro · 4 n_baro_rejected
///  5 n_gps_pos · 6 n_gps_pos_rejected · 7 n_gps_vel · 8 n_gps_vel_rejected
///  9 n_mag · 10 n_mag_rejected · 11 n_mag_reanchored
#[used]
pub static mut ESKF_COUNTS: [u32; 12] = [0; 12];

/// 递增全局诊断计数（诊断用 ✓，不与实例计数冲突 ✓）
#[inline]
fn bump(idx: usize) {
    unsafe {
        let p = core::ptr::addr_of_mut!(ESKF_COUNTS);
        let cur = core::ptr::read_volatile((*p).as_ptr().add(idx));
        core::ptr::write_volatile((*p).as_mut_ptr().add(idx), cur.wrapping_add(1));
    }
}

/// C1 适配器：把 [`Eskf`] 包成统一 [`Estimator`] ✓。
pub struct EskfEstimator {
    f: Eskf,
    /// 世界重力（NED，向下为正 ✓）
    g_ned: [f32; 3],
    /// 磁 `mag_I` 先验（世界系 ✓）—— `reset_mag_states` 的必需输入 ✓（§11.2 ✓）
    mag_i_prior: [f32; 3],
    /// 首个磁样本 ⇒ 触发代数反解 ✓
    mag_first_done: bool,
    /// 陀螺/加计读数（供 `state()` 的 `omega` ✓）
    omega_body: [f32; 3],

    // ---- ★机制计数（"证明它在运行" ✓）----
    /// `step` 调用次数 ✓
    pub n_step: u32,
    /// 重力辅助成功应用的次数（C1 版"锚定"✓）
    pub n_grav_applied: u32,
    /// 重力辅助被**加速度门**拒绝的次数 ✓（C1 版 A11 的机制 ✓）
    pub n_grav_gated: u32,
    /// 气压更新成功次数 ✓
    pub n_baro: u32,
    /// 磁更新成功次数 ✓
    pub n_mag: u32,
    /// 磁被门拒绝次数 ✓
    pub n_mag_rejected: u32,
    /// ★重锚定应用的【分量】次数 ✓（证明机制在运行 ✓）
    pub n_mag_reanchored: u32,
    /// GPS 位置/速度更新**成功**次数 ✓
    pub n_gps_pos: u32,
    pub n_gps_vel: u32,
    /// ★气压/GPS 更新**被门拒绝**次数 ✓（与"成功"分开计 ✗ 绝不混同 ✓）
    pub n_baro_rejected: u32,
    pub n_gps_pos_rejected: u32,
    pub n_gps_vel_rejected: u32,
    /// ★**无等价实现的通路**被显式拒绝的次数（不得静默 ✗）
    pub n_vio_refused: u32,
    pub n_rtk_refused: u32,
    pub n_airspeed_refused: u32,
}

impl EskfEstimator {
    /// 新建：`q0/v0/p0` 初值 + 观测门 `gate`（σ ✓）+ 磁 `mag_I` 先验 ✓。
    pub fn new(q0: Quaternion, v0: [f32; 3], p0: [f32; 3], gate: f32, mag_i_prior: [f32; 3]) -> Self {
        Self {
            f: Eskf::new(q0, v0, p0, gate),
            g_ned: [0.0, 0.0, 9.81],
            mag_i_prior,
            mag_first_done: false,
            omega_body: [0.0; 3],
            n_step: 0,
            n_grav_applied: 0,
            n_grav_gated: 0,
            n_baro: 0,
            n_mag: 0,
            n_mag_rejected: 0,
            n_mag_reanchored: 0,
            n_gps_pos: 0,
            n_gps_vel: 0,
            n_baro_rejected: 0,
            n_gps_pos_rejected: 0,
            n_gps_vel_rejected: 0,
            n_vio_refused: 0,
            n_rtk_refused: 0,
            n_airspeed_refused: 0,
        }
    }

    /// 四旋翼默认配置（产品路径用 ✓）：水平静止起步 + 磁 `mag_I` 先验（本场地典型值 ✓）。
    pub fn default_quad() -> Self {
        Self::new(
            Quaternion::from_axis_angle([0.0, 0.0, 1.0], crate::units::Radian(0.0)),
            [0.0; 3],
            [0.0; 3],
            5.0,
            [0.2, 0.0, 0.4],
        )
    }

    /// 内部滤波器（诊断/测试用 ✓）
    pub fn filter(&self) -> &Eskf {
        &self.f
    }
}

impl Estimator for EskfEstimator {
    fn step(
        &mut self,
        dt: Second,
        imu: ImuSample,
        pos: Option<PosSample>,
        airspeed: Option<AirspeedSample>,
    ) -> VehicleState {
        let dtf = dt.0;
        self.n_step = self.n_step.wrapping_add(1);
        bump(0);

        let gyr = [imu.gyro[0].0, imu.gyro[1].0, imu.gyro[2].0];
        let acc = [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0];
        self.omega_body = gyr;

        // 1) 标称态 + 协方差推进 ✓（速率×dt ✓）
        let d_ang = [gyr[0] * dtf, gyr[1] * dtf, gyr[2] * dtf];
        let d_vel = [acc[0] * dtf, acc[1] * dtf, acc[2] * dtf];
        crate::perf::probe(8); // ESKF: step 进入
        self.f.predict(d_ang, d_vel, dtf, self.g_ned);
        crate::perf::probe(9); // ESKF: predict 完成

        // 2) 重力辅助 ✓（= C1 版的"重力锚定" ✓；机制门在 C1 内部 ✓）
        match self.f.update_gravity(acc, self.g_ned) {
            Ok(n) if n > 0 => { self.n_grav_applied = self.n_grav_applied.wrapping_add(1); bump(1); }
            Ok(_) => {}
            Err(_) => { self.n_grav_gated = self.n_grav_gated.wrapping_add(1); bump(2); } // ★门关闭 ✓
        }

        crate::perf::probe(10); // ESKF: 重力辅助完成
        // 3) GPS 位置/速度 ✓
        let gps_on = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_GPS_ON))
        } >= 0.5;
        if let Some(p) = pos.filter(|_| gps_on) {
            let pm = [p.pos[0].0, p.pos[1].0, p.pos[2].0];
            crate::perf::probe(11); // ESKF: GPS 位置更新前
            if self.f.update_gps_pos(pm).is_ok() {
                self.n_gps_pos = self.n_gps_pos.wrapping_add(1);
                bump(5);
            } else {
                self.n_gps_pos_rejected = self.n_gps_pos_rejected.wrapping_add(1);
                bump(6);
            }
            crate::perf::probe(12); // ESKF: GPS 位置更新后
            if let Some(v) = p.vel {
                let vm = [v[0].0, v[1].0, v[2].0];
                if self.f.update_gps_vel(vm).is_ok() {
                    self.n_gps_vel = self.n_gps_vel.wrapping_add(1);
                    bump(7);
                } else {
                    self.n_gps_vel_rejected = self.n_gps_vel_rejected.wrapping_add(1);
                    bump(8);
                }
            }
        }

        crate::perf::probe(13); // ESKF: GPS 速度更新后
        // 4) 空速：C1 **无等价观测** ✗ ⇒ 显式拒绝 + 计数 ✓（绝不静默 ✗）
        if airspeed.is_some() {
            self.n_airspeed_refused = self.n_airspeed_refused.wrapping_add(1);
        }

        crate::perf::probe(14); // ESKF: 空速检查后
        crate::perf::probe(15); // ESKF: state() 组装前（单调 ✓：18→19→20→22..25→29）
        self.state()
    }

    fn update_alt(&mut self, alt: f32) {
        if self.f.update_baro(alt).is_ok() {
            self.n_baro = self.n_baro.wrapping_add(1);
            bump(3);
        } else {
            self.n_baro_rejected = self.n_baro_rejected.wrapping_add(1);
            bump(4);
        }
    }

    fn update_mag(&mut self, mag: Option<[f32; 3]>) {
        let Some(m) = mag else { return };
        // ★首个磁样本：代数反解 `mag_B`（需独立 `mag_I` 先验 ✓）—— 照参照 `resetMagStates` ✓
        if !self.mag_first_done {
            self.f.reset_mag_states(m, self.mag_i_prior);
            self.mag_first_done = true;
        }
        // ★重锚定（§3.6 ✓）：持续旋转时把 `mag_I` 软拉回先验（旋钮默认 0 = 关 ✓）
        let ra = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_REANCHOR))
        };
        if ra > 0.0 {
            let n = self.f.reanchor_mag_i(self.mag_i_prior, ra);
            self.n_mag_reanchored = self.n_mag_reanchored.wrapping_add(n);
        }
        match self.f.update_mag(m) {
            Ok(_) => { self.n_mag = self.n_mag.wrapping_add(1); bump(9); }
            Err(_) => { self.n_mag_rejected = self.n_mag_rejected.wrapping_add(1); bump(10); }
        }
    }

    fn reset(&mut self) {
        let q = self.f.st.q;
        let (v, p) = (self.f.st.v, self.f.st.p);
        self.f = Eskf::new(q, v, p, 5.0);
        self.mag_first_done = false;
    }

    fn set_initial_attitude(&mut self, q: Quaternion) {
        self.f.st.q = q;
    }

    fn set_initial_position(&mut self, ned: [f32; 3]) {
        self.f.st.p = ned;
    }

    /// ★VIO：C1 **尚无等价观测** ✗ ⇒ 显式拒绝 + 计数 ✓（登记为缺口 ✓）
    fn update_vio(&mut self, vio: Option<VioSample>) {
        if vio.is_some() {
            self.n_vio_refused = self.n_vio_refused.wrapping_add(1);
        }
    }

    /// ★RTK：同上 ✓（C1 目前用 `update_gps_pos` 的同一路 ✓；**不改噪声** ✗以免静默降级 ✓）
    fn update_rtk(&mut self, rtk: Option<RtkSample>) {
        if rtk.is_some() {
            self.n_rtk_refused = self.n_rtk_refused.wrapping_add(1);
        }
    }

    fn state(&self) -> VehicleState {
        let vs = VehicleState {
            time_boot_ms: 0,
            pos: [
                crate::units::Meter(self.f.st.p[0]),
                crate::units::Meter(self.f.st.p[1]),
                crate::units::Meter(self.f.st.p[2]),
            ],
            vel: [
                crate::units::MeterPerSecond(self.f.st.v[0]),
                crate::units::MeterPerSecond(self.f.st.v[1]),
                crate::units::MeterPerSecond(self.f.st.v[2]),
            ],
            att: self.f.st.q,
            omega: [
                crate::units::RadianPerSecond(self.omega_body[0]),
                crate::units::RadianPerSecond(self.omega_body[1]),
                crate::units::RadianPerSecond(self.omega_body[2]),
            ],
            airspeed: crate::units::MeterPerSecond(0.0),
            accel_bias: self.f.st.ba,
        };
        vs
    }

    fn accel_bias(&self) -> [f32; 3] {
        self.f.st.ba
    }
}
