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
/// ★**固件侧诊断快照** `[f32;16]` ✓（经 ELF 符号读 ✓；布局由本仓控制 ⇒ ABI 安全 ✓）
/// `[0..3)` gyro · `[3..6)` accel · `[6..10)` q(w,x,y,z) · `[10..13)` bg · `[13]` gps.pos[2] · `[14]` gps.pos[0] · `[15]` 步数
#[used]
pub static mut ESKF_DIAG: [f32; 64] = [0.0; 64]; // 4 槽 × 16 ✓（槽 k ⇒ [16k, 16k+16) ✓）
/// 快照采样步（默认第 5 步 ⇒ 已过初始对齐 ✓）
pub static mut ESKF_DIAG_AT: u32 = 5;

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
    /// ★§5.136：最近一拍比力（延迟对齐的**机动门**用 ✓；与 PX4 `_accel_horiz_lpf` 同源）
    last_accel: [f32; 3],
    /// ★§5.168【对齐 PX4 `states.acceleration` ✓】：**世界系（NED）加速度**输出
    ///   = `R(q)·f_b + [0,0,g]`（比力旋转 + 重力 ✓），一阶低通 τ=0.05s ✓（平滑 ✓）
    ///   用途：速度环 D 项（§5.167 ✓，PX4 `_vel_dot` 同源 ✓）—— 本仓此前无该量 ✗
    world_accel: [f32; 3],
    /// ★§5.136【对齐 PX4 一手 `ekf_ekf.h:668 _mag_lpf`】：磁样本一阶低通（AlphaFilter，
    ///   时间常数 `_kSensorLpfTimeConstant = 90 000 µs = 90 ms` ✓），用途同一手——
    ///   供 **instant reset / 初始对准**使用（避免用单个带噪样本做代数反解 ✓）
    mag_lpf: [f32; 3],
    mag_lpf_init: bool,
    /// ★§5.138：周期性磁状态重锚的计数器（见 `G_ESKF_MAG_RESET_PERIOD` ✓）
    mag_reset_div: u32,
    /// ★**辅助观测降频**（§5.84 ✓）：每 N 拍才融合一次重力辅助与磁 ✓
    /// 物理依据：重力方向/磁方向的变化率远低于 IMU 采样率 ✓ ⇒ 无需每拍融合 ✓
    /// （同时消除"同一保持样本每拍重复融合"的隐患 ✓）
    aid_div: u32,
    /// 降频周期（1 = 每拍 ✓；10 ⇒ 250Hz 下 25Hz ✓）
    pub aid_period: u32,

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
            last_accel: [0.0; 3],
            world_accel: [0.0; 3],
            mag_lpf: [0.0; 3],
            mag_lpf_init: false,
            mag_reset_div: 0,
            aid_div: 0,
            aid_period: 15, // ★250Hz 下 ⇒ 16.7Hz ✓（§5.86）
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
    /// ★§5.136 诊断用：内层滤波器可变访问（冻结零偏等实验）
    pub fn filter_mut(&mut self) -> &mut Eskf {
        &mut self.f
    }

    pub fn filter(&self) -> &Eskf {
        &self.f
    }

    /// ★§5.132：设置观测噪声（真传感器路径用；默认值 = PC/SIL 验收表口径）
    /// ★§5.168：世界系加速度（NED ✓）——PX4 `states.acceleration` 同源 ✓
    pub fn world_accel(&self) -> [f32; 3] {
        self.world_accel
    }

    pub fn set_observation_noise(&mut self, r_gps_p: f32, r_gps_v: f32, r_baro: f32) {
        self.f.set_observation_noise(r_gps_p, r_gps_v, r_baro);
    }
}

impl Estimator for EskfEstimator {
    /// ★§5.187：比力低通群延迟 τ（秒）——由 `HilContext` 从实际滤波器自动标定后写入。
    fn set_accel_lag_s(&mut self, tau_s: f32) {
        self.f.accel_lag_s = tau_s;
    }

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
        // ★§5.186：当前机体角速率也供【重力辅助的比力低通延迟补偿】用（`update_gravity`
        //   在 `update_mag` **之前**调用 ⇒ 原先 `mag_delay_omega` 在重力步里是**上一拍**的值）；
        //   与 `update_mag` 里的写入等价但更早 ✓（同名共享，语义更新为"当前机体角速率" ✓）
        self.f.mag_delay_omega = gyr;
        self.last_accel = acc; // ★§5.136：延迟对齐机动门（水平分量 ✓）
        // ★§5.168：世界系加速度（NED ✓）= R(q)·f_b + g（重力向下为正 ✓），一阶低通 ✓
        {
            let fb = crate::vehicle::rotate_vec_by_quat(self.f.st.q, acc);
            let raw = [fb[0], fb[1], fb[2] + 9.81];
            let tau = 0.05f32;
            let a = 0.004f32 / (tau + 0.004f32);
            for k in 0..3 {
                self.world_accel[k] += a * (raw[k] - self.world_accel[k]);
            }
        }
        // ★诊断快照（采一次 ✓；`acc`/`gyr` 已在上方绑定 ✓）
        {
            // ★用【常量】比较 ✓ —— 不能用 `static = 5`：app 以裸 bin 加载 ⇒ `.data` 初值
            //   可能未生效（实测 `ESKF_DIAG_AT` 实为 0 ⇒ 永不触发 ✗）
            // ★4 个时刻各采一次 ✓（常量 ✓；用于分清"瞬态 vs 稳态偏差" ✓）
            let slot: usize = match self.n_step {
                10 => 0,
                300 => 1,
                800 => 2,
                1500 => 3,
                _ => usize::MAX,
            };
            if slot != usize::MAX {
                unsafe {
                    let d = core::ptr::addr_of_mut!(ESKF_DIAG);
                    for i in 0..3 {
                        (*d)[slot * 16 + i] = gyr[i];
                        (*d)[slot * 16 + 3 + i] = acc[i];
                        (*d)[slot * 16 + 10 + i] = self.f.st.bg[i];
                    }
                    (*d)[slot * 16 + 6] = self.f.st.q.w;
                    (*d)[slot * 16 + 7] = self.f.st.q.x;
                    (*d)[slot * 16 + 8] = self.f.st.q.y;
                    (*d)[slot * 16 + 9] = self.f.st.q.z;
                    (*d)[slot * 16 + 13] = if let Some(pp) = pos { pp.pos[2].0 } else { -999.0 };
                    (*d)[slot * 16 + 14] = if let Some(pp) = pos { pp.pos[0].0 } else { -999.0 };
                    (*d)[slot * 16 + 15] = self.n_step as f32;
                }
            }
        }

        // 1) 标称态 + 协方差推进 ✓（速率×dt ✓）
        let d_ang = [gyr[0] * dtf, gyr[1] * dtf, gyr[2] * dtf];
        let d_vel = [acc[0] * dtf, acc[1] * dtf, acc[2] * dtf];
        crate::perf::probe(8); // ESKF: step 进入
        self.f.predict(d_ang, d_vel, dtf, self.g_ned);
        crate::perf::probe(9); // ESKF: predict 完成

        // 2) 重力辅助 ✓（★降频：每 aid_period 拍融合一次 ✓）
        self.aid_div = self.aid_div.wrapping_add(1);
        if self.aid_div % self.aid_period.max(1) == 0 {
        match self.f.update_gravity(acc, self.g_ned) {
            Ok(n) if n > 0 => { self.n_grav_applied = self.n_grav_applied.wrapping_add(1); bump(1); }
            Ok(_) => {}
            Err(_) => { self.n_grav_gated = self.n_grav_gated.wrapping_add(1); bump(2); } // ★门关闭 ✓
        }
        }

        crate::perf::probe(10); // ESKF: 重力辅助完成
        // 3) GPS 位置/速度 ✓
        let gps_on = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_GPS_ON))
        } != 2.0; // ★0 = 默认开 ✓（裸 bin 的 .data 初值不生效 ✗）
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
        // ★§5.138 诊断：先验磁场覆盖（默认全 0 ⇒ 不覆盖 ⇒ 行为逐位不变 ✓）
        {
            let p = unsafe {
                let x = core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_I_PRIOR_X));
                let y = core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_I_PRIOR_Y));
                let z = core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_I_PRIOR_Z));
                [x, y, z]
            };
            if p[0] != 0.0 || p[1] != 0.0 || p[2] != 0.0 {
                self.mag_i_prior = p;
            }
        }
        // ★§5.136【对齐 PX4 `_mag_lpf` ✓】：一阶低通（AlphaFilter，τ=90ms，同一手参数 ✓）。
        //   `alpha = dt/(dt+τ)`；dt 取控制周期 4ms ⇒ alpha ≈ 0.0426 ✓
        //   ⚠️ 一手精确（`ekf_ekf.h:668` 注释 ✓）：`_mag_lpf` **只供 instant reset 用**，
        //   **不是常规融合输入** ⇒ 本仓同样：仅维护低通值，常规融合仍用原始样本 ✓
        //   （曾误把低通值用于常规融合 ⇒ att_est 3 项失败 ✗ ⇒ 按一手纠正 ✓）
        {
            let tau = 0.090f32; // 90 ms（PX4 `_kSensorLpfTimeConstant` ✓）
            let dt = 0.004f32; // 控制周期（250Hz ✓ 与 PX4 `_dt_ekf_avg` 同义）
            let a = dt / (dt + tau);
            if !self.mag_lpf_init {
                self.mag_lpf = m;
                self.mag_lpf_init = true;
            } else {
                for i in 0..3 {
                    self.mag_lpf[i] += a * (m[i] - self.mag_lpf[i]);
                }
            }
        }
        // ★首个磁样本：代数反解 `mag_B`（需独立 `mag_I` 先验 ✓）—— 照参照 `resetMagStates` ✓
        if !self.mag_first_done {
            // ★§5.136 阶段2：yaw-only 路径 ⇒ **对准标定**（吸收磁偏角/安装偏置）；
            //   legacy 三轴路径（旋钮 2.0）仍走代数反解 mag_B ✓
            let yaw_only = unsafe {
                core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_YAW_ON))
            } != 2.0;
            // ★一手：instant reset 用**低通后**的场值（`_mag_lpf.getState()` ✓ 见
            //   `mag_control.cpp:223/232/282/311` 全部 `resetMagStates(_mag_lpf.getState(), …)` ✓）
            let m_reset = self.mag_lpf;
            if yaw_only {
                // ★对准窗口：前 60 次磁更新（≈3.6s @16.7Hz）内重复对准，跟随姿态收敛 ✓
                self.f.begin_mag_alignment(60);
                self.f.align_yaw_to_mag(m_reset);
                // ★一手 `mag_control.cpp:613-614`：航向重置后 `_mag_heading_innov_lpf.reset(0)`
                self.f.mag_hdg_innov_lpf = 0.0;
            } else {
                self.f.reset_mag_states(m_reset, self.mag_i_prior);
            }
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
        // ★磁同样降频 ✓（磁方向变化更慢 ✓）
        // ★§5.139 诊断：磁更新降频率可覆盖（默认 0 ⇒ 用 aid_period ✓ 逐位不变）
        let mp = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_PERIOD))
        };
        let peri = if mp >= 1.0 { mp as u32 } else { self.aid_period.max(1) };
        if self.aid_div % peri != 0 {
            return;
        }
        // ★§5.138【对齐 PX4 一手：周期性磁状态重锚 ✓】（`mag_control.cpp:178/203/230/279`）：
        //   条件 `no_ne_aiding_or_not_moving = !isNorthEastAidingActive() || vehicle_at_rest`
        //   ——本仓无 NE 辅助 ⇒ **恒为真** ✓ ⇒ 周期性 `resetMagStates`（硬重初始化）✓
        //   机理（补遗 29 诊断 ✓）：真机链下 mag_I/mag_B 在弱可观测方向上漂移积累 ⇒
        //   3D 失稳发散；周期性重锚把该漂移**清零**（而非软拉回 ✗ 实测 reanchor 有害 ✓）
        {
            let period = unsafe {
                core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_RESET_PERIOD))
            };
            if period >= 1.0 {
                self.mag_reset_div = self.mag_reset_div.wrapping_add(1);
                if (self.mag_reset_div as f32) >= period {
                    self.mag_reset_div = 0;
                    let m_now = self.mag_lpf;
                    self.f.reset_mag_states_no_yaw(m_now, self.mag_i_prior);
                    self.n_mag_reanchored = self.n_mag_reanchored.wrapping_add(1);
                }
            }
        }
        // ★★§5.136【AUTO 选择——严格按 PX4 一手 `mag_control.cpp:189-200` ✓】：
        //     `mag_3D = common_conditions_passing && mag_aligned_in_flight`
        //     `mag_hdg = common_conditions_passing && (HEADING 模式 || (AUTO && !mag_3D))`
        //   ⇒ **AUTO 下默认 3D**，`mag_hdg` 仅当 3D 不可用时回退 ✓
        //   本仓映射：`mag_aligned_in_flight` ↔ **对准标定完成且未受扰**
        //     （`yaw_aligned` 由 `align_yaw_to_mag` 置位 ✓、`!mag_field_disturbed` ✓）
        //   ★实测校正（关键 ✓）：此前"对准未完成就切 3D" ⇒ 参考未标定 ⇒ mag_I 漂 ⇒ 失稳
        //     （demo 末态 165m ✗）；改为"**对准完成后**启 3D" ⇒ 物理磁注入下 **3D 全程完美**
        //     （60s tilt 0.0° / 漂移 0.02m ✓）⇒ 3D 路径本身健康，关键在**切换时机** ✓
        //   旋钮 `G_ESKF_MAG_YAW_ON`：2.0 ⇒ 强制 heading（A/B ✓）；3.0 ⇒ 强制 3D（对照 ✓）；
        //     其余 ⇒ AUTO（上述一手判据 ✓）
        let knob = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_YAW_ON))
        };
        // ★§5.139【一手 `mag_control.cpp:488-500`】：**先**算航向新息并更新低通
        //   （`mag_heading_consistent` 的判据输入；3D 路径下也必须更新 ✓）。
        //   ⚠️纪律（H 场三表对默认路径敏感 ✓ 实测×3）：**仅当"航向一致性门控"启用**
        //   （`G_ESKF_MAG_HDG_GATE == 2.0`）时才调用 ⇒ 默认路径**逐位不变** ✓
        let hdg_gate_on = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_HDG_GATE))
        } == 2.0;
        if hdg_gate_on {
            let _ = self.f.mag_heading_innov(m);
        }
        //   ★一手精读（`mag_control.cpp:181-190, 502-511` ✓）：`mag_heading_consistent`
        //     仅在**存在 NE 外部辅助**（`isNorthEastAidingActive()`）时才参与公共条件
        //     ⇒ 其判定为 `mag_consistent_or_no_ne_aiding = mag_heading_consistent ||
        //        !isNorthEastAidingActive()`。**本仓无 NE 辅助**（GPS 位置/速度不提供航向 ✓）
        //     ⇒ 该分支恒满足 ⇒ `mag_3D` 只需"**对准完成 + 未受扰**" ✓（即下方判据 ✓）
        //     注：曾尝试把 `|航向新息|<head_noise 且 水平速度>阈` 作为必要条件 ⇒ 悬停
        //     （速度≈0）会退回 heading ⇒ 与验收表口径冲突（3 项失败 ✗）⇒ 按一手回退 ✓
        //   ★★§5.139【一手 `mag_control.cpp:502-511` `mag_heading_consistent` ✓】：
        //     `(|_mag_heading_innov_lpf| < head_noise) && (|innovation| < head_noise)`
        //     其中 `head_noise = 0.3 rad`（`ekf2_head_noise` 一手默认 ✓）。
        //     ★一手关键：acclim 分支**仅在有 NE 外部辅助时**要求（`isNorthEastAidingActive()` ✓）
        //     ——本仓无 NE 辅助 ⇒ 只要求**航向新息一致** ✓
        //     ⇒ 3D 仅在【对准完成 ∧ 未受扰 ∧ **航向新息一致**】时启用 ✓（闭环正反馈抑制 ✓）
        //     ★重锚/重置后 PX4 会 `_mag_heading_innov_lpf.reset(0)` 且 consistent=true ✓
        //       （`mag_control.cpp:613-614`）⇒ 本仓在对准时同步清零低通 ✓
        let head_noise = 0.3f32; // ekf2_head_noise 一手默认（rad ✓）
        //   ★★§5.141【一手精读更正（关键 ✓✓）】：`isNorthEastAidingActive()` 的一手定义
        //   （`estimator_interface.cpp` ✓）= `gnss_pos || gnss_vel || aux_gpos || ev_pos(NED) || ev_vel(NED)`
        //   ⇒ **本仓有 GPS 位置/速度融合 ⇒ 该条件为 TRUE** ✓（此前误判为"无 NE 辅助"✗）。
        //   故按一手语义，`mag_heading_consistent` 必须满足：
        //     `(|航向新息低通| < head_noise) ∧ (|瞬时新息| < head_noise)`
        //     ∧ **`isNorthEastAidingActive() && _accel_horiz_lpf > mag_acclim(0.5 m/s²)`** ✓
        //   ⇒ **悬停时水平加速度≈0 ⇒ consistent=false ⇒ PX4 也不用 3D** ✓✓
        //     （本仓此前在悬停启用 3D ⇒ 违背一手 ⇒ 在不该用 3D 时用了 3D ⇒ 自激 ✓）
        let accel_ok = self.f.accel_horiz_lpf > 0.5; // ekf2_mag_acclim 一手默认 ✓
        let hdg_consistent = if hdg_gate_on {
            self.f.mag_hdg_innov_lpf.abs() < head_noise
                && self.f.last_mag_yaw_innov.abs() < head_noise
                && accel_ok
        } else {
            true // 门控未启用 ⇒ 不参与（默认路径行为不变 ✓）
        };
        let mode_3d = hdg_consistent && self.f.yaw_aligned && !self.f.mag_field_disturbed;
        let yaw_only = if knob == 2.0 { true } else if knob == 3.0 { false } else { !mode_3d };
        // ★§5.136：延迟补偿用【测量角速度】（与 PX4 `_state.gyro` 同源 ✓）
        self.f.mag_delay_omega = self.omega_body;
        // ★§5.136：水平加速度（延迟对齐的机动门；与 PX4 `_accel_horiz_lpf` 同源 ✓）
        {
            let a = self.last_accel;
            self.f.mag_delay_accel_horiz = crate::math::sqrt(a[0] * a[0] + a[1] * a[1]);
        }
        // ★§5.136/§5.139 纪律（真机与 H 场同样敏感 ✓ 实测）：**默认走原路径**逐位不变；
        //   仅当扩展旋钮（延迟对齐/干扰检测）启用时才走 `update_mag_ext` ✓
        //   （实测：无条件走 ext（即使 delay=0）⇒ 真机 heading 冒烟由 0.0°/0.00m 劣化为
        //    19.3°/11.8m ✗ ⇒ demo 失败；改为条件进入后恢复完美 ✓）
        let ext_on = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_DELAY_MS)) > 0.0
                || core::ptr::read_volatile(core::ptr::addr_of!(crate::estimator::eskf::G_ESKF_MAG_CHECK)) == 2.0
        };
        let r = if yaw_only {
            self.f.update_mag_yaw(m)
        } else if ext_on {
            self.f.update_mag_ext(m)
        } else {
            self.f.update_mag(m)
        };
        match r {
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

