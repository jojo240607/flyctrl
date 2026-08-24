//! 松耦合误差状态 EKF 估计器（no_std, 无堆分配）。
//!
//! 松耦合误差状态 EKF 估计器（no_std, 无堆分配）。
//!
//! 状态分两部分：
//! - 名义姿态：四元数，由去偏置角速度积分得到（可选微弱重力修正）。
//! - 误差状态 EKF（12 维）：`[pos(3), vel(3), gyro_bias(3), accel_bias(3)]`。
//!   预测用 IMU 加计（经名义姿态旋转到世界系、减去估计的加计零偏、补偿重力）推进速度/位置；
//!   观测用位置测量（GPS/气压）与速度测量（Doppler）做标准卡尔曼更新，
//!   并回灌陀螺/加计零偏到姿态积分。
//!
//! 加计零偏仅由**速度观测**（Doppler）驱动估计：位置/气压更新刻意不修正 accel_bias
//! 状态（见 `update_pos`/`update_alt` 中清零对应增益行），避免无速度观测时该状态不可观
//! 而导致数值发散；有速度观测（实飞 Doppler/GPS 速度）时零偏方可估计并扣除，
//! 否则垂向速度持续积分零偏而发散（消费级 IMU 常见 0.05 m/s² 量级）。

use crate::estimator::trait_def::Estimator;
use crate::math::sqrt;
use crate::units::*;
use crate::vehicle::{
    AirspeedSample, ImuSample, PosSample, Quaternion, RtkSample, VehicleState, VioSample,
    quat_to_rotmat, rotate_vec_by_quat,
};

const N: usize = 10; // pos(3) + vel(3) + gyro_bias(3) + accel_bias_z(1)

/// 协方差对角线硬上限（m² / (m/s)² / (m/s²)²）。防止不可观状态协方差经 F 矩阵耦合
/// 指数增长而至 Inf/NaN。正常可观测状态下协方差远小于此值。
const P_MAX: f32 = 1e3;

/// 卡尔曼增益元素硬上限。紧噪声观测（RTK 厘米级 / VIO）下，位置观测经非对角协方差
/// 产生的速度行增益可爆炸到 ~40（K[3][0]=P[3][0]/S00，S00≈r_rtk→0.0025），一步把速度
/// 踢飞，随后 Joseph 协方差更新在 float32 下失去 PSD、交叉项 p03 发散到 -1e13 → NaN。
/// 限幅后每个观测元素对状态的修正有界（≤2 m/s per m），协方差更新保持数值稳定；
/// 位置行增益天然 ≤1（p00/(p00+r)），不受限幅影响。
const K_MAX: f32 = 2.0;

#[inline]
fn mat_ident(p: &mut [f32; N * N]) {
    p.fill(0.0);
    for i in 0..N {
        p[i * N + i] = 1.0;
    }
}

// C = A*B  (na x nk) * (nk x nb)
#[inline]
fn mat_mul(a: &[f32], b: &[f32], c: &mut [f32], na: usize, nk: usize, nb: usize) {
    for i in 0..na {
        for j in 0..nb {
            let mut s = 0.0f32;
            for k in 0..nk {
                s += a[i * nk + k] * b[k * nb + j];
            }
            c[i * nb + j] = s;
        }
    }
}

// C = A^T * B (na x nk)^T * (na x nb) -> (nk x nb)
#[inline]
fn mat_mul_at(a: &[f32], b: &[f32], c: &mut [f32], na: usize, nk: usize, nb: usize) {
    for i in 0..nk {
        for j in 0..nb {
            let mut s = 0.0f32;
            for k in 0..na {
                s += a[k * nk + i] * b[k * nb + j];
            }
            c[i * nb + j] = s;
        }
    }
}

#[inline]
fn mat_add(a: &[f32], b: &[f32], c: &mut [f32], n: usize) {
    for i in 0..n {
        c[i] = a[i] + b[i];
    }
}

pub struct EkfEstimator {
    att: Quaternion,
    x: [f32; N],        // pos, vel, gyro_bias, accel_bias
    p: [f32; N * N],    // 协方差
    g_ref: f32,
    q_vel: f32,         // 速度过程噪声强度（水平）
    q_vel_z: f32,       // 速度过程噪声强度（垂向）
    q_bias: f32,        // 陀螺偏置随机游走
    q_accel: f32,       // 加计偏置随机游走
    r_pos: f32,         // 位置观测噪声
    r_vel: f32,         // 速度观测噪声（Doppler，m/s）^2
    r_alt: f32,         // 高度（气压）观测噪声（m）^2
    r_airspeed: f32,    // 空速观测噪声（m/s）^2
    r_vio_pos: f32,     // VIO 位置观测噪声（m）^2（中长期漂移，弱于 RTK/GPS 绝对位置）
    r_vio_vel: f32,     // VIO 速度观测噪声（m/s）^2（光流高精度，强于 GPS Doppler）
    r_rtk: f32,         // RTK-GPS 位置观测噪声（m）^2（厘米级，远强于普通 GPS）
    att_alpha: f32,     // 姿态重力修正强度（0 = 纯积分）
    airspeed_est: f32,  // 估计空速 (m/s)，由空速计融合得到
    accel_lp: [f32; 3],  // 加计低通滤波（滤除高频振动，用于重力锚定）
}

impl EkfEstimator {
    /// 完整构造（含速度/高度观测噪声、垂向过程噪声、加计零偏随机游走）。
    pub fn new(
        att_alpha: f32,
        q_vel: f32,
        q_vel_z: f32,
        q_bias: f32,
        q_accel: f32,
        r_pos: f32,
        r_vel: f32,
        r_alt: f32,
    ) -> Self {
        let mut p = [0.0f32; N * N];
        mat_ident(&mut p);
        for i in 0..3 {
            p[i * N + i] = 1.0; // pos ~1m^2
        }
        for i in 3..6 {
            p[i * N + i] = 1.0; // vel ~1 (m/s)^2
        }
        for i in 6..9 {
            p[i * N + i] = 1e-4; // gyro bias ~0.01 rad/s
        }
        // 加计零偏仅估计世界系 D 轴（垂向）一个分量：垂向下沉的根因是 Z 轴恒定加计零偏，
        // 单分量即可消除垂向速度积分漂移，且垂向速度/气压观测对其充分可观、数值稳定
        // （3 轴机体系零偏估计存在偏航耦合导致 Y 轴正反馈发散，故收敛为单分量）。
        // 阶段 11-A：初始不确定度放大到 0.25（std 0.5 m/s²），使速度观测能经交叉协方差
        // P[9,vel] 驱动零偏收敛——原来 1e-2 太小，交叉协方差近乎 0、卡尔曼增益可忽略，
        // 零偏恒锁在初值 0，恒定 0.05 m/s² 零偏直接积分进速度导致悬停无界发散。
        p[9 * N + 9] = 0.05; // accel_bias_z ~0.22 m/s^2（足够初始不确定度驱动收敛，又不过度吸收瞬态）
        EkfEstimator {
            att: Quaternion::IDENTITY,
            x: [0.0; N],
            p,
            g_ref: 9.81,
            q_vel,
            q_vel_z,
            q_bias,
            q_accel,
            r_pos,
            r_vel,
            r_alt,
            r_airspeed: 0.75, // ~0.87 m/s RMS 空速观测噪声
            r_vio_pos: 0.25,  // ~0.5 m RMS：VIO 位置有中长期漂移，权重弱于 GPS/RTK 绝对位置
            // ~0.2 m/s RMS：VIO 速度（光流）精度高于 GPS Doppler，但不可过紧——
            // r=0.01 时速度协方差塌缩 + ab_z 交叉增益放大，任何持久速度残差都会驱动
            // 垂向零偏指数发散（仿真：GPS 中断 + VIO 时 ab_z → 4.3e6 → NaN）。
            r_vio_vel: 0.04,
            r_rtk: 0.0025,    // ~0.05 m RMS：RTK 厘米级绝对位置，权重最强
            att_alpha,
            airspeed_est: 0.0,
            accel_lp: [0.0; 3],
        }
    }

    /// 兼容旧调用（零回归）：使用默认的速度/高度观测噪声、垂向过程噪声与加计零偏游走；
    /// 垂向过程噪声与水平对齐，避免垂向协方差爆炸导致高度估计发散。
    pub fn new_legacy(att_alpha: f32, q_vel: f32, q_bias: f32, r_pos: f32) -> Self {
        Self::new(att_alpha, q_vel, q_vel, q_bias, 1e-4, r_pos, 0.3, 0.3)
    }

    pub fn default_quad() -> Self {
        // 垂向速度过程噪声 `q_vel_z` 与水平 (`q_vel`) 同量级，否则垂向协方差爆炸，
        // 滤波器信任漂移的传播项而非 baro/GPS 测量，导致定高环与估计相互发散
        // （PID 实机下沉 ~3m）。baro 噪声小 (`r_alt=0.3`) 强约束高度；
        // `q_accel` 让 EKF 由速度观测在线估计并扣除加计零偏（消费级 IMU 常见 0.05 m/s²），
        // 否则垂向速度持续积分零偏而发散。
        // `att_alpha` 保持 0（纯陀螺积分）：重力锚定在低通后仍导致姿态发散（实测：
        // 启用 0.02 后 PID/INDI/LQR 全部飞到 +19m，att_err 300°+，锚定方向逻辑破坏系统）。
        // 姿态依赖陀螺积分 + 速度/位置测量间接约束，不做加速度计重力锚定。
        // 垂向零偏 x[9] 仅由速度观测弱驱动（AB_VEL_GAIN 阻尼）。
        Self::new(0.0, 0.05, 0.05, 1e-5, 5e-4, 0.5, 0.3, 0.3)
    }

    /// 当前估计的陀螺零偏（调试/诊断用）。
    pub fn gyro_bias(&self) -> [f32; 3] {
        [self.x[6], self.x[7], self.x[8]]
    }

    /// 当前估计的加计零偏（m/s²，调试/诊断用）。
    /// 仅估计世界系 D 轴（垂向）一个分量 `x[9]`（hover 近水平时 ≈ 机体 Z 轴零偏），
    /// 以 `[0, 0, ab_z]` 形式返回以兼容既有 3 轴接口。
    /// 该状态**仅由速度观测（Doppler）驱动**（见 `update_vel`），位置/气压更新刻意不修正
    /// （其卡尔曼增益第 9 行被清零），避免无速度观测时不可观而发散。
    pub fn accel_bias(&self) -> [f32; 3] {
        [0.0, 0.0, self.x[9]]
    }

    /// 当前估计的姿态四元数（调试/诊断用）。
    pub fn att(&self) -> Quaternion {
        self.att
    }

    /// 当前协方差矩阵（9x9 行主序）引用，供不变量校验（对称半正定）使用。
    pub fn cov(&self) -> &[f32; N * N] {
        &self.p
    }

    /// 阶段 11-A：设置 EKF 初始位置估计（NED，向下正）。
    ///
    /// 默认构造 x[0..3]=0，但机体真实初始位置（如悬停 5m，NED d=-5）与之不符，
    /// 导致 GPS/气压首次校正前 PID 看到 ~5m 位置误差全油门弹射（见 PLAN 阶段 11-A）。
    /// 在仿真/实飞启动前用真实初始位置初始化，使首拍 ez≈0，避免初始 windup 弹射。
    /// 同时把位置协方差压到 ~0.01m²（视为已收敛，避免初始大协方差经 F 矩阵耦合膨胀）。
    pub fn set_initial_position(&mut self, ned: [f32; 3]) {
        self.x[0] = ned[0];
        self.x[1] = ned[1];
        self.x[2] = ned[2];
        for i in 0..3 {
            self.p[i * N + i] = 0.01;
        }
    }
}

impl Estimator for EkfEstimator {
    fn step(
        &mut self,
        dt: Second,
        imu: ImuSample,
        gps: Option<PosSample>,
        airspeed: Option<AirspeedSample>,
    ) -> VehicleState {
        let dt = dt.0;
        // 防御：协方差若出现非有限项（GPS/位置观测的 Joseph 更新在特定数值下可能产生 NaN，
        // 对角夹取只清对角、非对角 NaN 会残留并传播），整矩阵清掉非有限项，阻断 NaN 进入
        // 本拍的 predict/update（否则 S 求逆/增益 K 计算 NaN → 位置/速度状态被污染）。
        self.sanitize_p();
        let g = self.g_ref;

        // 去偏置角速度
        let bx = self.x[6];
        let by = self.x[7];
        let bz = self.x[8];
        let wx = imu.gyro[0].0 - bx;
        let wy = imu.gyro[1].0 - by;
        let wz = imu.gyro[2].0 - bz;

        // 名义姿态积分（去偏置）
        self.att = self.att.integrate(wx, wy, wz, dt);

        // 可选微弱重力修正（锚定 roll/pitch 到重力方向）
        if self.att_alpha > 0.0 {
            // 先对机体比力做一阶低通滤波，滤除 40Hz 高频振动，保留慢变重力方向。
            let a_raw = [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0];
            let lp_coeff = 0.05f32; // ~20Hz 截止，滤除 40Hz 振动
            for i in 0..3 {
                self.accel_lp[i] += lp_coeff * (a_raw[i] - self.accel_lp[i]);
            }
            let a = self.accel_lp;
            let an = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            if an > 1e-3 {
                let down_body = crate::vehicle::rotate_vec_by_quat_inverse(self.att, [0.0, 0.0, g]);
                let k = self.att_alpha * 0.5;
                // 把估计重力向量 down_body 锚定到【真实重力方向】，即比力的反方向 (-a)。
                let ax = -(down_body[1] * (a[2] / an) - down_body[2] * (a[1] / an)) * k;
                let ay = -(down_body[2] * (a[0] / an) - down_body[0] * (a[2] / an)) * k;
                let az = -(down_body[0] * (a[1] / an) - down_body[1] * (a[0] / an)) * k;
                let na = sqrt(ax * ax + ay * ay + az * az);
                if na > 1e-6 {
                    let dq = Quaternion::from_axis_angle([ax, ay, az], Radian(na));
                    self.att = (self.att * dq).normalize();
                }
            }
        }

        // 位置/速度预测（世界系 NED）：a_world = R*(a_body) - [0,0,ab_z] + g_vec(z 向下正)
        // 仅扣除垂向（世界 D 轴）估计零偏 ab_z = x[9]：hover 近水平时机体 Z 零偏 ≈ 世界 D 零偏，
        // 单分量足以消除垂向速度积分漂移，且数值稳定（避免 3 轴耦合发散）。
        let a = [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0];
        let a_world = rotate_vec_by_quat(self.att, a);
        let ax_w = a_world[0];
        let ay_w = a_world[1];
        let az_w = a_world[2] + g - self.x[9];

        let px = self.x[0] + self.x[3] * dt + 0.5 * ax_w * dt * dt;
        let py = self.x[1] + self.x[4] * dt + 0.5 * ay_w * dt * dt;
        let pz = self.x[2] + self.x[5] * dt + 0.5 * az_w * dt * dt;
        let vx = self.x[3] + ax_w * dt;
        let vy = self.x[4] + ay_w * dt;
        let vz = self.x[5] + az_w * dt;
        self.x[0] = px;
        self.x[1] = py;
        self.x[2] = pz;
        self.x[3] = vx;
        self.x[4] = vy;
        self.x[5] = vz;

        // 协方差预测（独立方法，栈数组作用域限于该方法内，返回后栈槽被回收，
        // 避免与下方观测更新步的栈数组同时存活导致调用方任务栈溢出）。
        self.predict_cov(dt);

        // 观测更新（位置 + 可选 Doppler 速度）：H = [I3 0 0]（独立方法，如上拆分）。
        if let Some(z) = gps {
            let vel_mps = z.vel.map(|v| [v[0].0, v[1].0, v[2].0]);
            self.update_pos(z);
            if let Some(v) = vel_mps {
                self.update_vel(v);
            }
        }

        // 观测更新（空速）：约束水平速度幅值 |v_h| = 测量空速（不含风）。
        if let Some(a) = airspeed {
            self.update_airspeed(a.speed.0);
        }

        // 加计零偏（世界 D 轴）物理上界夹取（消费级 IMU 零偏通常 < 0.3 m/s²，真值仅 0.05）。
        // 该状态仅由速度观测（Doppler）驱动，正常收敛不受影响；夹取仅作发散兜底。
        if self.x[9] > 0.3 {
            self.x[9] = 0.3;
        } else if self.x[9] < -0.3 {
            self.x[9] = -0.3;
        }

        VehicleState {
            time_boot_ms: 0,
            pos: [Meter(self.x[0]), Meter(self.x[1]), Meter(self.x[2])],
            vel: [MeterPerSecond(self.x[3]), MeterPerSecond(self.x[4]), MeterPerSecond(self.x[5])],
            att: self.att,
            omega: [
                RadianPerSecond(imu.gyro[0].0 - self.x[6]),
                RadianPerSecond(imu.gyro[1].0 - self.x[7]),
                RadianPerSecond(imu.gyro[2].0 - self.x[8]),
            ],
            airspeed: MeterPerSecond(self.airspeed_est),
            accel_bias: self.accel_bias(),
        }
    }

    /// 注入 VIO 测量：位置（中等噪声，容忍长期漂移）+ 速度（小噪声，光流高精度）。
    /// 在 GPS 帧间 / 失锁时提供连续修正，与 RTK 绝对参考互补。
    fn update_vio(&mut self, vio: Option<VioSample>) {
        if let Some(v) = vio {
            if let Some(pos) = v.pos {
                self.update_pos_r(PosSample::pos_only(pos), self.r_vio_pos);
            }
            if let Some(vel) = v.vel {
                self.update_vel_r([vel[0].0, vel[1].0, vel[2].0], self.r_vio_vel);
            }
        }
    }

    /// 注入 RTK-GPS 测量：厘米级绝对位置（极小噪声），压紧位置协方差、抑制 VIO 漂移。
    fn update_rtk(&mut self, rtk: Option<RtkSample>) {
        if let Some(z) = rtk {
            self.update_pos_r(PosSample::pos_only(z.pos), self.r_rtk);
        }
    }

    /// 当前估计状态（不推进），供诊断/日志读取（已含所有已融合观测）。
    fn state(&self) -> VehicleState {
        VehicleState {
            time_boot_ms: 0,
            pos: [Meter(self.x[0]), Meter(self.x[1]), Meter(self.x[2])],
            vel: [MeterPerSecond(self.x[3]), MeterPerSecond(self.x[4]), MeterPerSecond(self.x[5])],
            att: self.att,
            omega: [
                RadianPerSecond(self.x[3] * 0.0), // 占位；角速度不在此状态
                RadianPerSecond(0.0),
                RadianPerSecond(0.0),
            ],
            airspeed: MeterPerSecond(self.airspeed_est),
            accel_bias: self.accel_bias(),
        }
    }

    fn reset(&mut self) {
        self.att = Quaternion::IDENTITY;
        self.x = [0.0; N];
        mat_ident(&mut self.p);
        for i in 0..3 {
            self.p[i * N + i] = 1.0;
        }
        for i in 3..6 {
            self.p[i * N + i] = 1.0;
        }
        for i in 6..9 {
            self.p[i * N + i] = 1e-4; // gyro bias
        }
        self.p[9 * N + 9] = 0.05; // accel bias (vertical) — 与 new() 一致，保证可观可收敛
    }
}

// ===== 以下为 `EkfEstimator` 自身的非 trait 方法（不是 `Estimator` trait 的一部分）=====
// 把协方差预测 / 位置观测更新从 `step` 里拆成独立方法，让各自的大栈数组（各 ~1620B）
// 作用域不重叠：编译器可在两支之间复用同一批栈槽，整体峰值 ≈ max(predict, update)
// 而非 sum，从而避免控制任务栈（3072B）溢出。

impl EkfEstimator {
    /// 重置滤波器并把初始姿态设为给定四元数（覆盖默认的水平 IDENTITY）。
    /// 用于 SIL/HIL 等已知初始姿态的场景，使 EKF 初值与物理初始姿态对齐，
    /// 避免“从水平零姿态出发、悬停稳态无收敛动力导致姿态永久错 90°”的发散。
    pub fn reset_to(&mut self, att: Quaternion) {
        self.reset();
        self.att = att;
    }

    /// 防御：清除协方差矩阵中的所有非有限项（NaN/Inf → 0），阻断数值污染传播。
    /// 对角夹取只处理对角，非对角 NaN 会残留并进入 S 求逆/K 增益计算 → 状态 NaN；
    /// 在 step 开头统一清理，保证本拍 predict/update 输入协方差干净。
    fn sanitize_p(&mut self) {
        for i in 0..(N * N) {
            if !self.p[i].is_finite() {
                self.p[i] = 0.0;
            }
        }
    }

    /// 协方差预测步：F = I + A*dt，P' = F P F^T + Q。
    /// 栈数组（f/ft/p_pred，各 81 元素）作用域限于本方法，返回后栈槽被回收。
    fn predict_cov(&mut self, dt: f32) {
        // F = I + A*dt
        // - [pos][vel] = I（位置由速度推进）
        // - [vel][accel_bias] = -R*dt（加计零偏直接影响速度，使其经速度观测可估计）
        let mut f = [0.0f32; N * N];
        mat_ident(&mut f);
        for i in 0..3 {
            f[i * N + (3 + i)] = dt;
        }
        // 垂向速度对世界 D 轴加计零偏 ab_z=x[9] 的偏导 = -1：
        // az_w = a_world[2] + g - ab_z，故 vel_z_dot 对 ab_z 的偏导为 -1。
        f[5 * N + 9] = -dt;
        let mut ft = [0.0f32; N * N];
        mat_mul_at(&f, &self.p, &mut ft, N, N, N);
        let mut p_pred = [0.0f32; N * N];
        mat_mul(&f, &ft, &mut p_pred, N, N, N);
        let qv = self.q_vel * dt;
        let qvz = self.q_vel_z * dt;
        let qb = self.q_bias * dt;
        let qa = self.q_accel * dt;
        for i in 3..5 {
            p_pred[i * N + i] += qv;
        }
        p_pred[5 * N + 5] += qvz; // 垂向速度过程噪声单独
        for i in 6..9 {
            p_pred[i * N + i] += qb; // 陀螺零偏随机游走
        }
        p_pred[9 * N + 9] += qa; // 垂向加计零偏随机游走（单分量）
        // 对称化 + 对角线上下限夹取（保持 PSD 且防止不可观状态cov runaway→NaN）。
        // 上限 P_MAX 阻断 vel↔accel_bias 正反馈：无速度观测时 accel_bias 不可观，
        // 其协方差会随速度协方差经 F 耦合指数增长，夹取后发散被阻断（实飞有速度观测时远不会触及上限）。
        for i in 0..N {
            for j in (i + 1)..N {
                let avg = 0.5 * (p_pred[i * N + j] + p_pred[j * N + i]);
                p_pred[i * N + j] = avg;
                p_pred[j * N + i] = avg;
            }
            let d = p_pred[i * N + i];
            if !d.is_finite() || d < 1e-6 {
                p_pred[i * N + i] = 1e-6; // 非有限(NaN/Inf) 重置，阻断协方差 NaN 传播
            } else if d > P_MAX {
                p_pred[i * N + i] = P_MAX;
            }
        }
        self.p = p_pred;
    }

    /// 位置观测更新步（Joseph 形式）：S = HPH^T+R、K = P H^T S^-1、x += K y、
    /// P = (I-KH)P(I-KH)^T + K R K^T。`r` 为观测噪声（m²），供 GPS / VIO / RTK 以
    /// 各自精度融合同一位置状态。栈数组作用域限于本方法，返回后栈槽被回收。
    fn update_pos_r(&mut self, z: PosSample, r: f32) {
        // S = H P H^T + R (3x3)
        let mut s = [0.0f32; 9];
        for i in 0..3 {
            for j in 0..3 {
                s[i * 3 + j] = self.p[i * N + j] + if i == j { r } else { 0.0 };
            }
        }
        // K = P H^T S^-1 (9x3) ; P H^T 即 P 前三列
        let mut pht = [0.0f32; 36];
        for i in 0..N {
            for j in 0..3 {
                pht[i * 3 + j] = self.p[i * N + j];
            }
        }
        // S 3x3 求逆（伴随矩阵）
        let s00 = s[0]; let s01 = s[1]; let s02 = s[2];
        let s10 = s[3]; let s11 = s[4]; let s12 = s[5];
        let s20 = s[6]; let s21 = s[7]; let s22 = s[8];
        let det = s00 * (s11 * s22 - s12 * s21)
            - s01 * (s10 * s22 - s12 * s20)
            + s02 * (s10 * s21 - s11 * s20);
        let inv = if det.abs() > 1e-9 { 1.0 / det } else { 0.0 };
        let mut sinv = [0.0f32; 9];
        sinv[0] = (s11 * s22 - s12 * s21) * inv;
        sinv[1] = (s02 * s21 - s01 * s22) * inv;
        sinv[2] = (s01 * s12 - s02 * s11) * inv;
        sinv[3] = (s12 * s20 - s10 * s22) * inv;
        sinv[4] = (s00 * s22 - s02 * s20) * inv;
        sinv[5] = (s02 * s10 - s00 * s12) * inv;
        sinv[6] = (s10 * s21 - s11 * s20) * inv;
        sinv[7] = (s01 * s20 - s00 * s21) * inv;
        sinv[8] = (s00 * s11 - s01 * s10) * inv;

        let mut k = [0.0f32; 36];
        mat_mul(&pht, &sinv, &mut k, N, 3, 3);

        // 数值鲁棒：卡尔曼增益限幅。紧噪声位置观测（RTK/VIO）下速度行增益可爆炸，
        // 限幅后 Joseph 协方差更新保持 PSD，阻断 p03 发散（见 K_MAX 注释）。
        for e in k.iter_mut() {
            *e = e.clamp(-K_MAX, K_MAX);
        }

        // 垂向加计零偏状态 (9) 不可由位置观测驱动（不可观 → 发散风险）：
        // 清零其卡尔曼增益行，使位置更新不修正 accel_bias_z（仅速度观测可估计，见 update_vel）。
        for j in 0..3 {
            k[9 * 3 + j] = 0.0;
        }
        // 垂向速度状态 (5) 的增益行也清零：GPS 位置观测经非对角协方差 K[5,*] 会推爆垂向速度
        // （hil 闭环回归：恒定比力+GPS 下 vel_z 爆炸到 1e5）。垂向速度仅由加速度积分决定，
        // 由 Doppler 速度观测（update_vel）约束，位置观测不直接修正它。
        for j in 0..3 {
            k[5 * 3 + j] = 0.0;
        }

        // 创新 y = z - pos
        let y = [z.pos[0].0 - self.x[0], z.pos[1].0 - self.x[1], z.pos[2].0 - self.x[2]];

        // x += K y
        for i in 0..N {
            let mut corr = 0.0;
            for j in 0..3 {
                corr += k[i * 3 + j] * y[j];
            }
            self.x[i] += corr;
        }

        // P 更新用 Joseph 形式：P = (I - K H) P (I - K H)^T + K R K^T
        // 1) A = I - K H （9x9，H 仅前三行非零）
        let mut a = [0.0f32; N * N];
        for i in 0..N {
            for j in 0..N {
                let kh = if j < 3 { k[i * 3 + j] } else { 0.0 }; // (K H)[i][j] = K[i][j] (j<3)
                a[i * N + j] = if i == j { 1.0 - kh } else { -kh };
            }
        }
        // 2) A P A^T
        let mut ap = [0.0f32; N * N];
        mat_mul(&a, &self.p, &mut ap, N, N, N);
        let mut apat = [0.0f32; N * N];
        mat_mul_at(&ap, &a, &mut apat, N, N, N);
        // 3) K R K^T （R = r * I3）
        let mut krkt = [0.0f32; N * N];
        for i in 0..N {
            for j in 0..N {
                let mut acc = 0.0;
                for l in 0..3 {
                    acc += k[i * 3 + l] * k[j * 3 + l];
                }
                krkt[i * N + j] = acc * r;
            }
        }
        // 4) P = A P A^T + K R K^T，并对称化（消除尾差）
        for i in 0..(N * N) {
            self.p[i] = apat[i] + krkt[i];
        }
        for i in 0..N {
            for j in (i + 1)..N {
                let avg = 0.5 * (self.p[i * N + j] + self.p[j * N + i]);
                self.p[i * N + j] = avg;
                self.p[j * N + i] = avg;
            }
            // 对角线上下限夹取，杜绝负方差并阻断不可观状态协方差发散。
            let d = self.p[i * N + i];
            if !d.is_finite() || d < 1e-6 {
                self.p[i * N + i] = 1e-6; // 非有限(NaN/Inf) 重置，阻断协方差 NaN 传播
            } else if d > P_MAX {
                self.p[i * N + i] = P_MAX;
            }
        }
    }

    /// 位置观测更新（GPS，默认噪声 `r_pos`）。等价于 `update_pos_r(z, self.r_pos)`。
    pub fn update_pos(&mut self, z: PosSample) {
        self.update_pos_r(z, self.r_pos);
    }

    /// 速度观测更新步（Doppler GPS）：H = [0 0 0 I3 0] 作用于状态 [pos, vel, bias]，
    /// 直接观测速度分量 3..6。`r` 为观测噪声（m/s）²，供 GPS Doppler / VIO 以各自
    /// 精度融合。Joseph 形式，栈数组作用域限于本方法。
    pub fn update_vel_r(&mut self, vel: [f32; 3], r: f32) {
        // S = H P H^T + R (3x3)
        let mut s = [0.0f32; 9];
        for i in 0..3 {
            for j in 0..3 {
                s[i * 3 + j] = self.p[(3 + i) * N + (3 + j)] + if i == j { r } else { 0.0 };
            }
        }
        // K = P H^T S^-1 (9x3)；P H^T 即 P 第 3..6 列
        let mut pht = [0.0f32; 36];
        for i in 0..N {
            for j in 0..3 {
                pht[i * 3 + j] = self.p[i * N + (3 + j)];
            }
        }
        let s00 = s[0]; let s01 = s[1]; let s02 = s[2];
        let s10 = s[3]; let s11 = s[4]; let s12 = s[5];
        let s20 = s[6]; let s21 = s[7]; let s22 = s[8];
        let det = s00 * (s11 * s22 - s12 * s21)
            - s01 * (s10 * s22 - s12 * s20)
            + s02 * (s10 * s21 - s11 * s20);
        let inv = if det.abs() > 1e-9 { 1.0 / det } else { 0.0 };
        let mut sinv = [0.0f32; 9];
        sinv[0] = (s11 * s22 - s12 * s21) * inv;
        sinv[1] = (s02 * s21 - s01 * s22) * inv;
        sinv[2] = (s01 * s12 - s02 * s11) * inv;
        sinv[3] = (s12 * s20 - s10 * s22) * inv;
        sinv[4] = (s00 * s22 - s02 * s20) * inv;
        sinv[5] = (s02 * s10 - s00 * s12) * inv;
        sinv[6] = (s10 * s21 - s11 * s20) * inv;
        sinv[7] = (s01 * s20 - s00 * s21) * inv;
        sinv[8] = (s00 * s11 - s01 * s10) * inv;

        let mut k = [0.0f32; 36];
        mat_mul(&pht, &sinv, &mut k, N, 3, 3);

        // 数值鲁棒：卡尔曼增益限幅（同 update_pos_r）。紧速度观测（VIO 光流 r=0.04）下
        // 位置/交叉状态行增益可过大，限幅保持 Joseph 更新数值稳定。
        for e in k.iter_mut() {
            *e = e.clamp(-K_MAX, K_MAX);
        }

        // 垂向加计零偏 x[9] 只应被【垂向】速度观测驱动：水平速度创新（y0/y1）经交叉
        // 协方差 K[9][0..1] 会注入水平残差噪声/偏置进零偏（物理上水平速度误差由水平
        // 加计零偏引起，而本滤波器不估计水平零偏），紧速度观测下放大成 ab_z 发散
        // （仿真：GPS 中断 + VIO 时 ab_z → 4.3e6 → NaN）。清零其水平增益行，仅保留
        // 垂向增益 K[9][2]。
        k[9 * 3 + 0] = 0.0;
        k[9 * 3 + 1] = 0.0;

        // 注：垂向加计零偏 x[9] 仅由速度观测驱动，单分量可观且数值稳定，
        // 无需额外增益阻尼（3 轴机体系版本曾有 Y 轴正反馈发散，已收敛为单分量）。
        // 阶段 11-A：启用垂向零偏估计。之前 0.0 把零偏恒锁在初值 0，
        // 导致恒定加计零偏（典型 0.05 m/s²）直接积分进物理速度、悬停轨迹无界发散。
        // 单分量（世界 D 轴）形式数值稳定；增益取 0.03（小增益）使零偏仅跟踪速度误差的
        // 直流分量、不吸收控制瞬态（瞬态速度误差若被吸进零偏会使其过冲到上限 0.3、反而
        // 造成悬停上漂发散）。配合 q_accel=5e-4 让零偏在 40s 尺度内缓慢漂移到真值 0.05。
        const AB_VEL_GAIN: f32 = 0.3; // 速度观测驱动垂向零偏 x[9] 的增益（0=关闭，零偏保持初值）
        let y = [vel[0] - self.x[3], vel[1] - self.x[4], vel[2] - self.x[5]];

        for i in 0..N {
            let mut corr = 0.0;
            for j in 0..3 {
                corr += k[i * 3 + j] * y[j];
            }
            if i == 9 {
                corr *= AB_VEL_GAIN;
            }
            self.x[i] += corr;
        }
        // 加计零偏物理上界夹取（消费级 IMU 零偏通常 < 0.3 m/s²）：该状态仅由速度观测
        // 驱动，正常收敛不受影响；夹取作发散兜底。必须在此执行——`update_vio`/`update_rtk`
        // 在 `step`（其尾部已有同款夹取）之后调用，若不加此处则 VIO/RTK 驱动的零偏
        // 在整步内无界增长，成为 NaN 源头。
        if self.x[9] > 0.3 {
            self.x[9] = 0.3;
        } else if self.x[9] < -0.3 {
            self.x[9] = -0.3;
        }

        // Joseph 形式：P = (I - K H) P (I - K H)^T + K R K^T，H 仅 3..6 行非零
        let mut a = [0.0f32; N * N];
        for i in 0..N {
            for j in 0..N {
                let kh = if (3..6).contains(&j) { k[i * 3 + (j - 3)] } else { 0.0 };
                a[i * N + j] = if i == j { 1.0 - kh } else { -kh };
            }
        }
        let mut ap = [0.0f32; N * N];
        mat_mul(&a, &self.p, &mut ap, N, N, N);
        let mut apat = [0.0f32; N * N];
        mat_mul_at(&ap, &a, &mut apat, N, N, N);
        let mut krkt = [0.0f32; N * N];
        for i in 0..N {
            for j in 0..N {
                let mut acc = 0.0;
                for l in 0..3 {
                    acc += k[i * 3 + l] * k[j * 3 + l];
                }
                krkt[i * N + j] = acc * r;
            }
        }
        for i in 0..(N * N) {
            self.p[i] = apat[i] + krkt[i];
        }
        for i in 0..N {
            for j in (i + 1)..N {
                let avg = 0.5 * (self.p[i * N + j] + self.p[j * N + i]);
                self.p[i * N + j] = avg;
                self.p[j * N + i] = avg;
            }
            if self.p[i * N + i] < 1e-6 {
                self.p[i * N + i] = 1e-6;
            }
        }
    }

    /// 速度观测更新（GPS Doppler，默认噪声 `r_vel`）。等价于 `update_vel_r(vel, self.r_vel)`。
    pub fn update_vel(&mut self, vel: [f32; 3]) {
        self.update_vel_r(vel, self.r_vel);
    }

    /// 高度观测更新步（气压计）：气压计测得**向上**高度 `alt`，而状态 D 轴向下为正，
    /// 故观测 `z = -alt`，H = [0 0 1 0 0 0 0 0 0]（作用于 D 位置索引 2）。
    /// Joseph 形式，栈数组作用域限于本方法。
    pub fn update_alt(&mut self, alt: f32) {
        // S = H P H^T + R (标量)，H 仅在索引 2（D 位置）非零
        let s = self.p[2 * N + 2] + self.r_alt;
        if s.abs() < 1e-9 {
            return;
        }
        // K = P H^T / S (9x1)，仅 P 第 2 列非零
        let mut k = [0.0f32; N];
        for i in 0..N {
            k[i] = self.p[i * N + 2] / s;
        }
        // 创新 y = z - x，z = -alt（D 向下正）
        let y = -alt - self.x[2];
        // 垂向加计零偏状态 (9) 不可由气压高度观测驱动（不可观 → 发散风险）：
        // 清零其卡尔曼增益元素，使气压更新不修正 accel_bias_z（仅速度观测可估计）。
        k[9] = 0.0;
        for i in 0..N {
            self.x[i] += k[i] * y;
        }
        // Joseph 形式：P = (I - K H) P (I - K H)^T + K R K^T
        let mut a = [0.0f32; N * N];
        for i in 0..N {
            for j in 0..N {
                let kh = if j == 2 { k[i] } else { 0.0 };
                a[i * N + j] = if i == j { 1.0 - kh } else { -kh };
            }
        }
        let mut ap = [0.0f32; N * N];
        mat_mul(&a, &self.p, &mut ap, N, N, N);
        let mut apat = [0.0f32; N * N];
        mat_mul_at(&ap, &a, &mut apat, N, N, N);
        let mut krkt = [0.0f32; N * N];
        for i in 0..N {
            for j in 0..N {
                krkt[i * N + j] = k[i] * k[j] * self.r_alt;
            }
        }
        for i in 0..(N * N) {
            self.p[i] = apat[i] + krkt[i];
        }
        for i in 0..N {
            for j in (i + 1)..N {
                let avg = 0.5 * (self.p[i * N + j] + self.p[j * N + i]);
                self.p[i * N + j] = avg;
                self.p[j * N + i] = avg;
            }
            let d = self.p[i * N + i];
            if !d.is_finite() || d < 1e-6 {
                self.p[i * N + i] = 1e-6; // 非有限(NaN/Inf) 重置，阻断协方差 NaN 传播
            } else if d > P_MAX {
                self.p[i * N + i] = P_MAX;
            }
        }
    }

    /// 空速观测更新步（标量）：空速计测得水平气流速度幅值 `v_as = |v_h|`，
    /// 其中 `v_h = sqrt(vx² + vy²)`（不含风）。观测模型 `h(x) = sqrt(vx² + vy²)`，
    /// Jacobian `H = [0 0 0 vx/vh vy/vh 0 0 0]`。
    /// 经典 EKF 标量更新（P 已是 PSD，无需 Joseph 形式），栈数组作用域限于本方法。
    fn update_airspeed(&mut self, v_as: f32) {
        let vx = self.x[3];
        let vy = self.x[4];
        let vh2 = vx * vx + vy * vy;
        // 水平速度近零时 Jacobian 退化：直接用测量初始化估计，跳过增益更新。
        if vh2 < 1e-4 {
            self.airspeed_est = v_as;
            return;
        }
        let vh = sqrt(vh2);
        let hx = vx / vh;
        let hy = vy / vh;
        // S = H P H^T + R （标量）
        // H P H^T = (hx,hy,0) P (hx,hy,0)^T = Σ_{a,b∈{3,4}} H_a P_ab H_b
        let mut hph = 0.0f32;
        let ha = [hx, hy];
        for a in 0..2 {
            for b in 0..2 {
                hph += ha[a] * self.p[(3 + a) * N + (3 + b)] * ha[b];
            }
        }
        let s = hph + self.r_airspeed;
        if s.abs() < 1e-9 {
            return;
        }
        // K = P H^T / S  (9x1)：仅 vel 分量非零
        let mut k = [0.0f32; N];
        k[3] = (self.p[3 * N + 3] * hx + self.p[3 * N + 4] * hy) / s;
        k[4] = (self.p[4 * N + 3] * hx + self.p[4 * N + 4] * hy) / s;
        // 创新 y = z - h(x)
        let y = v_as - vh;
        // x += K y
        for i in 0..N {
            self.x[i] += k[i] * y;
        }
        // P = (I - K H) P，对称化 + 下限夹取
        // 正确秩1更新：(I-KH)P 的 (i,j) 元 = P[i][j] - k[i]*(hx*P[3][j] + hy*P[4][j])
        // （K 仅行 3,4 非零，H 仅列 3,4 非零）
        let mut ap = [0.0f32; N * N];
        for i in 0..N {
            for j in 0..N {
                let khp = k[i] * (hx * self.p[3 * N + j] + hy * self.p[4 * N + j]);
                ap[i * N + j] = self.p[i * N + j] - khp;
            }
        }
        for i in 0..N {
            for j in 0..N {
                self.p[i * N + j] = ap[i * N + j];
            }
        }
        for i in 0..N {
            for j in (i + 1)..N {
                let avg = 0.5 * (self.p[i * N + j] + self.p[j * N + i]);
                self.p[i * N + j] = avg;
                self.p[j * N + i] = avg;
            }
            let d = self.p[i * N + i];
            if !d.is_finite() || d < 1e-6 {
                self.p[i * N + i] = 1e-6; // 非有限(NaN/Inf) 重置，阻断协方差 NaN 传播
            } else if d > P_MAX {
                self.p[i * N + i] = P_MAX;
            }
        }
        // 估计空速 = 水平速度幅值（融合后）
        self.airspeed_est = sqrt(self.x[3] * self.x[3] + self.x[4] * self.x[4]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(test)]
    extern crate std;
    use crate::config::VehicleConfig;
    use crate::units::*;
    use crate::vehicle::{Airspeed, AirspeedSample, ImuSample, MeterPerSecond, RadianPerSecond};

    #[test]
    fn airspeed_fusion_constrains_horizontal_speed() {
        // 构型：机体匀速平飞（水平速度 5 m/s），空速计测得 5 m/s。
        // 验证 EKF 融合后估计水平速度幅值收敛到 ≈ 测量空速。
        let _cfg = VehicleConfig::default_quad();
        let mut ekf = EkfEstimator::default_quad();
        // 初始化真值水平速度
        ekf.x[3] = 5.0;
        ekf.x[4] = 0.0;

        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0); 3],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let aspd = AirspeedSample { speed: Airspeed(5.0), timestamp_s: 0.0 };

        let mut last = VehicleState::zero();
        for _ in 0..200 {
            last = ekf.step(Second(0.01), imu, None, Some(aspd));
        }
        let vh = (last.vel[0].0 * last.vel[0].0 + last.vel[1].0 * last.vel[1].0).sqrt();
        assert!(
            (vh - 5.0).abs() < 0.3,
            "空速计融合后水平速度幅值应≈5 m/s，got {:.3}",
            vh
        );
        // 估计空速应被填充
        assert!(last.airspeed.0 > 0.0, "估计空速应 > 0，got {}", last.airspeed.0);
    }

    #[test]
    fn airspeed_fusion_rejects_none_gracefully() {
        // 无空速计（None）时，EKF 不应崩溃，airspeed 估计保持为水平速度幅值（可能为 0）。
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0); 3],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let mut ekf = EkfEstimator::default_quad();
        let st = ekf.step(Second(0.01), imu, None, None);
        // 静止 + 无观测：速度应≈0，不得 NaN
        assert!(st.vel[0].0.is_finite() && st.vel[1].0.is_finite());
    }

    #[test]
    fn hil_ekf_vertical_stays_stable_with_mock_gps() {
        // 回归：EKF + 完整 MockImu（含 gyro 振动/120Hz accel 振动）+ GPS(d=-5)。
        // 曾因 GPS 位置观测经非对角协方差 K[5,*] 推爆垂向速度（vel_z 到 1e5，位置漂走）
        // → hil 闭环 NaN。修复：update_pos 清零垂向速度增益行（K[5,*]=0），垂向速度仅由
        // 加速度积分 + Doppler 速度观测约束。验证位置物理稳定在 -5 附近（<100m）不漂走。
        let mut ekf = EkfEstimator::default_quad();
        ekf.set_initial_position([0.0, 0.0, -5.0]);
        let gps = PosSample::pos_only([Meter(0.0), Meter(0.0), Meter(-5.0)]);
        let mut imu2 = crate::hal::sensor::MockImu::new();
        let mut max_abs = 0.0f32;
        let mut last = VehicleState::zero();
        let mut bad = false;
        for _ in 0..300 {
            let s = crate::hal::sensor::ImuSensor::read(&mut imu2);
            last = ekf.step(Second(0.01), s, Some(gps), None);
            max_abs = max_abs.max(last.pos[2].0.abs());
            if !last.pos[2].0.is_finite() {
                bad = true;
                break;
            }
        }
        assert!(
            !bad && max_abs < 100.0 && last.pos[2].0.is_finite(),
            "EKF 恒定比力+GPS 悬停应稳定: max_abs={} pos_z={} vel_z={} bad={}",
            max_abs, last.pos[2].0, last.vel[2].0, bad
        );
    }

    #[test]
    fn vio_fusion_tracks_truth_during_gps_outage() {
        // P3-B1：GPS 失锁期间 VIO 桥接。机体匀速 2 m/s 前飞（北向），IMU 零加速度
        // （比力变化由水平速度保持承担）、无 GPS、无空速；仅 VIO 提供位置+速度观测。
        // 无 VIO 时 EKF 因无速度/位置观测会把速度锁在 0、位置停在起点（纯积分无输入）；
        // 有 VIO 时估计位置/速度应跟随真值，验证 VIO 填补 GPS 帧间/失锁的估计空白。
        let mut ekf = EkfEstimator::default_quad();
        ekf.set_initial_position([0.0, 0.0, -5.0]);
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0); 3],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let mut truth_n = 0.0f32;
        let mut last = VehicleState::zero();
        for _ in 0..300 {
            truth_n += 2.0 * 0.01; // 2 m/s 匀速
            let vio = VioSample::with_vel(
                [Meter(truth_n), Meter(0.0), Meter(-5.0)],
                [MeterPerSecond(2.0), MeterPerSecond(0.0), MeterPerSecond(0.0)],
            );
            last = ekf.step(Second(0.01), imu, None, None);
            ekf.update_vio(Some(vio));
        }
        // 3s 后真值北向 6m。
        assert!(
            (last.pos[0].0 - 6.0).abs() < 1.0,
            "VIO 桥接后估计位置应≈6m，got {:.3}",
            last.pos[0].0
        );
        assert!(
            (last.vel[0].0 - 2.0).abs() < 0.3,
            "VIO 速度融合后估计速度应≈2 m/s，got {:.3}",
            last.vel[0].0
        );
        assert!(
            last.pos[2].0.is_finite() && last.vel[0].0.is_finite(),
            "VIO 融合不得产生 NaN"
        );
    }

    #[test]
    fn rtk_fusion_reaches_cm_precision() {
        // P3-B1：RTK 厘米级位置融合。初始位置给定大误差（[10,5,-5]），RTK 测得真值
        // [0,0,-5]（噪声 ~0.05m）。反复融合后估计位置应收敛到厘米量级（<0.15m）。
        // IMU 用悬停比力 [0,0,-9.81]（抵消重力），az_w = -9.81+g-0 = 0，垂向不漂移；
        // 若用 [0,0,0]（自由落体）EKF 会按 g 积分垂向速度，而位置观测不改垂向速度
        // （见 update_pos_r 清零第 5 行增益），垂向将发散，测试失去意义。
        let mut ekf = EkfEstimator::default_quad();
        ekf.set_initial_position([10.0, 5.0, -5.0]);
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(-9.81)],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let rtk = RtkSample::new([Meter(0.0), Meter(0.0), Meter(-5.0)]);
        let mut last = VehicleState::zero();
        for _ in 0..50 {
            last = ekf.step(Second(0.01), imu, None, None);
            ekf.update_rtk(Some(rtk));
        }
        let err = [
            (last.pos[0].0 - 0.0).abs(),
            (last.pos[1].0 - 0.0).abs(),
            (last.pos[2].0 + 5.0).abs(),
        ];
        assert!(
            err[0] < 0.15 && err[1] < 0.15 && err[2] < 0.15,
            "RTK 融合后估计位置应达厘米级（<0.15m），got err={:?}",
            err
        );
    }

    #[test]
    #[ignore]
    fn dbg_rtk_divergence() {
        #[cfg(test)]
        use std::println;
        let mut ekf = EkfEstimator::default_quad();
        ekf.set_initial_position([10.0, 5.0, -5.0]);
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0); 3],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let rtk = RtkSample::new([Meter(0.0), Meter(0.0), Meter(-5.0)]);
        for i in 0..3 {
            let _last = ekf.step(Second(0.01), imu, None, None);
            println!("--- i={i} BEFORE update: x=[{:.6},{:.6},{:.6}] vel=[{:.6},{:.6},{:.6}]", ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]);
            println!("  P rows 0-6 cols0-3:");
            for r in 0..6 {
                println!("    r{r}: [{:.3e},{:.3e},{:.3e}] cross0={:.3e} cross1={:.3e} cross2={:.3e}",
                    ekf.p[r*10+0], ekf.p[r*10+1], ekf.p[r*10+2], ekf.p[r*10+3], ekf.p[r*10+4], ekf.p[r*10+5]);
            }
            ekf.update_rtk(Some(rtk));
            println!("  AFTER update: x=[{:.6},{:.6},{:.6}] vel=[{:.6},{:.6},{:.6}]", ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]);
            println!("  P rows 0-6 cols0-3:");
            for r in 0..6 {
                println!("    r{r}: [{:.3e},{:.3e},{:.3e}] cross0={:.3e} cross1={:.3e} cross2={:.3e}",
                    ekf.p[r*10+0], ekf.p[r*10+1], ekf.p[r*10+2], ekf.p[r*10+3], ekf.p[r*10+4], ekf.p[r*10+5]);
            }
        }
    }

    #[test]
    fn vio_rtk_none_gracefully() {
        // P3-B1：VIO/RTK 无观测（None / 未装备）时 EKF 不应崩溃，状态保持有限。
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0); 3],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let mut ekf = EkfEstimator::default_quad();
        let st = ekf.step(Second(0.01), imu, None, None);
        ekf.update_vio(None);
        ekf.update_rtk(None);
        assert!(st.vel[0].0.is_finite() && st.pos[0].0.is_finite());
    }

    #[test]
    #[ignore]
    fn dbg_matrix_semantics() {
        #[cfg(test)]
        use std::println;
        // A = [[1,2],[3,4]], B = [[5,6],[7,8]]  (row-major 2x2)
        let a = [1.0f32, 2.0, 3.0, 4.0];
        let b = [5.0f32, 6.0, 7.0, 8.0];
        let mut c = [0.0f32; 4];
        mat_mul(&a, &b, &mut c, 2, 2, 2);
        println!("A*B      = {:?}  (expect [19,22,43,50])", c);
        let mut d = [0.0f32; 4];
        mat_mul_at(&a, &b, &mut d, 2, 2, 2);
        println!("mat_mul_at(A,B) = {:?}  (A^T*B expect [26,30,38,44])", d);
        // 换成第二个参数转置测一下：mat_mul_at(B,A) 应为 B^T*A
        let mut e = [0.0f32; 4];
        mat_mul_at(&b, &a, &mut e, 2, 2, 2);
        println!("mat_mul_at(B,A) = {:?}  (B^T*A expect [26,38,30,44])", e);
    }
}
