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
    mag_alpha: f32,     // 磁力计航向锚定强度（0 = 不锚定 yaw；纯陀螺积分 yaw 会漂移）
    mag_ref: [f32; 2],  // 世界系水平参考地磁方向（单位向量）：默认 (1,0)=地理北；
                        // 有磁偏角时 set_mag_declination 旋转该参考 → 磁航向转地理航向
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
            mag_alpha: 0.05,  // 微弱航向锚定：yaw 误差每拍吸收 2.5%（0.05*0.5）
            mag_ref: [1.0, 0.0], // 默认磁北=地理北（无偏角）
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
        // `att_alpha=0.02` 开启微弱重力锚定。历史教训（无门控时启用 0.02 → PID/INDI/LQR
        // 全飞到 +19m、att_err 300°+）已由**比力幅值门控**解决：仅当 |a|∈[0.5g, 2.5g]
        // 时锚定增益按 w 线性缩放，自由落体/落地碰撞尖峰自动关闭锚定，避免姿态被
        // 平移加速度分量错误引导。悬停/平稳飞行时持续锚定 roll/pitch，抑制纯陀螺积分漂移。
        // 垂向零偏 x[9] 仅由速度观测弱驱动（AB_VEL_GAIN 阻尼）。
        // att_alpha=0.02：微弱重力锚定 roll/pitch（悬停/平稳飞行时抑制纯陀螺
        // 积分漂移），与磁力计航向锚定（mag_alpha，yaw）互补成完整姿态锚定。
        // 早期 TEMP-EXPERIMENT 曾置 0（纯陀螺积分对照）——比力幅值门控 + 方向
        // 一致性门控（394c7e3 新增，夹角 >25.8° 关闭）已解决无门控时 0.02 的
        // 发散问题；394c7e3 为 SIL gps_bias_step 判据把本值一并置 0（纯陀螺
        // 积分），代价是正常悬停 roll/pitch 估计漂移 → HIL 闭环 4.8s 姿态发散
        // （mcu_p=+0.10 vs 物理 -0.54，推力饱和 0/1 边界翻滚）。门控本身已防
        // 平移误锚定，此处恢复 0.02（HIL/SIL 双回归守护）。
        Self::new(0.02, 0.05, 0.05, 1e-5, 5e-4, 0.5, 0.3, 0.3)
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
    /// 位置协方差压到 ~0.01m²（视为已收敛，避免初始大协方差经 F 矩阵耦合膨胀）。
    pub fn set_initial_position(&mut self, ned: [f32; 3]) {
        self.x[0] = ned[0];
        self.x[1] = ned[1];
        self.x[2] = ned[2];
        for i in 0..3 {
            self.p[i * N + i] = 0.01;
        }
    }

    /// HIL：设置 EKF 初始姿态四元数。
    ///
    /// 默认构造 att=IDENTITY（0°），若物理引擎起始姿态非 0°（或机头 yaw 非 0），
    /// 首拍即带姿态误差；陀螺零偏未收敛时会经积分累积放大（历史教训：pitch 恒定 90°）。
    /// 启动前用物理引擎真值姿态初始化，使首拍姿态误差≈0。
    pub fn set_initial_attitude(&mut self, q: Quaternion) {
        self.att = q.normalize();
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
        // 防御：IMU 输入非有限（HIL 注入异常/总线噪声）直接放弃本拍积分，返回上一状态。
        // 否则 att.integrate(NaN) 会把姿态四元数直接污染为 NaN（历史教训：电机指令 NaN
        // 即由状态/姿态 NaN 传播而来），进而经位置预测/观测更新污染整个状态向量。
        crate::perf::probe(8); // EKF::step 入口
        let gyro_finite = imu.gyro.iter().all(|g| g.0.is_finite());
        let accel_finite = imu.accel.iter().all(|a| a.0.is_finite());
        if !gyro_finite || !accel_finite {
            return self.state();
        }
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
            // 比力幅值门控：仅当 |a| 接近 g 时才信任加速度计的「重力方向」参考。
            //   - 自由落体/失重 |a|≈0：比力方向无意义，若照常锚定会把姿态拖向随机方向
            //     （历史教训：att_alpha=0.02 未加门控 → PID/INDI/LQR 全飞到 +19m、att_err 300°+）。
            //   - 高机动/落地碰撞尖峰 |a|≫g：比力含平移加速度分量，同样不代表重力方向。
            //   门控权重 w：|a|=g 时 w=1，向边界（0.5g / 2.5g）线性衰减到 0，增益按 w 缩放。
            if an > 1e-3 {
                let ratio = an / g;
                let w = if ratio < 0.5 || ratio > 2.5 {
                    0.0
                } else if ratio < 1.0 {
                    (ratio - 0.5) / 0.5
                } else {
                    (2.5 - ratio) / 1.5
                };
                let w = w.clamp(0.0, 1.0);
                let down_body = crate::vehicle::rotate_vec_by_quat_inverse(self.att, [0.0, 0.0, g]);
                // 方向一致性门控：比力反方向（-a，估计的重力参考）与当前估计重力方向
                // down_body 的夹角。静止/匀速悬停时比力≈纯重力（夹角≈0，仅陀螺漂移
                // 引入微小偏差）→ 全锚定；平移机动/倾角飞行时比力含平移加速度分量，
                // 方向偏离重力（夹角 >~26°）→ 关闭锚定。
                // 仅靠幅值门控（|a|≈g）挡不住匀速平移：幅值≈g 但方向不代表重力，
                // 锚定会把姿态拖向错误方向——GPS 偏置故障场景（SIL sensor_fault
                // gps_bias_step）实测 tilt 81°（应 <45°），加方向门控后恢复。
                let n_inv = [-a[0] / an, -a[1] / an, -a[2] / an];
                let cos_t = (down_body[0] * n_inv[0] + down_body[1] * n_inv[1] + down_body[2] * n_inv[2])
                    .clamp(-1.0, 1.0);
                // cos 0.90 ≈ 25.8°：夹角 <25.8° 线性加权，>25.8° 完全关闭。
                let w_align = if cos_t < 0.90 {
                    0.0
                } else {
                    ((cos_t - 0.90) / 0.10).clamp(0.0, 1.0)
                };
                // 陀螺幅值门控：机动/协调转弯时关闭重力锚定。
                // 协调转弯的向心加速度由 roll 平衡 → 比力方向≈竖直（幅值≈g），
                // 幅值/方向门控都无法区分"水平加速"与"重力"——锚定会把协调转弯
                // 的 roll 错误拉向 0（虚拟外设实测稳态 2.2° vs 真值 27°）。|omega|
                // 大 → 关闭锚定（纯陀螺积分跟踪机动姿态）；悬停/平稳（|omega|≈
                // 陀螺噪声量级）→ 全锚定（抑制纯陀螺积分漂移）。
                let om = sqrt(
                    imu.gyro[0].0 * imu.gyro[0].0
                        + imu.gyro[1].0 * imu.gyro[1].0
                        + imu.gyro[2].0 * imu.gyro[2].0,
                );
                let w_gyro = if om < 0.25 {
                    1.0
                } else if om < 0.6 {
                    (0.6 - om) / 0.35
                } else {
                    0.0
                };
                let k = self.att_alpha * 0.5 * w * w_align * w_gyro;
                // 把估计重力向量 down_body 锚定到【真实重力方向】，即比力的反方向 (-a)。
                // 修正轴 = down_body × (-a/an)：n 为垂直于二者的旋转轴，
                // 右乘（机体系）dq 使 down_body 旋转向 -a，姿态向水平收敛。
                //   （符号教训：若用 a_hat × down_body 方向反 → 锚定放大倾斜而非收敛，
                //   姿态被逐渐推向倒扣 180°，且倒扣处叉积恰为零 → 冻结在 r=-179.9°。）
                let ax = (down_body[1] * (a[2] / an) - down_body[2] * (a[1] / an)) * k;
                let ay = (down_body[2] * (a[0] / an) - down_body[0] * (a[2] / an)) * k;
                let az = (down_body[0] * (a[1] / an) - down_body[1] * (a[0] / an)) * k;
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
        crate::perf::probe(9); // 姿态积分 + 重力锚定完成
        self.predict_cov(dt);
        crate::perf::probe(10); // 协方差预测完成

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

        crate::perf::probe(11); // 观测更新（GPS 位置/速度）完成

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

    // HIL/共享单步：转发到同名的固有方法（trait 默认 no-op，EKF 实际生效）。
    fn set_initial_attitude(&mut self, q: Quaternion) {
        EkfEstimator::set_initial_attitude(self, q);
    }

    fn set_initial_position(&mut self, ned: [f32; 3]) {
        EkfEstimator::set_initial_position(self, ned);
    }

    fn update_alt(&mut self, alt: f32) {
        EkfEstimator::update_alt(self, alt);
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
        // F 只有 4 个非单位元（其余为单位阵）：
        //   F[0][3] = F[1][4] = F[2][5] = dt（pos ← vel）
        //   F[5][9] = -dt               （vel_z ← accel_bias_z；
        //                                 az_w = a_world[2] + g - ab_z ⇒ ∂vel_z/∂ab_z = -1）
        // 因此 Fᵀ·P 只改 4 行、F·(FᵀP) 只改 4 行，用稠密 10³ 乘是白算 25 倍。
        // 下面按稀疏结构展开，结果与 `mat_mul_at(f,p)` / `mat_mul(f,ft)` 等价。
        let p = &self.p;
        let mut ft = [0.0f32; N * N];
        ft.copy_from_slice(p);
        // ft = Fᵀ P：row3 += dt·row0, row4 += dt·row1, row5 += dt·row2, row9 -= dt·row5
        for (dst, src, coef) in [(3usize, 0usize, dt), (4, 1, dt), (5, 2, dt), (9, 5, -dt)] {
            for j in 0..N {
                ft[dst * N + j] += coef * p[src * N + j];
            }
        }
        // p_pred = F ft：row0 += dt·row3, row1 += dt·row4, row2 += dt·row5, row5 -= dt·row9
        let mut p_pred = [0.0f32; N * N];
        p_pred.copy_from_slice(&ft);
        for (dst, src, coef) in [(0usize, 3usize, dt), (1, 4, dt), (2, 5, dt), (5, 9, -dt)] {
            for j in 0..N {
                p_pred[dst * N + j] += coef * ft[src * N + j];
            }
        }
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

    /// 协方差更新步（与原实现的代数**逐项一致**，只是利用观测矩阵的稀疏结构省掉零元乘法）。
    ///
    /// 原实现写的是：
    /// ```text
    ///   ap   = A * P                    // A = I - K H
    ///   apat = mat_mul_at(ap, A)        // 注释写 "A P A^T"，但 mat_mul_at(x,y)=x^T*y，
    ///                                   // 实际算的是 ap^T * A = P A^T A
    ///   P    = apat + K R K^T ;  再对称化
    /// ```
    /// 本方法**复现这个代数**（含对称化），以保证本次改动只带来速度、不改变滤波行为。
    /// 注：`A = I-KH` 不对称，故 `P A^T A != A P A^T`；教科书 Joseph 形式应为后者。
    /// 该差异是原实现的潜在缺陷，另行评估（见 `P A^T A` 与 `A P A^T` 的对照测试）。
    ///
    /// 稀疏化依据：`H` 是"只在状态列 `c..c+ncols` 上取单位观测"的窄观测矩阵，
    /// 故 `A` 只在列 `c..c+ncols` 偏离单位阵，`ap` 与 `apat` 都只需 O(N²·ncols)：
    /// ```text
    ///   ap[i][j]    = P[i][j] - Σ_l K[i][l]·P[c+l][j]
    ///   apat[i][j]  = ap[j][i] - [j∈观测列]·g[i][j-c] ,  g = ap^T K
    /// ```
    ///
    /// `k` 为 N×ncols 行主序卡尔曼增益（已限幅/已清零不可观行）。
    /// `nan_reset`/`clamp_max` 保留各调用点原有的对角处理策略
    /// （`update_vel_r` 只做下限，位置/气压更新额外夹 P_MAX）。
    fn joseph_update_cov(
        &mut self,
        k: &[f32],
        ncols: usize,
        c: usize,
        r: f32,
        nan_reset: bool,
        clamp_max: bool,
    ) {
        let p = &self.p;
        // 1) ap = (I - K H) P
        let mut ap = [0.0f32; N * N];
        for i in 0..N {
            let ki = &k[i * ncols..i * ncols + ncols];
            for j in 0..N {
                let mut acc = p[i * N + j];
                for (l, kl) in ki.iter().enumerate() {
                    acc -= kl * p[(c + l) * N + j];
                }
                ap[i * N + j] = acc;
            }
        }
        // 2) g = ap^T K  （N×ncols）
        let mut g = [[0.0f32; 3]; N];
        for i in 0..N {
            for l in 0..ncols {
                let mut acc = 0.0f32;
                for kk in 0..N {
                    acc += ap[kk * N + i] * k[kk * ncols + l];
                }
                g[i][l] = acc;
            }
        }
        // 3) P = apat + K R K^T，apat[i][j] = ap[j][i] - [j 在观测列]·g[i][j-c]
        for i in 0..N {
            let ki = &k[i * ncols..i * ncols + ncols];
            for j in 0..N {
                let mut acc = ap[j * N + i];
                if j >= c && j < c + ncols {
                    acc -= g[i][j - c];
                }
                let kj = &k[j * ncols..j * ncols + ncols];
                for (l, kl) in ki.iter().enumerate() {
                    acc += r * kl * kj[l];
                }
                self.p[i * N + j] = acc;
            }
        }
        // 4) 对称化（消除尾差）+ 对角线处理
        for i in 0..N {
            for j in (i + 1)..N {
                let avg = 0.5 * (self.p[i * N + j] + self.p[j * N + i]);
                self.p[i * N + j] = avg;
                self.p[j * N + i] = avg;
            }
            let d = self.p[i * N + i];
            if nan_reset && !d.is_finite() {
                self.p[i * N + i] = 1e-6; // 非有限(NaN/Inf) 重置，阻断协方差 NaN 传播
            } else if d < 1e-6 {
                self.p[i * N + i] = 1e-6;
            } else if clamp_max && d > P_MAX {
                self.p[i * N + i] = P_MAX;
            }
        }
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

        // P 更新用 Joseph 形式（H 观测状态 0..3）
        self.joseph_update_cov(&k, 3, 0, r, true, true);
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

        // Joseph 形式（H 观测状态 3..6）；对角策略沿用本方法原有的"只做下限"
        self.joseph_update_cov(&k, 3, 3, r, false, false);
    }

    /// 速度观测更新（GPS Doppler，默认噪声 `r_vel`）。
    ///
    /// **只观测水平分量**：NMEA RMC 仅提供水平地速（无垂直速度），GPS Doppler
    /// 的垂向分量无物理意义（固件 GpsUblox 对 RMC 的 D 分量置 0）。若把 0 当
    /// 垂向速度观测注入，会与 baro 高度 + IMU 垂向融合冲突 → 高度环振荡发散
    /// （实测：RMC 激活后 pos[2] 在 ±11m 剧烈振荡）。垂向速度/零偏由 baro 与
    /// IMU 路径估计，水平速度由 GPS Doppler 约束（悬停水平漂移根治）。
    pub fn update_vel(&mut self, vel: [f32; 3]) {
        self.update_vel_r([vel[0], vel[1], self.x[5]], self.r_vel);
    }

    /// 高度观测更新步（气压计）：气压计测得**向上**高度 `alt`，而状态 D 轴向下为正，
    /// 故观测 `z = -alt`，H = [0 0 1 0 0 0 0 0 0]（作用于 D 位置索引 2）。
    /// Joseph 形式，栈数组作用域限于本方法。
    pub fn update_alt(&mut self, alt: f32) {
        // 防御：观测值本身非有限（传感器/HIL 注入异常）直接跳过，防止 y 为 NaN 污染状态。
        if !alt.is_finite() {
            return;
        }
        // S = H P H^T + R (标量)，H 仅在索引 2（D 位置）非零
        let s = self.p[2 * N + 2] + self.r_alt;
        if !s.is_finite() || s.abs() < 1e-9 {
            return;
        }
        // K = P H^T / S (9x1)，仅 P 第 2 列非零
        let mut k = [0.0f32; N];
        for i in 0..N {
            k[i] = self.p[i * N + 2] / s;
        }
        // 创新 y = z - x，z = -alt（D 向下正）；非有限（状态已被污染）则跳过本观测，
        // 阻断 x += k*y 把 NaN 传播进状态。
        let y = -alt - self.x[2];
        if !y.is_finite() {
            return;
        }
        // 垂向速度状态 (5) 的增益行清零：气压同为位置观测，经非对角协方差 K[5]
        // 会推爆垂向速度（hil 闭环回归：恒定比力+气压下 vel_z 单拍 +24.6 → 爆炸）。
        // 垂向速度仅由加速度积分决定（同 update_pos 的处理），位置观测不直接修正它。
        k[5] = 0.0;
        // 卡尔曼增益限幅（与 update_pos_r 一致，见 K_MAX 注释）：紧噪声观测下
        // 非对角增益可爆炸，限幅后 Joseph 协方差更新保持 PSD，阻断交叉项发散。
        for e in k.iter_mut() {
            *e = e.clamp(-K_MAX, K_MAX);
        }
        // 垂向加计零偏状态 (9) 不可由气压高度观测驱动（不可观 → 发散风险）：
        // 清零其卡尔曼增益元素，使气压更新不修正 accel_bias_z（仅速度观测可估计）。
        k[9] = 0.0;
        for i in 0..N {
            self.x[i] += k[i] * y;
        }
        // Joseph 形式（标量气压高度观测，H 只看状态 2）
        let r = self.r_alt;
        self.joseph_update_cov(&k, 1, 2, r, true, true);
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

    /// 设置磁偏角（度，东偏为正）：磁北相对地理北（世界 +X）的偏角。
    /// 0 = 磁北即地理北（默认）。EKF 用该角度旋转参考地磁方向，使磁力计
    /// 航向锚定对准【地理北】而非磁北（磁航向 decl 修正，对标真机导航）。
    pub fn set_mag_declination(&mut self, decl_deg: f32) {
        let (s, c) = crate::math::sin_cos(decl_deg.to_radians());
        self.mag_ref = [c, s];
    }

    /// 磁力计航向锚定：把世界系水平磁场方向拉回磁北参考（+X），锚定四元数 yaw。
    ///
    /// 与 `att_alpha` 重力修正对称：重力锚 roll/pitch（世界系重力 → +Z），
    /// 磁力计锚 yaw（世界系水平磁场 → 磁北）。纯陀螺积分时 yaw 无观测会缓慢漂移
    /// （陀螺零偏残余 → yaw 积分漂移），磁力计提供绝对航向参考。
    ///
    /// 原理：机体系磁场 `m` 经**估计**姿态旋转到世界系 `m_world`；若估计 yaw 偏差
    /// δ，`m_world` 水平分量相对磁北偏转 δ。`yaw_err = atan2(-my, mx)` 即该偏差，
    /// 绕世界 Z 轴按 `mag_alpha` 强度修正（符号：yaw 偏大 → 磁场偏东 → yaw_err<0
    /// → 绕 -Z 修正，yaw 减小）。roll/pitch 分量不受影响（绕世界 Z 纯 yaw 修正）。
    fn update_mag(&mut self, mag: Option<[f32; 3]>) {
        if self.mag_alpha <= 0.0 {
            return;
        }
        let m = match mag {
            Some(m) => m,
            None => return,
        };
        if !m.iter().all(|v| v.is_finite()) {
            return;
        }
        let m_world = rotate_vec_by_quat(self.att, m);
        let mh = sqrt(m_world[0] * m_world[0] + m_world[1] * m_world[1]);
        // 水平磁场过弱（磁力计几乎指向天顶/地磁水平分量≈0）→ 无法提供航向参考。
        if mh < 1e-3 {
            return;
        }
        // 世界系水平磁场相对【参考地磁方向（磁北）】的偏角 = yaw 估计误差。
        // mag_ref=(cos decl, sin decl)：把 m_world 水平分量旋转 -decl 投影到参考系，
        // 再取 atan2 —— 机头指向磁北（地理北+decl）时误差为 0，实现磁航向→地理航向。
        let (rx, ry) = (self.mag_ref[0], self.mag_ref[1]);
        let cx = m_world[0] * rx + m_world[1] * ry;
        let cy = -m_world[0] * ry + m_world[1] * rx;
        let yaw_err = crate::math::atan2(-cy, cx);
        // 归一化到 [-π, π]：atan2 已保证，无需 wrap。
        let k = self.mag_alpha * 0.5;
        let dq = Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(yaw_err * k));
        self.att = (dq * self.att).normalize();
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
    fn mag_declination_aligns_yaw_to_true_north() {
        // 磁北偏东 15°（decl=15°）。真实机头地理航向 15°（=磁北）时机体系
        // 水平磁场沿 +X（磁北方向旋转到机体系）。估计 yaw=0（偏差 -15°）。
        // EKF 应把 yaw 从 0 收敛到 +15°（地理航向），而非 0°（磁航向）。
        let mut ekf = EkfEstimator::default_quad();
        ekf.set_mag_declination(15.0);
        // 真实机头地理 yaw=15°：机体系磁场 = R^T(15°)·[0.2,0,0.4]（世界系磁场在 +15°）
        // 水平分量旋转到机体系后沿 +X → 读数即 [0.2, 0, 0.4]。
        let m_body = [0.2f32, 0.0, 0.4];
        for _ in 0..500 {
            ekf.update_mag(Some(m_body));
        }
        let yaw_deg = ekf.att.yaw().to_degrees();
        assert!(
            (yaw_deg - 15.0).abs() < 3.0,
            "decl=15° 时 yaw 应收敛到 15°（地理北），实际 {yaw_deg}°"
        );

        // 对照：decl=0（默认）时同一读数（磁北=地理北场景）收敛到 0°。
        let mut ekf0 = EkfEstimator::default_quad();
        for _ in 0..500 {
            ekf0.update_mag(Some(m_body));
        }
        let yaw0 = ekf0.att.yaw().to_degrees();
        assert!(yaw0.abs() < 3.0, "decl=0 时 yaw 应收敛到 0°，实际 {yaw0}°");
    }

    #[test]
    fn mag_heading_correction_converges_yaw() {
        // 初始估计 yaw 偏 30°（纯偏航误差，roll/pitch=0）。真实 yaw=0 时机体系
        // 磁场 = 北向参考 [0.2, 0, 0.4]（水平 +X 磁北）。update_mag 应把 yaw
        // 从 30° 收敛回 0°，且不动 roll/pitch。
        let mut ekf = EkfEstimator::default_quad();
        ekf.att = Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(30f32.to_radians()));
        let m_body = [0.2f32, 0.0, 0.4];
        for _ in 0..400 {
            ekf.update_mag(Some(m_body));
        }
        let yaw_deg = ekf.att.yaw().to_degrees();
        assert!(yaw_deg.abs() < 3.0, "yaw 应收敛到 0°，实际 {yaw_deg}°");
        assert!(ekf.att.roll().abs() < 1e-3, "磁力计不应扰动 roll");
        assert!(ekf.att.pitch().abs() < 1e-3, "磁力计不应扰动 pitch");

        // mag=None / 全零 / 非有限 → 不应改变姿态。
        let att_before = ekf.att;
        ekf.update_mag(None);
        assert_eq!(ekf.att, att_before);
        ekf.update_mag(Some([0.0, 0.0, 0.0]));
        assert_eq!(ekf.att, att_before);
        ekf.update_mag(Some([f32::NAN, 0.0, 0.4]));
        assert_eq!(ekf.att, att_before);

        // 反向偏置（yaw=-30°）同样收敛回 0°。
        let mut ekf2 = EkfEstimator::default_quad();
        ekf2.att = Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(-30f32.to_radians()));
        for _ in 0..400 {
            ekf2.update_mag(Some(m_body));
        }
        let yaw2 = ekf2.att.yaw().to_degrees();
        assert!(yaw2.abs() < 3.0, "反向 yaw 应收敛到 0°，实际 {yaw2}°");
    }

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
    }
}
