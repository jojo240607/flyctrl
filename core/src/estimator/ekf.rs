//! 松耦合误差状态 EKF 估计器（no_std, 无堆分配）。
//!
//! 状态分两部分：
//! - 名义姿态：四元数，由去偏置角速度积分得到（可选微弱重力修正）。
//! - 误差状态 EKF（9 维）：`[pos(3), vel(3), gyro_bias(3)]`。
//!   预测用 IMU 加计（经名义姿态旋转到世界系并补偿重力）推进速度/位置；
//!   观测用位置测量（GPS/气压）做标准卡尔曼更新，并回灌陀螺偏置到姿态积分。
//!
//! 与 [`super::ComplementaryEstimator`] 的区别：显式估计并补偿陀螺零偏，
//! 且位置/速度用卡尔曼增益融合观测，而非简单的 alpha 混合。

use crate::estimator::trait_def::Estimator;
use crate::math::sqrt;
use crate::units::*;
use crate::vehicle::{
    AirspeedSample, ImuSample, PosSample, Quaternion, VehicleState, rotate_vec_by_quat,
};

const N: usize = 9; // pos(3) + vel(3) + bias(3)

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
    x: [f32; N],        // pos, vel, gyro_bias
    p: [f32; N * N],    // 协方差
    g_ref: f32,
    q_vel: f32,         // 速度过程噪声强度
    q_bias: f32,        // 偏置随机游走
    r_pos: f32,         // 位置观测噪声
    r_airspeed: f32,    // 空速观测噪声（m/s）^2
    att_alpha: f32,     // 姿态重力修正强度（0 = 纯积分）
    airspeed_est: f32,  // 估计空速 (m/s)，由空速计融合得到
}

impl EkfEstimator {
    pub fn new(att_alpha: f32, q_vel: f32, q_bias: f32, r_pos: f32) -> Self {
        let mut p = [0.0f32; N * N];
        mat_ident(&mut p);
        for i in 0..3 {
            p[i * N + i] = 1.0; // pos ~1m^2
        }
        for i in 3..6 {
            p[i * N + i] = 1.0; // vel ~1 (m/s)^2
        }
        for i in 6..9 {
            p[i * N + i] = 1e-4; // bias ~0.01 rad/s
        }
        EkfEstimator {
            att: Quaternion::IDENTITY,
            x: [0.0; N],
            p,
            g_ref: 9.81,
            q_vel,
            q_bias,
            r_pos,
            r_airspeed: 0.75, // ~0.87 m/s RMS 空速观测噪声
            att_alpha,
            airspeed_est: 0.0,
        }
    }

    pub fn default_quad() -> Self {
        Self::new(0.0, 0.05, 1e-5, 0.5)
    }

    /// 当前估计的陀螺零偏（调试/诊断用）。
    pub fn gyro_bias(&self) -> [f32; 3] {
        [self.x[6], self.x[7], self.x[8]]
    }

    /// 当前估计的姿态四元数（调试/诊断用）。
    pub fn att(&self) -> Quaternion {
        self.att
    }

    /// 当前协方差矩阵（9x9 行主序）引用，供不变量校验（对称半正定）使用。
    pub fn cov(&self) -> &[f32; N * N] {
        &self.p
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
            let down_body = crate::vehicle::rotate_vec_by_quat_inverse(self.att, [0.0, 0.0, g]);
            let a = [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0];
            let an = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            if an > 1e-3 {
                let k = self.att_alpha * 0.5;
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

        // 位置/速度预测（世界系 NED）：a_world = R*a_body + g_vec(z 向下正)
        let a_world = rotate_vec_by_quat(self.att, [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0]);
        let ax_w = a_world[0];
        let ay_w = a_world[1];
        let az_w = a_world[2] + g;

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

        // 观测更新（位置）：H = [I3 0 0]（独立方法，同上理由拆分）。
        if let Some(z) = gps {
            self.update_pos(z);
        }

        // 观测更新（空速）：约束水平速度幅值 |v_h| = 测量空速（不含风）。
        if let Some(a) = airspeed {
            self.update_airspeed(a.speed.0);
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
            self.p[i * N + i] = 1e-4;
        }
    }
}

// ===== 以下为 `EkfEstimator` 自身的非 trait 方法（不是 `Estimator` trait 的一部分）=====
// 把协方差预测 / 位置观测更新从 `step` 里拆成独立方法，让各自的大栈数组（各 ~1620B）
// 作用域不重叠：编译器可在两支之间复用同一批栈槽，整体峰值 ≈ max(predict, update)
// 而非 sum，从而避免控制任务栈（3072B）溢出。

impl EkfEstimator {
    /// 协方差预测步：F = I + A*dt，P' = F P F^T + Q。
    /// 栈数组（f/ft/p_pred，各 81 元素）作用域限于本方法，返回后栈槽被回收。
    fn predict_cov(&mut self, dt: f32) {
        // F = I + A*dt, A 仅 [pos][vel]=I
        let mut f = [0.0f32; N * N];
        mat_ident(&mut f);
        for i in 0..3 {
            f[i * N + (3 + i)] = dt;
        }
        let mut ft = [0.0f32; N * N];
        mat_mul_at(&f, &self.p, &mut ft, N, N, N);
        let mut p_pred = [0.0f32; N * N];
        mat_mul(&f, &ft, &mut p_pred, N, N, N);
        let qv = self.q_vel * dt;
        let qb = self.q_bias * dt;
        for i in 3..6 {
            p_pred[i * N + i] += qv;
        }
        for i in 6..9 {
            p_pred[i * N + i] += qb;
        }
        // 对称化 + 对角线下限夹取（预测步也需保持 PSD）。
        for i in 0..N {
            for j in (i + 1)..N {
                let avg = 0.5 * (p_pred[i * N + j] + p_pred[j * N + i]);
                p_pred[i * N + j] = avg;
                p_pred[j * N + i] = avg;
            }
            if p_pred[i * N + i] < 1e-6 {
                p_pred[i * N + i] = 1e-6;
            }
        }
        self.p = p_pred;
    }

    /// 位置观测更新步（Joseph 形式）：S = HPH^T+R、K = P H^T S^-1、x += K y、
    /// P = (I-KH)P(I-KH)^T + K R K^T。
    /// 栈数组（s/pht/k/sinv/a/ap/apat/krkt）作用域限于本方法，返回后栈槽被回收。
    fn update_pos(&mut self, z: PosSample) {
        // S = H P H^T + R (3x3)
        let mut s = [0.0f32; 9];
        for i in 0..3 {
            for j in 0..3 {
                s[i * 3 + j] = self.p[i * N + j] + if i == j { self.r_pos } else { 0.0 };
            }
        }
        // K = P H^T S^-1 (9x3) ; P H^T 即 P 前三列
        let mut pht = [0.0f32; 27];
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

        let mut k = [0.0f32; 27];
        mat_mul(&pht, &sinv, &mut k, N, 3, 3);

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
        // 3) K R K^T （R = r_pos * I3）
        let mut krkt = [0.0f32; N * N];
        for i in 0..N {
            for j in 0..N {
                let mut acc = 0.0;
                for l in 0..3 {
                    acc += k[i * 3 + l] * k[j * 3 + l];
                }
                krkt[i * N + j] = acc * self.r_pos;
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
            // 对角线下限夹取，杜绝负方差。
            if self.p[i * N + i] < 1e-6 {
                self.p[i * N + i] = 1e-6;
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
        let mut ap = [0.0f32; N * N];
        for i in 0..N {
            for j in 0..N {
                let kh = if j == 3 { k[i] * hx } else if j == 4 { k[i] * hy } else { 0.0 };
                ap[i * N + j] = self.p[i * N + j] - kh * self.p[i * N + j];
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
            if self.p[i * N + i] < 1e-6 {
                self.p[i * N + i] = 1e-6;
            }
        }
        // 估计空速 = 水平速度幅值（融合后）
        self.airspeed_est = sqrt(self.x[3] * self.x[3] + self.x[4] * self.x[4]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
}
