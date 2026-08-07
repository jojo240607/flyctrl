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
    ImuSample, PosSample, Quaternion, VehicleState, rotate_vec_by_quat,
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
    att_alpha: f32,     // 姿态重力修正强度（0 = 纯积分）
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
            att_alpha,
        }
    }

    pub fn default_quad() -> Self {
        Self::new(0.0, 0.05, 1e-5, 0.5)
    }

    /// 当前估计的陀螺零偏（调试/诊断用）。
    pub fn gyro_bias(&self) -> [f32; 3] {
        [self.x[6], self.x[7], self.x[8]]
    }

    /// 当前协方差矩阵（9x9 行主序）引用，供不变量校验（对称半正定）使用。
    pub fn cov(&self) -> &[f32; N * N] {
        &self.p
    }
}

impl Estimator for EkfEstimator {
    fn step(&mut self, dt: Second, imu: ImuSample, gps: Option<PosSample>) -> VehicleState {
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

        // 协方差预测：F = I + A*dt, A 仅 [pos][vel]=I
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

        // 观测更新（位置）：H = [I3 0 0]
        if let Some(z) = gps {
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
            // Joseph 形式在浮点下保持对称半正定（<=> 真实方差），
            // 避免朴素 P = (I-KH)P 在数值误差下出现负对角元/非对称。
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

        VehicleState {
            pos: [Meter(self.x[0]), Meter(self.x[1]), Meter(self.x[2])],
            vel: [MeterPerSecond(self.x[3]), MeterPerSecond(self.x[4]), MeterPerSecond(self.x[5])],
            att: self.att,
            omega: [
                RadianPerSecond(imu.gyro[0].0 - self.x[6]),
                RadianPerSecond(imu.gyro[1].0 - self.x[7]),
                RadianPerSecond(imu.gyro[2].0 - self.x[8]),
            ],
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
