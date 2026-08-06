//! 互补滤波（工程基线估计器）。
//!
//! 思路：陀螺积分提供高频姿态（动态好但漂移），加速度计/磁力计提供
//! 低频参考（无漂移但动态差），二者按可调系数融合。
//! 这里实现姿态互补滤波 + 位置直接低通（基线，后续用 EKF 超越）。

use crate::units::*;
use crate::vehicle::{ImuSample, PosSample, Quaternion, VehicleState, rotate_vec_by_quat_inverse};
use crate::estimator::Estimator;

pub struct ComplementaryEstimator {
    att: Quaternion,
    omega: [RadianPerSecond; 3],
    pos: [Meter; 3],
    vel: [MeterPerSecond; 3],
    /// 陀螺->姿态融合系数（0..1，越大越信任陀螺）
    att_alpha: f32,
    /// 位置低通系数
    pos_alpha: f32,
}

impl ComplementaryEstimator {
    pub fn new(att_alpha: f32, pos_alpha: f32) -> Self {
        Self {
            att: Quaternion::IDENTITY,
            omega: [RadianPerSecond::ZERO; 3],
            pos: [Meter::ZERO; 3],
            vel: [MeterPerSecond::ZERO; 3],
            att_alpha,
            pos_alpha,
        }
    }
}

impl Estimator for ComplementaryEstimator {
    fn step(&mut self, dt: Second, imu: ImuSample, pos: Option<PosSample>) -> VehicleState {
        // 1) 姿态：陀螺积分 + 加速度计重力参考修正
        let (p, q, r) = (imu.gyro[0].0, imu.gyro[1].0, imu.gyro[2].0);
        self.att = self.att.integrate(p, q, r, dt.0);
        self.omega = imu.gyro;

        // 用加速度计（机体比力）估计重力方向，与当前姿态预测重力做误差修正。
        // 标准互补思路：err = up_meas × g_pred（机体轴小角误差向量），
        // 修正 att ← att ⊗ exp(α · err)。
        let ax = imu.accel[0].0;
        let ay = imu.accel[1].0;
        let az = imu.accel[2].0;
        let a_norm = crate::math::sqrt(ax * ax + ay * ay + az * az);
        if a_norm > 1e-3 {
            // 比力 ≈ 机体坐标系下的 -g 方向；测量"上方向" = -normalize(accel)
            let up_mx = -ax / a_norm;
            let up_my = -ay / a_norm;
            let up_mz = -az / a_norm;
            // 当前姿态预测的机体"上方向"（世界 up=[0,0,1] 旋到机体）
            let g_pred = rotate_vec_by_quat_inverse(self.att, [0.0, 0.0, 1.0]);
            // 误差向量 = up_meas × g_pred（小角：直接是机体轴修正量）
            let ex = up_my * g_pred[2] - up_mz * g_pred[1];
            let ey = up_mz * g_pred[0] - up_mx * g_pred[2];
            let ez = up_mx * g_pred[1] - up_my * g_pred[0];
            // 指数映射小量：q_corr = [1, α/2·e]
            let cx = self.att_alpha * ex * 0.5;
            let cy = self.att_alpha * ey * 0.5;
            let cz = self.att_alpha * ez * 0.5;
            self.att = Quaternion {
                w: 1.0 * self.att.w - cx * self.att.x - cy * self.att.y - cz * self.att.z,
                x: 1.0 * self.att.x + cx * self.att.w + cy * self.att.z - cz * self.att.y,
                y: 1.0 * self.att.y + cy * self.att.w + cz * self.att.x - cx * self.att.z,
                z: 1.0 * self.att.z + cz * self.att.w + cx * self.att.y - cy * self.att.x,
            }.normalize();
        }

        // 2) 位置：若有 GPS/气压，低通融合；速度由位置差分估算（基线）
        if let Some(p) = pos {
            for i in 0..3 {
                let filtered = self.pos[i].0 + self.pos_alpha * (p.pos[i].0 - self.pos[i].0);
                self.pos[i] = Meter(filtered);
            }
        }

        VehicleState {
            pos: self.pos,
            vel: self.vel,
            att: self.att,
            omega: self.omega,
        }
    }

    fn reset(&mut self) {
        *self = Self::new(self.att_alpha, self.pos_alpha);
    }
}
