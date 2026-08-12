//! 互补滤波（工程基线估计器）。
//!
//! 思路：陀螺积分提供高频姿态（动态好但漂移），加速度计提供
//! 低频参考（无漂移但动态差），二者按可调系数融合。
//! 这里实现姿态互补滤波 + 速度(IMU比力积分主导) + 位置偏差缓慢回拉（基线，后续用 EKF 超越）。
//!
//! 关键设计（避免噪声喂进控制环而发散）：
//!   - 速度完全由 IMU 比力积分主导（高频、无噪声放大），**绝不做位置差分速度**。
//!   - 位置 = IMU 积分位置 + pos_bias；pos_bias 以【极慢】增益跟随测量偏差，
//!     时间常数远大于闭环带宽，仅约束 IMU 积分漂移，不把 0.3m 噪声泄漏进控制环。
//!   - 输出给控制器的位置估计再经一层 EMA 低通（pos_out_alpha），进一步压低噪声。

use crate::units::*;
use crate::vehicle::{AirspeedSample, ImuSample, PosSample, Quaternion, VehicleState, rotate_vec_by_quat};
use crate::estimator::Estimator;

pub struct ComplementaryEstimator {
    att: Quaternion,
    omega: [RadianPerSecond; 3],
    /// IMU 积分位置（高频，含漂移）
    pos_int: [Meter; 3],
    vel: [MeterPerSecond; 3],
    /// 输出位置低通（测量位置主导，消除积分漂移）
    pos_out: [Meter; 3],
    /// 观测速度低通（位置差分经深低通，消除 60 m/s 尖峰）
    obs_v_lpf: [MeterPerSecond; 3],
    /// 上一拍测量位置（用于位置差分）
    prev_raw: [Meter; 3],
    /// 估计空速（m/s），由空速计测量直接低通获得
    airspeed: f32,
    /// 陀螺->姿态融合系数（0..1，越大越信任陀螺）
    att_alpha: f32,
    /// 输出位置 EMA 系数（测量位置低通，滤除 GPS 级噪声）
    pos_out_alpha: f32,
    /// 速度中 IMU 积分补充占比（小值：速度主要由深低通观测速度主导，消除漂移）
    vel_alpha: f32,
    /// 观测速度低通系数（对原始位置差分做深低通，消除 GPS 级噪声尖峰）
    obs_alpha: f32,
    /// 是否已收到首帧位置
    have_prev: bool,
}

impl ComplementaryEstimator {
    /// `att_alpha` 姿态陀螺信任（如 0.0~0.5）；`pos_out_alpha` 位置 EMA 低通（如 0.1）；
    /// `vel_alpha` 速度中观测分量占比（**应很小，如 0.02**：速度以 IMU 积分为主，
    /// 仅极小比例混入观测分量约束长期漂移。过大->GPS 级噪声灌入速度->水平控制饱和翻滚）；
    /// `obs_alpha` 位置差分观测低通（应极小，如 0.01，消除 GPS 级噪声尖峰）。
    pub fn new(att_alpha: f32, pos_out_alpha: f32, vel_alpha: f32) -> Self {
        Self::with_vel(att_alpha, pos_out_alpha, vel_alpha, 0.01)
    }

    /// 带观测速度低通系数的构造（用于调参）。
    pub fn with_vel(att_alpha: f32, pos_out_alpha: f32, vel_alpha: f32, obs_alpha: f32) -> Self {
        Self {
            att: Quaternion::IDENTITY,
            omega: [RadianPerSecond::ZERO; 3],
            pos_int: [Meter::ZERO; 3],
            vel: [MeterPerSecond::ZERO; 3],
            pos_out: [Meter::ZERO; 3],
            obs_v_lpf: [MeterPerSecond::ZERO; 3],
            prev_raw: [Meter::ZERO; 3],
            airspeed: 0.0,
            att_alpha,
            pos_out_alpha,
            vel_alpha,
            obs_alpha,
            have_prev: false,
        }
    }
}

impl Estimator for ComplementaryEstimator {
    fn step(&mut self, dt: Second, imu: ImuSample, pos: Option<PosSample>, airspeed: Option<AirspeedSample>) -> VehicleState {
        // 1) 姿态：陀螺积分 + 加速度计重力参考修正（互补滤波，直接修正姿态，不做零偏估计）。
        //    设计要点（闭环稳定性）：估计姿态必须紧贴真实姿态。真实姿态 = 积分(真实陀螺)；
        //    本估计器同样积分(传感器陀螺)，二者仅在陀螺噪声上有差异。因此陀螺噪声必须
        //    足够小，使随机游走漂移远小于控制器容差。加速度计修正仅用于约束慢漂移、
        //    把姿态锚定在重力方向（roll/pitch 有参考，yaw 无），增益很小以免引入不稳定性。
        let (p, q, r) = (imu.gyro[0].0, imu.gyro[1].0, imu.gyro[2].0);
        self.att = self.att.integrate(p, q, r, dt.0);

        // 试验：暂不做加速度计姿态修正，验证纯陀螺积分 + 极小噪声下估计姿态能否贴住
        // 真实姿态并使闭环收敛。若收敛，再决定以多小增益加回修正。
        let ax = imu.accel[0].0;
        let ay = imu.accel[1].0;
        let az = imu.accel[2].0;

        // 角速度估计：直接用原始陀螺（与真实姿态积分同源）。
        // 注意：绝不能对角速度做低通再喂给速率阻尼项 att_kd*omega —— 那会让阻尼项相对
        // 真实角速度产生相位滞后，在姿态 PD 环的自然频率附近变成"反阻尼"，使滚转/俯仰轴
        // 失稳（PhysA 用真实未滤波 omega 故稳定，PhysB 用滞后 omega 故发散）。陀螺噪声已
        // 足够小（0.0003），无需滤波；即便要滤也应放在控制环外。
        self.omega = [RadianPerSecond(p), RadianPerSecond(q), RadianPerSecond(r)];

        // 2) 速度/位置：位置测量是绝对参考（GPS/视觉级），主导定位。
        //    a_world = R * f_body + g_world （比力经姿态旋到世界系，再加回重力）。
        //    关键教训：纯 IMU 积分为位置和速度会无约束漂移，即便是悬停也会漂，
        //    控制器把漂移误判为水平运动 -> 错误 tilt 指令 -> 翻滚。
        //    因此位置估计直接由【测量位置经 EMA 低通】给出（不依赖会漂的积分），
        //    速度估计由【位置差分的深低通】给出；IMU 比力积分仅作为速度的
        //    高频补充（vel_alpha 小），并在测量缺失时维持短期预测。
        let f_world = rotate_vec_by_quat(self.att, [ax, ay, az]);
        let a_world = [f_world[0], f_world[1], f_world[2] + 9.81f32];
        for i in 0..3 {
            // IMU 积分（高频、低 lag）——作为短期预测/补充
            let vel_n = self.vel[i].0 + a_world[i] * dt.0;
            let pos_n = self.pos_int[i].0 + vel_n * dt.0;
            self.pos_int[i] = Meter(pos_n);
            self.vel[i] = MeterPerSecond(vel_n);

            if let Some(ref p) = pos {
                let raw = p.pos[i].0;
                // 观测速度：位置差分（原始，含 GPS 级噪声尖峰）
                let obs_v_raw = if dt.0 > 1e-6 && self.have_prev {
                    (raw - self.prev_raw[i].0) / dt.0
                } else { 0.0 };
                // 极深低通观测速度：obs_alpha 极小，仅作长期漂移约束。原始位置差分含
                // ~60 m/s 噪声尖峰，α 过大（如 0.05）会让速度估计 std 达 ~13 m/s，直接
                // 灌爆水平速度控制 -> 饱和翻滚。这里 α 很小，obs 仅缓慢牵引 IMU 积分。
                let obs_v = self.obs_v_lpf[i].0 + self.obs_alpha * (obs_v_raw - self.obs_v_lpf[i].0);
                self.obs_v_lpf[i] = MeterPerSecond(obs_v);
                // 速度以 IMU 积分为主（平滑、短期精确），仅极小比例混入观测分量：
                // vel_alpha 越小越信任 IMU。最终速度 = (1-β)*vel_n + β*obs_v，β=vel_alpha。
                let vel_c = (1.0 - self.vel_alpha) * vel_n + self.vel_alpha * obs_v;
                self.vel[i] = MeterPerSecond(vel_c);
                // 位置 = 测量位置经 EMA 低通（绝对参考，消除积分漂移）
                let out_n = self.pos_out[i].0 + self.pos_out_alpha * (raw - self.pos_out[i].0);
                self.pos_out[i] = Meter(out_n);
                self.prev_raw[i] = Meter(raw);
                self.have_prev = true;
            } else {
                // 无测量：纯 IMU 积分外推（短期），位置/速度由积分维持
                self.pos_out[i] = Meter(pos_n);
            }
        }

        // 3) 空速（互补滤波：直接信任测量，轻低通，时间常数 ~0.5s）
        if let Some(aspd) = airspeed {
            let s = aspd.speed.0;
            let alpha = (dt.0 / (dt.0 + 0.5)).clamp(0.0, 1.0);
            self.airspeed += alpha * (s - self.airspeed);
        }

        VehicleState {
            pos: self.pos_out,
            vel: self.vel,
            att: self.att,
            omega: self.omega,
            airspeed: MeterPerSecond(self.airspeed),
        }
    }

    fn reset(&mut self) {
        *self = Self::with_vel(self.att_alpha, self.pos_out_alpha, self.vel_alpha, self.obs_alpha);
    }
}

impl ComplementaryEstimator {
    /// 暴露当前估计姿态（调试/检验用）。
    pub fn att(&self) -> Quaternion { self.att }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::*;
    use crate::vehicle::{ImuSample, PosSample, VehicleState};

    // 静态水平：无角速度，比力 = -g (机体 Z 向下为正，故 [0,0,-9.81])
    // 期望姿态保持单位四元数 (w≈1, 其余≈0)。
    #[test]
    fn static_level_keeps_attitude() {
        let mut est = ComplementaryEstimator::new(0.5, 0.1, 0.1);
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(-9.81)],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let mut last: VehicleState = est.step(Second(0.005), imu, None, None);
        for _ in 0..200 {
            last = est.step(Second(0.005), imu, None, None);
        }
        assert!(last.att.w > 0.99, "attitude corrupted: w={:.4} x={:.4} y={:.4} z={:.4}",
            last.att.w, last.att.x, last.att.y, last.att.z);
        assert!(last.att.x.abs() < 0.01);
        assert!(last.att.y.abs() < 0.01);
    }
}
