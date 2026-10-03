//! ★design.md §5：**IMU 预处理（陷波/低通）** —— 从 `HilContext`（EKF 上下文）抽出的
//! **可复用部件**，归 **`wq:sensors`** 所有（PX4：`sensors` 模块滤波后发
//! `vehicle_angular_velocity` / `vehicle_acceleration`）。
//!
//! 链路（对齐 PX4 一手）：
//!   · **加计**：40Hz 陷波 → 20Hz Butterworth 低通 → EKF（`accel_filt`）
//!   · **陀螺**：陷波(opt-in) → 40Hz 低通(`IMU_GYRO_CUTOFF`) → **仅喂控制器**（`gyro_vel`）
//!   · EKF 用**未滤波陀螺**（PX4：notch/LPF "not the estimators" ✓）
//!
//! 参数全部**配置驱动**（`G_ESKF_GYR_LPF` / `G_ESKF_GYR_NOTCH_FRQ` / `_BW` / `_Q`），
//! 与 `HilContext` 原实现**逐位一致**（本文件为该构造逻辑的**唯一**实现 ✓）。

use crate::filter::Biquad;

pub struct ImuFilters {
    /// 加计 40Hz 陷波（每轴独立）。
    pub accel_notch: [Biquad; 3],
    /// 加计 20Hz 低通（Butterworth 2 阶，每轴独立）。
    pub accel_lpf: [Biquad; 3],
    /// 陀螺陷波（默认禁用 ⇒ 恒等；`G_ESKF_GYR_NOTCH_FRQ>0` 开）。
    pub gyro_notch: [Biquad; 3],
    /// 陀螺**控制器侧**低通（`IMU_GYRO_CUTOFF`，默认 40Hz；`<0` 强制关）。
    pub gyro_lpf: [Biquad; 3],
}

impl ImuFilters {
    /// 按采样率 `fs`（= 1/dt）构造；配置旋钮与 `HilContext` 原实现同源 ✓。
    pub fn new(fs: f32) -> Self {
        use crate::estimator::eskf::{
            G_ESKF_GYR_LPF, G_ESKF_GYR_NOTCH_BW, G_ESKF_GYR_NOTCH_FRQ, G_ESKF_GYR_NOTCH_Q,
        };
        let accel_notch = [Biquad::notch(40.0, fs, 5.0); 3];
        let accel_lpf = [Biquad::low_pass(20.0, fs, 0.7071); 3];
        let gyro_lpf = {
            let v = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_ESKF_GYR_LPF)) };
            if v < 0.0 {
                [Biquad::passthrough(); 3]
            } else {
                let fc = if v > 0.0 { v } else { 40.0 };
                [Biquad::low_pass(fc, fs, 0.7071); 3]
            }
        };
        let gyro_notch = {
            let frq = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_ESKF_GYR_NOTCH_FRQ)) };
            let bw = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_ESKF_GYR_NOTCH_BW)) };
            let bw = if bw > 0.0 { bw } else { 20.0 };
            let q_ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_ESKF_GYR_NOTCH_Q)) };
            if frq > 0.0 {
                let q = if q_ov > 0.0 { q_ov } else { frq / bw };
                [Biquad::notch(frq, fs, q); 3]
            } else {
                [Biquad::passthrough(); 3]
            }
        };
        Self { accel_notch, accel_lpf, gyro_notch, gyro_lpf }
    }

    /// 逐样本滤波 → `(accel_filt, gyro_vel)`（机体系三轴）。
    #[inline]
    pub fn process(&mut self, accel: [f32; 3], gyro: [f32; 3]) -> ([f32; 3], [f32; 3]) {
        let mut a = [0.0f32; 3];
        let mut g = [0.0f32; 3];
        for i in 0..3 {
            a[i] = self.accel_lpf[i].process(self.accel_notch[i].process(accel[i]));
            g[i] = self.gyro_lpf[i].process(self.gyro_notch[i].process(gyro[i]));
        }
        (a, g)
    }

    /// ★§5.187：比力链路群延迟 τ（供估计器重力辅助补偿）——与原实现同源 ✓。
    pub fn group_delay_accel(&self, f_ref: f32, fs: f32) -> f32 {
        self.gyro_notch[0].group_delay_s(f_ref, fs) + self.accel_lpf[0].group_delay_s(f_ref, fs)
    }
}
