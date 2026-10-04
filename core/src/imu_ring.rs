//! ★design.md §7 + PX4 同构：**IMU 样本环形**（生产者=采样/`sensors`，消费者=rate/ekf）。
//!
//! 为何存 **增量**（对齐 PX4 `imuSample.delta_ang/delta_vel` ✓，见 `ekf.cpp:184`
//! `predictState(imu_sample_delayed)` 用 `delta_ang/delta_vel`）：
//!   · 消费者频率不一（`rate` 1kHz / `ekf` 250Hz），若 EKF 只取"最新一帧"会**丢 3/4** ✗；
//!   · 存**样本间积分量 + 各自 dt** ⇒ EKF 可**排空环形、逐样本 predict**（design.md §7
//!     「积分用实际 dt」），**不丢样本** ✓；
//!   · 瞬时量（角速率/比力）由 `delta/dt` 反解 ✓。
//!
//! 并发：**SPSC**（生产者=采样，消费者=某任务）；单核 ⇒ 生产者只改 `head`、消费者只改 `tail`。

/// 环形容量（32 帧 @1kHz = 32ms 余量）。
pub const IMU_RING_N: usize = 32;

/// 一帧 **IMU 增量**（机体系；对齐 PX4 `imuSample` ✓）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuDelta {
    /// 相邻样本间**角增量**（rad，机体系；= ∫gyro dt）。
    pub delta_ang: [f32; 3],
    /// 相邻样本间**比力增量**（m/s，机体系；= ∫accel dt）。
    pub delta_vel: [f32; 3],
    /// ★★★2026-10-04【对齐 PX4 `imuSample` 的**双 dt** 设计 ✓】：
    ///   PX4 的陀螺与加计是**两条独立消息**（各有自己的 dt ✓）——
    ///   姿态积分用 `delta_ang_dt` ✓、速度/重力用 `delta_vel_dt` ✓、
    ///   协方差用 `0.5*(两者)` ✓（`cov.cpp:116` ✓）。
    ///   本仓原为**单一 dt** ✗ —— 只在"同器件同 DRDY（BMI088 ✓）"下凑巧成立 ✗，
    ///   **换独立陀螺/加计或两者 ODR 不同时就会错** ✗。现按 PX4 结构拆分 ✓
    ///   （当前两值恒等 ✓，行为不变 ✓，但结构已具前瞻性 ✓）。
    pub dt_ang: f32,
    /// 比力增量对应的时间间隔（s ✓）
    pub dt_vel: f32,
    /// 采样**硬件时间戳**（DWT CYCCNT，~6ns；design.md §7）。
    pub ts_cyc: u32,
}

impl ImuDelta {
    /// 全零（静态数组初始化用）。
    pub const ZERO: ImuDelta = ImuDelta { delta_ang: [0.0; 3], delta_vel: [0.0; 3], dt_ang: 0.0, dt_vel: 0.0, ts_cyc: 0 };

    /// 反解瞬时角速率（rad/s）。
    #[inline]
    pub fn gyro(&self) -> [f32; 3] {
        let inv = if self.dt_ang > 1e-9 { 1.0 / self.dt_ang } else { 0.0 };
        [self.delta_ang[0] * inv, self.delta_ang[1] * inv, self.delta_ang[2] * inv]
    }
    /// 反解瞬时比力（m/s²）。
    #[inline]
    pub fn accel(&self) -> [f32; 3] {
        let inv = if self.dt_vel > 1e-9 { 1.0 / self.dt_vel } else { 0.0 };
        [self.delta_vel[0] * inv, self.delta_vel[1] * inv, self.delta_vel[2] * inv]
    }
}

/// SPSC 环形缓冲。
pub struct ImuRing {
    buf: [ImuDelta; IMU_RING_N],
    /// 生产者写指针（下一写入位）。
    head: usize,
    /// 消费者读指针（下一读取位）。
    tail: usize,
    /// 满时丢最旧的累计计数（诊断）。
    dropped: u32,
}

impl ImuRing {
    pub const fn new() -> Self {
        Self {
            buf: [ImuDelta { delta_ang: [0.0; 3], delta_vel: [0.0; 3], dt_ang: 0.0, dt_vel: 0.0, ts_cyc: 0 }; IMU_RING_N],
            head: 0,
            tail: 0,
            dropped: 0,
        }
    }

    /// 生产者入队一帧。**满 ⇒ 丢最旧**（`tail` 前进）并计 `dropped`（§7「保最新」）。
    pub fn push(&mut self, d: ImuDelta) {
        let next = (self.head + 1) % IMU_RING_N;
        if next == self.tail {
            self.tail = (self.tail + 1) % IMU_RING_N;
            self.dropped = self.dropped.wrapping_add(1);
        }
        self.buf[self.head] = d;
        self.head = next;
    }

    /// 消费者取一帧（`None` = 空）。
    pub fn pop(&mut self) -> Option<ImuDelta> {
        if self.tail == self.head {
            return None;
        }
        let d = self.buf[self.tail];
        self.tail = (self.tail + 1) % IMU_RING_N;
        Some(d)
    }

    /// 最新一帧（**不消费**）——供 `rate`(1kHz) 取"当前陀螺" ✓。
    pub fn latest(&self) -> Option<ImuDelta> {
        if self.head == self.tail {
            return None;
        }
        Some(self.buf[(self.head + IMU_RING_N - 1) % IMU_RING_N])
    }

    pub fn len(&self) -> usize {
        (self.head + IMU_RING_N - self.tail) % IMU_RING_N
    }
    pub fn is_empty(&self) -> bool {
        self.head == self.tail
    }
    pub fn dropped(&self) -> u32 {
        self.dropped
    }
}

impl Default for ImuRing {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(ts: u32) -> ImuDelta {
        ImuDelta { delta_ang: [0.0, 0.0, ts as f32], delta_vel: [0.0, 0.0, 9.81], dt: 0.001, ts_cyc: ts }
    }

    #[test]
    fn fifo_order_and_latest() {
        let mut r = ImuRing::new();
        for i in 0..5 {
            r.push(d(i));
        }
        assert_eq!(r.len(), 5);
        assert_eq!(r.latest().unwrap().ts_cyc, 4);
        for i in 0..5 {
            assert_eq!(r.pop().unwrap().ts_cyc, i);
        }
        assert!(r.is_empty() && r.pop().is_none());
    }

    #[test]
    fn full_drops_oldest_and_counts() {
        let mut r = ImuRing::new();
        for i in 0..(IMU_RING_N as u32 + 3) {
            r.push(d(i));
        }
        // 可用容量 = N-1 ⇒ 压 N+3 帧丢 4
        assert_eq!(r.dropped(), 4);
        assert_eq!(r.pop().unwrap().ts_cyc, 4);
        assert_eq!(r.latest().unwrap().ts_cyc, IMU_RING_N as u32 + 2);
    }

    #[test]
    fn delta_derives_instantaneous() {
        let x = ImuDelta { delta_ang: [0.002, 0.0, 0.0], delta_vel: [0.0, 0.0, 0.00981], dt: 0.001, ts_cyc: 0 };
        assert!((x.gyro()[0] - 2.0).abs() < 1e-5);
        assert!((x.accel()[2] - 9.81).abs() < 1e-3);
    }
}
