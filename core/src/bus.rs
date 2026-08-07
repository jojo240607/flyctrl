//! 中间件层：类型安全消息总线（pub/sub）。
//!
//! 这是架构分层里的 **MIDDLEWARE**（消息总线/发布订阅，类型安全），建立在
//! `no_std`/零堆之上，为上层（任务 M9、编队 M8、FDIR）提供解耦的通信骨干：
//! 生产者发布到主题，消费者订阅主题，二者不直接互相持有引用。
//!
//! 设计取舍（贴合嵌入式现实）：
//! - **SPSC 环形缓冲** `Ring<M, CAP>`：单生产者单消费者，零分配、定容（`Copy`）。
//!   `try_push`/`try_pop` 在满/空时返回 `Err`/`None`（**拒绝而非静默覆盖/丢弃**），
//!   调用方据此背压或丢帧——符合飞行控制"宁可丢旧帧也不污染数据流"的偏好。
//! - **角色分离由 API 保证**：`Ring` 本身同时提供 `try_push`/`try_pop`；但 `Bus`
//!   只通过 `publish_*`（写生产者段）和消费者访问器（读消费者段）暴露对应能力，
//!   结构化地约束"谁写谁读"，等价于编译期角色边界而不需额外的句柄类型。
//! - **主题扇出**：飞行系统中一个估计状态常被多个消费者复用（控制器、FDIR、
//!   GCS 遥测）。`Bus::pump()` 从每个生产者通道弹出一次、复制推入该主题的
//!   所有消费者通道，实现零分配扇出（无需广播队列/堆）。
//! - 不依赖原子/临界区：调用方在 IRQ 上下文使用 `irq_lock` 包裹（与既有 HAL/驱动约定一致）。
//!
//! 这是"消息总线"的最小可用内核，后续可叠加：主题注册表（编译期 `TopicId`）、
//! 发布/订阅的运行时发现、时间标签与 QoS。

use crate::controller::trait_def::{ActuatorCmd, Setpoint};
use crate::fdir::Health;
use crate::flightmode::FlightMode;
use crate::vehicle::{ImuSample, PosSample, VehicleState};

/// 单生产者单消费者定容环形缓冲（零分配、`no_std`、`Copy`）。
///
/// 内部用 `head`/`tail`/`count` 三标记，避免 `head==tail` 的空/满歧义。
/// `Copy` 便于在聚合结构里按值持有多段独立通道（每段只被一方独占访问）。
#[derive(Debug, Clone, Copy)]
pub struct Ring<M: Copy, const CAP: usize> {
    buf: [Option<M>; CAP],
    head: usize, // 下一个写入位置
    tail: usize, // 下一个读出位置
    count: usize,
}

impl<M: Copy, const CAP: usize> Ring<M, CAP> {
    /// 空缓冲。要求 `CAP >= 1`。
    pub fn new() -> Self {
        Self {
            buf: [None; CAP],
            head: 0,
            tail: 0,
            count: 0,
        }
    }

    /// 当前元素数。
    pub fn len(&self) -> usize { self.count }

    /// 是否已满。
    pub fn is_full(&self) -> bool { self.count == CAP }

    /// 是否为空。
    pub fn is_empty(&self) -> bool { self.count == 0 }

    /// 容量。
    pub fn capacity(&self) -> usize { CAP }

    /// 推入一个元素；满则返回 `Err(msg)`（不覆盖旧数据）。
    pub fn try_push(&mut self, m: M) -> Result<(), M> {
        if self.count == CAP {
            return Err(m);
        }
        self.buf[self.head] = Some(m);
        self.head = (self.head + 1) % CAP;
        self.count += 1;
        Ok(())
    }

    /// 弹出一个元素（FIFO）；空则返回 `None`。
    pub fn try_pop(&mut self) -> Option<M> {
        if self.count == 0 {
            return None;
        }
        let m = self.buf[self.tail].take().unwrap();
        self.tail = (self.tail + 1) % CAP;
        self.count -= 1;
        Some(m)
    }
}

impl<M: Copy, const CAP: usize> Default for Ring<M, CAP> {
    fn default() -> Self { Self::new() }
}

/// 飞行系统消息总线：聚合固定主题通道，并提供 est 扇出与便捷访问。
///
/// 主题（及容量）：
/// - `imu`（8）：原始 IMU 样本（传感器→估计器）。
/// - `gps`（4）：位置样本（传感器→估计器）。
/// - `est`（4）：估计状态（估计器→控制器/FDIR/GCS）。
/// - `setpoint`（4）：设定点（任务/模式→控制器）。
/// - `actuator`（4）：执行器命令（控制器→执行器）。
/// - `mode`（4）：飞行模式 + 健康（模式治理→全体）。
///
/// 每段通道是独立的 `Ring`：生产者段（`*_in`，由 `publish_*` 写入、`pump` 读出）
/// 与消费者段（`*_out`，由 `pump` 写入、由对应访问器读出）。`est` 额外扇出到
/// `est_to_ctrl` 与 `est_to_fdir` 两个消费者段。
pub struct Bus {
    // 生产者段（仅 publish_* 写入；pump 读出）。
    imu_in: Ring<ImuSample, 8>,
    gps_in: Ring<PosSample, 4>,
    est_in: Ring<VehicleState, 4>,
    sp_in: Ring<Setpoint, 4>,
    act_in: Ring<ActuatorCmd, 4>,
    mode_in: Ring<(FlightMode, Health), 4>,

    // 消费者段（pump 写入；访问器读出）。
    imu_out: Ring<ImuSample, 8>,
    gps_out: Ring<PosSample, 4>,
    sp_out: Ring<Setpoint, 4>,
    act_out: Ring<ActuatorCmd, 4>,
    mode_out: Ring<(FlightMode, Health), 4>,
    est_to_ctrl: Ring<VehicleState, 4>,
    est_to_fdir: Ring<VehicleState, 4>,
}

impl Bus {
    /// 构造空总线（各段通道清零）。
    pub fn new() -> Self {
        Self {
            imu_in: Ring::new(), gps_in: Ring::new(), est_in: Ring::new(),
            sp_in: Ring::new(), act_in: Ring::new(), mode_in: Ring::new(),
            imu_out: Ring::new(), gps_out: Ring::new(), sp_out: Ring::new(),
            act_out: Ring::new(), mode_out: Ring::new(),
            est_to_ctrl: Ring::new(), est_to_fdir: Ring::new(),
        }
    }

    // ---- 生产者 API（写入生产者段） ----
    pub fn publish_imu(&mut self, m: ImuSample) -> Result<(), ImuSample> { self.imu_in.try_push(m) }
    pub fn publish_gps(&mut self, m: PosSample) -> Result<(), PosSample> { self.gps_in.try_push(m) }
    pub fn publish_est(&mut self, m: VehicleState) -> Result<(), VehicleState> { self.est_in.try_push(m) }
    pub fn publish_setpoint(&mut self, m: Setpoint) -> Result<(), Setpoint> { self.sp_in.try_push(m) }
    pub fn publish_actuator(&mut self, m: ActuatorCmd) -> Result<(), ActuatorCmd> { self.act_in.try_push(m) }
    pub fn publish_mode(&mut self, m: (FlightMode, Health)) -> Result<(), (FlightMode, Health)> {
        self.mode_in.try_push(m)
    }

    // ---- 消费者 API（读出消费者段） ----
    pub fn recv_imu(&mut self) -> Option<ImuSample> { self.imu_out.try_pop() }
    pub fn recv_gps(&mut self) -> Option<PosSample> { self.gps_out.try_pop() }
    pub fn recv_est_ctrl(&mut self) -> Option<VehicleState> { self.est_to_ctrl.try_pop() }
    pub fn recv_est_fdir(&mut self) -> Option<VehicleState> { self.est_to_fdir.try_pop() }
    pub fn recv_setpoint(&mut self) -> Option<Setpoint> { self.sp_out.try_pop() }
    pub fn recv_actuator(&mut self) -> Option<ActuatorCmd> { self.act_out.try_pop() }
    pub fn recv_mode(&mut self) -> Option<(FlightMode, Health)> { self.mode_out.try_pop() }

    /// 扇出泵：把各生产者段的最新数据复制到对应消费者段。
    ///
    /// 对 `est`：从 `est_in` 弹出并复制进 `est_to_ctrl` 与 `est_to_fdir`（扇出）。
    /// 其余主题：从 `*_in` 弹出推入对应 `*_out`。每调一次最多搬运一个元素/主题，
    /// 调用方通常每控制周期调一次（或循环调至各通道清空）。
    ///
    /// 返回本次搬运的元素总数（用于诊断背压/丢帧）。
    pub fn pump(&mut self) -> usize {
        let mut moved = 0usize;
        if let Some(m) = self.imu_in.try_pop() {
            if self.imu_out.try_push(m).is_ok() { moved += 1; }
        }
        if let Some(m) = self.gps_in.try_pop() {
            if self.gps_out.try_push(m).is_ok() { moved += 1; }
        }
        if let Some(m) = self.est_in.try_pop() {
            if self.est_to_ctrl.try_push(m).is_ok() { moved += 1; }
            if self.est_to_fdir.try_push(m).is_ok() { moved += 1; }
        }
        if let Some(m) = self.sp_in.try_pop() {
            if self.sp_out.try_push(m).is_ok() { moved += 1; }
        }
        if let Some(m) = self.act_in.try_pop() {
            if self.act_out.try_push(m).is_ok() { moved += 1; }
        }
        if let Some(m) = self.mode_in.try_pop() {
            if self.mode_out.try_push(m).is_ok() { moved += 1; }
        }
        moved
    }
}

impl Default for Bus {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flightmode::FlightMode;
    use crate::units::*;

    #[test]
    fn ring_fifo_order() {
        let mut r: Ring<u32, 4> = Ring::new();
        assert!(r.is_empty());
        for i in 0..4 {
            assert!(r.try_push(i).is_ok());
        }
        assert!(r.is_full());
        // 满时拒绝
        assert_eq!(r.try_push(99), Err(99));
        // FIFO 弹出
        for i in 0..4 {
            assert_eq!(r.try_pop(), Some(i));
        }
        assert!(r.is_empty());
        assert_eq!(r.try_pop(), None);
    }

    #[test]
    fn ring_wrap_around() {
        let mut r: Ring<u32, 3> = Ring::new();
        for i in 0..3 { assert!(r.try_push(i).is_ok()); }
        assert_eq!(r.try_pop(), Some(0));
        assert_eq!(r.try_pop(), Some(1));
        // 腾出空间后继续推，验证回绕无错乱
        assert!(r.try_push(10).is_ok());
        assert!(r.try_push(11).is_ok());
        assert_eq!(r.try_pop(), Some(2));
        assert_eq!(r.try_pop(), Some(10));
        assert_eq!(r.try_pop(), Some(11));
        assert_eq!(r.try_pop(), None);
    }

    #[test]
    fn bus_est_fanout_no_loss() {
        let mut bus = Bus::new();
        // 发布 3 个 est 状态
        for k in 0..3 {
            let mut st = VehicleState::zero();
            st.pos = [Meter(k as f32), Meter(0.0), Meter(-10.0)];
            assert!(bus.publish_est(st).is_ok());
        }
        // 泵 3 次
        let mut total = 0;
        for _ in 0..3 {
            total += bus.pump();
        }
        // 每个 est 扇出到 2 个订阅者 → 6 次搬运
        assert_eq!(total, 6);
        // 两个订阅者各收到 3 个，顺序一致
        let mut c = 0;
        while let Some(st) = bus.recv_est_ctrl() {
            assert_eq!(st.pos[0].0, c as f32);
            c += 1;
        }
        assert_eq!(c, 3);
        let mut d = 0;
        while let Some(st) = bus.recv_est_fdir() {
            assert_eq!(st.pos[0].0, d as f32);
            d += 1;
        }
        assert_eq!(d, 3);
    }

    #[test]
    fn bus_topics_independent() {
        let mut bus = Bus::new();
        bus.publish_imu(ImuSample { accel: [MeterPerSecondSquared(1.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0)], gyro: [RadianPerSecond(0.0); 3] });
        bus.publish_setpoint(Setpoint::hover([Meter(0.0); 3], Radian(0.0)));
        bus.publish_mode((FlightMode::Position, Health::Nominal));
        assert_eq!(bus.pump(), 3);
        // 各主题互不串扰：imu 里不应含 setpoint/mode
        assert!(bus.recv_imu().is_some());
        assert!(bus.recv_setpoint().is_some());
        assert!(bus.recv_mode().is_some());
        assert!(bus.recv_imu().is_none());
    }

    #[test]
    fn bus_full_rejects_no_overwrite() {
        // imu 容量 8：塞满后第 9 个被拒，已存数据不变（不覆盖最旧）。
        let mut bus = Bus::new();
        for k in 0..8 {
            bus.publish_imu(ImuSample { accel: [MeterPerSecondSquared(k as f32), MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0)], gyro: [RadianPerSecond(0.0); 3] });
        }
        let overflow = ImuSample { accel: [MeterPerSecondSquared(999.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0)], gyro: [RadianPerSecond(0.0); 3] };
        assert_eq!(bus.publish_imu(overflow), Err(overflow));
        // pump 每次每主题最多搬运 1 个；循环至清空。
        let mut total = 0;
        while bus.pump() > 0 { total += 1; }
        assert_eq!(total, 8);
        // 读出顺序仍是最先写入的（未被覆盖）
        let first = bus.recv_imu().unwrap();
        assert_eq!(first.accel[0].0, 0.0);
    }
}
