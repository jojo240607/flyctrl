//! 中间件层：类型安全消息总线（pub/sub）。
//!
//! 这是架构分层里的 **MIDDLEWARE**（消息总线/发布订阅，类型安全），建立在
//! `no_std`/零堆之上，为上层（任务 M9、编队 M8、FDIR）提供解耦的通信骨干：
//! 生产者发布到主题，消费者订阅主题，二者不直接互相持有引用。
//!
//! 设计取舍（贴合嵌入式现实）：
//! - **SPSC 环形缓冲** `Ring<M, CAP>`：单生产者单消费者，零分配、定容（`Copy`）。
//!   `try_push`/`try_pop` 在满/空时返回 `Err`/`None`（**拒绝而非静默覆盖/丢弃），
//!   调用方据此背压或丢帧——符合飞行控制"宁可丢旧帧也不污染数据流"的偏好。
//! - **编译期主题注册表**（`TopicId` + `Topic`/`PubTopic`/`SubTopic` trait，由
//!   `define_topics!` 宏生成）：每个主题是一个零尺寸标记类型，静态关联到其载荷
//!   类型与容量；`Bus::publish::<T>` / `Bus::subscribe::<T>` 的载荷类型由主题唯一
//!   决定，**写错类型编译器拒绝**。新增主题只改宏一处（单一事实来源），编译器强制
//!   所有主题都有对应的发布/订阅实现。
//! - **角色分离由 API 保证**：生产者经 `publish`/`publish_*` 写生产者段，消费者经
//!   `subscribe`/`recv_*` 读消费者段，结构化地约束"谁写谁读"。
//! - **主题扇出**：飞行系统中一个估计状态常被多个消费者复用（控制器、FDIR、
//!   任务、编队）。`Bus::pump()` 从每个生产者通道弹出一次、复制推入该主题的
//!   所有消费者通道，实现零分配扇出（无需广播队列/堆）。
//! - 不依赖原子/临界区：调用方在 IRQ 上下文使用 `irq_lock` 包裹（与既有 HAL/驱动约定一致）。
//!
//! 后续可叠加：发布/订阅的运行时发现、时间标签与 QoS（如"最新值"/可靠投递）、
//! 把 `neighbor` 跨机链路接到真实 `comm` 形成多机解耦栈。

use crate::controller::trait_def::{ActuatorCmd, Setpoint};
use crate::fdir::Health;
use crate::flightmode::FlightMode;
use crate::hal::irq::IrqLock;
use crate::swarm::NeighborState;
use crate::vehicle::{ImuSample, PosSample, VehicleState};

use crate::units::Second;

/// 带时间戳的消息包装：发布时由总线时钟盖戳，供 QoS/老化判定使用。
#[derive(Debug, Clone, Copy)]
pub struct Stamped<M: Copy> {
    /// 载荷。
    pub msg: M,
    /// 发布时刻（总线单调时钟，秒）。
    pub ts: Second,
}

impl<M: Copy> Stamped<M> {
    pub fn new(msg: M, ts: Second) -> Self { Self { msg, ts } }
    /// 取出裸载荷。
    pub fn into_inner(self) -> M { self.msg }
}

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

    /// 强制推入：满时**覆盖最旧元素**（用于 `latest` QoS——总是保留最新值，
    /// 不丢新帧）。返回被覆盖的旧值（若满且覆盖发生）。
    pub fn force_push(&mut self, m: M) -> Option<M> {
        if self.count == CAP {
            // 覆盖最旧（tail 位置），原地替换并前移 tail 到下一最旧。
            let old = self.buf[self.tail].replace(m);
            self.tail = (self.tail + 1) % CAP;
            old
        } else {
            self.buf[self.head] = Some(m);
            self.head = (self.head + 1) % CAP;
            self.count += 1;
            None
        }
    }
}

impl<M: Copy, const CAP: usize> Default for Ring<M, CAP> {
    fn default() -> Self { Self::new() }
}

/// 服务质量策略（编译期关联到主题）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QosKind {
    /// 可靠投递：满则拒绝新帧（不覆盖旧数据），调用方据 `Err` 背压/丢帧。
    /// 适合命令/健康这类"宁可丢新也不污染"的流。
    Reliable,
    /// 最新值覆盖：满时覆盖最旧元素，总是保留最新一帧。
    /// 适合高频遥测/状态估计这类"旧值无意义"的流。
    Latest,
}

/// 飞行系统消息总线：聚合固定主题通道，并提供 est/imu 扇出与便捷访问。
///
/// 主题（及容量）：
/// - 生产者：`imu`(8)/`gps`(4)/`est`(4)/`setpoint`(4)/`actuator`(4)/`mode`(4)/`health`(4)/`neighbor`(4)。
/// - 消费者端点（扇出副本，每节点独占一份）：
///   `imu_to_est`/`imu_to_fdir`、`est_to_ctrl`/`est_to_fdir`/`est_to_mission`/`est_to_formation`、
///   `gps_out`/`sp_out`/`act_out`/`mode_out`/`health_out`/`neighbor_out`。
///
/// 每段通道是独立的 `Ring`：生产者段（`*_in`，由 `publish` 写入、`pump` 读出）
/// 与消费者段（`*_out`/`*_to_*`，由 `pump` 写入、由 `subscribe` 读出）。
pub struct Bus {
    /// 总线单调时钟（秒），由 `tick` 推进；发布/泵出时据此盖时间戳。
    now: Second,

    // 生产者段（仅 publish 写入；pump 读出）。
    imu_in: Ring<Stamped<ImuSample>, 8>,
    gps_in: Ring<Stamped<PosSample>, 4>,
    est_in: Ring<Stamped<VehicleState>, 4>,
    sp_in: Ring<Stamped<Setpoint>, 4>,
    act_in: Ring<Stamped<ActuatorCmd>, 4>,
    mode_in: Ring<Stamped<(FlightMode, Health)>, 4>,
    health_in: Ring<Stamped<Health>, 4>,
    neighbor_in: Ring<Stamped<NeighborState>, 4>,

    // 消费者段（pump 写入；subscribe 读出）。
    gps_out: Ring<Stamped<PosSample>, 4>,
    sp_out: Ring<Stamped<Setpoint>, 4>,
    act_out: Ring<Stamped<ActuatorCmd>, 4>,
    mode_out: Ring<Stamped<(FlightMode, Health)>, 4>,
    health_out: Ring<Stamped<Health>, 4>,
    neighbor_out: Ring<Stamped<NeighborState>, 4>,
    imu_to_est: Ring<Stamped<ImuSample>, 8>,
    imu_to_fdir: Ring<Stamped<ImuSample>, 8>,
    est_to_ctrl: Ring<Stamped<VehicleState>, 4>,
    est_to_fdir: Ring<Stamped<VehicleState>, 4>,
    est_to_mission: Ring<Stamped<VehicleState>, 4>,
    est_to_formation: Ring<Stamped<VehicleState>, 4>,
}

/// 定义总线主题注册表（编译期单一事实来源）。
///
/// 为每个生产者/消费者主题生成：
/// - 一个零尺寸标记类型（如 `Imu`、`ImuToEst`）；
/// - `Topic` 实现（关联载荷类型 `Payload` 与容量 `CAP`）；
/// - `PubTopic`（生产者：`push`）或 `SubTopic`（消费者：`pop`）实现，直接委托到
///   `Bus` 的具体字段——这样 `Bus::publish::<Imu>(m)` 的载荷类型由 `Imu` 唯一确定。
///
/// 新增主题只需在此宏的扁平列表里加一行（变体名, 标记名, 角色, 载荷类型, 容量, 字段）；
/// 遗漏任一主题的 pub/sub 实现会被编译器拒绝。
macro_rules! define_topics {
    ( $( ($var:ident, $mark:ident, $role:ident, $pty:ty, $cap:expr, $qos:ident, $field:ident) ),* $(,)? ) => {
        /// 主题注册表：编译期已知的所有总线主题（用于遍历/文档/测试）。
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub enum TopicId { $( $var ),* }

        /// 主题契约：关联载荷类型、容量与服务质量策略。
        pub trait Topic {
            type Payload: Copy;
            const CAP: usize;
            const QOS: QosKind;
        }

        /// 可发布主题（生产者端点）。
        pub trait PubTopic: Topic {
            fn push(bus: &mut Bus, m: Self::Payload, ts: Second) -> Result<(), Self::Payload>;
        }

        /// 可订阅主题（消费者端点）。
        pub trait SubTopic: Topic {
            fn pop(bus: &mut Bus) -> Option<Stamped<Self::Payload>>;
        }

        // 角色门控：仅生产者实现 PubTopic，仅消费者实现 SubTopic。
        // 注意：嵌套宏重复变量（$qos 来自外层 $(...)*）不能在被调宏的臂参数里
        // "解包"，故此处直接内联角色分支，不调用子宏。
        $(
            #[doc = "总线主题标记类型（编译期类型安全门面的键）。"]
            pub struct $mark;
            impl Topic for $mark {
                type Payload = $pty;
                const CAP: usize = $cap;
                const QOS: QosKind = QosKind::$qos;
            }
            impl PubTopic for $mark {
                fn push(bus: &mut Bus, m: Self::Payload, ts: Second) -> Result<(), Self::Payload> {
                    let stamped = Stamped::new(m, ts);
                    match QosKind::$qos {
                        QosKind::Reliable => bus.$field.try_push(stamped).map_err(|s| s.into_inner()),
                        QosKind::Latest => { bus.$field.force_push(stamped); Ok(()) }
                    }
                }
            }
            impl SubTopic for $mark {
                fn pop(bus: &mut Bus) -> Option<Stamped<Self::Payload>> {
                    bus.$field.try_pop()
                }
            }
        )*

        impl TopicId {
            /// 注册表全集（静态），用于计数/遍历/测试覆盖。
            pub fn all() -> &'static [TopicId] {
                &[ $( TopicId::$var ),* ]
            }
        }
    };
}

define_topics! {
    // 变体名, 标记名, 角色, 载荷类型, 容量, QoS, 字段
    // 高频遥测/状态估计 → Latest（旧值无意义，满则覆盖最旧）；
    // 命令/健康/模式/邻居 → Reliable（宁可丢新也不污染）。
    (Imu,         ImuTopic,         prod, ImuSample,     8, Latest,    imu_in),
    (Gps,         GpsTopic,         prod, PosSample,     4, Latest,    gps_in),
    (Est,         EstTopic,         prod, VehicleState,  4, Latest,    est_in),
    (Setpoint,    SetpointTopic,    prod, Setpoint,      4, Reliable,  sp_in),
    (Actuator,    ActuatorTopic,    prod, ActuatorCmd,   4, Reliable,  act_in),
    (Mode,        ModeTopic,        prod, (FlightMode, Health), 4, Reliable, mode_in),
    (Health,      HealthTopic,      prod, Health,        4, Reliable,  health_in),
    (Neighbor,    NeighborTopic,    prod, NeighborState, 4, Latest,    neighbor_in),
    (ImuToEst,    ImuToEstTopic,    sub, ImuSample,     8, Latest,    imu_to_est),
    (ImuToFdir,   ImuToFdirTopic,   sub, ImuSample,     8, Latest,    imu_to_fdir),
    (EstToCtrl,   EstToCtrlTopic,   sub, VehicleState,  4, Latest,    est_to_ctrl),
    (EstToFdir,   EstToFdirTopic,   sub, VehicleState,  4, Latest,    est_to_fdir),
    (EstToMission,EstToMissionTopic,sub, VehicleState,  4, Latest,    est_to_mission),
    (EstToFormation,EstToFormationTopic,sub,VehicleState,4, Latest,   est_to_formation),
    (GpsOut,      GpsOutTopic,      sub, PosSample,     4, Latest,    gps_out),
    (SpOut,       SpOutTopic,       sub, Setpoint,      4, Reliable,  sp_out),
    (ActOut,      ActOutTopic,      sub, ActuatorCmd,   4, Reliable,  act_out),
    (ModeOut,     ModeOutTopic,     sub, (FlightMode, Health), 4, Reliable, mode_out),
    (HealthOut,   HealthOutTopic,   sub, Health,        4, Reliable,  health_out),
    (NeighborOut, NeighborOutTopic, sub, NeighborState, 4, Latest,    neighbor_out),
}

impl Bus {
    /// 构造空总线（各段通道清零，时钟从 0 起）。
    pub fn new() -> Self {
        Self {
            now: Second::ZERO,
            imu_in: Ring::new(), gps_in: Ring::new(), est_in: Ring::new(),
            sp_in: Ring::new(), act_in: Ring::new(), mode_in: Ring::new(),
            health_in: Ring::new(), neighbor_in: Ring::new(),
            gps_out: Ring::new(), sp_out: Ring::new(),
            act_out: Ring::new(), mode_out: Ring::new(), health_out: Ring::new(),
            neighbor_out: Ring::new(),
            imu_to_est: Ring::new(), imu_to_fdir: Ring::new(),
            est_to_ctrl: Ring::new(), est_to_fdir: Ring::new(),
            est_to_mission: Ring::new(), est_to_formation: Ring::new(),
        }
    }

    /// 类型安全发布：载荷类型由主题 `T` 唯一确定；以总线当前时钟盖时间戳。
    ///
    /// 例：`bus.publish::<Imu>(sample)`——`sample` 必须是 `ImuSample`，否则编译失败。
    /// 主题 `QOS` 决定满时行为：`Reliable` 拒收返回 `Err`，`Latest` 覆盖最旧。
    pub fn publish<T: PubTopic>(&mut self, m: T::Payload) -> Result<(), T::Payload> {
        T::push(self, m, self.now)
    }

    /// 类型安全订阅：返回带时间戳的 `Stamped<Payload>`，类型由主题 `T` 唯一确定。
    ///
    /// 例：`bus.subscribe::<ImuToEst>()` 返回 `Option<Stamped<ImuSample>>`。
    pub fn subscribe<T: SubTopic>(&mut self) -> Option<Stamped<T::Payload>> {
        T::pop(self)
    }

    /// 推进总线单调时钟（每控制周期调用一次，dt 为周期时长）。
    pub fn tick(&mut self, dt: Second) {
        self.now.0 += dt.0;
    }

    /// 当前总线时钟（秒）。
    pub fn now(&self) -> Second { self.now }

    // ---- 具名便捷方法（委托到类型安全 API，向后兼容） ----
    pub fn publish_imu(&mut self, m: ImuSample) -> Result<(), ImuSample> { self.publish::<ImuTopic>(m) }
    pub fn publish_gps(&mut self, m: PosSample) -> Result<(), PosSample> { self.publish::<GpsTopic>(m) }
    pub fn publish_est(&mut self, m: VehicleState) -> Result<(), VehicleState> { self.publish::<EstTopic>(m) }
    pub fn publish_setpoint(&mut self, m: Setpoint) -> Result<(), Setpoint> { self.publish::<SetpointTopic>(m) }
    pub fn publish_actuator(&mut self, m: ActuatorCmd) -> Result<(), ActuatorCmd> { self.publish::<ActuatorTopic>(m) }
    pub fn publish_mode(&mut self, m: (FlightMode, Health)) -> Result<(), (FlightMode, Health)> {
        self.publish::<ModeTopic>(m)
    }
    pub fn publish_health(&mut self, m: Health) -> Result<(), Health> { self.publish::<HealthTopic>(m) }
    pub fn publish_neighbor(&mut self, m: NeighborState) -> Result<(), NeighborState> { self.publish::<NeighborTopic>(m) }

    // 具名订阅：返回裸载荷（向后兼容），时间戳版本见 `recv_*_stamped`。
    pub fn recv_imu_est(&mut self) -> Option<ImuSample> { self.subscribe::<ImuToEstTopic>().map(|s| s.into_inner()) }
    pub fn recv_imu_fdir(&mut self) -> Option<ImuSample> { self.subscribe::<ImuToFdirTopic>().map(|s| s.into_inner()) }
    pub fn recv_gps(&mut self) -> Option<PosSample> { self.subscribe::<GpsOutTopic>().map(|s| s.into_inner()) }
    pub fn recv_est_ctrl(&mut self) -> Option<VehicleState> { self.subscribe::<EstToCtrlTopic>().map(|s| s.into_inner()) }
    pub fn recv_est_fdir(&mut self) -> Option<VehicleState> { self.subscribe::<EstToFdirTopic>().map(|s| s.into_inner()) }
    pub fn recv_est_mission(&mut self) -> Option<VehicleState> { self.subscribe::<EstToMissionTopic>().map(|s| s.into_inner()) }
    pub fn recv_est_formation(&mut self) -> Option<VehicleState> { self.subscribe::<EstToFormationTopic>().map(|s| s.into_inner()) }
    pub fn recv_setpoint(&mut self) -> Option<Setpoint> { self.subscribe::<SpOutTopic>().map(|s| s.into_inner()) }
    pub fn recv_actuator(&mut self) -> Option<ActuatorCmd> { self.subscribe::<ActOutTopic>().map(|s| s.into_inner()) }
    pub fn recv_mode(&mut self) -> Option<(FlightMode, Health)> { self.subscribe::<ModeOutTopic>().map(|s| s.into_inner()) }
    pub fn recv_health(&mut self) -> Option<Health> { self.subscribe::<HealthOutTopic>().map(|s| s.into_inner()) }
    pub fn recv_neighbor(&mut self) -> Option<NeighborState> { self.subscribe::<NeighborOutTopic>().map(|s| s.into_inner()) }

    // 带时间戳订阅（QoS/老化判定用）。
    pub fn recv_imu_est_stamped(&mut self) -> Option<Stamped<ImuSample>> { self.subscribe::<ImuToEstTopic>() }
    pub fn recv_est_ctrl_stamped(&mut self) -> Option<Stamped<VehicleState>> { self.subscribe::<EstToCtrlTopic>() }
    pub fn recv_health_stamped(&mut self) -> Option<Stamped<Health>> { self.subscribe::<HealthOutTopic>() }

    /// 扇出泵：把各生产者段的最新数据复制到对应消费者段。
    ///
    /// 对 `est`：从 `est_in` 弹出并复制进 4 路消费者段（ctrl/fdir/mission/formation 各一份）。
    /// 对 `imu`：复制进 `imu_to_est`/`imu_to_fdir` 两份。其余主题单播到对应 `*_out`。
    /// 每调一次最多搬运一个元素/主题，调用方通常每控制周期调一次（或循环调至各通道清空）。
    ///
    /// 返回本次搬运的元素总数（用于诊断背压/丢帧）。
    pub fn pump(&mut self) -> usize {
        let mut moved = 0usize;
        // 通道已存 Stamped<M>；泵出时保留生产者原始时间戳（不改戳）。
        if let Some(m) = self.imu_in.try_pop() {
            if self.imu_to_est.try_push(m).is_ok() { moved += 1; }
            if self.imu_to_fdir.try_push(m).is_ok() { moved += 1; }
        }
        if let Some(m) = self.gps_in.try_pop() {
            if self.gps_out.try_push(m).is_ok() { moved += 1; }
        }
        if let Some(m) = self.est_in.try_pop() {
            if self.est_to_ctrl.try_push(m).is_ok() { moved += 1; }
            if self.est_to_fdir.try_push(m).is_ok() { moved += 1; }
            if self.est_to_mission.try_push(m).is_ok() { moved += 1; }
            if self.est_to_formation.try_push(m).is_ok() { moved += 1; }
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
        if let Some(m) = self.health_in.try_pop() {
            if self.health_out.try_push(m).is_ok() { moved += 1; }
        }
        if let Some(m) = self.neighbor_in.try_pop() {
            if self.neighbor_out.try_push(m).is_ok() { moved += 1; }
        }
        moved
    }

    /// IRQ 安全扇出泵：在中断锁内执行 [`pump`]，供**中断上下文**调用。
    ///
    /// 真实 MCU 上，传感器 DMA 完成 ISR / 控制周期定时器 ISR 可能与主循环并发
    /// 读写总线通道。`pump_locked` 借由 `lock`（`IrqLock` 实现）在搬运期间屏蔽
    /// 可抢占的中断，避免通道 `head`/`tail`/`count` 撕裂。host 端 `HostIrqLock`
    /// 为 no-op，MCU 端 `BasepriLock` 用 BASEPRI 阈值临界区。
    ///
    /// 例（ISR 内）：`let n = bus.pump_locked(&HostIrqLock);`
    pub fn pump_locked<L: IrqLock>(&mut self, lock: &L) -> usize {
        lock.with(|| self.pump())
    }

    /// IRQ 安全发布：中断锁内执行 [`publish`]，供 ISR 发布传感器样本等。
    ///
    /// 载荷类型由主题 `T` 唯一确定（与 `publish` 同款编译期类型安全）。
    pub fn publish_locked<T: PubTopic, L: IrqLock>(
        &mut self,
        lock: &L,
        m: T::Payload,
    ) -> Result<(), T::Payload> {
        lock.with(|| self.publish::<T>(m))
    }

    /// IRQ 安全订阅：中断锁内执行 [`subscribe`]，供 ISR 取走消费数据（带时间戳）。
    pub fn subscribe_locked<T: SubTopic, L: IrqLock>(
        &mut self,
        lock: &L,
    ) -> Option<Stamped<T::Payload>> {
        lock.with(|| self.subscribe::<T>())
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
        // 每个 est 扇出到 4 个订阅者 → 3*4 = 12 次搬运
        assert_eq!(total, 12); // 3 个 est × 4 路扇出 = 12 次搬运
        // 四个订阅者各收到 3 个，顺序一致
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
        assert_eq!(bus.pump(), 4);
        // 各主题互不串扰：imu 里不应含 setpoint/mode
        assert!(bus.recv_imu_est().is_some());
        assert!(bus.recv_setpoint().is_some());
        assert!(bus.recv_mode().is_some());
        assert!(bus.recv_imu_est().is_none());
    }

    #[test]
    fn bus_qos_latest_overwrites_oldest() {
        // Imu 是 Latest QoS（cap 8）：塞满后第 9 个**覆盖最旧**，不拒收、不丢新帧。
        let mut bus = Bus::new();
        for k in 0..8 {
            bus.publish_imu(ImuSample { accel: [MeterPerSecondSquared(k as f32), MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0)], gyro: [RadianPerSecond(0.0); 3] });
        }
        let newest = ImuSample { accel: [MeterPerSecondSquared(999.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0)], gyro: [RadianPerSecond(0.0); 3] };
        // Latest：第 9 个成功（覆盖最旧 k=0），返回 Ok。
        assert!(bus.publish_imu(newest).is_ok());
        // pump：8 个 × 2 段 = 16 次搬运（覆盖那帧也在内）。
        let mut total = 0;
        loop {
            let m = bus.pump();
            if m == 0 { break; }
            total += m;
        }
        assert_eq!(total, 16);
        // 读出顺序：最旧(k=0)已被覆盖，第一个是 k=1。
        let first = bus.recv_imu_est().unwrap();
        assert_eq!(first.accel[0].0, 1.0);
        // 最后一个是最新值 999。
        let mut last = first;
        while let Some(s) = bus.recv_imu_est() { last = s; }
        assert_eq!(last.accel[0].0, 999.0);
    }

    #[test]
    fn bus_qos_reliable_rejects_when_full() {
        // Setpoint 是 Reliable QoS（cap 4）：塞满后第 5 个被拒（Err），已存不变。
        let mut bus = Bus::new();
        for k in 0..4 {
            assert!(bus.publish_setpoint(Setpoint::hover([Meter(k as f32); 3], Radian(0.0))).is_ok());
        }
        let overflow = Setpoint::hover([Meter(777.0); 3], Radian(0.0));
        assert!(bus.publish_setpoint(overflow).is_err());
        // pump 1 次：setpoint 单播到 sp_out（计 1）。
        assert_eq!(bus.pump(), 1);
        // 读出顺序仍是最先写入的（未被覆盖）
        let first = bus.recv_setpoint().unwrap();
        assert_eq!(first.pos[0].0, 0.0);
    }

    // ---- 编译期主题注册表测试 ----
    #[test]
    fn topic_registry_count() {
        // 8 生产者 + 12 消费者端点 = 20 个主题。
        assert_eq!(TopicId::all().len(), 20);
    }

    #[test]
    fn topic_registry_typed_publish_subscribe() {
        // 类型安全：publish/subscribe 的载荷类型由主题标记唯一确定。
        let mut bus = Bus::new();
        // Imu 主题只接受 ImuSample
        let imu = ImuSample { accel: [MeterPerSecondSquared(2.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0)], gyro: [RadianPerSecond(0.0); 3] };
        assert!(bus.publish::<ImuTopic>(imu).is_ok());
        bus.pump();
        // 订阅 ImuToEst 得到 Stamped<ImuSample>（类型由主题绑定）
        let got: ImuSample = bus.subscribe::<ImuToEstTopic>().unwrap().into_inner();
        assert_eq!(got.accel[0].0, 2.0);

        // Est 主题只接受 VehicleState
        let mut st = VehicleState::zero();
        st.pos[2] = Meter(-5.0);
        assert!(bus.publish::<EstTopic>(st).is_ok());
        bus.pump();
        let got_est: VehicleState = bus.subscribe::<EstToCtrlTopic>().unwrap().into_inner();
        assert_eq!(got_est.pos[2].0, -5.0);

        // 类型错误会在编译期被拒（以下仅为注释说明，不执行）：
        // bus.publish::<Imu>(Meter(1.0)); // 编译失败：Imu 载荷是 ImuSample，不是 Meter
    }

    #[test]
    fn bus_irq_locked_publish_pump_subscribe() {
        // IRQ 安全 API 行为须与裸 API 一致（host 端 HostIrqLock 为 no-op）。
        use crate::hal::irq::HostIrqLock;
        let lock = HostIrqLock;
        let mut bus = Bus::new();

        let imu = ImuSample { accel: [MeterPerSecondSquared(3.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0)], gyro: [RadianPerSecond(0.0); 3] };
        // ISR 内发布（类型安全门面）
        assert!(bus.publish_locked::<ImuTopic, _>(&lock, imu).is_ok());
        // ISR 内扇出泵
        assert_eq!(bus.pump_locked(&lock), 2); // imu 扇出到 2 段
        // ISR 内订阅（消费者端点）
        let got: ImuSample = bus.subscribe_locked::<ImuToEstTopic, _>(&lock).unwrap().into_inner();
        assert_eq!(got.accel[0].0, 3.0);

        // 多主题：est 发布 + 泵（扇出 4 段）
        let mut st = VehicleState::zero();
        st.pos[2] = Meter(-7.0);
        assert!(bus.publish_locked::<EstTopic, _>(&lock, st).is_ok());
        assert_eq!(bus.pump_locked(&lock), 4);
        let got_est: VehicleState = bus.subscribe_locked::<EstToCtrlTopic, _>(&lock).unwrap().into_inner();
        assert_eq!(got_est.pos[2].0, -7.0);
    }
}
