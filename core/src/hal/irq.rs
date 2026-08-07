//! 中断安全临界区原语（IRQ lock）。
//!
//! 飞控在 MCU 上运行于中断驱动环境：传感器 DMA 完成 ISR、控制周期定时器 ISR
//! 都可能在主循环读写 [`crate::bus::Bus`] 时抢占它。总线通道是普通 `Ring`，
//! 非原子；若 ISR 与主循环并发写同一通道，会出现撕裂的 `head`/`tail`/`count`
//! 并导致丢失或重复。
//!
//! 解决：所有跨 IRQ/主循环的 `Bus` 访问都用 [`IrqLock`] 包裹——进入时屏蔽
//! 相关优先级中断，退出时恢复。这与 joc-base RTOS 的 `rtos_crit_enter/exit`
//! （BASEPRI 阈值语义）一致：屏蔽前保存掩码、解除时原样恢复，保证嵌套临界区
//! 不破坏外层屏蔽态。
//!
//! 设计约束（贴合 §7 执行纪律）：
//! - `no_std`、零堆、执行时间有界；
//! - host/SIL 端为单线程自旋，无需真屏蔽，但仍保留调用契约（便于代码在
//!   MCU 上与 host 上共用同一份结构）。

/// 中断锁抽象：进入临界区时屏蔽可抢占总线访问的中断，退出时恢复原掩码。
///
/// 实现方必须保证：`enter` 返回的守卫 `Drop` 时（或用 `with` 闭包）原掩码被
/// 恢复——即使闭包内 `panic!` 也要恢复，否则会永久关中断导致系统假死。
pub trait IrqLock {
    /// 在临界区内执行 `f`，期间屏蔽相关中断。`f` 返回的值透传。
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R;
}

/// Host（SIL）实现：单线程自旋，无需真正屏蔽中断。
///
/// 保留 `with` 契约，使 host 与 MCU 端调用 `Bus` 的代码完全一致。若未来 host
/// 端引入多线程测试，可在此替换为真实互斥（仍零堆）。
pub struct HostIrqLock;

impl IrqLock for HostIrqLock {
    #[inline(always)]
    fn with<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        // 单线程：直接执行。MCU 端此处会插入 BASEPRI 掩码/恢复。
        f()
    }
}

#[cfg(feature = "stm32f407")]
pub mod stm32f407 {
    //! STM32F4 中断锁：基于 BASEPRI 阈值（与 joc-base RTOS 临界区同语义）。
    //!
    //! 控制总线访问需要屏蔽的最高优先级 = `RTOS_MAX_ZERO_LATENCY_IRQS`（默认 4）
    //! 之上的可抢占中断。此处阈值设为 `4 << (8 - NVIC_PRIO_BITS)`（NVIC_PRIO_BITS=4）。
    //! 进入：`mrs prev, BASEPRI` 保存当前掩码 → `msr BASEPRI, 阈值`；
    //! 退出：`msr BASEPRI, prev`（**绝不无条件清零**，保护外层临界区）。

    use super::IrqLock;

    /// 控制总线临界区需屏蔽的优先级阈值（BASEPRI 编码，未移位值 4 左移 4 位）。
    const BUS_LOCK_PRIO: u32 = 4u32 << 4;

    /// STM32F4 BASEPRI 中断锁。零成本（两条 MSR + 一条 MRS）。
    pub struct BasepriLock;

    #[cfg(target_arch = "arm")]
    impl BasepriLock {
        /// 保存当前 BASEPRI 并提升到总线锁阈值。
        #[inline(always)]
        fn enter() -> u32 {
            let prev: u32;
            unsafe {
                core::arch::asm!(
                    "mrs {prev}, BASEPRI",
                    "msr BASEPRI, {thr}",
                    prev = out(reg) prev,
                    thr = in(reg) BUS_LOCK_PRIO,
                    options(nomem, preserves_flags),
                );
            }
            prev
        }

        /// 恢复原 BASEPRI（由 `enter` 返回的值）。
        #[inline(always)]
        fn exit(prev: u32) {
            unsafe {
                core::arch::asm!(
                    "msr BASEPRI, {prev}",
                    prev = in(reg) prev,
                    options(nomem, preserves_flags),
                );
            }
        }
    }

    // host（非 arm，如 x86_64 SIL 验证 `--features stm32f407` 编译）退化为 no-op，
    // 与既有 stm32f407 占位模块风格一致：真实寄存器操作仅在 arm target 生效。
    #[cfg(not(target_arch = "arm"))]
    impl BasepriLock {
        #[inline(always)]
        fn enter() -> u32 { 0 }
        #[inline(always)]
        fn exit(_prev: u32) {}
    }

    impl IrqLock for BasepriLock {
        #[inline(always)]
        fn with<F, R>(&self, f: F) -> R
        where
            F: FnOnce() -> R,
        {
            let prev = Self::enter();
            // 即使 f panic，drop guard 也保证恢复 BASEPRI，避免永久关中断。
            let _guard = ScopeGuard(prev);
            let r = f();
            // 显式恢复（正常路径）；drop guard 覆盖 panic 路径。
            Self::exit(prev);
            r
        }
    }

    /// 退出时恢复 BASEPRI 的栈上守卫（panic 安全）。
    struct ScopeGuard(u32);
    #[inline(always)]
    fn scopeguard(prev: u32) -> ScopeGuard { ScopeGuard(prev) }
    impl Drop for ScopeGuard {
        fn drop(&mut self) { BasepriLock::exit(self.0); }
    }
}
