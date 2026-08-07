//! 运行时抽象：把"主控制循环如何被调度"与算法解耦。
//!
//! 真实飞控跑在 RTOS（如 joc-base 的 Rust RTOS / FreeRTOS）上，由定时器/调度器
//! 周期性唤醒控制任务。此处定义 [`Runtime`] trait，host 端提供 [`host::SpinLoop`]，
//! 嵌入式端可接入具体 RTOS 的任务/定时器原语。算法核心只调用 `schedule_periodic`。
//!
//! 所有实现必须保证：`tick` 回调在固定周期 `dt` 内被调用，且执行时间有界。

use crate::units::Second;

/// 周期性任务回调：每个控制周期执行一次。
pub type TickFn = fn(Second);

/// 运行时调度抽象。
pub trait Runtime {
    /// 注册并以固定周期 `dt` 启动主控制循环（`tick` 在每个周期被调用）。
    ///
    /// `iterations == 0` 表示无限运行（真实飞控）；host 测试可设有限次数。
    fn schedule_periodic(&mut self, dt: Second, iterations: usize, tick: TickFn);

    /// 进入失控保护（由 FDIR 触发）：停止常规调度、归零执行器。
    fn enter_failsafe(&mut self);

    /// 当前运行时间（秒），用于日志/超时判定。
    fn now(&self) -> Second;
}

// ─────────────────────────────────────────────────────────────
// Host 实现：自旋循环 + 简单忙等，贴近 SIL 行为。
// ─────────────────────────────────────────────────────────────

pub mod host {
    use super::*;

    /// Host 自旋运行时：每个 `dt` 调用一次 `tick`，可选有限迭代次数。
    pub struct SpinLoop {
        elapsed: f32,
        failed: bool,
    }

    impl SpinLoop {
        pub fn new() -> Self { Self { elapsed: 0.0, failed: false } }
        pub fn failed(&self) -> bool { self.failed }
    }

    impl Default for SpinLoop { fn default() -> Self { Self::new() } }

    impl Runtime for SpinLoop {
        fn schedule_periodic(&mut self, dt: Second, iterations: usize, tick: TickFn) {
            let steps = if iterations == 0 { 1 << 30 } else { iterations };
            for _ in 0..steps {
                if self.failed { break; }
                tick(dt);
                self.elapsed += dt.0;
            }
        }
        fn enter_failsafe(&mut self) { self.failed = true; }
        fn now(&self) -> Second { Second(self.elapsed) }
    }
}

// ─────────────────────────────────────────────────────────────
// STM32F407 占位实现
//
// 真实落地：控制任务由 RTOS 周期唤醒（如 joc-base 的 rtos_task_create +
// rtos_msleep / 定时器 ISR 触发），此处保留骨架。
// ─────────────────────────────────────────────────────────────

#[cfg(feature = "stm32f407")]
pub mod stm32f407 {
    use super::*;

    /// STM32F4 上基于 RTOS 定时器的周期性控制任务。
    pub struct RtosTask { period_ms: u32, failed: bool }
    impl RtosTask {
        pub const fn new(period_ms: u32) -> Self { Self { period_ms, failed: false } }
    }
    impl Runtime for RtosTask {
        fn schedule_periodic(&mut self, dt: Second, _iterations: usize, tick: TickFn) {
            let _ = (self.period_ms, dt);
            // 占位：rtos_task_create(..., prio=CONTROL_PRIO)；任务体内
            // `rtos_msleep(self.period_ms)` 后调用 tick(dt)。无限运行。
            let _ = tick;
        }
        fn enter_failsafe(&mut self) { self.failed = true; }
        fn now(&self) -> Second { Second(0.0) }
    }
}
