//! SysTick 裸机运行时：提供毫秒延时与一个周期性调度循环。
//!
//! 不依赖外部 RTOS。SysTick 配置为 1kHz（1ms 节拍）。`delay_ms` 用忙等实现；
//! `run_periodic` 以固定周期调用用户回调，实现与 `hal::Rtos::run` 等价的控制环路。

use cortex_m::peripheral::SYST;
use cortex_m::peripheral::syst::SystClkSource;
use crate::hal::stm32f407::clock::SYSTEM_HZ;

/// SysTick 控制句柄（单例）。
pub struct SysTick {
    syst: SYST,
}

impl SysTick {
    /// 以 1ms 节拍初始化并启动 SysTick。
    pub fn start(mut syst: SYST) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        // 重载值 = 每节拍周期数 - 1；内核 168MHz / 1000 = 168000。
        syst.set_reload((SYSTEM_HZ / 1000) - 1);
        syst.clear_current();
        syst.enable_counter();
        syst.enable_interrupt();
        SysTick { syst }
    }

    /// 忙等 `ms` 毫秒（独立取一份 SYST 实例，不依赖中断）。
    pub fn delay_ms(ms: u32) {
        let mut syst = cortex_m::peripheral::Peripherals::take()
            .map(|p| p.SYST)
            .expect("SYST already taken");
        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload((SYSTEM_HZ / 1000) - 1);
        syst.clear_current();
        syst.enable_counter();
        for _ in 0..ms {
            while !syst.has_wrapped() {}
            syst.clear_current();
        }
        syst.disable_counter();
    }

    /// 以固定 `period_ms` 运行控制环路：每周期调用 `f(period_ms)`。
    ///
    /// 用忙等节拍实现（适于裸机单环路飞控）。若需并发任务，应改用
    /// `enable_interrupt()` + 在 `#[interrupt] fn SysTick()` 中累加计数器并在主循环判定。
    pub fn run_periodic<F>(&mut self, period_ms: u32, mut f: F)
    where
        F: FnMut(u32),
    {
        self.syst.clear_current();
        loop {
            for _ in 0..period_ms {
                while !self.syst.has_wrapped() {}
                self.syst.clear_current();
            }
            f(period_ms);
        }
    }
}

/// 模块级毫秒忙等（convenience）。
pub fn systick_delay_ms(ms: u32) {
    SysTick::delay_ms(ms);
}
