//! TIM3 四路 PWM 输出（电机/舵机 ESC 信号）。
//!
//! 通道→引脚（AF2）：CH1=PB4, CH2=PB5, CH3=PB0, CH4=PB1。
//! 定时器时钟 = APB1 ×2 = 84MHz。默认 400Hz PWM（周期 2.5ms）。
//! 占空比以微秒量（1000us=最低, 2000us=最高）表达，内部换算到比较寄存器。

use stm32f4::stm32f407::{RCC, TIM3, GPIOB};
use cortex_m::interrupt::{self, Mutex};
use core::cell::RefCell;

use crate::hal::stm32f407::clock::APB1_HZ;
use crate::hal::stm32f407::gpio::{self, Pin, Port, AltFn};

/// PWM 频率（Hz）。
pub const PWM_HZ: u32 = 400;
/// 油门量程（us）：1000=最低，2000=最高。
pub const THR_MIN_US: u32 = 1000;
pub const THR_MAX_US: u32 = 2000;

/// TIM3 四路 PWM 电机输出。
pub struct PwmEsc {
    tim: &'static TIM3,
    period_us: u32,
}

// 各通道当前占空比（us），供诊断/调试读取。
static CH_US: Mutex<RefCell<[u32; 4]>> = Mutex::new(RefCell::new([THR_MIN_US; 4]));

impl PwmEsc {
    /// 初始化 TIM3 为四路 400Hz PWM。
    ///
    /// 调用方负责提供 `Peripherals` 中的 `TIM3` 与 `RCC` 引用（仅可取一次）。
    pub fn new(tim: &'static TIM3, rcc: &'static RCC) -> Self {
        rcc.apb1enr().modify(|_, w| w.tim3en().set_bit());
        // PB4/PB5/PB0/PB1 → AF2
        gpio::configure_af(rcc, Pin { port: Port::B, pin: 4 }, AltFn(2));
        gpio::configure_af(rcc, Pin { port: Port::B, pin: 5 }, AltFn(2));
        gpio::configure_af(rcc, Pin { port: Port::B, pin: 0 }, AltFn(2));
        gpio::configure_af(rcc, Pin { port: Port::B, pin: 1 }, AltFn(2));

        // 定时器时钟 = APB1 × 2 = 84MHz。预分频到 1MHz（1us 计数）。
        let psc = (APB1_HZ * 2 / 1_000_000) - 1;
        tim.psc().write(|w| unsafe { w.bits(psc) });
        let period = 1_000_000 / PWM_HZ; // 2500 计数 = 2.5ms
        tim.arr().write(|w| unsafe { w.bits(period - 1) });

        // 每通道：PWM 模式 1（CNT<CCR 时有效），预装载使能，输出使能，极性高。
        // CCMR1/CCMR2 类型不同，必须分两支分别 modify。
        tim.ccmr1_output().modify(|r, w| unsafe {
            let bits = r.bits();
            let ocm = 0b110u32 << 4; // CH1 OC1M = 110
            let ocpe = 1u32 << 3;
            let field = (ocm | ocpe) << 0;
            let mask = 0xFFu32 << 0;
            w.bits((bits & !mask) | (field & mask))
        });
        tim.ccmr2_output().modify(|r, w| unsafe {
            let bits = r.bits();
            let ocm = 0b110u32 << 4; // CH3 OC3M = 110
            let ocpe = 1u32 << 3;
            let field = (ocm | ocpe) << 0;
            let mask = 0xFFu32 << 0;
            w.bits((bits & !mask) | (field & mask))
        });
        // 初始占空比 = 最低油门
        for ch in 0..4 {
            let ccr = match ch {
                0 => &tim.ccr1(),
                1 => &tim.ccr2(),
                2 => &tim.ccr3(),
                _ => &tim.ccr4(),
            };
            ccr.write(|w| unsafe { w.bits(THR_MIN_US) });
        }

        tim.ccer()
            .write(|w| w.cc1e().set_bit().cc2e().set_bit().cc3e().set_bit().cc4e().set_bit());
        tim.cr1().write(|w| w.cen().set_bit().arpe().set_bit());

        PwmEsc { tim, period_us: period }
    }

    fn set_channel_us(&self, ch: usize, us: u32) {
        let tim = self.tim;
        let ccr = match ch {
            0 => &tim.ccr1(),
            1 => &tim.ccr2(),
            2 => &tim.ccr3(),
            _ => &tim.ccr4(),
        };
        let v = us.min(self.period_us);
        ccr.write(|w| unsafe { w.bits(v) });
        interrupt::free(|cs| CH_US.borrow(cs).borrow_mut()[ch] = v);
    }

    /// 写入四路油门（us，范围 THR_MIN_US..=THR_MAX_US）。
    pub fn write_us(&self, ch1: u32, ch2: u32, ch3: u32, ch4: u32) {
        self.set_channel_us(0, ch1);
        self.set_channel_us(1, ch2);
        self.set_channel_us(2, ch3);
        self.set_channel_us(3, ch4);
    }

    /// 写入归一化油门 0.0..=1.0。
    pub fn write_norm(&self, c: [f32; 4]) {
        for (i, &v) in c.iter().enumerate() {
            let us = THR_MIN_US + ((THR_MAX_US - THR_MIN_US) as f32 * v.clamp(0.0, 1.0)) as u32;
            self.set_channel_us(i, us);
        }
    }

    /// 读取某通道当前占空比（us）。
    pub fn read_us(ch: usize) -> u32 {
        interrupt::free(|cs| CH_US.borrow(cs).borrow()[ch])
    }
}

// 保留接口：将来若需动态改 ARR 需配置 TIM 写保护，此处占位。
#[allow(dead_code)]
fn _gpio_b_marker() {
    let _ = GPIOB::ptr();
}
