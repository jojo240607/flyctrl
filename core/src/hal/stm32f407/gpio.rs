//! GPIO 复位时钟使能与复用功能配置（最简子集，够 USART/PWM 用）。

use stm32f4::stm32f407::{RCC, GPIOA, GPIOB, GPIOC, GPIOE};

/// GPIO 端口枚举。
#[derive(Clone, Copy)]
pub enum Port {
    A,
    B,
    C,
    E,
}

/// 单引脚标识。
#[derive(Clone, Copy)]
pub struct Pin {
    pub port: Port,
    pub pin: u8,
}

/// 复用功能编号（AF0..AF15）。
#[derive(Clone, Copy)]
pub struct AltFn(pub u8);

/// 使能某端口的 GPIO 时钟。必须在配置引脚前调用。
pub fn enable_clock(rcc: &RCC, port: Port) {
    rcc.ahb1enr().modify(|_, w| match port {
        Port::A => w.gpioaen().set_bit(),
        Port::B => w.gpioben().set_bit(),
        Port::C => w.gpiocen().set_bit(),
        Port::E => w.gpioeen().set_bit(),
    });
}

/// 把引脚配置为复用推挽输出，并选择 AF。调用方需提供 `RCC`（仅可取一次）。
pub fn configure_af(rcc: &RCC, pin: Pin, af: AltFn) {
    enable_clock(rcc, pin.port);

    // 在匹配的各端口寄存器块上做同样的位域配置。
    macro_rules! cfg_port {
        ($gpio:expr) => {{
            let block = unsafe { &*$gpio };
            let n = pin.pin as usize;
            block
                .moder()
                .modify(|r, w| unsafe { w.bits(rmw(r.bits(), 2 * n, 2, 0b10)) });
            block
                .otyper()
                .modify(|r, w| unsafe { w.bits(rmw(r.bits(), n, 1, 0b0)) });
            block
                .ospeedr()
                .modify(|r, w| unsafe { w.bits(rmw(r.bits(), 2 * n, 2, 0b10)) });
            block
                .pupdr()
                .modify(|r, w| unsafe { w.bits(rmw(r.bits(), 2 * n, 2, 0b00)) });
            if n < 8 {
                block
                    .afrl()
                    .modify(|r, w| unsafe { w.bits(rmw(r.bits(), 4 * n, 4, af.0 as u32)) });
            } else {
                block.afrh().modify(|r, w| unsafe {
                    w.bits(rmw(r.bits(), 4 * (n - 8), 4, af.0 as u32))
                });
            }
        }};
    }

    match pin.port {
        Port::A => cfg_port!(GPIOA::ptr()),
        Port::B => cfg_port!(GPIOB::ptr()),
        Port::C => cfg_port!(GPIOC::ptr()),
        Port::E => cfg_port!(GPIOE::ptr()),
    }
}

/// 在 `v` 的 `[pos, pos+len)` 置为 `val`（低位对齐），其余位保持不变。
fn rmw(v: u32, pos: usize, len: usize, val: u32) -> u32 {
    let mask = ((1u32 << len) - 1) << pos;
    (v & !mask) | ((val << pos) & mask)
}
