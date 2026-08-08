//! RCC 时钟初始化：HSE=8MHz → PLL → SYSCLK=168MHz，HCLK=168/AHB=1, PCLK1=42(Cortex-M4 max 42),
//! PCLK2=84。与 joc-base 的 clock_hal.c 同款配置。APB 预分频器会把定时器时钟翻倍（×2）。
//!
//! 对外暴露 `SystemHz` 常量供 UART/PWM 波特率与 PWM 频率计算使用。

use stm32f4::stm32f407::RCC;

/// 系统主频（Hz）。
pub const SYSTEM_HZ: u32 = 168_000_000;
/// APB1 外设时钟（PCLK1, Hz）——定时器在其上 ×2 = 84MHz。
pub const APB1_HZ: u32 = 42_000_000;
/// APB2 外设时钟（PCLK2, Hz）。
pub const APB2_HZ: u32 = 84_000_000;

/// 初始化系统时钟。假定板载 HSE = 8MHz（Discovery 默认）。
///
/// 安全：仅在 `cortex-m-rt` 复位后、任何外设访问前调用一次。
pub fn clock_init(rcc: &RCC) {
    // 1) 使能 HSE 并等待就绪
    rcc.cr().modify(|_, w| w.hseon().set_bit());
    while rcc.cr().read().hserdy().bit_is_clear() {}

    // 2) 配置闪存延迟（168MHz 需要 5 WS）
    unsafe {
        let f = &*stm32f4::stm32f407::FLASH::ptr();
        f.acr()
            .modify(|_, w| w.latency().bits(5).icen().set_bit().dcen().set_bit());
    }

    // 3) 配置 PLL: PLLM=8, PLLN=336, PLLP=2 → 8/8*336/2 = 168MHz
    //    PLLQ=7 → 48MHz 供 OTG FS（若后续接 USB CDC 需要）
    rcc.pllcfgr().modify(|_, w| unsafe {
        w.pllsrc().set_bit()      // HSE
         .pllm().bits(8)          // /8  → 1MHz VCO 输入
         .plln().bits(336)        // ×336 → 336MHz VCO
         .pllp().div2()           // /2  → 168MHz SYSCLK
         .pllq().bits(7)          // /7  → 48MHz
    });
    rcc.cr().modify(|_, w| w.pllon().set_bit());
    while rcc.cr().read().pllrdy().bit_is_clear() {}

    // 4) 总线分频：AHB=1, APB1=/4(42MHz), APB2=/2(84MHz)
    rcc.cfgr().modify(|_, w| unsafe {
        w.hpre().bits(0b0000)        // AHB /1
         .ppre1().bits(0b101)        // APB1 /4
         .ppre2().bits(0b100)        // APB2 /2
    });

    // 5) 切换系统时钟到 PLL
    rcc.cfgr().modify(|_, w| w.sw().pll());
    while !rcc.cfgr().read().sws().is_pll() {}
}
