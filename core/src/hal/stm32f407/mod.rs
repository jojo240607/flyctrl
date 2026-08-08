//! STM32F407 真实寄存器级 HAL。
//!
//! 设计：直接操作 `stm32f4` PAC 寄存器（与 joc-base 风格一致），不开辟堆、不依赖外部 HAL 抽象层。
//! 本模块整体仅在 `feature = "stm32f407"` + `target_arch = "arm"` 下被 `hal/mod.rs` 引入编译；
//! host/SIL 构建完全不触碰这些代码。

pub mod clock;
pub mod uart;
pub mod pwm;
pub mod systick;
pub mod gpio;

pub use clock::clock_init;
pub use uart::{UartLink, usart2_isr};
pub use pwm::PwmEsc;
pub use systick::{SysTick, systick_delay_ms};
