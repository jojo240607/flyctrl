//! 硬件抽象层（HAL）：把传感器/执行器/RTOS 与具体芯片解耦。
//!
//! 设计原则（与 [`crate::estimator`] / [`crate::controller`] 同构）：
//! - 算法核心只依赖这里的 trait，不碰任何寄存器。
//! - 具体芯片（当前 `stm32f407`）提供 trait 实现；host 端提供 `mock` 实现用于单测/SIL。
//! - 全部 `no_std`、无堆分配、执行时间有界。

pub mod sensor;
pub mod actuator;
pub mod rtos;
pub mod irq;

/// 真实 STM32F407 寄存器级驱动（仅 `stm32f407` 特性 + arm 目标编译）。
#[cfg(feature = "stm32f407")]
pub mod stm32f407;

/// 当启用 `stm32f407` 特性时，把真实驱动提升到 `hal` 顶层命名空间，
/// 与 `mock` 实现对等，便于算法代码 `use flyctrl_core::hal::UartLink`。
#[cfg(feature = "stm32f407")]
pub use stm32f407::{UartLink, PwmEsc, clock_init, SysTick, usart2_isr};
