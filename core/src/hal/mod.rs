//! 硬件抽象层（HAL）：把传感器/执行器/RTOS 与具体芯片解耦。
//!
//! 设计原则（与 [`crate::estimator`] / [`crate::controller`] 同构）：
//! - 算法核心只依赖这里的 trait，不碰任何寄存器。
//! - 具体芯片（当前 `stm32f407`）提供 trait 实现；host 端提供 `mock` 实现用于单测/SIL。
//! - 全部 `no_std`、无堆分配、执行时间有界。

pub mod sensor;
pub mod actuator;
pub mod rtos;
