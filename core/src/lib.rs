//! flyctrl-core: 飞控核心逻辑（no_std 友好，无堆分配）。
//!
//! 设计原则：
//! - 所有物理量使用类型安全单位（见 [`units`]），杜绝单位混用类 bug。
//! - 飞控状态机用类型级编码（见 [`state`]），非法状态转换在编译期不可达。
//! - 估计算法与控制律均通过 trait 抽象（[`estimator::Estimator`] /
//!   [`controller::Controller`]），可运行时/编译期替换，便于横向对比。

#![no_std]

pub mod math;
pub mod units;
pub mod state;
pub mod vehicle;
pub mod config;
pub mod estimator;
pub mod controller;
pub mod fdir;
pub mod flightmode;
pub mod hil;
pub mod invariants;
pub mod mission;
pub mod swarm;
pub mod hal;
pub mod comm;
