//! 执行器 HAL：把控制律输出的 [`ActuatorCmd`] 写到真实电调/电机。
//!
//! 抽象为 `MotorActuator` trait；具体实现分 PWM（标准协议）与 DSHOT（数字协议）。
//! 真实硬件走定时器 PWM/DMAMUX，这里保持接口语义一致、无堆、有界耗时。

use crate::vehicle::ActuatorCmd;

/// 电机执行器接口：把归一化推力指令 [0,1]×4 写到四个 rotor。
pub trait MotorActuator {
    /// 应用一帧控制输出。必须做饱和与零油门保护（解锁前可拒绝）。
    fn apply(&mut self, cmd: &ActuatorCmd);

    /// 全部输出归零（失控保护 / 解锁前）。
    fn disarm(&mut self);

    /// 自检测（ESC 通信健康）。
    fn healthy(&self) -> bool;
}

/// 输出协议类型（用于板级配置与诊断）。
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum OutputProtocol {
    #[default]
    Pwm,    // 标准 1–2ms 模拟脉宽
    Dshot,  // DSHOT600/1200 数字协议
    Can,    // UAVCAN / 总线 ESC
}

/// 把归一化推力 [0,1] 夹到合法区间（防饱和）。
pub fn clamp_thrust(v: f32) -> f32 {
    if v < 0.0 { 0.0 } else if v > 1.0 { 1.0 } else { v }
}

// ─────────────────────────────────────────────────────────────
// Mock 实现（host / SIL）：只记录最近一次指令，供单测断言。
// ─────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, Default)]
pub struct MockMotors {
    last: ActuatorCmd,
    ok: bool,
    protocol: OutputProtocol,
}

impl MockMotors {
    pub fn new(protocol: OutputProtocol) -> Self {
        Self { last: ActuatorCmd::zero(), ok: true, protocol }
    }
    pub fn last_cmd(&self) -> ActuatorCmd { self.last }
    pub fn set_health(&mut self, h: bool) { self.ok = h; }
}

impl MotorActuator for MockMotors {
    fn apply(&mut self, cmd: &ActuatorCmd) {
        let mut m = [0.0f32; 4];
        for i in 0..4 { m[i] = clamp_thrust(cmd.motor[i]); }
        self.last = ActuatorCmd { motor: m };
    }
    fn disarm(&mut self) { self.last = ActuatorCmd::zero(); }
    fn healthy(&self) -> bool { self.ok }
}

// ─────────────────────────────────────────────────────────────
// STM32F407 占位实现
//
// 真实落地：PWM 走 TIMx + DMA  burst（joc-base 已验证 DMA 时序），
// DSHOT 走 TIMx 单线反向 + 位bang/PDM。此处保留骨架。
// ─────────────────────────────────────────────────────────────

#[cfg(feature = "stm32f407")]
pub mod stm32f407 {
    use super::*;

    /// STM32F4 标准 PWM ESC（TIM1_CH1..4 + DMA 更新比较寄存器）。
    pub struct PwmEsc { base: usize, ok: bool }
    impl PwmEsc {
        pub const fn new(base: usize) -> Self { Self { base, ok: true } }
    }
    impl MotorActuator for PwmEsc {
        fn apply(&mut self, cmd: &ActuatorCmd) {
            let _ = self.base;
            // 占位：归一化推力 -> 1000..2000µs 脉宽，写 TIM CCR 经 DMA。
            let _ = cmd;
        }
        fn disarm(&mut self) { let _ = self.base; }
        fn healthy(&self) -> bool { self.ok }
    }

    /// STM32F4 DSHOT ESC（TIMx 单线 + 位bang 帧）。
    pub struct DshotEsc { base: usize, ok: bool }
    impl DshotEsc {
        pub const fn new(base: usize) -> Self { Self { base, ok: true } }
    }
    impl MotorActuator for DshotEsc {
        fn apply(&mut self, cmd: &ActuatorCmd) {
            let _ = self.base;
            // 占位：推力 -> 11bit 油门帧 + CRC，经 TIM 输出比较位流。
            let _ = cmd;
        }
        fn disarm(&mut self) { let _ = self.base; }
        fn healthy(&self) -> bool { self.ok }
    }
}
