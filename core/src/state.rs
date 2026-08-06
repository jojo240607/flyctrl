//! 飞控状态机——类型级编码，非法状态转换在编译期不可达。
//!
//! 传统飞控用 `enum { ARMED, DISARMED, FAILSAFE, CALIBRATING }` + 一堆 `if` 守卫，
//! "未校准就解锁"、"失控保护中还能解锁"等危险组合靠运行时检查。
//! 这里用 Rust 类型系统把状态编码进类型参数：只有持有特定状态类型的实例
//! 才能调用对应操作，编译器保证非法流转不存在。

use crate::units::Second;

/// 状态机主体。类型参数 `S` 是当前状态类型。
pub struct Fcs<S> {
    pub armed_time: Second,
    _state: core::marker::PhantomData<S>,
}

// ---- 状态类型标记 ----
pub struct Disarmed;
pub struct Calibrating;
pub struct Armed;
pub struct Failsafe;

impl Fcs<Disarmed> {
    pub fn new() -> Self {
        Self { armed_time: Second::ZERO, _state: core::marker::PhantomData }
    }
    /// 只有 Disarmed 才能进入校准。
    pub fn start_calibration(self) -> Fcs<Calibrating> {
        Fcs { armed_time: Second::ZERO, _state: core::marker::PhantomData }
    }
    /// 校准完成后才能解锁（防止未校准就起飞）。
    pub fn arm(self) -> Fcs<Armed> {
        Fcs { armed_time: Second::ZERO, _state: core::marker::PhantomData }
    }
}

impl Fcs<Calibrating> {
    /// 校准完成回到锁定态（不能从校准直接解锁）。
    pub fn finish(self) -> Fcs<Disarmed> {
        Fcs { armed_time: Second::ZERO, _state: core::marker::PhantomData }
    }
}

impl Fcs<Armed> {
    pub fn tick(&mut self, dt: Second) { self.armed_time = Second(self.armed_time.0 + dt.0); }
    /// 解锁态可因失控进入保护（单向，除非人工重置）。
    pub fn failsafe(self) -> Fcs<Failsafe> {
        Fcs { armed_time: self.armed_time, _state: core::marker::PhantomData }
    }
    /// 正常上锁。
    pub fn disarm(self) -> Fcs<Disarmed> {
        Fcs { armed_time: Second::ZERO, _state: core::marker::PhantomData }
    }
}

impl Fcs<Failsafe> {
    /// 失控保护只能由人工（地面站）显式复位回锁定态。
    pub fn reset(self) -> Fcs<Disarmed> {
        Fcs { armed_time: Second::ZERO, _state: core::marker::PhantomData }
    }
}
