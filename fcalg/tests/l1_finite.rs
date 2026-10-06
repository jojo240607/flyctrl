//! L1 验收 —— 有限性纪律（契约 §4）：接受/拒绝边界 + 计数可观测 + 不静默。
//!
//! ⚠**测试隔离**：`finite` 的计数器是**进程级全局**的，而同一测试二进制内的用例
//! **默认并行** ⇒ `reset()`/`violations()` 会互相踩（非确定性失败，实测撞到过）。
//! 故本文件内所有碰计数器的用例一律先拿共享锁串行化。
//! （设计上的代价：全局计数器对固件方便，对测试需显式串行 —— 这里把它写明，而不是靠运气。）
use std::sync::{Mutex, MutexGuard};
static LOCK: Mutex<()> = Mutex::new(());
fn serial() -> MutexGuard<'static, ()> {
    LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

use fcalg::finite::{gate, gate_all, reset, violations, violations_total, Stage, Violation};

#[test]
fn gate_accepts_finite_and_rejects_nan_inf() {
    let _g = serial();
    reset();
    assert_eq!(gate(Stage::L2ImuFilter, 1.5), Ok(1.5));
    assert_eq!(gate(Stage::L2ImuFilter, f32::NAN), Err(Violation::Nan));
    assert_eq!(gate(Stage::L2ImuFilter, f32::INFINITY), Err(Violation::Inf));
    assert_eq!(gate(Stage::L2ImuFilter, f32::NEG_INFINITY), Err(Violation::Inf));
    // 边界：0 与极值都是"有限" ⇒ 必须放行，不得把"很小"当"坏"
    assert_eq!(gate(Stage::L2ImuFilter, 0.0), Ok(0.0));
    assert_eq!(gate(Stage::L2ImuFilter, f32::MIN_POSITIVE), Ok(f32::MIN_POSITIVE));
    assert_eq!(gate(Stage::L2ImuFilter, f32::MAX), Ok(f32::MAX));
}

#[test]
fn violations_are_counted_per_stage() {
    let _g = serial();
    reset();
    let _ = gate(Stage::L4Align, f32::NAN);
    let _ = gate(Stage::L4Align, f32::NAN);
    let _ = gate(Stage::L9ObsBaro, f32::INFINITY);
    assert_eq!(violations(Stage::L4Align), 2);
    assert_eq!(violations(Stage::L9ObsBaro), 1);
    assert_eq!(violations(Stage::L8Update), 0);
    assert_eq!(violations_total(), 3);
}

/// 逐分量：任一分量坏 ⇒ 整体拒绝。
#[test]
fn gate_all_rejects_on_any_bad_component() {
    let _g = serial();
    reset();
    assert_eq!(gate_all(Stage::L5Propagate, &[0.1, -0.2, 0.3]), Ok(()));
    assert_eq!(gate_all(Stage::L5Propagate, &[0.1, f32::NAN, 0.3]), Err(Violation::Nan));
    assert_eq!(violations(Stage::L5Propagate), 1);
}

/// 不变量：计数单调增，且 `reset` 可清零（测试隔离）。
#[test]
fn counters_are_monotonic_and_resettable() {
    let _g = serial();
    reset();
    assert_eq!(violations_total(), 0);
    let _ = gate(Stage::L7Covariance, f32::NAN);
    let a = violations_total();
    let _ = gate(Stage::L7Covariance, f32::NAN);
    let b = violations_total();
    assert!(b > a, "计数必须单调增: {a} → {b}");
    reset();
    assert_eq!(violations_total(), 0);
}
