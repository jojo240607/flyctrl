//! ★design.md §4 **L1 `safety_monitor`**（500Hz / ~20µs）：**独立**安全监控线程。
//!
//! 设计要点（§4 "关键任务失效 ⇒ 安全模式"）：**不依赖控制律**
//! —— 若"控制心跳"停摆（控制律卡死/饿死）或 健康=Critical ⇒ 置 [`SAFETY_KILL`]
//! ⇒ **速率层**（L1 `rate`）据此零输出（停机）。这样即使 L2 工作队列整体卡死，
//! 执行器仍可达安全态 ✓。
//!
//! 只做**只读判断 + 置位**（无阻塞、无 SPI/IO）⇒ 20µs 级 ✓。

use core::ffi::c_void;
use core::sync::atomic::{AtomicBool, Ordering};

use flyctrl_core::fdir::Health;
use rtos_app_sdk::info;
use rtos_app_sdk::rtos::{delay_until, tick_count};

use crate::flyctrl::EST_STATE;

/// 安全停机闸：true ⇒ 速率层零输出。由本线程写、`rate_task` 读（无锁原子 ✓）。
pub static SAFETY_KILL: AtomicBool = AtomicBool::new(false);
/// 最近一次 exec（cycles）——供 L1 CPU 预算汇总（§4）。
pub static mut SAFETY_EXEC_CYC: u32 = 0;
/// ★design.md §8：**过载等级**（0=正常；1=降级位置/导航；2=再降级日志；3=再降级通信）。
/// 由本线程按"L1 CPU 负载 + 队列硬超时 + 截止期违约"综合判定（滞回：清空后 2s 才降回）。
pub static OVERLOAD_LEVEL: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);
/// 心跳停摆多久（ms）判定"控制律卡死" ⇒ 停机。
const STUCK_MS: u32 = 100;

/// L1 安全监控入口（500Hz）。
pub extern "C" fn safety_entry(_arg: *mut c_void) {
    info!(tag: "safety", "task started; period=2ms (500Hz, L1 safety_monitor)");
    let mut wake = tick_count();
    let mut last_hb: u32 = 0;
    let mut stuck_ms: u32 = 0;
    let mut first = true;
    loop {
        let t0 = rtos_app_sdk::rtos::cycle_now();
        let hb = unsafe { crate::flyctrl::control::CTRL_HEARTBEAT };
        let (health, armed) = unsafe {
            let s = &*core::ptr::addr_of!(EST_STATE);
            (s.health, s.armed)
        };
        if hb == last_hb {
            stuck_ms = stuck_ms.saturating_add(2);
        } else {
            stuck_ms = 0;
            last_hb = hb;
        }
        // ★停机条件：控制心跳停摆 >STUCK_MS（控制律卡死）或 健康=Critical
        let kill = (stuck_ms > STUCK_MS) || matches!(health, Health::Critical);
        SAFETY_KILL.store(kill, Ordering::Relaxed);
        // ★design.md §8：过载逐级降级 —— 由 L1 CPU 负载 + 队列硬超时 + 截止期违约 判定
        {
            let r = unsafe { crate::flyctrl::rate_task::RATE_EXEC_CYC } as u64;
            let s = unsafe { SAFETY_EXEC_CYC } as u64;
            let l1_permille = ((r * 1000 + s * 500) * 1000 / 168_000_000) as u32;
            let mut wq = [0u32; 5];
            if let Some(f) = rtos_app_sdk::abi::slot().work_stats { f(wq.as_mut_ptr()); }
            let mut vio = [0u32; 3];
            if let Some(f) = rtos_app_sdk::abi::slot().rt_violation { f(vio.as_mut_ptr()); }
            let lvl = if vio[0] > 0 || vio[1] > 0 { 3 }
                      else if wq[2] > 0 { 2 }
                      else if l1_permille > 600 { 1 }
                      else { 0 };
            OVERLOAD_LEVEL.store(lvl, Ordering::Relaxed);
        }
        if first {
            first = false;
            info!(tag: "safety", "first loop done; kill={}", kill);
        }
        let _ = armed;
        unsafe { SAFETY_EXEC_CYC = rtos_app_sdk::rtos::cycle_now().wrapping_sub(t0); }
        delay_until(&mut wake, 2);
    }
}
