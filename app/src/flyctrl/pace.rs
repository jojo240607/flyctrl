//! 控制节拍源：**硬件定时器 + ISR + 信号量**（对齐 PX4 `hrt_call_every`）。
//!
//! ## 为什么需要它（见 `docs/c1-migration-plan.md` §5.91–5.94）
//!
//! 软件侧全部节拍原语（`msleep` / `tick_count` / `delay_until` /
//! `rtos_sleep_until_abs`）都落在 **1ms 系统 tick 网格**上。控制周期 4ms 一旦被该
//! 网格取整，就会退化成 4ms/5ms 交替（实测均值 **4.202ms ⇒ ≈239Hz** ✗，而目标
//! 是 4.000ms/250Hz）。
//!
//! 这里改用板级 **`timer3`（TIM7，basic timer，IRQ55）**：其周期由定时器**自己的
//! 84MHz 时钟域**决定（`timer_config_t.tick_hz = 250` ⇒ ARR 精确到 4.000ms ✓），
//! 不受 1ms 系统 tick 限制；唤醒相位**锚定**、无累积漂移。控制任务在信号量上
//! **阻塞等待**（不忙等 ✗），所以也不再有 `delay_until` 那种自旋开销。
//!
//! ## 依赖的既有事实（已核实 ✓）
//! · `irq.c:15`「a line fires ⇒ `irq_dispatch()` invokes **EVERY** handler」⇒ 本模块
//!   的 ISR 与 timer 驱动自身的 ISR **共存** ✓（UIF 由驱动清 ✓，我们只 `sem_give`）。
//! · app 侧注册 ISR 的先例：`flyctrl/app/src/intg_test.rs` Test I（TIM5 → `sem_give`）。
//! · `mcusimulater/src/peripheral/timer.rs` 建模 UIF/DIER.UIE 并挂共享 NVIC ⇒ 该路径
//!   在 M 场会**真实执行** ✓。
//!
//! ## ★铁律：不依赖 static 初始化器
//! 本案固件由 **raw bin** 载荷（`mcu_simulater/src/artifact.rs`）⇒ `.data` 初值不可
//! 靠（本 session 的 ESKF 诊断开关就栽在这里 ✗）。因此 `PACE_READY` 等状态**一律在
//! `init()` 里显式赋值**，绝不写在 `static` 初始化器里。

use core::ffi::c_void;
use core::ptr::{addr_of_mut, null_mut};

use rtos_app_sdk::abi::{g_app_slot, rtos_sem_t};
use rtos_app_sdk::device::Device;
use rtos_app_sdk::irq;
use rtos_app_sdk::ioctl;

/// 板级设备名（`joc-base/src/board/stm32f4_discovery.c` 的 `g_timer3` ⇒ TIM7）。
const PACE_DEV: &str = "timer3";
/// TIM7 在 STM32F407 上的 IRQ 号（`joc-base/src/hal/*/tim_hal.c` 的 TIM7_IRQn = 55）。
const TIM7_IRQN: u8 = 55;

/// 计数信号量：ISR 里 `give`，控制任务里 `wait`（上限 1 ⇒ 密集事件自动合并 ✓）。
static mut PACE_SEM: rtos_sem_t = rtos_sem_t { count: 0, limit: 0, waitq: null_mut() };
/// 节拍源是否可用（★在 `init()` 里显式赋值，不依赖 static 初值）。
static mut PACE_READY: bool = false;
/// 已收到的节拍数（诊断用；证明 ISR 真的在跑 ✓）。
static mut PACE_TICKS: u32 = 0;

/// 定时器溢出 ISR（ISR 上下文，只做 `sem_give` —— ISR 安全 ✓）。
///
/// UIF **不由我们清**：timer 驱动在 open 时已把自己注册到同一条 IRQ 线，驱动 ISR
/// 负责清 UIF ✓；`irq_dispatch` 会调用该线全部 handler ✓。
extern "C" fn pace_isr(_ctx: *mut c_void) {
    unsafe {
        PACE_TICKS = PACE_TICKS.wrapping_add(1);
        if let Some(f) = g_app_slot.sem_give {
            f(addr_of_mut!(PACE_SEM));
        }
    }
}

/// 拉起节拍源。返回 `true` = 精确节拍可用；`false` = 调用方必须**回退** `delay_until`
/// （反静默降级 ✓：绝不"看起来在工作"却其实没在节拍）。
pub fn init() -> bool {
    unsafe {
        PACE_READY = false;
        PACE_TICKS = 0;
        if let Some(f) = g_app_slot.sem_init {
            f(addr_of_mut!(PACE_SEM), 0, 1);
        }
    }

    let mut dev = match Device::open(PACE_DEV) {
        Some(d) => d,
        None => return false,
    };
    if dev.open_dev() != 0 {
        return false;
    }

    // 顺序固定：**先挂 handler 并使能该线**（避免"边沿先到、handler 未装"的空窗 ✗），
    // 再启动计数器。
    if irq::attach_and_enable(TIM7_IRQN, pace_isr, null_mut()) != 0 {
        return false;
    }
    // TIMER_IOCTL_ENABLE：启动计数器并 arm NVIC（驱动自身的 ISR 亦被使能 ✓）。
    if dev.ioctl(ioctl::TIMER_IOCTL_ENABLE, null_mut()) != 0 {
        return false;
    }

    unsafe { PACE_READY = true; }
    true
}

/// 阻塞等待下一个硬件节拍（周期 = 板级 `tick_hz` = 250Hz ⇒ 4.000ms ✓）。
#[inline]
pub fn wait_tick() {
    unsafe {
        if let Some(f) = g_app_slot.sem_wait {
            f(addr_of_mut!(PACE_SEM));
        }
    }
}

/// 节拍源是否已就绪（`init()` 成功后为真）。
#[inline]
pub fn is_ready() -> bool {
    unsafe { PACE_READY }
}

/// 已收到的节拍数（诊断：非零即证明 ISR 真的在跑 ✓ —— 仪器的自检 ✓）。
#[inline]
pub fn ticks() -> u32 {
    unsafe { PACE_TICKS }
}
