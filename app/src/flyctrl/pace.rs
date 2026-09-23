//! 任务节拍源：**硬件定时器 + ISR + 信号量**（对齐 PX4 `hrt_call_every`）。
//!
//! ## 为什么需要它（见 `docs/c1-migration-plan.md` §5.91–5.98）
//!
//! 软件侧全部节拍原语（`msleep` / `tick_count` / `delay_until` /
//! `rtos_sleep_until_abs`）都落在 **1ms 系统 tick 网格**上，且 `delay_until`
//! 在"已超期"时靠内核重同步 ⇒ 唤醒相位受网格取整影响 ✗。
//!
//! 这里改用**板级 timer 设备**：周期由定时器**自己的时钟域**决定（168/84MHz，
//! 由 `timer_config_t.tick_hz` 精确算出 ✓），不受 1ms 系统 tick 限制；唤醒相位
//! 锚定、无累积漂移；任务在信号量上**阻塞等待**（不忙等 ✗）。
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
//! 靠（本 session 的 ESKF 诊断开关就栽在这里 ✗）。因此各状态**一律在 `init()` 里
//! 显式赋值**，绝不写在 `static` 初始化器里。

use core::ffi::c_void;
use core::ptr::{addr_of_mut, null_mut};

use rtos_app_sdk::abi::{g_app_slot, rtos_sem_t};
use rtos_app_sdk::device::Device;
use rtos_app_sdk::irq;
use rtos_app_sdk::ioctl;

/// 实例化一个节拍源：`$name` 为模块名，`$dev` 板级设备名，`$irq` 其 IRQ 号。
///
/// 生成 `init()` / `wait_tick()` / `is_ready()` / `ticks()`（语义见本文件头）。
macro_rules! pacer {
    ($name:ident, $dev:expr, $irq:expr, $doc_dev:expr) => {
        #[doc = concat!("节拍源：`", $doc_dev, "`（硬件定时器 + ISR + 信号量）。")]
        pub mod $name {
            use super::*;

            /// 板级设备名（见 `joc-base/src/board/stm32f4_discovery.c`）。
            const DEV: &str = $dev;
            /// 该 TIM 在 STM32F407 上的 IRQ 号（见 `joc-base/src/hal/*/tim_hal.c`）。
            const IRQN: u8 = $irq;

            /// 计数信号量：ISR 里 `give`，任务里 `wait`（上限 1 ⇒ 密集事件自动合并 ✓）。
            static mut SEM: rtos_sem_t =
                rtos_sem_t { count: 0, limit: 0, waitq: null_mut() };
            /// 节拍源是否可用（★在 `init()` 里显式赋值，不依赖 static 初值）。
            static mut READY: bool = false;
            /// 已收到的节拍数（诊断：非零即证明 ISR 真的在跑 ✓ —— 仪器的自检 ✓）。
            static mut TICKS: u32 = 0;

            /// 定时器溢出 ISR（ISR 上下文，只做 `sem_give` —— ISR 安全 ✓）。
            ///
            /// UIF **不由我们清**：timer 驱动在 open 时已注册到同一条 IRQ 线，驱动
            /// ISR 负责清 UIF ✓；`irq_dispatch` 会调用该线全部 handler ✓。
            extern "C" fn isr(_ctx: *mut c_void) {
                unsafe {
                    TICKS = TICKS.wrapping_add(1);
                    if let Some(f) = g_app_slot.sem_give {
                        f(addr_of_mut!(SEM));
                    }
                }
            }

            /// 拉起节拍源。`true` = 精确节拍可用；`false` = 调用方**必须回退**
            /// `delay_until`（反静默降级 ✓：绝不"看起来在工作"却其实没在节拍）。
            pub fn init() -> bool {
                unsafe {
                    READY = false;
                    TICKS = 0;
                    if let Some(f) = g_app_slot.sem_init {
                        f(addr_of_mut!(SEM), 0, 1);
                    }
                }

                let mut dev = match Device::open(DEV) {
                    Some(d) => d,
                    None => return false,
                };
                if dev.open_dev() != 0 {
                    return false;
                }

                // 顺序固定：先挂 handler 并使能该线（避免"边沿先到、handler 未装"
                // 的空窗 ✗），再启动计数器。
                if irq::attach_and_enable(IRQN, isr, null_mut()) != 0 {
                    return false;
                }
                // TIMER_IOCTL_ENABLE：启动计数器并 arm NVIC（驱动自身 ISR 亦使能 ✓）。
                if dev.ioctl(ioctl::TIMER_IOCTL_ENABLE, null_mut()) != 0 {
                    return false;
                }

                unsafe { READY = true; }
                true
            }

            /// 阻塞等待下一个硬件节拍。
            #[inline]
            pub fn wait_tick() {
                unsafe {
                    if let Some(f) = g_app_slot.sem_wait {
                        f(addr_of_mut!(SEM));
                    }
                }
            }

            /// 节拍源是否已就绪。
            #[inline]
            pub fn is_ready() -> bool {
                unsafe { READY }
            }

            /// 已收到的节拍数（诊断/自检 ✓）。
            #[inline]
            pub fn ticks() -> u32 {
                unsafe { TICKS }
            }
        }
    };
}

// 控制任务：板级 `timer3` = TIM7（basic，84MHz APB1）⇒ 板级配置 **250Hz = 4.000ms** ✓
pacer!(control, "timer3", 55, "timer3/TIM7 @250Hz (4.000ms)");
// 传感器任务：板级 `timer5` = TIM9（general，168MHz APB2）⇒ 板级配置 **500Hz = 2.000ms** ✓
pacer!(sensors, "timer5", 24, "timer5/TIM9 @500Hz (2.000ms)");
