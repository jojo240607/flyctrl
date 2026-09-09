//! flyctrl-app：飞控应用分区（jOS RTOS 双分区轨 B）。
//! 直接依赖 `rtos-app-sdk`（joc-rtos-app-sdk 中间工程）+ `flyctrl-core` 编译成
//! `app.bin`（旧 joc-app-rust 独立工程已废弃并迁入本 crate）。
//!
//! 挂载方式：SDK 的 `rust_app_start`（.data 自拷贝 + ABI 校验 + 挂载自报）调用本
//! crate 实现的 `app_main()`；本层只经 g_app_slot 服务表间接调用内核，创建飞控
//! 硬实时任务（EKF + PID + FDIR + MAVLink 遥测），不碰任何裸 RTOS 符号/寄存器。

#![no_std]
#![allow(static_mut_refs)]

pub mod flyctrl;
pub mod sensors;

#[cfg(feature = "usbtest")]
pub mod usbtest;
#[cfg(any(feature = "integration-test", feature = "panic-test"))]
pub mod intg_test;

/// 应用入口（SDK 的 rust_app_start 调用）：按 feature 分发拉起对应入口后返回 0。
#[no_mangle]
pub extern "C" fn app_main() -> i32 {
    //  - demo feature：极简打日志任务，不碰任何外设，仅验证拉起链路；
    //  - usbtest feature：隔离验证 usb0 写通道是否通畅（不碰 EST_MTX/传感器）；
    //  - 默认：正式飞控多任务（采样/控制/遥测/监控，经 RTOS 设备 vtable）。
    #[cfg(feature = "demo")]
    demo::spawn_demo();
    #[cfg(feature = "usbtest")]
    crate::usbtest::start();
    #[cfg(any(feature = "integration-test", feature = "panic-test"))]
    crate::intg_test::run_intg_test();
    #[cfg(not(any(
        feature = "demo",
        feature = "usbtest",
        feature = "integration-test",
        feature = "panic-test"
    )))]
    crate::flyctrl::spawn_flyctrl();

    0
}

/* ===========================================================================
 * Demo 入口（feature = "demo"）：极简任务，只经 SDK 打周期日志，不碰任何外设。
 * 目的：先验证「RTOS 异步 app_host 任务 → rust_app_start → app_main → 创建 RTOS
 * 任务」的拉起链路是否跑通，并能从 App 经 g_app_slot 打日志到控制台。
 * 跑通后再切回正式 flyctrl（去掉 --features demo）。
 * =========================================================================== */
#[cfg(feature = "demo")]
mod demo {
    use core::ffi::c_void;

    use rtos_app_sdk::rtos::{msleep, spawn, tick_count, RTOS_PRIO_BH_MED};
    use rtos_app_sdk::info;

    // demo 任务独立栈（放 App RAM，4KB）。
    #[link_section = ".rust_bss"]
    static mut DEMO_STACK: [u8; 4096] = [0u8; 4096];

    extern "C" fn demo_task_entry(_arg: *mut c_void) {
        let mut n: u32 = 0;
        info!("demo", "task started (prio={})", RTOS_PRIO_BH_MED);
        loop {
            n = n.wrapping_add(1);
            if n % 20 == 0 {
                info!("demo", "alive seq={} ticks={}", n, tick_count());
            }
            msleep(50);
        }
    }

    pub fn spawn_demo() {
        unsafe {
            spawn(
                "demo_app",
                demo_task_entry,
                RTOS_PRIO_BH_MED,
                DEMO_STACK.as_mut_ptr(),
                DEMO_STACK.len(),
            );
        }
        info!("demo", "demo task spawned (link verified)");
    }
}
