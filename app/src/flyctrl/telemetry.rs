//! 遥测下行任务（20ms 周期）：从最新估计发标准 MAVLink。
//!
//! 读 EST_STATE（经 EST_MTX），经 usb0(USB CDC) 单通道下行
//! heartbeat / local_pos / sys_status。
//!
//! 注：uart3(USART6) 未接到 PC，无法闭环验证，故下行只用 usb0。

use core::ffi::c_void;

use flyctrl_core::comm::mavlink;
use flyctrl_core::fdir::Health;

use rtos_app_sdk::abi::RTOS_PRIO_MAIN;
use rtos_app_sdk::device::Device;
use rtos_app_sdk::{info, warn};
use rtos_app_sdk::rtos::msleep;
use core::sync::atomic::Ordering;
use crate::flyctrl::{EST_MTX, EST_STATE, USB_TX_MTX};
use crate::flyctrl::uplink::G_THROTTLE;

/// 帧缓冲放在静态区（不占任务栈）。
/// 遥测任务独占该缓冲，循环内串行复用，无需互斥。
#[link_section = ".rust_bss"]
static mut FRAME_BUF: [u8; flyctrl_core::comm::link::MAX_FRAME_LEN] =
    [0u8; flyctrl_core::comm::link::MAX_FRAME_LEN];

/// 遥测下行任务入口。
///
/// 下行通道：USB CDC(`usb0`) 单写。
/// USB CDC 是 CDC-ACM 虚拟串口，电脑端免 USB-TTL 转接即可直接收 MAVLink 流做仿真分析。
/// `usb_stream_write` 是非阻塞 staged 写：host 未连 / 未 IN-token 时 TX ring 填满后
/// 仅返回 0（丢帧），绝不阻塞任务。因此**不**用 `USB_IOCTL_CONNECTED`(DTR 控制线) 来决定
/// 是否写 usb0 —— 上位机开 COM9 但 DTR=False 时 conn 仍为 0，据此跳过会致上位机收不到
/// 数据。正确做法是无条件写，host 一连即收。usb0 与 C 侧 g_console(uart0) 相互独立。
///
/// uart3(USART6) 未接到 PC，无法闭环验证，下行暂不挂 uart3（其 IRQ 引擎 write 为阻塞式，
/// 一旦唤醒中断异常会永久卡死 telem；而 usb0 的非阻塞 staged 写天然满足"host 不连不卡死"）。
pub extern "C" fn telemetry_entry(_arg: *mut c_void) {
    info!(tag: "telem", "task started; period=20ms prio={} downlink=usb0", RTOS_PRIO_MAIN);

    // usb0 复用系统层已在 boot 阶段 open 的句柄（Device::get），
    // 切勿二次 Device::open —— 二次 open 会再次 USBD_Init + 重绑 ISR，
    // 重置 USB TX 状态机导致后续 write 阻塞/卡死（已实测复现）。
    let usb_dev = Device::get("usb0\0");
    if usb_dev.is_none() {
        warn!(tag: "telem", "usb0 (USB CDC) not available -> no downlink");
    }

    let frame_buf = unsafe { &mut FRAME_BUF };
    let mut seq: u8 = 0;
    let mut boot_ms: u32 = 0; // 下行 time_boot_ms 累加（20ms/周期）
    loop {
        // 读最新估计（短临界区）
        let (mut est, health, armed);
        {
            let _g = unsafe { EST_MTX.guard() };
            let s = unsafe { &*core::ptr::addr_of!(EST_STATE) };
            est = s.est;
            health = s.health;
            armed = s.armed;
        }
        est.time_boot_ms = boot_ms as i32;

        // 下行：USB CDC(usb0) 单通道写。
        //
        // 注意：USB CDC 的 usb_stream_write 是【非阻塞 staged】——host 未连 / 未 IN-token
        // 时 TX ring 填满后 write 仅返回 0（丢帧），绝不阻塞任务（已实测验证）。
        // 因此**不要**用 USB_IOCTL_CONNECTED(DTR 控制线) 决定是否写 usb0：上位机打开
        // COM9 但 DTR=False（避免 CH340 复位）时 conn 仍为 0，若据此跳过 usb0 会导致
        // 上位机连着 USB 却收不到 MAVLink。正确做法是无条件写，host 连上即收到。
        let mut wrote_usb = 0i32;
        // 心跳 custom_mode 反映上行指令设置的模式（地面站经 DO_SET_MODE 下发，ArduCopter 标准码）。
        let cmd_mode = crate::flyctrl::uplink::G_CMD_MODE.load(Ordering::Relaxed) as u8;
        let send = |d: &Device, fb: &mut [u8; flyctrl_core::comm::link::MAX_FRAME_LEN],
                    seq: u8, est: &_, armed: bool, health_ok: bool, mode: u8| -> i32 {
            let mut total = 0i32;
            // 逐个 encode 后【立即 write】。不能用 `for n in [enc(), enc(), enc()]`：
            // Rust 会【急切求值】数组三个元素，而三个 encode_* 都写入同一个共享 fb
            // 缓冲，数组构建完 fb 只剩最后一个 encode_sys_status 的 SYS_STATUS，
            // 循环里 write(&fb[..n]) 三次写的都是同一个 SS 帧的切片（21/40/43B 全
            // 是 SYS_STATUS 头 fd1f0000...）→ host 端只有 SS、无 HB/LP、每 seq 前缀
            // 重复 3 次、CRC 全错。改为每帧 encode 后立即 write，fb 在 write 前是
            // 正确的当前帧。
            let n1 = {
                #[cfg(feature = "hil")]
                { mavlink::encode_heartbeat_hil(mode, armed, seq, fb) }
                #[cfg(not(feature = "hil"))]
                { mavlink::encode_heartbeat(mode, armed, seq, fb) }
            };
            let k1 = d.write(&fb[..n1]);
            if k1 > 0 { total += k1; }

            let n2 = mavlink::encode_local_pos_from(mavlink::SYS_ID, est, seq, fb);
            let k2 = d.write(&fb[..n2]);
            if k2 > 0 { total += k2; }

            let n3 = mavlink::encode_sys_status(health_ok, seq, fb);
            let k3 = d.write(&fb[..n3]);
            if k3 > 0 { total += k3; }

            // 地面站(groundctrl/QGC)主盘所需：姿态球(ATTITUDE) / HUD(VFR_HUD) / 位置(GLOBAL_POSITION_INT)。
            let na = mavlink::encode_attitude(est, seq, fb);
            let ka = d.write(&fb[..na]);
            if ka > 0 { total += ka; }

            let nv = mavlink::encode_vfr_hud(est, G_THROTTLE.load(Ordering::Relaxed) as u16, seq, fb);
            let kv = d.write(&fb[..nv]);
            if kv > 0 { total += kv; }

            let ng = mavlink::encode_global_position_int(est, seq, fb);
            let kg = d.write(&fb[..ng]);
            if kg > 0 { total += kg; }
            total
        };
        if let Some(d) = usb_dev.as_ref() {
            // usb0 与 uplink 共享：RTOS 侧 TX ring 写无锁（假设单生产者），
            // 必须用 USB_TX_MTX 串行化，否则两任务写帧会在 ring 中交错损坏。
            let _g = unsafe { USB_TX_MTX.guard() };
            wrote_usb = send(d, frame_buf, seq, &est, armed, health != Health::Critical, cmd_mode);
        }

        // HIL：回传执行器指令（HIL_ACTUATOR_CONTROLS(93)），PC 端仿真器据此驱动 plant。
        // control 每周期把 motor[0..4] 写入 G_ACTUATOR_CMD，本处随心跳节奏一起下发。
        #[cfg(feature = "hil")]
        if let Some(d) = usb_dev.as_ref() {
            let _g = unsafe { USB_TX_MTX.guard() };
            use flyctrl_core::comm::mavlink::enums;
            let motors = crate::flyctrl::uplink::actuator_cmd();
            let mut controls = [0f32; 16];
            controls[0..4].copy_from_slice(&motors);
            let mode_bm = enums::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                | enums::MAV_MODE_FLAG_HIL_ENABLED
                | if armed { enums::MAV_MODE_FLAG_SAFETY_ARMED } else { 0 };
            let nh = mavlink::encode_hil_actuator_controls(
                (boot_ms as u64) * 1000, &controls, mode_bm, 0, seq, frame_buf);
            let _ = d.write(&frame_buf[..nh]);
        }

        seq = seq.wrapping_add(1);
        if seq == 1 {
            info!(tag: "telem", "first loop done; armed={}", armed);
        }
        // 注意：周期性 hb 健康日志已移除 —— 它每 1s 走阻塞式 uart0 控制台，曾在
        // UART DMA TX 上死锁（telem 永久卡在 tx_idle 信号量），导致 usb0 停止下行。
        // telem 的下行目标本就是 usb0，状态/健康数据已随 MAVLink 帧下行，无需再经
        // 共享控制台打周期日志。需要诊断时用 GDB 直接读任务状态/EST_MTX。

        boot_ms = boot_ms.wrapping_add(20);
        msleep(20);
    }
}
