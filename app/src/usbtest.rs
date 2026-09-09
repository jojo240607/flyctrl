/* ===========================================================================
 * USB CDC 下行验证 demo（feature = "usbtest"）
 *
 * 目的：隔离验证 RTOS 的 usb0（USB CDC-ACM，Win 侧 COM9）写通道是否通畅。
 * 不碰 EST_MTX / 传感器 / 控制律，只做一件事：
 *   每 200ms 经 usb0 写一行 `USBTEST seq=N ticks=T\r\n`，
 *   并把同一行 + usb0 CONNECTED 状态 + write 返回值经 uart0（COM8）镜像打印。
 *
 * 判定：
 *   - 任务持续运行（COM8 每轮都打印）→ 证明 USB write 不会阻塞 App 任务；
 *   - COM9 收到 `USBTEST ...` 字节 → 证明 USB CDC 下行数据通道真正打通；
 *   - 若 COM8 持续打印但 COM9 收不到 → USB 下行链路有问题（驱动/枚举/endpoint），
 *     再据此排查 usb.c 的 usb_stream_write / usb_tx_pump / DCD_EP_Tx。
 *
 * 注：usb0 在系统层 boot 时已注册并 open（作为第二控制台）。App 侧再次 open
 * 由 usb_dev_open 幂等处理；这里用 Device::open 贴近 telem 的真实用法以复现。
 * =========================================================================== */
use rtos_app_sdk::abi::*;
use rtos_app_sdk::device::Device;
use rtos_app_sdk::info;
use rtos_app_sdk::ioctl;
use rtos_app_sdk::rtos::{spawn_rt, tick_count, msleep, RTOS_RT_NONE};
use core::ffi::c_void;

    // 任务独立栈（放 App RAM，1.5KB 足够打印缓冲 + 调用深度）。
    #[link_section = ".rust_bss"]
    static mut USBTEST_STACK: [u8; 1536] = [0u8; 1536];

    extern "C" fn usbtest_task_entry(_arg: *mut c_void) {
        // 系统层 boot 时已 open 过 usb0（作为第二控制台）。这里【只 get 不 open】，
        // 避免 App 二次 USBD_Init 触发 USBRST 把 connected 清 0 且重枚举失败。
        let usb = Device::get("usb0\0");
        info!(tag: "usbtest", "usb0 get = {}",
              if usb.is_some() { "ok" } else { "NULL" });

        let mut seq: u32 = 0;
        loop {
            seq = seq.wrapping_add(1);
            let ticks = tick_count();

            // 构造一行下行文本。
            let mut line = [0u8; 64];
            let header = b"USBTEST seq=";
            let mut len = 0usize;
            for &b in header {
                line[len] = b; len += 1;
            }
            len += write_u32(&mut line[len..], seq);
            for &b in b" ticks=" {
                line[len] = b; len += 1;
            }
            len += write_u32(&mut line[len..], ticks);
            line[len] = b'\r'; len += 1;
            line[len] = b'\n'; len += 1;

            // 经 usb0 写，验证下行数据通道。
            // 关键：只 write，绝不调用任何 ioctl（USB_IOCTL_CONNECTED / USB_IOCTL_DBG_DUMP）。
            // 之前实测：App 每 200ms 反复 ioctl 查询会干扰系统层 USB 驱动状态机，
            // 导致 host 打开 COM 口（SetCommState → SET_LINE_CODING）时 EP0 控制传输
            // 失败 → Windows 报 error 31。write 本身是干净的（只进 TX staging ring，
            // 由 usb_tx_pump 后台发送），不碰 EP0/控制传输，不会干扰 host 枚举/打开。
            // write 返回值为负/0 表示未连接或 TX 满，属于正常背压，不阻塞本任务。
            let mut wr: i32 = -1;
            if let Some(dev) = &usb {
                wr = dev.write(&line[..len]);
            }

            // uart0 镜像：每轮状态，便于无 COM9 时也能看任务在跑。
            info!(tag: "usbtest", "seq={} usb_wr={} ticks={}",
                  seq, wr, ticks);

            msleep(200);
        }
    }

    /// 把 u32 十进制写入 buf，返回写入字节数（无分配）。
    fn write_u32(buf: &mut [u8], mut v: u32) -> usize {
        if v == 0 {
            if !buf.is_empty() { buf[0] = b'0'; }
            return 1;
        }
        let mut tmp = [0u8; 10];
        let mut i = 0;
        while v > 0 {
            tmp[i] = b'0' + (v % 10) as u8;
            v /= 10;
            i += 1;
        }
        let mut n = 0;
        while i > 0 {
            i -= 1;
            if n < buf.len() {
                buf[n] = tmp[i];
                n += 1;
            }
        }
        n
    }

    pub fn start() {
        unsafe {
            spawn_rt(
                "usbtest",
                usbtest_task_entry,
                RTOS_PRIO_BH_MED, // 中优先，不抢硬实时
                USBTEST_STACK.as_mut_ptr(),
                USBTEST_STACK.len(),
                1, // priv=1：App 任务保持特权，与正式 flyctrl 一致
                RTOS_RT_NONE,
                0,
                0,
            );
        }
        info!(tag: "usbtest", "usbtest task spawned (USB CDC downlink verify)");
    }
