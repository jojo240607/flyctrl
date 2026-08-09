//! USART2 真实驱动 + 帧化链路 `UartLink`（实现 `comm::Link`）。
//!
//! 引脚：PA2(TX, AF7) / PA3(RX, AF7)。波特率默认 921600，8N1。
//! RX 用中断填充一个小环形缓冲；`recv_frame` 从中按 MAVLink v2 (0xFD) 边界解帧（与 host 端
//! `LoopbackLink` 同款逻辑，保证 host/SIL 与 MCU 共用同一帧语义）。
//! TX 采用阻塞发送（飞控下行遥测量小，且避免在 ISR 里做复杂状态机）。

use stm32f4::stm32f407::{RCC, USART2, Interrupt};
use cortex_m::interrupt::{self, Mutex};
use core::cell::RefCell;

use crate::comm::link::{Frame, Link, MAX_FRAME_LEN};
use crate::hal::stm32f407::clock::APB1_HZ;
use crate::hal::stm32f407::gpio::{self, Pin, Port, AltFn};

/// RX 环形缓冲容量（字节）。
const RX_BUF: usize = 512;

// 全局 RX 环形缓冲（ISR 写、主循环读）。用 Mutex 保护避免临界区竞争。
struct RxRing {
    buf: [u8; RX_BUF],
    head: usize, // 写指针（ISR）
    tail: usize, // 读指针（主循环）
    count: usize,
}
static RX_RING: Mutex<RefCell<RxRing>> = Mutex::new(RefCell::new(RxRing {
    buf: [0u8; RX_BUF],
    head: 0,
    tail: 0,
    count: 0,
}));

/// USART2 帧化链路。
pub struct UartLink {
    usart: &'static USART2,
    healthy: bool,
}

impl UartLink {
    /// 构造并初始化 USART2 为 921600 8N1。
    ///
    /// 调用方负责提供 `Peripherals` 中的 `USART2` 与 `RCC` 引用（仅可取一次）。
    pub fn new(usart: &'static USART2, rcc: &'static RCC, baud: u32) -> Self {
        // 1) 时钟：USART2 在 APB1
        rcc.apb1enr().modify(|_, w| w.usart2en().set_bit());
        // 2) 引脚 PA2/PA3 → AF7
        gpio::configure_af(rcc, Pin { port: Port::A, pin: 2 }, AltFn(7));
        gpio::configure_af(rcc, Pin { port: Port::A, pin: 3 }, AltFn(7));

        // 3) 波特率：BRR = fPCLK / baud（USART2 在 APB1）
        let brr = APB1_HZ / baud;
        usart.brr().write(|w| unsafe { w.bits(brr as u16) });

        // 4) 使能 TE/RE/UE + RXNEIE（接收中断），8N1
        usart.cr1().write(|w| {
            w.ue()
                .set_bit()
                .te()
                .set_bit()
                .re()
                .set_bit()
                .rxneie()
                .set_bit()
                .m()
                .clear_bit()
                .pce()
                .clear_bit()
        });
        usart.cr2().write(|w| unsafe { w.bits(0) }); // 1 停止位
        usart.cr3().write(|w| unsafe { w.bits(0) });

        // 5) 注册中断（cortex-m 中断向量）
        unsafe {
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART2);
        }

        UartLink { usart, healthy: true }
    }

    /// 由 `#[interrupt] fn USART2()` 调用：把接收字节压入环形缓冲。
    pub fn on_rx_byte(b: u8) {
        interrupt::free(|cs| {
            let mut ring = RX_RING.borrow(cs).borrow_mut();
            if ring.count < RX_BUF {
                let head = ring.head;
                ring.buf[head] = b;
                ring.head = (head + 1) % RX_BUF;
                ring.count += 1;
            }
            // 满则丢弃（背压：真实链路应 NAK，此处简化）
        });
    }

    /// 从环形缓冲取一个字节（无则 None）。
    fn pop_byte() -> Option<u8> {
        interrupt::free(|cs| {
            let mut ring = RX_RING.borrow(cs).borrow_mut();
            if ring.count == 0 {
                None
            } else {
                let tail = ring.tail;
                let b = ring.buf[tail];
                ring.tail = (tail + 1) % RX_BUF;
                ring.count -= 1;
                Some(b)
            }
        })
    }
}

impl Link for UartLink {
    fn send_frame(&mut self, frame: &Frame) -> usize {
        let slc = frame.as_slice();
        for &b in slc {
            while self.usart.sr().read().txe().bit_is_clear() {}
            self.usart.dr().write(|w| unsafe { w.bits(b as u16) });
        }
        slc.len()
    }

    fn recv_frame(&mut self) -> Frame {
        // 与 LoopbackLink 同款：找 0xFD 起始，读齐 MAVLink v2 帧（10 + len + 2）。
        let mut data = [0u8; MAX_FRAME_LEN];
        let mut len = 0usize;

        // 找到起始符 0xFD（MAVLink v2 magic）
        while let Some(b) = Self::pop_byte() {
            if b == 0xFD {
                data[0] = b;
                len = 1;
                break;
            }
        }
        if len == 0 {
            return Frame::default();
        }
        // v2 头部 9 字节（len, incompat, compat, seq, sys, comp, msgid[3]）
        for _ in 0..9 {
            if let Some(b) = Self::pop_byte() {
                data[len] = b;
                len += 1;
            } else {
                return Frame::default();
            }
        }
        if len < 10 {
            return Frame::default();
        }
        let payload = data[1] as usize; // MAVLink len 域
        let total = 10 + payload + 2;
        while len < total {
            if let Some(b) = Self::pop_byte() {
                if len < MAX_FRAME_LEN {
                    data[len] = b;
                    len += 1;
                } else {
                    return Frame::default();
                }
            } else {
                return Frame::default();
            }
        }
        self.healthy = true;
        Frame::from_bytes(&data[..len])
    }

    fn healthy(&self) -> bool {
        self.healthy
    }
}

/// USART2 中断服务例程（在固件 crate 中通过 `#[interrupt] fn USART2()` 调用）。
///
/// 接收寄存器块引用（`USART2::ptr()` 返回 `&RegisterBlock`）。
pub fn usart2_isr(usart: &stm32f4::stm32f407::usart2::RegisterBlock) {
    if usart.sr().read().rxne().bit_is_set() {
        let b = usart.dr().read().bits() as u8;
        UartLink::on_rx_byte(b);
    }
}
