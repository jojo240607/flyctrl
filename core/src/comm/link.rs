//! 物理链路抽象：把"字节如何收发"与上层协议解耦。
//!
//! [`Link`] trait 只暴露 `send_frame` / `recv_frame`，语义为"一帧 MAVLink 报文"。
//! 具体实现可以是串口（UART+DMA）、USB-CDC、或是 host 端的回环/文件。
//! 所有实现无堆、有界耗时。

// 帧结构与长度上限统一取自 mavlink-core（单一事实来源），避免多处重复定义漂移。
// 这样 `link::Frame` 与 `mavlink::decode` 期望的帧类型是同一个类型。
pub use mavlink_core::frame::{Frame, MAX_FRAME_LEN};

/// 链路接口：发送/接收完整帧。
pub trait Link {
    /// 发送一帧（阻塞/有界超时由实现决定）。返回实际发出字节数。
    fn send_frame(&mut self, frame: &Frame) -> usize;

    /// 尝试接收一帧；无数据返回空帧（`is_empty()` 为 true）。
    fn recv_frame(&mut self) -> Frame;

    /// 链路健康（通信错误计数阈值内）。
    fn healthy(&self) -> bool;
}

// ─────────────────────────────────────────────────────────────
// Host 实现：内存回环（测试/SIL 用），无外部设备依赖。
// ─────────────────────────────────────────────────────────────

/// Host 回环链路：send 进一个 ring，recv 从 ring 取。
/// 固定容量，溢出丢弃最旧（无损优先但耗尽时丢，符合遥测语义）。
pub struct LoopbackLink {
    buf: [u8; 1024],
    head: usize,
    tail: usize,
    count: usize,
    ok: bool,
}

impl LoopbackLink {
    pub fn new() -> Self {
        Self { buf: [0u8; 1024], head: 0, tail: 0, count: 0, ok: true }
    }

    fn push_byte(&mut self, b: u8) {
        self.buf[self.head] = b;
        self.head = (self.head + 1) % self.buf.len();
        if self.count < self.buf.len() {
            self.count += 1;
        } else {
            // 满：丢弃最旧
            self.tail = (self.tail + 1) % self.buf.len();
        }
    }

    fn pop_byte(&mut self) -> Option<u8> {
        if self.count == 0 { return None; }
        let b = self.buf[self.tail];
        self.tail = (self.tail + 1) % self.buf.len();
        self.count -= 1;
        Some(b)
    }
}

impl Default for LoopbackLink { fn default() -> Self { Self::new() } }

impl Link for LoopbackLink {
    fn send_frame(&mut self, frame: &Frame) -> usize {
        let n = frame.len.min(MAX_FRAME_LEN);
        for i in 0..n { self.push_byte(frame.data[i]); }
        n
    }

    fn recv_frame(&mut self) -> Frame {
        // 读出直到遇到 MAVLink v2 帧边界：以 0xFD 开头，长度域在 [1]。
        // v2 头部 9 字节（len,incompat,compat,seq,sys,comp,msgid[3]），总长 = 10 + len + 2(crc)。
        // 简单策略：读到 0xFD 起始，至少收集齐头部+N+CRC 才返回。
        let mut data = [0u8; MAX_FRAME_LEN];
        let mut len = 0usize;
        // 找到起始符
        while self.count > 0 {
            if let Some(b) = self.pop_byte() {
                if b == 0xFD {
                    data[0] = b;
                    len = 1;
                    break;
                }
            }
        }
        if len == 0 { return Frame::default(); }
        // 需要再读 9 字节头部（len, incompat, compat, seq, sys, comp, msgid[3]）
        for _ in 0..9 {
            if let Some(b) = self.pop_byte() { data[len] = b; len += 1; }
            else { return Frame::default(); }
        }
        let payload_len = data[1] as usize;
        let total = 10 + payload_len + 2; // magic + 9 字节头部 + payload + 2 crc
        for _ in (len as usize)..total {
            if let Some(b) = self.pop_byte() { data[len] = b; len += 1; }
            else { return Frame::default(); }
        }
        Frame { data, len }
    }

    fn healthy(&self) -> bool { self.ok }
}

// ─────────────────────────────────────────────────────────────
// STM32F407 占位实现：UART/USB-CDC 字节流。真实落地替换为
// joc-base 的 uart.c / usb.c HAL（DMA 双缓冲）；此处保留骨架。
// ─────────────────────────────────────────────────────────────

#[cfg(feature = "stm32f407")]
pub mod stm32f407 {
    use super::*;

    /// STM32F4 串口 + DMA 的链路实现（如 USART2 <-> 数传 / USB CDC_ACM）。
    pub struct UartLink { base: usize, ok: bool }
    impl UartLink {
        pub const fn new(base: usize) -> Self { Self { base, ok: true } }
    }
    impl Link for UartLink {
        fn send_frame(&mut self, frame: &Frame) -> usize {
            let _ = self.base;
            // 占位：经 DMA TX 环形缓冲异步发出。
            frame.len
        }
        fn recv_frame(&mut self) -> Frame {
            let _ = self.base;
            // 占位：从 DMA RX 环形缓冲按 0xFE 边界组帧。
            Frame::default()
        }
        fn healthy(&self) -> bool { self.ok }
    }
}
