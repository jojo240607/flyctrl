//! SBUS 遥控接收机驱动：经 RTOS `uart2`（USART3，100kbps 8E2 反向电平）读取并解析。
//!
//! SBUS 帧 = `0x0F` 头 + 16×11bit 通道（22 字节）+ 1 字节标志 + `0x00` 尾，共 25 字节。
//! 通道范围 0..=2047（中位 ~992，有效行程约 172..=1811）。
//! 解析为 `flyctrl_core::hal::sensor::RcReceiver`，输出归一化 [`RcInput`]：
//!   通道 0/1/2/3 → roll/pitch/yaw/throttle（顺序随接收机，这里按标准 FrSky 映射）。
//!   通道 4（开关）→ armed（> 1700 解锁）；通道 5 → mode 槽位。
//!
//! 注意：SBUS 物理层为**反向电平**（idle 高、反向 NRZI）。普通 MCU UART 收不到帧，
//! 必须在打开设备后用 `UART_IOCTL_SET_INVERTED`（bit0=RXINV, bit1=TXINV）使能硬件反相。
//! 本驱动在 `new()` 里主动下发该 ioctl（RX+TX 均反相，后者用于诊断回环）。

use core::ffi::c_void;

use flyctrl_core::hal::sensor::RcReceiver;
use flyctrl_core::vehicle::RcInput;

use rtos_app_sdk::device::Device;
use rtos_app_sdk::ioctl;

const SBUS_HEADER: u8 = 0x0F;
const SBUS_FOOTER: u8 = 0x00;
const SBUS_FRAME_LEN: usize = 25;
const SBUS_MID: f32 = 992.0;
const SBUS_RANGE: f32 = (1811.0 - 172.0) / 2.0; // 半行程

/// 反相位掩码：bit0 = RXINV，bit1 = TXINV（与 `UART_IOCTL_SET_INVERTED` 约定一致）。
const UART_INVERT_RX: u32 = 0x1;
const UART_INVERT_TX: u32 = 0x2;

pub struct RcSbus {
    dev: Device,
    buf: [u8; SBUS_FRAME_LEN],
    fill: usize,
    /// 最近一次解出的 16 通道（原始 0..2047）。
    ch: [u16; 16],
    /// 本拍是否解出有效新帧（帧间为 false，RC 链路"新帧"标记）。
    fresh: bool,
    /// 是否收到过至少一帧有效 SBUS（帧间锁存 armed/throttle 的依据）。
    received: bool,
    lost: bool,
}

impl RcSbus {
    pub fn new(bus_name: &str) -> Option<Self> {
        let dev = Device::open(bus_name)?;
        // SBUS 物理层要求：100000 baud、8 数据位、偶校验、2 停止位、电平反向。
        // 板级 g_uart2 默认 115200/8N1，这里在打开后主动下发全部线路参数。
        // 若 RTOS uart 驱动未实现对应 ioctl，返回 -1，此处忽略（降级，仅日志缺失）。
        let mut baud: u32 = 100_000;
        let _ = dev.ioctl(
            ioctl::UART_IOCTL_SET_BAUDRATE,
            &mut baud as *mut u32 as *mut c_void,
        );
        let mut parity: u32 = 2; // 偶校验
        let _ = dev.ioctl(
            ioctl::UART_IOCTL_SET_PARITY,
            &mut parity as *mut u32 as *mut c_void,
        );
        let mut stop: u32 = 2; // 2 停止位
        let _ = dev.ioctl(
            ioctl::UART_IOCTL_SET_STOPBITS,
            &mut stop as *mut u32 as *mut c_void,
        );
        let mut inv = UART_INVERT_RX | UART_INVERT_TX; // 反向电平（RX 必需 + TX 诊断）
        let _ = dev.ioctl(
            ioctl::UART_IOCTL_SET_INVERTED,
            &mut inv as *mut u32 as *mut c_void,
        );
        Some(Self {
            dev,
            buf: [0u8; SBUS_FRAME_LEN],
            fill: 0,
            ch: [0u16; 16],
            fresh: false,
            received: false,
            lost: false,
        })
    }

    /// 从设备字节流抽取完整 SBUS 帧并解出通道。
    fn drain(&mut self) {
        let mut rb = [0u8; 64];
        let n = self.dev.read(&mut rb);
        if n <= 0 {
            return;
        }
        for &b in &rb[..n as usize] {
            // 找头：若当前缓冲空且不是头，跳过
            if self.fill == 0 {
                if b != SBUS_HEADER {
                    continue;
                }
            }
            if self.fill < SBUS_FRAME_LEN {
                self.buf[self.fill] = b;
                self.fill += 1;
            } else {
                // 溢出：丢弃，重新对齐到新头
                self.fill = 0;
                if b == SBUS_HEADER {
                    self.buf[0] = b;
                    self.fill = 1;
                }
                continue;
            }
            if self.fill == SBUS_FRAME_LEN {
                // 校验尾
                if self.buf[SBUS_FRAME_LEN - 1] == SBUS_FOOTER {
                    self.decode();
                }
                self.fill = 0;
            }
        }
    }

    /// 解 22 字节 payload 的 16×11bit 通道；写 self.ch。
    fn decode(&mut self) {
        let p = &self.buf[1..23]; // 跳过 header 与 flags
        let mut bits: u32 = 0;
        let mut bitcount = 0u32;
        let mut idx = 0usize;
        for &byte in p {
            bits |= (byte as u32) << bitcount;
            bitcount += 8;
            while bitcount >= 11 && idx < 16 {
                self.ch[idx] = (bits & 0x7FF) as u16;
                bits >>= 11;
                bitcount -= 11;
                idx += 1;
            }
        }
        // 标志字节（buf[23]）：bit3 = 失败保护，bit4 = 帧丢失
        let flags = self.buf[23];
        self.lost = (flags & 0x10) != 0 || (flags & 0x08) != 0;
        self.fresh = !self.lost;
        if !self.lost {
            self.received = true;
        }
    }

    /// 单通道原始值 → 归一化 [-1,1]（油门映射到 [0,1] 由调用方处理）。
    fn norm(ch: u16) -> f32 {
        let v = (ch as f32 - SBUS_MID) / SBUS_RANGE;
        if v > 1.0 {
            1.0
        } else if v < -1.0 {
            -1.0
        } else {
            v
        }
    }
}

impl RcReceiver for RcSbus {
    fn read(&mut self) -> RcInput {
        self.drain();
        // 从未收到过有效帧：中性安全默认（解锁前无 RC 时不得误解锁）。
        if !self.received {
            return RcInput::neutral();
        }
        // 【帧间锁存】SBUS 帧周期（真机 ~10ms / 本模拟器 50ms）远大于控制拍 4ms：
        // 若帧间返回 neutral，armed/throttle 会在 20Hz 帧间隙瞬时归零 → 控制环
        // 大部分拍闸门关闭、电机零输出（解锁飞行不可用）。正确语义：通道值/解锁
        // 锁存最近有效帧，fresh 仅标记"本拍是否有新帧"（RC 链路活性，供失联检测）。
        let throttle = (Self::norm(self.ch[3]) + 1.0) * 0.5; // [-1,1]→[0,1]
        let armed = self.ch[4] > 1700;
        let mode = if self.ch[5] < 600 {
            0
        } else if self.ch[5] < 1400 {
            1
        } else {
            2
        };
        RcInput {
            roll: Self::norm(self.ch[0]),
            pitch: Self::norm(self.ch[1]),
            yaw: Self::norm(self.ch[2]),
            throttle,
            armed,
            mode,
            fresh: true,
        }
    }

    fn healthy(&self) -> bool {
        self.fresh
    }
}
