//! u-blox GPS（NMEA 文本协议）驱动：经 RTOS `uart1`（USART2）读取字节流并解析。
//!
//! 不依赖 UBX 二进制配置（保持纯解析、最通用）：消费标准 NMEA-0183 的
//! `$GPGGA` / `$GPRMC` 语句，提取 WGS84 经纬度与定位状态。
//! 以**首次有效定位**为 NED 原点，后续位置转为相对原点的 NED 米（D 向下为正）。
//!
//! 实现 `flyctrl_core::hal::sensor::GpsSensor`，直接接入 EKF 的位置测量通道。

use flyctrl_core::hal::sensor::GpsSensor;
use flyctrl_core::units::Meter;
use flyctrl_core::vehicle::PosSample;

use core::ffi::c_void;

use rtos_app_sdk::abi::g_app_slot;
use rtos_app_sdk::device::Device;
use rtos_app_sdk::info;
use rtos_app_sdk::ioctl;

const DEG2RAD: f32 = core::f32::consts::PI / 180.0;
const R_EARTH: f32 = 6_378_137.0; // WGS84 半长轴 (m)

/// u-blox 出厂/常用波特率候选（升序）。`new()` 会依次尝试，锁到第一个能解出的为准。
const GPS_BAUD_CANDIDATES: [u32; 3] = [9600, 38400, 57600];

/// 单波特探测窗口（ms）。NMEA 通常 ≥1Hz，窗口内足以收到一条 GGA。
const GPS_PROBE_MS: u32 = 300;

/// RTOS 毫秒延时（经 g_app_slot.msleep）。
fn msleep(ms: u32) {
    unsafe {
        if let Some(f) = g_app_slot.msleep {
            f(ms);
        }
    }
}

/// 行内 NMEA 解析状态机：累积一行，遇 `\n` 完成。
struct NmeaLine {
    buf: [u8; 82],
    len: usize,
}

impl NmeaLine {
    fn new() -> Self {
        Self { buf: [0u8; 82], len: 0 }
    }
    /// 喂入一个字节；返回 Some(行) 表示收到完整一行（不含 `\n`），否则 None。
    fn push(&mut self, b: u8) -> Option<usize> {
        if b == b'\n' {
            if self.len > 0 && self.buf[self.len - 1] == b'\r' {
                self.len -= 1; // 去 \r
            }
            let n = self.len;
            self.len = 0;
            Some(n)
        } else {
            if self.len < self.buf.len() {
                self.buf[self.len] = b;
                self.len += 1;
            } else {
                self.len = 0; // 溢出：丢弃本行
            }
            None
        }
    }
}

/// 解析一个 NMEA 字段（`$` 之后、`,`/`*` 分隔），返回 f32（解析失败为 0）。
fn field_f32(line: &[u8], idx: usize) -> f32 {
    let mut start = 0;
    let mut cur = 0;
    for i in 0..line.len() {
        if line[i] == b',' || line[i] == b'*' {
            if cur == idx {
                if start >= i {
                    return 0.0;
                }
                // 安全解析（no_std 无 atof）：手写浮点
                return parse_float(&line[start..i]);
            }
            cur += 1;
            start = i + 1;
        }
    }
    if cur == idx {
        return parse_float(&line[start..]);
    }
    0.0
}

/// 取第 idx 个字段的原始字节切片（用于方向 N/S/E/W 等单字符）。
fn field_char(line: &[u8], idx: usize) -> u8 {
    let mut start = 0;
    let mut cur = 0;
    for i in 0..line.len() {
        if line[i] == b',' || line[i] == b'*' {
            if cur == idx && start < i {
                return line[start];
            }
            cur += 1;
            start = i + 1;
        }
    }
    0
}

/// 手写非负浮点解析（支持 `ddmm.mmmm` 与小数）。
fn parse_float(s: &[u8]) -> f32 {
    let mut val: f32 = 0.0;
    let mut frac: f32 = 0.0;
    let mut fscale: f32 = 0.1;
    let mut in_frac = false;
    for &b in s {
        if b == b'.' {
            in_frac = true;
        } else if b.is_ascii_digit() {
            let d = (b - b'0') as f32;
            if in_frac {
                frac += d * fscale;
                fscale *= 0.1;
            } else {
                val = val * 10.0 + d;
            }
        } else {
            break;
        }
    }
    val + frac
}

/// 将 NMEA 的 ddmm.mmmm 度分转为十进制度。
fn dm_to_deg(dm: f32) -> f32 {
    let deg = (dm as i32) / 100;
    let min = dm - (deg as f32) * 100.0;
    deg as f32 + min / 60.0
}

pub struct GpsUblox {
    dev: Device,
    line: NmeaLine,
    /// NED 原点经纬度（首次定位锁定），None 表示尚未建立原点。
    ref_lat: Option<f32>,
    ref_lon: Option<f32>,
    ref_alt: Option<f32>,
}

impl GpsUblox {
    pub fn new(bus_name: &str) -> Option<Self> {
        let mut dev = Device::open(bus_name)?;
        // 波特率自适应：依次尝试候选波特率，读到一个校验通过的 GGA 即锁定该波特。
        // 全部失败时回退到 u-blox 出厂默认 9600，healthy() 仍保持 false 直到真正定位。
        let mut locked = false;
        for &baud in GPS_BAUD_CANDIDATES.iter() {
            let mut b = baud;
            let _ = dev.ioctl(
                ioctl::UART_IOCTL_SET_BAUDRATE,
                &mut b as *mut u32 as *mut c_void,
            );
            if probe_baud(&mut dev, GPS_PROBE_MS) {
                locked = true;
                break;
            }
        }
        if !locked {
            let mut b = 9600u32; // 出厂默认回退
            let _ = dev.ioctl(
                ioctl::UART_IOCTL_SET_BAUDRATE,
                &mut b as *mut u32 as *mut c_void,
            );
        }
        Some(Self {
            dev,
            line: NmeaLine::new(),
            ref_lat: None,
            ref_lon: None,
            ref_alt: None,
        })
    }

    /// 从设备字节流抽取并解析一帧位置；返回 (lat_deg, lon_deg, alt_m, valid)。
    fn drain(&mut self) -> Option<(f32, f32, f32, bool)> {
        let mut rb = [0u8; 64];
        let n = self.dev.read(&mut rb);
        if n <= 0 {
            return None;
        }
        for &b in &rb[..n as usize] {
            if let Some(linelen) = self.line.push(b) {
                let line = &self.line.buf[..linelen];
                // 只关心 GGA / RMC
                let is_gga = line.starts_with(b"$GPGGA") || line.starts_with(b"$GNGGA");
                let is_rmc = line.starts_with(b"$GPRMC") || line.starts_with(b"$GNRMC");
                if !is_gga && !is_rmc {
                    continue;
                }
                if !nmea_checksum_ok(line) {
                    continue;
                }
                if is_gga {
                    // GGA: f2=lat f3=N/S f4=lon f5=E/W f6=quality(0=invalid) f9=alt
                    let quality = field_f32(line, 6);
                    if quality < 1.0 {
                        return None; // 未定位
                    }
                    let lat = dm_to_deg(field_f32(line, 2));
                    let lat = if field_char(line, 3) == b'S' { -lat } else { lat };
                    let lon = dm_to_deg(field_f32(line, 4));
                    let lon = if field_char(line, 5) == b'W' { -lon } else { lon };
                    let alt = field_f32(line, 9);
                    return Some((lat, lon, alt, true));
                } else {
                    // RMC: f3=status(A/V) f4=lat f5=N/S f6=lon f7=E/W
                    let status = field_char(line, 3);
                    if status != b'A' {
                        return None; // 无效/Void
                    }
                    let lat = dm_to_deg(field_f32(line, 4));
                    let lat = if field_char(line, 5) == b'S' { -lat } else { lat };
                    let lon = dm_to_deg(field_f32(line, 6));
                    let lon = if field_char(line, 7) == b'W' { -lon } else { lon };
                    return Some((lat, lon, 0.0, true));
                }
            }
        }
        None
    }
}

/// 校验 NMEA `*XX` 校验和（异或 `$` 与 `*` 之间所有字符）。
fn nmea_checksum_ok(line: &[u8]) -> bool {
    let mut star = None;
    for i in 1..line.len() {
        if line[i] == b'*' {
            star = Some(i);
            break;
        }
    }
    let star = match star {
        Some(s) => s,
        None => return false,
    };
    if star + 2 >= line.len() {
        return false;
    }
    let mut cs: u8 = 0;
    for &b in &line[1..star] {
        cs ^= b;
    }
    let hi = hex_val(line[star + 1]);
    let lo = hex_val(line[star + 2]);
    if hi < 0 || lo < 0 {
        return false;
    }
    cs == ((hi << 4) | lo) as u8
}

fn hex_val(b: u8) -> i32 {
    match b {
        b'0'..=b'9' => (b - b'0') as i32,
        b'A'..=b'F' => (b - b'A' + 10) as i32,
        b'a'..=b'f' => (b - b'a' + 10) as i32,
        _ => -1,
    }
}

/// 在给定已设波特率下轮询至多 `budget_ms`，返回是否解出一条校验通过的 GGA 语句。
/// 误波特率下收到的都是乱码，几乎不可能同时满足「以 `$GPGGA`/`$GNGGA` 开头 + 校验和正确」，
/// 故可作为稳健的波特判别条件。探测期间独立使用一个临时行缓冲，不改写实例状态。
fn probe_baud(dev: &mut Device, budget_ms: u32) -> bool {
    let mut line = NmeaLine::new();
    let mut waited: u32 = 0;
    let mut rb = [0u8; 64];
    while waited < budget_ms {
        let n = dev.read(&mut rb);
        if n > 0 {
            for &b in &rb[..n as usize] {
                if let Some(len) = line.push(b) {
                    let l = &line.buf[..len];
                    if (l.starts_with(b"$GPGGA") || l.starts_with(b"$GNGGA"))
                        && nmea_checksum_ok(l)
                    {
                        return true;
                    }
                }
            }
        }
        msleep(10);
        waited += 10;
    }
    false
}

impl GpsSensor for GpsUblox {
    fn read(&mut self) -> Option<PosSample> {
        let (lat, lon, alt, valid) = self.drain()?;
        if !valid {
            return None;
        }
        // 建立 NED 原点
        let (ref_lat, ref_lon, ref_alt) = match (self.ref_lat, self.ref_lon, self.ref_alt) {
            (Some(a), Some(b), Some(c)) => (a, b, c),
            _ => {
                self.ref_lat = Some(lat);
                self.ref_lon = Some(lon);
                self.ref_alt = Some(alt);
                // 首次有效定位：锁定 NED 原点（模拟器验收/真机调试均以此为 GPS 就绪标志）
                info!(tag: "gps", "fix established: lat={:.6} lon={:.6} alt={:.1} (NED origin locked)",
                      lat, lon, alt);
                (lat, lon, alt)
            }
        };
        // WGS84 → 局部 NED（首次定位原点；赤道近似 + 纬度余弦做经线缩放）
        let d_lat = (lat - ref_lat) * DEG2RAD;
        let d_lon = (lon - ref_lon) * DEG2RAD;
        let n = d_lat * R_EARTH;
        let e = d_lon * R_EARTH * libm::cosf(ref_lat * DEG2RAD);
        let d = -(alt - ref_alt); // 向下为正
        Some(PosSample::pos_only([Meter(n), Meter(e), Meter(d)]))
    }

    fn healthy(&self) -> bool {
        self.ref_lat.is_some()
    }
}
