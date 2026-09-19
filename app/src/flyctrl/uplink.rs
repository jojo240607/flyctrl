//! 上行接收：地面站 -> 飞控的命令通道（经 usb0 / USB CDC）。
//!
//! 设计要点（与下行 telemetry 对称、无堆、非阻塞）：
//! - `usb_dev.read` 是**非阻塞**的：RX ring 空时立即返回 0，不阻塞任务；
//!   因此 uplink 任务用轮询循环 + `msleep(1)` 让出 CPU，绝不 busy-yield。
//!   HIL 下 PC 每 4ms 注入一帧（74B），1ms 轮询保证 RX ring（256B）不被积满、
//!   bulk-OUT 端点随时可 re-arm，避免 PC 端 USB 写阻塞拖慢注入节奏（实测 10ms
//!   轮询时每 64B 块阻塞 ~3ms → 5.1x 仿真慢放）。
//! - 增量解析器 `FxParser` 单字节状态机，跨多次 read 拼出完整 MAVLink v2 帧，
//!   解决 USB-CDC 分包/粘包（一次 read 可能含半帧、多帧、或错位字节）。
//! - 路由层只处理三类上行消息：COMMAND_LONG（解锁/SET_MODE/请求能力）、
//!   PARAM_REQUEST_LIST（参数表流水）、PARAM_SET（写参数）。其余消息忽略。
//! - 应答（COMMAND_ACK / PARAM_VALUE / 能力心跳）经同一个 `usb_dev` 非阻塞写出，
//!   与下行 telemetry 共享 usb0；两者 seq 各自独立计数，互不干扰。
//! - 解锁/模式等"持久影响控制律"的状态，经 `core::G_*` 原子共享给 control 任务，
//!   避免 uplink 直接调用控制律（保持任务边界清晰、零竞争）。

use rtos_app_sdk::info;
use rtos_app_sdk::device::Device;
use crate::flyctrl::control;
use rtos_app_sdk::rtos::msleep;
use core::ffi::c_void;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU8, Ordering};
use flyctrl_core::comm::link::{Frame, MAX_FRAME_LEN, MAX_FRAME_LEN as ML_MAX};
use flyctrl_core::comm::mavlink::{self, enums, CommandLong, MAVLINK_MAGIC};
use rtos_app_sdk::abi::RTOS_PRIO_MAIN;

// HIL：注入仿真真值到 SENSOR_FRAME 所需类型（本任务为 usb0 唯一读者，承担真值写入）。
#[cfg(feature = "hil")]
use crate::flyctrl::{HIL_EVT, SENSOR_FRAME, SENSOR_SEQ};
#[cfg(feature = "hil")]
use flyctrl_core::comm::mavlink::SetPositionTargetLocalNed;
#[cfg(feature = "hil")]
use flyctrl_core::vehicle::{ImuSample, PosSample, RcInput};

// ── 全局共享状态（uplink -> control / telemetry） ──────────────────────
/// 指令解锁位：地面站经 COMMAND_LONG(ARM/DISARM) 置位；control 任务用 `rc_armed || G_CMD_ARMED`。
pub static G_CMD_ARMED: AtomicBool = AtomicBool::new(false);
/// 指令模式：地面站经 COMMAND_LONG(DO_SET_MODE) 或 TAKEOFF/LAND/RTL 设置；存 ArduCopter 标准 custom_mode。
/// 映射：STABILIZE=0, ALT_HOLD=2, LOITER=5, RTL=6, LAND=9, GUIDED=4。telemetry 心跳 custom_mode 反映此值。
pub static G_CMD_MODE: AtomicU16 = AtomicU16::new(0);
/// 当前油门百分比(0..100)：control 任务每周期写入，telemetry 经 VFR_HUD 下发。
pub static G_THROTTLE: AtomicU8 = AtomicU8::new(0);
/// 参数流水游标：PARAM_REQUEST_LIST 触发后，uplink 逐条发 PARAM_VALUE。
pub static G_PARAM_TX_IDX: AtomicU16 = AtomicU16::new(0);
/// 参数请求进行中标志（避免与周期性流水冲突）。
pub static G_PARAM_REQ: AtomicBool = AtomicBool::new(false);
/// 地面站请求自驾仪能力（COMMAND_LONG REQUEST_AUTOPILOT_CAPABILITIES）。
pub static G_CAP_REQ: AtomicBool = AtomicBool::new(false);

// ── HIL（硬件在环）共享状态 ─────────────────────────────────────────
// 说明：HIL 模式下 usb0 的【唯一读者】是本任务（sensors_task 的采样路径被
// `cfg(not(feature="hil"))` 关掉），因此仿真真值（HIL_SENSOR）与设定点
// （SET_POSITION_TARGET_LOCAL_NED）都在本任务解析，再写入 SENSOR_FRAME
// （seqlock 单写者不变式保持不变）与 G_HIL_SETPOINT。control/telemetry 只读。

/// HIL：最近一帧 PC 设定点（SET_POSITION_TARGET_LOCAL_NED 解码结果）。
/// 首个设定点到达前为全零；`G_HIL_SETPOINT_VALID` 标记是否已收到。
#[cfg(feature = "hil")]
#[link_section = ".rust_bss"]
static mut G_HIL_SETPOINT: SetPositionTargetLocalNed = SetPositionTargetLocalNed {
    time_boot_ms: 0, type_mask: 0,
    x: 0.0, y: 0.0, z: 0.0, vx: 0.0, vy: 0.0, vz: 0.0,
    afx: 0.0, afy: 0.0, afz: 0.0, yaw: 0.0, yaw_rate: 0.0,
};
/// HIL：是否已收到首个 PC 设定点（sim 连接信号）。
#[cfg(feature = "hil")]
static mut G_HIL_SETPOINT_VALID: bool = false;

/// HIL 真值累积器（HIL_SENSOR 与 SET_POSITION 分帧到达，先各自暂存，
/// 再统一写 SENSOR_FRAME，避免某一帧缺失时把另一帧的真值清空）。
#[cfg(feature = "hil")]
#[link_section = ".rust_bss"]
static mut G_HIL_IMU: Option<ImuSample> = None;
#[cfg(feature = "hil")]
#[link_section = ".rust_bss"]
static mut G_HIL_BARO: Option<f32> = None;
#[cfg(feature = "hil")]
#[link_section = ".rust_bss"]
static mut G_HIL_GPS: Option<PosSample> = None;

/// HIL：最近一帧执行器指令（motor[0..4] 归一化推力）。control 每周期写、telemetry 回传。
#[cfg(feature = "hil")]
#[link_section = ".rust_bss"]
static mut G_ACTUATOR_CMD: [f32; 4] = [0.0; 4];

/// HIL：control 任务写入执行器指令（供 telemetry 回传 HIL_ACTUATOR_CONTROLS）。
#[cfg(feature = "hil")]
pub fn set_actuator_cmd(cmd: &[f32; 4]) {
    unsafe { *core::ptr::addr_of_mut!(G_ACTUATOR_CMD) = *cmd; }
}

/// HIL：读取最近一帧执行器指令。
#[cfg(feature = "hil")]
pub fn actuator_cmd() -> [f32; 4] {
    unsafe { *core::ptr::addr_of!(G_ACTUATOR_CMD) }
}

/// HIL：读取最近一帧 PC 设定点（control 任务生成 Setpoint 用）。
#[cfg(feature = "hil")]
pub fn hil_setpoint() -> SetPositionTargetLocalNed {
    unsafe { *core::ptr::addr_of!(G_HIL_SETPOINT) }
}

/// HIL：是否已收到首个 PC 设定点（sim 连接信号）。
#[cfg(feature = "hil")]
pub fn hil_setpoint_valid() -> bool {
    unsafe { G_HIL_SETPOINT_VALID }
}

/// HIL：写入 PC 设定点 + 有效标志（共享内存路径注入；USB 路径直接 decode 后写）。
#[cfg(feature = "hil")]
pub fn set_hil_setpoint(sp: SetPositionTargetLocalNed, valid: bool) {
    unsafe {
        *core::ptr::addr_of_mut!(G_HIL_SETPOINT) = sp;
        G_HIL_SETPOINT_VALID = valid;
    }
}

/// HIL：HIL 模式遥控接收机不接；设定点来自 PC(SET_POSITION)，
/// RC 仅需 `fresh=true` 使控制环输出（armed 由 COMMAND_LONG 指令置位）。
#[cfg(feature = "hil")]
fn rc_fresh() -> RcInput {
    RcInput { roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.0, armed: false, mode: 0, fresh: true }
}

/// HIL：把累积的仿真真值整体写入 SENSOR_FRAME（seqlock，本任务为单写者）。
/// - `take()`：每类真值只投递一次。HIL_SENSOR / SET_POSITION 分帧到达，写入即清空累积器，
///   避免下一帧把过期的 IMU/气压/GPS 再次搬入 SENSOR_FRAME。
/// - 合并写入：只覆盖本拍新到的字段，`None` 不清已有真值。这样紧随 HIL_SENSOR 之后的
///   SET_POSITION 帧不会把刚写入的 IMU 覆盖成 None（IMU 是否单次消费由 control 任务负责）。
#[cfg(feature = "hil")]
fn hil_write_sensor_frame() {
    let (imu, baro, gps) = unsafe { (G_HIL_IMU.take(), G_HIL_BARO.take(), G_HIL_GPS.take()) };
    let rc = rc_fresh();
    unsafe {
        SENSOR_SEQ = SENSOR_SEQ.wrapping_add(1); // 奇：写入中
        let f = &mut *core::ptr::addr_of_mut!(SENSOR_FRAME);
        if let Some(v) = imu { f.imu = Some(v); }
        if let Some(v) = baro { f.baro_alt = Some(v); }
        if let Some(v) = gps { f.gps = Some(v); }
        f.rc = rc;
        f.armed = rc.armed;
        f.imu_ok = f.imu.is_some();
        f.baro_ok = f.baro_alt.is_some();
        f.gps_ok = f.gps.is_some();
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        SENSOR_SEQ = SENSOR_SEQ.wrapping_add(1); // 偶：写入完成
    }
    // 【HIL 事件驱动】通知 control：新一帧真值已就绪。二进制信号量计数上限 1、
    // give 已为 1 时 no-op → 同一物理步的 HIL_SENSOR/GPS/SET_POSITION 多帧连续到达时
    // 合并为一次唤醒（与 SIL "每拍取最新样本"推模式一致）；control 阻塞 `wait()` 直到
    // 此 give，实现一输入一输出闭环、不依赖自身 4ms 时钟。
    unsafe { HIL_EVT.give(); }
}

// ── 航点（MISSION）存储 + 握手状态机全局 ─────────────────────────────
/// 板载航点存储（固定数组，无堆）。最多 64 条航点（地面站常见任务规模足够）。
/// 复用 flyctrl_core::comm::mavlink::MissionItem（含 target_system 等完整字段）。
#[link_section = ".rust_bss"]
static mut G_MISSION: [mavlink::MissionItem; MISSION_MAX] =
    [mavlink::MissionItem { target_system: 0, target_component: 0, seq: 0, command: 0,
        param1: 0.0, param2: 0.0, param3: 0.0, param4: 0.0, x: 0, y: 0, z: 0.0, frame: 0, current: 0, autocontinue: 0, mission_type: 0 }; MISSION_MAX];
/// 当前已存储航点数。
#[link_section = ".rust_bss"]
static mut G_MISSION_COUNT: u16 = 0;
/// 接收握手状态：RCV_IDLE=0 不在接收；RCV_WAIT_ITEM=1 等 MISSION_ITEM_INT。
#[link_section = ".rust_bss"]
static mut G_MISSION_RCV_STATE: u8 = 0;
/// 接收握手中期望的下一个 seq（upload 时递增）。
#[link_section = ".rust_bss"]
static mut G_MISSION_RCV_NEXT: u16 = 0;
/// 接收握手目标总数（来自 MISSION_COUNT）。
#[link_section = ".rust_bss"]
static mut G_MISSION_RCV_TOTAL: u16 = 0;
const MISSION_MAX: usize = 64;
const RCV_IDLE: u8 = 0;
const RCV_WAIT_ITEM: u8 = 1;

// ── 围栏（FENCE）存储 + 握手状态机全局 ─────────────────────────────
/// 板载围栏顶点存储（固定数组，无堆）。最多 64 个顶点（含闭合点）。
/// 复用 flyctrl_core::comm::mavlink::FencePoint。
#[link_section = ".rust_bss"]
static mut G_FENCE: [mavlink::FencePoint; FENCE_MAX] =
    [mavlink::FencePoint { target_system: 0, target_component: 0, idx: 0, count: 0, lat: 0, lon: 0 }; FENCE_MAX];
/// 当前已存储围栏顶点数。
#[link_section = ".rust_bss"]
static mut G_FENCE_COUNT: u8 = 0;
const FENCE_MAX: usize = 64;

// ── RC 通道覆盖全局 ──────────────────────────────────────────────────
/// 地面站经 RC_CHANNELS_OVERRIDE 下发的 8 通道 PWM（微秒，1000-2000）。
/// `valid != 0` 表示 override 生效（control 任务据此优先于 sim RC）。
#[link_section = ".rust_bss"]
static mut G_RC_OVERRIDE: [u16; 8] = [0; 8];
#[link_section = ".rust_bss"]
static mut G_RC_OVERRIDE_VALID: u8 = 0;
/// override 新鲜度时间戳（App 单调 ticks，单位 10ms）；control 任务据此判断超时（>200 即 2s 失效）。
#[link_section = ".rust_bss"]
static mut G_RC_OVERRIDE_TICK: u32 = 0;
/// 全局 App 单调 tick（每 10ms 由 uplink 主循环 +1）；供 RC_OVERRIDE 超时判断。
#[link_section = ".rust_bss"]
static mut G_APP_TICKS: u32 = 0;

/// App 单调 tick 计数（单位 10ms）。uplink 主循环每轮 +1，control 任务读取判断超时。
fn app_ticks() -> u32 {
    unsafe { G_APP_TICKS }
}

// ── SET_MESSAGE_INTERVAL 静态存储 ───────────────────────────────────
/// 记录地面站经 MAV_CMD_SET_MESSAGE_INTERVAL 设置的 per-msgid 间隔（微秒）。
/// 0 = 未设置/默认（20ms 固定周期）。板子遥测暂固定 20ms，此处仅记录、预留给后续接入。
#[link_section = ".rust_bss"]
static mut G_MSG_INTERVAL: [i32; 256] = [0; 256];

/// 记录某消息的目标间隔（微秒）。索引为 msg_id（仅 0..255 有效）。
fn set_message_interval(msg_id: u32, interval_us: i32) {
    if msg_id < 256 {
        unsafe { G_MSG_INTERVAL[msg_id as usize] = interval_us; }
    }
}

// ── 增量帧解析器（单字节状态机，无堆） ────────────────────────────────
#[derive(Clone, Copy, PartialEq)]
enum FxState {
    Idle,    // 寻找 magic
    Header,  // 收齐 9 字节头部（含 len 域）
    Payload, // 收齐 payload + 2 CRC
}

/// MAVLink v2 增量解析器：跨多次 read 拼帧，容忍分包/粘包/错位。
#[derive(Clone, Copy)]
pub struct FxParser {
    state: FxState,
    frame: Frame, // data[..len] 为已收字节
    need: usize,  // 当前状态还需收多少字节才成帧
}

impl FxParser {
    pub const fn new() -> Self {
        FxParser {
            state: FxState::Idle,
            frame: Frame { data: [0u8; MAX_FRAME_LEN], len: 0 },
            need: 0,
        }
    }

    /// 喂一个字节；返回 Some(完整帧) 表示该字节使一帧闭合。
    fn feed_byte(&mut self, b: u8) -> Option<Frame> {
        match self.state {
            FxState::Idle => {
                if b == MAVLINK_MAGIC {
                    self.frame.data[0] = b;
                    self.frame.len = 1;
                    self.state = FxState::Header;
                    self.need = 9; // 还需 9 字节头部（incompat..msgid[3]）
                }
                // 非 magic 字节直接丢弃（容忍错位/噪声）
                None
            }
            FxState::Header => {
                self.frame.data[self.frame.len] = b;
                self.frame.len += 1;
                self.need -= 1;
                if self.need == 0 {
                    // 头部齐：payload_len = data[1]
                    let plen = self.frame.data[1] as usize;
                    let total = 10 + plen + 2; // magic(1)+9头部+payload+2crc
                    if total > MAX_FRAME_LEN {
                        // 非法长度：复位
                        self.reset();
                        return None;
                    }
                    self.need = total - self.frame.len;
                    self.state = FxState::Payload;
                }
                None
            }
            FxState::Payload => {
                self.frame.data[self.frame.len] = b;
                self.frame.len += 1;
                self.need -= 1;
                if self.need == 0 {
                    let f = self.frame;
                    self.reset();
                    return Some(f);
                }
                None
            }
        }
    }

    /// 喂一段字节，对每个闭合帧调用 `on_frame`（避免返回 Vec，零堆）。
    fn feed(&mut self, buf: &[u8], mut on_frame: impl FnMut(&Frame)) {
        for &b in buf {
            if let Some(f) = self.feed_byte(b) {
                on_frame(&f);
            }
        }
    }

    fn reset(&mut self) {
        self.state = FxState::Idle;
        self.frame.len = 0;
        self.need = 0;
    }
}

// ── 参数表（可读写，固定数组，无堆） ──────────────────────────────────
/// 参数名表（const，不可变）。名字映射到 PidController 的真实增益字段。
/// 顺序必须与 `G_PARAM_VALS` / `pid_gain_apply` 的索引约定一致（见 control.rs）。
const PARAM_NAMES: &[[u8; 16]] = &[
    *b"KpXY\0\0\0\0\0\0\0\0\0\0\0\0",
    *b"KpZ\0\0\0\0\0\0\0\0\0\0\0\0\0",
    *b"KvXY\0\0\0\0\0\0\0\0\0\0\0\0",
    *b"KvZ\0\0\0\0\0\0\0\0\0\0\0\0\0",
    *b"HoverThrust\0\0\0\0\0",
];

/// 每个参数的合法取值范围（含端点），用于 PARAM_SET 写前校验。
/// 越界写入直接拒绝（回 MAV_RESULT_FAILED），绝不写入非法增益导致控制律发散。
const PARAM_MIN: [f32; 5] = [0.0, 0.0, 0.0, 0.0, 0.1];
const PARAM_MAX: [f32; 5] = [5.0, 5.0, 5.0, 5.0, 1.0];

/// 参数值表（可读写，地面站 PARAM_SET 写入；初始值与 SIL 侧 `PidController::from_config`
/// 派生结果对齐——即 `VehicleConfig::default_quad().ctrl_params()`：kp_xy=0.3、kv_xy=0.8、
/// kv_z 保留 default_quad 基值 1.5、kp_z=0.5、hover_thrust=0.5）。
/// control 任务每周期原子读此表并应用到 pid 增益，使参数设置真正生效。
///
/// 注意：此表被强制进 `.rust_bss`（见 link_section），而系统区加载器只清零 APP_RAM、
/// 不拷贝 `.app_data` 初值——Rust 运行时也不会为 `.bss` 重填非零初值。因此源码里的
/// `[0.3,0.5,0.8,1.5,0.5]` 初值会被丢弃、运行期全 0。必须在 `init_param_defaults()`
/// 里显式写入（与 mod.rs 里 EST_STATE 的运行时填充同款手法）。
#[link_section = ".rust_bss"]
static mut G_PARAM_VALS: [f32; 5] = [0.3, 0.5, 0.8, 1.5, 0.5];

/// 运行时填充 G_PARAM_VALS 初始值（`.rust_bss` 初值被加载器清零，必须显式写）。
/// 由 spawn_flyctrl 在任务创建前调用一次。
pub fn init_param_defaults() {
    unsafe {
        *core::ptr::addr_of_mut!(G_PARAM_VALS) = [0.3, 0.5, 0.8, 1.5, 0.5];
    }
}

/// 参数个数。
const PARAM_COUNT: usize = 5;

/// 把 16 字节参数名转成 &str（截断到首个 NUL），用于日志。
fn param_name_str(name: &[u8; 16]) -> &str {
    let mut end = 0;
    while end < 16 && name[end] != 0 { end += 1; }
    // 安全：name 是源文件常量或协议字段，均为 ASCII。
    unsafe { core::str::from_utf8_unchecked(&name[..end]) }
}

// ── 应答发送 + 读取解析（非阻塞，与 telemetry 共享 usb0） ──────────────
struct UplinkTx {
    dev: Device,
    seq: u8,
    parser: FxParser,
}

impl UplinkTx {
    fn new(dev: Device) -> Self {
        UplinkTx { dev, seq: 0, parser: FxParser::new() }
    }

    /// 非阻塞轮询一次 usb0.read，喂入增量解析器；每闭合一帧即路由。
    fn poll_read(&mut self, buf: &mut [u8]) {
        let n = self.dev.read(buf);
        // 诊断：只在真正读到字节时打印（含 n==0 的噪声会淹没串口，去掉）。
        // 防御：驱动 read 可能返回负数错误码或异常长度；只在 0 < n <= buf.len() 时解析，
        // 否则跳过（避免切片越界），不阻塞轮询。
        if n > 0 && (n as usize) <= buf.len() {
            // 诊断：确认板子确实收到 OUT 字节（临时日志，验证后删除）
            info!(tag: "uplink", "RX raw n={} first16={:02X?}",
                  n, &buf[..core::cmp::min(n as usize, 16)]);
            // 解耦：feed 闭包借用 &mut self.parser，故路由用自由函数，
            // 仅捕获 self.dev（共享）与 self.seq（可变）的独立字段引用。
            let dev = &self.dev;
            let seq = &mut self.seq;
            self.parser.feed(&buf[..n as usize], |f: &Frame| {
                match mavlink::decode(f) {
                    Some((msgid, payload)) => {
                        info!(tag: "uplink", "frame decoded msgid={} plen={}", msgid, f.len);
                        route_frame(dev, seq, msgid, payload);
                    }
                    None => {
                        info!(tag: "uplink", "frame decode FAILED magic={:02X} len={}",
                              f.data[0], f.len);
                    }
                }
            });
        } else if n != 0 {
            info!(tag: "uplink", "poll_read n={} (buf.len={})", n, buf.len());
        }
    }
}

/// 非阻塞写出一帧；host 不连/无 IN-token 时 usb write 返回 0（丢帧），不阻塞。
/// 与 telemetry 共享 usb0：RTOS 侧 TX ring 写无锁（假设单生产者），此处用
/// USB_TX_MTX 串行化所有上行应答写，避免与下行遥测帧在 ring 中交错损坏。
fn send_frame(dev: &Device, out: &[u8; ML_MAX], n: usize) -> i32 {
    let _g = unsafe { crate::flyctrl::USB_TX_MTX.guard() };
    let r = dev.write(&out[..n]);
    if r <= 0 {
        info!(tag: "uplink", "send_frame FAILED n={} ret={}", n, r);
    }
    r
}

fn next_seq(seq: &mut u8) -> u8 {
    let s = *seq;
    *seq = seq.wrapping_add(1);
    s
}

fn ack(dev: &Device, seq: &mut u8, command: u16, result: u8) {
    let s = next_seq(seq);
    let mut out = [0u8; ML_MAX];
    let n = mavlink::encode_command_ack(command, result, 0, 0, s, &mut out);
    let _ = send_frame(dev, &out, n);
}

fn param_value(dev: &Device, seq: &mut u8, idx: u16) {
    let idx = idx as usize;
    if idx >= PARAM_COUNT { return; }
    let name = PARAM_NAMES[idx];
    let val = unsafe { G_PARAM_VALS[idx] };
    let s = next_seq(seq);
    let mut out = [0u8; ML_MAX];
    let n = mavlink::encode_param_value(
        &name,
        val,
        enums::MAV_PARAM_TYPE_REAL32,
        PARAM_COUNT as u16,
        idx as u16,
        s,
        &mut out,
    );
    let _ = send_frame(dev, &out, n);
}

// ── 命令路由 ──────────────────────────────────────────────────────────
/// 处理一帧已 decode 的 MAVLink 消息（自由函数，避免与 parser 的可变借用冲突）。
fn route_frame(dev: &Device, seq: &mut u8, msgid: u32, payload: &[u8]) {
    match msgid {
        mavlink::msg_id::COMMAND_LONG => {
            if let Some(cmd) = mavlink::decode_command_long(payload) {
                handle_command_long(dev, seq, cmd);
            }
        }
        mavlink::msg_id::PARAM_REQUEST_LIST => {
            // 触发参数流水（下次循环逐条发）
            G_PARAM_TX_IDX.store(0, Ordering::Relaxed);
            G_PARAM_REQ.store(true, Ordering::Relaxed);
            info!(tag: "uplink", "PARAM_REQUEST_LIST rx; start streaming {} params", PARAM_COUNT);
        }
        mavlink::msg_id::PARAM_SET => {
            if let Some(ps) = mavlink::decode_param_set(payload) {
                handle_param_set(dev, seq, ps);
            }
        }
        mavlink::msg_id::PARAM_REQUEST_READ => {
            // 按参数名点读单个参数（地面站参数表单点刷新）。
            if let Some((id, _idx)) = mavlink::decode_param_request_read(payload) {
                let name = param_name_str(&id);
                if let Some(i) = find_param(&id) {
                    param_value(dev, seq, i as u16);
                    info!(tag: "uplink", "PARAM_REQUEST_READ '{}' -> idx={}", name, i);
                } else {
                    info!(tag: "uplink", "PARAM_REQUEST_READ '{}' not found", name);
                }
            }
        }
        // ── 航点（MISSION）握手 ──────────────────────────────────
        mavlink::msg_id::MISSION_REQUEST_LIST => {
            mission_handle_request_list(dev);
        }
        mavlink::msg_id::MISSION_COUNT => {
            if let Some(count) = mavlink::decode_mission_count(payload) {
                mission_handle_count(dev, count);
            }
        }
        mavlink::msg_id::MISSION_ITEM_INT => {
            if let Some(item) = mavlink::decode_mission_item_int(payload) {
                mission_handle_item(dev, &item);
            }
        }
        // ── 围栏（FENCE）握手 ──────────────────────────────────
        mavlink::msg_id::FENCE_POINT => {
            if let Some(pt) = mavlink::decode_fence_point(payload) {
                fence_handle_point(&pt);
            }
        }
        mavlink::msg_id::FENCE_FETCH_POINT => {
            if let Some(idx) = mavlink::decode_fence_fetch_point(payload) {
                fence_handle_fetch(dev, idx);
            }
        }
        // ── 数据流速率控制（REQUEST_DATA_STREAM / DATA_STREAM）────────
        mavlink::msg_id::REQUEST_DATA_STREAM => {
            if let Some((stream_id, rate_hz, start_stop)) = mavlink::decode_request_data_stream(payload) {
                // 板子遥测固定 20ms；以请求的 rate 回 DATA_STREAM 确认（on_off 随 start_stop）。
                let on_off: u8 = if start_stop > 0 && rate_hz > 0 { 1 } else { 0 };
                let mut out = [0u8; ML_MAX];
                let n = mavlink::encode_data_stream(stream_id, rate_hz, on_off, &mut out);
                let _ = send_frame(dev, &out, n);
                info!(tag: "uplink", "REQUEST_DATA_STREAM stream={} rate={}Hz on={} -> DATA_STREAM", stream_id, rate_hz, on_off);
            }
        }
        // ── RC 通道覆盖（地面站手动操控）──────────────────────────
        mavlink::msg_id::RC_CHANNELS_OVERRIDE => {
            if let Some(ch) = mavlink::decode_rc_channels_override(payload) {
                unsafe {
                    G_RC_OVERRIDE = ch;
                    G_RC_OVERRIDE_VALID = 1;
                    G_RC_OVERRIDE_TICK = app_ticks();
                }
                info!(tag: "uplink", "RC_OVERRIDE ch1={} ch2={} -> valid", ch[0], ch[1]);
            }
        }
        // ── HIL（硬件在环）：PC 仿真器注入传感器真值 + 设定点 ─────
        #[cfg(feature = "hil")]
        mavlink::msg_id::HIL_SENSOR => {
            if let Some((_t, xacc, yacc, zacc, xgyro, ygyro, zgyro,
                         _mx, _my, _mz, _ap, _dp, p_alt, _temp, _fu))
                = mavlink::decode_hil_sensor(payload)
            {
                use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond};
                let imu = ImuSample {
                    accel: [MeterPerSecondSquared(xacc), MeterPerSecondSquared(yacc), MeterPerSecondSquared(zacc)],
                    gyro: [RadianPerSecond(xgyro), RadianPerSecond(ygyro), RadianPerSecond(zgyro)],
                };
                unsafe { G_HIL_IMU = Some(imu); G_HIL_BARO = Some(p_alt); }
                hil_write_sensor_frame();
                info!(tag: "uplink", "HIL_SENSOR accel=({:.2},{:.2},{:.2}) gyro=({:.2},{:.2},{:.2}) alt={:.2}",
                      xacc, yacc, zacc, xgyro, ygyro, zgyro, p_alt);
            }
        }
        #[cfg(feature = "hil")]
        mavlink::msg_id::SET_POSITION_TARGET_LOCAL_NED => {
            if let Some(sp) = mavlink::decode_set_position_target_local_ned(payload) {
                unsafe {
                    *core::ptr::addr_of_mut!(G_HIL_SETPOINT) = sp;
                    G_HIL_SETPOINT_VALID = true;
                    // 注意：本消息【只作设定点】。GPS 位置/速度真值改由独立的
                    // HIL_GPS(113) 分支注入（G_HIL_GPS），避免真值被固定为设定点
                    // 导致 EKF 无法跟踪真实位置/速度（见 HIL_GPS 分支注释）。
                }
                hil_write_sensor_frame();
                info!(tag: "uplink", "HIL_SETPOINT pos=({:.2},{:.2},{:.2}) vel=({:.2},{:.2},{:.2}) yaw={:.2}",
                      sp.x, sp.y, sp.z, sp.vx, sp.vy, sp.vz, sp.yaw);
            }
        }
        #[cfg(feature = "hil")]
        mavlink::msg_id::HIL_GPS => {
            // GPS 真值（HIL_GPS(113)）：与 SET_POSITION 设定点解耦。PC 把物理引擎
            // 真实 NED 位置/速度经 lat/lon/alt(·1e7/·1e3) 与 vn/ve/vd(·1e2) 缩放装入，
            // 这里按同一缩放还原。EKF 融合真实位置（update_pos）+ Doppler 速度
            // （update_vel），才能正确跟踪爬升/位移并让位置环有真实误差可修。
            if let Some((_t, _fix, lat, lon, alt, _eph, _epv, _vel, vn, ve, vd, _cog, _sat))
                = mavlink::decode_hil_gps(payload)
            {
                use flyctrl_core::units::{Meter, MeterPerSecond};
                unsafe {
                    G_HIL_GPS = Some(PosSample::with_vel(
                        [Meter(lat as f32 * 1e-7), Meter(lon as f32 * 1e-7), Meter(alt as f32 * 1e-3)],
                        [MeterPerSecond(vn as f32 * 1e-2), MeterPerSecond(ve as f32 * 1e-2), MeterPerSecond(vd as f32 * 1e-2)],
                    ));
                }
                hil_write_sensor_frame();
            }
        }
        _ => {
            // 其余消息（HEARTBEAT/ATTITUDE 等上行）第一版忽略
        }
    }
}

fn handle_command_long(dev: &Device, seq: &mut u8, cmd: CommandLong) {
    // 仅响应广播(0)或本机(sys=1)目标
    if cmd.target_system != 0 && cmd.target_system != mavlink::SYS_ID {
        return;
    }
    match cmd.command {
        enums::MAV_CMD_COMPONENT_ARM_DISARM => {
            let arm = cmd.params[0] > 0.5;
            G_CMD_ARMED.store(arm, Ordering::Relaxed);
            control::set_cmd_armed(arm);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "ARM_DISARM cmd={} -> armed={}", cmd.command, arm);
        }
        enums::MAV_CMD_DO_SET_MODE => {
            // 标准 MAVLink：param1 = base_mode（含 CUSTOM_MODE_ENABLED 位 0x80），param2 = custom_mode。
            let custom = cmd.params[1] as u16; // ArduCopter 自定义模式码（0/2/5/6/9...）
            G_CMD_MODE.store(custom, Ordering::Relaxed);
            control::set_cmd_mode(custom);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "DO_SET_MODE -> custom_mode={}", custom);
        }
        // 起飞/降落/返航：直接切到对应 ArduCopter 自定义模式，地面站据此显示模式名。
        enums::MAV_CMD_NAV_TAKEOFF => {
            // TAKEOFF 在悬停类模式中按 ALT_HOLD（带目标高度）处理。
            let mode = enums::COPTER_MODE_ALT_HOLD;
            G_CMD_MODE.store(mode, Ordering::Relaxed);
            control::set_cmd_mode(mode);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "TAKEOFF -> mode={}", mode);
        }
        enums::MAV_CMD_NAV_LAND => {
            let mode = enums::COPTER_MODE_LAND;
            G_CMD_MODE.store(mode, Ordering::Relaxed);
            control::set_cmd_mode(mode);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "LAND -> mode={}", mode);
        }
        enums::MAV_CMD_NAV_RETURN_TO_LAUNCH => {
            let mode = enums::COPTER_MODE_RTL;
            G_CMD_MODE.store(mode, Ordering::Relaxed);
            control::set_cmd_mode(mode);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "RTL -> mode={}", mode);
        }
        enums::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES => {
            G_CAP_REQ.store(true, Ordering::Relaxed);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "REQUEST_AUTOPILOT_CAPABILITIES rx");
        }
        enums::MAV_CMD_SET_MESSAGE_INTERVAL => {
            // param1 = msg_id（要改频率的消息），param2 = interval_us（0=停止）。
            // 板子遥测为固定 20ms 周期，此处仅记录请求并 ACK 确认；
            // 真正的动态间隔（per-msgid 表）留待后续可视需要接入 telemetry。
            let target_msg: u32 = cmd.params[0] as u32;
            let interval_us: i32 = cmd.params[1] as i32;
            set_message_interval(target_msg, interval_us);
            ack(dev, seq, cmd.command, enums::MAV_RESULT_ACCEPTED);
            info!(tag: "uplink", "SET_MESSAGE_INTERVAL msg={} interval_us={} -> ACK", target_msg, interval_us);
        }
        _ => {
            // 第一版未实现的指令：明确拒绝（便于地面站诊断）
            ack(dev, seq, cmd.command, enums::MAV_RESULT_UNSUPPORTED);
            info!(tag: "uplink", "CMD {} unsupported -> ACK_UNSUPPORTED", cmd.command);
        }
    }
}

/// 按 16 字节名查找参数索引；找不到返回 None。
fn find_param(id: &[u8; 16]) -> Option<usize> {
    for i in 0..PARAM_COUNT {
        if PARAM_NAMES[i] == *id {
            return Some(i);
        }
    }
    None
}

// ── 航点（MISSION）握手处理函数 ─────────────────────────────────────
/// MISSION 下载：地面站请求全部航点 -> 回 MISSION_COUNT + 逐个 MISSION_ITEM_INT + MISSION_ACK。
fn mission_handle_request_list(dev: &Device) {
    let count = unsafe { G_MISSION_COUNT };
    let mut out = [0u8; ML_MAX];
    let n = mavlink::encode_mission_count(count, &mut out);
    let _ = send_frame(dev, &out, n);
    for seq in 0..count {
        let item = unsafe { G_MISSION[seq as usize] };
        let n = mavlink::encode_mission_item_int(&item, &mut out);
        let _ = send_frame(dev, &out, n);
    }
    let n = mavlink::encode_mission_ack(0, &mut out); // ACCEPTED
    let _ = send_frame(dev, &out, n);
    info!(tag: "uplink", "MISSION_REQUEST_LIST -> download {} items", count);
}

/// MISSION 上传开始：地面站宣布总数 -> 进入接收态并请求第 0 条。
fn mission_handle_count(dev: &Device, count: u16) {
    if count as usize > MISSION_MAX {
        let mut out = [0u8; ML_MAX];
        let n = mavlink::encode_mission_ack(4, &mut out); // NO_SPACE
        let _ = send_frame(dev, &out, n);
        return;
    }
    unsafe {
        G_MISSION_RCV_STATE = RCV_WAIT_ITEM;
        G_MISSION_RCV_NEXT = 0;
        G_MISSION_RCV_TOTAL = count;
    }
    let mut out = [0u8; ML_MAX];
    let n = mavlink::encode_mission_request(0, &mut out);
    let _ = send_frame(dev, &out, n);
    info!(tag: "uplink", "MISSION_COUNT={} -> requesting seq 0", count);
}

/// MISSION 上传：收到单条航点 -> 存储 + 请求下一条或结束握手。
fn mission_handle_item(dev: &Device, item: &mavlink::MissionItem) {
    let (state, next, total) = unsafe { (G_MISSION_RCV_STATE, G_MISSION_RCV_NEXT, G_MISSION_RCV_TOTAL) };
    if state != RCV_WAIT_ITEM {
        return;
    }
    if item.seq != next {
        // 序号不符：回 INVALID_SEQUENCE，要求重发当前 next
        let mut out = [0u8; ML_MAX];
        let n = mavlink::encode_mission_ack(5, &mut out); // INVALID_SEQUENCE
        let _ = send_frame(dev, &out, n);
        info!(tag: "uplink", "MISSION item seq={} != expected {} -> INVALID_SEQUENCE", item.seq, next);
        return;
    }
    unsafe {
        G_MISSION[item.seq as usize] = *item;
    }
    let next2 = next + 1;
    if next2 >= total {
        unsafe {
            G_MISSION_COUNT = total;
            G_MISSION_RCV_STATE = RCV_IDLE;
            G_MISSION_RCV_NEXT = 0;
        }
        let mut out = [0u8; ML_MAX];
        let n = mavlink::encode_mission_ack(0, &mut out); // ACCEPTED
        let _ = send_frame(dev, &out, n);
        info!(tag: "uplink", "MISSION upload complete: {} items", total);
    } else {
        unsafe { G_MISSION_RCV_NEXT = next2; }
        let mut out = [0u8; ML_MAX];
        let n = mavlink::encode_mission_request(next2, &mut out);
        let _ = send_frame(dev, &out, n);
        info!(tag: "uplink", "MISSION item seq={} stored, requesting seq={}", item.seq, next2);
    }
}

// ── 围栏（FENCE）握手处理函数 ─────────────────────────────────────
/// FENCE 下载：地面站请求某条围栏顶点 -> 回 FENCE_POINT（含 count）。
fn fence_handle_fetch(dev: &Device, idx: u8) {
    let count = unsafe { G_FENCE_COUNT };
    if idx as usize >= count as usize {
        info!(tag: "uplink", "FENCE_FETCH idx={} >= count={} -> ignored", idx, count);
        return;
    }
    let pt = unsafe { G_FENCE[idx as usize] };
    let mut out = [0u8; ML_MAX];
    let n = mavlink::encode_fence_point(&pt, &mut out);
    let _ = send_frame(dev, &out, n);
    info!(tag: "uplink", "FENCE_FETCH idx={} -> lat={} lon={} (count={})", idx, pt.lat, pt.lon, count);
}

/// FENCE 上传：地面站发单条 FENCE_POINT -> 存储（idx 即写入位置），count 到齐即收尾。
fn fence_handle_point(pt: &mavlink::FencePoint) {
    let idx = pt.idx as usize;
    if idx >= FENCE_MAX {
        info!(tag: "uplink", "FENCE_POINT idx={} >= FENCE_MAX -> ignored", idx);
        return;
    }
    unsafe {
        G_FENCE[idx] = *pt;
        // 以 idx+1 作为当前总数（地面站按顺序上传，最后一条的 count 即总点数）。
        if pt.count as usize > G_FENCE_COUNT as usize {
            G_FENCE_COUNT = pt.count;
        }
        if (idx + 1) as u8 > G_FENCE_COUNT {
            G_FENCE_COUNT = (idx + 1) as u8;
        }
    }
    info!(tag: "uplink", "FENCE_POINT idx={} lat={} lon={} count={} -> stored", idx, pt.lat, pt.lon, pt.count);
}

/// 读取围栏顶点数（control 任务或 telemetry 可调用）。
pub fn get_fence_count() -> u8 {
    unsafe { G_FENCE_COUNT }
}

/// 读取围栏顶点（地面站下载用，已由 FENCE_FETCH_POINT 直接经 encode 发出）。
pub fn get_fence_point(idx: usize) -> Option<mavlink::FencePoint> {
    if idx >= unsafe { G_FENCE_COUNT } as usize { return None; }
    Some(unsafe { G_FENCE[idx] })
}

/// 读取 RC_OVERRIDE（control 任务调用）：返回 (ch[8], valid)。
/// 超过 2s 未刷新视为失效，control 应回退到 sim RC。
pub fn get_rc_override() -> ([u16; 8], bool) {
    unsafe {
        let valid = G_RC_OVERRIDE_VALID != 0
            && (app_ticks().wrapping_sub(G_RC_OVERRIDE_TICK) < 200);
        (G_RC_OVERRIDE, valid)
    }
}

fn handle_param_set(dev: &Device, seq: &mut u8, ps: mavlink::ParamSet) {
    // 参数表可读写：找到同名项 -> 范围校验 -> 写入并回显（带 MAV_RESULT）。
    let name = param_name_str(&ps.id);
    match find_param(&ps.id) {
        Some(i) => {
            // 写前范围校验：越界直接拒绝，绝不写入非法增益。
            let v = ps.value;
            if v < PARAM_MIN[i] || v > PARAM_MAX[i] {
                // 越界：回 COMMAND_ACK(FAILED)，command 填 PARAM_SET(msg_id=23) 作约定。
                ack(dev, seq, mavlink::msg_id::PARAM_SET as u16, enums::MAV_RESULT_FAILED);
                info!(tag: "uplink", "PARAM_SET '{}' = {:.4} OUT OF RANGE [{:.2},{:.2}] -> REJECT",
                      name, v, PARAM_MIN[i], PARAM_MAX[i]);
                return;
            }
            unsafe { G_PARAM_VALS[i] = v; }
            // 标准做法：仅回显新值（PARAM_VALUE），地面站据此确认写入成功。
            // 越界分支才回 COMMAND_ACK(FAILED)，合法分支靠回显帧确认。
            param_value(dev, seq, i as u16);
            info!(tag: "uplink", "PARAM_SET '{}' = {:.4} (written, applied next ctrl tick)", name, v);
        }
        None => {
            info!(tag: "uplink", "PARAM_SET '{}' not found -> ignored", name);
        }
    }
}

// ── 参数 -> 控制律桥接（control 任务每周期调用，零锁、原子读） ──────────
/// 把地面站参数表 `G_PARAM_VALS` 应用到给定 PID 控制器。
/// control 任务每周期调用一次，使 PARAM_SET 在下一控制拍立即生效。
/// 读取用 Relaxed 原子序（uplink 单写、control 单读，无需 Acquire/Release 同步语义）。
pub fn sync_gains_to_pid(pid: &mut flyctrl_core::controller::PidController) {
    let mut g = [0f32; 5];
    for i in 0..PARAM_COUNT {
        g[i] = unsafe { G_PARAM_VALS[i] };
    }
    pid.apply_gains(&g);
}

// ── uplink 任务入口 ───────────────────────────────────────────────────
pub extern "C" fn uplink_task(_arg: *mut c_void) {
    info!(tag: "uplink", "task started; poll usb0.read, prio={}", RTOS_PRIO_MAIN);

    // 复用 boot 已打开的 usb0（见 telemetry 的 Device::get 约定，避免二次 open 重置 USB 状态机）。
    let usb_dev = match Device::get("usb0\0") {
        Some(d) => d,
        None => {
            info!(tag: "uplink", "usb0 not found; uplink disabled");
            return;
        }
    };

    let mut tx = UplinkTx::new(usb_dev);
    let mut rx_buf = [0u8; 64];

    let mut loops: u32 = 0;
    loop {
        // 非阻塞轮询 usb0.read + 增量解析 + 路由（RX ring 空时返回 0，不阻塞）。
        // 一期固件不带机载电脑 → 默认不轮询 usb0（见 Cargo.toml `usb-link` 说明）。
        #[cfg(feature = "usb-link")]
        tx.poll_read(&mut rx_buf);
        #[cfg(not(feature = "usb-link"))]
        let _ = (&tx, &mut rx_buf);

        // HIL 共享内存直连（SRAM3）：PC 仿真器每 4ms 写 pc_seq，本任务 1ms 轮询
        // 检测变化后全量注入 SENSOR_FRAME 并唤醒 control。与 USB 注入并存：
        // 有 magic 且 pc_seq 变化即注入，无则静默（USB 路径不受影响）。
        #[cfg(feature = "hil")]
        crate::flyctrl::hil_shmem::shmem_poll_once();

        // 参数流水：每次循环最多发一条，避免单次 burst 占满 USB 下行缓冲。
        if G_PARAM_REQ.load(Ordering::Relaxed) {
            let idx = G_PARAM_TX_IDX.load(Ordering::Relaxed) as usize;
            if idx < PARAM_COUNT {
                param_value(&tx.dev, &mut tx.seq, idx as u16);
                G_PARAM_TX_IDX.store((idx + 1) as u16, Ordering::Relaxed);
            } else {
                // 流水完成
                G_PARAM_REQ.store(false, Ordering::Relaxed);
                info!(tag: "uplink", "param stream done ({} items)", PARAM_COUNT);
            }
        }

        // 能力请求：真发 AUTOPILOT_VERSION 帧（响应 REQUEST_AUTOPILOT_CAPABILITIES）。
        if G_CAP_REQ.load(Ordering::Relaxed) {
            G_CAP_REQ.store(false, Ordering::Relaxed);
            let caps = enums::MAV_PROTOCOL_CAPABILITY_MAVLINK2
                | enums::MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
            let s = next_seq(&mut tx.seq);
            let mut out = [0u8; ML_MAX];
            let n = mavlink::encode_autopilot_version(caps, s, &mut out);
            let _ = send_frame(&tx.dev, &out, n);
            info!(tag: "uplink", "AUTOPILOT_VERSION sent (cap=MAVLINK2|PARAM_FLOAT)");
        }

        // 低频存活日志（约每 10s 一次），用于联调确认 uplink 任务未卡死。
        loops += 1;
        unsafe { G_APP_TICKS = G_APP_TICKS.wrapping_add(1); }
        if loops % 10000 == 0 {
            info!(tag: "uplink", "poll alive loop={}", loops);
        }

        // 让出 CPU 1ms（HIL 下保证 RX ring 快速 drain/re-arm，降低 PC 写阻塞）。
        msleep(1);
    }
}
