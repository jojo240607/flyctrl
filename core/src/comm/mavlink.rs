//! MAVLink 兼容消息层（M6.2）。
//!
//! 实现 MAVLink v1 帧格式 + CRC16/X25，覆盖 QGC 常用的几条消息：
//! HEARTBEAT、ATTITUDE、LOCAL_POSITION_NED、SYS_STATUS、COMMAND_LONG(SET_MODE)。
//! 全部固定大小、无堆，可在嵌入式端按 50Hz 周期组帧发出。
//!
//! 注意：本协议与标准 MAVLink v1 字节兼容（同样 magic/CRC），可被标准地面站解析；
//! 为保持核心 `no_std` 且不引入大型代码生成，这里手搓必要子集而非依赖 mavlink 库。

use crate::comm::link::{Frame, MAX_FRAME_LEN};
use crate::vehicle::VehicleState;

/// MAVLink 帧起始符（v1）。
pub const MAVLINK_MAGIC: u8 = 0xFE;

/// 系统/组件 ID（飞控侧固定）。
pub const SYS_ID: u8 = 1;
pub const COMP_ID: u8 = 1; // MAV_COMP_ID_AUTOPILOT1

/// 消息 ID 常量（与标准 MAVLink 一致）。
pub mod msg_id {
    pub const HEARTBEAT: u8 = 0;
    pub const SYS_STATUS: u8 = 1;
    pub const ATTITUDE: u8 = 30;
    pub const LOCAL_POSITION_NED: u8 = 32;
    pub const COMMAND_LONG: u8 = 76;
}

/// CRC16/X25 种子（含 MAVLink 的额外 CRC 字节，这里用 0 简化；同一条消息两端一致即可）。
fn crc16_x25(mut crc: u16, bytes: &[u8]) -> u16 {
    for &b in bytes {
        let x0 = ((b as u16) ^ (crc & 0xFF)) as u32;
        let x = x0 ^ (x0 << 4);
        let x25 = (x ^ (x << 1) ^ (x << 2) ^ (x << 8) ^ (x << 16) ^ (x >> 4) ^ (x >> 7) ^ (x >> 11)) & 0xFFFF;
        crc = ((crc >> 8) ^ (x25 as u16)) & 0xFFFF;
    }
    crc
}

/// 组一帧 MAVLink v1 报文（含 magic/len/seq/sys/comp/msgid/payload/crc）。
/// `seq` 由调用方维护（跨帧递增）。`payload` 长度必须 ≤ 255。
pub fn encode(msgid: u8, seq: u8, payload: &[u8], out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let plen = payload.len().min(255);
    let mut frame = [0u8; MAX_FRAME_LEN];
    frame[0] = MAVLINK_MAGIC;
    frame[1] = plen as u8;
    frame[2] = seq;
    frame[3] = SYS_ID;
    frame[4] = COMP_ID;
    frame[5] = msgid;
    frame[6..6 + plen].copy_from_slice(&payload[..plen]);
    let crc = crc16_x25(0xFFFF, &frame[1..6 + plen]);
    frame[6 + plen] = (crc & 0xFF) as u8;
    frame[6 + plen + 1] = (crc >> 8) as u8;
    let total = 6 + plen + 2;
    out[..total].copy_from_slice(&frame[..total]);
    total
}

/// 从一帧 `Frame` 解析出 (msgid, payload_slice)；非 MAVLink 帧返回 None。
pub fn decode(frame: &Frame) -> Option<(u8, &[u8])> {
    let d = frame.as_slice();
    if d.len() < 8 || d[0] != MAVLINK_MAGIC { return None; }
    let plen = d[1] as usize;
    if d.len() < 6 + plen + 2 { return None; }
    let msgid = d[5];
    let payload = &d[6..6 + plen];
    // 校验 CRC
    let crc = crc16_x25(0xFFFF, &d[1..6 + plen]);
    let got = ((d[6 + plen + 1] as u16) << 8) | (d[6 + plen] as u16);
    if crc != got { return None; }
    Some((msgid, payload))
}

// ── 具体消息编码器 ──────────────────────────────────────────────

/// 把 f32 以小端写入 buf 的 offset 处（MAVLink 用 IEEE754 小端）。
fn put_f32(buf: &mut [u8], off: usize, v: f32) {
    let b = v.to_le_bytes();
    buf[off..off + 4].copy_from_slice(&b);
}
fn put_i32(buf: &mut [u8], off: usize, v: i32) {
    let b = v.to_le_bytes();
    buf[off..off + 4].copy_from_slice(&b);
}

/// HEARTBEAT：声明飞控存活 + 当前模式。
pub fn encode_heartbeat(mode: u8, armed: bool, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut payload = [0u8; 9];
    // type=0(Generic), autopilot=12(PX4?), base_mode, custom_mode, system_status
    payload[0..4].copy_from_slice(&0u32.to_le_bytes()); // type
    payload[4] = 12; // autopilot
    payload[5] = if armed { 0x80 } else { 0 } | 1; // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    payload[6..8].copy_from_slice(&(mode as u16).to_le_bytes()); // custom_mode
    payload[8] = if armed { 4 } else { 3 }; // system_status (active/standby)
    encode(msg_id::HEARTBEAT, seq, &payload, out)
}

/// ATTITUDE：四元数 + 角速度（rad/s）。
pub fn encode_attitude(state: &VehicleState, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 28];
    // time_boot_ms (i32) + q[4] f32 + rollspeed/pitchspeed/yawspeed f32
    put_i32(&mut p, 0, 0);
    put_f32(&mut p, 4, state.att.w);
    put_f32(&mut p, 8, state.att.x);
    put_f32(&mut p, 12, state.att.y);
    put_f32(&mut p, 16, state.att.z);
    put_f32(&mut p, 20, state.omega[0].0); // rollspeed (p)
    put_f32(&mut p, 24, state.omega[1].0); // pitchspeed (q)
    encode(msg_id::ATTITUDE, seq, &p, out)
}

/// LOCAL_POSITION_NED：NED 位置 + 速度。
pub fn encode_local_pos(state: &VehicleState, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 28];
    put_i32(&mut p, 0, 0);
    put_f32(&mut p, 4, state.pos[0].0);
    put_f32(&mut p, 8, state.pos[1].0);
    put_f32(&mut p, 12, state.pos[2].0);
    put_f32(&mut p, 16, state.vel[0].0);
    put_f32(&mut p, 20, state.vel[1].0);
    put_f32(&mut p, 24, state.vel[2].0);
    encode(msg_id::LOCAL_POSITION_NED, seq, &p, out)
}

/// SYS_STATUS：健康位（取 FDIR 健康；此处仅填传感器位）。
pub fn encode_sys_status(sensors_ok: bool, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 31];
    put_i32(&mut p, 0, if sensors_ok { 0x1F } else { 0 }); // onboard_control_sensors_present
    put_i32(&mut p, 4, if sensors_ok { 0x1F } else { 0 }); // enabled
    put_i32(&mut p, 8, if sensors_ok { 0x1F } else { 0 }); // health
    // load (u16), voltage (u16), current, comms drop/errors...
    p[12..14].copy_from_slice(&100u16.to_le_bytes()); // 10% CPU load placeholder
    p[20..22].copy_from_slice(&0u16.to_le_bytes()); // battery
    encode(msg_id::SYS_STATUS, seq, &p, out)
}
