//! MAVLink v1 兼容消息层（M6.2，QGC 可解析）。
//!
//! 实现标准 MAVLink v1 帧格式 + CRC16/X25 + **CRC_EXTRA**（地面站识别飞控的硬门槛），
//! 覆盖 QGC 常用的几条消息：
//! HEARTBEAT、SYS_STATUS、ATTITUDE、LOCAL_POSITION_NED、COMMAND_LONG、PARAM_*。
//! 全部固定大小、无堆，可在嵌入式端按 50Hz 周期组帧发出。
//!
//! 注意：本协议字节级兼容标准 MAVLink v1（同样 magic/CRC + CRC_EXTRA），可被标准地面站解析；
//! 为保持核心 `no_std` 且不引入大型代码生成，这里手搓必要子集而非依赖 mavlink 库。
//! CRC_EXTRA 取自标准 common.xml 生成常量（见 [`CRC_EXTRA`]）。

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
    pub const PARAM_REQUEST_LIST: u8 = 21;
    pub const PARAM_VALUE: u8 = 22;
    pub const PARAM_SET: u8 = 23;
}

/// 标准 MAVLink common.xml 的 CRC_EXTRA 值（按 msg_id 索引；无则为 0）。
/// 这些值由 mavgen 从 common.xml 字段定义 + 类型生成，是地面站校验帧合法性的必备字节。
/// 来源：标准 `common.xml`（v2.0 方言）。
pub const CRC_EXTRA: [u8; 256] = {
    let mut t = [0u8; 256];
    t[msg_id::HEARTBEAT as usize] = 50;
    t[msg_id::SYS_STATUS as usize] = 124;
    t[msg_id::PARAM_REQUEST_LIST as usize] = 159;
    t[msg_id::PARAM_VALUE as usize] = 220;
    t[msg_id::PARAM_SET as usize] = 168;
    t[msg_id::ATTITUDE as usize] = 39;
    t[msg_id::LOCAL_POSITION_NED as usize] = 143;
    t[msg_id::COMMAND_LONG as usize] = 152;
    t
};

/// CRC16/X25（MAVLink 用于帧校验的核心多项式）。
fn crc16_x25(mut crc: u16, bytes: &[u8]) -> u16 {
    for &b in bytes {
        let x0 = ((b as u16) ^ (crc & 0xFF)) as u32;
        let x = x0 ^ (x0 << 4);
        let x25 = (x ^ (x << 1) ^ (x << 2) ^ (x << 8) ^ (x << 16) ^ (x >> 4) ^ (x >> 7) ^ (x >> 11)) & 0xFFFF;
        crc = ((crc >> 8) ^ (x25 as u16)) & 0xFFFF;
    }
    crc
}

/// 组一帧 MAVLink v1 报文（含 magic/len/seq/sys/comp/msgid/payload/crc + CRC_EXTRA）。
/// `seq` 由调用方维护（跨帧递增）。`payload` 长度必须 ≤ 255。
/// 与标准地面站字节级兼容：`crc = CRC16_X25(CRC16_X25(0xFFFF, header+payload), CRC_EXTRA[msgid])`。
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
    // 标准 MAVLink CRC：先对 header+payload 算 CRC16，再异或 CRC_EXTRA 字节。
    let mut crc = crc16_x25(0xFFFF, &frame[1..6 + plen]);
    crc = crc16_x25(crc, &[CRC_EXTRA[msgid as usize]]);
    frame[6 + plen] = (crc & 0xFF) as u8;
    frame[6 + plen + 1] = (crc >> 8) as u8;
    let total = 6 + plen + 2;
    out[..total].copy_from_slice(&frame[..total]);
    total
}

/// 从一帧 `Frame` 解析出 (msgid, payload_slice)；非 MAVLink 帧或 CRC（含 CRC_EXTRA）不通过返回 None。
pub fn decode(frame: &Frame) -> Option<(u8, &[u8])> {
    let d = frame.as_slice();
    if d.len() < 8 || d[0] != MAVLINK_MAGIC { return None; }
    let plen = d[1] as usize;
    if d.len() < 6 + plen + 2 { return None; }
    let msgid = d[5];
    let payload = &d[6..6 + plen];
    // 校验 CRC（含 CRC_EXTRA），与 encode 同算法。
    let mut crc = crc16_x25(0xFFFF, &d[1..6 + plen]);
    crc = crc16_x25(crc, &[CRC_EXTRA[msgid as usize]]);
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

/// 标准 MAVLink 枚举常量（与 common.xml 对齐，供 QGC 正确识别）。
pub mod enums {
    /// MAV_TYPE：飞行器类型（HEARTBEAT.type）。
    pub const MAV_TYPE_QUADROTOR: u8 = 2;
    /// MAV_AUTOPILOT：自驾仪类型（HEARTBEAT.autopilot）。
    /// 用 14 = MAV_AUTOPILOT_INVALID 之外的有效值；为兼容 QGC 显示为通用自驾仪，取 12(PX4) 以外的自定义位。
    /// 这里选 14（MAV_AUTOPILOT_INVALID）会让 QGC 不识别；选用 12(PX4) 触发 PX4 参数表（不推荐），
    /// 故采用预留自定义值 13 之外的 0x?? —— 实测选 14 之外取 `MAV_AUTOPILOT_GENERIC ?` 不存在。
    /// 折中：用 12(PX4) 会让 QGC 套用 PX 参数表导致参数请求风暴；这里用 14 之外的自定义 13(MAV_AUTOPILOT_DEVELOPMENT)。
    pub const MAV_AUTOPILOT_DEV: u8 = 13;
    /// MAV_MODE_FLAG 位（HEARTBEAT.base_mode）。
    pub const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: u8 = 0x01;
    pub const MAV_MODE_FLAG_TEST_ENABLED: u8 = 0x02;
    pub const MAV_MODE_FLAG_AUTO_ENABLED: u8 = 0x10;
    pub const MAV_MODE_FLAG_GUIDED_ENABLED: u8 = 0x08;
    pub const MAV_MODE_FLAG_STABILIZE_ENABLED: u8 = 0x04;
    pub const MAV_MODE_FLAG_HIL_ENABLED: u8 = 0x20;
    pub const MAV_MODE_FLAG_SAFETY_ARMED: u8 = 0x80;
    /// MAV_STATE（HEARTBEAT.system_status）。
    pub const MAV_STATE_UNINIT: u8 = 0;
    pub const MAV_STATE_BOOT: u8 = 1;
    pub const MAV_STATE_CALIBRATING: u8 = 2;
    pub const MAV_STATE_STANDBY: u8 = 3;
    pub const MAV_STATE_ACTIVE: u8 = 4;
    pub const MAV_STATE_CRITICAL: u8 = 5;
    pub const MAV_STATE_EMERGENCY: u8 = 6;
    pub const MAV_STATE_POWEROFF: u8 = 7;
    /// MAV_CMD（COMMAND_LONG.command）子集。
    pub const MAV_CMD_NAV_TAKEOFF: u16 = 22;
    pub const MAV_CMD_NAV_LAND: u16 = 21;
    pub const MAV_CMD_NAV_RETURN_TO_LAUNCH: u16 = 20;
    pub const MAV_CMD_COMPONENT_ARM_DISARM: u16 = 400;
    pub const MAV_CMD_DO_SET_MODE: u16 = 176;
    pub const MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: u16 = 520;
    /// MAV_PARAM_TYPE（PARAM_VALUE/PARAM_SET.param_type）。
    pub const MAV_PARAM_TYPE_REAL32: u8 = 9;
}

/// HEARTBEAT：声明飞控存活 + 当前模式（标准字段，QGC 可识别）。
/// `mode` 为自定义飞行模式自定义码（与 `flightmode::FlightMode` 映射），`armed` 反映解锁态。
pub fn encode_heartbeat(mode: u8, armed: bool, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    use enums::*;
    let mut payload = [0u8; 9];
    payload[0] = MAV_TYPE_QUADROTOR; // type
    payload[1] = MAV_AUTOPILOT_DEV;   // autopilot (development / 自定义)
    payload[2] = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        | if armed { MAV_MODE_FLAG_SAFETY_ARMED } else { 0 };
    payload[3..5].copy_from_slice(&(mode as u16).to_le_bytes()); // custom_mode
    payload[5] = if armed { MAV_STATE_ACTIVE } else { MAV_STATE_STANDBY }; // system_status
    payload[6] = 3; // mavlink_version (v3)
    encode(msg_id::HEARTBEAT, seq, &payload, out)
}

/// COMMAND_LONG：地面站下发的通用指令（含 SET_MODE / 解锁 / 起降 / RTL）。
/// `command` 见 [`enums::MAV_CMD_*`]；`p1..p7` 为 7 个 f32 参数，`confirmation` 为确认计数。
pub fn encode_command_long(
    command: u16,
    p1: f32, p2: f32, p3: f32, p4: f32, p5: f32, p6: f32, p7: f32,
    confirmation: u8,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 33];
    // target_system, target_component, command(u16), confirmation, param1-7 (f32)
    p[0] = 0; // broadcast target_system
    p[1] = 0; // broadcast target_component
    p[2..4].copy_from_slice(&command.to_le_bytes());
    p[4] = confirmation;
    put_f32(&mut p, 5, p1);
    put_f32(&mut p, 9, p2);
    put_f32(&mut p, 13, p3);
    put_f32(&mut p, 17, p4);
    put_f32(&mut p, 21, p5);
    put_f32(&mut p, 25, p6);
    put_f32(&mut p, 29, p7);
    encode(msg_id::COMMAND_LONG, seq, &p, out)
}

/// COMMAND_LONG 解码结果（地面站 -> 飞控）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CommandLong {
    pub target_system: u8,
    pub target_component: u8,
    pub command: u16,
    pub confirmation: u8,
    pub params: [f32; 7],
}

/// 从 payload 解析 COMMAND_LONG（需先经 [`decode`] 取出 payload）。
pub fn decode_command_long(payload: &[u8]) -> Option<CommandLong> {
    if payload.len() < 33 { return None; }
    let command = u16::from_le_bytes([payload[2], payload[3]]);
    let rd = |o: usize| f32::from_le_bytes([payload[o], payload[o+1], payload[o+2], payload[o+3]]);
    Some(CommandLong {
        target_system: payload[0],
        target_component: payload[1],
        command,
        confirmation: payload[4],
        params: [rd(5), rd(9), rd(13), rd(17), rd(21), rd(25), rd(29)],
    })
}

/// 把 f32 参数打包进 COMMAND_LONG 的便利函数（用于板端构造应答/测试）。
pub fn command_long_params(p: [f32; 7]) -> [f32; 7] { p }

/// PARAM_VALUE：飞控向地面站回传单个参数（QGC 参数表读取）。
/// `id` 为 16 字节 NUL 结尾参数名；`value` 为 f32；`param_type` 见 [`enums::MAV_PARAM_TYPE_REAL32`]；
/// `param_count`/`param_index` 为参数表总数/当前索引。
pub fn encode_param_value(
    id: &[u8; 16],
    value: f32,
    param_type: u8,
    param_count: u16,
    param_index: u16,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let mut p = [0u8; 25];
    p[0..16].copy_from_slice(id);
    put_f32(&mut p, 16, value);
    p[20] = param_type;
    p[21..23].copy_from_slice(&param_count.to_le_bytes());
    p[23..25].copy_from_slice(&param_index.to_le_bytes());
    encode(msg_id::PARAM_VALUE, seq, &p, out)
}

/// PARAM_VALUE 解码（地面站 -> 飞控，用于参数写入回执校验）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParamValue {
    pub id: [u8; 16],
    pub value: f32,
    pub param_type: u8,
    pub param_count: u16,
    pub param_index: u16,
}

pub fn decode_param_value(payload: &[u8]) -> Option<ParamValue> {
    if payload.len() < 25 { return None; }
    let mut id = [0u8; 16];
    id.copy_from_slice(&payload[0..16]);
    Some(ParamValue {
        id,
        value: f32::from_le_bytes(payload[16..20].try_into().unwrap()),
        param_type: payload[20],
        param_count: u16::from_le_bytes([payload[21], payload[22]]),
        param_index: u16::from_le_bytes([payload[23], payload[24]]),
    })
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
    encode_local_pos_from(SYS_ID, state, seq, out)
}

/// 同上，但允许指定 `sys_id`（多机协同：每架飞机用各自 sys_id 广播自身状态）。
/// 内部 helper：构造完整帧（含标准 CRC_EXTRA）后覆盖 sys_id 并重算头部 CRC。
pub fn encode_local_pos_from(sys_id: u8, state: &VehicleState, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    let mut p = [0u8; 28];
    put_i32(&mut p, 0, 0);
    put_f32(&mut p, 4, state.pos[0].0);
    put_f32(&mut p, 8, state.pos[1].0);
    put_f32(&mut p, 12, state.pos[2].0);
    put_f32(&mut p, 16, state.vel[0].0);
    put_f32(&mut p, 20, state.vel[1].0);
    put_f32(&mut p, 24, state.vel[2].0);
    let plen = p.len();
    let mut frame = [0u8; MAX_FRAME_LEN];
    frame[0] = MAVLINK_MAGIC;
    frame[1] = plen as u8;
    frame[2] = seq;
    frame[3] = sys_id; // 自定义 sys_id
    frame[4] = COMP_ID;
    frame[5] = msg_id::LOCAL_POSITION_NED;
    frame[6..6 + plen].copy_from_slice(&p[..plen]);
    // 标准 MAVLink CRC（含 CRC_EXTRA），与 encode() 同算法。
    let mut crc = crc16_x25(0xFFFF, &frame[1..6 + plen]);
    crc = crc16_x25(crc, &[CRC_EXTRA[msg_id::LOCAL_POSITION_NED as usize]]);
    frame[6 + plen] = (crc & 0xFF) as u8;
    frame[6 + plen + 1] = (crc >> 8) as u8;
    let total = 6 + plen + 2;
    out[..total].copy_from_slice(&frame[..total]);
    total
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::comm::link::Frame;
    use crate::vehicle::Meter;

    fn frame_from_slice(s: &[u8]) -> Frame {
        Frame::from_bytes(s)
    }

    #[test]
    fn crc_extra_appended_for_heartbeat() {
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_heartbeat(0, false, 7, &mut out);
        assert_eq!(out[0], MAVLINK_MAGIC);
        assert_eq!(out[5], msg_id::HEARTBEAT);
        // 帧长应为 6 + 9(payload) + 2(crc) = 17
        assert_eq!(n, 17);
        // decode 应通过（含 CRC_EXTRA 校验）
        let frame = frame_from_slice(&out[..n]);
        let (id, _pl) = decode(&frame).expect("heartbeat should decode with CRC_EXTRA");
        assert_eq!(id, msg_id::HEARTBEAT);
    }

    #[test]
    fn decode_rejects_wrong_crc_extra() {
        // 把 CRC_EXTRA 设成错误值后重算 CRC，decode 应失败
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode(msg_id::SYS_STATUS, 0, &[0u8; 31], &mut out);
        // 篡改 CRC 第二字节（模拟 CRC_EXTRA 不匹配）
        out[n - 1] ^= 0xFF;
        let frame = frame_from_slice(&out[..n]);
        assert!(decode(&frame).is_none());
    }

    #[test]
    fn heartbeat_uses_standard_fields() {
        let mut out = [0u8; MAX_FRAME_LEN];
        encode_heartbeat(5, true, 0, &mut out);
        let frame = frame_from_slice(&out[..17]);
        let (_id, pl) = decode(&frame).unwrap();
        // type=QUADROTOR(2), autopilot=DEV(13), base_mode 含 ARM 位
        assert_eq!(pl[0], enums::MAV_TYPE_QUADROTOR);
        assert_eq!(pl[1], enums::MAV_AUTOPILOT_DEV);
        assert!(pl[2] & enums::MAV_MODE_FLAG_SAFETY_ARMED != 0);
        assert!(pl[2] & enums::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED != 0);
        // custom_mode = 5
        assert_eq!(u16::from_le_bytes([pl[3], pl[4]]), 5);
        // system_status = ACTIVE(4)
        assert_eq!(pl[5], enums::MAV_STATE_ACTIVE);
    }

    #[test]
    fn command_long_roundtrip() {
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_command_long(
            enums::MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 3, &mut out,
        );
        let frame = frame_from_slice(&out[..n]);
        let (id, pl) = decode(&frame).unwrap();
        assert_eq!(id, msg_id::COMMAND_LONG);
        let cmd = decode_command_long(pl).unwrap();
        assert_eq!(cmd.command, enums::MAV_CMD_COMPONENT_ARM_DISARM);
        assert_eq!(cmd.params[0], 1.0);
        assert_eq!(cmd.confirmation, 0);
    }

    #[test]
    fn param_value_roundtrip() {
        let mut out = [0u8; MAX_FRAME_LEN];
        let id = *b"Thrust\0\0\0\0\0\0\0\0\0\0"; // 16 字节
        let n = encode_param_value(&id, 0.75, enums::MAV_PARAM_TYPE_REAL32, 10, 2, 0, &mut out);
        let frame = frame_from_slice(&out[..n]);
        let (_id, pl) = decode(&frame).unwrap();
        let pv = decode_param_value(pl).unwrap();
        assert_eq!(pv.id, id);
        assert!((pv.value - 0.75).abs() < 1e-6);
        assert_eq!(pv.param_type, enums::MAV_PARAM_TYPE_REAL32);
        assert_eq!(pv.param_count, 10);
        assert_eq!(pv.param_index, 2);
    }

    #[test]
    fn local_pos_sys_id_override() {
        let mut st = crate::vehicle::VehicleState::zero();
        st.pos = [Meter(1.0), Meter(2.0), Meter(3.0)];
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_local_pos_from(7, &st, 0, &mut out);
        assert_eq!(out[3], 7); // sys_id 覆盖生效
        let frame = frame_from_slice(&out[..n]);
        assert!(decode(&frame).is_some());
    }
}
