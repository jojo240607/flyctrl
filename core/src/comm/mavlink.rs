//! MAVLink v2 兼容消息层（M6.2，QGC 可解析）。
//!
//! 本文件自 `mavlink-core` 抽取后改为 **re-export 薄层**：帧层（magic/9 字节头部/
//! CRC16+CRC_EXTRA/msg_id/enums）与全部具体消息编解码统一收口到
//! [`mavlink_core`]（单一事实来源），固件/仿真器/地面站/App 共用同一份实现。
//!
//! 本文件只保留两样东西：
//! 1. 对 [`mavlink_core::frame`] / [`mavlink_core::codec`] 的 re-export，
//!    使调用方 `use crate::comm::mavlink` 的接口完全不变；
//! 2. 依赖 [`VehicleState`] 的便捷封装（`encode_attitude` / `encode_local_pos` /
//!    `encode_vfr_hud` / `encode_global_position_int` 等），内部换算后委托给
//!    `mavlink_core` 的 `*_raw` 函数。

use crate::vehicle::VehicleState;

// ── 帧层与编解码：re-export 自 mavlink-core ─────────────────────

pub use mavlink_core::frame::{
    crc16_x25, decode, encode, put_f32, put_i16, put_i32, put_u16, MAVLINK_MAGIC, CRC_EXTRA,
    COMP_ID, MAX_FRAME_LEN, SYS_ID, Frame, enums, msg_id,
};
pub use mavlink_core::codec::{
    Attitude, CommandLong, FencePoint, HilActuatorControls, LocalPositionNed, MissionItem,
    ParamSet, ParamValue, SetPositionTargetLocalNed, command_long_params,
    decode_attitude, decode_command_ack, decode_command_long, decode_fence_fetch_point,
    decode_fence_point, decode_hil_rc_inputs_raw, decode_hil_sensor, decode_hil_actuator_controls,
    decode_hil_gps, decode_local_position_ned,
    decode_mission_count, decode_mission_item_int, decode_mission_request,
    decode_mission_request_list, decode_param_request_list, decode_param_request_read,
    decode_param_set, decode_param_value, decode_rc_channels_override, decode_request_data_stream,
    decode_set_position_target_local_ned, encode_attitude_raw, encode_autopilot_version,
    encode_command_ack, encode_command_long, encode_data_stream, encode_fence_point,
    encode_global_position_int_raw, encode_heartbeat, encode_heartbeat_ap, encode_heartbeat_hil,
    encode_hil_actuator_controls, encode_hil_sensor, encode_hil_gps, encode_local_pos_from_raw,
    encode_local_pos_raw, encode_set_position_target_local_ned,
    encode_mission_ack, encode_mission_count, encode_mission_item_int, encode_mission_request,
    encode_param_request_list, encode_param_value, encode_sys_status, encode_vfr_hud_raw,
};

// ── VehicleState 便捷封装（内部委托 mavlink-core 的 *_raw） ───────

/// ATTITUDE：姿态欧拉角 + 角速度（rad/s）。标准布局（28B）。
/// 从 [`VehicleState`] 取欧拉角与机体角速度，委托 [`encode_attitude_raw`]。
pub fn encode_attitude(state: &VehicleState, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    encode_attitude_raw(
        state.time_boot_ms,
        state.att.roll(),
        state.att.pitch(),
        state.att.yaw(),
        state.omega[0].0, // rollspeed (p)
        state.omega[1].0, // pitchspeed (q)
        state.omega[2].0, // yawspeed (r)
        seq,
        out,
    )
}

/// LOCAL_POSITION_NED：NED 位置 + 速度。默认用固定 `SYS_ID` 广播。
pub fn encode_local_pos(state: &VehicleState, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    encode_local_pos_from(SYS_ID, state, seq, out)
}

/// 同上，但允许指定 `sys_id`（多机协同：每架飞机用各自 sys_id 广播自身状态）。
pub fn encode_local_pos_from(
    sys_id: u8,
    state: &VehicleState,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    encode_local_pos_from_raw(
        sys_id,
        state.time_boot_ms,
        state.pos[0].0,
        state.pos[1].0,
        state.pos[2].0,
        state.vel[0].0,
        state.vel[1].0,
        state.vel[2].0,
        seq,
        out,
    )
}

/// VFR_HUD：空速/地速/高度/航向/油门（标准 HUD 主盘字段）。标准布局（20B）。
/// 本项目无空速计，airspeed=0；groundspeed 取 NED 速度的平面幅值；alt 用本地高度（-pos.z），
/// heading 由姿态 yaw 导出（厘度 = deg*100）；throttle 由外部控制律写入（0..100）。
pub fn encode_vfr_hud(
    state: &VehicleState,
    throttle_pct: u16,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    let gnd = libm::sqrtf(state.vel[0].0 * state.vel[0].0 + state.vel[1].0 * state.vel[1].0);
    let heading_cdeg = (state.att.yaw_deg() * 100.0) as i16;
    encode_vfr_hud_raw(gnd, heading_cdeg, throttle_pct, -state.pos[2].0, state.vel[2].0, seq, out)
}

/// GLOBAL_POSITION_INT：GPS 全局位置（lat/lon 为 1e7 整数度；无 GPS 时给 0）。
/// 高度用相对高度（-pos.z * 1000 mm），航向由 yaw 导出。
pub fn encode_global_position_int(
    state: &VehicleState,
    seq: u8,
    out: &mut [u8; MAX_FRAME_LEN],
) -> usize {
    encode_global_position_int_raw(
        state.time_boot_ms,
        (-state.pos[2].0 * 1000.0) as i32, // relative alt mm
        (state.vel[0].0 * 100.0) as i16,
        (state.vel[1].0 * 100.0) as i16,
        (state.vel[2].0 * 100.0) as i16,
        state.att.yaw_deg() as u16 * 100, // heading cdeg
        seq,
        out,
    )
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
        assert_eq!(u32::from(out[7]), msg_id::HEARTBEAT);
        // v2 帧长应为 10(头部) + 9(payload) + 2(crc) = 21
        assert_eq!(n, 21);
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
        let frame = frame_from_slice(&out[..21]);
        let (_id, pl) = decode(&frame).unwrap();
        // type=QUADROTOR(2), autopilot=ARDUPILOT(3), base_mode 含 ARM 位
        assert_eq!(pl[0], enums::MAV_TYPE_QUADROTOR);
        assert_eq!(pl[1], enums::MAV_AUTOPILOT_ARDUPILOTMEGA);
        assert!(pl[2] & enums::MAV_MODE_FLAG_SAFETY_ARMED != 0);
        assert!(pl[2] & enums::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED != 0);
        // custom_mode = 5
        assert_eq!(u16::from_le_bytes([pl[3], pl[4]]), 5);
        // system_status = ACTIVE(4) —— 标准布局位于 payload[7]
        assert_eq!(pl[7], enums::MAV_STATE_ACTIVE);
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
        assert_eq!(out[5], 7); // v2 头部位置 [5] = sys_id，覆盖生效
        let frame = frame_from_slice(&out[..n]);
        assert!(decode(&frame).is_some());
    }

    #[test]
    fn attitude_uses_euler_layout() {
        // 标准 ATTITUDE：28B = time_boot_ms i32 + roll/pitch/yaw f32 + 3 rates f32。
        let mut st = crate::vehicle::VehicleState::zero();
        st.time_boot_ms = 1234;
        st.att = crate::vehicle::Quaternion::from_euler(
            crate::units::Radian(0.1),
            crate::units::Radian(0.2),
            crate::units::Radian(0.3),
        );
        st.omega = [
            crate::units::RadianPerSecond(1.0),
            crate::units::RadianPerSecond(2.0),
            crate::units::RadianPerSecond(3.0),
        ];
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_attitude(&st, 0, &mut out);
        assert_eq!(n, 10 + 28 + 2);
        let frame = frame_from_slice(&out[..n]);
        let (_id, pl) = decode(&frame).expect("ATTITUDE should decode (CRC_EXTRA ok)");
        assert_eq!(pl.len(), 28);
        assert_eq!(i32::from_le_bytes([pl[0], pl[1], pl[2], pl[3]]), 1234);
        // roll ~ 0.1 rad（from_euler 构造，应严格等于 roll）
        assert!((f32::from_le_bytes([pl[4], pl[5], pl[6], pl[7]]) - 0.1).abs() < 1e-3);
        // yaw ~ 0.3 rad
        assert!((f32::from_le_bytes([pl[12], pl[13], pl[14], pl[15]]) - 0.3).abs() < 1e-3);
        // yawspeed = 3.0
        assert!((f32::from_le_bytes([pl[24], pl[25], pl[26], pl[27]]) - 3.0).abs() < 1e-3);
    }

    #[test]
    fn vfr_hud_standard_layout() {
        // 标准 VFR_HUD：20B = airspeed f32, groundspeed f32, heading i16(cdeg),
        // throttle u16(%), alt f32, climb f32。
        let mut st = crate::vehicle::VehicleState::zero();
        st.vel = [
            crate::units::MeterPerSecond(3.0),
            crate::units::MeterPerSecond(4.0),
            crate::units::MeterPerSecond(-1.0),
        ];
        st.pos = [crate::units::Meter(0.0), crate::units::Meter(0.0), crate::units::Meter(-50.0)];
        st.att = crate::vehicle::Quaternion::from_euler(
            crate::units::Radian(0.0),
            crate::units::Radian(0.0),
            crate::units::Radian(0.5),
        );
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = encode_vfr_hud(&st, 42, 0, &mut out);
        assert_eq!(n, 10 + 20 + 2);
        let frame = frame_from_slice(&out[..n]);
        let (_id, pl) = decode(&frame).expect("VFR_HUD should decode (CRC_EXTRA ok)");
        assert_eq!(pl.len(), 20);
        // airspeed = 0
        assert_eq!(f32::from_le_bytes([pl[0], pl[1], pl[2], pl[3]]), 0.0);
        // groundspeed = 5.0 (sqrt(3^2+4^2))
        assert!((f32::from_le_bytes([pl[4], pl[5], pl[6], pl[7]]) - 5.0).abs() < 1e-3);
        // heading cdeg = 0.5*180/pi*100 ~ 2865
        let heading = i16::from_le_bytes([pl[8], pl[9]]);
        assert!((heading as f32 - 0.5f32 * 180.0 / core::f32::consts::PI * 100.0).abs() < 2.0);
        // throttle = 42 (%)
        assert_eq!(u16::from_le_bytes([pl[10], pl[11]]), 42);
        // alt = 50.0 (relative = -pos.z)
        assert!((f32::from_le_bytes([pl[12], pl[13], pl[14], pl[15]]) - 50.0).abs() < 1e-3);
        // climb = -1.0
        assert!((f32::from_le_bytes([pl[16], pl[17], pl[18], pl[19]]) - (-1.0)).abs() < 1e-3);
    }
}
