//! MAVLink 命令桥接层（M6.2）。
//!
//! 把链路上的 MAVLink 帧（命令 / 参数请求）翻译成仿真可消费的高层语义事件
//! （[`MavCommand`]），并把遥测帧（由 [`Telemetry`] 组好的字节帧）经 [`Link`] 下发；
//! 对收到的命令自动回 `COMMAND_ACK`。参数表经 [`ParamProvider`] 抽象，支持
//! 一次 `PARAM_REQUEST_LIST` 流式回传全部 `PARAM_VALUE`。
//!
//! 全部 `no_std`、无堆、有界耗时。

use crate::comm::link::{Frame, Link, MAX_FRAME_LEN};
use crate::comm::mavlink;
use crate::comm::telemetry::Telemetry;

/// 由 COMMAND_LONG 翻译出的高层命令事件。
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MavCommand {
    /// 解锁（起飞前必须）。
    Arm,
    /// 上锁（立即停转）。
    Disarm,
    /// 设置飞行模式（MAV custom mode 低字节）。
    SetMode(u8),
    /// 起飞到指定绝对高度（m）。
    Takeoff(f32),
    /// 降落（原地垂直着陆）。
    Land,
    /// 返航（RTL）。
    Rtl,
    /// 启动任务（自动航线）。
    StartMission,
    /// 地面站请求参数表（应回 PARAM_VALUE 流）。
    RequestParamList,
    /// 其它未识别命令（保留原始命令号）。
    Other(u16),
}

impl MavCommand {
    /// 该命令是否可映射到仿真动作（用于决定 COMMAND_ACK 的 result）。
    pub fn is_supported(&self) -> bool {
        !matches!(self, MavCommand::Other(_))
    }
}

/// 参数表提供者：把飞行控制器内部可调参数暴露给地面站。
pub trait ParamProvider {
    /// 参数总数。
    fn param_count(&self) -> u16;
    /// 第 `idx` 个参数的 ID 字符串（16 字节，C 风格，不足补 0）。
    fn param_id(&self, idx: u16) -> [u8; 16];
    /// 第 `idx` 个参数的浮点值。
    fn param_value(&self, idx: u16) -> f32;
    /// 第 `idx` 个参数的类型（默认 MAV_PARAM_TYPE_REAL32=9）。
    fn param_type(&self, _idx: u16) -> u8 { 9 }
}

/// MAVLink 桥接器：持有一条链路 + 帧序号。
pub struct MavlinkBridge<L: Link> {
    link: L,
    seq: u8,
}

impl<L: Link> MavlinkBridge<L> {
    pub fn new(link: L) -> Self {
        Self { link, seq: 0 }
    }

    /// 取出链路（如要替换/查询健康）。
    pub fn link(&self) -> &L { &self.link }
    pub fn link_mut(&mut self) -> &mut L { &mut self.link }

    /// 从链路收一帧原始字节（测试/高级用途：绕过命令解析直接看链路内容）。
    pub fn recv_raw(&mut self) -> Frame { self.link.recv_frame() }

    fn next_seq(&mut self) -> u8 {
        let s = self.seq;
        self.seq = self.seq.wrapping_add(1);
        s
    }

    /// 把一帧字节经链路发出。
    fn send_raw(&mut self, data: &[u8], len: usize) {
        if len == 0 { return; }
        let frame = Frame::from_bytes(&data[..len.min(MAX_FRAME_LEN)]);
        self.link.send_frame(&frame);
    }

    /// 周期性下发遥测：把 `Telemetry` ring 里的所有帧清空并经链路发出。
    /// 不阻塞控制回路（ring 本身已是异步缓冲）。返回实际下发的帧数。
    pub fn drain_telemetry(&mut self, tel: &mut Telemetry) -> usize {
        let mut n = 0usize;
        while tel.pending() > 0 {
            let f = tel.pop();
            self.link.send_frame(&f);
            n += 1;
        }
        n
    }

    /// 处理一帧入站报文（若有）。
    ///
    /// - COMMAND_LONG：翻译成 [`MavCommand`] 并自动回 COMMAND_ACK（supported→ACCEPTED，
    ///   否则 UNSUPPORTED）；返回事件。
    /// - PARAM_REQUEST_LIST：经 `params` 流式回传全部 PARAM_VALUE，返回
    ///   [`MavCommand::RequestParamList`]。
    /// - 其它：忽略，返回 `None`。
    pub fn handle_frame(&mut self, frame: &Frame, params: &dyn ParamProvider) -> Option<MavCommand> {
        if frame.is_empty() { return None; }
        let (msgid, payload) = mavlink::decode(frame)?;
        match msgid {
            mavlink::msg_id::COMMAND_LONG => {
                let cl = mavlink::decode_command_long(payload)?;
                let cmd = match cl.command {
                    mavlink::enums::MAV_CMD_COMPONENT_ARM_DISARM => {
                        if cl.params[0] > 0.5 { MavCommand::Arm } else { MavCommand::Disarm }
                    }
                    mavlink::enums::MAV_CMD_NAV_TAKEOFF => MavCommand::Takeoff(cl.params[6]),
                    mavlink::enums::MAV_CMD_NAV_LAND => MavCommand::Land,
                    mavlink::enums::MAV_CMD_NAV_RETURN_TO_LAUNCH => MavCommand::Rtl,
                    mavlink::enums::MAV_CMD_MISSION_START => MavCommand::StartMission,
                    mavlink::enums::MAV_CMD_DO_SET_MODE => MavCommand::SetMode(cl.params[0] as u8),
                    _ => MavCommand::Other(cl.command),
                };
                let result = if cmd.is_supported() {
                    mavlink::enums::MAV_RESULT_ACCEPTED
                } else {
                    mavlink::enums::MAV_RESULT_UNSUPPORTED
                };
                self.send_command_ack(cl.command, result);
                Some(cmd)
            }
            mavlink::msg_id::PARAM_REQUEST_LIST => {
                self.stream_params(params);
                Some(MavCommand::RequestParamList)
            }
            _ => None,
        }
    }

    /// 从链路收一帧并尝试处理；无数据返回 None。
    pub fn poll(&mut self, params: &dyn ParamProvider) -> Option<MavCommand> {
        let f = self.link.recv_frame();
        if f.is_empty() { return None; }
        self.handle_frame(&f, params)
    }

    /// 回 COMMAND_ACK。
    fn send_command_ack(&mut self, cmd: u16, result: u8) {
        let mut buf = [0u8; MAX_FRAME_LEN];
        let seq = self.next_seq();
        let n = mavlink::encode_command_ack(cmd, result, 0, 0, seq, &mut buf);
        self.send_raw(&buf, n);
    }

    /// 流式回传参数表（PARAM_REQUEST_LIST 的应答）。
    pub fn stream_params(&mut self, params: &dyn ParamProvider) {
        let count = params.param_count();
        let mut buf = [0u8; MAX_FRAME_LEN];
        for idx in 0..count {
            let seq = self.next_seq();
            let n = mavlink::encode_param_value(
                &params.param_id(idx),
                params.param_value(idx),
                params.param_type(idx),
                count,
                idx,
                seq,
                &mut buf,
            );
            self.send_raw(&buf, n);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::comm::link::LoopbackLink;
    use crate::comm::mavlink;
    use crate::vehicle::VehicleState;

    struct EmptyParams;
    impl ParamProvider for EmptyParams {
        fn param_count(&self) -> u16 { 0 }
        fn param_id(&self, _: u16) -> [u8; 16] { [0u8; 16] }
        fn param_value(&self, _: u16) -> f32 { 0.0 }
    }

    fn build_command_long(cmd: u16, params: [f32; 7]) -> Frame {
        let mut buf = [0u8; MAX_FRAME_LEN];
        let n = mavlink::encode_command_long(cmd, params[0], params[1], params[2], params[3], params[4], params[5], params[6], 0, 0, &mut buf);
        Frame::from_bytes(&buf[..n])
    }

    #[test]
    fn arm_disarm_decode_and_ack() {
        let mut link = LoopbackLink::new();
        let mut bridge = MavlinkBridge::new(link);

        // 解锁
        let f = build_command_long(mavlink::enums::MAV_CMD_COMPONENT_ARM_DISARM, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let cmd = bridge.handle_frame(&f, &EmptyParams);
        assert_eq!(cmd, Some(MavCommand::Arm));

        // 上锁
        let f2 = build_command_long(mavlink::enums::MAV_CMD_COMPONENT_ARM_DISARM, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let cmd2 = bridge.handle_frame(&f2, &EmptyParams);
        assert_eq!(cmd2, Some(MavCommand::Disarm));

        // 链路里应已有两条 COMMAND_ACK（msgid=77）。
        let ack1 = bridge.recv_raw();
        assert!(!ack1.is_empty());
        let (id1, payload1) = mavlink::decode(&ack1).unwrap();
        assert_eq!(id1, mavlink::msg_id::COMMAND_ACK);
        let (ack_cmd, ack_res) = mavlink::decode_command_ack(payload1).unwrap();
        assert_eq!(ack_cmd, mavlink::enums::MAV_CMD_COMPONENT_ARM_DISARM);
        assert_eq!(ack_res, mavlink::enums::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn takeoff_land_rtl_modes() {
        let mut link = LoopbackLink::new();
        let mut bridge = MavlinkBridge::new(link);

        let t = build_command_long(mavlink::enums::MAV_CMD_NAV_TAKEOFF, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 12.5]);
        assert_eq!(bridge.handle_frame(&t, &EmptyParams), Some(MavCommand::Takeoff(12.5)));

        let l = build_command_long(mavlink::enums::MAV_CMD_NAV_LAND, [0.0; 7]);
        assert_eq!(bridge.handle_frame(&l, &EmptyParams), Some(MavCommand::Land));

        let r = build_command_long(mavlink::enums::MAV_CMD_NAV_RETURN_TO_LAUNCH, [0.0; 7]);
        assert_eq!(bridge.handle_frame(&r, &EmptyParams), Some(MavCommand::Rtl));

        let m = build_command_long(mavlink::enums::MAV_CMD_DO_SET_MODE, [4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        assert_eq!(bridge.handle_frame(&m, &EmptyParams), Some(MavCommand::SetMode(4)));

        // 未知命令 -> Other + UNSUPPORTED ack。缓冲区里已排着前面若干命令的 ACK，
        // 全部读出并定位命令 4242 的那条确认其 result=UNSUPPORTED(3)。
        let o = build_command_long(4242, [0.0; 7]);
        assert_eq!(bridge.handle_frame(&o, &EmptyParams), Some(MavCommand::Other(4242)));
        let mut found = false;
        while {
            let ack = bridge.recv_raw();
            if ack.is_empty() { false } else {
                let (_, payload) = mavlink::decode(&ack).unwrap();
                let (ack_cmd, res) = mavlink::decode_command_ack(payload).unwrap();
                if ack_cmd == 4242 { assert_eq!(res, mavlink::enums::MAV_RESULT_UNSUPPORTED); found = true; }
                true
            }
        } {}
        assert!(found);
    }

    #[test]
    fn param_request_list_streams_values() {
        struct P;
        impl ParamProvider for P {
            fn param_count(&self) -> u16 { 2 }
            fn param_id(&self, idx: u16) -> [u8; 16] {
                let mut id = [0u8; 16];
                let s: &[u8] = if idx == 0 { b"THR_MUL\0\0\0\0\0" } else { b"ATT_P\0\0\0\0\0\0\0\0" };
                id[..s.len()].copy_from_slice(s);
                id
            }
            fn param_value(&self, idx: u16) -> f32 { if idx == 0 { 1.5 } else { 0.25 } }
        }
        let mut link = LoopbackLink::new();
        let mut bridge = MavlinkBridge::new(link);

        let mut buf = [0u8; MAX_FRAME_LEN];
        let n = mavlink::encode_param_request_list(1, 1, 0, 0, &mut buf);
        let req = Frame::from_bytes(&buf[..n]);
        assert_eq!(bridge.handle_frame(&req, &P), Some(MavCommand::RequestParamList));

        // 应收到 2 条 PARAM_VALUE
        let pv1 = bridge.recv_raw();
        assert!(!pv1.is_empty());
        let (id1, payload1) = mavlink::decode(&pv1).unwrap();
        assert_eq!(id1, mavlink::msg_id::PARAM_VALUE);
        let _pv = mavlink::decode_param_value(payload1).unwrap();
        let pv2 = bridge.recv_raw();
        assert!(!pv2.is_empty());
        let (id2, _) = mavlink::decode(&pv2).unwrap();
        assert_eq!(id2, mavlink::msg_id::PARAM_VALUE);
    }

    #[test]
    fn telemetry_drain_sends_frames() {
        let mut link = LoopbackLink::new();
        let mut bridge = MavlinkBridge::new(link);
        let mut tel = Telemetry::new(50);
        let st = VehicleState::zero();
        // 控制帧数使其字节总量 (< 1024) 能完整容纳于 LoopbackLink 缓冲，避免帧被截断。
        for _ in 0..15 { tel.update(5, &st, true); }

        let n = bridge.drain_telemetry(&mut tel);
        assert!(n >= 8);
        // 链路里应可逐帧解回
        let mut seen = 0;
        while {
            let f = bridge.recv_raw();
            if f.is_empty() { false } else { let _ = mavlink::decode(&f); seen += 1; true }
        } {}
        assert_eq!(seen, n);
    }
}
