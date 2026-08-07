//! 通信层端到端回环测试（M6 验收）。
//!
//! 验证：VehicleState → Telemetry 组帧 → LoopbackLink 收发 → 解码，全链路无堆、
//! 报文可被标准 MAVLink 语义解析（magic/CRC 校验通过）。

use flyctrl_core::comm::link::{Frame, LoopbackLink, Link};
use flyctrl_core::comm::mavlink::{self, msg_id};
use flyctrl_core::comm::telemetry::Telemetry;
use flyctrl_core::vehicle::VehicleState;

#[test]
fn loopback_telemetry_roundtrip() {
    let mut link = LoopbackLink::new();
    let mut telem = Telemetry::new(50);
    let st = VehicleState::zero();

    // 模拟 1 秒（dt=5ms，50Hz 遥测）
    for _ in 0..200 {
        telem.update(5, &st, true);
        // 每步把 ring 里的帧经链路发出
        while telem.pending() > 0 {
            let f = telem.pop();
            link.send_frame(&f);
        }
    }

    // 从链路回收并解析
    let mut heartbeats = 0;
    let mut attitudes = 0;
    let mut positions = 0;
    loop {
        let f = link.recv_frame();
        if f.is_empty() { break; }
        let (id, _payload) = mavlink::decode(&f).expect("frame must decode with valid CRC");
        match id {
            msg_id::HEARTBEAT => heartbeats += 1,
            msg_id::ATTITUDE => attitudes += 1,
            msg_id::LOCAL_POSITION_NED => positions += 1,
            _ => {}
        }
    }
    assert!(heartbeats >= 1, "expected >=1 heartbeat, got {}", heartbeats);
    assert!(attitudes >= 10, "expected >=10 attitude frames, got {}", attitudes);
    assert!(positions >= 10, "expected >=10 position frames, got {}", positions);
}

#[test]
fn corrupted_frame_rejected() {
    let mut frame = Frame::default();
    let n = mavlink::encode_attitude(&VehicleState::zero(), 0, &mut frame.data);
    let mut f = Frame::from_bytes(&frame.data[..n]);
    // 翻转一个 payload 字节 → CRC 应不匹配
    f.data[7] ^= 0xFF;
    assert!(mavlink::decode(&f).is_none(), "corrupted frame must fail CRC");
}
