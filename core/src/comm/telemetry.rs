//! 内部遥测 / 日志通道（M6.3）。
//!
//! [`Telemetry`] 把 [`VehicleState`] 周期序列化为 MAVLink 帧，写入一个固定容量
//! ring buffer（不阻塞控制回路）。地面站循环从 ring buffer 取出并下发。
//!
//! 设计要点：
//! - 序列化（组帧）在 `push` 时完成一次，ring 存的是已组好的字节帧；
//! - ring 满则丢弃最旧（遥测允许丢，不可阻塞控制）；
//! - 全部固定数组，无堆。

use crate::comm::link::{Frame, MAX_FRAME_LEN};
use crate::comm::mavlink;
use crate::vehicle::VehicleState;

/// ring buffer 容量（帧数）。
pub const TELEM_RING_FRAMES: usize = 32;

/// 遥测通道：状态 → MAVLink 帧 → ring。
pub struct Telemetry {
    ring: [Frame; TELEM_RING_FRAMES],
    head: usize,
    tail: usize,
    count: usize,
    seq: u8,
    rate_hz: u32,
    tick: u32,
    drop_count: u32,
}

impl Telemetry {
    pub fn new(rate_hz: u32) -> Self {
        Self {
            ring: [Frame::default(); TELEM_RING_FRAMES],
            head: 0,
            tail: 0,
            count: 0,
            seq: 0,
            rate_hz,
            tick: 0,
            drop_count: 0,
        }
    }

    /// 每个控制步调用：按 `rate_hz` 节流，把当前状态组帧入 ring。
    /// `sensors_ok` 来自 FDIR（决定 SYS_STATUS 健康位）。
    pub fn update(&mut self, dt_ms: u32, state: &VehicleState, sensors_ok: bool) {
        self.tick += dt_ms;
        let interval = 1000u32 / self.rate_hz.max(1);
        if self.tick < interval { return; }
        self.tick = 0;

        let mut buf = [0u8; MAX_FRAME_LEN];
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);

        // 一个遥测周期发多帧（ATTITUDE + LOCAL_POS + 周期性 HEARTBEAT/SYS_STATUS）
        let n1 = mavlink::encode_attitude(state, seq, &mut buf);
        self.push(&buf, n1);
        let n2 = mavlink::encode_local_pos(state, seq, &mut buf);
        self.push(&buf, n2);
        // 每 10 帧补一个 HEARTBEAT + SYS_STATUS（约 5Hz @ 50Hz 遥测）
        if seq % 10 == 0 {
            let n3 = mavlink::encode_heartbeat(0, true, seq, &mut buf);
            self.push(&buf, n3);
            let n4 = mavlink::encode_sys_status(sensors_ok, seq, &mut buf);
            self.push(&buf, n4);
        }
    }

    fn push(&mut self, buf: &[u8], len: usize) {
        if len == 0 { return; }
        let frame = Frame::from_bytes(&buf[..len]);
        self.ring[self.head] = frame;
        self.head = (self.head + 1) % TELEM_RING_FRAMES;
        if self.count < TELEM_RING_FRAMES {
            self.count += 1;
        } else {
            self.tail = (self.tail + 1) % TELEM_RING_FRAMES;
            self.drop_count += 1;
        }
    }

    /// 取出下一帧下发（无则空帧）。消费式（取出即从 ring 移除）。
    pub fn pop(&mut self) -> Frame {
        if self.count == 0 { return Frame::default(); }
        let f = self.ring[self.tail];
        self.tail = (self.tail + 1) % TELEM_RING_FRAMES;
        self.count -= 1;
        f
    }

    pub fn pending(&self) -> usize { self.count }
    pub fn dropped(&self) -> u32 { self.drop_count }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telemetry_produces_parseable_frames() {
        let mut t = Telemetry::new(50);
        let st = VehicleState::zero();
        // 跑 600ms（足够触发多次 ATTITUDE/LOCAL_POS 及至少一个 HEARTBEAT）
        for _ in 0..120 {
            t.update(5, &st, true);
        }
        let mut seen = 0;
        while t.pending() > 0 {
            let f = t.pop();
            assert!(!f.is_empty());
            let (_id, _p) = mavlink::decode(&f).expect("frame should decode");
            seen += 1;
        }
        assert!(seen >= 8, "expected several telemetry frames, got {}", seen);
    }
}
