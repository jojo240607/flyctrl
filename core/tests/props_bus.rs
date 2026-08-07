//! M10 消息总线属性测试（host 端）。
//!
//! 用确定性 LCG 随机化穷举，验证四类性质：
//! - 环形缓冲 FIFO：随机压入/弹出序列始终保序。
//! - 满拒绝：容量达到上限后 `try_push` 返回 `Err`（不覆盖、不静默丢）。
//! - 多主题独立：各主题通道互不串扰。
//! - 扇出无丢失：est 扇出到两个消费者，元素数与顺序一致。
//!
//! 零外部依赖，自实现小型 LCG。

use flyctrl_core::bus::{Bus, Ring};
use flyctrl_core::controller::trait_def::Setpoint;
use flyctrl_core::fdir::Health;
use flyctrl_core::flightmode::FlightMode;
use flyctrl_core::units::*;
use flyctrl_core::vehicle::{ImuSample, VehicleState};

struct Lcg(u64);
impl Lcg {
    fn new(seed: u64) -> Self {
        Lcg(seed.max(1))
    }
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.0
    }
    fn f(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32
    }
    fn u(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

#[test]
fn prop_ring_fifo_under_random_ops() {
    let mut rng = Lcg::new(0x6010_9a17);
    for _ in 0..500 {
        let mut ring: Ring<u32, 8> = Ring::new();
        // 引用队列以"真实环容量"为界：仅当环未满才同步 push。
        let mut ref_q: std::collections::VecDeque<u32> = std::collections::VecDeque::new();
        for _ in 0..200 {
            if rng.u(2) == 0 || ref_q.is_empty() {
                // push：当且仅当环未满时，环与引用同步入队（满则两者都不入）。
                let v = rng.next() as u32;
                if !ring.is_full() {
                    assert!(ring.try_push(v).is_ok());
                    ref_q.push_back(v);
                } else {
                    // 满：环应拒绝，且引用也不入队
                    assert_eq!(ring.try_push(v), Err(v));
                }
            } else {
                // pop：应与 ref 队首一致
                let got = ring.try_pop();
                let exp = ref_q.pop_front();
                assert_eq!(got, exp);
            }
        }
        // 排空后应完全一致
        while let Some(exp) = ref_q.pop_front() {
            assert_eq!(ring.try_pop(), Some(exp));
        }
        assert!(ring.is_empty());
    }
}

#[test]
fn prop_ring_full_rejects() {
    let mut rng = Lcg::new(0xbeef_0123);
    for _ in 0..1000 {
        let v = rng.next() as u32;
        let mut ring: Ring<u32, 4> = Ring::new();
        for i in 0..4 {
            assert!(ring.try_push(i).is_ok());
        }
        // 满后任何 push 都被拒，且返回值就是原值（未覆盖）
        assert_eq!(ring.try_push(v), Err(v));
        // 已有数据不变：FIFO 头部仍为 0
        assert_eq!(ring.try_pop(), Some(0));
    }
}

#[test]
fn prop_bus_topics_independent() {
    let mut rng = Lcg::new(0x0ade_5eed);
    for _ in 0..2000 {
        let mut bus = Bus::new();
        let n = 1 + (rng.next() % 6) as usize;
        let mut imu_count = 0u32;
        let mut sp_count = 0u32;
        let mut mode_count = 0u32;
        for _ in 0..n {
            match rng.u(3) {
                0 => { if bus.publish_imu(ImuSample { accel: [MeterPerSecondSquared(rng.f()), MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0)], gyro: [RadianPerSecond(0.0); 3] }).is_ok() { imu_count += 1; } }
                1 => { if bus.publish_setpoint(Setpoint::hover([Meter(0.0); 3], Radian(0.0))).is_ok() { sp_count += 1; } }
                _ => { if bus.publish_mode((FlightMode::Position, Health::Nominal)).is_ok() { mode_count += 1; } }
            }
        }
        // 泵若干次（每主题每次搬运 1 个）。
        let mut guard = 0;
        while bus.pump() > 0 && guard < 16 { guard += 1; }
        // 各自消费数量必须守恒（无串扰、无丢）。
        let mut rcv_imu = 0;
        while bus.recv_imu().is_some() { rcv_imu += 1; }
        let mut rcv_sp = 0;
        while bus.recv_setpoint().is_some() { rcv_sp += 1; }
        let mut rcv_mode = 0;
        while bus.recv_mode().is_some() { rcv_mode += 1; }
        assert_eq!(rcv_imu, imu_count, "imu 收/发数量应守恒");
        assert_eq!(rcv_sp, sp_count, "setpoint 收/发数量应守恒");
        assert_eq!(rcv_mode, mode_count, "mode 收/发数量应守恒");
    }
}

#[test]
fn prop_bus_est_fanout_no_loss() {
    let mut rng = Lcg::new(0xfeed_10c5);
    for _ in 0..1000 {
        let mut bus = Bus::new();
        let n = 1 + (rng.next() % 4) as usize; // 1..4（不超过扇出消费者段容量 4，保证无背压丢失）
        // 发布 n 个 est，每个带唯一计数器于 pos[0]；每发一次泵一下，避免 est_in(cap=4) 溢出。
        for k in 0..n {
            let mut st = VehicleState::zero();
            st.pos = [Meter(k as f32), Meter(0.0), Meter(-10.0)];
            assert!(bus.publish_est(st).is_ok());
            bus.pump(); // 把 est_in 搬运到两个扇出消费者段
        }
        // 排空 est_in 残留（若 n 较小、pump 已清空则无影响）
        let mut guard = 0;
        while bus.pump() > 0 && guard < 16 { guard += 1; }
        // 两个消费者各应收到 n 个，顺序一致
        let mut seq_ctrl = Vec::new();
        while let Some(st) = bus.recv_est_ctrl() { seq_ctrl.push(st.pos[0].0); }
        let mut seq_fdir = Vec::new();
        while let Some(st) = bus.recv_est_fdir() { seq_fdir.push(st.pos[0].0); }
        assert_eq!(seq_ctrl.len(), n);
        assert_eq!(seq_fdir.len(), n);
        for k in 0..n {
            assert_eq!(seq_ctrl[k], k as f32);
            assert_eq!(seq_fdir[k], k as f32);
        }
    }
}
