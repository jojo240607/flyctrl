//! 物理基础健全性测试（需要 std，放 tests/ 下）。
use flyctrl_sim::physics::{Physics, PhysicsParams};
use flyctrl_core::units::*;
use flyctrl_core::vehicle::{ActuatorCmd, VehicleState, Quaternion};

/// 近似 roll/pitch（Z-Y-X 小角），仅用于测试符号核对。
fn approx_rp(q: Quaternion) -> (f32, f32) {
    let roll = (2.0 * (q.w * q.x + q.y * q.z)).atan2(1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    let pitch = (2.0 * (q.w * q.y - q.z * q.x)).asin();
    (roll, pitch)
}

fn run(mut phys: Physics, cmd: ActuatorCmd, secs: f32) -> VehicleState {
    let dt = Second(0.005);
    let n = (secs / dt.0) as u32;
    let mut s = phys.state();
    for _ in 0..n {
        let _ = phys.step(dt, cmd);
        s = phys.state();
    }
    s
}

#[test]
fn hover_command_stays_level() {
    let phys = Physics::new(PhysicsParams::default());
    // 悬停油门：4 电机各 0.5（应近似平衡重力，姿态保持水平）
    let s = run(phys, ActuatorCmd { motor: [0.5; 4] }, 3.0);
    println!("hover att.w={:.3} x={:.3} y={:.3} z={:.3} alt={:.2}",
        s.att.w, s.att.x, s.att.y, s.att.z, -s.pos[2].0);
    assert!(s.att.w > 0.95, "hover attitude drifted: w={}", s.att.w);
}

#[test]
fn front_right_high_rolls_pitches_right() {
    // 前右电机(0)更高 -> 应产生右滚(+roll about X) 与上仰(+pitch about Y)
    // 用小不对称 + 短时间，避免姿态翻转导致 approx_rp 符号缠绕。
    let phys = Physics::new(PhysicsParams::default());
    let cmd = ActuatorCmd { motor: [0.55, 0.45, 0.45, 0.45] };
    let s = run(phys, cmd, 0.1);
    let (roll, pitch) = approx_rp(s.att);
    println!("asym att.w={:.3} roll={:.3} pitch={:.3}", s.att.w, roll, pitch);
    // 期望 roll>0（右滚）且 pitch>0（上仰）
    assert!(roll > 0.01, "expected right roll, got {}", roll);
    assert!(pitch > 0.01, "expected nose-up pitch, got {}", pitch);
}
