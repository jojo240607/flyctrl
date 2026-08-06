//! SITL 入口：在 host 上跑通 PID + 互补滤波的闭环仿真，并打印轨迹与指标。
//!
//! 这一步的目标是验证"核心控制逻辑 <-> 物理仿真"解耦是否干净：
//! 控制器不关心物理，仿真不关心算法。后续加入 LQR/EKF 时只需替换 trait 实现。

use flyctrl_core::controller::{Controller, PidController, Setpoint};
use flyctrl_core::estimator::{ComplementaryEstimator, Estimator};
use flyctrl_core::state::{Fcs, Disarmed, Armed};
use flyctrl_core::units::*;
use flyctrl_sim::harness::{Harness, Metrics};
use flyctrl_sim::physics::{Physics, PhysicsParams};
use flyctrl_sim::world::{World, WorldParams};

fn main() {
    println!("=== flyctrl SITL: PID + ComplementaryFilter baseline ===");

    // --- 飞控状态机：类型级保证"先校准/解锁才能飞" ---
    let fcs: Fcs<Disarmed> = Fcs::new();
    let fcs: Fcs<Armed> = fcs.arm(); // 仿真中直接解锁（已隐含"校准完成"前置）
    println!("[fcs] armed, type-state guarantees calibration-before-arm");

    // --- 物理参数：典型 450mm 四旋翼 ---
    let physics = Physics::new(PhysicsParams::default());
    // --- 世界：带传感器噪声，无风（先干净基线） ---
    let world = World::new(WorldParams::default());

    // --- 算法组合（基线） ---
    let est = ComplementaryEstimator::new(0.98, 0.1);
    let ctrl = PidController::default_quad();

    let dt = Second(0.005); // 200 Hz 控制环
    let mut harness = Harness::new(physics, world, est, ctrl, dt);

    // 目标：悬停在 (0,0,-10) NED（即离地 10m 高度）
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));

    println!("[sim] running 8s @ 200Hz, target hover at z=-10m ...");
    // 临时：每秒打印一次状态用于调试
    for sec in 0..8 {
        let _ = harness.run(Second(1.0), &sp);
        let s = harness.state();
        println!("  t={}s pos=({:.2},{:.2},{:.2}) alt={:.2}",
            sec+1, s.pos[0].0, s.pos[1].0, s.pos[2].0, -s.pos[2].0);
    }
    let m = harness.run(Second(0.0), &sp);
    report("PID+Complementary", m);

    // 打印最终状态
    let s = harness.state();
    println!(
        "[final] pos=({:.2},{:.2},{:.2})m  alt={:.2}m  att.w={:.3}",
        s.pos[0].0, s.pos[1].0, s.pos[2].0, -s.pos[2].0, s.att.w
    );
    let _ = fcs;
}

fn report(name: &str, m: Metrics) {
    println!("--- {name} ---");
    println!("  steps        : {}", m.steps);
    println!("  pos_rms (m)  : {:.4}", m.pos_rms);
    println!("  pos_max (m)  : {:.4}", m.pos_max);
    println!("  settle (s)   : {}", if m.settle_time < 0.0 { "N/A".into() } else { format!("{:.2}", m.settle_time) });
    println!("  worst_step(ms): {:.4}", m.worst_step_ms);
    println!("  nan/diverge  : {}", m.nan_detected);
}
