//! SITL 主机端：把 `flyctrl-core` 的仿真后端（Physics + World）与
//! 估计器/控制器组合驱动起来，打印统一收敛指标。
//!
//! 用法：
//!   cargo run -p flyctrl-sitl
//!   cargo run -p flyctrl-sitl -- --seconds 12
//!   cargo run -p flyctrl-sitl -- --est all --ctrl all
//!   cargo run -p flyctrl-sitl -- --est ekf --ctrl lqr
//!
//! 默认对所有 (估计器 × 控制器) 组合跑一遍并输出对比表。

use flyctrl_core::controller::lqr::LqrController;
use flyctrl_core::controller::pid::PidController;
use flyctrl_core::controller::trait_def::{ActuatorCmd, Controller};
use flyctrl_core::estimator::complementary::ComplementaryEstimator;
use flyctrl_core::estimator::ekf::EkfEstimator;
use flyctrl_core::estimator::trait_def::Estimator;
use flyctrl_core::vehicle::{Second, VehicleState};
use flyctrl_sim::harness::{Physics, PhysicsParams, World, WorldParams};
use flyctrl_sim::scenario::{Scenario, ScenarioKind};

#[derive(Clone, Copy, PartialEq)]
enum EstKind {
    Complementary,
    Ekf,
}
#[derive(Clone, Copy, PartialEq)]
enum CtrlKind {
    Pid,
    Lqr,
}

fn est_name(e: EstKind) -> &'static str {
    match e {
        EstKind::Complementary => "Complementary",
        EstKind::Ekf => "EKF",
    }
}
fn ctrl_name(c: CtrlKind) -> &'static str {
    match c {
        CtrlKind::Pid => "PID",
        CtrlKind::Lqr => "LQR",
    }
}

struct Metrics {
    pos_rms: f32,
    settle_time: f32,
    diverge: bool,
    final_alt: f32,
    final_att_w: f32,
    ctrl_effort: f32,
}

/// 跑一个 (估计器×控制器) 组合，返回收敛指标。
/// 目标随 `scenario` 变化（定点/阶跃/轨迹/风扰）。
fn run_combo(est_kind: EstKind, ctrl_kind: CtrlKind, scenario: &Scenario, seconds: f32) -> Metrics {
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());

    // 应用场景风扰（仅 Wind 场景生效，其余为 0）
    let wp = scenario.world_params();
    phys.set_wind(wp.wind);
    phys.set_wind_gust(wp.wind_gust);

    let mut comp = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let mut ekf = EkfEstimator::default_quad();
    let mut pid = PidController::default_quad();
    let mut lqr = LqrController::default_quad();

    let dt = Second(0.005);
    let mut cmd = ActuatorCmd { motor: [0.5; 4] };
    let steps = (seconds / dt.0) as usize;

    let mut sum_sq = 0.0f32;
    let mut settle = seconds;
    let mut settled = false;
    let mut diverge = false;
    let mut ctrl_eff = 0.0f32;
    let mut final_att_w = 1.0f32;
    let mut final_alt = 0.0f32;

    for k in 0..steps {
        let t = k as f32 * dt.0;
        let sp = scenario.setpoint_at(Second(t));

        let ideal = phys.step(dt, cmd);
        let s = phys.state();
        let (imu, gps) = world.sense(dt, ideal, s.pos);
        let est: VehicleState = match est_kind {
            EstKind::Complementary => comp.step(dt, imu, gps),
            EstKind::Ekf => ekf.step(dt, imu, gps),
        };
        cmd = match ctrl_kind {
            CtrlKind::Pid => pid.control(dt, &sp, &est),
            CtrlKind::Lqr => lqr.control(dt, &sp, &est),
        };

        // 轨迹误差 = 估计位置与当前目标之差
        let tg = [sp.pos[0].0, sp.pos[1].0, sp.pos[2].0];
        let pe = [
            est.pos[0].0 - tg[0],
            est.pos[1].0 - tg[1],
            est.pos[2].0 - tg[2],
        ];
        let e = (pe[0] * pe[0] + pe[1] * pe[1] + pe[2] * pe[2]).sqrt();
        sum_sq += e * e;

        // 稳定判定：Step 场景仅在阶跃之后计
        let settle_ok = scenario.kind() != ScenarioKind::Step || t >= scenario.step_time();
        if !settled {
            if settle_ok && e < 0.5 {
                settled = true;
                settle = t;
            }
        } else if e > 1.0 {
            settled = false;
        }

        let msum: f32 = cmd.motor.iter().sum();
        ctrl_eff += msum / 4.0;

        if est.att.w < 0.0 || e > 20.0 {
            diverge = true;
        }
        final_att_w = s.att.w; // 物理真值姿态：<0 表示翻转
        final_alt = s.pos[2].0;
    }

    Metrics {
        pos_rms: (sum_sq / steps as f32).sqrt(),
        settle_time: if diverge { seconds } else { settle },
        diverge,
        final_alt,
        final_att_w,
        ctrl_effort: ctrl_eff / steps as f32,
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut seconds = 12.0f32;
    let mut est_filter = "all".to_string();
    let mut ctrl_filter = "all".to_string();
    let mut scenario_filter = "hover".to_string();

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--seconds" => {
                if i + 1 < args.len() {
                    seconds = args[i + 1].parse().unwrap_or(12.0);
                    i += 2;
                } else {
                    i += 1;
                }
            }
            "--est" => {
                if i + 1 < args.len() {
                    est_filter = args[i + 1].clone();
                    i += 2;
                } else {
                    i += 1;
                }
            }
            "--ctrl" => {
                if i + 1 < args.len() {
                    ctrl_filter = args[i + 1].clone();
                    i += 2;
                } else {
                    i += 1;
                }
            }
            "--scenario" => {
                if i + 1 < args.len() {
                    scenario_filter = args[i + 1].clone();
                    i += 2;
                } else {
                    i += 1;
                }
            }
            _ => i += 1,
        }
    }

    let ests = if est_filter == "all" {
        vec![EstKind::Complementary, EstKind::Ekf]
    } else if est_filter == "comp" || est_filter == "complementary" {
        vec![EstKind::Complementary]
    } else if est_filter == "ekf" {
        vec![EstKind::Ekf]
    } else {
        vec![EstKind::Complementary, EstKind::Ekf]
    };

    let ctrls = if ctrl_filter == "all" {
        vec![CtrlKind::Pid, CtrlKind::Lqr]
    } else if ctrl_filter == "pid" {
        vec![CtrlKind::Pid]
    } else if ctrl_filter == "lqr" {
        vec![CtrlKind::Lqr]
    } else {
        vec![CtrlKind::Pid, CtrlKind::Lqr]
    };

    let scenario = Scenario::new(ScenarioKind::parse(&scenario_filter));

    println!(
        "flyctrl SITL — 算法对比表 (scenario={}, T={}s, dt=5ms)",
        scenario.name(),
        seconds
    );
    println!(
        "{:<16}{:<8}{:>9}{:>9}{:>10}{:>9}{:>9}",
        "estimator", "ctrl", "posRMS", "settle", "finalAlt", "attW", "ctrlEff"
    );
    println!("{}", "-".repeat(68));

    for &e in &ests {
        for &c in &ctrls {
            let m = run_combo(e, c, &scenario, seconds);
            println!(
                "{:<16}{:<8}{:>9.3}{:>9.2}{:>10.2}{:>9.3}{:>9.3}{}",
                est_name(e),
                ctrl_name(c),
                m.pos_rms,
                m.settle_time,
                m.final_alt,
                m.final_att_w,
                m.ctrl_effort,
                if m.diverge { "  DIVERGED" } else { "" }
            );
        }
    }

    // 单独再跑一遍 PID+Complementary 并把细节打印出来（便于肉眼核对）
    print_detail(&scenario, seconds);
}

/// 详细打印 PID+Complementary 的时间序列（对齐 PLAN 的 M1 验收）。
fn print_detail(scenario: &Scenario, seconds: f32) {
    println!(
        "\n[detail] PID + Complementary — scenario={}:", scenario.name()
    );
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());
    let wp = scenario.world_params();
    phys.set_wind(wp.wind);
    phys.set_wind_gust(wp.wind_gust);
    let mut comp = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let mut pid = PidController::default_quad();
    let dt = Second(0.005);
    let mut cmd = ActuatorCmd { motor: [0.5; 4] };
    let steps = (seconds / dt.0) as usize;
    println!(
        "{:>6}{:>10}{:>10}{:>10}{:>10}",
        "t(s)", "EST_x", "EST_z", "TRUE_z", "attW"
    );
    for k in 0..steps {
        let t = k as f32 * dt.0;
        let sp = scenario.setpoint_at(Second(t));
        let ideal = phys.step(dt, cmd);
        let s = phys.state();
        let (imu, gps) = world.sense(dt, ideal, s.pos);
        let est = comp.step(dt, imu, gps);
        cmd = pid.control(dt, &sp, &est);
        if (k % 200) == 0 || k == steps - 1 {
            println!(
                "{:>6.2}{:>10.3}{:>10.3}{:>10.3}{:>10.3}",
                t, est.pos[0].0, est.pos[2].0, s.pos[2].0, est.att.w
            );
        }
    }
}
