//! SITL 主机端：把 `flyctrl-core` 的仿真后端（Physics + World）与
//! 估计器/控制器组合驱动起来，打印统一收敛指标。
//!
//! 用法：
//!   cargo run -p flyctrl-sitl
//!   cargo run -p flyctrl-sitl -- --seconds 12
//!   cargo run -p flyctrl-sitl -- --est all --ctrl all
//!   cargo run -p flyctrl-sitl -- --est ekf --ctrl mpc
//!   cargo run -p flyctrl-sitl -- --scenario wind
//!   cargo run -p flyctrl-sitl -- --fdir            # FDIR 失效注入演示
//!
//! 默认对所有 (估计器 × 控制器) 组合跑一遍并输出对比表。

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::lqr::LqrController;
use flyctrl_core::controller::mpc::MpcController;
use flyctrl_core::controller::pid::PidController;
use flyctrl_core::controller::trait_def::{ActuatorCmd, Controller};
use flyctrl_core::estimator::complementary::ComplementaryEstimator;
use flyctrl_core::estimator::ekf::EkfEstimator;
use flyctrl_core::estimator::trait_def::Estimator;
use flyctrl_core::fdir::Fdir;
use flyctrl_core::vehicle::{Second, VehicleState};
use flyctrl_sim::harness::{Physics, World, WorldParams};
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
    Mpc,
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
        CtrlKind::Mpc => "MPC",
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
/// `fdir`: 若 Some，则启用 FDIR 监控，并在 `dropout` 窗口内强制 GPS 丢失（喂 None），
/// FDIR 降级时估计器自然忽略位置测量（仅姿态/高度保持）。
fn run_combo(
    est_kind: EstKind,
    ctrl_kind: CtrlKind,
    scenario: &Scenario,
    seconds: f32,
    fdir: Option<(f32, f32)>, // GPS dropout 窗口 [t0,t1]
) -> Metrics {
    let cfg = VehicleConfig::default_quad();
    let mut phys = Physics::new(cfg.dyn_params().into());
    let mut world = World::new(WorldParams::default());

    // 应用场景风扰（仅 Wind 场景生效，其余为 0）
    let wp = scenario.world_params();
    phys.set_wind(wp.wind);
    phys.set_wind_gust(wp.wind_gust);

    let mut comp = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let mut ekf = EkfEstimator::default_quad();
    let mut pid = PidController::from_config(&cfg.ctrl_params());
    let mut lqr = LqrController::from_config(&cfg.ctrl_params());
    let mut mpc = MpcController::from_config(&cfg.ctrl_params());

    let mut fdir_state = Fdir::new();

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
        let (imu, gps_true) = world.sense(dt, ideal, s.pos);

        // FDIR：检测 IMU 冻结 / GPS 丢失
        let gps_available = match fdir {
            Some((t0, t1)) => !(t >= t0 && t < t1),
            None => true,
        };
        let health = fdir_state.update(&imu, gps_available);
        // 降级（GPS 丢失）或无 GPS 可用性时，估计器忽略位置测量
        let gps = if gps_available { gps_true } else { None };

        let est: VehicleState = match est_kind {
            EstKind::Complementary => comp.step(dt, imu, gps),
            EstKind::Ekf => ekf.step(dt, imu, gps),
        };
        cmd = match ctrl_kind {
            CtrlKind::Pid => pid.control(dt, &sp, &est),
            CtrlKind::Lqr => lqr.control(dt, &sp, &est),
            CtrlKind::Mpc => mpc.control(dt, &sp, &est),
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

        // 稳定判定：Step 场景仅在阶跃之后计；FDIR dropout 期间不计入
        let settle_ok = (scenario.kind() != ScenarioKind::Step || t >= scenario.step_time())
            && gps_available;
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

        if fdir.is_some() && (k % 400 == 0 || k == steps - 1) {
            println!(
                "  [fdir t={:5.2}s] health={:?} gps={} est_z={:7.2} attW={:5.2}",
                t, health, gps_available, est.pos[2].0, est.att.w
            );
        }
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
    let mut fdir_demo = false;

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
            "--fdir" => {
                fdir_demo = true;
                i += 1;
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
        vec![CtrlKind::Pid, CtrlKind::Lqr, CtrlKind::Mpc]
    } else if ctrl_filter == "pid" {
        vec![CtrlKind::Pid]
    } else if ctrl_filter == "lqr" {
        vec![CtrlKind::Lqr]
    } else if ctrl_filter == "mpc" {
        vec![CtrlKind::Mpc]
    } else {
        vec![CtrlKind::Pid, CtrlKind::Lqr, CtrlKind::Mpc]
    };

    let scenario = Scenario::new(ScenarioKind::parse(&scenario_filter));

    if fdir_demo {
        // FDIR 演示：EKF+MPC，注入 GPS dropout 窗口 [4s,8s)，观察降级保持与恢复
        println!("flyctrl SITL — FDIR 演示 (EKF+MPC, GPS dropout [4s,8s))");
        let _ = run_combo(EstKind::Ekf, CtrlKind::Mpc, &scenario, seconds, Some((4.0, 8.0)));
        return;
    }

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
            let m = run_combo(e, c, &scenario, seconds, None);
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
    let cfg = VehicleConfig::default_quad();
    let mut phys = Physics::new(cfg.dyn_params().into());
    let mut world = World::new(WorldParams::default());
    let wp = scenario.world_params();
    phys.set_wind(wp.wind);
    phys.set_wind_gust(wp.wind_gust);
    let mut comp = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let mut pid = PidController::from_config(&cfg.ctrl_params());
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
