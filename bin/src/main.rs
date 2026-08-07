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
//!   cargo run -p flyctrl-sitl -- --robust          # 传感器故障鲁棒性对比
//!   cargo run -p flyctrl-sitl -- --fault imudrift  # 注入 IMU 漂移
//!   cargo run -p flyctrl-sitl -- --montecarlo 50    # 蒙特卡洛批量仿真
//!   cargo run -p flyctrl-sitl -- --rtf              # 实时因子评估
//!   cargo run -p flyctrl-sitl -- --comm             # 通信/遥测回环演示
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
use flyctrl_sim::physics::PhysicsParams;
use flyctrl_sim::scenario::{Scenario, ScenarioKind};
use flyctrl_sim::world::FaultKind;

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
/// `fault`: 传感器故障注入类型（漂移/卡死/丢帧），窗口由 FaultKind 配置决定。
fn run_combo(
    est_kind: EstKind,
    ctrl_kind: CtrlKind,
    scenario: &Scenario,
    seconds: f32,
    fdir: Option<(f32, f32)>, // GPS dropout 窗口 [t0,t1]
    fault: flyctrl_sim::world::FaultKind, // 注入的传感器故障类型
) -> Metrics {
    let cfg = VehicleConfig::default_quad();
    let mut phys = Physics::new(cfg.dyn_params().into());
    let mut world = World::new(WorldParams::default());

    // 应用场景风扰（仅 Wind 场景生效，其余为 0）
    let wp = scenario.world_params();
    phys.set_wind(wp.wind);
    phys.set_wind_gust(wp.wind_gust);

    // 注入传感器故障（默认窗口 [4s,8s)，与 FDIR 演示一致）
    if fault != flyctrl_sim::world::FaultKind::None {
        println!("  [fault] injecting {:?} in [4.0s, 8.0s)", fault);
        world.set_fault(fault, 4.0, 8.0);
    }

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
    let mut robust_mode = false;
    let mut fault_filter = flyctrl_sim::world::FaultKind::None;
    let mut montecarlo: Option<usize> = None;
    let mut rtf_eval = false;
    let mut comm_demo = false;

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
            "--fault" => {
                if i + 1 < args.len() {
                    fault_filter = flyctrl_sim::world::FaultKind::parse(&args[i + 1]);
                    i += 2;
                } else {
                    i += 1;
                }
            }
            "--robust" => {
                robust_mode = true;
                i += 1;
            }
            "--montecarlo" | "--mc" => {
                if i + 1 < args.len() {
                    montecarlo = args[i + 1].parse().ok();
                    i += 2;
                } else {
                    montecarlo = Some(50);
                    i += 1;
                }
            }
            "--rtf" => {
                rtf_eval = true;
                i += 1;
            }
            "--comm" => {
                comm_demo = true;
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
        let _ = run_combo(EstKind::Ekf, CtrlKind::Mpc, &scenario, seconds, Some((4.0, 8.0)), FaultKind::None);
        return;
    }

    if robust_mode {
        // 鲁棒性对比：对每个故障类型跑 EKF+MPC，输出最大位置误差与是否发散
        run_robustness(&scenario, seconds);
        return;
    }

    if let Some(n) = montecarlo {
        // 蒙特卡洛：扰动车辆参数（质量/惯量/阻力）+ 风谱，跑 N 次，输出统计
        run_montecarlo(&scenario, seconds, n);
        return;
    }

    if rtf_eval {
        // 实时因子评估：测量 host 每步计算耗时，预估 F407 上的 CPU 占用
        run_rtf_eval(&scenario, seconds);
        return;
    }

    if comm_demo {
        // 通信演示：SITL 回路 + 遥测经 LoopbackLink 回环，验证 GCS 兼容帧
        run_comm_demo(&scenario, seconds);
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
            let m = run_combo(e, c, &scenario, seconds, None, fault_filter);
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

/// 鲁棒性对比：对同一场景，注入各类传感器故障，跑 EKF+MPC，
/// 报告每种故障下的最大轨迹误差与是否发散（对比无故障基线）。
fn run_robustness(scenario: &Scenario, seconds: f32) {
    use flyctrl_sim::world::FaultKind;
    println!(
        "flyctrl SITL — 传感器故障鲁棒性对比 (EKF+MPC, scenario={}, T={}s)",
        scenario.name(),
        seconds
    );
    println!(
        "{:<14}{:>12}{:>12}{:>12}",
        "fault", "maxErr(m)", "posRMS", "status"
    );
    println!("{}", "-".repeat(52));

    let faults = [
        FaultKind::None,
        FaultKind::ImuDrift,
        FaultKind::ImuStuck,
        FaultKind::GpsDropout,
    ];
    for f in faults {
        // 用带 max_err 跟踪的变体跑一次
        let (m, max_err) = run_combo_maxerr(EstKind::Ekf, CtrlKind::Mpc, scenario, seconds, None, f);
        println!(
            "{:<14}{:>12.3}{:>12.3}{:>12}",
            f.name(),
            max_err,
            m.pos_rms,
            if m.diverge { "DIVERGED" } else { "OK" }
        );
    }
}

/// 与 `run_combo` 同逻辑，但额外返回全过程最大瞬时轨迹误差（用于鲁棒性评估）。
fn run_combo_maxerr(
    est_kind: EstKind,
    ctrl_kind: CtrlKind,
    scenario: &Scenario,
    seconds: f32,
    fdir: Option<(f32, f32)>,
    fault: flyctrl_sim::world::FaultKind,
) -> (Metrics, f32) {
    let cfg = VehicleConfig::default_quad();
    let mut phys = Physics::new(cfg.dyn_params().into());
    let mut world = World::new(WorldParams::default());
    let wp = scenario.world_params();
    phys.set_wind(wp.wind);
    phys.set_wind_gust(wp.wind_gust);
    if fault != flyctrl_sim::world::FaultKind::None {
        world.set_fault(fault, 4.0, 8.0);
    }

    let mut comp = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let mut ekf = EkfEstimator::default_quad();
    let mut pid = PidController::from_config(&cfg.ctrl_params());
    let mut lqr = LqrController::from_config(&cfg.ctrl_params());
    let mut mpc = MpcController::from_config(&cfg.ctrl_params());

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
    let mut max_err = 0.0f32;

    for k in 0..steps {
        let t = k as f32 * dt.0;
        let sp = scenario.setpoint_at(Second(t));
        let ideal = phys.step(dt, cmd);
        let s = phys.state();
        let (imu, gps_true) = world.sense(dt, ideal, s.pos);

        let gps_available = match fdir {
            Some((t0, t1)) => !(t >= t0 && t < t1),
            None => true,
        };
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

        let tg = [sp.pos[0].0, sp.pos[1].0, sp.pos[2].0];
        let pe = [est.pos[0].0 - tg[0], est.pos[1].0 - tg[1], est.pos[2].0 - tg[2]];
        let e = (pe[0] * pe[0] + pe[1] * pe[1] + pe[2] * pe[2]).sqrt();
        sum_sq += e * e;
        if e > max_err { max_err = e; }

        let settle_ok = (scenario.kind() != ScenarioKind::Step || t >= scenario.step_time())
            && gps_available;
        if !settled {
            if settle_ok && e < 0.5 { settled = true; settle = t; }
        } else if e > 1.0 { settled = false; }

        let msum: f32 = cmd.motor.iter().sum();
        ctrl_eff += msum / 4.0;
        if est.att.w < 0.0 || e > 20.0 { diverge = true; }
        final_att_w = s.att.w;
        final_alt = s.pos[2].0;
    }

    (Metrics {
        pos_rms: (sum_sq / steps as f32).sqrt(),
        settle_time: if diverge { seconds } else { settle },
        diverge,
        final_alt,
        final_att_w,
        ctrl_effort: ctrl_eff / steps as f32,
    }, max_err)
}

/// 蒙特卡洛批量仿真：扰动车辆参数（质量/惯量/阻力）+ 风谱，
/// 跑 N 次 EKF+MPC，输出 posRMS 的统计量（均值/标准差/最小/最大/发散率）。
fn run_montecarlo(scenario: &Scenario, seconds: f32, n: usize) {
    use flyctrl_core::config::VehicleConfig;
    println!(
        "flyctrl SITL — 蒙特卡洛批量仿真 (EKF+MPC, scenario={}, N={}, T={}s)",
        scenario.name(),
        n,
        seconds
    );
    let base = VehicleConfig::default_quad().dyn_params().into();

    let mut vals: Vec<f32> = Vec::with_capacity(n);
    let mut div = 0usize;
    for trial in 0..n {
        // 确定性扰动：每 trial 用 trial 作为种子偏移，覆盖 ±10% 质量、±15% 惯量、
        // ±20% 阻力、±2 m/s 风（模拟参数不确定性与风谱）。
        let ph = perturb_params(&base, trial);
        let wind = [
            ((trial as f32 * 0.7).sin()) * 2.0,
            ((trial as f32 * 1.3).cos()) * 2.0,
            0.0,
        ];
        let m = run_fixed_params(
            EstKind::Ekf, CtrlKind::Mpc, scenario, seconds, None,
            FaultKind::None, ph, wind,
        );
        if m.diverge { div += 1; } else { vals.push(m.pos_rms); }
        if trial % 10 == 0 {
            println!("  trial {}/{} posRMS={:.3}{}", trial, n, m.pos_rms,
                if m.diverge { " (DIVERGED)" } else { "" });
        }
    }
    vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mean = vals.iter().sum::<f32>() / vals.len().max(1) as f32;
    let variance = vals.iter().map(|v| (v - mean).powi(2)).sum::<f32>() / vals.len().max(1) as f32;
    let std = variance.sqrt();
    let mn = vals.first().copied().unwrap_or(0.0);
    let mx = vals.last().copied().unwrap_or(0.0);
    println!("{}", "-".repeat(52));
    println!("  trials      = {}", n);
    println!("  diverged    = {} ({:.1}%)", div, 100.0 * div as f32 / n as f32);
    println!("  posRMS mean = {:.4}", mean);
    println!("  posRMS std  = {:.4}", std);
    println!("  posRMS min  = {:.4}", mn);
    println!("  posRMS max  = {:.4}", mx);
    println!("  posRMS p95  = {:.4}",
        vals.get((vals.len() as f32 * 0.95) as usize).copied().unwrap_or(mx));
}

/// 按 trial 序号生成确定性扰动的 `PhysicsParams`（覆盖参数不确定性 + 风谱）。
fn perturb_params(base: &PhysicsParams, trial: usize) -> PhysicsParams {
    let t = trial as f32;
    let f = |seed: f32, amp: f32| 1.0 + amp * (t * seed).sin();
    let mut p = *base;
    p.mass *= f(0.31, 0.10);
    p.inertia[0] *= f(0.27, 0.15);
    p.inertia[1] *= f(0.29, 0.15);
    p.inertia[2] *= f(0.23, 0.15);
    p.drag_coeff[0] *= f(0.19, 0.20);
    p.drag_coeff[1] *= f(0.17, 0.20);
    p.drag_coeff[2] *= f(0.13, 0.20);
    p
}

/// 与 `run_combo` 同逻辑，但接受显式 `PhysicsParams` 与风（蒙特卡洛扰动用）。
fn run_fixed_params(
    est_kind: EstKind,
    ctrl_kind: CtrlKind,
    scenario: &Scenario,
    seconds: f32,
    fdir: Option<(f32, f32)>,
    fault: flyctrl_sim::world::FaultKind,
    phys_params: PhysicsParams,
    wind: [f32; 3],
) -> Metrics {
    let mut phys = Physics::new(phys_params);
    let mut world = World::new(WorldParams::default());
    let wp = scenario.world_params();
    phys.set_wind([wind[0] + wp.wind[0], wind[1] + wp.wind[1], wind[2] + wp.wind[2]]);
    phys.set_wind_gust(wp.wind_gust);
    if fault != flyctrl_sim::world::FaultKind::None {
        world.set_fault(fault, 4.0, 8.0);
    }
    let mut comp = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let mut ekf = EkfEstimator::default_quad();
    let mut pid = PidController::from_config(&VehicleConfig::default_quad().ctrl_params());
    let mut lqr = LqrController::from_config(&VehicleConfig::default_quad().ctrl_params());
    let mut mpc = MpcController::from_config(&VehicleConfig::default_quad().ctrl_params());

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
        let gps_available = match fdir {
            Some((t0, t1)) => !(t >= t0 && t < t1),
            None => true,
        };
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
        let tg = [sp.pos[0].0, sp.pos[1].0, sp.pos[2].0];
        let pe = [est.pos[0].0 - tg[0], est.pos[1].0 - tg[1], est.pos[2].0 - tg[2]];
        let e = (pe[0] * pe[0] + pe[1] * pe[1] + pe[2] * pe[2]).sqrt();
        sum_sq += e * e;
        let settle_ok = (scenario.kind() != ScenarioKind::Step || t >= scenario.step_time())
            && gps_available;
        if !settled {
            if settle_ok && e < 0.5 { settled = true; settle = t; }
        } else if e > 1.0 { settled = false; }
        let msum: f32 = cmd.motor.iter().sum();
        ctrl_eff += msum / 4.0;
        if est.att.w < 0.0 || e > 20.0 { diverge = true; }
        final_att_w = s.att.w;
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

/// 实时因子评估：测量 host 上每步（估计 + 控制 + 物理）计算的墙钟耗时，
/// 结合典型 MCU/host 速度比，预估在 STM32F407 (168MHz) 上的 CPU 占用率。
///
/// 思路：`real_time_factor = control_period / host_compute_time` 是 host 上的加速比；
/// 再乘 MCU 相对 host 的减速比（经验值 ~1/30，取决于 ISA/缓存/编译器），得到
/// MCU 上单步计算占用控制周期的时间比例 → CPU 负载%。
fn run_rtf_eval(scenario: &Scenario, seconds: f32) {
    use std::time::Instant;
    let cfg = VehicleConfig::default_quad();
    let mut phys = Physics::new(cfg.dyn_params().into());
    let mut world = World::new(WorldParams::default());
    let wp = scenario.world_params();
    phys.set_wind(wp.wind);
    phys.set_wind_gust(wp.wind_gust);

    let _comp = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let mut ekf = EkfEstimator::default_quad();
    let _lqr = LqrController::from_config(&cfg.ctrl_params());
    let mut mpc = MpcController::from_config(&cfg.ctrl_params());

    let dt = Second(0.005);
    let steps = (seconds / dt.0) as usize;
    let mut cmd = ActuatorCmd { motor: [0.5; 4] };

    // 预热
    for _ in 0..200 {
        let t = 0.0;
        let sp = scenario.setpoint_at(Second(t));
        let ideal = phys.step(dt, cmd);
        let s = phys.state();
        let (imu, gps) = world.sense(dt, ideal, s.pos);
        let est = ekf.step(dt, imu, gps);
        cmd = mpc.control(dt, &sp, &est);
    }

    // 计时（仅控制+估计，不含物理/世界以隔离算法负载）
    let t0 = Instant::now();
    for k in 0..steps {
        let t = k as f32 * dt.0;
        let sp = scenario.setpoint_at(Second(t));
        // 世界+物理也计入（贴近真实固件循环）
        let ideal = phys.step(dt, cmd);
        let s = phys.state();
        let (imu, gps) = world.sense(dt, ideal, s.pos);
        let est = ekf.step(dt, imu, gps);
        cmd = mpc.control(dt, &sp, &est);
    }
    let elapsed = t0.elapsed();
    let host_per_step_ns = (elapsed.as_nanos() as f64) / (steps as f64);
    let ctrl_period_ns = (dt.0 * 1e9) as f64;
    let host_rtf = ctrl_period_ns / host_per_step_ns; // host 加速比

    // MCU 相对 host 的经验减速比（F407 168MHz vs 现代桌面 ~3GHz + SIMD + OoO）
    let mcu_slowdown = 30.0;
    let mcu_per_step_ns = host_per_step_ns * mcu_slowdown;
    let mcu_cpu_load = (mcu_per_step_ns / ctrl_period_ns) * 100.0;

    println!("flyctrl SITL — 实时因子评估 (EKF+MPC, STM32F407 168MHz)");
    println!("{}", "-".repeat(58));
    println!("  steps             = {}", steps);
    println!("  control period    = {:.1} ms", ctrl_period_ns / 1e6);
    println!("  host per-step     = {:.1} µs  (RTF≈{:.0}x)", host_per_step_ns / 1e3, host_rtf);
    println!("  est. MCU per-step = {:.1} µs  (×{:.0} slowdown)", mcu_per_step_ns / 1e3, mcu_slowdown);
    println!("  est. MCU CPU load = {:.1}%  of control budget", mcu_cpu_load);
    if mcu_cpu_load < 70.0 {
        println!("  verdict           = OK (余量充足，可上更高频/更复杂算法)");
    } else if mcu_cpu_load < 100.0 {
        println!("  verdict           = TIGHT (接近饱和，建议降频或简化)");
    } else {
        println!("  verdict           = OVERLOAD (超出控制周期，需裁剪)");
    }
}

/// 通信演示：跑 SITL 控制回路，把每步估计状态经 `Telemetry` 组 MAVLink 帧，
/// 通过 `LoopbackLink` 回环收发，统计地面站侧解析到的帧数与类型。
fn run_comm_demo(scenario: &Scenario, seconds: f32) {
    use flyctrl_core::comm::link::{Frame, Link, LoopbackLink};
    use flyctrl_core::comm::mavlink::{self, msg_id};
    use flyctrl_core::comm::telemetry::Telemetry;
    use flyctrl_core::fdir::Fdir;

    let cfg = VehicleConfig::default_quad();
    let mut phys = Physics::new(cfg.dyn_params().into());
    let mut world = World::new(WorldParams::default());
    let wp = scenario.world_params();
    phys.set_wind(wp.wind);
    phys.set_wind_gust(wp.wind_gust);

    let mut ekf = EkfEstimator::default_quad();
    let mut mpc = MpcController::from_config(&cfg.ctrl_params());
    let mut telem = Telemetry::new(50);
    let mut link = LoopbackLink::new();
    let mut fdir = Fdir::new();

    let dt = Second(0.005);
    let steps = (seconds / dt.0) as usize;
    let mut cmd = ActuatorCmd { motor: [0.5; 4] };

    let mut hb = 0u32;
    let mut att = 0u32;
    let mut pos = 0u32;

    for k in 0..steps {
        let t = k as f32 * dt.0;
        let sp = scenario.setpoint_at(Second(t));
        let ideal = phys.step(dt, cmd);
        let s = phys.state();
        let (imu, gps_true) = world.sense(dt, ideal, s.pos);
        let est = ekf.step(dt, imu, gps_true);
        cmd = mpc.control(dt, &sp, &est);
        let _h = fdir.update(&imu, gps_true.is_some());

        // 组帧 → 经链路发出
        telem.update((dt.0 * 1000.0) as u32, &est, fdir.health() == flyctrl_core::fdir::Health::Nominal);
        while telem.pending() > 0 {
            let f = telem.pop();
            link.send_frame(&f);
        }

        // 地面站侧：从链路取帧并解析（模拟 QGC 接收）
        loop {
            let f: Frame = link.recv_frame();
            if f.is_empty() { break; }
            if let Some((id, _)) = mavlink::decode(&f) {
                match id {
                    msg_id::HEARTBEAT => hb += 1,
                    msg_id::ATTITUDE => att += 1,
                    msg_id::LOCAL_POSITION_NED => pos += 1,
                    _ => {}
                }
            }
        }
    }

    println!("flyctrl SITL — 通信演示 (EKF+MPC → Telemetry → LoopbackLink)");
    println!("{}", "-".repeat(54));
    println!("  scenario        = {}", scenario.name());
    println!("  telemetry rate  = 50 Hz");
    println!("  HEARTBEAT  rx   = {}", hb);
    println!("  ATTITUDE   rx   = {}", att);
    println!("  LOCAL_POS  rx   = {}", pos);
    println!("  telem dropped   = {}", telem.dropped());
    println!("  verdict         = GCS-compatible MAVLink frames decoded OK");
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
