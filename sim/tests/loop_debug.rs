//! 全闭环调试：物理+估计+控制，逐秒打印姿态，定位发散点。
use flyctrl_sim::physics::{Physics, PhysicsParams};
use flyctrl_sim::world::{World, WorldParams};
use flyctrl_sim::harness::Harness;
use flyctrl_core::controller::{PidController, Setpoint, Controller};
use flyctrl_core::estimator::{ComplementaryEstimator, Estimator};
use flyctrl_core::units::*;
use flyctrl_core::vehicle::Quaternion;

fn approx_rp(q: Quaternion) -> (f32, f32) {
    let roll = (2.0 * (q.w * q.x + q.y * q.z)).atan2(1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    let pitch = (2.0 * (q.w * q.y - q.z * q.x)).asin();
    (roll, pitch)
}

#[test]
fn diag_b_mixer_onset() {
    // 复刻 PhysB 闭环，逐拍打印控制器内部混控分量，定位 cmd0=0 的来源。
    use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, PosSample, VehicleState};
    use flyctrl_core::vehicle::{quat_mul, quat_conj};
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());
    let mut est = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let mut ctrl = PidController::default_quad();
    let dt = Second(0.005);
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
    let mut cmd = ActuatorCmd { motor: [0.5; 4] };
    for k in 0..200 {
        let ideal = phys.step(dt, cmd);
        let s = phys.state();
        let imu = world.sense(dt, ideal, s.pos).0;
        let e = est.step(dt, imu, Some(PosSample{pos:[Meter(s.pos[0].0),Meter(s.pos[1].0),Meter(s.pos[2].0)]}));
        // 重算控制器内部（复制 pid.rs 逻辑以便打印）
        let ex = sp.pos[0].0 - e.pos[0].0;
        let ey = sp.pos[1].0 - e.pos[1].0;
        let ez = sp.pos[2].0 - e.pos[2].0;
        let des_vx = clampf(0.5*ex, -2.0, 2.0);
        let des_vy = clampf(0.5*ey, -2.0, 2.0);
        let des_vz = clampf(0.5*ez, -2.0, 2.0);
        let acc_n = 0.8*(des_vx - e.vel[0].0);
        let acc_e = 0.8*(des_vy - e.vel[1].0);
        let acc_d = 0.8*(des_vz - e.vel[2].0);
        let des_thrust = clampf(0.5 - acc_d/9.81, 0.1, 1.0);
        let tilt_n = clampf(acc_n/9.81, -0.35, 0.35);
        let tilt_e = clampf(acc_e/9.81, -0.35, 0.35);
        let q_des = Quaternion::from_euler(Radian(tilt_e), Radian(-tilt_n), Radian(0.0));
        let q_err = quat_mul(quat_conj(e.att), q_des);
        let sgn = if q_err.w < 0.0 {-2.0} else {2.0};
        let ex_b = sgn*q_err.x; let ey_b = sgn*q_err.y; let ez_b = sgn*q_err.z;
        let p_cmd = 3.0*ex_b - 0.3*e.omega[0].0;
        let q_cmd = 3.0*ey_b - 0.3*e.omega[1].0;
        let r_cmd = 3.0*ez_b - 0.3*e.omega[2].0;
        let m0 = des_thrust + 0.5*(q_cmd + p_cmd + r_cmd);
        cmd = ctrl.control(dt, &sp, &e);
        if k >= 55 && k <= 100 {
            eprintln!("k={} ez={:.2} e_vz={:.2} acc_d={:.2} dThr={:.3} | e_vx={:.2} e_vy={:.2} til_n={:.3} til_e={:.3} | eAtt=({:.3},{:.3},{:.3},{:.3}) om=({:.2},{:.2},{:.2}) | TRUE att.x={:.4} om0={:.3} | pcmd={:.2} qcmd={:.2} m0={:.3}",
                k, ez, e.vel[2].0, acc_d, des_thrust,
                e.vel[0].0, e.vel[1].0, tilt_n, tilt_e,
                e.att.w, e.att.x, e.att.y, e.att.z,
                e.omega[0].0, e.omega[1].0, e.omega[2].0,
                s.att.x, s.omega[0].0,
                p_cmd, q_cmd, m0.clamp(0.0,1.0));
        }
    }
}

fn clampf(v: f32, lo: f32, hi: f32) -> f32 { if v<lo {lo} else if v>hi {hi} else {v} }

#[test]
fn estimator_tracks_hover() {
    // 物理用稳定悬停电机，把真实 IMU+位置喂给估计器，看姿态/位置是否漂移。
    use flyctrl_core::estimator::ComplementaryEstimator;
    use flyctrl_core::units::Second;
    use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, PosSample};
    let mut phys = Physics::new(PhysicsParams::default());
    let mut est = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let dt = Second(0.005);
    for sec in 0..4 {
        for k in 0..200 {
            let imu: ImuSample = phys.step(dt, ActuatorCmd { motor: [0.5; 4] });
            let s = phys.state();
            let pos = PosSample { pos: [s.pos[0], s.pos[1], s.pos[2]] };
            let est_s = est.step(dt, imu, Some(pos));
            if sec == 0 && k % 40 == 0 {
                eprintln!("  k={} TRUE_pos=({:.3},{:.3},{:.3}) EST_pos=({:.3},{:.3},{:.3}) EST_vel=({:.3},{:.3},{:.3})",
                    k, s.pos[0].0, s.pos[1].0, s.pos[2].0,
                    est_s.pos[0].0, est_s.pos[1].0, est_s.pos[2].0,
                    est_s.vel[0].0, est_s.vel[1].0, est_s.vel[2].0);
            }
            if sec == 3 {
                let (roll, pitch) = approx_rp(est_s.att);
                eprintln!("  gyro=({:.3},{:.3},{:.3}) accel_z={:.2} est.att.w={:.3} roll={:.3} pitch={:.3}",
                    imu.gyro[0].0, imu.gyro[1].0, imu.gyro[2].0,
                    imu.accel[2].0, est_s.att.w, roll, pitch);
            }
        }
        let s = phys.state();
        let est_s = est.step(dt, ImuSample { gyro: [flyctrl_core::units::RadianPerSecond(0.0); 3], accel: [flyctrl_core::units::MeterPerSecondSquared(0.0); 3] }, None);
        eprintln!("t={}s phys_alt={:.2} est.att.w={:.3}", sec+1, -s.pos[2].0, est_s.att.w);
    }
}

#[test]
fn imu_integrate_true_att_diag() {
    use flyctrl_core::vehicle::ActuatorCmd;
    use flyctrl_core::estimator::{ComplementaryEstimator, Estimator};
    use flyctrl_core::vehicle::{ImuSample, PosSample};
    // 诊断：对比 (a) 真实姿态积分速度 (b) 纯陀螺估计姿态积分速度，
    // 确认 -8m/s 偏差是否来自估计姿态微小误差。
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());
    let mut est = ComplementaryEstimator::new(0.0, 0.0, 0.0); // 纯陀螺
    let mut ctrl = PidController::default_quad();
    let dt = Second(0.005);
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
    let mut cmd = ActuatorCmd { motor: [0.5; 4] };
    let mut vz_true = 0.0f32;
    let mut vz_est = 0.0f32;
    for k in 0..1600 {
        let ideal = phys.step(dt, cmd);
        let s = phys.state();
        let imu = world.sense(dt, ideal, s.pos).0;
        let e = est.step(dt, imu, None);
        let f_t = flyctrl_core::vehicle::rotate_vec_by_quat(s.att, [ideal.accel[0].0, ideal.accel[1].0, ideal.accel[2].0]);
        let f_e = flyctrl_core::vehicle::rotate_vec_by_quat(e.att, [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0]);
        vz_true += (f_t[2] + 9.81) * dt.0;
        vz_est  += (f_e[2] + 9.81) * dt.0;
        cmd = ctrl.control(dt, &sp, &e);
        if k % 400 == 399 {
            eprintln!("t={}s TRUE_vz={:.2} EST_vz={:.2} dAtt=({:.3},{:.3},{:.3},{:.3}) alt={:.2}",
                (k+1) as f32*dt.0, vz_true, vz_est,
                e.att.w-s.att.w, e.att.x-s.att.x, e.att.y-s.att.y, e.att.z-s.att.z,
                -s.pos[2].0);
        }
    }
}

#[test]
fn controller_with_true_state() {
    // 用物理真实状态直接喂控制器（绕过估计器），隔离控制器稳定性。
    use flyctrl_core::controller::{PidController, Controller};
    let mut phys = Physics::new(PhysicsParams::default());
    let mut ctrl = PidController::default_quad();
    let dt = Second(0.005);
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
    for sec in 0..6 {
        let mut s = phys.state();
        for _ in 0..200 {
            let cmd = ctrl.control(dt, &sp, &s);
            let _ = phys.step(dt, cmd);
            s = phys.state();
        }
        eprintln!("t={}s alt={:.2} att.w={:.3}", sec+1, -s.pos[2].0, s.att.w);
    }
}

#[test]
fn scan_estimator_gain() {
    use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, PosSample};
    // 扫描位置偏差回拉(pos_bias_alpha)与输出低通(pos_out_alpha)，
    // 找能收敛到目标高度(-10m附近)且不翻车的组合。
    for &bias_a in &[0.005f32, 0.01, 0.02, 0.05] {
        for &out_a in &[0.02f32, 0.05, 0.1] {
            let mut phys = Physics::new(PhysicsParams::default());
            let mut world = World::new(WorldParams::default());
            let mut est = ComplementaryEstimator::new(0.0, bias_a, out_a);
            let mut ctrl = PidController::default_quad();
            let dt = Second(0.005);
            let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
            let mut cmd = ActuatorCmd { motor: [0.5; 4] };
            let mut alt8 = 0.0f32; let mut minw = 1.0f32;
            for k in 0..1600 { // 8s
                let ideal = phys.step(dt, cmd);
                let s = phys.state();
                let imu = world.sense(dt, ideal, s.pos).0;
                let e = est.step(dt, imu, Some(PosSample{pos:[Meter(s.pos[0].0),Meter(s.pos[1].0),Meter(s.pos[2].0)]}));
                cmd = ctrl.control(dt, &sp, &e);
                if s.att.w.abs() < minw { minw = s.att.w.abs(); }
                if k >= 1599 { alt8 = -s.pos[2].0; }
            }
            eprintln!("bias_a={:.3} out_a={:.3} -> alt8={:.2}m min|w|={:.3}", bias_a, out_a, alt8, minw);
        }
    }
}

#[test]
fn controller_with_true_state_via_harness() {
    // 复刻 Harness 闭环，但控制器吃物理真实姿态（绕过估计器），
    // 用于一锤定音：问题在"控制器+物理"还是"估计器姿态误差"。
    use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, PosSample};
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());
    let mut ctrl = PidController::default_quad();
    let dt = Second(0.005);
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
    let mut last_cmd = ActuatorCmd { motor: [0.5; 4] };
    for sec in 0..8 {
        for _ in 0..200 {
            let ideal = phys.step(dt, last_cmd);
            let s = phys.state();
            let _ = world.sense(dt, ideal, s.pos); // 仍走 world（但不影响控制输入）
            let cmd = ctrl.control(dt, &sp, &s);   // 真实姿态
            last_cmd = cmd;
        }
        let s = phys.state();
        eprintln!("t={}s alt={:.2} att.w={:.3} roll={:.3} pitch={:.3}",
            sec+1, -s.pos[2].0, s.att.w, approx_rp(s.att).0, approx_rp(s.att).1);
    }
}

#[test]
fn est_att_true_vel_diag() {
    // 用估计姿态 + 物理真实速度 做控制：若收敛 -> 问题纯在速度估计；若仍发散 -> 姿态或其他。
    use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, PosSample, VehicleState};
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());
    let mut est = ComplementaryEstimator::new(0.0, 0.1, 0.1);
    let mut ctrl = PidController::default_quad();
    let dt = Second(0.005);
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
    let mut cmd = ActuatorCmd { motor: [0.5; 4] };
    let mut last_e = phys.state();
    for sec in 0..8 {
        for k in 0..200 {
            let ideal = phys.step(dt, cmd);
            let s = phys.state();
            let imu = world.sense(dt, ideal, s.pos).0;
            let e = est.step(dt, imu, Some(PosSample{pos:[Meter(s.pos[0].0),Meter(s.pos[1].0),Meter(s.pos[2].0)]}));
            last_e = e;
            // 诊断：用真实 att/vel/omega，但位置用【带噪估计位置】e.pos
            let fused = VehicleState { pos: e.pos, vel: s.vel, att: s.att, omega: s.omega };
            cmd = ctrl.control(dt, &sp, &fused);
            if k >= 100 && k <= 140 {
                eprintln!("  step{} cmd=({:.3},{:.3},{:.3},{:.3}) TRUE_z={:.2} EST_z={:.2} TRUE_vz={:.2} EST_vz={:.2} EST_vxy=({:.2},{:.2})",
                    k, cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3],
                    s.pos[2].0, e.pos[2].0, s.vel[2].0, e.vel[2].0, e.vel[0].0, e.vel[1].0);
            }
        }
        let s = phys.state();
        eprintln!("t={}s alt={:.2} att.w={:.3} e.om=({:.2},{:.2},{:.2}) s.om=({:.2},{:.2},{:.2})",
            sec+1, -s.pos[2].0, s.att.w,
            last_e.omega[0].0, last_e.omega[1].0, last_e.omega[2].0,
            s.omega[0].0, s.omega[1].0, s.omega[2].0);
    }
}

#[test]
fn est_posvel_true_omega_diag() {
    // 用估计 pos/vel + 【真实 omega】 做控制：隔离 omega 估计（噪声陀螺）是否导致翻滚。
    use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, PosSample, VehicleState};
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());
    let mut est = ComplementaryEstimator::new(0.0, 0.1, 0.1);
    let mut ctrl = PidController::default_quad();
    let dt = Second(0.005);
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
    let mut cmd = ActuatorCmd { motor: [0.5; 4] };
    let mut last_e = phys.state();
    for sec in 0..8 {
        for _ in 0..200 {
            let ideal = phys.step(dt, cmd);
            let s = phys.state();
            let imu = world.sense(dt, ideal, s.pos).0;
            let e = est.step(dt, imu, Some(PosSample{pos:[Meter(s.pos[0].0),Meter(s.pos[1].0),Meter(s.pos[2].0)]}));
            let fused = VehicleState { pos: e.pos, vel: e.vel, att: e.att, omega: s.omega }; // 真实 omega
            cmd = ctrl.control(dt, &sp, &fused);
            last_e = e;
        }
        let s = phys.state();
        eprintln!("t={}s alt={:.2} est_z={:.2} att.w={:.3}", sec+1, -s.pos[2].0, -last_e.pos[2].0, s.att.w);
    }
}

#[test]
fn est_posvel_true_att_diag() {
    // 用真实 att + 估计 pos/vel/omega 做控制：隔离 att 估计是否为翻滚元凶。
    use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, PosSample, VehicleState};
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());
    let mut est = ComplementaryEstimator::new(0.0, 0.1, 0.1);
    let mut ctrl = PidController::default_quad();
    let dt = Second(0.005);
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
    let mut cmd = ActuatorCmd { motor: [0.5; 4] };
    let mut last_e = phys.state();
    for sec in 0..8 {
        for _ in 0..200 {
            let ideal = phys.step(dt, cmd);
            let s = phys.state();
            let imu = world.sense(dt, ideal, s.pos).0;
            let e = est.step(dt, imu, Some(PosSample{pos:[Meter(s.pos[0].0),Meter(s.pos[1].0),Meter(s.pos[2].0)]}));
            let fused = VehicleState { pos: e.pos, vel: e.vel, att: s.att, omega: e.omega }; // 真实 att
            cmd = ctrl.control(dt, &sp, &fused);
            last_e = e;
        }
        let s = phys.state();
        eprintln!("t={}s alt={:.2} est_z={:.2} att.w={:.3}", sec+1, -s.pos[2].0, -last_e.pos[2].0, s.att.w);
    }
}

#[test]
fn parallel_true_vs_est() {
    // 终极并列：两个独立 phys，初始相同。physA 用全真实状态控制，physB 用全估计状态控制。
    // 逐步对比两者 cmd 与轨迹，看 physB 何时开始偏离 physA（即估计量如何引爆发散）。
    use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, PosSample};
    let mut physA = Physics::new(PhysicsParams::default());
    let mut physB = Physics::new(PhysicsParams::default());
    let mut worldA = World::new(WorldParams::default());
    let mut worldB = World::new(WorldParams::default());
    let mut estB = ComplementaryEstimator::new(0.5, 0.1, 0.1);
    let mut ctrlA = PidController::default_quad();
    let mut ctrlB = PidController::default_quad();
    let dt = Second(0.005);
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
    let mut cmdA = ActuatorCmd { motor: [0.5; 4] };
    let mut cmdB = ActuatorCmd { motor: [0.5; 4] };
    for sec in 0..8 {
        for k in 0..200 {
            // A: 全真实
            let _ = worldA.sense(dt, physA.step(dt, cmdA), physA.state().pos);
            let sA = physA.state();
            cmdA = ctrlA.control(dt, &sp, &sA);
            // B: 全估计
            let idealB = physB.step(dt, cmdB);
            let sB = physB.state();
            let imuB = worldB.sense(dt, idealB, sB.pos).0;
            let eB = estB.step(dt, imuB, Some(PosSample{pos:[Meter(sB.pos[0].0),Meter(sB.pos[1].0),Meter(sB.pos[2].0)]}));
            cmdB = ctrlB.control(dt, &sp, &eB);
            if sec <= 1 && k % 20 == 0 {
                eprintln!("  step{} A: z={:.2} vz={:.2} w={:.3} cmd0={:.3} | B: z={:.2}[e{:.2}] vz={:.2}[e{:.2}] w={:.3} cmd0={:.3}",
                    sec*200+k, sA.pos[2].0, sA.vel[2].0, sA.att.w, cmdA.motor[0],
                    sB.pos[2].0, eB.pos[2].0, sB.vel[2].0, eB.vel[2].0, sB.att.w, cmdB.motor[0]);
            }
        }
        let sA = physA.state();
        let sB = physB.state();
        eprintln!("t={}s A: alt={:.2} w={:.3} | B: alt={:.2} w={:.3}",
            sec+1, -sA.pos[2].0, sA.att.w, -sB.pos[2].0, sB.att.w);
    }
}

#[test]
fn trace_loop() {
    let phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());
    let est = ComplementaryEstimator::new(0.5, 0.1, 0.1); // 姿态带 accel 修正 + 位置测量低通主导
    let ctrl = PidController::default_quad();
    let dt = Second(0.005);
    let mut h = Harness::new(phys, world, est, ctrl, dt);
    let sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
    for sec in 0..8 {
        let _ = h.run(Second(1.0), &sp);
        let s = h.state();
        let e = h.last_est();
        let datt = s.att.w - e.att.w;
        let (er, ep) = approx_rp(e.att);
        let (sr, sp2) = approx_rp(s.att);
        eprintln!("t={}s TRUE_z={:.2} TRUE_vz={:.2} EST_z={:.2} EST_vz={:.2} dAttW={:+.3} EST_rp=({:.4},{:.4}) TRUE_rp=({:.4},{:.4}) cmd0={:.3}",
            sec+1, s.pos[2].0, s.vel[2].0, e.pos[2].0, e.vel[2].0, datt, er, ep, sr, sp2,
            h.last_cmd().motor[0]);
    }
}
