//! P3-A1 轨迹跟踪 + 倾斜补偿：全部标准场景指标表。
//!
//! 用同一套闭环（物理 + 世界 + EKF + PID）跑 Hover/Step/Wind/Square/Circle，
//! 输出统一指标（位置 RMS/峰值、整定时间、数值发散、单步最坏耗时），
//! 观察速度前馈 + 加速度前馈 + cos_tilt 倾斜补偿下的实际跟踪效果。

use flyctrl_core::controller::pid::PidController;
use flyctrl_core::estimator::ekf::EkfEstimator;
use flyctrl_core::units::Second;
use flyctrl_sim::harness::Harness;
use flyctrl_sim::physics::{Physics, PhysicsParams};
use flyctrl_sim::scenario::{Scenario, ScenarioKind};
use flyctrl_sim::world::{World, WorldParams};

#[test]
fn scenario_metrics_table() {
    let dt = Second(0.005);
    let dur = Second(20.0);
    println!("\n=== flyctrl-sim 场景指标表（PID + EKF，dt=5ms，{}s）===", dur.0);
    println!(
        "{:<8} {:>10} {:>10} {:>10} {:>8} {:>12}",
        "场景", "pos_RMS(m)", "pos_max(m)", "整定(s)", "发散", "单步最坏(ms)"
    );
    for kind in ScenarioKind::all() {
        let phys = Physics::new(PhysicsParams::default());
        let world = World::new(WorldParams::default());
        let est = EkfEstimator::default_quad();
        let ctrl = PidController::default_quad();
        let mut h = Harness::new(phys, world, est, ctrl, dt);
        let scenario = Scenario::new(*kind);
        let m = h.run_scenario(dur, &scenario);
        println!(
            "{:<8} {:>10.3} {:>10.3} {:>10.2} {:>8} {:>12.4}",
            kind.name(),
            m.pos_rms,
            m.pos_max,
            m.settle_time,
            if m.nan_detected { "YES" } else { "no" },
            m.worst_step_ms,
        );
    }
}

#[test]
fn vertical_osc_probe() {
    // 垂直环阻尼探针：Hover 跑 20s，统计稳态段(t>=10s)的 z/vz 幅值，量化慢振荡。
    use flyctrl_core::controller::Controller;
    use flyctrl_core::estimator::Estimator;
    use flyctrl_core::vehicle::ActuatorCmd;
    let dt = Second(0.005);
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());
    let mut est = EkfEstimator::default_quad();
    let mut ctrl = PidController::default_quad();
    let scenario = Scenario::new(ScenarioKind::Hover);
    let mut cmd = ActuatorCmd::zero();
    let mut z_min = f32::MAX;
    let mut z_max = f32::MIN;
    let mut vz_min = f32::MAX;
    let mut vz_max = f32::MIN;
    let mut z2_min = f32::MAX;
    let mut z2_max = f32::MIN;
    let mut vz2_min = f32::MAX;
    let mut vz2_max = f32::MIN;
    // 前 10s 含下潜暂态；统计 t>=10s 与 t>=20s 两段振幅，区分慢衰减暂态 vs 持续极限环。
    for sec in 0..30u32 {
        for _ in 0..200 {
            let ideal = phys.step(dt, cmd);
            let s = phys.state();
            let (imu, gps) = world.sense(dt, ideal, s.pos);
            let e = est.step(dt, imu, gps, None);
            cmd = ctrl.control(dt, &scenario.setpoint_at(Second(sec as f32 + 1.0)), &e);
        }
        let s = phys.state();
        if sec >= 10 && s.pos[2].0.is_finite() {
            z_min = z_min.min(s.pos[2].0);
            z_max = z_max.max(s.pos[2].0);
            vz_min = vz_min.min(s.vel[2].0);
            vz_max = vz_max.max(s.vel[2].0);
        }
        if sec >= 20 && s.pos[2].0.is_finite() {
            z2_min = z2_min.min(s.pos[2].0);
            z2_max = z2_max.max(s.pos[2].0);
            vz2_min = vz2_min.min(s.vel[2].0);
            vz2_max = vz2_max.max(s.vel[2].0);
        }
        if sec >= 10 && sec % 2 == 0 {
            let dbg = ctrl.debug_pid_internal();
            println!(
                "t={:>2}s z={:>7.3} vz={:>6.3} | ez={:>6.3} iz={:>5.3} des_vz={:>6.3} acc_d={:>6.3} des_thr={:>5.3}",
                sec, s.pos[2].0, s.vel[2].0, dbg.4, dbg.5, dbg.6, dbg.7, dbg.8
            );
        }
    }
    println!(
        "Hover 稳态 t>=10s: z∈[{:.3},{:.3}] Δz={:.3}m  vz∈[{:.3},{:.3}] Δvz={:.3}m/s",
        z_min, z_max, z_max - z_min, vz_min, vz_max, vz_max - vz_min
    );
    println!(
        "Hover 稳态 t>=20s: z∈[{:.3},{:.3}] Δz={:.3}m  vz∈[{:.3},{:.3}] Δvz={:.3}m/s",
        z2_min, z2_max, z2_max - z2_min, vz2_min, vz2_max, vz2_max - vz2_min
    );
}

#[test]
fn vertical_wind_probe() {
    // 垂直环抗垂向风扰探针：悬停稳定后施加恒定下洗风（wind_z>0，NED 向下），
    // 记录垂直位置峰值偏差 / 稳态偏差 / 撤风恢复时间，量化 ki_z 对垂直抗扰的影响。
    use flyctrl_core::controller::{Controller, Setpoint};
    use flyctrl_core::estimator::Estimator;
    use flyctrl_core::units::{Meter, Radian};
    use flyctrl_core::vehicle::ActuatorCmd;
    let dt = Second(0.005);
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(WorldParams::default());
    let mut est = EkfEstimator::default_quad();
    let mut ctrl = PidController::default_quad();
    let mut cmd = ActuatorCmd::zero();
    let mut sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));
    let mut z = -10.0f32;
    macro_rules! step_once {
        () => {
            let ideal = phys.step(dt, cmd);
            let s = phys.state();
            let (imu, gps) = world.sense(dt, ideal, s.pos);
            let e = est.step(dt, imu, gps, None);
            cmd = ctrl.control(dt, &sp, &e);
            z = phys.state().pos[2].0;
        };
    }

    // 1) 无风悬停稳定到 -10m（20s）
    for _ in 0..4000 {
        step_once!();
    }

    // 2) 施加垂直下洗风 wind_z=1.5 m/s（NED 向下 -> 恒定向下阻力），10s
    phys.set_wind([0.0, 0.0, 1.5]);
    let mut z_min = f32::MAX;
    let mut z_max = f32::MIN;
    let mut ss_sum = 0.0f32; // 稳态段（扰动最后 3s）z 累计
    let mut ss_n = 0u32;
    for sec in 0..10u32 {
        for _ in 0..200 {
            step_once!();
        }
        if z.is_finite() {
            z_min = z_min.min(z);
            z_max = z_max.max(z);
        }
        if sec >= 7 {
            ss_sum += z;
            ss_n += 1;
        }
        if sec % 2 == 1 {
            println!("  下洗 t={:>3}s z={:>7.3}m", 10 + sec, z);
        }
    }
    let ss_mean = ss_sum / ss_n as f32;
    println!(
        "下洗扰动: z 峰值∈[{:.3},{:.3}] Δz={:.3}m | 稳态段均值 z={:.3}m（相对 -10 偏差 {:.3}m）",
        z_min, z_max, z_max - z_min, ss_mean, ss_mean - (-10.0)
    );

    // 3) 撤风，记录恢复时间（z 回到 |z+10|<0.3m 后视为恢复）
    phys.set_wind([0.0, 0.0, 0.0]);
    let mut rec_time = f32::NAN;
    let mut last = -10.0f32;
    for sec in 0..10u32 {
        for _ in 0..200 {
            step_once!();
        }
        last = z;
        if sec % 2 == 1 {
            println!("  撤风 t={:>3}s z={:>7.3}m", 20 + sec, z);
        }
        if !rec_time.is_finite() && (z - (-10.0)).abs() < 0.3 {
            rec_time = sec as f32 + 1.0;
        }
    }
    println!(
        "撤风后: 10s 末 z={:.3}m | 恢复(Δ<0.3m) t_rec={:.1}s",
        last, rec_time
    );
}

#[test]
fn vertical_margin_probe() {
    // 垂直环频域裕度探针：闭环扫频（sp_d=-10+A·cos），单频 DFT 重建开环传递 L=Z/E，
    // 提取增益裕度(GM)与相位裕度(PM)。裕度是线性系统特性，与噪声无关，故去掉传感器噪声。
    use flyctrl_core::controller::{Controller, Setpoint};
    use flyctrl_core::estimator::Estimator;
    use flyctrl_core::units::{Meter, Radian};
    use flyctrl_core::vehicle::ActuatorCmd;
    use std::f32::consts::TAU;
    let dt = Second(0.005);
    let fs = 200.0f32;
    let wp = WorldParams {
        accel_noise: 0.0,
        gyro_noise: 0.0,
        pos_noise: 0.0,
        ..WorldParams::default()
    };
    let mut phys = Physics::new(PhysicsParams::default());
    let mut world = World::new(wp);
    let mut est = EkfEstimator::default_quad();
    let mut ctrl = PidController::default_quad();
    let mut cmd = ActuatorCmd::zero();
    let mut sp = Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian(0.0));

    // 1) 下潜并稳定到 -10m
    for _ in 0..3000 {
        let ideal = phys.step(dt, cmd);
        let s = phys.state();
        let (imu, gps) = world.sense(dt, ideal, s.pos);
        let e = est.step(dt, imu, gps, None);
        cmd = ctrl.control(dt, &sp, &e);
    }

    // 2) 扫频：每个频点跑 8 个周期、取后 5 个周期做单频 DFT（相干累加）。
    let amp = 0.05f32;
    let mut freqs: Vec<f32> = Vec::new();
    let mut f = 0.06f32;
    while f <= 2.5f32 {
        freqs.push(f);
        f *= 1.25;
    }
    // 单频 DFT（e^{+jωt} 正频约定）：y=B·cos(ωt+φ) -> 幅相 (B·cosφ, B·sinφ)
    let dft_bin = |samples: &[f32], freq: f32| -> (f32, f32) {
        let w = TAU * freq / fs;
        let mut re = 0.0f32;
        let mut im = 0.0f32;
        for (i, &y) in samples.iter().enumerate() {
            let ph = w * i as f32;
            re += y * ph.cos();
            im -= y * ph.sin();
        }
        let n = samples.len() as f32;
        (2.0 * re / n, 2.0 * im / n)
    };

    println!("\n=== 垂直环开环 Bode（实测闭环扫频，Z=filt_d 反馈点）===");
    println!("{:<8} {:>9} {:>10} | {:<6} {:>9} {:>10}", "f(Hz)", "|L|(dB)", "∠L(°)", "", "", "");
    let mut rows: Vec<(f32, f32, f32)> = Vec::new(); // (f, gain_db, phase_deg)
    for &freq in &freqs {
        let n_total = (8.0 * fs / freq) as usize;
        let n_warm = n_total - (5.0 * fs / freq) as usize;
        let mut e_samples: Vec<f32> = Vec::with_capacity(n_total - n_warm);
        let mut z_samples: Vec<f32> = Vec::with_capacity(n_total - n_warm);
        let w = TAU * freq;
        for i in 0..n_total {
            sp.pos[2] = Meter(-10.0 + amp * (w * i as f32 * dt.0).cos());
            let ideal = phys.step(dt, cmd);
            let s = phys.state();
            let (imu, gps) = world.sense(dt, ideal, s.pos);
            let e = est.step(dt, imu, gps, None);
            cmd = ctrl.control(dt, &sp, &e);
            if i >= n_warm {
                let dbg = ctrl.debug_pid_internal();
                e_samples.push(dbg.4); // ez = sp_d - est_d（误差，环路输入侧）
                z_samples.push(dbg.2); // filt_d = est_d（反馈量，环路输出侧）
            }
        }
        let (z_re, z_im) = dft_bin(&z_samples, freq);
        let (e_re, e_im) = dft_bin(&e_samples, freq);
        let mag_z = (z_re * z_re + z_im * z_im).sqrt();
        let mag_e = (e_re * e_re + e_im * e_im).sqrt();
        let gain_db = if mag_e > 1e-9 {
            20.0 * (mag_z / mag_e).log10()
        } else {
            f32::NAN
        };
        let phase = (z_im.atan2(z_re) - e_im.atan2(e_re)).to_degrees();
        let phase = ((phase + 180.0).rem_euclid(360.0)) - 180.0; // 折叠到 (-180,180]
        rows.push((freq, gain_db, phase));
        println!("{:<8.3} {:>9.2} {:>10.1}", freq, gain_db, phase);
        // 回到 -10 纯悬停 0.5s，保证下一频点起点一致
        sp.pos[2] = Meter(-10.0);
        for _ in 0..100 {
            let ideal = phys.step(dt, cmd);
            let s = phys.state();
            let (imu, gps) = world.sense(dt, ideal, s.pos);
            let e = est.step(dt, imu, gps, None);
            cmd = ctrl.control(dt, &sp, &e);
        }
    }

    // 3) 提取裕度：增益穿越处插值相位 -> PM；相位穿越处插值增益 -> GM
    let interp_x = |f1: f32, y1: f32, f2: f32, y2: f32, yt: f32| -> f32 {
        let l1 = f1.log10();
        let l2 = f2.log10();
        10f32.powf(l1 + (yt - y1) / (y2 - y1) * (l2 - l1))
    };
    let mut pm = f32::NAN;
    let mut fc = f32::NAN;
    for i in 0..rows.len() - 1 {
        let (f1, g1, _) = rows[i];
        let (f2, g2, p2) = rows[i + 1];
        if g1.is_finite() && g2.is_finite() && g1 > 0.0 && g2 <= 0.0 {
            fc = interp_x(f1, g1, f2, g2, 0.0);
            let t = (0.0 - g1) / (g2 - g1);
            pm = 180.0 + (rows[i].2 + t * (p2 - rows[i].2));
            break;
        }
    }
    let mut gm = f32::INFINITY;
    let mut fp = f32::NAN;
    for i in 0..rows.len() - 1 {
        let (f1, g1, p1) = rows[i];
        let (f2, g2, p2) = rows[i + 1];
        if p1.is_finite() && p2.is_finite() && p1 < -180.0 && p2 >= -180.0 {
            fp = interp_x(f1, p1, f2, p2, -180.0);
            let t = (-180.0 - p1) / (p2 - p1);
            gm = -(g1 + t * (g2 - g1)); // 增益裕度 = -|L|@-180°（dB）
            break;
        }
    }
    println!(
        "\n增益穿越 fc={:.3} Hz -> 相位裕度 PM = {:.1}°\n相位穿越 fp={:.3} Hz -> 增益裕度 GM = {:.1} dB",
        fc, pm, fp, gm
    );
}

#[test]
fn open_loop_torque_sign_probe() {
    // 开环探针：固定悬停油门 + 单一力矩偏置，测量物理角速度加速度方向。
    // 验证控制器混控的 roll/pitch/yaw 力矩符号是否与物理模型一致。
    use flyctrl_core::vehicle::ActuatorCmd;
    let dt = Second(0.005);
    // 每个探针：基准 0.5，注入 Δmotor，跑 1s 看角速度变化。
    let probes: [(&str, [f32; 4]); 3] = [
        // 偏航：f0+f1 增（CCW 高）→ m_yaw>0，期望 +ωz（若 yaw 符号约定为右旋+）
        ("yaw(CCW up)     ", [0.6, 0.6, 0.4, 0.4]),
        // 俯仰：f0+f2 增（前高）→ m_pitch>0，期望 +ωy（机头上仰）
        ("pitch(front up) ", [0.6, 0.4, 0.6, 0.4]),
        // 横滚：f0+f3 增（右高）→ m_roll>0，期望 +ωx（右滚）
        ("roll(right up)  ", [0.6, 0.4, 0.4, 0.6]),
    ];
    for (name, m) in probes {
        let mut phys = Physics::new(PhysicsParams::default());
        let cmd = ActuatorCmd { motor: m };
        let mut w = [0.0f32; 3];
        for _ in 0..200 {
            let _ = phys.step(dt, cmd);
            w = [
                phys.state().omega[0].0,
                phys.state().omega[1].0,
                phys.state().omega[2].0,
            ];
        }
        println!(
            "{:<18} -> omega@1s=({:+.3}, {:+.3}, {:+.3}) rad/s",
            name, w[0], w[1], w[2]
        );
    }
}
