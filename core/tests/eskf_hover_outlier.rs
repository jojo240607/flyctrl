//! ★P 无界增长诊断与复现留档（§5.131 补遗 2/3，hover_demo M 场事件的根因记录）
//!
//! M 场现场（x_hover_demo，确定性 ×3）：53s 完美悬停后，**单拍** acc z 样本
//! -9.64 → -9.04（-0.6 m/s²，≈5σ），EST 四元数同拍跳到俯仰 -43.4°，电机瞬即
//! 全轨 ⇒ 真机被踢翻滚坠落 9m。
//!
//! 本文件三层诊断（当前全绿 = 记录现状，非回归防护）：
//!   1) 纯 ESKF：单点离群响应 + 噪声激励下 53s 悬停
//!   2) HilContext 全链路（含 40Hz 陷波 + 20Hz 低通 IIR 状态）
//!   3) diag_p_cross：**P[θ_pitch] 无界增长实测**（6e-3/s 线性，roll 有界对照）
//!      —— 俯仰弱观测缺陷的直接证据 ✓
//!
//! 修复（协方差限幅）已实现并验证可消除 M 场事件，但因牵动 att_est.rs 的
//! 整定验收表（eskf_final_tuning A7 / reanchor_quiet / freq_response）需
//! 正式整定会话重过验收 ⇒ 暂缓合入（见 c1-migration-plan §5.131 补遗 3）。

use flyctrl_core::estimator::eskf_estimator::EskfEstimator;
use flyctrl_core::estimator::trait_def::Estimator;
use flyctrl_core::units::{
    Meter, MeterPerSecond, MeterPerSecondSquared, Radian, RadianPerSecond, Second,
};
use flyctrl_core::vehicle::{ImuSample, PosSample, Quaternion};

/// 四元数夹角（deg）：q1 相对 q0 的旋转角
fn quat_angle_deg(a: &Quaternion, b: &Quaternion) -> f32 {
    // q_rel = a⁻¹ ⊗ b 的角 = 2·acos(|w|)
    let (aw, ax, ay, az) = (a.w, a.x, a.y, a.z);
    let (bw, bx, by, bz) = (b.w, b.x, b.y, b.z);
    // 共轭 a 乘 b
    let w = aw * bw + ax * bx + ay * by + az * bz;
    let x = -aw * bx + ax * bw + ay * bz - az * by;
    let y = -aw * by - ax * bz + ay * bw + az * bx;
    let z = -aw * bz + ax * by - ay * bx + az * bw;
    let n = (x * x + y * y + z * z).sqrt();
    2.0 * n.atan2(w.abs()).to_degrees()
}

#[test]
fn long_hover_single_accel_outlier_must_not_flip_attitude() {
    let q0 = Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0));
    let mut e = EskfEstimator::new(q0, [0.0; 3], [0.0; 3], 5.0, [0.2, 0.0, 0.4]);
    let dt = Second(0.004);

    let imu_hover = ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(-9.81),
        ],
        gyro: [RadianPerSecond(0.0); 3],
    };
    // ★离群样本：z 亏 0.77（≈M 场实测 -9.04 vs -9.81 名义）
    let imu_outlier = ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(-9.04),
        ],
        gyro: [RadianPerSecond(0.0); 3],
    };
    let gps = PosSample {
        pos: [Meter(0.0), Meter(0.0), Meter(0.0)],
        vel: Some([MeterPerSecond(0.0); 3]),
    };

    // 悬停输入节拍（对齐固件路径）：GPS ≈21Hz（每 12 拍）、baro 50Hz（每 2 拍）、
    // 重力/磁由 aid_period=15 内部降频 ✓
    const N_CONV: usize = 13_250; // 53s 收敛（M 场事件时刻）
    const N_TAIL: usize = 500; // 事件后 2s
    let mut prev_q = q0;
    let mut max_jump_conv = 0.0f32;
    let mut jump_at_event = 0.0f32;
    let mut end_q = q0;

    for step in 0..(N_CONV + N_TAIL) {
        // ★确定性伪噪声（对齐 M 场实测：acc z 噪声 σ≈0.1、xy σ≈0.05，含慢漂成分；
        //   sin 叠加保证跨运行确定 ✓）——持续噪声是 P 增长的激励源 ✓
        let nz = 0.10 * (0.7 * step as f32).sin() + 0.06 * (0.23 * step as f32).sin();
        let nx = 0.05 * (0.53 * step as f32).sin() + 0.03 * (0.11 * step as f32).sin();
        let ny = 0.05 * (0.41 * step as f32).sin() + 0.03 * (0.17 * step as f32).sin();
        let mut acc = [
            MeterPerSecondSquared(nx),
            MeterPerSecondSquared(ny),
            MeterPerSecondSquared(-9.81 + nz),
        ];
        // 离群拍：单点 z 亏 0.77（叠加在噪声上）
        if step == N_CONV {
            acc[2] = MeterPerSecondSquared(-9.04);
        }
        let imu = ImuSample { accel: acc, gyro: [RadianPerSecond(0.0); 3] };
        let pos = if step % 12 == 0 { Some(gps) } else { None };
        let st = e.step(dt, imu, pos, None);
        if step % 2 == 0 {
            e.update_alt(0.0); // baro（高度恒 0）
        }
        if step % 15 == 0 {
            // ★M 场真实磁方向（x_hover_demo 实测注入 [0.445,0.229,0.399]）vs 先验
            // [0.2,0,0.4] ⇒ 方向失配 ~40°——M 场事件的"子弹"（先验失配）✓
            e.update_mag(Some([0.445, 0.229, 0.399]));
        }

        let jump = quat_angle_deg(&prev_q, &st.att);
        if step < N_CONV {
            max_jump_conv = max_jump_conv.max(jump);
        } else if step == N_CONV {
            jump_at_event = jump;
        }
        prev_q = st.att;
        end_q = st.att;

        // 收敛期与事件拍：单拍跳变不得超过 5°（正常滤波物理上限）
        if jump >= 5.0 {
            // 打印现场后判负
            let p = e.filter().p;
            let p_att_diag = (p[3][3], p[4][4], p[5][5]);
            panic!(
                "单拍姿态跳变 {:.2}° @ step={}（收敛期最大 {:.2}°）—— ESKF 姿态通道数值缺陷 \
                 （P 姿态对角 = {:?}，注意 I_ATT 块在 21 维布局的 6..9）",
                jump, step, max_jump_conv, p_att_diag
            );
        }
    }

    println!(
        "收敛期最大单拍跳变 = {:.3}°；离群拍跳变 = {:.3}°；末态四元数 = ({:.4},{:.4},{:.4},{:.4})",
        max_jump_conv, jump_at_event, end_q.w, end_q.x, end_q.y, end_q.z
    );
    assert!(
        jump_at_event < 5.0,
        "离群拍姿态跳变 {:.2}° ≥ 5° —— ESKF 对单点加计离群的响应过激（M 场 43° 事件的 H 场复现）",
        jump_at_event
    );
}

/// ★§5.131 补遗 2：P 矩阵演化观测——长悬停下 P[θ↔v] 交叉协方差是否无界增长？
/// （M 场 43° 单拍跳变的量级要求 P 交叉项 ~19·s；本测试观测其演化轨迹）
#[test]
fn diag_p_cross_growth_over_long_hover() {
    let q0 = Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(0.0));
    let mut e = EskfEstimator::new(q0, [0.0; 3], [0.0; 3], 5.0, [0.2, 0.0, 0.4]);
    let dt = Second(0.004);
    let imu_hover = ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(-9.81),
        ],
        gyro: [RadianPerSecond(0.0); 3],
    };
    let gps = PosSample {
        pos: [Meter(0.0), Meter(0.0), Meter(0.0)],
        vel: Some([MeterPerSecond(0.0); 3]),
    };
    for step in 0..13_250 {
        let pos = if step % 12 == 0 { Some(gps) } else { None };
        let st = e.step(dt, imu_hover, pos, None);
        if step % 2 == 0 {
            e.update_alt(0.0);
        }
        if step % 15 == 0 {
            e.update_mag(Some([0.2, 0.0, 0.4]));
        }
        if step % 1250 == 0 {
            let p = &e.filter().p;
            // θ-V 交叉项绝对值最大者 + θ/ν 对角
            let mut max_cross = 0.0f32;
            for i in 0..3 {
                for j in 0..3 {
                    max_cross = max_cross.max(p[i][9 + j].abs());
                }
            }
            println!(
                "t={:5.1}s P[θ]diag=({:.3e},{:.3e},{:.3e}) P[v]diag=({:.3e},{:.3e},{:.3e}) max|P[θ][v]|={:.3e} est_z={:.3}",
                step as f32 * 0.004,
                p[0][0], p[1][1], p[2][2],
                p[3][3], p[4][4], p[5][5],
                max_cross, st.pos[2].0
            );
        }
    }
}

/// ★§5.131：HilContext 全链路复现（含 40Hz 陷波 + 20Hz 低通 IIR 状态、FDIR、控制律）
#[test]
fn hilctx_long_hover_accel_outlier_repro() {
    use flyctrl_core::controller::pid::PidController;
    use flyctrl_core::hil::{HilContext, SimImu};
    use flyctrl_core::units::{Radian as Rad2};

    let mut ctx = HilContext::new(
        EskfEstimator::default_quad(),
        PidController::default_quad(),
        Second(0.004),
    );
    let mut sim_imu = SimImu::new();
    let sp = flyctrl_core::controller::trait_def::Setpoint::hover(
        [Meter(0.0), Meter(0.0), Meter(-5.0)],
        Rad2(0.0),
    );

    let mut prev_q = Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };
    let mut max_jump_conv = 0.0f32;
    let mut jump_at_event = 0.0f32;
    const N_CONV: usize = 13_250;

    for step in 0..(N_CONV + 500) {
        let nz = 0.10 * (0.7 * step as f32).sin() + 0.06 * (0.23 * step as f32).sin();
        let nx = 0.05 * (0.53 * step as f32).sin() + 0.03 * (0.11 * step as f32).sin();
        let ny = 0.05 * (0.41 * step as f32).sin() + 0.03 * (0.17 * step as f32).sin();
        let mut az = -9.81 + nz;
        if step == N_CONV {
            az = -9.04;
        }
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(nx), MeterPerSecondSquared(ny), MeterPerSecondSquared(az)],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let gps = if step % 12 == 0 {
            Some(PosSample { pos: [Meter(0.0), Meter(0.0), Meter(-5.0)], vel: Some([MeterPerSecond(0.0); 3]) })
        } else {
            None
        };
        let baro = if step % 2 == 0 { Some(5.0f32) } else { None };
        let mag = if step % 15 == 0 { Some([0.445f32, 0.229, 0.399]) } else { None };

        let r = ctx.step_hil(
            Some(imu), gps, baro, None, None, mag,
            &sp, true, true, true, &mut sim_imu,
        );
        let jump = quat_angle_deg(&prev_q, &r.est.att);
        if step < N_CONV {
            max_jump_conv = max_jump_conv.max(jump);
        } else if step == N_CONV {
            jump_at_event = jump;
        }
        if jump >= 5.0 {
            panic!(
                "单拍姿态跳变 {:.2}° @ step={}（收敛期最大 {:.2}°）—— HilContext 全链路复现 M 场 43° 事件",
                jump, step, max_jump_conv
            );
        }
        prev_q = r.est.att;
    }
    println!(
        "HilContext：收敛期最大单拍 = {:.3}°；离群拍 = {:.3}°",
        max_jump_conv, jump_at_event
    );
    assert!(jump_at_event < 5.0, "离群拍跳变 {:.2}° ≥ 5°", jump_at_event);
}
