//! fcalg 桥接验收（feature `fcalg-est`）—— **通过 `Estimator` trait 驱动**，证明接线可用。
//!
//! 口径 (b)：只要求桥接正确 + 新栈满足自身契约，**不与旧栈逐位等价**。
#![cfg(feature = "fcalg-est")]

use std::sync::atomic::Ordering;

use flyctrl_core::estimator::fcalg_bridge::{FcalgEstimator, NOT_IMPLEMENTED_CALLS};
use flyctrl_core::estimator::trait_def::Estimator;
use flyctrl_core::units::{
    Meter, MeterPerSecond, MeterPerSecondSquared, RadianPerSecond, Second,
};
use flyctrl_core::vehicle::{ImuSample, PosSample, Quaternion};

const G: f32 = 9.806_65;

/// 静态悬停（真值恒停）下，经 trait 驱动必须收敛且不发散。
/// 比力取 `(0,0,−g)`（契约：静止水平时比力指向天，机体系 z 向下）。
#[test]
fn bridge_converges_on_static_hover() {
    // ★增量式快照：`NOT_IMPLEMENTED_CALLS` 是**进程级全局**，而同一测试二进制的用例
    //   **默认并行** —— 断言绝对值会被相邻用例污染（本会话第二次栽在这个模式上：
    //   上次是 L1 的 finite 计数器）。故只断言"本用例"没有增加它 ⇒ 并行安全。
    // 按**实例**断言（并行安全）：全局静态会被相邻用例并发增加，增量法也挡不住。
    let mut e = FcalgEstimator::new();
    e.set_initial_attitude(Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    e.set_initial_position([0.0; 3]);

    let dt = 0.005f32;
    let acc = [0.0f32, 0.0, -G];
    for k in 0..600 {
        // 逐样本 predict（固件环形路径）
        e.predict_delta([0.0; 3], [acc[0] * dt, acc[1] * dt, acc[2] * dt], dt, dt);
        // 气压：局部高度 0（契约 alt == −p_z）
        e.update_alt(0.0);
        if k % 10 == 0 {
            // GPS 位置 + 速度（`vel` 是 Option ⇒ 桥接分两路）
            e.update_fusion(
                Some(PosSample { pos: [Meter(0.0); 3], vel: Some([MeterPerSecond(0.0); 3]) }),
                None,
            );
            e.update_mag(None); // 无磁 ⇒ 不得有副作用
        }
    }
    // 单帧路径也过一遍（`step` 是另一条入口，必须同样可用）
    let imu = ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(-G),
        ],
        gyro: [RadianPerSecond(0.0); 3],
    };
    let s = e.step(Second(dt), imu, None, None);

    assert!(s.att.w.abs() > 0.999, "静态悬停下姿态应≈单位四元数: {:?}", s.att);
    eprintln!(
        "[hov] 终态 pos=({:.4},{:.4},{:.4}) vel=({:.4},{:.4},{:.4}) qw={:.6} rej={} ba=({:.5},{:.5},{:.5})",
        s.pos[0].0, s.pos[1].0, s.pos[2].0, s.vel[0].0, s.vel[1].0, s.vel[2].0, s.att.w,
        e.fuse_rejects, s.accel_bias[0], s.accel_bias[1], s.accel_bias[2]
    );
    // 界**有依据**：GPS 位置 σ=0.5 m（参数表 obs.sigma_gps_p）⇒ 取 2σ = 1.0 m。
    // 实测终值为 0.0000（见上面的 [hov] 行），远在界内 —— 这个界不是"调大到绿"。
    for k in 0..3 {
        assert!(s.pos[k].0.abs() < 1.0, "位置须在 2σ(GPS) 内: {:?}", s.pos);
        assert!(s.vel[k].0.abs() < 1.0, "速度须在 2σ 内: {:?}", s.vel);
    }
    assert!(s.accel_bias.iter().all(|x| x.is_finite()));
    assert!(
        e.accel_bias().iter().all(|x| x.is_finite()),
        "accel_bias 通路必须可用"
    );
    assert_eq!(e.not_impl_calls, 0, "本用例只走已实现通路 ⇒ 实例计数必须为 0");
}

/// **"绝不静默"在接线层的判据**：三处"无对应"被调用时必须**计数**，
/// 而不是悄悄 no-op（否则将来会有人以为群延迟补偿/VIO/RTK 在起作用）。
#[test]
fn unimplemented_entry_points_are_counted_not_silent() {
    let mut e = FcalgEstimator::new();
    e.set_accel_lag_s(0.015);
    e.update_vio(None);
    e.update_rtk(None);
    assert_eq!(e.not_impl_calls, 3, "三处无对应必须各计一次（绝不静默 no-op）");
    // 全局静态只作聚合，**不断言其绝对值**（并行下不可靠 —— 本会话两次教训）
    assert!(NOT_IMPLEMENTED_CALLS.load(Ordering::Relaxed) >= 3);
}

/// `reset` 必须回到可用初态（接线后会被调用，不得留下脏状态）。
#[test]
fn reset_returns_to_usable_initial_state() {
    let mut e = FcalgEstimator::new();
    e.set_initial_position([100.0, -50.0, 20.0]);
    e.reset();
    let s = e.state();
    for k in 0..3 {
        assert_eq!(s.pos[k].0, 0.0, "reset 后位置必须归零");
    }
    assert!(s.att.w.abs() > 0.999, "reset 后姿态必须为单位");
}

/// ★**闭合判据**：桥接的诊断计数器必须与**实际调用次数**一致
/// （app 侧的诊断日志读的正是这些数 —— 漏计会静默地误导）。
#[test]
fn diagnostic_counters_close_with_actual_calls() {
    let mut e = FcalgEstimator::new();
    e.set_initial_attitude(Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    let dt = 0.005f32;
    let f_b = [0.0f32, 0.0, -G];
    let n_pred = 100;
    let n_fuse = 10;
    for k in 0..n_pred {
        e.predict_delta([0.0; 3], [f_b[0] * dt, f_b[1] * dt, f_b[2] * dt], dt, dt);
        if k % (n_pred / n_fuse) == 0 {
            e.update_fusion(
                Some(PosSample { pos: [Meter(0.0); 3], vel: Some([MeterPerSecond(0.0); 3]) }),
                None,
            );
        }
    }
    assert_eq!(e.n_step, n_pred as u64, "步数计数必须闭合");
    assert_eq!(e.n_gps_pos, n_fuse, "GPS 位接受数必须闭合");
    assert_eq!(e.n_gps_vel, n_fuse, "GPS 速接受数必须闭合");
    assert_eq!(e.n_gps_pos_rejected, 0);
    assert_eq!(e.n_gps_vel_rejected, 0);
    // 重力：每拍都试 ⇒ 应用 + 门控必须恰好等于步数
    assert_eq!(
        e.n_grav_applied + e.n_grav_gated,
        n_pred as u32,
        "重力路每拍必计数（applied={} gated={}）",
        e.n_grav_applied,
        e.n_grav_gated
    );
    assert!(e.n_grav_applied > 0, "静止悬停的比力应通过量级门");
    // 无对应项必须恒 0（登记为"不假装有数"）
    assert_eq!(e.n_mag_reanchored, 0);
    // 磁路未喂 ⇒ 两计数都应为 0（不得凭空增加）
    assert_eq!((e.n_mag, e.n_mag_rejected), (0, 0));

    // 喂一个荒谬 GPS ⇒ 拒收计数必须 +1，而接受数不变（闭合仍然成立）
    let before = (e.n_gps_pos, e.n_gps_pos_rejected);
    let bogus = PosSample {
        pos: [Meter(1e5), Meter(-1e5), Meter(1e5)],
        vel: None,
    };
    e.update_fusion(Some(bogus), None);
    assert_eq!(e.n_gps_pos, before.0, "被拒不得计入接受数");
    assert_eq!(e.n_gps_pos_rejected, before.1 + 1, "被拒必须计入拒收数");
}

/// ★`world_accel()` 的**定义性判据**：静止悬停（水平 + 比力 (0,0,−g)）时世界系加速度必须 ≈ 0
/// （推力恰好抵消重力）。这也是它当初存在的理由：给 HIL 诊断一个"净加速度"读数。
#[test]
fn world_accel_is_zero_at_static_hover() {
    let mut e = FcalgEstimator::new();
    e.set_initial_attitude(Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 });
    let dt = 0.005f32;
    let f_b = [0.0f32, 0.0, -G];
    for _ in 0..50 {
        e.predict_delta([0.0; 3], [f_b[0] * dt, f_b[1] * dt, f_b[2] * dt], dt, dt);
    }
    let wa = e.world_accel();
    for a in 0..3 {
        assert!(
            wa[a].abs() < 0.05,
            "静止悬停下世界系加速度应 ≈ 0: {wa:?}（推力未抵消重力？）"
        );
    }
    // 对照：比力**大于** G（推得比悬停更用力）⇒ 净加速度**向上**（NED 里 z 为**负**）。
    // ★首版写成 `-G + 1.0`（= −8.8，**比悬停更轻**）却断言向上 ✗ —— 模块给的 +1.0
    //   才是对的（推得轻 ⇒ 向下加速）。**又一次是我的直觉错、模块对**（同 NED 的 p_z 那次）。
    let f_up = [0.0f32, 0.0, -G - 1.0];
    e.predict_delta([0.0; 3], [f_up[0] * dt, f_up[1] * dt, f_up[2] * dt], dt, dt);
    let wa2 = e.world_accel();
    assert!(wa2[2] < -0.5, "净上行比力应给出向上的（NED −z）加速度: {wa2:?}");
}

/// ★装配层判据（2026-10-05）：**水平静止 400 步后姿态必须仍是水平**。
///
/// 动因：真链路里 `est_layout_probe` 报 `q.w = −0.664`（≈150° 倾角）✗，
/// 而模块层 131 条判据全绿 ✓ —— 说明缺口在**装配层**（桥接喂什么、按什么次序喂）。
/// 本判据直接用**桥接**（不是裸 `Eskf`）跑与仿真同口径的输入：
///   · 水平静止 ⇒ `accel = (0,0,−9.81)` ✓（与 `hil.rs:61` 同口径 ✓）、陀螺全 0 ✓
///   · 步长 13 ms（工具链真链路节奏 ✓）、400 步 ✓
///   · 只喂 IMU（`pos = None`）⇒ 只考验 **predict + 重力观测** 这条最小闭环 ✓
///
/// 若本判据在 host 上失败 ⇒ 根因在**桥接/fcalg 路径**，且可在 0.02 s 内复现 ✓✓；
/// 若通过 ⇒ 嫌疑转向 `step` 之外的融合路径（磁/位置/GPS ✓）。
#[test]
fn bridge_level_static_keeps_attitude() {
    use flyctrl_core::estimator::fcalg_bridge::FcalgEstimator;
    use flyctrl_core::estimator::trait_def::Estimator;
    use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond, Second};
    use flyctrl_core::vehicle::ImuSample;

    let mut est = FcalgEstimator::new();
    let dt = Second(0.013);
    let mut worst_abs_w = 1.0f32;
    let mut first_bad = None;
    for k in 0..400 {
        let imu = ImuSample {
            accel: [
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(-9.81),
            ],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let st = est.step(dt, imu, None, None);
        // 水平静止 ⇒ 单位四元数 ±q 都合法 ⇒ 比 |w| ✓
        let aw = st.att.w.abs();
        if aw < 0.9 && first_bad.is_none() {
            first_bad = Some((k, st.att.w, st.att.x, st.att.y, st.att.z));
        }
        worst_abs_w = worst_abs_w.min(aw);
    }
    assert!(
        worst_abs_w > 0.9,
        "水平静止 400 步后姿态必须仍水平：worst |w| = {worst_abs_w}；首次越界 = {first_bad:?}"
    );
}

/// ★装配层判据 #2（2026-10-05）：**加磁观测**后水平静止仍须水平。
///
/// 动因（嫌疑 #1）：真链路 `q.w = −0.664`（≈150°）✗，而判据 #1 已排除 predict+重力 ✓
/// ⇒ 嫌疑转向**磁观测路径**（`update_mag` → `mag_yaw`）。依据：L9b 是 yaw-only，
/// 但 **roll/pitch 有物理耦合（H 为精确导数）** ✓；且固件路径**无人设置**
/// `set_mag_ref`/`set_mag_bias` ✗ ⇒ 未标定的参考可经耦合污染倾角 ✓
///
/// 做法：用**桥接自己的磁参考** `mag_i()` 当"水平姿态下应有的机体测量" ✓
///（`R = I` ⇒ `mag_body = mag_i` ⇒ 创新应为 0 ⇒ 正确行为是**姿态不动** ✓）。
/// 若姿态被拉走 ⇒ 磁路径有缺陷 ✓；若不动 ⇒ 嫌疑 #1 排除，转向位置/GPS 路径 ✓。
#[test]
fn bridge_level_static_with_mag_keeps_attitude() {
    use flyctrl_core::estimator::fcalg_bridge::FcalgEstimator;
    use flyctrl_core::estimator::trait_def::*;
    use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond, Second};
    use flyctrl_core::vehicle::ImuSample;

    let mut est = FcalgEstimator::new();
    // ★★仪器修正（2026-10-05）：原来喂 `est.mag_i()` 当测量 ⇒ **而它就是零矢量** ✗
    //   ⇒ 零测量被门掉 ⇒ 磁路径**根本没被测到**，"通过"是**空转** ✗（绿灯即噪声 ✗）。
    //   改喂**真实非零磁场**（对应仿真 `hil.rs:635` 喂真磁场 ✓）。
    let m = [0.45f32, 0.0, -0.28];
    let dt = Second(0.013);
    let mut worst_abs_w = 1.0f32;
    let mut first_bad = None;
    for k in 0..400 {
        let imu = ImuSample {
            accel: [
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(-9.81),
            ],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let st = est.step(dt, imu, None, None);
        est.update_mag(Some(m));
        let st2 = est.state();
        let aw = st2.att.w.abs();
        let _ = st;
        if aw < 0.9 && first_bad.is_none() {
            first_bad = Some((k, st2.att.w, st2.att.x, st2.att.y, st2.att.z));
        }
        worst_abs_w = worst_abs_w.min(aw);
    }
    // ★仪器可信性断言（**这条才是关键**）：必须证明磁路径**确实被走到** ✓ ——
    //   未配置参考 ⇒ 按契约 §4 必须**显式拒绝并计数**（而不是静默漂走 ✗）。
    assert!(
        est.n_mag_ref_missing > 0,
        "磁参考未配置（mag_i 为零矢量）⇒ 必须【显式拒绝并计数】；         若该计数为 0 则本判据**空转** ✗（磁路径没被测到）。n_mag={} n_mag_rejected={}",
        est.n_mag,
        est.n_mag_rejected
    );
    assert!(
        worst_abs_w > 0.9,
        "加磁观测后水平静止 400 步仍须水平：worst |w| = {worst_abs_w}；首次越界 = {first_bad:?}；         n_mag_ref_missing={} n_mag={}",
        est.n_mag_ref_missing,
        est.n_mag
    );
}

/// ★装配层判据 #3（2026-10-05）：**加位置/GPS 观测**后水平静止仍须水平。
///
/// 动因（嫌疑 #1 排除后的剩余项）：真链路 `q.w = −0.664`（≈150°）✗，
/// 而判据 #1（predict+重力 ✓）、#2（磁 ✓）均绿 ⇒ 覆盖缺口只剩**位置/GPS 路径** ✗。
/// 做法：`step()`（水平静止 IMU）+ `update_fusion(Some(pos_only(0,0,0)), None)`
/// （静止悬停 ⇒ 位置观测应为 (0,0,0) ⇒ 创新为 0 ⇒ 姿态应不动 ✓），400 步 ✓。
/// **附带断言**：位置观测必须**真的被接受**（否则本判据没测到东西 ✗ —— 仪器自身要可信 ✓）。
#[test]
fn bridge_level_static_with_pos_keeps_attitude() {
    use flyctrl_core::estimator::fcalg_bridge::FcalgEstimator;
    use flyctrl_core::estimator::trait_def::*;
    use flyctrl_core::units::{Meter, MeterPerSecondSquared, RadianPerSecond, Second};
    use flyctrl_core::vehicle::{ImuSample, PosSample};

    let mut est = FcalgEstimator::new();
    let dt = Second(0.013);
    let mut worst_abs_w = 1.0f32;
    let mut first_bad = None;
    for k in 0..400 {
        let imu = ImuSample {
            accel: [
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(-9.81),
            ],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let _ = est.step(dt, imu, None, None);
        est.update_fusion(Some(PosSample::pos_only([Meter(0.0); 3])), None);
        let st2 = est.state();
        let aw = st2.att.w.abs();
        if aw < 0.9 && first_bad.is_none() {
            first_bad = Some((k, st2.att.w, st2.att.x, st2.att.y, st2.att.z));
        }
        worst_abs_w = worst_abs_w.min(aw);
    }
    // 仪器可信性：位置观测确实被采用过（否则本判据是空转 ✗）
    assert!(
        est.n_gps_pos > 0,
        "位置观测必须真的被接受（n_gps_pos={}，n_gps_pos_rejected={}）—— 否则本判据未测到东西 ✗",
        est.n_gps_pos,
        est.n_gps_pos_rejected
    );
    assert!(
        worst_abs_w > 0.9,
        "加位置观测后水平静止 400 步仍须水平：worst |w| = {worst_abs_w}；首次越界 = {first_bad:?}；\
         n_gps_pos={} n_gps_pos_rejected={}",
        est.n_gps_pos,
        est.n_gps_pos_rejected
    );
}

/// ★偏航漂移探针（2026-10-05）：**对齐固件的更新次数**（250 Hz × 5.2 s ≈ 1300 次）。
///
/// 动因（真链路实测）：真值 `TRUTH att(rpy) = [0,0,0]`（完全水平、航向 0 ✓）而
/// 估计为 `wxyz = [0.5654, 0.0013, 0.1109, −0.8173]` ⇒ **纯偏航 ~111°** ✗ ✓。
/// 而既有判据 #1 只跑 **400 次** update（13 ms/步），固件跑 **~1300 次**（250 Hz）
/// ⇒ 同一总时长、更新次数差 3.25× ⇒ 若存在"每次更新推一点偏航"的机制，
/// 判据 #1 会因次数少而**恰好不越阈值** ✗✓ —— 故本探针把次数补齐。
///
/// 只**打印**（前 400 步的硬判据仍由判据 #1 负责 ✓，本探针不制造红灯 ✓）。
#[test]
fn bridge_yaw_drift_probe() {
    use flyctrl_core::estimator::fcalg_bridge::FcalgEstimator;
    use flyctrl_core::estimator::trait_def::*;
    use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond, Second};
    use flyctrl_core::vehicle::ImuSample;

    let mut est = FcalgEstimator::new();
    let dt = Second(4.0 / 1000.0); // ★与固件同口径（ekf_task.rs:32 ✓）
    let mut worst = 1.0f32;
    for k in 0..1300 {
        let imu = ImuSample {
            accel: [
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(-9.81),
            ],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let _ = est.step(dt, imu, None, None);
        worst = worst.min(est.state().att.w.abs());
        if k == 399 || k == 799 || k == 1299 {
            eprintln!(
                "[yaw] after {} steps: yaw = {:.5} rad ({:.2} deg) | worst|w| = {:.5}",
                k + 1,
                est.yaw_rad(),
                est.yaw_rad().to_degrees(),
                worst
            );
        }
    }
}

/// ★装配层判据 #4（2026-10-05）：**丢帧 ⇒ sample-and-hold ⇒ 陀螺重复积分** 是否造成偏航漂移。
///
/// 机制（全链最后未审的一环，`hil.rs:408-410` ✓）：
///   "无真实帧则回退最近真实帧（sample-and-hold，**角速度继续积分**、比力继续锚定）"
///   ⇒ 固件 **250 Hz 调用**（4 ms ✓）而样本 **≈77 Hz 到达**（13 ms ✓）⇒ **非新帧占 3.25×** ✗
///   ⇒ 若"保持住的那个陀螺"非零，角度会按 **3.25× 过积分** ✗✓
/// 而 `hil.rs` 注释记载 M 场**确有**垃圾帧（"陀螺 z 挖出 **2.7e6 rad/s**" ✗）⇒ §5.134 门
///   （>100 rad/s ⇒ 丢帧 + 计数 ✓）会把它拒掉 ✗ ⇒ **丢帧后保持 = 用上上一帧继续积分** ✓✓
/// 之所以可疑：**偏航不被任何观测锚定**（磁参考为零 ⇒ 按契约拒绝 ✓、重力观测对偏航无感 ✓）
///   ⇒ 一旦积分偏了就**永远留下** ✓ ⇒ 与"真值水平/航向 0 但估计 ~111° 纯偏航"✓ 完全对得上 ✓
///
/// 本判据按固件口径跑：250 Hz 调用 ✓、每 3 拍一个真帧（≈77 Hz ✓）、第 500 拍插垃圾帧 ✓。
/// **只打印**（不制造红灯 ✓）；若偏航漂了 ⇒ 机制坐实 ✓。
#[test]
fn hil_garbage_frame_yaw_drift_probe() {
    use flyctrl_core::controller::PidController;
    use flyctrl_core::estimator::select::AnyEstimator;
    use flyctrl_core::hil::{HilContext, SimImu};
    use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond, Second};
    use flyctrl_core::vehicle::ImuSample;

    // 与 app 同构（`ekf_task.rs:29-33` ✓）
    let mut hil = HilContext::new(
        AnyEstimator::default_product(),
        PidController::default_quad(),
        Second(4.0 / 1000.0),
    );
    let mut sim = SimImu::new();
    // 与 app 同做法：`SETPOINT` 就是 `core::mem::zeroed()` ✓（字段未知也无碍 ✓）
    let sp = unsafe { core::mem::zeroed() };
    let level = |gz: f32| ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(-9.81),
        ],
        gyro: [RadianPerSecond(0.0), RadianPerSecond(0.0), RadianPerSecond(gz)],
    };
    let mut worst_yaw = 0.0f32;
    for k in 0..1300 {
        let imu = if k == 500 {
            Some(level(1.0e6)) // 垃圾帧：陀螺 1e6 rad/s ⇒ §5.134 丢弃（照 M 场 2.7e6 的事故 ✓）
        } else if k % 3 == 0 {
            Some(level(0.0)) // 真帧（≈77 Hz ✓）
        } else {
            None // 非新帧 ⇒ sample-and-hold + 角速度继续积分 ✗（被测机制 ✓）
        };
        let _ = hil.ekf_hil(imu, None, None, None, None, None, &sp, false, false, true, &mut sim);
        worst_yaw = worst_yaw.max(hil.est.inner.yaw_rad().abs());
    }
    eprintln!(
        "[hil] 1300 拍（含 1 个垃圾帧）：worst |yaw| = {:.6} rad = {:.3} deg",
        worst_yaw,
        worst_yaw.to_degrees()
    );
}

/// ★装配层判据 #5（2026-10-05）：**门内陀螺尖峰** ⇒ 偏航跳变探针（**非空转** ✓）。
///
/// 修正判据 #4 的空转 ✗：#4 的真帧陀螺全 0 ⇒ 保持到的也是 0 ⇒ 没东西可积分 ✗。
/// 本判据按**收敛结论**设计（见 #4 与本文件上文）：
///   · 111° 量级 ⇒ 只能来自**一次大事件** ✓（采样保持已用算术否证 ✗）
///   · `§5.134` 门只拦 **>100 rad/s** ✗ ⇒ **50 rad/s 的垃圾帧会被放行并直接积分** ✓
///   · 偏航**不可观测**（磁参考为零 ⇒ 按契约拒绝 ✓；重力观测对偏航无感 ✓）
///     ⇒ 一旦积分偏了就**永久保留** ✓
/// 预期（若机制成立）：单帧 50 rad/s ⇒ 0.2 rad/拍；该样本随后被 sample-and-hold
///   继续积分约 3 拍（下一真帧到达前）⇒ 约 **0.6 rad ≈ 34°/帧** ✗ ⇒ 3 帧即 ~100° ✓
///
/// **只打印**（不制造红灯 ✓）；若偏航确实跳变 ⇒ 机制坐实 ✓。
#[test]
fn hil_gate_passing_gyro_spike_probe() {
    use flyctrl_core::controller::PidController;
    use flyctrl_core::estimator::select::AnyEstimator;
    use flyctrl_core::hil::{HilContext, SimImu};
    use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond, Second};
    use flyctrl_core::vehicle::ImuSample;

    let mut hil = HilContext::new(
        AnyEstimator::default_product(),
        PidController::default_quad(),
        Second(4.0 / 1000.0),
    );
    let mut sim = SimImu::new();
    let sp = unsafe { core::mem::zeroed() };
    let frame = |gz: f32| ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(-9.81),
        ],
        gyro: [RadianPerSecond(0.0), RadianPerSecond(0.0), RadianPerSecond(gz)],
    };
    // 三个"门内尖峰"帧：50 rad/s（< 100 ⇒ **不会被 §5.134 拒** ✓）
    let spikes = [500usize, 700, 900];
    for k in 0..1300 {
        let imu = if spikes.contains(&k) {
            Some(frame(50.0))
        } else if k % 3 == 0 {
            Some(frame(0.0))
        } else {
            None
        };
        let _ = hil.ekf_hil(imu, None, None, None, None, None, &sp, false, false, true, &mut sim);
        if spikes.contains(&(k + 1)) || k == 1299 {
            eprintln!(
                "[spike] 第 {} 拍：yaw = {:.4} rad = {:.2} deg",
                k + 1,
                hil.est.inner.yaw_rad(),
                hil.est.inner.yaw_rad().to_degrees()
            );
        }
    }
}

/// ★装配层判据 #6（2026-10-05）：**比力 z 符号**对姿态初始化的影响（带对照组 ✓）。
///
/// 来源（`hil.rs:512-520` 自记载的缺陷 + 本轮真链路签名）：
///   真链路 `est wxyz = [-0.0814, …]`（**w≈0 ⇒ 旋转≈171°** ✗）—— 而 `±π`（w≈0）只需
///   一个条件：`a[2] > 0` ✗（`roll = atan2(−a[1], −a[2])` 在该条件下给出 ±π ✓，
///   与量纲无关 ✓）。而 fcalg 全体（`State::level()`、`update_gravity`、
///   `observe.rs:112` 的 `h = Rᵀ(−GRAVITY_NED)+ba`）都假设 **z 向下、静止 `a_z=−9.81`** ✓。
/// 故本判据做**对照实验**：
///   · A 组（对照 ✓）：`accel = (0,0,−9.81)` ⇒ 期望 `w ≈ ±1`（水平 ✓）
///   · B 组（假设 ✗）：`accel = (0,0,+9.81)` ⇒ 若签名成立，应得 `w ≈ 0`（±π ✗）
/// **只打印**（不制造红灯 ✓）；B 组复现 ⇒ 假设坐实 ✓。
#[test]
fn hil_accel_z_sign_init_probe() {
    use flyctrl_core::controller::PidController;
    use flyctrl_core::estimator::select::AnyEstimator;
    use flyctrl_core::hil::{HilContext, SimImu};
    use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond, Second};
    use flyctrl_core::vehicle::ImuSample;

    for (label, az) in [("A 对照 az=-9.81", -9.81f32), ("B 假设 az=+9.81", 9.81f32)] {
        let mut hil = HilContext::new(
            AnyEstimator::default_product(),
            PidController::default_quad(),
            Second(4.0 / 1000.0),
        );
        let mut sim = SimImu::new();
        let sp = unsafe { core::mem::zeroed() };
        for k in 0..1300 {
            let imu = if k % 3 == 0 {
                Some(ImuSample {
                    accel: [
                        MeterPerSecondSquared(0.0),
                        MeterPerSecondSquared(0.0),
                        MeterPerSecondSquared(az),
                    ],
                    gyro: [RadianPerSecond(0.0); 3],
                })
            } else {
                None
            };
            let _ = hil.ekf_hil(imu, None, None, None, None, None, &sp, false, false, true, &mut sim);
        }
        // 从四元数取姿态（`Quaternion` 有 w/x/y/z ✓）
        let q = hil.est.state().att;
        eprintln!(
            "[sign] {} ⇒ att wxyz = [{:.4}, {:.4}, {:.4}, {:.4}]  (yaw={:.2} deg)",
            label,
            q.w,
            q.x,
            q.y,
            q.z,
            hil.est.inner.yaw_rad().to_degrees()
        );
    }
}

/// ★装配层判据 #7（2026-10-05）：**坏初始化之后能否恢复**（机制判定 ✓）。
///
/// 来源：帧内比力已**测得**为 −9.81 ✓（harness 注入 `st.imu_acc=[0,0,-9.81]` ✓，
/// 外设纯单位换算 ✓，驱动解码 ✓）⇒ 边界/仿真侧全部无罪 ✓ ⇒ 171° 另有其处 ✗。
/// 新机制假设（与全部事实自洽 ✓，本判据判定）：
///   ① 初始化的那一拍比力若方向错（IIR 瞬态/首帧垃圾 ✗）⇒ 初值被格成 ~180° ✗
///   ② 之后靠重力观测**修不回来** ✗ —— 修正量约 2g，远超 NIS 门限 ⇒ **每拍被拒** ✗
///   ③ 偏航本就不可观测（磁参考为零 ⇒ 按契约拒 ✓）⇒ 永久冻结 ✓
/// 本判据：第一帧 `az=+9.81`（照判据 #6 的 180° 签名 ✓），其后全部 `az=−9.81`。
///   · 姿态长期冻在翻转 ⇒ ② 成立 ✓（修法：持续性大创新时放行一次重灌/重对齐 ✓）
///   · 姿态恢复水平 ⇒ ② 不成立 ✗
/// **只打印**（不制造红灯 ✓）。
#[test]
fn hil_bad_init_then_good_probe() {
    use flyctrl_core::controller::PidController;
    use flyctrl_core::estimator::select::AnyEstimator;
    use flyctrl_core::hil::{HilContext, SimImu};
    use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond, Second};
    use flyctrl_core::vehicle::ImuSample;

    let mut hil = HilContext::new(
        AnyEstimator::default_product(),
        PidController::default_quad(),
        Second(4.0 / 1000.0),
    );
    let mut sim = SimImu::new();
    let sp = unsafe { core::mem::zeroed() };
    let frame = |az: f32| ImuSample {
        accel: [
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(0.0),
            MeterPerSecondSquared(az),
        ],
        gyro: [RadianPerSecond(0.0); 3],
    };
    for k in 0..1300 {
        // 第 0 拍（首个被接受的帧，触发初始化）给 +9.81 ⇒ 坏初值；其后一律 −9.81 ✓
        let az = if k == 0 { 9.81 } else { -9.81 };
        let imu = if k % 3 == 0 { Some(frame(az)) } else { None };
        let _ = hil.ekf_hil(imu, None, None, None, None, None, &sp, false, false, true, &mut sim);
        if k == 0 || k == 9 || k == 99 || k == 399 || k == 1299 {
            let q = hil.est.state().att;
            eprintln!(
                "[recov] 第 {:4} 拍（az={:+.2}）：wxyz = [{:.4}, {:.4}, {:.4}, {:.4}]",
                k + 1, az, q.w, q.x, q.y, q.z
            );
        }
    }
}
