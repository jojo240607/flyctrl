//! 控制律硬实时任务（核心，4ms 周期）。
//!
//! 流程：取最新传感器帧 → EKF → FDIR → PID → PWM。
//! 读 SENSOR_FRAME（经 seqlock，见 `SENSOR_SEQ`）、写 EST_STATE（经 EST_MTX）。

use core::ffi::c_void;

use flyctrl_core::controller::{PidController, Setpoint};
use flyctrl_core::estimator::EkfEstimator;
use flyctrl_core::fdir::Health;
use flyctrl_core::hil::{HilContext, SimImu};
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, Second};
use flyctrl_core::vehicle::{RcInput, VehicleState};

use rtos_app_sdk::abi::RTOS_PRIO_BH_HIGH;
use rtos_app_sdk::device::Device;
use rtos_app_sdk::ioctl;
use rtos_app_sdk::{info, warn};
#[cfg(not(feature = "hil"))]
use rtos_app_sdk::rtos::msleep;
use core::sync::atomic::Ordering;

/// 诊断开关：开启后会在启动前几圈打印大量 dbg 行，极易压垮开机瞬间的
/// 设备串口 TX 缓冲、导致同期的传感器任务日志被丢弃（误判传感器任务“死亡”）。
/// 正常验证时关闭。
const VERBOSE: bool = false;
#[cfg(feature = "hil")]
use crate::flyctrl::HIL_EVT;
use crate::flyctrl::{make_name, EST_MTX, EST_STATE, SENSOR_FRAME, SENSOR_SEQ};

/// 上行指令解锁：地面站经 COMMAND_LONG(ARM/DISARM) 设置。
/// 与控制律内部 RC 解锁做逻辑或（任一为真即解锁）。
pub fn set_cmd_armed(arm: bool) {
    crate::flyctrl::uplink::G_CMD_ARMED.store(arm, Ordering::Relaxed);
}

/// 上行指令模式：地面站经 COMMAND_LONG(DO_SET_MODE) 设置。
/// telemetry 心跳 custom_mode 会读取此值反映当前模式。
pub fn set_cmd_mode(mode: u16) {
    crate::flyctrl::uplink::G_CMD_MODE.store(mode, Ordering::Relaxed);
}

/// 控制律硬实时任务入口。
pub extern "C" fn control_entry(_arg: *mut c_void) {
    info!(tag: "ctrl", "task started; period=4ms prio={}", RTOS_PRIO_BH_HIGH);

    // 控制律对象（共享单步：SIL/HIL 同一份编排，见 `flyctrl_core::hil::step_hil`）。
    // 姿态/位置初始化门控、SimImu 回退、EKF + 气压观测、FDIR、控制环健康闸、
    // 执行器限幅全部由 `step_hil` 完成，与 SIL（fly-sim-core）完全一致。
    let mut hil = HilContext::new(
        EkfEstimator::default_quad(),
        PidController::default_quad(),
        Second(4.0 / 1000.0),
    );
    // 共享单步回退 IMU（与 SIL 同源实现，保证注入饥饿时回退数据完全一致）。
    let mut sim_imu = SimImu::new();
    let mut hold_alt = Meter(0.0);
    let mut alt_locked = false;
    // 上一拍估计状态（供非 HIL 设定点高度基准 / HIL 链路未建立时定高；
    // EKF 位置 4ms 内变化远小于 1mm，用上一拍等价）。
    let mut last_est: Option<VehicleState> = None;
    let mut seq: u32 = 0;

    // [联调诊断] 最近一拍执行器指令（静态，测试直读；定位后移除）
    #[used]
    static mut DBG_MOTOR: [f32; 4] = [0.0; 4];
    // PWM 设备（4 路，control 专用）
    let mut pwm_dev: [Option<Device>; 4] = [None, None, None, None];
    let mut pwm_period: [u32; 4] = [0; 4];
    for i in 0..4 {
        let name = make_name(i as u8);
        if let Some(d) = Device::open(name) {
            // 设 400Hz（2500us 周期），取回 period_ticks 供占空比换算
            let mut freq = 400u32;
            let _ = d.ioctl(ioctl::PWM_IOCTL_SET_FREQ, &mut freq as *mut u32 as *mut c_void);
            let mut ticks = 0u32;
            let _ = d.ioctl(ioctl::PWM_IOCTL_GET_PERIOD_TICKS, &mut ticks as *mut u32 as *mut c_void);
            pwm_period[i] = ticks;
            pwm_dev[i] = Some(d);
        } else {
            warn!(tag: "ctrl", "pwm{} not available -> actuator disabled", i);
        }
    }

    let mut first = true;
    loop {
        let _dt = Second(4.0 / 1000.0);
        // 应用地面站参数（每周期原子读 G_PARAM_VALS -> pid 增益；PARAM_SET 即时生效）。
        crate::flyctrl::uplink::sync_gains_to_pid(&mut hil.ctrl);
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: loop enter"); }

        // --- 取最新传感器帧（seqlock：control 优先级高于所有写者，读不被打断） ---
        // 写者：sensors(prio=5，非 HIL) / uplink(prio=10，HIL)。两者优先级均低于本任务
        // (control, prio=4)，因此读过程不可能被写者抢占 → 单次读即原子一致，无需重试。
        // 【关键】绝不能 `continue` 忙等重试：若赶上写者正处于写入中（SENSOR_SEQ 为奇，
        // 写者被本任务抢占在置奇与置偶之间），忙等会让低优先级写者永远得不到调度，
        // control 无限自旋 → 整机卡死（HIL 注入期间已实测复现：运行数秒后日志/下行全停）。
        // 正确处理：直接采用本拍快照（可能新老混合/略旧），下一 4ms 拍自然取得一致新帧。
        let (imu, rc, gps, baro_alt, armed);
        unsafe {
            let f = &mut *core::ptr::addr_of_mut!(SENSOR_FRAME);
            imu = f.imu;
            rc = f.rc;
            gps = f.gps;
            baro_alt = f.baro_alt;
            armed = f.armed;
            // 【HIL 关键】IMU 单次消费：PC 每 ~32ms 才注入一帧 HIL_SENSOR，而本任务 4ms 一拍，
            // 若读后不清空，同一陀螺样本会被连续积分 8 拍（重复积分同一角速度 → 姿态过积分发散）。
            // 安全前提：control(prio=4) 高于所有写者(uplink prio=10 / sensors prio=5)，本拍读写之间
            // 不可能被写者抢占，因此可就地清空、不会误清新注入帧；清空后下一拍无新 IMU 时，
            // 自然回退 SimImu（零角速度 → 不漂移、不触发 FDIR 冻结误判）。
            #[cfg(feature = "hil")]
            {
                f.imu = None;
            }
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        }
        // 指令解锁：与地面站上行命令做逻辑或（RC 解锁 或 指令解锁 任一为真）。
        let cmd_armed = crate::flyctrl::uplink::G_CMD_ARMED.load(Ordering::Relaxed);
        let armed_eff = armed || cmd_armed;
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: sen-mtx got"); }

        // 地面站 RC 通道覆盖（RC_CHANNELS_OVERRIDE）：生效时以地面站通道优先于 sim RC。
        // 通道映射：ch1=roll, ch2=pitch, ch3=throttle, ch4=yaw（标准 MAVLink 约定）。
        // PWM 1000-2000us 归一化到 0.0-1.0。
        let (rc_ov, rc_ov_valid) = crate::flyctrl::uplink::get_rc_override();
        let rc = if rc_ov_valid {
            let norm = |pwm: u16| -> f32 {
                let v = (pwm as f32 - 1000.0) / 1000.0;
                v.clamp(0.0, 1.0)
            };
            RcInput {
                throttle: norm(rc_ov[2]),
                roll: norm(rc_ov[0]),
                pitch: norm(rc_ov[1]),
                yaw: norm(rc_ov[3]),
                armed: rc_ov[0] > 1500, // 暂以 ch1 高位作为地面站解锁指示（占位，主解锁仍靠 COMMAND_LONG）
                mode: rc.mode,
                fresh: true,
            }
        } else {
            rc
        };

        // --- 期望状态：模式决定目标（原点定高 / RTL 回原点 / LAND 缓降） ---
        // custom_mode 用 ArduCopter 标准码（G_CMD_MODE 由上行 DO_SET_MODE/TAKEOFF/LAND/RTL 写入）。
        // HIL：设定点直接来自 PC 仿真器（SET_POSITION_TARGET_LOCAL_NED），RC 路径编译期关闭。
        // 设定点在共享单步**之前**构造：非 HIL 高度基准 / HIL 回退定高均用上一拍估计
        // `last_est`（EKF 位置 4ms 内变化远小于 1mm，与"本拍估计后构造"等价）。
        let (setpoint, setpoint_valid) = {
            #[cfg(feature = "hil")]
            {
                use flyctrl_core::units::Radian;
                let sp = crate::flyctrl::uplink::hil_setpoint();
                if crate::flyctrl::uplink::hil_setpoint_valid() {
                    (
                        Setpoint {
                            pos: [Meter(sp.x), Meter(sp.y), Meter(sp.z)],
                            yaw: Radian(sp.yaw),
                            vel: [MeterPerSecond(sp.vx), MeterPerSecond(sp.vy), MeterPerSecond(sp.vz)],
                            acc: [MeterPerSecondSquared(sp.afx), MeterPerSecondSquared(sp.afy), MeterPerSecondSquared(sp.afz)],
                        },
                        true,
                    )
                } else {
                    // sim 尚未连接：保持当前位置定高，避免悬停指令冲击。
                    let hold_z = last_est.map(|e| e.pos[2].0).unwrap_or(0.0);
                    (
                        Setpoint {
                            pos: [Meter(0.0), Meter(0.0), Meter(hold_z)],
                            yaw: Radian(0.0),
                            vel: [MeterPerSecond(0.0); 3],
                            acc: [MeterPerSecondSquared(0.0); 3],
                        },
                        false,
                    )
                }
            }
            #[cfg(not(feature = "hil"))]
            {
                use flyctrl_core::units::Radian;
                let cmd_mode = crate::flyctrl::uplink::G_CMD_MODE.load(Ordering::Relaxed);
                use flyctrl_core::comm::mavlink::enums::COPTER_MODE_LAND;
                let thr_off = (rc.throttle - 0.5) * 2.0;
                // 默认目标：锁定高度基准（原点）。LAND 模式触发持续缓降。
                let mut target_alt = hold_alt.0 - thr_off * 2.0;
                if cmd_mode == COPTER_MODE_LAND {
                    // LAND：在基准高度上每周期降 0.02m，趋向地面（D 向下，地面=0）。
                    let cur_z = last_est.map(|e| e.pos[2].0).unwrap_or(hold_alt.0);
                    target_alt = (cur_z - 0.02).max(0.0);
                }
                // RTL/LOITER 水平目标已为原点（N=0,E=0）；STABILIZE 保持同样基准，确保联调可观测。
                // 【演示机动】摇杆 roll/pitch 映射为水平位置目标（×8.5m/满偏）：PC 端 vperiph
                // 注入 SBUS 摇杆正弦 → 飞机按 8 字轨迹拉向目标点，半径 = 摇杆幅度×8.5（实测
                // amp 0.4 → ±3.4m 目标，Position 环跟随 ~0.89 → ~3.0m）。摇杆中位 → 原点
                // （悬停），不影响定高（高度仍由油门决定）。
                (
                    Setpoint {
                        pos: [Meter(rc.pitch * 8.5), Meter(-rc.roll * 8.5), Meter(target_alt)],
                        yaw: Radian(rc.yaw * 0.5),
                        vel: [MeterPerSecond(0.0); 3],
                        acc: [MeterPerSecondSquared(0.0); 3],
                    },
                    // 非 HIL 模式的「原点定高」设定点恒有效：RC 油门/模式/已锁基准
                    // 构成的 setpoint 是真实控制目标。若传 false，`hil_pos_inited`
                    // 永不置位（step_hil 位置闸要求 setpoint_valid），执行器恒零——
                    // 正是联调 F 的「非 HIL 无 PWM」阻塞根因（旧版临时放宽绕开的点）。
                    true,
                )
            }
        };

        // --- 共享单步（SIL/HIL 同一份编排，见 `flyctrl_core::hil::step_hil`） ---
        // IMU 单次消费已在上方 SENSOR_FRAME 读取时完成（HIL 下 `f.imu = None`）；
        // SimImu 回退、姿态/位置初始化门控、EKF 估计 + 气压观测、FDIR、控制环健康闸、
        // 执行器限幅全部在 `step_hil` 内部完成，与 SIL（fly-sim-core）完全一致。
        let r = hil.step_hil(
            imu, gps, baro_alt, None, None, &setpoint, setpoint_valid, armed_eff, rc.fresh, &mut sim_imu,
        );
        let est = r.est;
        let health = r.health;
        let cmd = r.cmd;
        last_est = Some(est);

        // 解锁瞬间锁定高度基准
        // [BISECT] armed_eff 已退化为 armed
        if armed_eff && !alt_locked {
            hold_alt = est.pos[2];
            alt_locked = true;
        } else if !armed_eff {
            alt_locked = false;
        }

        // HIL：回传执行器指令供 telemetry 组 HIL_ACTUATOR_CONTROLS（PC 端注入 plant）。
        #[cfg(feature = "hil")]
        crate::flyctrl::uplink::set_actuator_cmd(&cmd.motor);

        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: pid ok"); }

        // --- 输出 PWM（4 路 ioctl 设占空比 ticks） ---
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: before pwm"); }
        // [联调诊断] 指令观测（静态，无栈开销；测试直读定位无推力来源）
        unsafe {
            DBG_MOTOR = cmd.motor;
        }
        for i in 0..4 {
            if let Some(d) = &pwm_dev[i] {
                let m = cmd.motor[i].clamp(0.0, 1.0);
                let us = 1000.0 + 1000.0 * m;
                let ticks = (us * pwm_period[i] as f32 / 2500.0) as u32;
                let mut t = ticks;
                let rc = d.ioctl(ioctl::PWM_IOCTL_SET_DUTY_TICKS, &mut t as *mut u32 as *mut c_void);
                if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: pwm{} rc={} ticks={}", i, rc, ticks); }
            }
        }
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: after pwm"); }
        if VERBOSE && seq == 0 {
            let ec = unsafe { EST_MTX.debug_count() };
            info!(tag: "ctrl", "dbg: est-mtx count={} sensor-seq={}", ec, unsafe { SENSOR_SEQ });
        }

        // --- 发布估计状态（telemetry/monitor 读） ---
        {
            let _g = unsafe { EST_MTX.guard() };
            if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: in est-guard"); }
            let s = unsafe { &mut *core::ptr::addr_of_mut!(EST_STATE) };
            if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: est-addr got"); }
            s.armed = armed_eff; // [BISECT] 退化为 armed
            if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: est-armed written"); }
            s.est = est;
            s.health = health;
            if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: est-written"); }
            // 油门百分比(0..100) 供 telemetry 经 VFR_HUD 下发。
            let throttle_avg = (cmd.motor[0] + cmd.motor[1] + cmd.motor[2] + cmd.motor[3]) / 4.0;
            crate::flyctrl::uplink::G_THROTTLE.store((throttle_avg.clamp(0.0, 1.0) * 100.0) as u8, Ordering::Relaxed);
        }
        if VERBOSE && seq == 0 { info!(tag: "ctrl", "dbg: est-mtx got"); }

        seq = seq.wrapping_add(1);
        if first {
            first = false;
            info!(tag: "ctrl",
                  "first loop done; imu_ok={} armed={} crit={} alt={:.2}",
                  imu.is_some(), armed_eff, health == Health::Critical, est.pos[2].0);
        }
        if seq % 25 == 0 {
            // [DIAG] EKF 状态演变诊断：每次 25 拍(≈100ms) 打印姿态/位置/速度/有限性，
            // 定位 NaN 出现的时刻与当时的 EKF 状态（闭环发散排查用，定位后移除）。
            let (r, p, y) = (est.att.roll(), est.att.pitch(), est.att.yaw());
            let fin = est.att.w.is_finite() && est.att.x.is_finite()
                && est.att.y.is_finite() && est.att.z.is_finite()
                && est.pos.iter().all(|v| v.0.is_finite())
                && est.vel.iter().all(|v| v.0.is_finite());
            info!(tag: "ctrl", "dbg est r={:.1} p={:.1} y={:.1}deg p=({:.2},{:.2},{:.2}) v=({:.2},{:.2},{:.2}) fin={} imu_ok={}",
                  r.to_degrees(), p.to_degrees(), y.to_degrees(),
                  est.pos[0].0, est.pos[1].0, est.pos[2].0,
                  est.vel[0].0, est.vel[1].0, est.vel[2].0, fin, imu.is_some());
        }
        if seq % 250 == 0 {
            // [DIAG] hb + GPS Doppler 速度（验证 RMC 速度链路：gps_v 为 SENSOR_FRAME
            // 中 PosSample.vel，Some 表示固件解析到了 $GNRMC 的速度字段）
            let gps_v = gps.and_then(|g| g.vel).map(|v| [v[0].0, v[1].0, v[2].0]);
            let (gv, gv_n) = match gps_v {
                Some(v) => (v, 1u8),
                None => ([0.0, 0.0, 0.0], 0u8),
            };
            info!(tag: "ctrl", "hb seq={} armed={} crit={} alt={:.2} imu_ok={} gps={} gps_v={} gv=({:.2},{:.2},{:.2}) baro={} baro_h={:.2} gpsd={:.2} gz={:.2} m=[{:.3},{:.3},{:.3},{:.3}]",
                  seq, armed_eff, health == Health::Critical, est.pos[2].0,
                  imu.is_some(), gps.is_some(), gv_n, gv[0], gv[1], gv[2],
                  baro_alt.is_some(), baro_alt.unwrap_or(0.0),
                  gps.map(|g| g.pos[2].0).unwrap_or(0.0), est.vel[2].0,
                  cmd.motor[0], cmd.motor[1], cmd.motor[2], cmd.motor[3]);
        }

        // 【HIL 事件驱动】不依赖 control 自身 4ms 时钟：阻塞等待下一帧 HIL_SENSOR
        // 注入（uplink 写完真值即 `give()`），收到一帧执行一拍 `step_hil`——与 SIL
        // 的"每物理步一拍、读最新样本"推模式 1:1 对齐，消除双时钟失配导致的输入流
        // 差异（93.2% 控制拍缺 IMU 回退陈旧数据 → 姿态发散）。非 HIL 保持 4ms 周期轮询。
        #[cfg(feature = "hil")]
        unsafe { HIL_EVT.wait(); }
        #[cfg(not(feature = "hil"))]
        msleep(4);
        if VERBOSE && seq < 5 {
            info!(tag: "ctrl", "dbg: after sleep seq={}", seq);
        }
    }
}
