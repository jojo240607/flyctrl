//! ★design.md **L1 `rate` 薄硬实时线程**（1kHz）：陀螺 → 速率 PID → 混控 → PWM。
//!
//! 分层依据（`docs/fc-task-architecture.md`）：
//!   · **本任务（L1，最高优先级）**：只做速率环 + 控制分配 + 执行器输出（薄，栈小）；
//!   · L2 `ekf`(250Hz)：ESKF/FDIR → `EST_STATE`；
//!   · L2 `control`(250Hz)：姿态层 → `RATE_CMD`。
//!
//! 与 PX4 对齐：本任务 ≈ `mc_rate_control` + `control_allocator`（1kHz，跑在陀螺率）；
//! EKF（`ekf2`）与姿态（`mc_att_control`）在 L2 ✓。**禁止阻塞**（只做计算 + 寄存器写）。

use core::ffi::c_void;

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::{PidController, RateSetpoint};
use flyctrl_core::hal::actuator::clamp_thrust;
use flyctrl_core::units::Second;
use flyctrl_core::vehicle::ActuatorCmd;

use rtos_app_sdk::device::Device;
use rtos_app_sdk::ioctl;
use rtos_app_sdk::rtos::{delay_until, tick_count};
use rtos_app_sdk::{info, warn};

use crate::flyctrl::rt_stat::RtStat;
use crate::flyctrl::{make_name, HIL_DIAG, RATE_CMD, SENSOR_FRAME};

/// L1 速率环最近一次 exec（cycles）——供 safety 汇总 L1 负载（§8 过载判定）。
pub static mut RATE_EXEC_CYC: u32 = 0;

/// L1 速率环任务入口（1kHz）。
pub extern "C" fn rate_entry(_arg: *mut c_void) {
    info!(tag: "rate", "task started; period=1ms (1kHz, L1 mc_rate_control+allocator)");

    // 速率层控制器（仅用 `rate_step`：kp_rate / ki_rate / dgyro_k / mix_mode ✓）。
    let cfg = VehicleConfig::default_quad();
    let mut pid = PidController::from_config(&cfg.ctrl_params());

    // PWM 设备：本任务为唯一写者 ✓。
    let mut pwm_dev: [Option<Device>; 4] = [None, None, None, None];
    let mut pwm_period: [u32; 4] = [0; 4];
    for i in 0..4 {
        let name = make_name(i as u8);
        if let Some(d) = Device::open(name) {
            let mut freq = 400u32; // 400Hz（2500us 周期）
            let _ = d.ioctl(ioctl::PWM_IOCTL_SET_FREQ, &mut freq as *mut u32 as *mut c_void);
            let mut ticks = 0u32;
            let _ = d.ioctl(ioctl::PWM_IOCTL_GET_PERIOD_TICKS, &mut ticks as *mut u32 as *mut c_void);
            pwm_period[i] = ticks;
            pwm_dev[i] = Some(d);
        } else {
            warn!(tag: "rate", "pwm{} not available -> actuator disabled", i);
        }
    }

    // 节拍源：timer2/TIM6 @1kHz（专用 IRQ54）；失败则回退 1ms `delay_until`（反静默降级 ✓）。
    let paced = crate::flyctrl::pace::rate::init();
    info!(tag: "rate", "pace: {}",
          if paced { "timer2/TIM6 1.000ms 精确节拍 ✓" } else { "回退 delay_until(1 tick) ✗" });

    let mut wake_tick = tick_count();
    let mut last_ticks = wake_tick;
    let mut first = true;
    let mut st = RtStat::new("rate");
    let mut it: u32 = 0;
    loop {
        let t0 = st.tick();
        // ★design.md §7：dt 用**样本戳**（环形最新样本的实测 dt），不用 1ms tick 网格 ✗。
        let now_ticks = tick_count();
        let period_ticks = now_ticks.wrapping_sub(last_ticks);
        last_ticks = now_ticks;

        // 速率设定值（姿态层 250Hz 发布；`valid` 已含健康/解锁闸 ✓）。
        let rsp = unsafe {
            let c = &*core::ptr::addr_of!(RATE_CMD);
            if c.valid != 0 {
                RateSetpoint::new(c.rates, c.thrust)
            } else {
                RateSetpoint::INVALID
            }
        };
        // 新鲜原始陀螺（≈ PX4 `vehicle_angular_velocity` ✓；`rate_step` 内部做 rate_lpf/dgyro 滤波 ✓）。
        // ★design.md §7：取 `IMU_RING` 最新陀螺（带硬件戳；不再读原始帧）。
        // ★design.md §7：取 `IMU_RING` 最新陀螺（带硬件戳）。
        let sample_dt = unsafe {
            let r = &*core::ptr::addr_of!(crate::flyctrl::IMU_RING);
            match r.latest() {
                Some(d) => d.dt_ang,
                None => 0.001,
            }
        };
        let gyro = unsafe {
            let r = &*core::ptr::addr_of!(crate::flyctrl::IMU_RING);
            match r.latest() {
                Some(d) => d.gyro(),
                None => [0.0; 3],
            }
        };
        // ★修复 C2：速率环是**离散控制器**，dt 必须用【本环迭代间隔】（tick 实测），
        //   不能用 IMU 样本 dt —— 样本率 < 环频率时会把同一增量积分 N 倍（过积分）。
        //   PX4 同构：HRT 速率环的 dt 就是环周期。样本 dt 仍供 EKF 逐样本 predict 用 ✓。
        let _ = sample_dt;
        let dt = Second((period_ticks.clamp(1, 20)) as f32 / 1000.0);
        // ★修复 C1 配套：HIL 无 DRDY ISR ⇒ 由本环（1kHz）驱动共享内存注入，
        //   否则注入仅在 50Hz uplink item ⇒ 1kHz 环拿到 20ms 滞后陀螺 ⇒ 振荡发散。
        #[cfg(feature = "hil")]
        crate::flyctrl::hil_shmem::shmem_poll_once();

        // 地面站参数即时生效（速率层增益 ✓）。
        crate::flyctrl::uplink::sync_gains_to_pid(&mut pid);

        // ★design.md §4：`rate_ctrl` **只算力矩/推力**（不再直接出电机）——
        //   经 `RATE_TT` 交独立 L1 线程 `control_allocator` 做分配+PWM ✓（职责单一）。
        let tt = if crate::flyctrl::safety_task::SAFETY_KILL
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            flyctrl_core::controller::TorqueThrust::default() // valid=false ⇒ 分配器零输出
        } else {
            pid.rate_step_torque(dt, &rsp, gyro)
        };
        unsafe {
            let t = &mut *core::ptr::addr_of_mut!(crate::flyctrl::RATE_TT);
            t.torque = tt.torque;
            t.thrust = tt.thrust;
            t.valid = tt.valid as u32;
            t.seq = t.seq.wrapping_add(1);
        }

        // 执行器输出/PWM 已迁至 L1 `alloc`（`control_allocator`）✓。

        let exec = st.tick().wrapping_sub(t0);
        unsafe { RATE_EXEC_CYC = exec; }
        st.sample(exec, period_ticks.saturating_mul(168_000), 168_000); // 1kHz 名义
        it = it.wrapping_add(1);
        // ★design.md §4：**L1 CPU 预算校验（≤60%）** —— rate(1kHz)+safety(500Hz) 占用
        {
            let r_cyc = st.last_exec() as u64;
            let a_cyc = unsafe { crate::flyctrl::alloc_task::ALLOC_EXEC_CYC } as u64;
            let s_cyc = unsafe { crate::flyctrl::safety_task::SAFETY_EXEC_CYC } as u64;
            // design.md §4：L1 = rate(1kHz) + control_allocator(1kHz) + safety(500Hz)
            let l1_permille = ((r_cyc * 1000 + a_cyc * 1000 + s_cyc * 500) * 1000 / 168_000_000) as u32;
            if it % 1000 == 0 && l1_permille > 600 {
                info!(tag: "rate", "L1 CPU budget exceeded: {}permille (>600, design.md §4)", l1_permille);
            }
        }
        if it % 500 == 0 { st.report_and_reset(); } // ★P1-2 可观测
        if first {
            first = false;
            info!(tag: "rate", "first loop done; valid={} rates=({:.3},{:.3},{:.3})",
                  rsp.valid, rsp.rates[0], rsp.rates[1], rsp.rates[2]);
        }

        if paced {
            crate::flyctrl::pace::rate::wait_tick();
        } else {
            delay_until(&mut wake_tick, 1);
        }
    }
}
