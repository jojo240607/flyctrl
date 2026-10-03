//! ★design.md §4：**L1 `control_allocator`**（独立硬实时线程，1kHz，预算 40µs）。
//!
//! 职责单一：消费 `rate_ctrl` 产出的**力矩/推力**（`RATE_TT`，PX4 `vehicle_torque_setpoint`
//! + `vehicle_thrust_setpoint` 同构）→ 控制分配（逐优先级去饱和，`attitude.rs` ✓）→ 执行器
//! （PWM）+ HIL 回传。与 `rate_ctrl` 各有独立 WCET 预算（design.md §4：80µs / 40µs ✓）。

use core::ffi::c_void;

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::{PidController, TorqueThrust};
use flyctrl_core::hal::actuator::clamp_thrust;
use flyctrl_core::vehicle::ActuatorCmd;
use rtos_app_sdk::device::Device;
use rtos_app_sdk::ioctl;
use rtos_app_sdk::rtos::delay_until;
use rtos_app_sdk::{info, warn};

use crate::flyctrl::rt_stat::RtStat;
use crate::flyctrl::{make_name, HIL_DIAG, RATE_TT};

/// L1 分配器最近一次 exec（cycles）——供 safety 汇总 L1 负载（§8 过载判定）。
pub static mut ALLOC_EXEC_CYC: u32 = 0;

pub extern "C" fn alloc_entry(_arg: *mut c_void) {
    info!(tag: "alloc", "task started; period=1ms (1kHz, L1 control_allocator)");
    // 分配只用 `mix_mode`（有效性矩阵/去饱和在 `attitude.rs` ✓），增益无关。
    let cfg = VehicleConfig::default_quad();
    let pid = PidController::from_config(&cfg.ctrl_params());

    // PWM 设备：本任务为**唯一写者**（从 `rate_task` 迁入 ⇒ 两线程职责单一 ✓）。
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
            warn!(tag: "alloc", "pwm{} not available -> actuator disabled", i);
        }
    }

    let mut wake_tick = rtos_app_sdk::rtos::tick_count();
    let mut st = RtStat::new("alloc");
    let mut it: u32 = 0;
    loop {
        let t0 = st.tick();
        // 取 `rate_ctrl` 最新力矩/推力。写者 prio=2 **高于**本任务 prio=3 ⇒ 读必为完整写 ✓
        // （同 `SENSOR_FRAME`/`RATE_CMD` 的优先级论证 ✓，无需 seqlock 重试）。
        let tt = unsafe {
            let t = &*core::ptr::addr_of!(RATE_TT);
            if t.valid != 0 {
                TorqueThrust { torque: t.torque, thrust: t.thrust, valid: true }
            } else {
                TorqueThrust::default() // valid=false ⇒ 分配器零输出 ✓
            }
        };
        let cmd = if crate::flyctrl::safety_task::SAFETY_KILL
            .load(core::sync::atomic::Ordering::Relaxed)
        {
            ActuatorCmd::zero() // safety_monitor：控制卡死/Critical ⇒ 停机
        } else {
            pid.allocate(&tt)
        };
        // 限幅 + 输出 PWM（硬件 TIM 比较匹配 ✓）。
        let mut motors = [0.0f32; 4];
        for i in 0..4 {
            motors[i] = clamp_thrust(cmd.motor[i]);
            if let Some(d) = &pwm_dev[i] {
                let us = 1000.0 + 1000.0 * motors[i];
                let ticks = (us * pwm_period[i] as f32 / 2500.0) as u32;
                let mut t = ticks;
                let _ = d.ioctl(ioctl::PWM_IOCTL_SET_DUTY_TICKS, &mut t as *mut u32 as *mut c_void);
            }
        }
        unsafe {
            (*core::ptr::addr_of_mut!(HIL_DIAG)).motor = motors;
        }
        // HIL：回传执行器（HIL_ACTUATOR_CONTROLS）。
        #[cfg(feature = "hil")]
        crate::flyctrl::uplink::set_actuator_cmd(&motors);

        let exec = st.tick().wrapping_sub(t0);
        unsafe { ALLOC_EXEC_CYC = exec; }
        let cpu = crate::flyctrl::rt_stat::cycles_per_us() * 1000;
        st.sample(exec, cpu, cpu); // 1kHz 名义
        it = it.wrapping_add(1);
        if it % 500 == 0 { st.report_and_reset(); } // ★design.md §9 可观测
        delay_until(&mut wake_tick, 1); // 1 tick = 1ms（独立 1kHz 网格 ✓）
    }
}
