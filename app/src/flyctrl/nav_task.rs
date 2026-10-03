//! ★design.md **L3 `wq:l3` WorkItem**：位置/速度外环 → 姿态设定点 `ATT_SP`（50Hz）。
//!
//! 由 L3 工作队列 worker 调用（**不是线程**；L3 只有 1 个 worker/1 个栈 ✓）。
//! 状态放模块级静态（WorkItem 无自己的栈）。

use core::mem::MaybeUninit;

use flyctrl_core::config::VehicleConfig;
use flyctrl_core::controller::PidController;
use flyctrl_core::units::Second;

use rtos_app_sdk::info;

use crate::flyctrl::{ATT_SP, EST_STATE, SETPOINT};

static mut NAV_PID: MaybeUninit<PidController> = MaybeUninit::uninit();
static mut NAV_FIRST: bool = false;

/// ★装配（在 setup 任务里调用一次）。
pub fn nav_init() {
    unsafe {
        NAV_PID.write(PidController::from_config(&VehicleConfig::default_quad().ctrl_params()));
        NAV_FIRST = true;
    }
    info!(tag: "nav", "nav item init (L3 wq:l3, 50Hz)");
}

/// ★L3 WorkItem：**一拍**（由工作队列 worker 调用）。
pub fn nav_step() {
    // ★design.md §8：过载等级 ≥ 1 ⇒ 降级本项
    if crate::flyctrl::safety_task::OVERLOAD_LEVEL.load(core::sync::atomic::Ordering::Relaxed) >= 1 { return; }
    let pid = unsafe { NAV_PID.assume_init_mut() };
    // 设定点（暂由 L2 attitude 构造/发布 ✓）+ 估计（L2 estimator 发布 ✓）。
    let (sp, sp_valid) = unsafe {
        let s = &*core::ptr::addr_of!(SETPOINT);
        (s.sp, s.valid != 0)
    };
    let est = unsafe { (*core::ptr::addr_of!(EST_STATE)).est };

    crate::flyctrl::uplink::sync_gains_to_pid(pid);

    // 位置/速度外环 → 期望姿态 + 总推力。
    let (q_des, thrust) = pid.outer_step(Second(0.02), &sp, &est);
    unsafe {
        let a = &mut *core::ptr::addr_of_mut!(ATT_SP);
        a.q = [q_des.w, q_des.x, q_des.y, q_des.z];
        a.thrust = thrust;
        a.valid = if sp_valid { 1 } else { 0 };
    }

    let first = unsafe { NAV_FIRST };
    if first {
        unsafe { NAV_FIRST = false; }
        info!(tag: "nav", "first loop done; sp_valid={}", sp_valid);
    }
}
