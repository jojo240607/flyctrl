//! [PERF] 固件侧性能探针：把热点函数的分段号写到固定 VMA，供宿主机（测试）经
//! block hook 读两次翻号之间的退休指令差，从而把单拍耗时归到具体分段。
//!
//! 地址由 `joc-rtos-app-sdk/linker/app.ld` 的 `.app_hilprobe` 节钉死，测试侧常量直读。
//! 分段号分配（跨模块唯一）：
//! - `1..=7`：`hil.rs` 的 `step_hil` 外层分段
//! - `8..=11`：`estimator/ekf.rs` 的 `EkfEstimator::step` 内部分段（Legacy ✓）
//! - **`8..=18`（现役 ✓）**：`estimator/eskf_estimator.rs` + `hil.rs` 的 **ESKF 分段** ✓
//!   8 step 进入 · 9 predict 完 · 10 重力完 · 11 GPS位前 · 12 GPS位后 · 13 GPS速后
//!   14 空速后 · 15 state 前 · 16 气压前 · 17 气压后 · 18 磁前 ✓
//!   ★**必须连续编号** ✓（Legacy 用 8..11 ✓；照其方式 ✓），且每个 id 一拍内只出现一次 ✓
//!
//! 写探针的开销是两次 32 位存 + 一次比较，相对被测分段（数十微秒以上）可忽略。

/// `[0]` = 当前分段号，`[1]` = 调用计数（段号 0 时自增，用于确认采样有效性）。
#[used]
#[link_section = ".app_hilprobe"]
pub static mut PROBE: [u32; 2] = [0, 0];

/// ★★2026-10-04【相位剖分（固件侧自计，ELFSYM 直读 ✓）】：
///   每段累计 DWT 周期 + 调用次数 ⇒ 宿主一次读回即可得**每相位精确耗时** ✓
///   （此前只能"读两次翻号之间的退休指令差"⇒ 需要宿主高频采样 ✗）。
///   读 DWT->CYCCNT（Cortex-M 固定地址 ✓，仿真器已把它作为计时器 ✓，零副作用 ✓）。
#[used]
#[no_mangle]
pub static mut STAGE_CYC: [u32; 24] = [0; 24];
#[used]
#[no_mangle]
pub static mut STAGE_N: [u32; 24] = [0; 24];
static mut LAST_STAGE: u32 = 0xFF;
static mut LAST_CYC: u32 = 0;

#[inline(always)]
fn dwt_cyccnt() -> u32 {
    // 0xE000_1004 = DWT->CYCCNT ✓
    unsafe { core::ptr::read_volatile(0xE000_1004 as *const u32) }
}

/// 写分段探针（热路径）。
#[inline(always)]
pub fn probe(stage: u32) {
    unsafe {
        let p = core::ptr::addr_of_mut!(PROBE);
        (*p)[0] = stage;
        if stage == 0 {
            (*p)[1] = (*p)[1].wrapping_add(1);
        }
        // 把**上一段**的耗时记到上一段的账上 ✓
        let now = dwt_cyccnt();
        let last = LAST_STAGE;
        if last < 24 {
            let d = now.wrapping_sub(LAST_CYC);
            let sc = core::ptr::addr_of_mut!(STAGE_CYC);
            (*sc)[last as usize] = (*sc)[last as usize].wrapping_add(d);
            let sn = core::ptr::addr_of_mut!(STAGE_N);
            (*sn)[last as usize] = (*sn)[last as usize].wrapping_add(1);
        }
        LAST_STAGE = stage;
        LAST_CYC = now;
    }
}
