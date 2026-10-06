//! [PERF] 固件侧性能探针：把热点函数的分段号写到固定 VMA，供宿主机（测试）经
//! block hook 读两次翻号之间的退休指令差，从而把单拍耗时归到具体分段。
//!
//! 地址由 `joc-rtos-app-sdk/linker/app.ld` 的 `.app_hilprobe` 节钉死，测试侧常量直读。
//!
//! ## ★★host 侧必须是空实现（`target_os != "none"`）
//!
//! 探针本体读 **DWT 的 MMIO 地址 `0xE000_1004`** —— 该地址**只在 Cortex-M 上有意义**。
//! host（SIL / `cargo test`）上读它会**直接段错误** ✗：
//! 实测 `estimator::eskf::tests::bg_learning_converges_to_true_gyro_bias` SIGSEGV
//! @ `perf.rs:41`（调用栈 `probe(22) ← Eskf::predict`）⇒ **整个估计器单测族**
//! （含 `c1_*` / `c2_*` / FD 裁判这些数值护栏 ✗）都跑不了。
//! 故：**裸机**走真实现，**host** 编译为**零开销空实现** ✓。
//!
//! 分段号分配（跨模块唯一）：
//! - `0..=7`：`hil.rs` 的 `step_hil` / `ekf_hil` 外层分段
//! - `8..=18`：`estimator/eskf_estimator.rs` + `hil.rs` 的 **ESKF 分段**
//! - `19..=21`：`eskf.rs::update_gps_pos` 内部分段
//! - `22..=23, 32`：`eskf.rs::Eskf::predict`（F 构建 / Q 构建 / 协方差传播）
//! - `24..=31`：`eskf.rs::update_vec3`（Joseph 形式）逐步细分
//! - `40..=46`：`app/src/flyctrl/control.rs` 姿态项分段
//!   ★**必须唯一** ✓，且每个 id 一拍内只出现一次 ✓
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
///
/// ★2026-10-05【容量 24→48】：原容量 24 使 `control.rs` 的 40..46 段**全被静默丢弃** ✗
///   （`probe` 的 `last < CAP` 守卫跳过 ✗）⇒ 姿态项相位一直看不见。
///   同时为 ESKF 内部细分（`predict` 的 F/协方差、`update_vec3` 的 8 小段）腾出号段 ✓。
#[used]
#[no_mangle]
pub static mut STAGE_CYC: [u32; 48] = [0; 48];
#[used]
#[no_mangle]
pub static mut STAGE_N: [u32; 48] = [0; 48];

/// ★★【仪器验证用汇点】—— 扰动实验里用来**消费**"多加的那一遍"的计算结果，
///   防止 LLVM 把纯函数调用 DCE 掉（否则扰动量=0，验证失效 ✗）。
///   验证完即应移除调用点（本静态体量小、无副作用，留着也无害 ✓）。
#[used]
#[no_mangle]
pub static mut PERF_SINK: [f32; 4] = [0.0; 4];

#[cfg(target_os = "none")]
static mut LAST_STAGE: u32 = 0xFF;
#[cfg(target_os = "none")]
static mut LAST_CYC: u32 = 0;

#[cfg(target_os = "none")]
#[inline(always)]
fn dwt_cyccnt() -> u32 {
    // 0xE000_1004 = DWT->CYCCNT ✓
    unsafe { core::ptr::read_volatile(0xE000_1004 as *const u32) }
}

/// 写分段探针（热路径）—— **裸机实现** ✓
#[cfg(target_os = "none")]
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
        if last < 48 {
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

/// 写分段探针 —— **host 侧空实现**（见模块头：host 读 DWT MMIO 会段错误 ✗）
#[cfg(not(target_os = "none"))]
#[inline(always)]
pub fn probe(_stage: u32) {}
