//! ★★§5.204【每拍「旋钮读」计数】——补 §5.192 `cost_guard` 的**覆盖缺口** ✓。
//!
//! # 缺口（§5.201 实测 ✓）
//!
//! §5.192 的 `cost_guard` 只数**超越函数**（sin/cos/atan2…）✗ ⇒ **不覆盖 volatile 读/访存** ✗。
//! 而实测证明"每拍 `read_volatile`"**真实消耗控制预算**：§5.197 给推力补偿加**一个**每拍旋钮读
//! ⇒ 把 `x_env_motion::turn_yaw_rate_tracks` 从 4/0 推到 3/1（`est_omz` 0.087 vs 真值 0.5 ✗，
//! 与 §5.186 的预算类回归同签名 ✓）。控制任务卡在 4ms 边缘（§5.135/§5.161/§5.186 ✓）
//! ⇒ **调参旋钮不是免费的** ✓。
//!
//! # 做法
//!
//! 把所有"读运行时旋钮"的 `core::ptr::read_volatile(core::ptr::addr_of!(G_*))` 改走
//! [`knob_read`] ✓ —— feature 开时**顺带计数** ✓，feature 关时**与 `read_volatile` 等价**
//! （零开销 ✓）。由 `fly-sim-core` 的 `cost_guard` 断言"每拍读数 ≤ 基线" ✓。
//!
//! # 纪律
//!
//! 阈值 = **实测基线**。**新增每拍旋钮读会失败** ⇒ 必须显式回答"这个读是必要的吗？"
//! （若必要 ⇒ 更新基线并在台账说明 ✓；或把它移出热路径/改成编译期常量 ✓）。

/// 每拍旋钮读计数（仅 `cost-probe` / test 下存在 ✓）
#[cfg(any(test, feature = "cost-probe"))]
pub static mut COST_VREAD: u32 = 0;

#[cfg(any(test, feature = "cost-probe"))]
macro_rules! cost_vread {
    () => {
        unsafe {
            crate::cost::COST_VREAD = crate::cost::COST_VREAD.wrapping_add(1);
        }
    };
}
#[cfg(not(any(test, feature = "cost-probe")))]
macro_rules! cost_vread {
    () => {};
}

/// 读运行时旋钮（`read_volatile` 的计数替身 ✓）。
///
/// # Safety
///
/// 与 [`core::ptr::read_volatile`] 相同：`src` 必须合法、对齐，且读不被优化掉是**有意为之**。
#[inline(always)]
pub unsafe fn knob_read<T: Copy>(src: *const T) -> T {
    cost_vread!();
    core::ptr::read_volatile(src)
}

/// 复位/读取探针（仅 `cost-probe` / test 下存在 ✓）
#[cfg(any(test, feature = "cost-probe"))]
pub fn vread_reset() {
    unsafe { COST_VREAD = 0; }
}
#[cfg(any(test, feature = "cost-probe"))]
pub fn vread_count() -> u32 {
    unsafe { COST_VREAD }
}

#[cfg(test)]
mod tests {
    /// ★★§5.204【静态守卫 ✓】：热路径的**每拍旋钮读点**数必须 ≤ 基线。
    ///
    /// 为何用**静态**（而非只靠 `cost_guard` 的动态计数）：§5.201 的失效是**加了一个**每拍读
    /// （成本仅 ~2 周期 ✓）⇒ 动态计数的 ±1 分辨率太脆 ✗；而**代码点数**是确定性的 ✓
    /// ⇒ 直接编码"**别往控制热路径加运行时旋钮读**"这条规则 ✓。
    ///
    /// 基线 = 实测（§5.204 ✓）。新增会失败 ⇒ 必须显式回答"该读是否必要" ✓：
    ///   · 非必要 ⇒ 别加（或移出热路径 / 改成每 N 拍读一次的缓存值 ✓）
    ///   · 必要 ⇒ 更新基线 + 台账 §5.204 说明原因 ✓
    #[test]
    fn hot_path_knob_reads_bounded() {
        const FILES: &[(&str, &str, usize)] = &[
            ("controller/pid.rs", include_str!("controller/pid.rs"), 25),
            ("estimator/eskf.rs", include_str!("estimator/eskf.rs"), 22),
            ("estimator/eskf_estimator.rs", include_str!("estimator/eskf_estimator.rs"), 12),
            ("hil.rs", include_str!("hil.rs"), 2),
        ];
        for (name, src, base) in FILES {
            let n = src.matches("crate::cost::knob_read(").count();
            assert!(
                n <= *base,
                "{name}: 热路径旋钮读点 {n} 超基线 {base} ⇒ **新增了每拍 `read_volatile`** ✗\n\
                 （§5.201 实证：**加一个**就把 `x_env_motion::turn_yaw_rate_tracks` 推过 4ms 预算边缘 ✗）\n\
                 若必要 ⇒ 更新本表基线 + 台账 §5.204 说明；否则移出热路径或改成缓存值 ✓"
            );
        }
    }
}
