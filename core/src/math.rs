//! no_std 数学函数 shim。
//!
//! 核心 crate 禁用 std，故 `f32` 的 `.sin_cos()`/`.asin()` 等方法不可用。
//! 这里统一转发到后端实现，控制层与估计层都从本模块取，避免在多处散落调用。
//!
//! 【为什么固件不用 libm】libm 0.2 的 `sinf`/`cosf`/`atan2f`/`logf`/`expf` 虽然
//! 接口是 f32（`...f` 后缀），但**内部实现走 f64 通用路径**（`rem_pio2f` /
//! `scalbn::<f64>`），会把整套 double 软件运行时链接进固件。F407 的 FPU 只支持
//! 单精度，每个 `__aeabi_dmul`/`__aeabi_dadd`/`__aeabi_dcmp*` 都是几十条指令的
//! 软件模拟。换 fpmath 后固件里 `bl __aeabi_d*` 从 63 处降到 **0**。
//!
//! 【后端按目标平台分离】（见 `core/Cargo.toml`）：
//!   - `target_os = "none"`（固件）→ `fpmath`（纯 f32，用 `soft-float` feature）
//!   - 其它（host / SIL / 单元测试）→ `libm`
//!
//! 两者精度承诺都优于 1 ULP，SIL 与固件的数值差异远小于其它误差源。
//!
//! 【历史误判已订正】曾把模拟器的 `UC_ERR_INSN_INVALID` 归因于"host_f32 发的
//! 硬件 FPU 编码不被 Unicorn/QEMU 支持"，因而退到纯软件 `soft-float`。真实原因
//! 是 `cargo vendor` 覆盖掉了 mcu_simulater 本地对 vendored QEMU 的补丁（见
//! `vendor/unicorn-engine-sys/qemu/target/arm/translate.c` 中 `gen_set_condexec`
//! 的 IT 状态回收），与 FPU 编码无关。补丁恢复后 `host_f32` 正常工作，且实测
//! 明显快于 libm（sensors 采样率 64.6Hz vs 41.7Hz），故用默认 `host_f32`。
//! `soft-float` 反而会拉进 rustc_apfloat/smallvec（需要 alloc，零堆 no_std 用不了）。

/// 通用钳位（与后端无关）。
pub fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

// ★★§5.192【计算量探针（H 场"预算类回归"前移 ✓）】：统计**每拍昂贵运算**调用数。
//
// 动机（§5.191 ✓）：控制任务卡在 4ms 预算边缘（§5.135 启用 ESKF 即超预算；
//   §5.161 仅加一条 16B store 就使 `x_hover_noise` 从 20° 到 142°；§5.186 每拍三角函数
//   使 `x_env_motion::turn_yaw_rate_tracks` 回归 ✗）。而这些**只有 M 场（指令级时序模型）
//   能发现** ✗ ⇒ 本探针把"每拍昂贵运算数"做成**H 场可断言**的代理量 ✓。
//
// 口径 ✓：`COST_TRANS` 只数**超越函数**（sin/cos/sin_cos/asin/atan2/exp/ln/pow —— M4F
//   上是多项式/软件实现，几十~上百条指令 ✓）；`COST_SQRT` 单列（M4F 有 VSQRT，1 条 ✓）。
// 编译期开关 ✓：`#[cfg(any(test, feature = "cost-probe"))]` ⇒ **默认固件零开销** ✓
//   （feature 关闭时宏展开为空 ✓）。
#[cfg(any(test, feature = "cost-probe"))]
pub static mut COST_TRANS: u32 = 0;
#[cfg(any(test, feature = "cost-probe"))]
pub static mut COST_SQRT: u32 = 0;

#[cfg(any(test, feature = "cost-probe"))]
macro_rules! cost_trans { () => { unsafe { crate::math::COST_TRANS = crate::math::COST_TRANS.wrapping_add(1); } }; }
#[cfg(not(any(test, feature = "cost-probe")))]
macro_rules! cost_trans { () => {}; }
#[cfg(any(test, feature = "cost-probe"))]
macro_rules! cost_sqrt { () => { unsafe { crate::math::COST_SQRT = crate::math::COST_SQRT.wrapping_add(1); } }; }
#[cfg(not(any(test, feature = "cost-probe")))]
macro_rules! cost_sqrt { () => {}; }

/// 复位/读取探针（仅 `cost-probe` 下存在 ✓）
#[cfg(any(test, feature = "cost-probe"))]
pub fn cost_reset() {
    unsafe { COST_TRANS = 0; COST_SQRT = 0; }
}
#[cfg(any(test, feature = "cost-probe"))]
pub fn cost_read() -> (u32, u32) {
    unsafe { (COST_TRANS, COST_SQRT) }
}

#[cfg(target_os = "none")]
mod backend {
    use fpmath::FloatMath;

    pub fn sin(x: f32) -> f32 {
        cost_trans!();
        fpmath::sin(x)
    }
    pub fn cos(x: f32) -> f32 {
        cost_trans!();
        fpmath::cos(x)
    }
    pub fn sin_cos(x: f32) -> (f32, f32) {
        cost_trans!();
        fpmath::sin_cos(x)
    }
    pub fn asin(x: f32) -> f32 {
        cost_trans!();
        fpmath::asin(x)
    }
    pub fn atan2(y: f32, x: f32) -> f32 {
        cost_trans!();
        fpmath::atan2(y, x)
    }
    pub fn sqrt(x: f32) -> f32 {
        cost_sqrt!();
        fpmath::sqrt(x)
    }
    pub fn ln(x: f32) -> f32 {
        cost_trans!();
        fpmath::log(x)
    }
    pub fn exp(x: f32) -> f32 {
        cost_trans!();
        fpmath::exp(x)
    }
    pub fn abs(x: f32) -> f32 {
        fpmath::abs(x)
    }
    pub fn round(x: f32) -> f32 {
        fpmath::round(x)
    }
    pub fn pow(x: f32, y: f32) -> f32 {
        cost_trans!();
        fpmath::pow(x, y)
    }
}

#[cfg(not(target_os = "none"))]
mod backend {
    pub fn sin(x: f32) -> f32 {
        cost_trans!();
        libm::sinf(x)
    }
    pub fn cos(x: f32) -> f32 {
        cost_trans!();
        libm::cosf(x)
    }
    pub fn sin_cos(x: f32) -> (f32, f32) {
        cost_trans!();
        (libm::sinf(x), libm::cosf(x))
    }
    pub fn asin(x: f32) -> f32 {
        cost_trans!();
        libm::asinf(x)
    }
    pub fn atan2(y: f32, x: f32) -> f32 {
        cost_trans!();
        libm::atan2f(y, x)
    }
    pub fn sqrt(x: f32) -> f32 {
        cost_sqrt!();
        libm::sqrtf(x)
    }
    pub fn ln(x: f32) -> f32 {
        cost_trans!();
        libm::logf(x)
    }
    pub fn exp(x: f32) -> f32 {
        cost_trans!();
        libm::expf(x)
    }
    pub fn abs(x: f32) -> f32 {
        libm::fabsf(x)
    }
    pub fn round(x: f32) -> f32 {
        libm::roundf(x)
    }
    pub fn pow(x: f32, y: f32) -> f32 {
        cost_trans!();
        libm::powf(x, y)
    }
}

pub use backend::*;
