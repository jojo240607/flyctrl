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

#[cfg(target_os = "none")]
mod backend {
    use fpmath::FloatMath;

    pub fn sin(x: f32) -> f32 {
        fpmath::sin(x)
    }
    pub fn cos(x: f32) -> f32 {
        fpmath::cos(x)
    }
    pub fn sin_cos(x: f32) -> (f32, f32) {
        fpmath::sin_cos(x)
    }
    pub fn asin(x: f32) -> f32 {
        fpmath::asin(x)
    }
    pub fn atan2(y: f32, x: f32) -> f32 {
        fpmath::atan2(y, x)
    }
    pub fn sqrt(x: f32) -> f32 {
        fpmath::sqrt(x)
    }
    pub fn ln(x: f32) -> f32 {
        fpmath::log(x)
    }
    pub fn exp(x: f32) -> f32 {
        fpmath::exp(x)
    }
    pub fn abs(x: f32) -> f32 {
        fpmath::abs(x)
    }
    pub fn round(x: f32) -> f32 {
        fpmath::round(x)
    }
    pub fn pow(x: f32, y: f32) -> f32 {
        fpmath::pow(x, y)
    }
}

#[cfg(not(target_os = "none"))]
mod backend {
    pub fn sin(x: f32) -> f32 {
        libm::sinf(x)
    }
    pub fn cos(x: f32) -> f32 {
        libm::cosf(x)
    }
    pub fn sin_cos(x: f32) -> (f32, f32) {
        (libm::sinf(x), libm::cosf(x))
    }
    pub fn asin(x: f32) -> f32 {
        libm::asinf(x)
    }
    pub fn atan2(y: f32, x: f32) -> f32 {
        libm::atan2f(y, x)
    }
    pub fn sqrt(x: f32) -> f32 {
        libm::sqrtf(x)
    }
    pub fn ln(x: f32) -> f32 {
        libm::logf(x)
    }
    pub fn exp(x: f32) -> f32 {
        libm::expf(x)
    }
    pub fn abs(x: f32) -> f32 {
        libm::fabsf(x)
    }
    pub fn round(x: f32) -> f32 {
        libm::roundf(x)
    }
    pub fn pow(x: f32, y: f32) -> f32 {
        libm::powf(x, y)
    }
}

pub use backend::*;
