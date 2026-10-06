//! L0 · 数学 shim —— 全模块唯一数学入口。
//!
//! 移植到 `no_std`（`target_os = "none"`）时只改本文件，其余模块一行不改，
//! 使"数学实现"与"算法约定"不再纠缠。

#[inline]
pub fn sin(x: f32) -> f32 {
    x.sin()
}
#[inline]
pub fn cos(x: f32) -> f32 {
    x.cos()
}
#[inline]
pub fn asin(x: f32) -> f32 {
    x.asin()
}
#[inline]
pub fn atan2(y: f32, x: f32) -> f32 {
    y.atan2(x)
}
#[inline]
pub fn sqrt(x: f32) -> f32 {
    x.sqrt()
}
