//! 传感器 HAL：IMU / GPS / 气压计 / 罗盘 的统一采集接口。
//!
//! 四种传感器各自实现独立 trait，主循环按需 `read()` 取最新样本。
//! 每个 `read` 都是无堆、有界耗时的（真实硬件走 DMA/中断缓冲，这里语义一致）。
//!
//! 具体实现：
//! - [`mock`]：host 端确定性 mock（SIL 单测用）。
//! - [`stm32f407`]：结构对齐 STM32F4 外设布局的占位实现（编译期 `cfg` 门控，
//!   真实 PAC 接入时替换内部寄存器访问即可，算法层零改动）。

use crate::units::*;
use crate::vehicle::{AirspeedSample, ImuSample, PosSample, RcInput, RtkSample, VioSample};

/// IMU（陀螺 + 加速度计）传感器。
pub trait ImuSensor {
    /// 读取一帧最新 IMU 样本（机体坐标：accel 不含重力，gyro 为 p,q,r）。
    fn read(&mut self) -> ImuSample;
    /// 自检测（返回 true 表示健康）。
    fn healthy(&self) -> bool;
}

/// GPS / 位置源（也可由视觉/光流替代）。
pub trait GpsSensor {
    /// 读取最新 NED 位置；无锁定时返回 `None`。
    fn read(&mut self) -> Option<PosSample>;
    /// 自检测。
    fn healthy(&self) -> bool;
}

/// 气压高度计（提供 D 轴高度，融合进 PosSample 的 z 分量）。
pub trait BaroSensor {
    /// 读取相对起飞点的高度（m，向下为正）。
    fn read_altitude(&mut self) -> Meter;
    fn healthy(&self) -> bool;
}

/// 磁力计（提供机体系磁场向量，用于 yaw 观测 / 罗盘）。
pub trait MagSensor {
    /// 读取机体系磁场向量（任意单位，归一化用）。
    fn read(&mut self) -> [f32; 3];
    fn healthy(&self) -> bool;
}

/// 空速管 / 差分气压（测量总压 - 静压差换算真空速，不含风）。
pub trait AirspeedSensor {
    /// 读取一次空速样本；故障/无效时返回 `None`。
    fn read(&mut self) -> Option<AirspeedSample>;
    fn healthy(&self) -> bool;
}

/// 视觉里程计（VIO）：机载相机 + IMU 融合出的高频相对位置/速度。
pub trait VioSensor {
    /// 读取最新 VIO 样本（NED）；故障/无锁定时返回 `None`。
    fn read(&mut self) -> Option<VioSample>;
    /// 自检测。
    fn healthy(&self) -> bool;
}

/// RTK-GPS：载波相位差分厘米级高精度位置（NED）。
pub trait RtkSensor {
    /// 读取最新 RTK 位置；无固定解/无锁定时返回 `None`。
    fn read(&mut self) -> Option<RtkSample>;
    /// 自检测。
    fn healthy(&self) -> bool;
}

/// 遥控接收机（SBUS / PPM / CRSF 等）。输出已归一化的 [`RcInput`]。
pub trait RcReceiver {
    /// 读取最新一帧遥控指令；链路丢帧时 `RcInput.fresh` 为 false。
    fn read(&mut self) -> RcInput;
    /// 自检测。
    fn healthy(&self) -> bool;
}

// ─────────────────────────────────────────────────────────────
// Mock 实现（host / SIL）
// ─────────────────────────────────────────────────────────────

/// 确定性 mock IMU：输出近零噪声的小幅振荡，模拟悬停姿态。
pub struct MockImu {
    healthy: bool,
    t: f32,
}

impl MockImu {
    pub fn new() -> Self { Self { healthy: true, t: 0.0 } }
    pub fn set_health(&mut self, h: bool) { self.healthy = h; }
}

impl Default for MockImu { fn default() -> Self { Self::new() } }

impl ImuSensor for MockImu {
    fn read(&mut self) -> ImuSample {
        self.t += 0.005;
        // 悬停：加速度计受重力（机体 -Z 向下）≈ +g 在 z；轻微振荡模拟振动。
        let vib = crate::math::sin(self.t * 120.0) * 0.02;
        ImuSample {
            accel: [
                MeterPerSecondSquared(vib),
                MeterPerSecondSquared(vib * 0.8),
                MeterPerSecondSquared(9.81 + vib),
            ],
            gyro: [
                RadianPerSecond(crate::math::sin(self.t * 0.7) * 0.01),
                RadianPerSecond(crate::math::cos(self.t * 0.9) * 0.01),
                RadianPerSecond(crate::math::sin(self.t * 1.1) * 0.005),
            ],
        }
    }
    fn healthy(&self) -> bool { self.healthy }
}

/// Mock GPS：始终锁定，返回慢漂位置。
pub struct MockGps { base: [Meter; 3], t: f32, healthy: bool }

impl MockGps {
    pub fn new() -> Self {
        Self { base: [Meter(0.0); 3], t: 0.0, healthy: true }
    }
    pub fn set_health(&mut self, h: bool) { self.healthy = h; }
}

impl Default for MockGps { fn default() -> Self { Self::new() } }

impl GpsSensor for MockGps {
    fn read(&mut self) -> Option<PosSample> {
        if !self.healthy { return None; }
        self.t += 0.005;
        Some(PosSample::pos_only([
            Meter(self.base[0].0 + crate::math::sin(self.t * 0.3) * 0.1),
            Meter(self.base[1].0 + crate::math::cos(self.t * 0.4) * 0.1),
            Meter(self.base[2].0),
        ]))
    }
    fn healthy(&self) -> bool { self.healthy }
}

/// Mock 气压计：恒定高度 0（起飞点）。
pub struct MockBaro { alt: Meter, healthy: bool }
impl MockBaro {
    pub fn new() -> Self { Self { alt: Meter(0.0), healthy: true } }
    pub fn set_health(&mut self, h: bool) { self.healthy = h; }
}
impl Default for MockBaro { fn default() -> Self { Self::new() } }
impl BaroSensor for MockBaro {
    fn read_altitude(&mut self) -> Meter { self.alt }
    fn healthy(&self) -> bool { self.healthy }
}

/// Mock 罗盘：指向北（世界 -Z 向上为地磁，机体系近似固定向量）。
pub struct MockMag { healthy: bool }
impl MockMag { pub fn new() -> Self { Self { healthy: true } } }
impl Default for MockMag { fn default() -> Self { Self::new() } }
impl MagSensor for MockMag {
    fn read(&mut self) -> [f32; 3] { [0.1, 0.0, 0.99] }
    fn healthy(&self) -> bool { self.healthy }
}

/// Mock 空速管：恒定空速 `speed`（m/s），可设故障。
#[derive(Clone)]
pub struct MockAirspeed {
    speed: f32,       // 真空速 m/s（不含风）
    healthy: bool,
    t: f32,
}

impl MockAirspeed {
    pub fn new(speed: f32) -> Self {
        Self { speed, healthy: true, t: 0.0 }
    }
    /// 设恒定的模拟空速（用于驱动 EKF 融合测试）。
    pub fn set_speed(&mut self, v: f32) { self.speed = v; }
    pub fn set_health(&mut self, h: bool) { self.healthy = h; }
}

impl Default for MockAirspeed {
    fn default() -> Self { Self::new(0.0) }
}

impl AirspeedSensor for MockAirspeed {
    fn read(&mut self) -> Option<AirspeedSample> {
        if !self.healthy { return None; }
        self.t += 0.005;
        // 轻微抖动模拟风噪，量级 ~0.05 m/s。
        let noise = crate::math::sin(self.t * 50.0) * 0.05;
        Some(AirspeedSample {
            speed: Airspeed((self.speed + noise).max(0.0)),
            timestamp_s: self.t as f64,
        })
    }
    fn healthy(&self) -> bool { self.healthy }
}

/// Mock VIO：高更新率，位置带缓慢累积漂移 + 白噪声，速度带小噪声。
///
/// 默认位置漂移率 ~0.02 m/s（长期累积）、噪声 ~0.3m；速度噪声 ~0.05 m/s。
/// 通过 `set_drift` / `set_health` 可注入漂移/故障，供 EKF 融合单测使用。
pub struct MockVio {
    /// 位置漂移累积（m），模拟 VIO 无绝对参考的长期漂移。
    drift: [f32; 3],
    /// 位置白噪声幅值（m）。
    pos_noise: f32,
    /// 速度噪声幅值（m/s）。
    vel_noise: f32,
    healthy: bool,
    t: f32,
}

impl MockVio {
    pub fn new() -> Self {
        Self { drift: [0.0; 3], pos_noise: 0.3, vel_noise: 0.05, healthy: true, t: 0.0 }
    }
    /// 设置位置漂移速率（m/s，各轴），模拟 VIO 长期漂移累积。
    pub fn set_drift(&mut self, d: [f32; 3]) { self.drift = d; }
    pub fn set_health(&mut self, h: bool) { self.healthy = h; }
}

impl Default for MockVio { fn default() -> Self { Self::new() } }

impl VioSensor for MockVio {
    fn read(&mut self) -> Option<VioSample> {
        if !self.healthy { return None; }
        self.t += 0.005;
        // 漂移按时间线性累积（VIO 无绝对参考）。
        let drift = [self.drift[0] * self.t, self.drift[1] * self.t, self.drift[2] * self.t];
        let w = |amp: f32| crate::math::sin(self.t * 37.0) * amp;
        Some(VioSample::with_vel(
            [
                Meter(drift[0] + w(self.pos_noise)),
                Meter(drift[1] + w(self.pos_noise * 0.8)),
                Meter(drift[2] + w(self.pos_noise * 0.6)),
            ],
            [
                MeterPerSecond(w(self.vel_noise)),
                MeterPerSecond(w(self.vel_noise * 0.8)),
                MeterPerSecond(w(self.vel_noise * 0.6)),
            ],
        ))
    }
    fn healthy(&self) -> bool { self.healthy }
}

/// Mock RTK-GPS：厘米级高精度位置（噪声 ~0.05m），低更新率（1Hz 由调用方控制）。
pub struct MockRtk {
    base: [Meter; 3],
    noise: f32,
    healthy: bool,
    t: f32,
}

impl MockRtk {
    pub fn new() -> Self {
        Self { base: [Meter(0.0); 3], noise: 0.05, healthy: true, t: 0.0 }
    }
    pub fn set_noise(&mut self, n: f32) { self.noise = n; }
    pub fn set_health(&mut self, h: bool) { self.healthy = h; }
}

impl Default for MockRtk { fn default() -> Self { Self::new() } }

impl RtkSensor for MockRtk {
    fn read(&mut self) -> Option<RtkSample> {
        if !self.healthy { return None; }
        self.t += 0.005;
        // 厘米级噪声（0.05m 量级）。
        let w = |amp: f32| crate::math::sin(self.t * 41.0) * amp;
        Some(RtkSample::new([
            Meter(self.base[0].0 + w(self.noise)),
            Meter(self.base[1].0 + w(self.noise * 0.8)),
            Meter(self.base[2].0 + w(self.noise * 0.6)),
        ]))
    }
    fn healthy(&self) -> bool { self.healthy }
}

// ─────────────────────────────────────────────────────────────
// STM32F407 占位实现
//
// 真实落地时，这里替换为对 PAC（如 stm32f4xx-hal）的调用；
// 寄存器布局与 DMA 流已在 joc-base 项目中验证（见项目 memory），
// 此处仅保留结构骨架，保证算法层零改动即可移植。
// ─────────────────────────────────────────────────────────────

#[cfg(feature = "stm32f407")]
pub mod stm32f407 {
    use super::*;

    /// STM32F4 上的 ICM-20602（SPI1）IMU 读取封装。
    /// 真实代码走 `dma_hal` 双缓冲 + 中断取数；此处占位。
    pub struct Icm20602 { base: usize, ok: bool }
    impl Icm20602 {
        /// `base` 为外设/总线映射地址（真实板级由 PAC 提供）。
        pub const fn new(base: usize) -> Self { Self { base, ok: true } }
    }
    impl ImuSensor for Icm20602 {
        fn read(&mut self) -> ImuSample {
            // 占位：真实实现从 SPI RX FIFO 解包 6 轴原始值并标定。
            // 标定与单位换算在 driver 层完成，这里只返回语义正确的样本。
            let _ = self.base;
            ImuSample {
                accel: [MeterPerSecondSquared(0.0); 3],
                gyro: [RadianPerSecond(0.0); 3],
            }
        }
        fn healthy(&self) -> bool { self.ok }
    }

    /// STM32F4 上的 u-blox M8N（UART+DMA）GPS。
    pub struct UbloxM8n { base: usize, ok: bool }
    impl UbloxM8n {
        pub const fn new(base: usize) -> Self { Self { base, ok: true } }
    }
    impl GpsSensor for UbloxM8n {
        fn read(&mut self) -> Option<PosSample> {
            let _ = self.base;
            // 占位：解析 UBX-NAV-POSLLH -> NED。
            None
        }
        fn healthy(&self) -> bool { self.ok }
    }

    /// STM32F4 上的 SPL06（I2C）气压计。
    pub struct Spl06 { ok: bool }
    impl Spl06 { pub const fn new() -> Self { Self { ok: true } } }
    impl BaroSensor for Spl06 {
        fn read_altitude(&mut self) -> Meter { Meter(0.0) }
        fn healthy(&self) -> bool { self.ok }
    }

    /// STM32F4 上的 QMC5883L（I2C）罗盘。
    pub struct Qmc5883l { ok: bool }
    impl Qmc5883l { pub const fn new() -> Self { Self { ok: true } } }
    impl MagSensor for Qmc5883l {
        fn read(&mut self) -> [f32; 3] { [0.0; 3] }
        fn healthy(&self) -> bool { self.ok }
    }

    /// STM32F4 上的 MS4525DO（I2C）差分气压空速管。
    pub struct PitotMs4525do { ok: bool }
    impl PitotMs4525do { pub const fn new() -> Self { Self { ok: true } } }
    impl AirspeedSensor for PitotMs4525do {
        fn read(&mut self) -> Option<AirspeedSample> {
            let _ = self.ok;
            // 占位：读取差分 ADC -> 动压 -> 空速 = sqrt(2·Δp/ρ_air)。
            None
        }
        fn healthy(&self) -> bool { self.ok }
    }

    /// STM32F4 上的 VIO 源（外部视觉处理器的 UART 输出）占位。
    pub struct VioAux { ok: bool }
    impl VioAux { pub const fn new() -> Self { Self { ok: true } } }
    impl VioSensor for VioAux {
        fn read(&mut self) -> Option<VioSample> {
            let _ = self.ok;
            // 占位：解析视觉处理器输出的位姿/速度流。
            None
        }
        fn healthy(&self) -> bool { self.ok }
    }

    /// STM32F4 上的 RTK-GPS（双天线 / 差分接收机 UART）占位。
    pub struct RtkUblox { ok: bool }
    impl RtkUblox { pub const fn new() -> Self { Self { ok: true } } }
    impl RtkSensor for RtkUblox {
        fn read(&mut self) -> Option<RtkSample> {
            let _ = self.ok;
            // 占位：解析 RTCM -> NED 厘米级位置。
            None
        }
        fn healthy(&self) -> bool { self.ok }
    }
}
