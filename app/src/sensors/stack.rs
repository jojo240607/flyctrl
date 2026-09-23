//! 传感器数据源统一封装（虚拟 / 真实 编译期切换）。
//!
//! 飞控算法、控制环、遥测层只依赖 `flyctrl_core::hal::sensor` 定义的四个 trait
//! （`ImuSensor` / `BaroSensor` / `GpsSensor` / `RcReceiver`，外加 `MagSensor`），
//! 不关心底层是虚拟回放还是真实 I2C/UART 驱动。
//!
//! 切换方式：编译期 `cfg(feature = "real-sensors")`。
//! - 默认（不开启）：四路全部使用虚拟源（`VirtualImu` 等），无需外接硬件即可闭环调试。
//! - 开启 `real-sensors` 后：四路自动替换为真实驱动（**`ImuBmi088`** / `BaroBmp280` /
//!   `GpsUblox` / `RcSbus`）。真实驱动读取失败时返回安全值并 `healthy()==false`，
//!   由上层 FDIR 降级，无需改算法层。
//!
//! 注：真实驱动始终编译进镜像（读取失败安全降级），仅在工厂处决定实例化哪套，
//! 后续接真实设备只需开启 feature，无需改动任何上层代码。

use flyctrl_core::hal::sensor::{
    BaroSensor, GpsSensor, ImuSensor, MagSensor, RcReceiver,
};
use flyctrl_core::vehicle::{ImuSample, PosSample, RcInput};

#[cfg(not(feature = "real-sensors"))]
pub type ImuSource = crate::sensors::sim::VirtualImu;
#[cfg(not(feature = "real-sensors"))]
pub type BaroSource = crate::sensors::sim::VirtualBaro;
#[cfg(not(feature = "real-sensors"))]
pub type GpsSource = crate::sensors::sim::VirtualGps;
#[cfg(not(feature = "real-sensors"))]
pub type RcSource = crate::sensors::sim::VirtualRc;
#[cfg(not(feature = "real-sensors"))]
pub type MagSource = crate::sensors::sim::VirtualMag;

#[cfg(feature = "real-sensors")]
pub type ImuSource = crate::sensors::imu::ImuBmi088;
#[cfg(feature = "real-sensors")]
pub type BaroSource = crate::sensors::baro::BaroBmp280;
#[cfg(feature = "real-sensors")]
pub type GpsSource = crate::sensors::gps::GpsUblox;
#[cfg(feature = "real-sensors")]
pub type RcSource = crate::sensors::rc::RcSbus;
#[cfg(feature = "real-sensors")]
pub type MagSource = crate::sensors::mag::MagQmc5883;

/// 四路数据源统一栈。字段类型为编译期别名，算法层只调用 trait 方法。
///
/// 全部字段为 `Option`：真实源探测/打开失败时对应槽位为 `None`（不 panic），
/// 读方法返回安全值、`health()` 报告 false → 上层 FDIR 降级（真机传感器缺失/
/// 断线语义：启动不崩溃，控制环/FDIR 按缺失传感器处理）。
pub struct SensorStack {
    pub imu: Option<ImuSource>,
    pub baro: Option<BaroSource>,
    pub gps: Option<GpsSource>,
    pub rc: Option<RcSource>,
    pub mag: Option<MagSource>,
}

impl SensorStack {
    pub fn new() -> Self {
        // 虚拟源：无参构造（始终成功）。
        #[cfg(not(feature = "real-sensors"))]
        {
            Self {
                imu: Some(crate::sensors::sim::VirtualImu::new()
                    .expect("virtual imu")),
                baro: Some(crate::sensors::sim::VirtualBaro::new()
                    .expect("virtual baro")),
                gps: Some(crate::sensors::sim::VirtualGps::new()
                    .expect("virtual gps")),
                rc: Some(crate::sensors::sim::VirtualRc::new()
                    .expect("virtual rc")),
                mag: Some(crate::sensors::sim::VirtualMag::new()
                    .expect("virtual mag")),
            }
        }
        // 真实源：需指定总线名 / I2C 地址；硬件缺失时 `new` 返回 None（启动即报错）。
        #[cfg(feature = "real-sensors")]
        {
            // 探测/打开失败 → None（传感器缺失/断线），不 panic；health() 报 false
            // 由 FDIR 降级（历史：旧 IMU 驱动唤醒写 NACK 直接 expect panic，启动即崩）。
            Self {
                imu: crate::sensors::imu::ImuBmi088::new("bmi088"),
                baro: crate::sensors::baro::BaroBmp280::new("i2c2", 0x76),
                gps: crate::sensors::gps::GpsUblox::new("uart1"),
                rc: crate::sensors::rc::RcSbus::new("uart2"),
                mag: crate::sensors::mag::MagQmc5883::new("i2c2", 0x0D),
            }
        }
    }

    /// 读取一帧 IMU（accel+gyro）；槽位缺失（探测失败）返回零值。
    pub fn read_imu(&mut self) -> ImuSample {
        self.imu.as_mut().map(|s| s.read()).unwrap_or(ImuSample {
            accel: [flyctrl_core::units::MeterPerSecondSquared(0.0); 3],
            gyro: [flyctrl_core::units::RadianPerSecond(0.0); 3],
        })
    }

    /// 读取磁力计；槽位缺失返回零值。
    pub fn read_mag(&mut self) -> [f32; 3] {
        self.mag.as_mut().map(|s| s.read()).unwrap_or([0.0; 3])
    }

    /// 读取气压高度（向上为正，米）；槽位缺失返回 0。
    pub fn read_altitude(&mut self) -> f32 {
        self.baro.as_mut().map(|s| s.read_altitude().0).unwrap_or(0.0)
    }

    /// 读取 GPS 位置（None 表示暂未定位 / 无数据 / 槽位缺失）。
    pub fn read_gps(&mut self) -> Option<PosSample> {
        self.gps.as_mut().and_then(|s| s.read())
    }

    /// 读取遥控输入；槽位缺失返回中性安全默认（不解锁）。
    pub fn read_rc(&mut self) -> RcInput {
        self.rc.as_mut().map(|s| s.read()).unwrap_or_else(RcInput::neutral)
    }

    /// 汇总各源健康状态（供 FDIR / 遥测健康位使用）；槽位缺失视为不健康。
    pub fn health(&self) -> SensorHealth {
        SensorHealth {
            imu: self.imu.as_ref().map(|s| s.healthy()).unwrap_or(false),
            baro: self.baro.as_ref().map(|s| s.healthy()).unwrap_or(false),
            gps: self.gps.as_ref().map(|s| s.healthy()).unwrap_or(false),
            rc: self.rc.as_ref().map(|s| s.healthy()).unwrap_or(false),
            mag: self.mag.as_ref().map(|s| s.healthy()).unwrap_or(false),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SensorHealth {
    pub imu: bool,
    pub baro: bool,
    pub gps: bool,
    pub rc: bool,
    pub mag: bool,
}
