//! 虚拟 IMU 驱动：从全局 `PLAYBACK` 读取回放数据集，伪装成真实 IMU（accel+gyro）。
//!
//! 与真实驱动（现为 `imu::bmi088::ImuBmi088` ✓）实现同一 `ImuSensor` trait，使 `sensors_task`
//! 只需切换数据源即可，无需改动采集逻辑。

use flyctrl_core::hal::sensor::ImuSensor;
use flyctrl_core::units::{MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::ImuSample;

use crate::sensors::sim::dataset::{Frame, PLAYBACK};

/// ★★**约定对照与修复**（2026-09-23，§5.43/§5.44 ✓）
///
/// **原实现的两处错** ✗：
/// ```text
/// let az = f.imu_accel[2] - GRAVITY;   // ✗✗
/// ```
/// ① **概念错** ✗：加速度计**测的就是比力** ✓ —— 数据集的值**已是比力**（加速度计原理 ✓），
///    再减 9.81 会把整个比力抹掉（实测 |accel| 仅 0.72 ✗，应为 9.81 ✓）。
/// ② **坐标系错** ✗：数据集为 **FLU 机体约定**（x 前 / y 左 / z 上 ✓；水平静止 z≈**+9.78** ✓），
///    而估计器（及 H 场 `TrajSample::specific_force_body()` ✓）按 **FRD/NED** 约定
///    （静止水平应 a_body ≈ [0,0,**−9.81**] ✓）⇒ 需 **y、z 取反** ✓（非 x↔y 交换 ✗）。
///
/// **与 H 场对齐** ✓：H 场 `a_body = Rᵀ(a_world − g)`，`g = [0,0,+9.81]`（NED ✓）
/// ⇒ 静止水平 = [0,0,−9.81] ✓ —— 本实现修好后同为此值 ✓。
pub struct VirtualImu;

impl VirtualImu {
    pub fn new() -> Option<Self> {
        Some(Self)
    }
}

impl ImuSensor for VirtualImu {
    fn read(&mut self) -> ImuSample {
        let f: Frame = unsafe { PLAYBACK.current() };
        // ★FLU → FRD（y、z 取反 ✓）；**不再减 GRAVITY** ✗（原值即比力 ✓）
        let ax = f.imu_accel[0];
        let ay = -f.imu_accel[1];
        let az = -f.imu_accel[2];
        // 陀螺同属 FLU ⇒ 同样 y、z 取反 ✓
        ImuSample {
            accel: [
                MeterPerSecondSquared(ax),
                MeterPerSecondSquared(ay),
                MeterPerSecondSquared(az),
            ],
            gyro: [
                RadianPerSecond(f.imu_gyro[0]),
                RadianPerSecond(-f.imu_gyro[1]),
                RadianPerSecond(-f.imu_gyro[2]),
            ],
        }
    }

    fn healthy(&self) -> bool {
        true
    }
}
