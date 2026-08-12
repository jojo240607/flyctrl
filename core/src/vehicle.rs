//! 飞行器运动状态与传感器测量值的数据结构。
//!
//! 这里只放"纯数据"，不含任何算法逻辑。状态估计与控制都围绕这些结构交换数据。

pub use crate::units::*;

/// 机体坐标系（前-X，右-Y，下-Z，右手系）下的姿态四元数。
///
/// 用四元数表示姿态，避免欧拉角万向锁，且数值积分稳定。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub const IDENTITY: Self = Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    /// 由 Z-Y-X 欧拉角（roll, pitch, yaw）构造，用于初始/测试。
    pub fn from_euler(roll: Radian, pitch: Radian, yaw: Radian) -> Self {
        let (sr, cr) = crate::math::sin_cos(roll.0);
        let (sp, cp) = crate::math::sin_cos(pitch.0);
        let (sy, cy) = crate::math::sin_cos(yaw.0);
        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    /// 归一化（积分后可能略微偏离单位四元数）。
    pub fn normalize(self) -> Self {
        let n = crate::math::sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z);
        if n < 1e-8 { return Self::IDENTITY; }
        Self { w: self.w / n, x: self.x / n, y: self.y / n, z: self.z / n }
    }

    /// 对机体角速度 (rad/s) 做四元数微分方程一步积分（显式欧拉）。
    /// 角速度为机体坐标系分量 (p, q, r)。
    pub fn integrate(self, p: f32, q: f32, r: f32, dt: f32) -> Self {
        // dq/dt = 0.5 * q ⊗ (0, ω)
        let w = self.w;
        let x = self.x;
        let y = self.y;
        let z = self.z;
        let dw = -0.5 * (x * p + y * q + z * r);
        let dx = 0.5 * (w * p + y * r - z * q);
        let dy = 0.5 * (w * q + z * p - x * r);
        let dz = 0.5 * (w * r + x * q - y * p);
        Self { w: w + dw * dt, x: x + dx * dt, y: y + dy * dt, z: z + dz * dt }.normalize()
    }
}

impl core::ops::Mul for Quaternion {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        quat_mul(self, rhs)
    }
}

/// 飞行器完整运动状态（世界系 NED：北-X，东-Y，下-Z）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleState {
    pub pos: [Meter; 3],          // 位置 (N, E, D)  注意 D 向下为正
    pub vel: [MeterPerSecond; 3], // 速度 (N, E, D)
    pub att: Quaternion,          // 姿态四元数（机体->世界）
    pub omega: [RadianPerSecond; 3], // 机体角速度 (p, q, r)
    /// 估计空速（m/s），由空速计测量（经 EKF 融合）；无空速计时为 0。
    pub airspeed: MeterPerSecond,
}

impl VehicleState {
    pub fn zero() -> Self {
        Self {
            pos: [Meter::ZERO, Meter::ZERO, Meter::ZERO],
            vel: [MeterPerSecond::ZERO, MeterPerSecond::ZERO, MeterPerSecond::ZERO],
            att: Quaternion::IDENTITY,
            omega: [RadianPerSecond::ZERO, RadianPerSecond::ZERO, RadianPerSecond::ZERO],
            airspeed: MeterPerSecond::ZERO,
        }
    }
}

/// NED 位置（北-X，东-Y，下-Z，右手系），单位 m。
///
/// 封装 `[Meter; 3]` 以明确 NED 语义，供估计器、FDIR、RTL 等模块统一使用。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ned(pub [Meter; 3]);

impl Ned {
    pub fn origin() -> Self {
        Ned([Meter::ZERO, Meter::ZERO, Meter::ZERO])
    }
    pub fn new(n: f32, e: f32, d: f32) -> Self {
        Ned([Meter(n), Meter(e), Meter(d)])
    }
    /// 水平距离（忽略 D 分量），单位 m。
    pub fn horizontal(&self, other: Ned) -> f32 {
        let dn = self.0[0].0 - other.0[0].0;
        let de = self.0[1].0 - other.0[1].0;
        libm::sqrtf(dn * dn + de * de)
    }
}

/// IMU 原始测量（含噪声由 world 层注入，这里只描述"干净值"语义）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub accel: [MeterPerSecondSquared; 3], // 机体加速度（不含重力，陀螺积分用）
    pub gyro: [RadianPerSecond; 3],        // 机体角速度 (p, q, r)
}

/// 高度计 / GPS 位置测量。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosSample {
    pub pos: [Meter; 3], // NED 位置
}

/// 空速计（皮托管 / 差分气压）单次测量：总压 - 静压差换算的真空速（IAS≈TAS，忽略空气压缩）。
/// 空速计测的是**气流相对机体的速度大小**，不含风。EKF 用它对水平速度幅值做约束。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedSample {
    pub speed: Airspeed,
    pub timestamp_s: f64,
}

/// 控制输出：四个电机的归一化推力指令 [0,1]。
/// 索引对应 X 型四旋翼：0=前右(CCW) 1=后左(CCW) 2=前左(CW) 3=后右(CW)。
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ActuatorCmd {
    pub motor: [f32; 4],
}

impl ActuatorCmd {
    pub fn zero() -> Self { Self { motor: [0.0; 4] } }
}

/// 遥控接收机解算后的归一化指令。
///
/// 所有通道已归一化到 `[-1, 1]`（油门 `[0, 1]`），摇杆中位为 0。
/// 由 Rust 应用层的 RC 驱动（SBUS/PPM/CRSF 等）解析后产出，控制律直接消费。
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RcInput {
    /// 副翼（右移为正，roll）。
    pub roll: f32,
    /// 升降（后拉为正，pitch）。
    pub pitch: f32,
    /// 方向舵（右移为正，yaw）。
    pub yaw: f32,
    /// 油门（[0,1]，推满为 1）。
    pub throttle: f32,
    /// 解锁/上锁（true=armed）。由固定通道的开关位决定。
    pub armed: bool,
    /// 模式开关（0/1/2 映射飞行模式槽位，由具体驱动解释）。
    pub mode: u8,
    /// 接收机链路健康（最近一帧在超时窗内收到）。
    pub fresh: bool,
}

impl RcInput {
    /// 中位、未解锁、无链路的安全默认值。
    pub fn neutral() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.0,
            armed: false,
            mode: 0,
            fresh: false,
        }
    }
}

/// 四元数旋转向量 v（q 为机体->世界旋转，返回世界系向量）。
pub fn rotate_vec_by_quat(q: Quaternion, v: [f32; 3]) -> [f32; 3] {
    let w = q.w; let x = q.x; let y = q.y; let z = q.z;
    let r00 = 1.0 - 2.0 * (y * y + z * z);
    let r01 = 2.0 * (x * y - w * z);
    let r02 = 2.0 * (x * z + w * y);
    let r10 = 2.0 * (x * y + w * z);
    let r11 = 1.0 - 2.0 * (x * x + z * z);
    let r12 = 2.0 * (y * z - w * x);
    let r20 = 2.0 * (x * z - w * y);
    let r21 = 2.0 * (y * z + w * x);
    let r22 = 1.0 - 2.0 * (x * x + y * y);
    [
        r00 * v[0] + r01 * v[1] + r02 * v[2],
        r10 * v[0] + r11 * v[1] + r12 * v[2],
        r20 * v[0] + r21 * v[1] + r22 * v[2],
    ]
}

/// 反向旋转（世界->机体）。
pub fn rotate_vec_by_quat_inverse(q: Quaternion, v: [f32; 3]) -> [f32; 3] {
    rotate_vec_by_quat(Quaternion { w: q.w, x: -q.x, y: -q.y, z: -q.z }, v)
}

/// 四元数共轭（单位四元数即逆）：机体->世界映射的反向。
pub fn quat_conj(q: Quaternion) -> Quaternion {
    Quaternion { w: q.w, x: -q.x, y: -q.y, z: -q.z }
}

/// 标准 Hamilton 积 q1 ⊗ q2。
pub fn quat_mul(q1: Quaternion, q2: Quaternion) -> Quaternion {
    Quaternion {
        w: q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
        x: q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
        y: q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
        z: q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
    }
}

impl Quaternion {
    /// 由旋转轴（不必单位化）与旋转角构造四元数（Rodrigues）。
    pub fn from_axis_angle(axis: [f32; 3], angle: Radian) -> Self {
        let n = crate::math::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
        if n < 1e-8 {
            return Self::IDENTITY;
        }
        let half = angle.0 * 0.5;
        let s = crate::math::sin(half) / n;
        Self {
            w: crate::math::cos(half),
            x: axis[0] * s,
            y: axis[1] * s,
            z: axis[2] * s,
        }
    }
}

