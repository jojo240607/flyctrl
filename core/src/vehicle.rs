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
    /// 标准公式使用半角：R = Rz(yaw)·Ry(pitch)·Rx(roll)。
    pub fn from_euler(roll: Radian, pitch: Radian, yaw: Radian) -> Self {
        let (sr, cr) = crate::math::sin_cos(roll.0 * 0.5);
        let (sp, cp) = crate::math::sin_cos(pitch.0 * 0.5);
        let (sy, cy) = crate::math::sin_cos(yaw.0 * 0.5);
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

    /// 由四元数（机体->世界，Z-Y-X 约定）提取 yaw（航向角，弧度）。
    /// 公式：yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))。
    pub fn yaw(self) -> f32 {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        crate::math::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    }

    /// yaw 转成度（地面站 heading 字段用）。
    pub fn yaw_deg(self) -> f32 {
        self.yaw() * 180.0 / core::f32::consts::PI
    }

    /// 由四元数（机体->世界，Z-Y-X 约定）提取 roll（横滚角，弧度）。
    /// 公式：roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))。
    pub fn roll(self) -> f32 {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        crate::math::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    }

    /// 由四元数（机体->世界，Z-Y-X 约定）提取 pitch（俯仰角，弧度）。
    /// 公式：pitch = asin(2*(w*y - z*x))，夹取到 [-pi/2, pi/2]。
    pub fn pitch(self) -> f32 {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let v = 2.0 * (w * y - z * x);
        crate::math::asin(libm::fmaxf(-1.0, libm::fminf(1.0, v)))
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
    /// 系统启动以来的启动时长（ms），MAVLink 多消息 time_boot_ms 字段共用，便于地面站对齐时序。
    pub time_boot_ms: i32,
    pub pos: [Meter; 3],          // 位置 (N, E, D)  注意 D 向下为正
    pub vel: [MeterPerSecond; 3], // 速度 (N, E, D)
    pub att: Quaternion,          // 姿态四元数（机体->世界）
    pub omega: [RadianPerSecond; 3], // 机体角速度 (p, q, r)
    /// 估计空速（m/s），由空速计测量（经 EKF 融合）；无空速计时为 0。
    pub airspeed: MeterPerSecond,
    /// 估计的加计零偏（m/s²，世界系），由速度观测（Doppler）驱动；无速度观测时为 0。
    /// 阶段 11-A 诊断：用于确认垂向零偏是否被 EKF 正确估计并扣除。
    pub accel_bias: [f32; 3],
}

impl VehicleState {
    pub fn zero() -> Self {
        Self {
            time_boot_ms: 0,
            pos: [Meter::ZERO, Meter::ZERO, Meter::ZERO],
            vel: [MeterPerSecond::ZERO, MeterPerSecond::ZERO, MeterPerSecond::ZERO],
            att: Quaternion::IDENTITY,
            omega: [RadianPerSecond::ZERO, RadianPerSecond::ZERO, RadianPerSecond::ZERO],
            airspeed: MeterPerSecond::ZERO,
            accel_bias: [0.0; 3],
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
///
/// `vel` 为可选的 GPS Doppler 速度观测（多普勒测速）。`None` 表示仅位置观测
/// （MCU/GPS 默认路径零回归）；仿真侧在 GPS 样本里附带 Doppler 速度时用
/// [`PosSample::with_vel`]，EKF 融合位置后额外做速度观测。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PosSample {
    pub pos: [Meter; 3],                       // NED 位置
    pub vel: Option<[MeterPerSecond; 3]>,      // 可选 Doppler 速度（NED）
}

impl PosSample {
    /// 仅位置观测（默认，零回归）。
    pub fn pos_only(pos: [Meter; 3]) -> Self {
        PosSample { pos, vel: None }
    }
    /// 位置 + Doppler 速度观测。
    pub fn with_vel(pos: [Meter; 3], vel: [MeterPerSecond; 3]) -> Self {
        PosSample { pos, vel: Some(vel) }
    }
}

/// 空速计（皮托管 / 差分气压）单次测量：总压 - 静压差换算的真空速（IAS≈TAS，忽略空气压缩）。
/// 空速计测的是**气流相对机体的速度大小**，不含风。EKF 用它对水平速度幅值做约束。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedSample {
    pub speed: Airspeed,
    pub timestamp_s: f64,
}

/// 视觉里程计（VIO）单次测量：机载相机 + IMU 融合出的**相对**运动增量。
///
/// 特点：更新率高（30–60Hz）、短期精度高（速度误差 ~0.1 m/s 量级）、
/// 但**无绝对参考、随时间缓慢漂移**（位置长期误差累积）。
/// 因此 EKF 用中等位置噪声 + 较小速度噪声融合：作为 GPS 帧之间的连续修正，
/// 填补 GPS 失锁 / 低更新率时的位置与速度估计空白。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VioSample {
    /// NED 位置（可选：相对起点累积的估计，存在长期漂移）。
    pub pos: Option<[Meter; 3]>,
    /// NED 速度（可选：光流 / 特征点跟踪估计，精度高于 GPS Doppler）。
    pub vel: Option<[MeterPerSecond; 3]>,
}

impl VioSample {
    /// 空样本（传感器故障 / 未装备）。
    pub fn none() -> Self {
        VioSample { pos: None, vel: None }
    }
    /// 仅位置观测。
    pub fn pos_only(pos: [Meter; 3]) -> Self {
        VioSample { pos: Some(pos), vel: None }
    }
    /// 仅速度观测。
    pub fn vel_only(vel: [MeterPerSecond; 3]) -> Self {
        VioSample { pos: None, vel: Some(vel) }
    }
    /// 位置 + 速度观测。
    pub fn with_vel(pos: [Meter; 3], vel: [MeterPerSecond; 3]) -> Self {
        VioSample { pos: Some(pos), vel: Some(vel) }
    }
}

/// RTK-GPS 单次测量：载波相位差分后的**厘米级高精度**位置（NED）。
///
/// 特点：更新率低（1–5Hz），但绝对精度比普通 GPS 高一个量级以上
/// （水平 ~2cm、垂向 ~4cm，即标准差 ~0.05m）。
/// EKF 用极小位置观测噪声融合，把位置协方差压到厘米量级，
/// 同时抑制 VIO 长期漂移（RTK 提供绝对参考、VIO 提供帧间连续）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtkSample {
    /// NED 厘米级位置。
    pub pos: [Meter; 3],
}

impl RtkSample {
    pub fn new(pos: [Meter; 3]) -> Self {
        RtkSample { pos }
    }
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

/// 由四元数（机体->世界）导出 3x3 旋转矩阵，按行主序写入长度为 9 的数组
/// `[r00 r01 r02 r10 r11 r12 r20 r21 r22]`，与 [`rotate_vec_by_quat`] 一致。
pub fn quat_to_rotmat(q: Quaternion) -> [f32; 9] {
    let w = q.w; let x = q.x; let y = q.y; let z = q.z;
    [
        1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),     2.0 * (x * z + w * y),
        2.0 * (x * y + w * z),     1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
        2.0 * (x * z - w * y),     2.0 * (y * z + w * x),     1.0 - 2.0 * (x * x + y * y),
    ]
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

