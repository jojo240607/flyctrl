//! L0 · 姿态核心（四元数 + 帧约定）。约定见 `CONTRACT.md` §2，本文件是其唯一实现处。

use crate::finite::{gate_all, Stage};
use crate::math;

/// 重力（NED 世界系，z 向下为正）。
pub const GRAVITY_NED: [f32; 3] = [0.0, 0.0, 9.806_65];

/// 姿态四元数：机体 → 世界，分量序 w,x,y,z。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    pub const IDENTITY: Self = Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    /// 绕单位轴 `axis` 转 `ang`（右手）。
    pub fn from_axis_angle(axis: [f32; 3], ang: f32) -> Self {
        let h = 0.5 * ang;
        let s = math::sin(h);
        Self { w: math::cos(h), x: axis[0] * s, y: axis[1] * s, z: axis[2] * s }
    }

    /// ZYX 内旋欧拉角 ⇒ 四元数（`rpy[0]=roll,[1]=pitch,[2]=yaw`）。
    pub fn from_euler_zyx(rpy: [f32; 3]) -> Self {
        let (r, p, y) = (0.5 * rpy[0], 0.5 * rpy[1], 0.5 * rpy[2]);
        let (sr, cr) = (math::sin(r), math::cos(r));
        let (sp, cp) = (math::sin(p), math::cos(p));
        let (sy, cy) = (math::sin(y), math::cos(y));
        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }

    /// ⇒ ZYX 内旋欧拉角 `[roll, pitch, yaw]`。
    pub fn to_euler_zyx(self) -> [f32; 3] {
        let (w, x, y, z) = (self.w, self.x, self.y, self.z);
        let roll = math::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        let pitch = math::asin((2.0 * (w * y - z * x)).clamp(-1.0, 1.0));
        let yaw = math::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        [roll, pitch, yaw]
    }

    /// `self ∘ r`：先施加 `r`，再施加 `self`（与矩阵乘法同序）。
    pub fn mul(self, r: Self) -> Self {
        Self {
            w: self.w * r.w - self.x * r.x - self.y * r.y - self.z * r.z,
            x: self.w * r.x + self.x * r.w + self.y * r.z - self.z * r.y,
            y: self.w * r.y - self.x * r.z + self.y * r.w + self.z * r.x,
            z: self.w * r.z + self.x * r.y - self.y * r.x + self.z * r.w,
        }
    }

    /// 共轭（单位模时等于逆）。
    pub fn conj(self) -> Self {
        Self { w: self.w, x: -self.x, y: -self.y, z: -self.z }
    }

    pub fn norm(self) -> f32 {
        math::sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    pub fn is_finite(self) -> bool {
        self.w.is_finite() && self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    /// 归一化。**非有限或零模 ⇒ `None`**（调用方必须显式处理，不得静默替换 —— 契约 §4）。
    pub fn normalize(self) -> Option<Self> {
        let n = self.norm();
        if !(n.is_finite() && n > 1e-12) {
            return None;
        }
        let inv = 1.0 / n;
        Some(Self { w: self.w * inv, x: self.x * inv, y: self.y * inv, z: self.z * inv })
    }

    /// 机体系 → 世界系：`v_w = q · v_b · q*`（要求单位模）。
    pub fn rotate(self, v: [f32; 3]) -> [f32; 3] {
        let u = [self.x, self.y, self.z];
        let uv = cross(u, v);
        let uuv = cross(u, uv);
        [
            v[0] + 2.0 * (self.w * uv[0] + uuv[0]),
            v[1] + 2.0 * (self.w * uv[1] + uuv[1]),
            v[2] + 2.0 * (self.w * uv[2] + uuv[2]),
        ]
    }
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// **静止时比力的期望值（机体系）** —— 契约 §2 定义式 `f_b = R(q)ᵀ·(−g_ned)`。
/// L4 静止对齐必须反解本式；本函数是它与姿态约定自洽的判据。
pub fn specific_force_at_rest(q: Quat) -> [f32; 3] {
    let g_body = q.conj().rotate(GRAVITY_NED);
    [-g_body[0], -g_body[1], -g_body[2]]
}

/// 契约 §4：姿态进出模块必须有限。
#[inline]
pub fn gate_quat(stage: Stage, q: Quat) -> Result<Quat, crate::finite::Violation> {
    gate_all(stage, &[q.w, q.x, q.y, q.z])?;
    Ok(q)
}
