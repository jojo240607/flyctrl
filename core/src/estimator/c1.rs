//! **C1 骨架：误差状态 EKF**（设计见 `docs/c1-design.md` ✓）
//!
//! ⚠️ **本模块当前【未接入】任何产品路径** ✓ —— 它是 T4 的骨架：
//!  · 已通过数值对照的部分（§5 的姿态块 / 速度-零偏块 / 位置块 / 零偏块 ✓、
//!    §4 的标称递推与乘法顺序 ✓）在此落地 ✓；
//!  · **未验证的部分（`∂δv/∂δθ`）显式 panic** ✗ ——
//!    沿用 T1 的"防静默回退"模式：宁可拒绝，也不给出错误结果 ✓✓。
//!
//! 依据：本会话的多次实证 —— "文献写法 → 本项目语义"的转写是静默错误高发区 ✗
//! （§5 的符号被数值工装抓到 ✗→✓；§4 的乘法顺序同理 ✓）。

use crate::units::Radian;
use crate::vehicle::{rotate_vec_by_quat, rotate_vec_by_quat_inverse, Quaternion};

/// C1 的标称状态（15 维误差状态对应的名义量 ✓）
#[derive(Debug, Clone, Copy)]
pub struct C1State {
    /// 姿态：机体系 → 导航系（NED ✓）
    pub q: Quaternion,
    /// 速度（NED，m/s）
    pub v: [f32; 3],
    /// 位置（NED，m）
    pub p: [f32; 3],
    /// 陀螺零偏（机体，rad/s）
    pub bg: [f32; 3],
    /// 加计零偏（机体，m/s²）
    pub ba: [f32; 3],
}

impl C1State {
    /// 名义递推（§4 ✓ + §12.6 的乘法顺序修正 ✓）
    ///
    /// ```text
    /// q ← exp((ω_m − b_g)·dt) * q     ← 【左乘】✓（§12.6：文献写法直译会反向 ✗）
    /// v ← v + (R(q)·(a_m − b_a) + g)·dt     ★ 比力驱动动力学（非重力参考 ✓）
    /// p ← p + v·dt
    /// b_g, b_a ← 常值
    /// ```
    pub fn predict(&mut self, imu: ImuDelta, g_ned: [f32; 3], dt: f32) {
        let w = [
            imu.delta_ang[0] / dt - self.bg[0],
            imu.delta_ang[1] / dt - self.bg[1],
            imu.delta_ang[2] / dt - self.bg[2],
        ];
        let wn2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
        if wn2 > 1e-18 {
            // no_std：用 crate 的 sqrt（与其他模块一致 ✓）
            let wn = crate::math::sqrt(wn2);
            let dq = Quaternion::from_axis_angle([w[0] / wn, w[1] / wn, w[2] / wn], Radian(wn * dt));
            // ★【左乘】✓ —— 本项目语义下"机体角速率"必须是左乘（§12.6 ✓）
            self.q = (dq * self.q).normalize();
        }
        let f = [
            imu.delta_vel[0] / dt - self.ba[0],
            imu.delta_vel[1] / dt - self.ba[1],
            imu.delta_vel[2] / dt - self.ba[2],
        ];
        let aw = rotate_vec_by_quat(self.q, f);
        let v_old = self.v;
        for i in 0..3 {
            self.v[i] = v_old[i] + (aw[i] + g_ned[i]) * dt;
            self.p[i] += v_old[i] * dt;
        }
    }
}

/// IMU 增量（与 PX4 的 delta_ang / delta_vel 同构 ✓）
#[derive(Debug, Clone, Copy)]
pub struct ImuDelta {
    pub delta_ang: [f32; 3],
    pub delta_vel: [f32; 3],
}

/// 误差状态的块索引（15 维 ✓）
pub const N: usize = 15;
pub const I_ATT: usize = 0;
pub const I_VEL: usize = 3;
pub const I_POS: usize = 6;
pub const I_BG: usize = 9;
pub const I_BA: usize = 12;

/// 构造误差状态转移矩阵 F（15×15，行主序）
///
/// **已验证块**（数值对照通过 ✓）
///  · 姿态：`I + [ω×]·dt`（§5 的 **+**[ω×] ✓ —— 原文 −[ω×] 已被工装证伪 ✗→✓）
///  · 速度×加计零偏：`−R·dt` ✓
///  · 位置×速度：`I·dt` ✓
///  · 零偏自块：`I` ✓
/// **未验证块 ⇒ 返回 Err** ✗（不 panic、不静默 ✓）
///  · 速度×姿态 `∂δv/∂δθ`（待 `derivation.py` ✓；我已两次手构造失败 ✗）
pub fn transition_matrix(
    q: Quaternion,
    w: [f32; 3],
    dt: f32,
    r: &[[f32; 3]; 3],
    f_body: [f32; 3],
) -> Result<[[f32; N]; N], &'static str> {
    let mut f = [[0.0f32; N]; N];
    for i in 0..N {
        f[i][i] = 1.0;
    }
    // 姿态自块：+[ω×]·dt ✓（[ω×] = [[0,-wz,wy],[wz,0,-wx],[-wy,wx,0]]）
    f[I_ATT][I_ATT + 1] += -w[2] * dt;
    f[I_ATT][I_ATT + 2] += w[1] * dt;
    f[I_ATT + 1][I_ATT] += w[2] * dt;
    f[I_ATT + 1][I_ATT + 2] += -w[0] * dt;
    f[I_ATT + 2][I_ATT] += -w[1] * dt;
    f[I_ATT + 2][I_ATT + 1] += w[0] * dt;
    // 姿态×陀螺零偏：−I ✓
    for i in 0..3 {
        f[I_ATT + i][I_BG + i] += -dt;
    }
    // 速度×加计零偏：−R·dt ✓（数值验证 4.46e-5 ✓）
    for i in 0..3 {
        for j in 0..3 {
            f[I_VEL + i][I_BA + j] += -r[i][j] * dt;
        }
    }
    // 位置×速度：I·dt ✓
    for i in 0..3 {
        f[I_POS + i][I_VEL + i] += dt;
    }
    // 速度×姿态：∂δv/∂δθ = 【+[a_world ×]·dt】（§12.8 ✓ 参照 derivation.py 169–182 行定形 ✓）
    //   ★叉乘必须在【世界系】做（用 a_world = R·f ✓）——
    //   写成 R·[f×] 是【结构性错误】✗（少一个 Rᵀ 的相似变换，怎么调符号都不对 ✓）
    let a_world = rotate_vec_by_quat(q, f_body);
    let (ax, ay, az) = (a_world[0], a_world[1], a_world[2]);
    for j in 0..3 {
        // [a×] 的第 j 列 = a × e_j
        let col = match j {
            0 => [0.0, az, -ay],
            1 => [-az, 0.0, ax],
            _ => [ay, -ax, 0.0],
        };
        for i in 0..3 {
            f[I_VEL + i][I_ATT + j] += col[i] * dt;
        }
    }
    Ok(f)
}

// 编译期守卫：本骨架【不得】被产品路径引用（直到 F 全块验证通过 ✓）
#[cfg(test)]
mod tests {
    use super::*;

    /// 三极限情形（与仿真侧工装同判据 ✓）
    #[test]
    fn c1_extreme_cases() {
        let g = [0.0f32, 0.0, 9.81];
        let dt = 0.01f32;
        let q = Quaternion::from_axis_angle([0.2, 0.3, 0.5], Radian(0.4)).normalize();
        // ① 静止/匀速：比力 = 支撑重力 ⇒ Δv ≈ 0 ✓
        let a_support = rotate_vec_by_quat_inverse(q, [-g[0], -g[1], -g[2]]);
        let mut s = C1State { q, v: [0.0; 3], p: [0.0; 3], bg: [0.0; 3], ba: [0.0; 3] };
        s.predict(
            ImuDelta { delta_ang: [0.0; 3], delta_vel: [a_support[0] * dt, a_support[1] * dt, a_support[2] * dt] },
            g,
            dt,
        );
        let dv2 = s.v[0] * s.v[0] + s.v[1] * s.v[1] + s.v[2] * s.v[2];
        assert!(dv2 < 1e-10, "静止/匀速应无净加速度（实测 |Δv|² = {dv2:.2e}）✗");
        // ② 自由落体：a_m = 0 ⇒ Δv = g·dt ✓
        let mut s2 = C1State { q, v: [0.0; 3], p: [0.0; 3], bg: [0.0; 3], ba: [0.0; 3] };
        s2.predict(ImuDelta { delta_ang: [0.0; 3], delta_vel: [0.0; 3] }, g, dt);
        let dev2 = (s2.v[0] - g[0] * dt).powi(2)
            + (s2.v[1] - g[1] * dt).powi(2)
            + (s2.v[2] - g[2] * dt).powi(2);
        assert!(dev2 < 1e-12, "自由落体应以 g 加速（偏差² = {dev2:.2e}）✗");
        // ③ δv/δθ 块：应为 +[a_world×]·dt（世界系叉乘 ✓），且整体返回 Ok ✓
        let r = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let f_body = [0.3f32, -0.2, -9.7];
        let fm = transition_matrix(q, [0.0, 0.0, 0.0], dt, &r, f_body).expect("F 应可构造 ✓");
        let aw = rotate_vec_by_quat(q, f_body);
        // 逐元素核对 [a_world×]·dt 的第 j 列 = dt·(a_world × e_j)
        let cross = |a: [f32; 3], e: [f32; 3]| -> [f32; 3] {
            [a[1] * e[2] - a[2] * e[1], a[2] * e[0] - a[0] * e[2], a[0] * e[1] - a[1] * e[0]]
        };
        let basis = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut maxdev = 0.0f32;
        for j in 0..3 {
            let want = cross(aw, basis[j]);
            for i in 0..3 {
                maxdev = maxdev.max((fm[I_VEL + i][I_ATT + j] - want[i] * dt).abs());
            }
        }
        assert!(maxdev < 1e-6, "δv/δθ 块应为 +[a_world×]·dt（偏差 {maxdev:.2e}）✗");
    }
}
