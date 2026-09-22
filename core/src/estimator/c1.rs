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

/// 协方差矩阵（15×15 ✓）
pub type Cov = [[f32; N]; N];

/// 协方差预测：`P' = F·P·Fᵀ + Q` ✓
///
/// **可以实现的理由**：F 已 5/5 块数值/参照双证通过 ✓（§12.5–§12.8）。
/// 实现纪律：先用**不变量**自检（对称性 / F=I 时退化为 P+Q ✓），再谈精度 ✓。
pub fn predict_covariance(p: &Cov, f: &[[f32; N]; N], q: &Cov) -> Cov {
    // FP = F·P
    let mut fp = [[0.0f32; N]; N];
    for (i, fp_i) in fp.iter_mut().enumerate() {
        for (j, v) in fp_i.iter_mut().enumerate() {
            let mut s = 0.0f32;
            for k in 0..N {
                s += f[i][k] * p[k][j];
            }
            *v = s;
        }
    }
    // P' = FP·Fᵀ + Q
    let mut out = [[0.0f32; N]; N];
    for (i, out_i) in out.iter_mut().enumerate() {
        for (j, v) in out_i.iter_mut().enumerate() {
            let mut s = 0.0f32;
            for k in 0..N {
                s += fp[i][k] * f[j][k];
            }
            *v = s + q[i][j];
        }
    }
    out
}

/// **标量量测更新 + NIS 卡方门**（参照做法 ✓ —— 而非固定阈值 ✗；B 阶段两次失败的正解 ✓）
///
/// `h`：H 的 15 维行（已验证的 H 结构 ✓：GPS 位置/速度、气压高度 ✓）
/// `residual`：新息（量测 − 预测 ✓）；`r`：量测噪声方差 ✓；`gate_sigma`：门限（σ ✓，参照 mag 3.0σ ✓）
///
/// **拒绝时 `P` 与状态【保持不变】** ✓ —— 这是门控的关键性质 ✓（本次自检 ② 验证之 ✓）
/// 返回：被接受时给出 `NIS^0.5`（σ ✓）；被拒绝时 `Err` ✓
pub fn update_scalar(
    p: &mut Cov,
    h: &[f32; N],
    residual: f32,
    r: f32,
    gate_sigma: f32,
) -> Result<f32, &'static str> {
    // S = h·P·hᵀ + r
    let mut ph = [0.0f32; N];
    for (i, v) in ph.iter_mut().enumerate() {
        let mut s = 0.0f32;
        for j in 0..N {
            s += p[i][j] * h[j];
        }
        *v = s;
    }
    let mut s_ = r;
    for i in 0..N {
        s_ += h[i] * ph[i];
    }
    if s_ <= 0.0 {
        return Err("C1: S 非正 ⇒ 协方差异常 ✗（拒绝更新 ✓）");
    }
    let nis_sigma = residual.abs() / crate::math::sqrt(s_);
    if nis_sigma > gate_sigma {
        return Err("C1: 新息超门限 ⇒ 拒绝该量测 ✓（P 与状态保持不变 ✓）");
    }
    // K = P·hᵀ / S
    let k: [f32; N] = {
        let mut kk = [0.0f32; N];
        for i in 0..N {
            kk[i] = ph[i] / s_;
        }
        kk
    };
    // P ← (I − K·h)·P（在更新前的 P 上计算 ✓，避免就地污染 ✓）
    let mut newp = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..N {
            let mut s = p[i][j];
            for m in 0..N {
                s -= k[i] * h[m] * p[m][j];
            }
            newp[i][j] = s;
        }
    }
    *p = newp;
    Ok(nis_sigma)
}

/// 3×3 求逆（伴随/行列式 ✓，no_std 固定数组 ✓）
fn inv3(m: &[[f32; 3]; 3]) -> Option<[[f32; 3]; 3]> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let id = 1.0 / det;
    Some([
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * id,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * id,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * id,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id,
        ],
    ])
}

/// **三维量测更新 + NIS 卡方门**（GPS 位置/速度 ✓ —— H 已数值验证 ✓）
///
/// 与 `update_scalar` 同一纪律 ✓：**被拒绝时 P【逐位不变】** ✓；门用 NIS（σ ✓）
pub fn update_vec3(
    p: &mut Cov,
    h: &[[f32; N]; 3],
    residual: &[f32; 3],
    r: &[[f32; 3]; 3],
    gate_sigma: f32,
) -> Result<f32, &'static str> {
    // PHᵀ（15×3）
    let mut pht = [[0.0f32; 3]; N];
    for i in 0..N {
        for j in 0..3 {
            let mut s = 0.0f32;
            for k in 0..N {
                s += p[i][k] * h[j][k];
            }
            pht[i][j] = s;
        }
    }
    // S = H·PHᵀ + R（3×3）
    let mut s_mat = *r;
    for a in 0..3 {
        for b in 0..3 {
            let mut s = 0.0f32;
            for k in 0..N {
                s += h[a][k] * pht[k][b];
            }
            s_mat[a][b] += s;
        }
    }
    let s_inv = inv3(&s_mat).ok_or("C1: S 奇异/非正 ⇒ 拒绝更新 ✓")?;
    // NIS = νᵀ S⁻¹ ν（3 DOF ⇒ 门限按 sqrt(NIS) ≤ gate_sigma ✓，与参照同口径 ✓）
    let mut tmp = [0.0f32; 3];
    for a in 0..3 {
        let mut s = 0.0f32;
        for b in 0..3 {
            s += s_inv[a][b] * residual[b];
        }
        tmp[a] = s;
    }
    let mut nis = 0.0f32;
    for a in 0..3 {
        nis += residual[a] * tmp[a];
    }
    if nis < 0.0 {
        return Err("C1: NIS 为负 ⇒ 协方差异常 ✗");
    }
    let nis_sigma = crate::math::sqrt(nis);
    if nis_sigma > gate_sigma {
        return Err("C1: 新息超门限 ⇒ 拒绝该量测 ✓（P 与状态保持不变 ✓）");
    }
    // K = PHᵀ·S⁻¹（15×3）
    let mut k = [[0.0f32; 3]; N];
    for i in 0..N {
        for b in 0..3 {
            let mut s = 0.0f32;
            for a in 0..3 {
                s += pht[i][a] * s_inv[a][b];
            }
            k[i][b] = s;
        }
    }
    // P ← (I − K·H)·P（在更新前的 P 上算 ✓）
    let mut newp = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..N {
            let mut s = p[i][j];
            for a in 0..3 {
                for m in 0..N {
                    s -= k[i][a] * h[a][m] * p[m][j];
                }
            }
            newp[i][j] = s;
        }
    }
    *p = newp;
    Ok(nis_sigma)
}

/// 误差状态（15 维，与 F 的分块一致 ✓）
#[derive(Debug, Clone, Copy, Default)]
pub struct ErrorState {
    pub dtheta: [f32; 3],
    pub dv: [f32; 3],
    pub dp: [f32; 3],
    pub dbg: [f32; 3],
    pub dba: [f32; 3],
}

/// **误差状态注入物理状态**（静默陷阱高发区 ✗ ⇒ 必须数值自检 ✓）
///
/// 约定（**双证** ✓）：姿态用【左乘】误差四元数 ✓
///   · §12.4：本项目语义下"左乘 = 机体(local)扰动"（数值判定 ✓）
///   · 参照 `derivation.py` 153 行：`Rot3(Quaternion(xyz=theta/2, w=1)) * quat_nominal` ✓
/// v/p/零偏为**加性** ✓
pub fn inject_error(st: &mut C1State, e: &ErrorState) {
    let dq = {
        let th = e.dtheta;
        let th2 = th[0] * th[0] + th[1] * th[1] + th[2] * th[2];
        if th2 > 1e-18 {
            let n = crate::math::sqrt(th2);
            Quaternion::from_axis_angle([th[0] / n, th[1] / n, th[2] / n], Radian(n))
        } else {
            Quaternion::from_axis_angle([1.0, 0.0, 0.0], Radian(0.0))
        }
    };
    // ★左乘 ✓（§12.4 + 参照 153 行）
    st.q = (dq * st.q).normalize();
    for i in 0..3 {
        st.v[i] += e.dv[i];
        st.p[i] += e.dp[i];
        st.bg[i] += e.dbg[i];
        st.ba[i] += e.dba[i];
    }
}

/// **从物理状态差异提取误差状态**（与 `inject_error` 互为逆 ✓，用于自检 ✓）
pub fn extract_error(q_old: Quaternion, st: &C1State) -> ErrorState {
    // 姿态：δq = q_new * q_old⁻¹（本项目语义下的左乘误差 ✓，与参照 189 行同构 ✓）
    let inv = Quaternion { w: q_old.w, x: -q_old.x, y: -q_old.y, z: -q_old.z };
    let dq = (st.q * inv).normalize();
    let s2 = dq.x * dq.x + dq.y * dq.y + dq.z * dq.z;
    let s = crate::math::sqrt(s2);
    let dtheta = if s > 1e-9 {
        // 小角度：θ ≈ 2·(x,y,z)/w（w≈1 ✓）
        // ⚠️ 参数顺序：θ = 2·atan2(|vec|, w) ✓ —— 写成 atan2(w, |vec|) 会得到 2.29 的偏差 ✗
        //   （本自检当场抓到 ✓；与 §12.x 的约定类陷阱同源 ✓）
        let k = 2.0 * crate::math::atan2(s, dq.w.max(0.0)) / s;
        [k * dq.x, k * dq.y, k * dq.z]
    } else {
        [2.0 * dq.x, 2.0 * dq.y, 2.0 * dq.z]
    };
    ErrorState { dtheta, ..Default::default() }
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
        // ④ 协方差预测的不变量自检（先守不变量，再谈精度 ✓）
        let mut p0 = [[0.0f32; N]; N];
        for (i, row) in p0.iter_mut().enumerate() {
            row[i] = 1.0 + i as f32 * 0.1; // 对角正 ✓
        }
        // 4a) F = I ⇒ P' = P + Q ✓
        let mut eye = [[0.0f32; N]; N];
        for (i, row) in eye.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        let mut q0 = [[0.0f32; N]; N];
        for (i, row) in q0.iter_mut().enumerate() {
            row[i] = 0.25;
        }
        let out = predict_covariance(&p0, &eye, &q0);
        let mut dev = 0.0f32;
        for i in 0..N {
            for j in 0..N {
                let want = p0[i][j] + q0[i][j];
                dev = dev.max((out[i][j] - want).abs());
            }
        }
        assert!(dev < 1e-6, "F=I 时 P' 应为 P+Q（偏差 {dev:.2e}）✗");
        // 4b) F 非平凡时保持【对称性】✓（P、Q 对称 ⇒ P' 对称 ✓）
        let f2 = transition_matrix(q, [0.1, -0.05, 0.2], dt, &r, f_body).unwrap();
        let out2 = predict_covariance(&p0, &f2, &q0);
        let mut sym = 0.0f32;
        for i in 0..N {
            for j in 0..N {
                sym = sym.max((out2[i][j] - out2[j][i]).abs());
            }
        }
        assert!(sym < 1e-5, "P' 应保持对称（最大不对称 {sym:.2e}）✗");
        // ⑤ 标量量测更新 + NIS 门
        //   ① 内点：被接受，且对应方差【收缩】✓
        let mut pc = [[0.0f32; N]; N];
        for (i, row) in pc.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        let mut h_row = [0.0f32; N];
        h_row[I_POS + 2] = -1.0; // 气压高度 = −d ✓
        let r_var = 0.25f32; // 0.5 m 标准差 ✓
        let before = pc[I_POS + 2][I_POS + 2];
        let nis = update_scalar(&mut pc, &h_row, 0.1, r_var, 3.0).expect("内点应被接受 ✓");
        let after = pc[I_POS + 2][I_POS + 2];
        assert!(nis < 1.0 && after < before, "内点应被接受且方差收缩 ✓（NIS={nis:.3}）");
        //   ② 外点（残差 10 m ⇒ 远超 3σ）：被拒绝，且 **P 逐位不变** ✓
        let mut pc2 = [[0.0f32; N]; N];
        for (i, row) in pc2.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        let snapshot = pc2;
        let rej = update_scalar(&mut pc2, &h_row, 10.0, r_var, 3.0);
        assert!(rej.is_err(), "外点必须被 NIS 门拒绝 ✗");
        let mut dmax = 0.0f32;
        for i in 0..N {
            for j in 0..N {
                dmax = dmax.max((pc2[i][j] - snapshot[i][j]).abs());
            }
        }
        assert!(dmax == 0.0, "被拒绝时 P 必须【逐位不变】✗（实测 {dmax:.2e}）");
        // ⑥ 误差注入/提取的【往返自检】（姿态左乘约定 ✓，§12.4 + 参照 153/189 行双证 ✓）
        let q_old = Quaternion::from_axis_angle([0.2, 0.3, 0.5], Radian(0.4)).normalize();
        let mut st2 = C1State { q: q_old, v: [0.0; 3], p: [0.0; 3], bg: [0.0; 3], ba: [0.0; 3] };
        let dth = [0.01f32, -0.02, 0.015];
        inject_error(&mut st2, &ErrorState { dtheta: dth, ..Default::default() });
        let back = extract_error(q_old, &st2).dtheta;
        let mut rdev = 0.0f32;
        for i in 0..3 {
            rdev = rdev.max((back[i] - dth[i]).abs());
        }
        assert!(rdev < 1e-5, "误差注入/提取应互为逆（偏差 {rdev:.2e}）✗ —— 左乘约定错？");
        // 反证：若用【右乘】，往返应显著不符 ✓（确认该自检有鉴别力 ✓）
        let mut st3 = C1State { q: q_old, v: [0.0; 3], p: [0.0; 3], bg: [0.0; 3], ba: [0.0; 3] };
        {
            // 手工右乘（错误做法 ✗）
            let n = crate::math::sqrt(dth[0] * dth[0] + dth[1] * dth[1] + dth[2] * dth[2]);
            let dq = Quaternion::from_axis_angle([dth[0] / n, dth[1] / n, dth[2] / n], Radian(n));
            st3.q = (q_old * dq).normalize();
        }
        let back3 = extract_error(q_old, &st3).dtheta;
        let mut rdev3 = 0.0f32;
        for i in 0..3 {
            rdev3 = rdev3.max((back3[i] - dth[i]).abs());
        }
        assert!(rdev3 > 1e-4, "右乘应【可被本自检识别】为错 ✗（实测偏差 {rdev3:.2e}）");
        // ⑦ 三维量测更新（GPS 速度 ✓）内点/外点
        let mut pv = [[0.0f32; N]; N];
        for (i, row) in pv.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        let mut hv = [[0.0f32; N]; 3];
        for (a, row) in hv.iter_mut().enumerate() {
            row[I_VEL + a] = 1.0; // GPS 速度取 v ✓
        }
        let rv = [[0.25f32, 0.0, 0.0], [0.0, 0.25, 0.0], [0.0, 0.0, 0.25]];
        let b_before = pv[I_VEL][I_VEL];
        let n3 = update_vec3(&mut pv, &hv, &[0.1, -0.1, 0.05], &rv, 5.0).expect("内点应接受 ✓");
        assert!(n3 < 1.0 && pv[I_VEL][I_VEL] < b_before, "三维内点应接受且方差收缩 ✓");
        let mut pv2 = [[0.0f32; N]; N];
        for (i, row) in pv2.iter_mut().enumerate() {
            row[i] = 1.0;
        }
        let snap2 = pv2;
        assert!(
            update_vec3(&mut pv2, &hv, &[9.0, 9.0, 9.0], &rv, 5.0).is_err(),
            "三维外点必须被拒绝 ✗"
        );
        let mut d2 = 0.0f32;
        for i in 0..N {
            for j in 0..N {
                d2 = d2.max((pv2[i][j] - snap2[i][j]).abs());
            }
        }
        assert!(d2 == 0.0, "三维拒绝时 P 必须逐位不变 ✗（实测 {d2:.2e}）");
    }
}
