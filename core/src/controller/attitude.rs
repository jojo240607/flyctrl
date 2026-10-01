//! 姿态内环与电机混控（多旋翼控制器共享内核，P3-A3 提取）。
//!
//! 多个控制器（PID / TECS / …）共用同一姿态内环与 X 型四旋翼混控，
//! 保证不同外环算法之间的"内环行为完全一致"，便于公平对比与复用。
//! 此模块代码由 `pid.rs` 原内环/混控原样提取（行为不变，仅消除重复）。

use crate::vehicle::Quaternion;

/// 姿态内环输出：期望机体角速度 + 姿态误差旋转向量（供调试）。
pub struct AttitudeOut {
    /// 期望机体角速度 (p_cmd, q_cmd, r_cmd)。
    pub rates: [f32; 3],
    /// 姿态误差旋转向量（机体系，≈ 2·sign(w)·(x,y,z)，调试用）。
    pub err: [f32; 3],
}

/// 姿态内环：四元数姿态误差 -> 期望机体角速度（PD，无欧拉角奇点）。
///
/// q_err = q_est⁻¹ ⊗ q_des（机体坐标系下的误差旋转）；误差旋转向量 ≈ 2·sign(w)·(x,y,z)；
/// 期望机体角速度 = Kp_att·误差向量 - Kd_att·当前角速度（阻尼）。
pub fn attitude_rates(
    est_att: Quaternion,
    q_des: Quaternion,
    att_kp: f32,
    att_kd: f32,
    omega: [f32; 3],
) -> AttitudeOut {
    let q_err = crate::vehicle::quat_mul(crate::vehicle::quat_conj(est_att), q_des);
    let sgn = if q_err.w < 0.0 { -2.0 } else { 2.0 };
    let ex_b = sgn * q_err.x;
    let ey_b = sgn * q_err.y;
    let ez_b = sgn * q_err.z;
    AttitudeOut {
        rates: [
            att_kp * ex_b - att_kd * omega[0],
            att_kp * ey_b - att_kd * omega[1],
            att_kp * ez_b - att_kd * omega[2],
        ],
        err: [ex_b, ey_b, ez_b],
    }
}

/// X 型四旋翼混控：总推力 + 三轴机体角速度 -> 4 路归一化油门（未限幅）。
///
/// 布局 0=前右 1=后左 2=前左 3=后右；spin 0,1 CCW / 2,3 CW：
///   τx = l(m0+m3-m1-m2)  τy = l(m0+m2-m1-m3)  τz = k(m0+m1-m2-m3)
/// 解得 m0 = T + 0.5(p+q+r)，…
/// 符号修正（open_loop_torque_sign_probe 实测，2026-08-21）：yaw 项取 +r_cmd，
/// roll/pitch 保持原符号（见 pid.rs 混控注释）。
/// ★★§5.215【控制分配：**推力优先**的饱和管理 ✓】——把理想混控结果映射进 `[0,1]` 四路。
///
/// ## 现状问题 ✗（实测 ✓）
/// 现做法是四路**各自** `clampf(m,0,1)` ⇒ 差动触界时被**非对称**削掉 ⇒
/// **均值（总推力）丢失** ⇒ 掉高 ✗。
/// 台账实测：切向偏航 R=2m ⇒ **饱和 98.9% + 稳态偏 13.750m**；同类场景高度掉 **10.002m** ✗。
/// 仓内已有正解先例 ✓：`indi.rs` 的"保垂向推力"注释（个体电机夹紧会破坏均值 ⇒ 定高发散 ✓）。
///
/// ## 策略 ✓（"推力优先"的一手语义 ✓：高度不可协商、姿态可短时欠驱动 ✓）
/// 1. **未触界** ⇒ 原样返回（**逐位不变** ✓）
/// 2. 触界且**极差 ≤ 1** ⇒ **整体平移**进 `[0,1]`（差动完整保留 ✓ —— 等价 airmode 抬底 ✓）
/// 3. 触界且**极差 > 1** ⇒ 绕**均值缩放差动** ⇒ **均值逐位保持** ✓✓（推力精确保住 ✓）
///    —— 这是与"各自 clamp"的本质区别 ✓：后者均值会漂 ✗
///
/// 只在**第 3 种**情形下改变行为 ✓ ⇒ A/B 可归因 ✓。
pub fn x4_mix_sat(des_thrust: f32, pqr: [f32; 3]) -> [f32; 4] {
    let m = x4_mix(des_thrust, pqr);
    let mut mmin = f32::INFINITY;
    let mut mmax = f32::NEG_INFINITY;
    for v in m.iter() {
        if *v < mmin { mmin = *v; }
        if *v > mmax { mmax = *v; }
    }
    let span = mmax - mmin;
    if span <= 1.0 {
        // ② 整体平移（差动完整 ✓）；已在界内则平移量=0 ⇒ 与 `x4_mix` 逐位相同 ✓
        let shift = if mmin < 0.0 { -mmin } else if mmax > 1.0 { 1.0 - mmax } else { 0.0 };
        let mut o = [0.0f32; 4];
        for i in 0..4 {
            o[i] = (m[i] + shift).clamp(0.0, 1.0);
        }
        o
    } else {
        // ③ 绕均值缩放差动 ⇒ **均值逐位不变** ✓（推力优先 ✓）
        //   ⚠️ 自检抓出的错 ✗：第一版用 `s = 1/span` —— 差动**关于均值不对称**时
        //   （本用例 +1.2 / −0.4）缩放后仍越界（0.5+1.2·0.625=1.25 > 1 ✗）。
        //   正解 ✓：缩放量由**可用余量**决定 —— `room = min(c, 1−c)`、
        //   `s = room / max|m_i − c|` ⇒ 极端值恰好落在 0 或 1 ⇒ 均值逐位保持 ✓
        let c = (m[0] + m[1] + m[2] + m[3]) * 0.25;
        let mut dev = 0.0f32;
        for i in 0..4 {
            let d = (m[i] - c).abs();
            if d > dev { dev = d; }
        }
        let room = if c < 0.0 { 0.0 } else if c > 1.0 { 0.0 } else { c.min(1.0 - c) };
        let s = if dev > 0.0 { room / dev } else { 0.0 };
        let mut o = [0.0f32; 4];
        for i in 0..4 {
            o[i] = (c + (m[i] - c) * s).clamp(0.0, 1.0);
        }
        o
    }
}

pub fn x4_mix(des_thrust: f32, pqr: [f32; 3]) -> [f32; 4] {
    let [p_cmd, q_cmd, r_cmd] = pqr;
    [
        des_thrust + 0.5 * (p_cmd + q_cmd + r_cmd),
        des_thrust + 0.5 * (-p_cmd - q_cmd + r_cmd),
        des_thrust + 0.5 * (-p_cmd + q_cmd - r_cmd),
        des_thrust + 0.5 * (p_cmd - q_cmd - r_cmd),
    ]
}

#[cfg(test)]
mod mix_sat_tests {
    use super::*;

    /// ★★§5.215【零件级自检 ✓】控制分配饱和管理：
    ///  ① **未触界 ⇒ 与原路径逐位相同** ✓（零回归保证 ✓）
    ///  ② **触界且极差 ≤1 ⇒ 整体平移**：差动量（max−min）**逐位保持** ✓ + 全在 [0,1] ✓
    ///  ③ **触界且极差 >1 ⇒ 绕均值缩放**：**均值（总推力）逐位保持** ✓ + 全在 [0,1] ✓
    ///     —— 与"四路各自 clamp"的**本质区别** ✗：后者均值会丢（本测试给出反例数值 ✓）
    #[test]
    fn mix_sat_preserves_thrust_and_range() {
        let tol = 1e-6f32;
        // ① 未触界：与 clamp(x4_mix) 逐位相同
        //   ⚠️ 自检第一版把 `(0.3, 0.3, 0.3, -0.3)` 放进本组 ✗ —— 它其实**已触界**
        //   （m1 = 0.3 − 0.45 = −0.15 < 0 ✓）⇒ 被自检当场抓出 ✓ 移到 ② 组 ✓
        for (t, p, q, r) in [
            (0.5f32, 0.0f32, 0.0f32, 0.0f32),
            (0.5, 0.2, -0.15, 0.05),
            (0.6, 0.1, -0.1, 0.2),
        ] {
            let raw = x4_mix(t, [p, q, r]);
            let a = x4_mix_sat(t, [p, q, r]);
            for i in 0..4 {
                assert_eq!(a[i], raw[i].clamp(0.0, 1.0), "未触界应逐位相同 ✓");
            }
        }
        // ② 极差 ≤1 的平移用例：t=0.05, r=-0.6 ⇒ 两路负、且极差 0.6 ≤1
        let (t, pqr) = (0.05f32, [0.0f32, 0.0, -0.6]);
        let raw = x4_mix(t, pqr);
        let out = x4_mix_sat(t, pqr);
        for i in 0..4 {
            assert!((0.0..=1.0).contains(&out[i]), "应在 [0,1]，实测 {}", out[i]);
        }
        let span_raw = raw.iter().cloned().fold(f32::NEG_INFINITY, f32::max)
            - raw.iter().cloned().fold(f32::INFINITY, f32::min);
        let span_out = out.iter().cloned().fold(f32::NEG_INFINITY, f32::max)
            - out.iter().cloned().fold(f32::INFINITY, f32::min);
        assert!(
            (span_raw - span_out).abs() < tol,
            "极差≤1 时应**整体平移**（差动完整 ✓）：{span_raw} vs {span_out}"
        );
        // ③ 极差 >1 的缩放用例：t=0.5, 三轴大差动 ⇒ 极差 >1
        let (t, pqr) = (0.5f32, [0.8f32, 0.8, 0.8]);
        let raw = x4_mix(t, pqr);
        let out = x4_mix_sat(t, pqr);
        let s_raw: f32 = raw.iter().sum();
        let s_out: f32 = out.iter().sum();
        assert!(
            (s_raw - s_out).abs() < 1e-4,
            "极差>1 时应**逐位保均值（总推力）** ✓：Σ {s_raw} vs {s_out}"
        );
        for i in 0..4 {
            assert!((0.0..=1.0).contains(&out[i]), "应在 [0,1]，实测 {}", out[i]);
        }
        // ④ **负对照** ✗：旧的"四路各自 clamp"在同一用例下**均值明显丢失**
        let old = [
            raw[0].clamp(0.0, 1.0),
            raw[1].clamp(0.0, 1.0),
            raw[2].clamp(0.0, 1.0),
            raw[3].clamp(0.0, 1.0),
        ];
        let s_old: f32 = old.iter().sum();
        assert!(
            (s_old - s_raw).abs() > 0.5,
            "负对照失效：旧路径本应显著丢均值（Σ {s_old} vs {s_raw}）"
        );
        assert!(
            (s_out - s_raw).abs() < (s_old - s_raw).abs() * 0.05,
            "新路径的均值误差应远小于旧路径 ✓"
        );
    }
}
