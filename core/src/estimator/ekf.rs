//! 松耦合误差状态 EKF 估计器（no_std, 无堆分配）。
//!
//! 松耦合误差状态 EKF 估计器（no_std, 无堆分配）。
//!
//! 状态分两部分：
//! - 名义姿态：四元数，由去偏置角速度积分得到（可选微弱重力修正）。
//! - 误差状态 EKF（12 维）：`[pos(3), vel(3), gyro_bias(3), accel_bias(3)]`。
//!   预测用 IMU 加计（经名义姿态旋转到世界系、减去估计的加计零偏、补偿重力）推进速度/位置；
//!   观测用位置测量（GPS/气压）与速度测量（Doppler）做标准卡尔曼更新，
//!   并回灌陀螺/加计零偏到姿态积分。
//!
//! 加计零偏仅由**速度观测**（Doppler）驱动估计：位置/气压更新刻意不修正 accel_bias
//! 状态（见 `update_pos`/`update_alt` 中清零对应增益行），避免无速度观测时该状态不可观
//! 而导致数值发散；有速度观测（实飞 Doppler/GPS 速度）时零偏方可估计并扣除，
//! 否则垂向速度持续积分零偏而发散（消费级 IMU 常见 0.05 m/s² 量级）。

use crate::estimator::trait_def::Estimator;
use crate::math::sqrt;
use crate::units::*;
use crate::vehicle::{
    AirspeedSample, ImuSample, PosSample, Quaternion, RtkSample, VehicleState, VioSample,
    quat_to_rotmat, rotate_vec_by_quat,
};

const N: usize = 10; // pos(3) + vel(3) + gyro_bias(3) + accel_bias_z(1)

/// 陀螺零偏估计的**收敛速率**（1/s）。**0 = 关闭**（既有行为：`x[6..8]` 恒为 0）。
///
/// 见 `step` 里"陀螺零偏在线估计"处的推导。默认关的理由：属新增估计通道，
/// 需先 A/B 量化收益与代价（会不会把姿态估计带坏），再定值——不拍脑袋。
#[used]
pub static mut G_GYRO_BIAS_K: f32 = 0.0;

/// 陀螺零偏估计限幅（rad/s）。消费级 IMU 零偏远小于此；仅防异常值。
const GYRO_BIAS_MAX: f32 = 0.05;

/// [标定] 重力锚定的**水平非重力加速度门控**满闭阈值（m/s²）。哨兵 <0 = 编译期默认。
///
/// 为何需要第四个门控：锚定的前提是"比力方向 = 重力方向"。该前提只要求
/// **水平非重力加速度 ≈ 0** —— 而这一点**前面三个门控都看不出来**：
///   - 幅值门控 w：稳态抗风时比力幅值仍 = g（推力与阻力水平相消、合力竖直），
///     幅值门控**大开**；
///   - 方向门控 w_align：同理，比力方向仍竖直、与估计重力夹角≈0，**大开**；
///   - 陀螺门控 w_gyro：平稳抗风时 |ω| 很小，**大开**。
/// 但阵风/湍流/机动时比力确实含水平分量 ⇒ 锚定把姿态错误拉向"比力反方向"
/// （那不是重力）⇒ 反而劣化。实测（H 场）：att_alpha 由 0 改 0.02 后，
/// 无风档漂移 9.36 -> 3.12m（受益），但 **B3 风档 3.39 -> 19.22m**（劣化），
/// 并附带 4 项新失败（mag_hover/sil/avoidance/monte_carlo）。
///
/// 判据：`a_h = |R·a|` 的水平分量（世界系非重力加速度）越小越可信。
/// `a_h < 阈值/3` 全开，`> 阈值` 全闭，中间线性。
#[used]
pub static mut G_ATT_ACC_GATE: f32 = -1.0;

/// 水平非重力加速度门控的编译期默认满闭阈值（m/s²）。
/// 注意：**默认 0 = 关闭**（保持历史"三门槛"行为）。实测瞬时门控过于抖动，
/// 见 `G_ATT_ACC_AC`（交流能量门控，本项的正解）。
const ATT_ACC_GATE_DEFAULT: f32 = 0.0;

/// [标定] 重力锚定的**交流能量门控**满闭阈值（m/s²，a_h 的交流幅度）。哨兵 <0 = 编译期默认。
///
/// # 为何必须用"交流能量"而不是瞬时值或均值
/// 扰动（阵风 @0.12Hz + Dryden 湍流）是**零均值交流**：
///   - **瞬时 `a_h`** 抓得住，但悬停噪声也让它抖 ⇒ 门控反复开闭（实测 2.0 阈值处
///     无风档恶化到 88.73m）；
///   - **均值 `a_h`** 在零均值扰动下 ≈0 ⇒ 门控根本不关（抓不住）；
/// ⇒ 只有**交流幅度**（偏离均值的量）既能稳定又能反映"扰动有多剧烈"。
///
/// 实现：`a_h` 的 EMA 均值 + EMA 绝对偏差（一阶包络），门控看偏差。
/// 平稳无风悬停 → 偏差小 → 全锚定（拿陀螺零偏抑制的收益）；
/// 阵风/湍流/机动 → 偏差大 → 全撤（避免把姿态错误拉向"比力反方向"）。
#[used]
pub static mut G_ATT_ACC_AC: f32 = -1.0;

/// 交流门控的编译期默认满闭阈值（m/s²）。**默认 0 = 关闭**；取值由扫描数据定。
const ATT_ACC_AC_DEFAULT: f32 = 0.0;

/// [标定] `q_accel` 运行时覆盖（0 = 用 `EkfEstimator` 的编译期值）。
///
/// 用途：协方差更新改用 Joseph 形式后，位置/高度通道需要重标定——原误实现的协方差
/// 膨胀曾意外充当鲁棒性拐杖。把 `q_accel`（加计零偏过程噪声）做成可运行时写入的旋钮，
/// 即可在**不重编固件**的前提下扫描"加计偏置容忍度 vs 噪声鲁棒性"，用数据选值；
/// 定稿后应写回 `default_quad()` 并把本旋钮保持 0。
#[used]
pub static mut G_Q_ACCEL: f32 = 0.0;

/// [标定] `q_vel`（速度过程噪声）运行时覆盖（0 = 用编译期值）。
#[used]
pub static mut G_Q_VEL: f32 = 0.0;

/// [标定] `mag_alpha`（磁力计航向锚定强度）运行时覆盖。
///
/// ⚠️ 哨兵取值与其它旋钮**不同**：本旋钮用 **< 0（默认 -1）= 用编译期值**。
/// 理由：`0` 是本旋钮的**有效取值**（= 关闭磁锚定，用于隔离磁路影响的对照实验），
/// 不能用 0 当"未设置"哨兵，否则无法表达"显式关闭"。
#[used]
pub static mut G_MAG_ALPHA: f32 = -1.0;

/// [标定] **磁参考全姿态修正**强度（1/s）。**0 = 关闭**（既有行为：只修 yaw）。
///
/// 路线 2.2e：把磁参考从"只绕世界 Z 修 yaw"扩为**修全姿态（含 roll/pitch）**。
/// 动机：比力锚定（`att_alpha`）会被水平加速度污染（H 场湍流风劣化 3.39→19.22m
/// 的根源），而**磁场与世界加速度无耦合** ⇒ 是加速度免疫的姿态参考。
///
/// 前置条件（已验证）：必须扣除硬铁 —— 实测未补偿时磁场方向偏差最坏 **39.54°**
/// （解析 40.06°），补偿后 **0°**（`fly-sim-core/tests/mag_attitude_ref.rs`）。
/// 硬铁同步注入见 `FlyController::new`（F7 使用约定）。
///
/// 局限：需已知**磁倾角**（`mag_ref3d`）；软铁未建模；标定残差与磁噪声直接进姿态。
#[used]
pub static mut G_MAG3D_ALPHA: f32 = 0.0;

/// [标定] `att_alpha`（重力锚定强度）运行时覆盖。
///
/// ⚠️ 哨兵同 `G_MAG_ALPHA`：**< 0（默认 -1）= 用编译期值**（`0` 是有效值 = 关闭锚定）。
///
/// 为何需要可调：路线 2.2e 提供了**加速度免疫**的磁参考后，重力锚定（其参考=比力，
/// 会被水平加速度污染）应当**下调甚至取消** —— 这将使 "0.0 vs 0.02" 那个两难
/// （H/M 口径不一致 vs 4 项测试红）**从根上消失**，故必须能扫。
#[used]
pub static mut G_ATT_ALPHA: f32 = -1.0;

/// [标定] `r_vel`（Doppler 速度观测噪声）运行时覆盖（0 = 用编译期值）。
#[used]
pub static mut G_R_VEL: f32 = 0.0;

/// [标定] `r_pos`（GPS 位置观测噪声）运行时覆盖（0 = 用编译期值）。
#[used]
pub static mut G_R_POS: f32 = 0.0;

/// [标定] **GPS/Doppler 差分推导 `a_world`** 的开关（0 = 关，默认）。
///
/// 启用后，`update_vel_r` 会把相邻多普勒速度观测差分（一阶低通 tau=0.25s）
/// 写入 `world_accel`，从而在重力锚定中扣除平移分量（阶段2 P4 方案 C）。
///
/// **为何默认关**：开启后它**能**修好阶段 4 的 `vel×att` 耦合、也能减少真实平移时的
/// 姿态污染，**但在“本无平移”的场景全部变差，且 A9 自由落体是灾难级**。
///
/// **2026-09-21 全场景 A/B 复测**（`tests/att_est.rs::g_aw_gps_default_evaluation`，
/// 姿态 RMSE；注：早期“A1 0.948°→0.558°、A3 6.10°→3.01°”那组数字取自 F1/F5
/// 修复之前的版本，已过期，见下表。）：
///
/// | 场景 | 关 | 开 | |
/// |---|---|---|---|
/// | A3 急刹 0.5g | 23.42° | **18.59°** | ✅ +20.6%（low_noise 14.39°→2.67°，+81%）|
/// | A7 慢转+0.5g | 38.64° | **29.70°** | ✅ +23%（但 max 50.5°→**93.8°**，峰值反而变差）|
/// | A1/A2/A8 无平移 | — | — | ❌ 略变差（low_noise A2 0.017°→0.406°）|
/// | **A9 自由落体** | 24.60° | **132.65°** | ❌❌ max 28°→**173.6°**（low_noise 0.023°→29.28°）|
///
/// ⇒ **只在“有真实平移”时帮忙（2/10），其余全变差**；而 **A9 自由落体是灾难级**：
/// 扣除 `a_world` 后 `|a|≈g`，**把幅值门的失重保护骗开了**（本该生效的失效保护被绕过）。
/// 自由落体/抛飞/强下洗是真实工况，估计器必须**优雅退化**而非发散 —— 这是
/// “默认关”的**安全理由**。
///
/// 根因是 **`a_world` 源的质量**（噪声 + 0.15s 延迟 + 0.25s 低通 + 20Hz）。
/// 待该源改善（延迟补偿/更强滤波/更高帧率）后重新评估；更稳的做法是
/// **按飞行状态门控**（仅动力飞行、非失重时启用）。两者均记入阶段 6。
///
/// ⚠️ 历史：本静态初值曾为 `1.0`，与上面“为何默认关”的结论**自相矛盾**
#[used]
pub static mut G_AW_GPS: f32 = 0.0;

/// [标定] `a_world` 差分低通时间常数（s）。运行时覆盖（0 = 用内置默认 0.25）。
///
/// 权衡：Doppler 速度噪声（消费级 ~0.1 m/s @20Hz）被差分放大 ⇒ 必须低通；
/// 而低通带来滞后（叠加 GPS 0.15s 延迟）⇒ 平移补偿的定时误差。
/// 本旋钮用于扫这条曲线（见 `docs/stage4-outer-loop-findings.md` P4）。
#[used]
pub static mut G_AW_TAU: f32 = 0.0;

/// [标定] **垂向速度观测增益上限** `|k[5]|`。
///
/// 语义：`0` = 用**内置默认 0.1**；`<0` = **强制 0**（历史对照，垂速纯 IMU 积分）；
/// `>0` = 覆盖。
///
/// 背景：`update_alt` 里长期写 `k[5] = 0.0`（气压/位置观测不修正垂速），
/// 原因见下方注释（“单拍 +24.6 → 爆炸”）。后果是**垂速纯 IMU 积分**：
/// 零偏/噪声留下稳态速度误差 → 高度环跟着走 → 真值持续漂移。
///
/// 2026-09-21 垂向专项（`pos_ctrl::outer_hover_vertical_sink_sixty_sec`）：
/// 真值反馈下高度**完美保持**（下沉 -0.000m），而估计反馈 + realistic 悬停 60s
/// 漂 **+2.642m**（max\|dz\| **15.529m**）⇒ 问题 100% 在垂向估计注入外环。
///
/// 扫描（`pos_ctrl::outer_vertical_kvz_sweep`，60s realistic 悬停）：
///
/// | \|k[5]\|上限 | 净漂移 | max\|dz\| |
/// |---|---|---|
/// | **0（历史）** | **+2.642m** | **15.529m** |
/// | 0.02 | -0.724m | 1.549m |
/// | **0.10** | **-0.486m** | **1.591m** |
/// | 0.20 ~ 2.00 | -0.486m | 1.591m（**与 0.10 逐位相同**）|
///
/// ⇒ 0.10 以上结果不再变 ⇒ **实际 `|k[5]|` 从未超过 0.1**，
/// 当年“爆炸”那种大修正有界增益下不会发生。故内置默认取 **0.1**。
#[used]
pub static mut G_KVZ_FROM_ALT: f32 = 0.0;

/// 协方差对角线硬上限（m² / (m/s)² / (m/s²)²）。防止不可观状态协方差经 F 矩阵耦合
/// 指数增长而至 Inf/NaN。正常可观测状态下协方差远小于此值。
const P_MAX: f32 = 1e3;

/// 卡尔曼增益元素硬上限。紧噪声观测（RTK 厘米级 / VIO）下，位置观测经非对角协方差
/// 产生的速度行增益可爆炸到 ~40（K[3][0]=P[3][0]/S00，S00≈r_rtk→0.0025），一步把速度
/// 踢飞，随后 Joseph 协方差更新在 float32 下失去 PSD、交叉项 p03 发散到 -1e13 → NaN。
/// 限幅后每个观测元素对状态的修正有界（≤2 m/s per m），协方差更新保持数值稳定；
/// 位置行增益天然 ≤1（p00/(p00+r)），不受限幅影响。
const K_MAX: f32 = 2.0;

#[inline]
fn mat_ident(p: &mut [f32; N * N]) {
    p.fill(0.0);
    for i in 0..N {
        p[i * N + i] = 1.0;
    }
}

// C = A*B  (na x nk) * (nk x nb)
#[inline]
fn mat_mul(a: &[f32], b: &[f32], c: &mut [f32], na: usize, nk: usize, nb: usize) {
    for i in 0..na {
        for j in 0..nb {
            let mut s = 0.0f32;
            for k in 0..nk {
                s += a[i * nk + k] * b[k * nb + j];
            }
            c[i * nb + j] = s;
        }
    }
}

#[inline]
fn mat_add(a: &[f32], b: &[f32], c: &mut [f32], n: usize) {
    for i in 0..n {
        c[i] = a[i] + b[i];
    }
}

pub struct EkfEstimator {
    att: Quaternion,
    x: [f32; N],        // pos, vel, gyro_bias, accel_bias
    p: [f32; N * N],    // 协方差
    g_ref: f32,
    q_vel: f32,         // 速度过程噪声强度（水平）
    q_vel_z: f32,       // 速度过程噪声强度（垂向）
    q_bias: f32,        // 陀螺偏置随机游走
    q_accel: f32,       // 加计偏置随机游走
    r_pos: f32,         // 位置观测噪声
    r_vel: f32,         // 速度观测噪声（Doppler，m/s）^2
    r_alt: f32,         // 高度（气压）观测噪声（m）^2
    r_airspeed: f32,    // 空速观测噪声（m/s）^2
    r_vio_pos: f32,     // VIO 位置观测噪声（m）^2（中长期漂移，弱于 RTK/GPS 绝对位置）
    r_vio_vel: f32,     // VIO 速度观测噪声（m/s）^2（光流高精度，强于 GPS Doppler）
    r_rtk: f32,         // RTK-GPS 位置观测噪声（m）^2（厘米级，远强于普通 GPS）
    att_alpha: f32,     // 姿态重力修正强度（0 = 纯积分）
    mag_alpha: f32,     // 磁力计航向锚定强度（0 = 不锚定 yaw；纯陀螺积分 yaw 会漂移）
    /// 锚定交流门控状态：`a_h` 的 EMA 均值与绝对偏差（一阶包络）。
    acc_h_mean: f32,
    acc_h_dev: f32,
    /// 最近一拍 dt（s）：供 `update_mag` 等"无 dt 参数的观测回调"使用。
    last_dt: f32,
    /// 阻力加速度系数 `k = 0.5·ρ·Cd/m`（水平 2 轴共用）。
    ///
    /// 阻力加速度 `a_drag = -k·|v_rel|·v_rel`（沿相对风速反向），与
    /// `plant.rs::aero_drag_body` **同一模型**、同一参数来源（`VehicleConfig`）。
    /// **0 = 未设置/关闭**（阶段 2 的风状态在 k=0 时退化为不启用，既有行为逐位不变）。
    ///
    /// 为何需要显式设置：EKF 是**纯参数**的（`new(att_alpha, q_vel, …)`），
    /// 拿不到 `VehicleConfig` ⇒ 由调用方（固件/SIL harness）在构造后注入。
    drag_k: f32,
    /// 水平风估计（世界系 NED，m/s）——阶段 2 的观测器输出，供锚定扣减用。
    wind_est: [f32; 2],
    /// 世界系参考地磁场**三维**方向（含倾角；不必单位化），供路线 2.2e 的全姿态
    /// 修正用（强度旋钮见模块级 `G_MAG3D_ALPHA`）。
    /// 全姿态修正用；`mag_ref`（二维水平方向）继续供只修 yaw 的旧路径使用。
    mag_ref3d: [f32; 3],
    mag_ref: [f32; 2],  // 世界系水平参考地磁方向（单位向量）：默认 (1,0)=地理北；
                        // 有磁偏角时 set_mag_declination 旋转该参考 → 磁航向转地理航向
    /// 机体硬铁偏置（与磁力计同单位），由**离线标定**得到，默认零。
    /// `update_mag` 使用前先从读数中扣除。
    ///
    /// 为何需要：硬铁是机体固定偏置，会使磁航向产生**与姿态相关的常数偏置**
    /// （残余 b_h 时航向误差 ≈ atan(|b_h|/|B_h|)）。而它**无法被任何门控发现**
    /// （静止时 |ω|≈0、偏置恒定时模长/方向也不变）→ 必须靠标定。（参见
    /// `docs/stage1-attitude-findings.md` F7。）
    ///
    /// 注：工程上硬铁普遍用**离线标定**（多姿态采集取 min/max 中心）而非在线估计
    /// ——后者在静止悬停下不可观。本字段即标定结果接口，与 `mag_ref` 同形态。
    mag_hard_iron: [f32; 3],
    /// **世界系平移加速度估计**（NED，m/s²）：用于从比力中扣除平移分量后再做重力锚定。
    ///
    /// 背景（`docs/stage2-attitude-ctrl-findings.md` P4）：比力 `a_body = R^T(a_world − g)`，
    /// 重力锚直接把比力方向当重力方向。平移机动会污染它，而**两道门都盖不住**：
    ///   - 幅值门从不关：平移时 `|a|/g` 最高仅 1.28（门限 [0.5,2.5]）；
    ///   - 方向门不足：0.2g 平移只偏重力 11.3°（阈值 25.8°）。
    /// 实测 A3 急刹：平移 0.2/0.5/0.8g → 姿态误差 **6.1/14.5/21.6°**（全在 pitch）。
    ///
    /// 补偿：`a_comp = a_body − R̂ᵀ·a_world`，其方向即真实重力方向、幅值≈g。
    /// 默认 `[0,0,0]` → **与历史行为逐位一致**（零平移时补偿项为 0）。
    ///
    /// 注入源（由陷主决定）：仿真可用真值（oracle 实验）；实机可用 GPS/Doppler
    /// 速度差分。本字段只提供接口，不隐含数据来源。
    world_accel: [f32; 3],
    /// `world_accel` 是否为**可信源**（如测试 oracle / 高质量外部估计）。
    /// `false`（默认，含 Doppler 差分路径）→ 锚定侧施加**平移门控**。
    world_accel_trusted: bool,
    /// Doppler 差分用：上一次速度观测（`vel_obs_dt < 0` = 尚未初始化）。
    prev_vel_obs: [f32; 3],
    /// 自上次**新鲜**速度观测起累计的时间（s）；负值表示未初始化。
    vel_obs_dt: f32,
    airspeed_est: f32,  // 估计空速 (m/s)，由空速计融合得到
}

impl EkfEstimator {
    /// 完整构造（含速度/高度观测噪声、垂向过程噪声、加计零偏随机游走）。
    pub fn new(
        att_alpha: f32,
        q_vel: f32,
        q_vel_z: f32,
        q_bias: f32,
        q_accel: f32,
        r_pos: f32,
        r_vel: f32,
        r_alt: f32,
    ) -> Self {
        let mut p = [0.0f32; N * N];
        mat_ident(&mut p);
        for i in 0..3 {
            p[i * N + i] = 1.0; // pos ~1m^2
        }
        for i in 3..6 {
            p[i * N + i] = 1.0; // vel ~1 (m/s)^2
        }
        for i in 6..9 {
            p[i * N + i] = 1e-4; // gyro bias ~0.01 rad/s
        }
        // 加计零偏仅估计世界系 D 轴（垂向）一个分量：垂向下沉的根因是 Z 轴恒定加计零偏，
        // 单分量即可消除垂向速度积分漂移，且垂向速度/气压观测对其充分可观、数值稳定
        // （3 轴机体系零偏估计存在偏航耦合导致 Y 轴正反馈发散，故收敛为单分量）。
        // 阶段 11-A：初始不确定度放大到 0.25（std 0.5 m/s²），使速度观测能经交叉协方差
        // P[9,vel] 驱动零偏收敛——原来 1e-2 太小，交叉协方差近乎 0、卡尔曼增益可忽略，
        // 零偏恒锁在初值 0，恒定 0.05 m/s² 零偏直接积分进速度导致悬停无界发散。
        p[9 * N + 9] = 0.05; // accel_bias_z ~0.22 m/s^2（足够初始不确定度驱动收敛，又不过度吸收瞬态）
        EkfEstimator {
            att: Quaternion::IDENTITY,
            x: [0.0; N],
            p,
            g_ref: 9.81,
            q_vel,
            q_vel_z,
            q_bias,
            q_accel,
            r_pos,
            r_vel,
            r_alt,
            r_airspeed: 0.75, // ~0.87 m/s RMS 空速观测噪声
            r_vio_pos: 0.25,  // ~0.5 m RMS：VIO 位置有中长期漂移，权重弱于 GPS/RTK 绝对位置
            // ~0.2 m/s RMS：VIO 速度（光流）精度高于 GPS Doppler，但不可过紧——
            // r=0.01 时速度协方差塌缩 + ab_z 交叉增益放大，任何持久速度残差都会驱动
            // 垂向零偏指数发散（仿真：GPS 中断 + VIO 时 ab_z → 4.3e6 → NaN）。
            r_vio_vel: 0.04,
            r_rtk: 0.0025,    // ~0.05 m RMS：RTK 厘米级绝对位置，权重最强
            att_alpha,
            mag_alpha: 0.05,  // 微弱航向锚定：yaw 误差每拍吸收 2.5%（0.05*0.5）
            acc_h_mean: 0.0,
            acc_h_dev: 0.0,
            last_dt: 0.0,
            drag_k: 0.0,
            wind_est: [0.0; 2],
            mag_ref3d: [0.0; 3], // 未设置 ⇒ 全姿态修正不生效（见 update_mag）
            mag_ref: [1.0, 0.0], // 默认磁北=地理北（无偏角）
            mag_hard_iron: [0.0; 3], // 默认未标定（零偏置）
            world_accel: [0.0; 3],   // 默认零平移（→ 与历史行为一致）
            world_accel_trusted: false,
            prev_vel_obs: [0.0; 3],
            vel_obs_dt: -1.0,
            airspeed_est: 0.0,
        }
    }

    /// 兼容旧调用（零回归）：使用默认的速度/高度观测噪声、垂向过程噪声与加计零偏游走；
    /// 垂向过程噪声与水平对齐，避免垂向协方差爆炸导致高度估计发散。
    pub fn new_legacy(att_alpha: f32, q_vel: f32, q_bias: f32, r_pos: f32) -> Self {
        Self::new(att_alpha, q_vel, q_vel, q_bias, 1e-4, r_pos, 0.3, 0.3)
    }

    pub fn default_quad() -> Self {
        // 垂向速度过程噪声 `q_vel_z` 与水平 (`q_vel`) 同量级，否则垂向协方差爆炸，
        // 滤波器信任漂移的传播项而非 baro/GPS 测量，导致定高环与估计相互发散
        // （PID 实机下沉 ~3m）。baro 噪声小 (`r_alt=0.3`) 强约束高度；
        // `q_accel` 让 EKF 由速度观测在线估计并扣除加计零偏（消费级 IMU 常见 0.05 m/s²），
        // 否则垂向速度持续积分零偏而发散。
        // `att_alpha=0.02` 开启微弱重力锚定。历史教训（无门控时启用 0.02 → PID/INDI/LQR
        // 全飞到 +19m、att_err 300°+）已由**比力幅值门控**解决：仅当 |a|∈[0.5g, 2.5g]
        // 时锚定增益按 w 线性缩放，自由落体/落地碰撞尖峰自动关闭锚定，避免姿态被
        // 平移加速度分量错误引导。悬停/平稳飞行时持续锚定 roll/pitch，抑制纯陀螺积分漂移。
        // 垂向零偏 x[9] 仅由速度观测弱驱动（AB_VEL_GAIN 阻尼）。
        // att_alpha=0.02：微弱重力锚定 roll/pitch（悬停/平稳飞行时抑制纯陀螺
        // 积分漂移），与磁力计航向锚定（mag_alpha，yaw）互补成完整姿态锚定。
        // 早期 TEMP-EXPERIMENT 曾置 0（纯陀螺积分对照）——比力幅值门控 + 方向
        // 一致性门控（394c7e3 新增，夹角 >25.8° 关闭）已解决无门控时 0.02 的
        // 发散问题；394c7e3 为 SIL gps_bias_step 判据把本值一并置 0（纯陀螺
        // 积分），代价是正常悬停 roll/pitch 估计漂移 → HIL 闭环 4.8s 姿态发散
        // （mcu_p=+0.10 vs 物理 -0.54，推力饱和 0/1 边界翻滚）。门控本身已防
        // 平移误锚定，此处恢复 0.02（HIL/SIL 双回归守护）。
        // 【Joseph 形式重标定】`q_vel` 0.05 → 0.5。
        //
        // 协方差更新改成正确的 Joseph 形式后，P 不再被原误实现意外抬高，K 随之变小
        // → 估计对观测的信任变弱。实测（x_hover_noise，逼真噪声 + ALT_HOLD）：
        //   q_vel=0.05：机体获得约 1.8 m/s 水平漂移后速度环拉不回（估计算出 est_v≈0
        //               → 控制以为没漂），位置漂到 15m，max|pitch|=19.47°（超 15° 界）
        //   q_vel=0.5 ：峰值 0.64 m/s 且被拉回，位置有界 ±1.6m，max|pitch|=13.10° ✓
        //   q_vel=2.0 ：过冲，max|roll| 17.17°、max|pitch| 16.42° ✗
        // 即：把原先"白送"的协方差膨胀换成**显式的速度过程噪声裕度**。
        // 注：`globe` 侧对 Q 不敏感的用例（x_env_noise_perturb::accel_bias_tolerated）
        // 不随本项改善，另行处理。
        Self::new(0.02, 0.5, 0.05, 1e-5, 5e-4, 0.5, 0.3, 0.3)
    }

    /// 当前估计的陀螺零偏（调试/诊断用）。
    pub fn gyro_bias(&self) -> [f32; 3] {
        [self.x[6], self.x[7], self.x[8]]
    }

    /// 当前估计的加计零偏（m/s²，调试/诊断用）。
    /// 仅估计世界系 D 轴（垂向）一个分量 `x[9]`（hover 近水平时 ≈ 机体 Z 轴零偏），
    /// 以 `[0, 0, ab_z]` 形式返回以兼容既有 3 轴接口。
    /// 该状态**仅由速度观测（Doppler）驱动**（见 `update_vel`），位置/气压更新刻意不修正
    /// （其卡尔曼增益第 9 行被清零），避免无速度观测时不可观而发散。
    pub fn accel_bias(&self) -> [f32; 3] {
        [0.0, 0.0, self.x[9]]
    }

    /// 当前估计的姿态四元数（调试/诊断用）。
    pub fn att(&self) -> Quaternion {
        self.att
    }

    /// 当前协方差矩阵（9x9 行主序）引用，供不变量校验（对称半正定）使用。
    pub fn cov(&self) -> &[f32; N * N] {
        &self.p
    }

    /// 阶段 11-A：设置 EKF 初始位置估计（NED，向下正）。
    ///
    /// 默认构造 x[0..3]=0，但机体真实初始位置（如悬停 5m，NED d=-5）与之不符，
    /// 导致 GPS/气压首次校正前 PID 看到 ~5m 位置误差全油门弹射（见 PLAN 阶段 11-A）。
    /// 在仿真/实飞启动前用真实初始位置初始化，使首拍 ez≈0，避免初始 windup 弹射。
    /// 位置协方差压到 ~0.01m²（视为已收敛，避免初始大协方差经 F 矩阵耦合膨胀）。
    /// 注入阻力加速度系数 `k = 0.5·ρ·Cd/m`（水平 2 轴共用）。见 `drag_k` 字段说明。
    ///
    /// 调用方应从 `VehicleConfig` 按其定义算出（ρ 取 `air_density`、Cd 取
    /// `drag_coeff[0]`（水平）、m 取 `mass`）。**传 0 = 关闭阶段 2 风模型**。
    /// 设置世界系参考地磁场**三维**方向（含倾角），供全姿态修正（路线 2.2e）用。
    /// 传全零 = 不启用（回到只修 yaw）。典型：本仓世界场 `[0.5, 0, 0.4]`
    /// （水平 0.5、垂向 0.4，倾角 ≈38.7°）。
    pub fn set_mag_ref3d(&mut self, b: [f32; 3]) {
        self.mag_ref3d = b;
    }

    pub fn set_drag_k(&mut self, k: f32) {
        self.drag_k = if k.is_finite() && k > 0.0 { k } else { 0.0 };
    }

    /// 当前水平风估计（世界系 NED，m/s）。
    pub fn wind_estimate(&self) -> [f32; 2] {
        self.wind_est
    }

    pub fn set_initial_position(&mut self, ned: [f32; 3]) {
        self.x[0] = ned[0];
        self.x[1] = ned[1];
        self.x[2] = ned[2];
        for i in 0..3 {
            self.p[i * N + i] = 0.01;
        }
    }

    /// HIL：设置 EKF 初始姿态四元数。
    ///
    /// 默认构造 att=IDENTITY（0°），若物理引擎起始姿态非 0°（或机头 yaw 非 0），
    /// 首拍即带姿态误差；陀螺零偏未收敛时会经积分累积放大（历史教训：pitch 恒定 90°）。
    /// 启动前用物理引擎真值姿态初始化，使首拍姿态误差≈0。
    pub fn set_initial_attitude(&mut self, q: Quaternion) {
        self.att = q.normalize();
    }
}

impl Estimator for EkfEstimator {
    fn step(
        &mut self,
        dt: Second,
        imu: ImuSample,
        gps: Option<PosSample>,
        airspeed: Option<AirspeedSample>,
    ) -> VehicleState {
        let dt = dt.0;
        self.last_dt = dt;
        // 标定旋钮（易失读：由外部写入，普通读会被常量折叠掉）
        let gyro_bias_k = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_GYRO_BIAS_K)) };
        let att_acc_ac = {
            let v = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_ATT_ACC_AC)) };
            if v >= 0.0 { v } else { ATT_ACC_AC_DEFAULT }
        };
        let att_alpha_eff = {
            let v = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_ATT_ALPHA)) };
            if v >= 0.0 { v } else { self.att_alpha }
        };
        let att_acc_gate = {
            let v = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_ATT_ACC_GATE)) };
            if v >= 0.0 { v } else { ATT_ACC_GATE_DEFAULT }
        };
        // 累计自上次新鲜速度观测的时间（供 `update_vel_r` 的 Doppler 差分用）。
        if self.vel_obs_dt >= 0.0 {
            self.vel_obs_dt += dt;
        }
        // 防御：IMU 输入非有限（HIL 注入异常/总线噪声）直接放弃本拍积分，返回上一状态。
        // 否则 att.integrate(NaN) 会把姿态四元数直接污染为 NaN（历史教训：电机指令 NaN
        // 即由状态/姿态 NaN 传播而来），进而经位置预测/观测更新污染整个状态向量。
        crate::perf::probe(8); // EKF::step 入口
        let gyro_finite = imu.gyro.iter().all(|g| g.0.is_finite());
        let accel_finite = imu.accel.iter().all(|a| a.0.is_finite());
        if !gyro_finite || !accel_finite {
            return self.state();
        }
        // 防御：协方差若出现非有限项（GPS/位置观测的 Joseph 更新在特定数值下可能产生 NaN，
        // 对角夹取只清对角、非对角 NaN 会残留并传播），整矩阵清掉非有限项，阻断 NaN 进入
        // 本拍的 predict/update（否则 S 求逆/增益 K 计算 NaN → 位置/速度状态被污染）。
        self.sanitize_p();
        let g = self.g_ref;

        // 去偏置角速度
        let bx = self.x[6];        let by = self.x[7];
        let bz = self.x[8];
        let wx = imu.gyro[0].0 - bx;
        let wy = imu.gyro[1].0 - by;
        let wz = imu.gyro[2].0 - bz;

        // 名义姿态积分（去偏置）
        self.att = self.att.integrate(wx, wy, wz, dt);

        // 可选微弱重力修正（锚定 roll/pitch 到重力方向）
        if att_alpha_eff > 0.0 {
            // 比力直接使用，**不做额外低通**。
            //
            // 历史：此处原有一级 `accel_lp`（`lp_coeff=0.05`，fc≈2.04Hz，注释称"滤 40Hz 振动"）。
            // 实测证明它**冗余且是重力锚定相位滞后的唯一根因**：`step_hil` 在把 accel
            // 交给 EKF **之前**已完成 40Hz 陷波 + 20Hz 低通，而锚定自身的低增益
            // （`k≈0.01`/拍）本身就是低通。它使 roll/pitch 在 1Hz 滞后 27.4°、增益 0.790，
            // 直接侵占姿态环相位裕度（`att_kp=3.0 rad/s ≈ 0.48Hz` 交叉频率）。
            //
            // 移除后（H 场实测，`docs/stage1-attitude-findings.md` F1）：
            // |        | 0.5Hz | 1Hz | 2Hz | 3Hz | 纯静止振动 RMSE |
            // |--------|-------|-----|-----|-----|-------|
            // | 有低通 | 0.941/+15.1° | 0.790/+27.4° | 0.424/+34.8° | 0.255/— | 0.5301° |
            // | 已移除 | 0.995/+2.1° | 0.982/+4.0° | 0.937/+7.0° | 0.881/+8.3° | 0.5343° |
            // （增益/滞后）→ **抗振不变、漂移抑制不变**（`att_alpha` 未动），纯收益。
            // 2–3Hz 残余滞后（7.0°/8.3°）来自 `step_hil` 上游 20Hz 低通，非此级。
            //
            // 回归守卫：
            // - `att_est.rs::spec_attitude_estimator_frequency_response`（频响硬门槛）
            //   —— 若有人重新加回慢低通，1Hz 滞后会立即超标、该门槛变红。
            // - `att_est.rs::vibration_rejection_without_reference_lpf`（抗振不能退化）
            let a = [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0];
            // ---- 平移补偿（见 `world_accel` 字段文档 / stage2 P4）----
            // a_body = R^T(a_world − g)。扣除平移分量 R^T·a_world 后余下纯重力项
            // R^T(−g)：其**方向**才是真实重力方向，其**幅值**也才真正≈g
            // （使幅值门恢复有效性）。
            // 默认 world_accel=[0,0,0] → 本块为 no-op，行为逐位不变。
            let a = if self.world_accel[0] != 0.0
                || self.world_accel[1] != 0.0
                || self.world_accel[2] != 0.0
            {
                // 平移补偿：`a_comp = a_body − gate·R̂ᵀ·a_world`。
                //
                // **平移门控**：Doppler 差分的噪声/滞后在**小平移**时是纯负担
                // （实测悬停 RMSE 0.95°→2.20°），而小平移本就无需补偿 ⇒ 按
                // `|a_world|/g` 在 [0.05, 0.20] 线性开启（0.05≈3° 倾角）。
                // 平移大时补偿收益远大于其噪声（实测 52~61%）。
                // **可信源**（`set_world_accel`，如测试 oracle）免门控。
                let aw = self.world_accel;
                let gate = if self.world_accel_trusted {
                    1.0
                } else {
                    let m = sqrt(aw[0] * aw[0] + aw[1] * aw[1] + aw[2] * aw[2]) / g;
                    ((m - 0.05) / 0.15).clamp(0.0, 1.0)
                };
                let aw_b = crate::vehicle::rotate_vec_by_quat_inverse(self.att, aw);
                [
                    a[0] - gate * aw_b[0],
                    a[1] - gate * aw_b[1],
                    a[2] - gate * aw_b[2],
                ]
            } else {
                a
            };
            let an = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            // 比力幅值门控：仅当 |a| 接近 g 时才信任加速度计的「重力方向」参考。
            //   - 自由落体/失重 |a|≈0：比力方向无意义，若照常锚定会把姿态拖向随机方向
            //     （历史教训：att_alpha=0.02 未加门控 → PID/INDI/LQR 全飞到 +19m、att_err 300°+）。
            //   - 高机动/落地碰撞尖峰 |a|≫g：比力含平移加速度分量，同样不代表重力方向。
            //   门控权重 w：|a|=g 时 w=1，向边界（0.5g / 2.5g）线性衰减到 0，增益按 w 缩放。
            if an > 1e-3 {
                let ratio = an / g;
                let w = if ratio < 0.5 || ratio > 2.5 {
                    0.0
                } else if ratio < 1.0 {
                    (ratio - 0.5) / 0.5
                } else {
                    (2.5 - ratio) / 1.5
                };
                let w = w.clamp(0.0, 1.0);
                let down_body = crate::vehicle::rotate_vec_by_quat_inverse(self.att, [0.0, 0.0, g]);
                // 方向一致性门控：比力反方向（-a，估计的重力参考）与当前估计重力方向
                // down_body 的夹角。静止/匀速悬停时比力≈纯重力（夹角≈0，仅陀螺漂移
                // 引入微小偏差）→ 全锚定；平移机动/倾角飞行时比力含平移加速度分量，
                // 方向偏离重力（夹角 >~26°）→ 关闭锚定。
                // 仅靠幅值门控（|a|≈g）挡不住匀速平移：幅值≈g 但方向不代表重力，
                // 锚定会把姿态拖向错误方向——GPS 偏置故障场景（SIL sensor_fault
                // gps_bias_step）实测 tilt 81°（应 <45°），加方向门控后恢复。
                let n_inv = [-a[0] / an, -a[1] / an, -a[2] / an];
                let cos_t = (down_body[0] * n_inv[0] + down_body[1] * n_inv[1] + down_body[2] * n_inv[2])
                    .clamp(-1.0, 1.0);
                // cos 0.90 ≈ 25.8°：夹角 <25.8° 线性加权，>25.8° 完全关闭。
                let w_align = if cos_t < 0.90 {
                    0.0
                } else {
                    ((cos_t - 0.90) / 0.10).clamp(0.0, 1.0)
                };
                // 陀螺幅值门控：机动/协调转弯时关闭重力锚定。
                // 协调转弯的向心加速度由 roll 平衡 → 比力方向≈竖直（幅值≈g），
                // 幅值/方向门控都无法区分"水平加速"与"重力"——锚定会把协调转弯
                // 的 roll 错误拉向 0（虚拟外设实测稳态 2.2° vs 真值 27°）。|omega|
                // 大 → 关闭锚定（纯陀螺积分跟踪机动姿态）；悬停/平稳（|omega|≈
                // 陀螺噪声量级）→ 全锚定（抑制纯陀螺积分漂移）。
                let om = sqrt(
                    imu.gyro[0].0 * imu.gyro[0].0
                        + imu.gyro[1].0 * imu.gyro[1].0
                        + imu.gyro[2].0 * imu.gyro[2].0,
                );
                let w_gyro = if om < 0.25 {
                    1.0
                } else if om < 0.6 {
                    (0.6 - om) / 0.35
                } else {
                    0.0
                };
                // 第四个门控：水平非重力加速度（见 G_ATT_ACC_GATE 的说明）。
                // 世界系比力 = R·a；重力竖直 ⇒ 其水平分量即非重力水平加速度。
                let a_world_h = crate::vehicle::rotate_vec_by_quat(self.att, a);
                let a_h = sqrt(a_world_h[0] * a_world_h[0] + a_world_h[1] * a_world_h[1]);
                let gate_full = att_acc_gate;
                let gate_zero = att_acc_gate / 3.0;
                let w_inst = if gate_full <= 0.0 {
                    1.0 // 瞬时门控关闭
                } else if a_h <= gate_zero {
                    1.0
                } else if a_h >= gate_full {
                    0.0
                } else {
                    (gate_full - a_h) / (gate_full - gate_zero)
                };
                // 交流能量门控：a_h 的 EMA 均值/绝对偏差（一阶包络），看**偏差**。
                // EMA 系数按 ~1s 量级（与阵风主频 0.12Hz≈8.3s 周期相比足够快，
                // 能跟上包络；又比单拍噪声慢，不抖）。
                self.acc_h_mean += 0.02 * (a_h - self.acc_h_mean);
                self.acc_h_dev += 0.02 * ((a_h - self.acc_h_mean).abs() - self.acc_h_dev);
                let w_ac = if att_acc_ac <= 0.0 {
                    1.0 // 交流门控关闭
                } else if self.acc_h_dev <= att_acc_ac / 3.0 {
                    1.0
                } else if self.acc_h_dev >= att_acc_ac {
                    0.0
                } else {
                    (att_acc_ac - self.acc_h_dev) / (att_acc_ac - att_acc_ac / 3.0)
                };
                let w_acc = w_inst * w_ac;
                let k = att_alpha_eff * 0.5 * w * w_align * w_gyro * w_acc;
                // 把估计重力向量 down_body 锚定到【真实重力方向】，即比力的反方向 (-a)。
                // 修正轴 = down_body × (-a/an)：n 为垂直于二者的旋转轴，
                // 右乘（机体系）dq 使 down_body 旋转向 -a，姿态向水平收敛。
                //   （符号教训：若用 a_hat × down_body 方向反 → 锚定放大倾斜而非收敛，
                //   姿态被逐渐推向倒扣 180°，且倒扣处叉积恰为零 → 冻结在 r=-179.9°。）
                let ax = (down_body[1] * (a[2] / an) - down_body[2] * (a[1] / an)) * k;
                let ay = (down_body[2] * (a[0] / an) - down_body[0] * (a[2] / an)) * k;
                let az = (down_body[0] * (a[1] / an) - down_body[1] * (a[0] / an)) * k;
                let na = sqrt(ax * ax + ay * ay + az * az);
                if na > 1e-6 {
                    let dq = Quaternion::from_axis_angle([ax, ay, az], Radian(na));
                    self.att = (self.att * dq).normalize();
                }

                // ---- 陀螺零偏在线估计（把 `x[6..8]` 从不被观测更新接上）----
                //
                // 依据：锚定每拍施加的修正角 `(ax,ay,az)` 就是"零偏造成的姿态漂移"
                // 被拉回的体现。**稳态下修正速率 = −陀螺零偏**（零偏使姿态漂走、
                // 锚定把它拉回，二者速率相等反向）⇒ 以 `-(ax,ay,az)/dt` 为观测量
                // 一阶收敛到它，即得零偏估计（互补滤波的标准做法）。
                //
                // 关键：**只复用上方已算好的门控**（w 幅值 / w_align 方向 /
                // w_gyro 角速率）—— 机动、失重、大角速率时门控归零 ⇒ 不学习，
                // 避免"把机动学成零偏"。
                //
                // 历史：`x[6..8]` 一直是状态向量的一部分、传播时也被扣除
                // （`gyro - x[6..8]`），但**从未被任何观测更新** ⇒ 恒为 0，
                // 姿态漂移只能靠锚定"当场拉"，没有前馈补偿。实测后果（H 场无风档）：
                // 陀螺漂移 0.34°/min 造成 **9.36m/60s 持续位置漂移且仍在增长**；
                // 而只注入加计漂移时仅 1.52m（= 完全无注入）⇒ 该漂移**完全**
                // 由陀螺零偏贡献（分离实验，见 wind_turb_scan::imu_bias_separation_no_wind）。
                let gyro_learn = gyro_bias_k * w * w_align * w_gyro * w_acc;
                if gyro_learn > 0.0 {
                    let inv_dt = 1.0 / dt;
                    let obs = [-ax * inv_dt, -ay * inv_dt, -az * inv_dt];
                    self.x[6] += gyro_learn * (obs[0] - self.x[6]) * dt;
                    self.x[7] += gyro_learn * (obs[1] - self.x[7]) * dt;
                    self.x[8] += gyro_learn * (obs[2] - self.x[8]) * dt;
                    for i in 6..9 {
                        self.x[i] = self.x[i].clamp(-GYRO_BIAS_MAX, GYRO_BIAS_MAX);
                    }
                }
            }
        }

        // ---- 阶段 2：水平风观测器（drag_k > 0 时生效；默认 0 = 完全不执行）----
        //
        // 依据（准稳态力平衡）：机体 -Z（推力方向）水平分量与阻力相消
        //     m·g·tanθ = k·m·|v_rel|·v_rel        （k = 0.5ρCd/m，见 drag_k）
        // ⇒ |v_rel| = sqrt(g·tanθ / k)，方向 = 推力水平分量方向（即倾斜指向）
        // ⇒ v_wind = v_ground − v_rel
        //
        // 用途（阶段 2.3）：把**模型化的阻力**从比力中扣掉，锚定看到的才是纯重力 ——
        // 这正是"H 场湍流风下锚定劣化"的正面解（湍流瞬间的 ΔD 被解释掉）。
        //
        // 局限（如实标注）：本式是**准稳态代数解**，湍流瞬态下 θ 与 v_rel 不同步 ⇒
        // 估计滞后；且悬停时若 θ≈0 则 v_rel≈0（与"风在无相对运动时不可观"一致）。
        // 之所以先做这个而不直接扩协方差（N:10→12）：**不动 N/P/F 的维度**，
        // 风险与验证成本低得多；后续若要更准可升级为风状态（方案阶段 2.2 已备规格）。
        if self.drag_k > 0.0 {
            let up_b = crate::vehicle::rotate_vec_by_quat(self.att, [0.0, 0.0, 1.0]);
            let hx = up_b[0];
            let hy = up_b[1];
            let hnorm = sqrt(hx * hx + hy * hy);
            let mut vw = [0.0f32; 2];
            if hnorm > 1e-3 && up_b[2].abs() > 1e-3 {
                let tan_t = hnorm / up_b[2].abs();
                let vr = sqrt((g * tan_t / self.drag_k).max(0.0));
                // ⚠️ 符号（首版写错，开环对真值验证时抓到）：
                // `up_b = rotate_vec_by_quat(att, [0,0,1])` 在 NED/FRD 约定下是机体的
                // **+Z（朝下）**，而**推力沿 −Z** ⇒ 推力的水平方向 = 该向量水平分量的
                // **负向**。首版漏了这个负号，估计出的风与真值**符号相反**
                // （实测：真值 北+5.40/东+2.16，估计 北-3.31/东-2.23）。
                let dir = [-hx / hnorm, -hy / hnorm]; // 推力水平方向（朝上风）
                // v_rel 指向推力水平分量方向；v_wind = v_ground − v_rel
                vw = [self.x[3] - dir[0] * vr, self.x[4] - dir[1] * vr];
            }
            // 一阶低通（~0.5s 量级）：代数解瞬时噪声大，且湍流下需平滑
            const W_ALPHA: f32 = 0.02;
            self.wind_est[0] += W_ALPHA * (vw[0] - self.wind_est[0]);
            self.wind_est[1] += W_ALPHA * (vw[1] - self.wind_est[1]);
            for i in 0..2 {
                self.wind_est[i] = self.wind_est[i].clamp(-30.0, 30.0);
            }
        } else {
            self.wind_est = [0.0; 2];
        }

        // 位置/速度预测（世界系 NED）：a_world = R*(a_body) - [0,0,ab_z] + g_vec(z 向下正)
        // 仅扣除垂向（世界 D 轴）估计零偏 ab_z = x[9]：hover 近水平时机体 Z 零偏 ≈ 世界 D 零偏，
        // 单分量足以消除垂向速度积分漂移，且数值稳定（避免 3 轴耦合发散）。
        let a = [imu.accel[0].0, imu.accel[1].0, imu.accel[2].0];
        let a_world = rotate_vec_by_quat(self.att, a);
        let ax_w = a_world[0];
        let ay_w = a_world[1];
        let az_w = a_world[2] + g - self.x[9];

        let px = self.x[0] + self.x[3] * dt + 0.5 * ax_w * dt * dt;
        let py = self.x[1] + self.x[4] * dt + 0.5 * ay_w * dt * dt;
        let pz = self.x[2] + self.x[5] * dt + 0.5 * az_w * dt * dt;
        let vx = self.x[3] + ax_w * dt;
        let vy = self.x[4] + ay_w * dt;
        let vz = self.x[5] + az_w * dt;
        self.x[0] = px;
        self.x[1] = py;
        self.x[2] = pz;
        self.x[3] = vx;
        self.x[4] = vy;
        self.x[5] = vz;

        // 协方差预测（独立方法，栈数组作用域限于该方法内，返回后栈槽被回收，
        // 避免与下方观测更新步的栈数组同时存活导致调用方任务栈溢出）。
        crate::perf::probe(9); // 姿态积分 + 重力锚定完成
        self.predict_cov(dt);
        crate::perf::probe(10); // 协方差预测完成

        // 观测更新（位置 + 可选 Doppler 速度）：H = [I3 0 0]（独立方法，如上拆分）。
        if let Some(z) = gps {
            let vel_mps = z.vel.map(|v| [v[0].0, v[1].0, v[2].0]);
            self.update_pos(z);
            if let Some(v) = vel_mps {
                self.update_vel(v);
            }
        }

        // 观测更新（空速）：约束水平速度幅值 |v_h| = 测量空速（不含风）。
        if let Some(a) = airspeed {
            self.update_airspeed(a.speed.0);
        }

        crate::perf::probe(11); // 观测更新（GPS 位置/速度）完成

        // 加计零偏（世界 D 轴）物理上界夹取（消费级 IMU 零偏通常 < 0.3 m/s²，真值仅 0.05）。
        // 该状态仅由速度观测（Doppler）驱动，正常收敛不受影响；夹取仅作发散兜底。
        if self.x[9] > 0.3 {
            self.x[9] = 0.3;
        } else if self.x[9] < -0.3 {
            self.x[9] = -0.3;
        }

        VehicleState {
            time_boot_ms: 0,
            pos: [Meter(self.x[0]), Meter(self.x[1]), Meter(self.x[2])],
            vel: [MeterPerSecond(self.x[3]), MeterPerSecond(self.x[4]), MeterPerSecond(self.x[5])],
            att: self.att,
            omega: [
                RadianPerSecond(imu.gyro[0].0 - self.x[6]),
                RadianPerSecond(imu.gyro[1].0 - self.x[7]),
                RadianPerSecond(imu.gyro[2].0 - self.x[8]),
            ],
            airspeed: MeterPerSecond(self.airspeed_est),
            accel_bias: self.accel_bias(),
        }
    }

    /// 注入 VIO 测量：位置（中等噪声，容忍长期漂移）+ 速度（小噪声，光流高精度）。
    /// 在 GPS 帧间 / 失锁时提供连续修正，与 RTK 绝对参考互补。
    fn update_vio(&mut self, vio: Option<VioSample>) {
        if let Some(v) = vio {
            if let Some(pos) = v.pos {
                self.update_pos_r(PosSample::pos_only(pos), self.r_vio_pos);
            }
            if let Some(vel) = v.vel {
                self.update_vel_r([vel[0].0, vel[1].0, vel[2].0], self.r_vio_vel);
            }
        }
    }

    /// 注入 RTK-GPS 测量：厘米级绝对位置（极小噪声），压紧位置协方差、抑制 VIO 漂移。
    fn update_rtk(&mut self, rtk: Option<RtkSample>) {
        if let Some(z) = rtk {
            self.update_pos_r(PosSample::pos_only(z.pos), self.r_rtk);
        }
    }

    /// 当前估计状态（不推进），供诊断/日志读取（已含所有已融合观测）。
    fn state(&self) -> VehicleState {
        VehicleState {
            time_boot_ms: 0,
            pos: [Meter(self.x[0]), Meter(self.x[1]), Meter(self.x[2])],
            vel: [MeterPerSecond(self.x[3]), MeterPerSecond(self.x[4]), MeterPerSecond(self.x[5])],
            att: self.att,
            omega: [
                RadianPerSecond(self.x[3] * 0.0), // 占位；角速度不在此状态
                RadianPerSecond(0.0),
                RadianPerSecond(0.0),
            ],
            airspeed: MeterPerSecond(self.airspeed_est),
            accel_bias: self.accel_bias(),
        }
    }

    fn reset(&mut self) {
        self.att = Quaternion::IDENTITY;
        self.x = [0.0; N];
        mat_ident(&mut self.p);
        for i in 0..3 {
            self.p[i * N + i] = 1.0;
        }
        for i in 3..6 {
            self.p[i * N + i] = 1.0;
        }
        for i in 6..9 {
            self.p[i * N + i] = 1e-4; // gyro bias
        }
        self.p[9 * N + 9] = 0.05; // accel bias (vertical) — 与 new() 一致，保证可观可收敛
    }

    // HIL/共享单步：转发到同名的固有方法（trait 默认 no-op，EKF 实际生效）。
    fn set_initial_attitude(&mut self, q: Quaternion) {
        EkfEstimator::set_initial_attitude(self, q);
    }

    fn set_initial_position(&mut self, ned: [f32; 3]) {
        EkfEstimator::set_initial_position(self, ned);
    }

    fn update_alt(&mut self, alt: f32) {
        EkfEstimator::update_alt(self, alt);
    }

    /// 磁力计航向锚定（yaw）。
    ///
    /// ⚠️ **本委托不可删**：`update_mag` 的实现在下面的固有 impl 块
    /// （"非 trait 方法"区），而 `HilContext<E: Estimator, C>` 持有的是泛型参数，
    /// 方法解析只看 trait 方法。缺本委托时会命中 `trait_def.rs` 的默认空实现
    /// （`fn update_mag(&mut self, _mag: Option<[f32;3]>) {}`）→
    /// **磁航向锚定在整条 SIL/HIL/MCU 链路静默失效，yaw 退化为纯陀螺积分**。
    ///
    /// 历史教训：该 bug 曾长期存在而未被发现，因为
    /// ① 单测直接对具体类型 `ekf.update_mag(...)` → 命中固有方法 → 全绿；
    /// ② 集成测试用 `SensorConfig::default()`（零陀螺零偏）→ yaw 本就不漂 → 空过。
    /// 回归守卫：`fly-sim-core/tests/att_est.rs::drift_rejection_still_works_after_fix`
    /// （经 `step_hil` 注入非零陀螺零偏 + 磁力计，断言 yaw 仍有界）。
    /// 与 `update_alt` 同一模式。
    fn update_mag(&mut self, mag: Option<[f32; 3]>) {
        EkfEstimator::update_mag(self, mag);
    }
}

// ===== 以下为 `EkfEstimator` 自身的非 trait 方法（不是 `Estimator` trait 的一部分）=====
// 把协方差预测 / 位置观测更新从 `step` 里拆成独立方法，让各自的大栈数组（各 ~1620B）
// 作用域不重叠：编译器可在两支之间复用同一批栈槽，整体峰值 ≈ max(predict, update)
// 而非 sum，从而避免控制任务栈（3072B）溢出。

impl EkfEstimator {
    /// 重置滤波器并把初始姿态设为给定四元数（覆盖默认的水平 IDENTITY）。
    /// 用于 SIL/HIL 等已知初始姿态的场景，使 EKF 初值与物理初始姿态对齐，
    /// 避免“从水平零姿态出发、悬停稳态无收敛动力导致姿态永久错 90°”的发散。
    pub fn reset_to(&mut self, att: Quaternion) {
        self.reset();
        self.att = att;
    }

    /// 防御：清除协方差矩阵中的所有非有限项（NaN/Inf → 0），阻断数值污染传播。
    /// 对角夹取只处理对角，非对角 NaN 会残留并进入 S 求逆/K 增益计算 → 状态 NaN；
    /// 在 step 开头统一清理，保证本拍 predict/update 输入协方差干净。
    fn sanitize_p(&mut self) {
        for i in 0..(N * N) {
            if !self.p[i].is_finite() {
                self.p[i] = 0.0;
            }
        }
    }

    /// 协方差预测步：F = I + A*dt，P' = F P F^T + Q。
    /// 栈数组（f/ft/p_pred，各 81 元素）作用域限于本方法，返回后栈槽被回收。
    fn predict_cov(&mut self, dt: f32) {
        // F = I + A*dt
        // - [pos][vel] = I（位置由速度推进）
        // - [vel][accel_bias] = -R*dt（加计零偏直接影响速度，使其经速度观测可估计）
        // F 只有 4 个非单位元（其余为单位阵）：
        //   F[0][3] = F[1][4] = F[2][5] = dt（pos ← vel）
        //   F[5][9] = -dt               （vel_z ← accel_bias_z；
        //                                 az_w = a_world[2] + g - ab_z ⇒ ∂vel_z/∂ab_z = -1）
        // 因此 Fᵀ·P 只改 4 行、F·(FᵀP) 只改 4 行，用稠密 10³ 乘是白算 25 倍。
        // 下面按稀疏结构展开，结果与 `mat_mul_at(f,p)` / `mat_mul(f,ft)` 等价。
        let p = &self.p;
        let mut ft = [0.0f32; N * N];
        ft.copy_from_slice(p);
        // ft = Fᵀ P：row3 += dt·row0, row4 += dt·row1, row5 += dt·row2, row9 -= dt·row5
        for (dst, src, coef) in [(3usize, 0usize, dt), (4, 1, dt), (5, 2, dt), (9, 5, -dt)] {
            for j in 0..N {
                ft[dst * N + j] += coef * p[src * N + j];
            }
        }
        // p_pred = F ft：row0 += dt·row3, row1 += dt·row4, row2 += dt·row5, row5 -= dt·row9
        let mut p_pred = [0.0f32; N * N];
        p_pred.copy_from_slice(&ft);
        for (dst, src, coef) in [(0usize, 3usize, dt), (1, 4, dt), (2, 5, dt), (5, 9, -dt)] {
            for j in 0..N {
                p_pred[dst * N + j] += coef * ft[src * N + j];
            }
        }
        let q_vel_eff = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_Q_VEL)) };
            if ov != 0.0 { ov } else { self.q_vel }
        };
        let qv = q_vel_eff * dt;
        let qvz = self.q_vel_z * dt;
        let qb = self.q_bias * dt;
        // 易失读：标定旋钮由外部写入（见 G_Q_ACCEL），普通读会被常量折叠掉
        let q_accel_eff = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_Q_ACCEL)) };
            if ov != 0.0 { ov } else { self.q_accel }
        };
        let qa = q_accel_eff * dt;
        for i in 3..5 {
            p_pred[i * N + i] += qv;
        }
        p_pred[5 * N + 5] += qvz; // 垂向速度过程噪声单独
        for i in 6..9 {
            p_pred[i * N + i] += qb; // 陀螺零偏随机游走
        }
        p_pred[9 * N + 9] += qa; // 垂向加计零偏随机游走（单分量）
        // 对称化 + 对角线上下限夹取（保持 PSD 且防止不可观状态cov runaway→NaN）。
        // 上限 P_MAX 阻断 vel↔accel_bias 正反馈：无速度观测时 accel_bias 不可观，
        // 其协方差会随速度协方差经 F 耦合指数增长，夹取后发散被阻断（实飞有速度观测时远不会触及上限）。
        for i in 0..N {
            for j in (i + 1)..N {
                let avg = 0.5 * (p_pred[i * N + j] + p_pred[j * N + i]);
                p_pred[i * N + j] = avg;
                p_pred[j * N + i] = avg;
            }
            let d = p_pred[i * N + i];
            if !d.is_finite() || d < 1e-6 {
                p_pred[i * N + i] = 1e-6; // 非有限(NaN/Inf) 重置，阻断协方差 NaN 传播
            } else if d > P_MAX {
                p_pred[i * N + i] = P_MAX;
            }
        }
        self.p = p_pred;
    }

    /// 协方差更新步：**Joseph 形式 `P = A P A^T + K R K^T`（A = I - KH）**。
    ///
    /// **【已决策：以 Joseph 形式为准，不得退回下列误实现】**
    ///
    /// 原实现是：
    /// ```text
    ///   ap   = A * P
    ///   apat = mat_mul_at(ap, A)   // 注释写 "A P A^T"，但 mat_mul_at(x,y)=x^T*y，
    ///                              // 实际算的是 ap^T·A = P A^T A
    ///   P    = apat + K R K^T ;  再对称化
    /// ```
    /// 该式既非简化式 `(I-KH)P` 也非 Joseph 式：`A` 不对称（`A[i][j] = -K[i][j-c]`
    /// 而 `A[j][i] = 0`），等于把纠正项乘在错误的一侧，**得到的不是卡尔曼后验协方差**
    /// （实测其 `P[9][9]` 偏大 5.4×、`P[0][0]` 偏大 16%）。
    ///
    /// 为何选 Joseph 式而非更省算力的简化式 `P - K H P`：
    /// 1) 两者在 K 为最优增益时代数等价；但 Joseph 式是**两个对称半正定项之和**，
    ///    舍入后仍保对称与半正定，而 `P - KHP` 会因大数相减失去半正定；
    /// 2) Joseph 式对**非最优 K** 二次稳定 —— 本实现恰有 `K_MAX` 限幅与"不可观行清零"；
    /// 3) 本实现的 `H` 是窄观测矩阵，稀疏化后 Joseph 式仍为 `O(N²·ncols)`，
    ///    与简化式同算力等级，没有为省算力牺牲鲁棒性的理由。
    /// （业界参考：PX4 ECL / ArduPilot NavEKF3 采用简化式并额外做对称化 + 方差下限
    /// 钳制；教科书 Simon《Optimal State Estimation》§5.2、Maybeck 把 Joseph 式列为
    /// 数值鲁棒首选。二者都远优于原误实现。）
    ///
    /// ⚠️ **该式会改变位置/高度通道行为，必须配套重标定**：原误实现的协方差膨胀曾
    /// 意外充当鲁棒性拐杖，`EkfEstimator::default_quad()` 的 Q/R 是围绕它标定的。
    /// 修正后（未重标定）的基线实测：`x_env_noise_perturb::accel_bias_tolerated` FAIL
    /// （加计偏置下位置 5.52m > 5.0m 界）、`x_hover_noise` max|roll|=21.94°；修正前分别
    /// PASS / 5.03°。补偿应把拐杖换成**显式的 Q/R 设计裕度**（优先增大 `q_accel`，让
    /// 加计零偏状态的协方差按需增长），**不得退回错式**。
    ///
    /// **姿态通道不受影响**：陀螺零偏 `x[6..8]` 从不被任何观测更新（观测只选
    /// pos/vel/alt 列，增益恒为 0），且姿态是名义四元数积分、其重力/磁修正
    /// （`att_alpha`/`mag_alpha`）在协方差之外做固定-α 直接修正。宿主对照（弱磁让零偏
    /// 可观）显示两种式下姿态输出逐位相同。
    ///
    /// 稀疏化依据：`H` 只在状态列 `c..c+ncols` 上取单位观测，`A` 只在那些列偏离单位阵：
    /// ```text
    ///   ap[i][j]    = P[i][j]    - Σ_l K[i][l]·P[c+l][j]
    ///   (A P A^T)[i][j] = ap[i][j] - Σ_l K[i][l]·ap[c+l][j]
    /// ```
    ///
    /// `k` 为 N×ncols 行主序卡尔曼增益（已限幅/已清零不可观行）。
    /// `nan_reset`/`clamp_max` 保留各调用点原有的对角处理策略
    /// （`update_vel_r` 只做下限，位置/气压更新额外夹 P_MAX）。
    fn joseph_update_cov(
        &mut self,
        k: &[f32],
        ncols: usize,
        c: usize,
        r: f32,
        nan_reset: bool,
        clamp_max: bool,
    ) {
        let p = &self.p;
        // 1) ap = (I - K H) P
        let mut ap = [0.0f32; N * N];
        for i in 0..N {
            let ki = &k[i * ncols..i * ncols + ncols];
            for j in 0..N {
                let mut acc = p[i * N + j];
                for (l, kl) in ki.iter().enumerate() {
                    acc -= kl * p[(c + l) * N + j];
                }
                ap[i * N + j] = acc;
            }
        }
        // 2) P = A P A^T + K R K^T（Joseph 形式）
        //    (A P A^T)[i][j] = Σ_k A[i][k]·ap[k][j] = ap[i][j] - Σ_l K[i][l]·ap[c+l][j]
        //    A = I - KH 只在列 c..c+ncols 偏离单位阵，故为 O(N²·ncols)。
        for i in 0..N {
            let ki = &k[i * ncols..i * ncols + ncols];
            for j in 0..N {
                let mut acc = ap[i * N + j];
                for (l, kl) in ki.iter().enumerate() {
                    acc -= kl * ap[(c + l) * N + j];
                }
                let kj = &k[j * ncols..j * ncols + ncols];
                for (l, kl) in ki.iter().enumerate() {
                    acc += r * kl * kj[l];
                }
                self.p[i * N + j] = acc;
            }
        }
        // 4) 对称化（消除尾差）+ 对角线处理
        for i in 0..N {
            for j in (i + 1)..N {
                let avg = 0.5 * (self.p[i * N + j] + self.p[j * N + i]);
                self.p[i * N + j] = avg;
                self.p[j * N + i] = avg;
            }
            let d = self.p[i * N + i];
            if nan_reset && !d.is_finite() {
                self.p[i * N + i] = 1e-6; // 非有限(NaN/Inf) 重置，阻断协方差 NaN 传播
            } else if d < 1e-6 {
                self.p[i * N + i] = 1e-6;
            } else if clamp_max && d > P_MAX {
                self.p[i * N + i] = P_MAX;
            }
        }
    }

    /// 位置观测更新步（Joseph 形式）：S = HPH^T+R、K = P H^T S^-1、x += K y、
    /// P = (I-KH)P(I-KH)^T + K R K^T。`r` 为观测噪声（m²），供 GPS / VIO / RTK 以
    /// 各自精度融合同一位置状态。栈数组作用域限于本方法，返回后栈槽被回收。
    fn update_pos_r(&mut self, z: PosSample, r: f32) {
        // S = H P H^T + R (3x3)
        let mut s = [0.0f32; 9];
        for i in 0..3 {
            for j in 0..3 {
                s[i * 3 + j] = self.p[i * N + j] + if i == j { r } else { 0.0 };
            }
        }
        // K = P H^T S^-1 (9x3) ; P H^T 即 P 前三列
        let mut pht = [0.0f32; 36];
        for i in 0..N {
            for j in 0..3 {
                pht[i * 3 + j] = self.p[i * N + j];
            }
        }
        // S 3x3 求逆（伴随矩阵）
        let s00 = s[0]; let s01 = s[1]; let s02 = s[2];
        let s10 = s[3]; let s11 = s[4]; let s12 = s[5];
        let s20 = s[6]; let s21 = s[7]; let s22 = s[8];
        let det = s00 * (s11 * s22 - s12 * s21)
            - s01 * (s10 * s22 - s12 * s20)
            + s02 * (s10 * s21 - s11 * s20);
        let inv = if det.abs() > 1e-9 { 1.0 / det } else { 0.0 };
        let mut sinv = [0.0f32; 9];
        sinv[0] = (s11 * s22 - s12 * s21) * inv;
        sinv[1] = (s02 * s21 - s01 * s22) * inv;
        sinv[2] = (s01 * s12 - s02 * s11) * inv;
        sinv[3] = (s12 * s20 - s10 * s22) * inv;
        sinv[4] = (s00 * s22 - s02 * s20) * inv;
        sinv[5] = (s02 * s10 - s00 * s12) * inv;
        sinv[6] = (s10 * s21 - s11 * s20) * inv;
        sinv[7] = (s01 * s20 - s00 * s21) * inv;
        sinv[8] = (s00 * s11 - s01 * s10) * inv;

        let mut k = [0.0f32; 36];
        mat_mul(&pht, &sinv, &mut k, N, 3, 3);

        // 数值鲁棒：卡尔曼增益限幅。紧噪声位置观测（RTK/VIO）下速度行增益可爆炸，
        // 限幅后 Joseph 协方差更新保持 PSD，阻断 p03 发散（见 K_MAX 注释）。
        for e in k.iter_mut() {
            *e = e.clamp(-K_MAX, K_MAX);
        }

        // 垂向加计零偏状态 (9) 不可由位置观测驱动（不可观 → 发散风险）：
        // 清零其卡尔曼增益行，使位置更新不修正 accel_bias_z（仅速度观测可估计，见 update_vel）。
        for j in 0..3 {
            k[9 * 3 + j] = 0.0;
        }
        // 垂向速度状态 (5) 的增益行也清零：GPS 位置观测经非对角协方差 K[5,*] 会推爆垂向速度
        // （hil 闭环回归：恒定比力+GPS 下 vel_z 爆炸到 1e5）。垂向速度仅由加速度积分决定，
        // 由 Doppler 速度观测（update_vel）约束，位置观测不直接修正它。
        for j in 0..3 {
            k[5 * 3 + j] = 0.0;
        }

        // 创新 y = z - pos
        let y = [z.pos[0].0 - self.x[0], z.pos[1].0 - self.x[1], z.pos[2].0 - self.x[2]];

        // x += K y
        for i in 0..N {
            let mut corr = 0.0;
            for j in 0..3 {
                corr += k[i * 3 + j] * y[j];
            }
            self.x[i] += corr;
        }

        // P 更新用 Joseph 形式（H 观测状态 0..3）
        self.joseph_update_cov(&k, 3, 0, r, true, true);
    }

    /// 位置观测更新（GPS，默认噪声 `r_pos`）。等价于 `update_pos_r(z, self.r_pos)`。
    pub fn update_pos(&mut self, z: PosSample) {
        let r_pos_eff = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_R_POS)) };
            if ov != 0.0 { ov } else { self.r_pos }
        };
        self.update_pos_r(z, r_pos_eff);
    }

    /// 速度观测更新步（Doppler GPS）：H = [0 0 0 I3 0] 作用于状态 [pos, vel, bias]，
    /// 直接观测速度分量 3..6。`r` 为观测噪声（m/s）²，供 GPS Doppler / VIO 以各自
    /// 精度融合。Joseph 形式，栈数组作用域限于本方法。
    pub fn update_vel_r(&mut self, vel: [f32; 3], r: f32) {
        // ---- Doppler 差分 → 世界系平移加速度（重力锚定的平移补偿，阶段2 P4 方案 C）----
        //
        // 仅当 `G_AW_GPS > 0` 时启用。**静态初值 = 0.0（关）**，理由见该静态量的文档：
        // 开启后在无平移场景（尤其**自由落体**）会把幅值门的失重保护骗开 → 姿态发散。
        // 平移量大的任务可显式打开（旋钮保留）。
        // 新鲜判据：观测相对上次发生变化（GPS 20Hz 而控制 250Hz → 大量重复样本；
        // 对重复样本差分只会得到 0，对变化样本差分才是真实加速度）。
        let enabled =
            unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_AW_GPS)) } > 0.0;
        if enabled {
            if self.vel_obs_dt < 0.0 {
                // 首次观测：只建立基准。
                self.prev_vel_obs = vel;
                self.vel_obs_dt = 0.0;
            } else {
                let fresh = (vel[0] - self.prev_vel_obs[0]).abs() > 1e-6
                    || (vel[1] - self.prev_vel_obs[1]).abs() > 1e-6
                    || (vel[2] - self.prev_vel_obs[2]).abs() > 1e-6;
                if fresh && self.vel_obs_dt > 1e-4 {
                    let inv = 1.0 / self.vel_obs_dt;
                    // 一阶低通 tau=0.25s：Doppler 差分噪声大，且 GPS 有延迟。
                    let tau = {
                        let ov = unsafe {
                            core::ptr::read_volatile(core::ptr::addr_of!(G_AW_TAU))
                        };
                        if ov > 0.0 { ov } else { 0.25f32 }
                    };
                    let alpha = (self.vel_obs_dt / (tau + self.vel_obs_dt)).clamp(0.0, 1.0);
                    for k in 0..3 {
                        let a = (vel[k] - self.prev_vel_obs[k]) * inv;
                        self.world_accel[k] += alpha * (a - self.world_accel[k]);
                    }
                    // 标记为**非可信源**（噪声大）→ 锚定侧会施加平移门控。
                    // 注：**不把门乘进 LPF 状态**（否则门控自锁：门压低 → 模长变小 → 门更低）。
                    self.world_accel_trusted = false;
                    self.prev_vel_obs = vel;
                    self.vel_obs_dt = 0.0;
                }
            }
        }
        // S = H P H^T + R (3x3)
        let mut s = [0.0f32; 9];
        for i in 0..3 {
            for j in 0..3 {
                s[i * 3 + j] = self.p[(3 + i) * N + (3 + j)] + if i == j { r } else { 0.0 };
            }
        }
        // K = P H^T S^-1 (9x3)；P H^T 即 P 第 3..6 列
        let mut pht = [0.0f32; 36];
        for i in 0..N {
            for j in 0..3 {
                pht[i * 3 + j] = self.p[i * N + (3 + j)];
            }
        }
        let s00 = s[0]; let s01 = s[1]; let s02 = s[2];
        let s10 = s[3]; let s11 = s[4]; let s12 = s[5];
        let s20 = s[6]; let s21 = s[7]; let s22 = s[8];
        let det = s00 * (s11 * s22 - s12 * s21)
            - s01 * (s10 * s22 - s12 * s20)
            + s02 * (s10 * s21 - s11 * s20);
        let inv = if det.abs() > 1e-9 { 1.0 / det } else { 0.0 };
        let mut sinv = [0.0f32; 9];
        sinv[0] = (s11 * s22 - s12 * s21) * inv;
        sinv[1] = (s02 * s21 - s01 * s22) * inv;
        sinv[2] = (s01 * s12 - s02 * s11) * inv;
        sinv[3] = (s12 * s20 - s10 * s22) * inv;
        sinv[4] = (s00 * s22 - s02 * s20) * inv;
        sinv[5] = (s02 * s10 - s00 * s12) * inv;
        sinv[6] = (s10 * s21 - s11 * s20) * inv;
        sinv[7] = (s01 * s20 - s00 * s21) * inv;
        sinv[8] = (s00 * s11 - s01 * s10) * inv;

        let mut k = [0.0f32; 36];
        mat_mul(&pht, &sinv, &mut k, N, 3, 3);

        // 数值鲁棒：卡尔曼增益限幅（同 update_pos_r）。紧速度观测（VIO 光流 r=0.04）下
        // 位置/交叉状态行增益可过大，限幅保持 Joseph 更新数值稳定。
        for e in k.iter_mut() {
            *e = e.clamp(-K_MAX, K_MAX);
        }

        // 垂向加计零偏 x[9] 只应被【垂向】速度观测驱动：水平速度创新（y0/y1）经交叉
        // 协方差 K[9][0..1] 会注入水平残差噪声/偏置进零偏（物理上水平速度误差由水平
        // 加计零偏引起，而本滤波器不估计水平零偏），紧速度观测下放大成 ab_z 发散
        // （仿真：GPS 中断 + VIO 时 ab_z → 4.3e6 → NaN）。清零其水平增益行，仅保留
        // 垂向增益 K[9][2]。
        k[9 * 3 + 0] = 0.0;
        k[9 * 3 + 1] = 0.0;

        // 注：垂向加计零偏 x[9] 仅由速度观测驱动，单分量可观且数值稳定，
        // 无需额外增益阻尼（3 轴机体系版本曾有 Y 轴正反馈发散，已收敛为单分量）。
        // 阶段 11-A：启用垂向零偏估计。之前 0.0 把零偏恒锁在初值 0，
        // 导致恒定加计零偏（典型 0.05 m/s²）直接积分进物理速度、悬停轨迹无界发散。
        // 单分量（世界 D 轴）形式数值稳定；增益取 0.03（小增益）使零偏仅跟踪速度误差的
        // 直流分量、不吸收控制瞬态（瞬态速度误差若被吸进零偏会使其过冲到上限 0.3、反而
        // 造成悬停上漂发散）。配合 q_accel=5e-4 让零偏在 40s 尺度内缓慢漂移到真值 0.05。
        const AB_VEL_GAIN: f32 = 0.3; // 速度观测驱动垂向零偏 x[9] 的增益（0=关闭，零偏保持初值）
        let y = [vel[0] - self.x[3], vel[1] - self.x[4], vel[2] - self.x[5]];

        for i in 0..N {
            let mut corr = 0.0;
            for j in 0..3 {
                corr += k[i * 3 + j] * y[j];
            }
            if i == 9 {
                corr *= AB_VEL_GAIN;
            }
            self.x[i] += corr;
        }
        // 加计零偏物理上界夹取（消费级 IMU 零偏通常 < 0.3 m/s²）：该状态仅由速度观测
        // 驱动，正常收敛不受影响；夹取作发散兜底。必须在此执行——`update_vio`/`update_rtk`
        // 在 `step`（其尾部已有同款夹取）之后调用，若不加此处则 VIO/RTK 驱动的零偏
        // 在整步内无界增长，成为 NaN 源头。
        if self.x[9] > 0.3 {
            self.x[9] = 0.3;
        } else if self.x[9] < -0.3 {
            self.x[9] = -0.3;
        }

        // Joseph 形式（H 观测状态 3..6）；对角策略沿用本方法原有的"只做下限"
        self.joseph_update_cov(&k, 3, 3, r, false, false);
    }

    /// 速度观测更新（GPS Doppler，默认噪声 `r_vel`）。
    ///
    /// **只观测水平分量**：NMEA RMC 仅提供水平地速（无垂直速度），GPS Doppler
    /// 的垂向分量无物理意义（固件 GpsUblox 对 RMC 的 D 分量置 0）。若把 0 当
    /// 垂向速度观测注入，会与 baro 高度 + IMU 垂向融合冲突 → 高度环振荡发散
    /// （实测：RMC 激活后 pos[2] 在 ±11m 剧烈振荡）。垂向速度/零偏由 baro 与
    /// IMU 路径估计，水平速度由 GPS Doppler 约束（悬停水平漂移根治）。
    pub fn update_vel(&mut self, vel: [f32; 3]) {
        let r_vel_eff = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_R_VEL)) };
            if ov != 0.0 { ov } else { self.r_vel }
        };
        self.update_vel_r([vel[0], vel[1], self.x[5]], r_vel_eff);
    }

    /// 高度观测更新步（气压计）：气压计测得**向上**高度 `alt`，而状态 D 轴向下为正，
    /// 故观测 `z = -alt`，H = [0 0 1 0 0 0 0 0 0]（作用于 D 位置索引 2）。
    /// Joseph 形式，栈数组作用域限于本方法。
    pub fn update_alt(&mut self, alt: f32) {
        // 防御：观测值本身非有限（传感器/HIL 注入异常）直接跳过，防止 y 为 NaN 污染状态。
        if !alt.is_finite() {
            return;
        }
        // S = H P H^T + R (标量)，H 仅在索引 2（D 位置）非零
        let s = self.p[2 * N + 2] + self.r_alt;
        if !s.is_finite() || s.abs() < 1e-9 {
            return;
        }
        // K = P H^T / S (9x1)，仅 P 第 2 列非零
        let mut k = [0.0f32; N];
        for i in 0..N {
            k[i] = self.p[i * N + 2] / s;
        }
        // 创新 y = z - x，z = -alt（D 向下正）；非有限（状态已被污染）则跳过本观测，
        // 阻断 x += k*y 把 NaN 传播进状态。
        let y = -alt - self.x[2];
        if !y.is_finite() {
            return;
        }
        // 垂向速度状态 (5) 的增益行。
        //
        // 历史（一直写到 2026-09-21）：清零——气压同为位置观测，经非对角协方差 K[5]
        // 会推爆垂速（hil 闭环回归：恒定比力+气压下 vel_z 单拍 +24.6 → 爆炸）。
        // **但清零的代价**：垂速退化为纯 IMU 积分 → 零偏留下稳态速度误差 →
        // 高度环跟着走 → 真值持续漂移（实测 realistic 悬停漂 2.64m / max 15.5m）。
        //
        // 改为**有界增益**（内置默认 0.1）：实测 0.10 与 2.00 结果逐位相同 ⇒
        // 实际 |k[5]| 从未超过 0.1，上限内就足以把垂速拉回来。
        // `G_KVZ_FROM_ALT`：0 = 内置默认；<0 = 强制 0（历史对照）；>0 = 覆盖。
        let kvz_ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_KVZ_FROM_ALT)) };
        let kvz = if kvz_ov > 0.0 {
            kvz_ov
        } else if kvz_ov < 0.0 {
            0.0
        } else {
            0.1f32
        };
        k[5] = k[5].clamp(-kvz, kvz);
        // 卡尔曼增益限幅（与 update_pos_r 一致，见 K_MAX 注释）：紧噪声观测下
        // 非对角增益可爆炸，限幅后 Joseph 协方差更新保持 PSD，阻断交叉项发散。
        for e in k.iter_mut() {
            *e = e.clamp(-K_MAX, K_MAX);
        }
        // 垂向加计零偏状态 (9) 不可由气压高度观测驱动（不可观 → 发散风险）：
        // 清零其卡尔曼增益元素，使气压更新不修正 accel_bias_z（仅速度观测可估计）。
        k[9] = 0.0;
        for i in 0..N {
            self.x[i] += k[i] * y;
        }
        // Joseph 形式（标量气压高度观测，H 只看状态 2）
        let r = self.r_alt;
        self.joseph_update_cov(&k, 1, 2, r, true, true);
    }

    /// 空速观测更新步（标量）：空速计测得水平气流速度幅值 `v_as = |v_h|`，
    /// 其中 `v_h = sqrt(vx² + vy²)`（不含风）。观测模型 `h(x) = sqrt(vx² + vy²)`，
    /// Jacobian `H = [0 0 0 vx/vh vy/vh 0 0 0]`。
    /// 经典 EKF 标量更新（P 已是 PSD，无需 Joseph 形式），栈数组作用域限于本方法。
    fn update_airspeed(&mut self, v_as: f32) {
        let vx = self.x[3];
        let vy = self.x[4];
        let vh2 = vx * vx + vy * vy;
        // 水平速度近零时 Jacobian 退化：直接用测量初始化估计，跳过增益更新。
        if vh2 < 1e-4 {
            self.airspeed_est = v_as;
            return;
        }
        let vh = sqrt(vh2);
        let hx = vx / vh;
        let hy = vy / vh;
        // S = H P H^T + R （标量）
        // H P H^T = (hx,hy,0) P (hx,hy,0)^T = Σ_{a,b∈{3,4}} H_a P_ab H_b
        let mut hph = 0.0f32;
        let ha = [hx, hy];
        for a in 0..2 {
            for b in 0..2 {
                hph += ha[a] * self.p[(3 + a) * N + (3 + b)] * ha[b];
            }
        }
        let s = hph + self.r_airspeed;
        if s.abs() < 1e-9 {
            return;
        }
        // K = P H^T / S  (9x1)：仅 vel 分量非零
        let mut k = [0.0f32; N];
        k[3] = (self.p[3 * N + 3] * hx + self.p[3 * N + 4] * hy) / s;
        k[4] = (self.p[4 * N + 3] * hx + self.p[4 * N + 4] * hy) / s;
        // 创新 y = z - h(x)
        let y = v_as - vh;
        // x += K y
        for i in 0..N {
            self.x[i] += k[i] * y;
        }
        // P = (I - K H) P，对称化 + 下限夹取
        // 正确秩1更新：(I-KH)P 的 (i,j) 元 = P[i][j] - k[i]*(hx*P[3][j] + hy*P[4][j])
        // （K 仅行 3,4 非零，H 仅列 3,4 非零）
        let mut ap = [0.0f32; N * N];
        for i in 0..N {
            for j in 0..N {
                let khp = k[i] * (hx * self.p[3 * N + j] + hy * self.p[4 * N + j]);
                ap[i * N + j] = self.p[i * N + j] - khp;
            }
        }
        for i in 0..N {
            for j in 0..N {
                self.p[i * N + j] = ap[i * N + j];
            }
        }
        for i in 0..N {
            for j in (i + 1)..N {
                let avg = 0.5 * (self.p[i * N + j] + self.p[j * N + i]);
                self.p[i * N + j] = avg;
                self.p[j * N + i] = avg;
            }
            let d = self.p[i * N + i];
            if !d.is_finite() || d < 1e-6 {
                self.p[i * N + i] = 1e-6; // 非有限(NaN/Inf) 重置，阻断协方差 NaN 传播
            } else if d > P_MAX {
                self.p[i * N + i] = P_MAX;
            }
        }
        // 估计空速 = 水平速度幅值（融合后）
        self.airspeed_est = sqrt(self.x[3] * self.x[3] + self.x[4] * self.x[4]);
    }

    /// 设置磁偏角（度，东偏为正）：磁北相对地理北（世界 +X）的偏角。
    /// 0 = 磁北即地理北（默认）。EKF 用该角度旋转参考地磁方向，使磁力计
    /// 航向锚定对准【地理北】而非磁北（磁航向 decl 修正，对标真机导航）。
    pub fn set_mag_declination(&mut self, decl_deg: f32) {
        let (s, c) = crate::math::sin_cos(decl_deg.to_radians());
        self.mag_ref = [c, s];
    }

    /// 设置**离线标定**得到的机体硬铁偏置（与磁力计读数同单位）。
    ///
    /// `update_mag` 使用前会从读数中扣除。默认 `[0,0,0]`（不补偿）。
    ///
    /// 工程上硬铁用**离线标定**（多姿态采集取各轴 min/max 中心）而非在线估计：
    /// 硬铁在静止悬停下不可观（无旋转 → 偏置与场无法区分）。与 `set_mag_declination`
    /// 同形态：两者都是“把已标定的磁环境参数告诉 EKF”。参见
    /// `docs/stage1-attitude-findings.md` F7。
    pub fn set_mag_hard_iron(&mut self, hard_iron: [f32; 3]) {
        self.mag_hard_iron = hard_iron;
    }

    /// 注入**世界系平移加速度估计**（NED，m/s²），用于重力锚定的平移补偿。
    ///
    /// 默认 `[0,0,0]`（不补偿，与历史行为一致）。详见字段文档与
    /// `docs/stage2-attitude-ctrl-findings.md` P4。
    pub fn set_world_accel(&mut self, a_world: [f32; 3]) {
        self.world_accel = if a_world.iter().all(|v| v.is_finite()) {
            a_world
        } else {
            [0.0; 3]
        };
        // 调用方直接注入 ⇒ 视为**可信源**（不经平移门控）。
        self.world_accel_trusted = true;
    }

    /// 磁力计航向锚定：把世界系水平磁场方向拉回磁北参考（+X），锚定四元数 yaw。
    ///
    /// 与 `att_alpha` 重力修正对称：重力锚 roll/pitch（世界系重力 → +Z），
    /// 磁力计锚 yaw（世界系水平磁场 → 磁北）。纯陀螺积分时 yaw 无观测会缓慢漂移
    /// （陀螺零偏残余 → yaw 积分漂移），磁力计提供绝对航向参考。
    ///
    /// 原理：机体系磁场 `m` 经**估计**姿态旋转到世界系 `m_world`；若估计 yaw 偏差
    /// δ，`m_world` 水平分量相对磁北偏转 δ。`yaw_err = atan2(-my, mx)` 即该偏差，
    /// 绕世界 Z 轴按 `mag_alpha` 强度修正（符号：yaw 偏大 → 磁场偏东 → yaw_err<0
    /// → 绕 -Z 修正，yaw 减小）。roll/pitch 分量不受影响（绕世界 Z 纯 yaw 修正）。
    fn update_mag(&mut self, mag: Option<[f32; 3]>) {
        // 标定旋钮（易失读：由外部写入，普通读会被常量折叠掉）。
        let alpha = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_MAG_ALPHA)) };
            if ov >= 0.0 { ov } else { self.mag_alpha }
        };
        if alpha <= 0.0 {
            return;
        }
        let m = match mag {
            Some(m) => m,
            None => return,
        };
        if !m.iter().all(|v| v.is_finite()) {
            return;
        }
        // [已回退 2026-09-20] 磁锚陀螺门控：曾与重力锚定对称加了一道门，
        // 动机是追 M 场**闭环**测试 `x_hover_noise` 的劣化。但：
        //   ① 阶段 1 的任何开环测试都不需要它；
        //   ② 加入后 M 场 `x_env_faults::mag_disturb_keeps_attitude` 出现**固件崩溃**
        //      （`UC_ERR_INSN_INVALID` @ step 307）；
        //   ③ 属越出阶段 1 范围的改动。
        // 故回退，连同 `x_hover_noise` 一并移交阶段 2/4。
        // 详见 `docs/stage1-attitude-findings.md` F7。
        const W_GYRO: f32 = 1.0;
        // 扣除**离线标定**的硬铁偏置（默认零 = 不补偿）。
        let m = [
            m[0] - self.mag_hard_iron[0],
            m[1] - self.mag_hard_iron[1],
            m[2] - self.mag_hard_iron[2],
        ];
        let m_world = rotate_vec_by_quat(self.att, m);
        let mh = sqrt(m_world[0] * m_world[0] + m_world[1] * m_world[1]);
        // 水平磁场过弱（磁力计几乎指向天顶/地磁水平分量≈0）→ 无法提供航向参考。
        if mh < 1e-3 {
            return;
        }
        // 世界系水平磁场相对【参考地磁方向（磁北）】的偏角 = yaw 估计误差。
        // mag_ref=(cos decl, sin decl)：把 m_world 水平分量旋转 -decl 投影到参考系，
        // 再取 atan2 —— 机头指向磁北（地理北+decl）时误差为 0，实现磁航向→地理航向。
        let (rx, ry) = (self.mag_ref[0], self.mag_ref[1]);
        let cx = m_world[0] * rx + m_world[1] * ry;
        let cy = -m_world[0] * ry + m_world[1] * rx;
        let yaw_err = crate::math::atan2(-cy, cx);
        // 归一化到 [-π, π]：atan2 已保证，无需 wrap。
        //
        // 【已否决 B 方案：场模长一致性门控】2026-09-21
        // 曾尝试：`|m_world|` 偏离慢 EMA 标称超容差 → 按比例关闭锚定。
        // 实测否决（H 场扫容差，硬铁 `[0.3,-0.2,0.4]` + 10° 摆动）：
        //   tol ≥ 0.02 → 门**从不触发**（10° 倾角下 |m| 仅变 ±1.1%）；
        //   tol < 0.02 → 在噪声上乱触发，yaw RMSE 反而从 21.9° 劣化到 24.8°。
        // 根因：硬铁是**方向**误差，不是模长误差 —— `|m_world| = |R^T·B + h|`
        // 对小倾角只是二阶变化。⇒ 模长门控在原理上盖不住它。
        // 正确修法是**离线标定扣除**（`mag_hard_iron` / `set_mag_hard_iron`）。
        // 详见 `docs/stage1-attitude-findings.md` F7。
        // ---- 路线 2.2e：全姿态修正（含 roll/pitch），默认关 ----
        //
        // 与重力锚定对称的叉积形式，但参考是**磁场**而非比力 ⇒ 与加速度无耦合。
        // 修正轴 = 估计世界场方向 × 参考世界场方向（世界系），左乘施加。
        let a3 = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_MAG3D_ALPHA)) };
        if a3 > 0.0 && self.mag_ref3d != [0.0, 0.0, 0.0] {
            let r = self.mag_ref3d;
            let rn = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
            let mn = sqrt(m_world[0] * m_world[0] + m_world[1] * m_world[1] + m_world[2] * m_world[2]);
            if rn > 1e-6 && mn > 1e-6 {
                let rh = [r[0] / rn, r[1] / rn, r[2] / rn];
                let mh3 = [m_world[0] / mn, m_world[1] / mn, m_world[2] / mn];
                // 叉积（旋转轴）：把 mh3 转向 rh
                let ax = mh3[1] * rh[2] - mh3[2] * rh[1];
                let ay = mh3[2] * rh[0] - mh3[0] * rh[2];
                let az = mh3[0] * rh[1] - mh3[1] * rh[0];
                let s = sqrt(ax * ax + ay * ay + az * az);
                let c = (mh3[0] * rh[0] + mh3[1] * rh[1] + mh3[2] * rh[2]).clamp(-1.0, 1.0);
                let ang = crate::math::atan2(s, c) * a3 * 0.5 * self.last_dt * 250.0;
                if s > 1e-9 && ang.abs() > 0.0 {
                    let dq3 = Quaternion::from_axis_angle([ax / s, ay / s, az / s], Radian(ang));
                    self.att = (dq3 * self.att).normalize();
                }
            }
        }

        let k = alpha * 0.5 * W_GYRO;
        let corr = yaw_err * k;
        let dq = Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(corr));
        self.att = (dq * self.att).normalize();

        // ---- 陀螺零偏（Z 分量）在线估计：**以磁力计为参考** ----
        //
        // ## 为何参考源必须是磁力计，而不是比力（阶段 1 的方案修正）
        // 原方案打算"零偏改由速度/位置创新观测"。分析后**否决**：
        //   速度观测经标准卡尔曼增益更新 `x[6..8]`，需要协方差里存在**姿态状态**
        //   （零偏经姿态→重力投影→速度的耦合）。而我们的姿态**不在协方差里**
        //   （是独立的固定-α 锚定）⇒ 交叉项恒≈0 ⇒ 这正是 `x[6..8]`"从未被任何
        //   观测更新"的**根本原因**；速度对该零偏的耦合还是**二阶**的（经姿态二次
        //   积分），即使硬接也很弱。
        //
        // 而**磁力计是世界系方向参考**（不像比力会被水平加速度污染）⇒ 抗加速、
        // 抗风的零偏参考 ✓。`update_mag` 施加的 yaw 修正正是"Z 陀螺零偏造成的
        // 航向漂移"被拉回的体现：稳态下**修正速率 = −零偏_z**（同重力锚定的推导）。
        //
        // 只估 Z 分量：本处修正只绕世界 Z（= 航向），故只能观测 Z 零偏。roll/pitch
        // 的零偏需 Stage 2（风状态）或 Stage 3（误差状态 + NIS）方能可靠观测。
        let gbk = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_GYRO_BIAS_K)) };
        if gbk > 0.0 && self.last_dt > 1e-6 {
            let inv_dt = 1.0 / self.last_dt;
            let obs = -corr * inv_dt;
            self.x[8] += gbk * (obs - self.x[8]) * self.last_dt;
            self.x[8] = self.x[8].clamp(-GYRO_BIAS_MAX, GYRO_BIAS_MAX);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(test)]
    extern crate std;
    use crate::config::VehicleConfig;
    use crate::units::*;
    use crate::vehicle::{Airspeed, AirspeedSample, ImuSample, MeterPerSecond, RadianPerSecond};



    #[test]
    fn mag_declination_aligns_yaw_to_true_north() {
        // 磁北偏东 15°（decl=15°）。真实机头地理航向 15°（=磁北）时机体系
        // 水平磁场沿 +X（磁北方向旋转到机体系）。估计 yaw=0（偏差 -15°）。
        // EKF 应把 yaw 从 0 收敛到 +15°（地理航向），而非 0°（磁航向）。
        let mut ekf = EkfEstimator::default_quad();
        ekf.set_mag_declination(15.0);
        // 真实机头地理 yaw=15°：机体系磁场 = R^T(15°)·[0.2,0,0.4]（世界系磁场在 +15°）
        // 水平分量旋转到机体系后沿 +X → 读数即 [0.2, 0, 0.4]。
        let m_body = [0.2f32, 0.0, 0.4];
        for _ in 0..500 {
            ekf.update_mag(Some(m_body));
        }
        let yaw_deg = ekf.att.yaw().to_degrees();
        assert!(
            (yaw_deg - 15.0).abs() < 3.0,
            "decl=15° 时 yaw 应收敛到 15°（地理北），实际 {yaw_deg}°"
        );

        // 对照：decl=0（默认）时同一读数（磁北=地理北场景）收敛到 0°。
        let mut ekf0 = EkfEstimator::default_quad();
        for _ in 0..500 {
            ekf0.update_mag(Some(m_body));
        }
        let yaw0 = ekf0.att.yaw().to_degrees();
        assert!(yaw0.abs() < 3.0, "decl=0 时 yaw 应收敛到 0°，实际 {yaw0}°");
    }

    #[test]
    fn mag_heading_correction_converges_yaw() {
        // 初始估计 yaw 偏 30°（纯偏航误差，roll/pitch=0）。真实 yaw=0 时机体系
        // 磁场 = 北向参考 [0.2, 0, 0.4]（水平 +X 磁北）。update_mag 应把 yaw
        // 从 30° 收敛回 0°，且不动 roll/pitch。
        let mut ekf = EkfEstimator::default_quad();
        ekf.att = Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(30f32.to_radians()));
        let m_body = [0.2f32, 0.0, 0.4];
        for _ in 0..400 {
            ekf.update_mag(Some(m_body));
        }
        let yaw_deg = ekf.att.yaw().to_degrees();
        assert!(yaw_deg.abs() < 3.0, "yaw 应收敛到 0°，实际 {yaw_deg}°");
        assert!(ekf.att.roll().abs() < 1e-3, "磁力计不应扰动 roll");
        assert!(ekf.att.pitch().abs() < 1e-3, "磁力计不应扰动 pitch");

        // mag=None / 全零 / 非有限 → 不应改变姿态。
        let att_before = ekf.att;
        ekf.update_mag(None);
        assert_eq!(ekf.att, att_before);
        ekf.update_mag(Some([0.0, 0.0, 0.0]));
        assert_eq!(ekf.att, att_before);
        ekf.update_mag(Some([f32::NAN, 0.0, 0.4]));
        assert_eq!(ekf.att, att_before);

        // 反向偏置（yaw=-30°）同样收敛回 0°。
        let mut ekf2 = EkfEstimator::default_quad();
        ekf2.att = Quaternion::from_axis_angle([0.0, 0.0, 1.0], Radian(-30f32.to_radians()));
        for _ in 0..400 {
            ekf2.update_mag(Some(m_body));
        }
        let yaw2 = ekf2.att.yaw().to_degrees();
        assert!(yaw2.abs() < 3.0, "反向 yaw 应收敛到 0°，实际 {yaw2}°");
    }

    #[test]
    fn airspeed_fusion_constrains_horizontal_speed() {
        // 构型：机体匀速平飞（水平速度 5 m/s），空速计测得 5 m/s。
        // 验证 EKF 融合后估计水平速度幅值收敛到 ≈ 测量空速。
        let _cfg = VehicleConfig::default_quad();
        let mut ekf = EkfEstimator::default_quad();
        // 初始化真值水平速度
        ekf.x[3] = 5.0;
        ekf.x[4] = 0.0;

        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0); 3],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let aspd = AirspeedSample { speed: Airspeed(5.0), timestamp_s: 0.0 };

        let mut last = VehicleState::zero();
        for _ in 0..200 {
            last = ekf.step(Second(0.01), imu, None, Some(aspd));
        }
        let vh = (last.vel[0].0 * last.vel[0].0 + last.vel[1].0 * last.vel[1].0).sqrt();
        assert!(
            (vh - 5.0).abs() < 0.3,
            "空速计融合后水平速度幅值应≈5 m/s，got {:.3}",
            vh
        );
        // 估计空速应被填充
        assert!(last.airspeed.0 > 0.0, "估计空速应 > 0，got {}", last.airspeed.0);
    }

    #[test]
    fn airspeed_fusion_rejects_none_gracefully() {
        // 无空速计（None）时，EKF 不应崩溃，airspeed 估计保持为水平速度幅值（可能为 0）。
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0); 3],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let mut ekf = EkfEstimator::default_quad();
        let st = ekf.step(Second(0.01), imu, None, None);
        // 静止 + 无观测：速度应≈0，不得 NaN
        assert!(st.vel[0].0.is_finite() && st.vel[1].0.is_finite());
    }

    #[test]
    fn hil_ekf_vertical_stays_stable_with_mock_gps() {
        // 回归：EKF + 完整 MockImu（含 gyro 振动/120Hz accel 振动）+ GPS(d=-5)。
        // 曾因 GPS 位置观测经非对角协方差 K[5,*] 推爆垂向速度（vel_z 到 1e5，位置漂走）
        // → hil 闭环 NaN。修复：update_pos 清零垂向速度增益行（K[5,*]=0），垂向速度仅由
        // 加速度积分 + Doppler 速度观测约束。验证位置物理稳定在 -5 附近（<100m）不漂走。
        let mut ekf = EkfEstimator::default_quad();
        ekf.set_initial_position([0.0, 0.0, -5.0]);
        let gps = PosSample::pos_only([Meter(0.0), Meter(0.0), Meter(-5.0)]);
        let mut imu2 = crate::hal::sensor::MockImu::new();
        let mut max_abs = 0.0f32;
        let mut last = VehicleState::zero();
        let mut bad = false;
        for _ in 0..300 {
            let s = crate::hal::sensor::ImuSensor::read(&mut imu2);
            last = ekf.step(Second(0.01), s, Some(gps), None);
            max_abs = max_abs.max(last.pos[2].0.abs());
            if !last.pos[2].0.is_finite() {
                bad = true;
                break;
            }
        }
        assert!(
            !bad && max_abs < 100.0 && last.pos[2].0.is_finite(),
            "EKF 恒定比力+GPS 悬停应稳定: max_abs={} pos_z={} vel_z={} bad={}",
            max_abs, last.pos[2].0, last.vel[2].0, bad
        );
    }

    #[test]
    fn vio_fusion_tracks_truth_during_gps_outage() {
        // P3-B1：GPS 失锁期间 VIO 桥接。机体匀速 2 m/s 前飞（北向），IMU 零加速度
        // （比力变化由水平速度保持承担）、无 GPS、无空速；仅 VIO 提供位置+速度观测。
        // 无 VIO 时 EKF 因无速度/位置观测会把速度锁在 0、位置停在起点（纯积分无输入）；
        // 有 VIO 时估计位置/速度应跟随真值，验证 VIO 填补 GPS 帧间/失锁的估计空白。
        let mut ekf = EkfEstimator::default_quad();
        ekf.set_initial_position([0.0, 0.0, -5.0]);
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0); 3],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let mut truth_n = 0.0f32;
        let mut last = VehicleState::zero();
        for _ in 0..300 {
            truth_n += 2.0 * 0.01; // 2 m/s 匀速
            let vio = VioSample::with_vel(
                [Meter(truth_n), Meter(0.0), Meter(-5.0)],
                [MeterPerSecond(2.0), MeterPerSecond(0.0), MeterPerSecond(0.0)],
            );
            last = ekf.step(Second(0.01), imu, None, None);
            ekf.update_vio(Some(vio));
        }
        // 3s 后真值北向 6m。
        assert!(
            (last.pos[0].0 - 6.0).abs() < 1.0,
            "VIO 桥接后估计位置应≈6m，got {:.3}",
            last.pos[0].0
        );
        assert!(
            (last.vel[0].0 - 2.0).abs() < 0.3,
            "VIO 速度融合后估计速度应≈2 m/s，got {:.3}",
            last.vel[0].0
        );
        assert!(
            last.pos[2].0.is_finite() && last.vel[0].0.is_finite(),
            "VIO 融合不得产生 NaN"
        );
    }

    #[test]
    fn rtk_fusion_reaches_cm_precision() {
        // P3-B1：RTK 厘米级位置融合。初始位置给定大误差（[10,5,-5]），RTK 测得真值
        // [0,0,-5]（噪声 ~0.05m）。反复融合后估计位置应收敛到厘米量级（<0.15m）。
        // IMU 用悬停比力 [0,0,-9.81]（抵消重力），az_w = -9.81+g-0 = 0，垂向不漂移；
        // 若用 [0,0,0]（自由落体）EKF 会按 g 积分垂向速度，而位置观测不改垂向速度
        // （见 update_pos_r 清零第 5 行增益），垂向将发散，测试失去意义。
        let mut ekf = EkfEstimator::default_quad();
        ekf.set_initial_position([10.0, 5.0, -5.0]);
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(-9.81)],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let rtk = RtkSample::new([Meter(0.0), Meter(0.0), Meter(-5.0)]);
        let mut last = VehicleState::zero();
        for _ in 0..50 {
            last = ekf.step(Second(0.01), imu, None, None);
            ekf.update_rtk(Some(rtk));
        }
        let err = [
            (last.pos[0].0 - 0.0).abs(),
            (last.pos[1].0 - 0.0).abs(),
            (last.pos[2].0 + 5.0).abs(),
        ];
        assert!(
            err[0] < 0.15 && err[1] < 0.15 && err[2] < 0.15,
            "RTK 融合后估计位置应达厘米级（<0.15m），got err={:?}",
            err
        );
    }

    #[test]
    #[ignore]
    fn dbg_rtk_divergence() {
        #[cfg(test)]
        use std::println;
        let mut ekf = EkfEstimator::default_quad();
        ekf.set_initial_position([10.0, 5.0, -5.0]);
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0); 3],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let rtk = RtkSample::new([Meter(0.0), Meter(0.0), Meter(-5.0)]);
        for i in 0..3 {
            let _last = ekf.step(Second(0.01), imu, None, None);
            println!("--- i={i} BEFORE update: x=[{:.6},{:.6},{:.6}] vel=[{:.6},{:.6},{:.6}]", ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]);
            println!("  P rows 0-6 cols0-3:");
            for r in 0..6 {
                println!("    r{r}: [{:.3e},{:.3e},{:.3e}] cross0={:.3e} cross1={:.3e} cross2={:.3e}",
                    ekf.p[r*10+0], ekf.p[r*10+1], ekf.p[r*10+2], ekf.p[r*10+3], ekf.p[r*10+4], ekf.p[r*10+5]);
            }
            ekf.update_rtk(Some(rtk));
            println!("  AFTER update: x=[{:.6},{:.6},{:.6}] vel=[{:.6},{:.6},{:.6}]", ekf.x[0], ekf.x[1], ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]);
            println!("  P rows 0-6 cols0-3:");
            for r in 0..6 {
                println!("    r{r}: [{:.3e},{:.3e},{:.3e}] cross0={:.3e} cross1={:.3e} cross2={:.3e}",
                    ekf.p[r*10+0], ekf.p[r*10+1], ekf.p[r*10+2], ekf.p[r*10+3], ekf.p[r*10+4], ekf.p[r*10+5]);
            }
        }
    }

    #[test]
    fn vio_rtk_none_gracefully() {
        // P3-B1：VIO/RTK 无观测（None / 未装备）时 EKF 不应崩溃，状态保持有限。
        let imu = ImuSample {
            accel: [MeterPerSecondSquared(0.0); 3],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let mut ekf = EkfEstimator::default_quad();
        let st = ekf.step(Second(0.01), imu, None, None);
        ekf.update_vio(None);
        ekf.update_rtk(None);
        assert!(st.vel[0].0.is_finite() && st.pos[0].0.is_finite());
    }

    #[test]
    #[ignore]
    fn dbg_matrix_semantics() {
        #[cfg(test)]
        use std::println;
        // A = [[1,2],[3,4]], B = [[5,6],[7,8]]  (row-major 2x2)
        let a = [1.0f32, 2.0, 3.0, 4.0];
        let b = [5.0f32, 6.0, 7.0, 8.0];
        let mut c = [0.0f32; 4];
        mat_mul(&a, &b, &mut c, 2, 2, 2);
        println!("A*B      = {:?}  (expect [19,22,43,50])", c);
    }

    /// 磁硬铁**离线标定**的有效性：标定后航向偏置应被消除。
    ///
    /// 硬铁是机体固定偏置 → 磁航向产生常数偏置；且**无法被任何门控发现**
    /// （静止时 |ω|≈0、偏置恒定时模长/方向也不变）。工程上靠离线标定扣除。
    /// 本测试：传感器带硬铁 `hi`，把同一值经 `set_mag_hard_iron` 告知 EKF
    /// （模拟标定结果）→ yaw 应≈0；不告知 → 明显偏。
    fn mag_anchor_yaw(hard_iron: [f32; 3], calib: Option<[f32; 3]>, steps: u32) -> f32 {
        let dt = 0.004f32;
        let mut ekf = EkfEstimator::default_quad();
        if let Some(c) = calib {
            ekf.set_mag_hard_iron(c);
        }
        ekf.set_initial_attitude(Quaternion::IDENTITY);
        // 世界系地磁（NED：水平指北 + 垂直向下为负，见 plant.rs 符号约定）
        let field_world = [0.2f32, 0.0, -0.4];
        for _ in 0..steps {
            // 静止水平：比力 = [0,0,-9.81]，陀螺零（隔离出纯航向偏置）
            let imu = ImuSample {
                accel: [
                    MeterPerSecondSquared(0.0),
                    MeterPerSecondSquared(0.0),
                    MeterPerSecondSquared(-9.81),
                ],
                gyro: [RadianPerSecond(0.0); 3],
            };
            // 机体磁场 = R^T(world)（att=identity → 等于 world）+ 硬铁
            let m = [
                field_world[0] + hard_iron[0],
                field_world[1] + hard_iron[1],
                field_world[2] + hard_iron[2],
            ];
            ekf.step(Second(dt), imu, None, None);
            ekf.update_mag(Some(m));
        }
        ekf.state().att.yaw()
    }

    #[test]
    fn mag_hard_iron_calibration_removes_heading_bias() {
        let hi = [0.3f32, -0.2, 0.4];
        let uncal = mag_anchor_yaw(hi, None, 2500).to_degrees();
        let cal = mag_anchor_yaw(hi, Some(hi), 2500).to_degrees();
        std::println!("[mag-calib] 未标定 yaw={uncal:.2}°  标定后 yaw={cal:.2}°");
        assert!(
            uncal.abs() > 10.0,
            "未标定时硬铁应产生明显航向偏置，实际 {uncal:.2}°"
        );
        assert!(
            cal.abs() < 2.0,
            "标定后航向偏置应被消除（<2°），实际 {cal:.2}°"
        );
    }
}
