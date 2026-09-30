//! PID 控制器（工程基线，串级：位置外环 -> 姿态内环）。
//!
//! 结构：
//!   外环：位置误差 -> 期望速度（限幅）        (P)
//!   中环：速度误差 -> 期望世界系加速度 -> 期望姿态（四元数）  (P)
//!   内环：四元数姿态误差 -> 期望机体角速度 -> 混控            (PD，无欧拉角奇点)
//! 最终把总推力 + 三轴机体角速度映射到 4 个电机（X 型混控）。
//!
//! 这是与 PX4/APM 同级的工程基线。后续 LQR/MPC 实现将复用同一 `Controller`
//! 接口，在同一仿真场景下直接 PK。

use crate::units::*;
use crate::vehicle::{ActuatorCmd, Quaternion, VehicleState};
use crate::controller::{Controller, trait_def::Setpoint};


/// [联调诊断] PID 内部量观测（静态，无栈开销；测试直读，定位后移除）。
#[used]
pub static mut DBG_PID: [f32; 12] = [0.0; 12];
/// [联调诊断] clamp 前原始值 / clamp 后推力（独立 static 强制求值顺序）。
#[used]
pub static mut DBG_PRE: f32 = 0.0;
#[used]
pub static mut DBG_THR: f32 = 0.0;

/// [联调诊断] **倾角指令观测**：`[acc_n, acc_e, tilt_n, tilt_e]`（clamp 前后各两个）。
///
/// 用途（H2 专项）：判断倾角指令是否**顶满 `tilt_max`**。
/// `tilt = clamp(acc/g, ±tilt_max)` —— 一旦顶满，倾角指令打满 ⇒ 姿态环被推到极限
/// ⇒ 电机饱和 ⇒ 0.3~0.7Hz 极限环（见 `docs/stage4-outer-loop-findings.md` P14）。
/// 不补这个观测点，就只能看到“电机饱和”这个下游现象，看不到根因那一步。
#[used]
pub static mut DBG_TILT: [f32; 4] = [0.0; 4];

/// [标定] `ki_xy`（**水平位置积分增益**）运行时覆盖。
///
/// ⚠️ 哨兵与 `G_MAG_ALPHA` 同规：**< 0（默认 -1）= 用编译期值**。
/// 理由：`0` 是本旋钮的**有效取值**（= 关闭水平积分，即既有行为），
/// 不能拿 0 当"未设置"，否则表达不了"显式关闭"。
#[used]
pub static mut G_KI_XY: f32 = -1.0;

/// [标定] `tilt_max`（倾角指令上限，rad）运行时覆盖。哨兵同规：**<0 = 用编译期值**。
///
/// 为何可调：H 场能力曲线显示 **5.4 m/s（蒲福 3 级上限）处漂移悬崖**（0.85→5.59m）。
/// 物理解释：该风速下抗风稳态倾角需 ≈15.3°(pitch)+2.5°(roll)，叠加阵风摆动后逼近
/// `tilt_max=20°` ⇒ **位置环饱和、失去纠偏能力**。本旋钮用于验证该解释：
/// 若漂移随 `tilt_max` 单调改善，则"倾角权限"就是那个工程杠杆（代价：需推力余量）。
#[used]
pub static mut G_TILT_MAX: f32 = -1.0;

/// [标定] `vmax_xy`（水平期望速度上限，m/s）运行时覆盖。哨兵同规：**<0 = 用编译期值**。
///
/// 用途：验证"持续跟踪误差 = 纠偏权限不足"猜想 —— 依据 PX4 文档对"pure-P law's
/// steady-state tracking lag"的说明与其 `MC_REF_*` 参考模型解法。仓内可算机理：
/// `e = (vmax_xy − v_ff)/kp_xy`（饱和平衡点）。
#[used]
pub static mut G_VMAX_XY: f32 = -1.0;
/// ★§5.180 整定旋钮：速度环积分上限 `I_V_MAX`（默认 2.0 ✓；`-1` ⇒ 编译期值 ✓）
///   动机（§5.177 ✓）：`i_v` 顶满（±2.0）⇒ 与 `des_vx` 顶满形成正反馈漂移 ⇒ 收紧以减轻 ✓
#[no_mangle]
#[used]
pub static mut G_IV_MAX: f32 = -1.0;
/// ★§5.181【对齐 PX4 `ControlMath::constrainXY` ✓】：`2.0` ⇒ 期望速度按**合成模长**限幅
///   （**保方向** ✓）；`0` ⇒ 原**逐轴**限幅（默认 ✓ 逐位不变）
#[no_mangle]
#[used]
pub static mut G_CONSTRAIN_XY: f32 = 0.0;
/// ★§5.182【对齐 PX4 `_vel_int` **累积时序** ✓】：`2.0` ⇒ 积分在**饱和判定之后**累积
///   （PX4 一手 `:190-199` 位于推力饱和之后 ✓）；`0` ⇒ 原时序（默认 ✓ 逐位不变）
#[no_mangle]
#[used]
pub static mut G_IV_AFTER_SAT: f32 = 0.0;

/// [标定] 水平积分**上限**（m/s）运行时覆盖。哨兵同为 **<0 = 用编译期默认 2.0**。
/// 为何必须可调：该夹子直接决定残余稳态偏移 —— 当所需稳态速度指令超过它时，
/// 偏移被夹死为 `(des_v_need - 上限)/kp_xy`，此时**再加 `ki_xy` 也无用**。
#[used]
pub static mut G_I_XY_MAX: f32 = -1.0;

/// [标定] **水平速度环积分增益** `ki_v_xy` 运行时覆盖。
///
/// ⚠️ 哨兵同规：**< 0（默认 -1）= 用编译期值**（`0` 是有效值 = 关闭）。
///
/// # 为何需要（2026-09-21 定根因，本会话唯一开放项）
/// 速度环原为 **P-only**（`acc = kv_xy·(des_v − v)`），而**平飞巡航时倾角必须非零**
/// （平衡阻力）⇒ `acc ≠ 0` ⇒ P-only 下 **`des_v ≠ v`** ⇒ **固有速度偏置**
/// ⇒ 该偏置积分成**位置斜坡**。
///
/// 实测（H 场）：速度误差峰值 **2.497 m/s**（轨迹速度才 2 m/s）、位置滞后 **7.7~9.0m**；
/// 而**位置环积分 `ki_xy` 完全无效** ✗ —— 因为它是位置层，救不了速度层的恒定偏置 ✓。
///
/// **成熟飞控对照**：PX4 速度控制器 = **PID（含 I）**；ArduPilot 速度环 =
/// **PID + `sqrt_controller`** ⇒ 本仓水平速度环**缺 I**，是真实缺口 ✓。
///
/// **尚未实现积分主体** —— 属控制律变更，须按已写流程：零件级自检（"给定恒定速度指令，
/// 稳态实际速度应等于指令速度，误差 <5%"）→ 启用 → 全量回归 → 按失败清单重导判据。
#[used]
pub static mut G_KI_V_XY: f32 = -1.0;
/// ★§5.196【垂向通道整定旋钮 ✓】：`kp_z` / `kv_z` / `ki_z` 运行时覆盖（哨兵同规 **<0 = 编译期值** ✓）。
/// 动机：§5.195 把水平游走压到 ~0.55m 后，**垂向（~0.77m）成了主要误差源** ✗ ⇒ 需单独扫 ✓。
pub static mut G_KP_Z: f32 = -1.0;
pub static mut G_KV_Z: f32 = -1.0;
pub static mut G_KI_Z: f32 = -1.0;

/// **速度环 P 增益（水平）**运行时旋钮 —— 位置阶跃的**阻尼**主要来自它（默认 0.8）。
/// 语义同其他 `G_*`：`<0` = 用编译期值 ✓。用于阶段 7 的阻尼整定（免去反复重编译 ✓）。
pub static mut G_KV_XY: f32 = -1.0;
/// ★§5.166d 整定旋钮：水平**位置环**增益覆盖（`-1` ⇒ 默认 0.5 ✓）
#[no_mangle]
#[used]
pub static mut G_KP_XY: f32 = -1.0;
/// ★§5.167【对齐 PX4 一手 `PositionControl.cpp:150` ✓】：**速度环 D 项增益** `Kd_vel`
///   PX4：`acc_sp = Kv·(vel_sp − vel) + vel_int − **Kd·vel_dot**`（`_vel_dot = states.acceleration` ✓）
///   本仓缺该 D 项 ⇒ 外环相位裕度不足（§5.166 实测：噪声激励的不稳定模态 ✓）
///   旋钮：**`-1` ⇒ 关闭**（默认 ✓ 逐位不变）；`>=0` ⇒ 启用
#[no_mangle]
#[used]
pub static mut G_KV_D: f32 = -1.0;
/// ★§5.167 定号旋钮：`2.0` ⇒ D 项**反向**（`+Kd·vel_dot` 而非 PX4 的 `−Kd·vel_dot` ✓）
#[no_mangle]
#[used]
pub static mut G_KV_D_FLIP: f32 = 0.0;
/// ★§5.169【对齐 PX4 一手 `PositionControl.cpp:210-225 _accelerationControl` ✓】：
///   `2.0` ⇒ 用 **`limitTilt` 语义**（限制**合成**倾角 `√(tilt_n²+tilt_e²) ≤ tilt_max` ✓）
///   本仓原为**分量独立 clamp** ✗ ⇒ 合成倾角可达 `√2 × tilt_max`（**41% 超限** ✗✓）
///   默认 `0` ⇒ 原行为（逐位不变 ✓）
#[no_mangle]
#[used]
pub static mut G_TILT_LIMIT_SYNTH: f32 = 0.0;
/// ★§5.171【对齐 PX4 一手 `PositionControl.cpp:190-199` 的 **ARW（积分抗饱和）** ✓】：
///   `vel_error -= arw_gain·(acc_sp − acc_produced)`（`arw_gain = 2/gain_vel_p` ✓）
///   ⇒ 饱和时用"**实际产出加速度**"反推积分 ⇒ 防 windup ✓
///   本仓原只做**幅值上限**（`±I_V_MAX` ✓）而无反推 ✗ ⇒ 饱和时积分继续累积 ⇒ windup ✓
///   `2.0` ⇒ 启用（默认 `0` ⇒ 原行为，逐位不变 ✓）
#[no_mangle]
#[used]
pub static mut G_ARW: f32 = 0.0;
/// ★§5.174【参考模型/速率整形（PX4 `FlightTask` 层思想 ✓）】：**期望速度速率限制**
///   `|Δdes_v| ≤ a_max·dt`（`a_max` = 本旋钮值，m/s² ✓；`0` ⇒ 关 ⇒ 逐位不变 ✓）
///   动机（§5.173 ✓）：外环"缺相位裕度"且各项增益/带宽调整**无效** ⇒ 唯一未试的**结构性**
///   手段 = 让**期望值本身**不含高频（避免激励结构模态 ✓）
#[no_mangle]
#[used]
pub static mut G_VEL_SLEW: f32 = 0.0;
/// ★§5.177 探针：速度环积分 `i_v_xy`（windup 判定 ✓）
#[no_mangle]
#[used]
pub static mut G_IV_DBG: [f32; 2] = [0.0; 2];

/// [标定] 水平一阶低通时间常数（s）运行时覆盖。哨兵同规：**<0 = 用编译期值**。
/// 0 是有效取值（= 关闭低通，即既有行为），故哨兵取 <0。
#[used]
pub static mut G_VEL_LPF_H_TAU: f32 = -1.0;
/// ★§5.139 诊断旋钮：姿态-速率环增益覆盖（**-1 = 不覆盖** ⇒ 默认逐位不变 ✓）
///   用于判定"真机 3D 自激（≈4Hz 俯仰）是否即该环的环路特性" ✓
/// ★§5.155 诊断探针（导出 ✓）：[0..3)=position error (ex,ey,ez)
///   [3..6)=desired velocity (des_vx,des_vy,des_vz) [6]=rate_mode_xy
#[no_mangle]
#[used]
pub static mut G_CTRL_DBG: [f32; 8] = [0.0; 8];

/// ★§5.156【关键 ✓✓】把全部"负哨兵"标定旋钮**显式初始化**为 `-1.0`（= 用编译期默认 ✓）。
///
/// 为何必须（本仓既有坑 ✓ §5.136 同族）：这些旋钮的初值在 **`.data`** 段，而**裸 bin
/// 加载时 `.data` 初值不生效** ✗（实测符号表：`G_KV_XY`/`G_KI_XY`/… 全在 `D/d` 段 ✓）
/// ⇒ 实际读到 **0** ⇒ 而判据多为 `if v >= 0.0 { v }` ⇒ 0 被当作**有效值** ✗✓
/// 实测后果：`G_KV_XY=0` ⇒ **水平速度环增益 = 0** ⇒ `acc_n = kv·(des_v − v) = 0` ⇒
///   速度指令进不了姿态环 ⇒ **机体不动**（LOITER 摇杆微调"位移恒 0"的真根因 ✓✓）
///
/// 调用点：真机 `control_entry` 启动时 ✓（SIL 走 `.data` 正常加载 ⇒ 无此问题 ✓，
///   但调用无害 ✓ 幂等 ✓）
#[inline(never)]
#[no_mangle]
pub extern "C" fn init_runtime_knobs() {
    unsafe {
        // ★§5.157 逐项排查（只初始化 `G_KV_XY` ✓；其余保持"读到 0"的既有行为以免
        //   混淆变量 ✓——排查完成后再决定是否全部初始化 ✓）
        core::ptr::write_volatile(core::ptr::addr_of_mut!(G_KV_XY), -1.0);
    }
}

/// ★§5.158 A/B 旋钮：`2.0` ⇒ 用 **legacy** 期望姿态构造（`from_euler(+tilt_e, -tilt_n, yaw)` ✓）；
///   其余 ⇒ 用推力矢量版 `thrust_to_attitude`（当前默认 ✓）
#[no_mangle]
#[used]
pub static mut G_ATT_LEGACY: f32 = 0.0;
/// ★§5.158 A/B 旋钮：`2.0` ⇒ 水平加速度符号翻转（排查"外环→姿态"符号约定 ✓）
#[no_mangle]
#[used]
pub static mut G_ACC_FLIP: f32 = 0.0;
/// ★§5.165 整定旋钮：速度外推时域覆盖（秒 ✓；`0` ⇒ 用编译期默认 `VEL_PRED_HORIZON` ✓）
#[no_mangle]
#[used]
pub static mut G_VEL_PRED: f32 = -1.0;
/// ★§5.165 整定旋钮：水平速度低通 τ 覆盖（秒 ✓；`-1` ⇒ 用默认 ✓）
#[no_mangle]
#[used]
pub static mut G_VEL_LPF_TAU_K: f32 = -1.0;
/// ★§5.159 探针：[0..3)=姿态误差(机体系) [3..6)=rates(p,q,r) [6]=des_thrust
///   ⚠️§5.159 修：原名 `G_ATT_DBG` 与 `estimator/ekf.rs` 的**同名 `#[no_mangle]` 符号冲突** ✗
///   （实测 ELF 里两个符号都叫 `G_ATT_DBG` ⇒ linker 只保留一个 ⇒ 探针读到 0 ✗）
///   ⇒ 改名 `G_PID_ATT_DBG`（唯一 ✓）
#[no_mangle]
#[used]
pub static mut G_PID_ATT_DBG: [f32; 7] = [0.0; 7];
/// ★§5.161 调用计数（`PidController::control` 被调用次数 ✓；用于确认实际控制路径 ✓）
#[no_mangle]
#[used]
pub static mut G_PID_CALLS: u32 = 0;
pub static mut G_ATT_KP: f32 = -1.0;
pub static mut G_ATT_KD: f32 = -1.0;

pub struct PidController {
    // 位置外环 P：位置误差 -> 期望速度（世界系）
    kp_xy: f32,
    kp_z: f32,
    // 速度中环 P：速度误差 -> 期望加速度（世界系），再映射为倾角/推力
    kv_xy: f32,
    kv_z: f32,
    // 最大速度/倾角限制（防饱和、保稳定）
    vmax_xy: f32,
    vmax_z: f32,
    tilt_max: f32,
    // 速率模式（水平）：位置外环旁路，期望速度 = sp.vel（摇杆直通，推杆飞/松杆停）。
    // 默认 false（位置模式）；非 HIL 演示固件开启（大疆手感），HIL 保持位置模式。
    rate_mode_xy: bool,
    // 姿态内环：四元数误差 -> 机体角速度 的 P（比例）与 D（角速度阻尼）增益
    att_kp: f32,
    att_kd: f32,
    // 推力基值（悬停油门）与重力（用于倾角->加速度映射）
    hover_thrust: f32,
    gravity: f32,
    // 垂向位置积分项（消除传感器噪声下的稳态下沉）：iz 为积分累积，ki_z 为积分增益
    ki_z: f32,
    iz: f32,
    // 水平位置积分项（抗**恒定扰动**下的稳态偏移，如侧风）：与垂向 iz **对称**。
    //
    // 为何需要：水平外环原为 **P-only** ⇒ 恒风下必须靠位置误差换稳态速度指令
    // （`des_v = kp_xy·e`）⇒ 稳态偏移 `e = des_v/kp_xy`。实测 B3 风（5.4m/s、
    // 位置环口径）漂移 **6.64m**，与 `2.0/0.3 = 6.67m` 吻合到 0.5%；仓库自己的
    // 注释也记着这个特征（`mission.rs`："kp_xy=0.3、2m/s 巡航约 6.7m"）。
    // 垂向早有 `iz` 正是为此（"抗稳态下沉"），水平此前没有 —— 这是能力缺口。
    //
    // **默认 0 = 关闭**（保持既有行为不变），取值由扫描数据定，不由实现者拍。
    ki_xy: f32,
    /// 水平位置积分累积（NED 北/东）。抗饱和与垂向同用**回算**（back-calculation）。
    i_xy: [f32; 2],
    /// **水平速度环积分**累积（m/s，NED 北/东）与增益。
    ///
    /// 根因见 `G_KI_V_XY` 的说明：速度环原为 P-only，而平飞巡航须有非零倾角（平衡阻力）
    /// ⇒ `acc ≠ 0` ⇒ P-only 下 `des_v ≠ v` ⇒ 固有速度偏置 ⇒ 积分成位置斜坡。
    /// 与垂向 `ki_z`/`iz` **对称**：同样用**回算**抗饱和。
    ki_v_xy: f32,
    i_v_xy: [f32; 2],
    // 阶段 11-A：EKF 估计的垂直速度/位置一阶低通（EMA）状态，滤除 IMU 高频噪声。
    // 噪声经 EKF 估计后直接驱动油门会导致悬停发散；LPF 时间常数由 vel_lpf_tau 控制（0=不过滤）。
    vel_lpf_tau: f32,
    /// 速率环（内环 omega）一阶低通时间常数（s）。omega = EKF 的 gyro-bias，其噪声
    /// 直达内环 D 项会自激（x_hover_noise 实测 w_y ±1.4、电机差动饱和）。0=不过滤。
    rate_lpf_tau: f32,
    filt_w: [f32; 3], // 滤波后的机体角速度
    rate_filt_init: bool,
    filt_vd: f32,   // 滤波后的垂直速度（NED，向下正）
    filt_d: f32,    // 滤波后的垂直位置（NED，向下正）
    /// 水平一阶低通时间常数（s）。**0 = 关闭（既有行为）**。
    ///
    /// 为何需要：水平位置/速度此前**没有任何低通**就直驱倾角指令，而垂向一直有
    /// （`vel_lpf_tau`，注释写明"噪声经 EKF 估计后直接驱动油门会导致悬停发散"）。
    /// 后果实测（H 场，位置环，60s）：**零均值扰动下位置无界游走** ——
    /// 无恒风时峰值≡末态（9.36m 且仍在增长）；有恒风时因速度估计有确定偏置
    /// 反而有界（0.85m）。即文档 §4.2 早已写下的"估计器输出的速度噪声直接进了
    /// 位置外环…水平通道没有等效处理"。
    vel_lpf_h_tau: f32,
    /// ★§5.167 速度环 D 项状态：上一拍水平速度（世界系 ✓）+ 微分低通状态 ✓
    prev_vel_n: f32,
    prev_vel_e: f32,
    vel_dot_lpf: [f32; 2],
    /// ★§5.174 速率整形状态：上一拍期望速度（世界系 ✓）
    prev_des_vx: f32,
    prev_des_vy: f32,
    /// 水平低通状态 `[pos_n, pos_e, vel_n, vel_e]`；`filt_h_init` 首帧直接赋值。
    filt_h: [f32; 4],
    filt_h_init: bool,
    filt_init: bool, // 首帧直接赋值避免启动瞬态
    // 调试快照：最近一次内环计算的姿态误差向量与期望机体角速度
    dbg_err: [f32; 3],
    dbg_pqr: [f32; 3],
    dbg_omega: [f32; 3],
    // 阶段 11-A 诊断：控制律内部步计数（仅用于一次性 stderr 诊断，固定步后停止）
    dbg_step: u32,
    /// 上一拍的偏航设定点（rad）：用于微分出**偏航速率**，作为速率环的**参考前馈**。
    ///
    /// 动机（2026-09-21 实测）：纯 P-D 速率环（`rates = Kp·err − Kd·ω`）**无前馈** ⇒
    /// 期望姿态随时间旋转时（尤其**偏航以 1 rad/s 持续旋转**）姿态环只能"追"误差 ✗
    /// ⇒ 持续大幅电机差动 ⇒ **单电机贴边 99.5% 的时间**（实测）⇒ 推力/倾角权限被夺
    /// ⇒ 切向偏航下高度掉 **10.002m**、水平掉 10~13m ✗。
    ///
    /// 修法按成熟飞控（PX4 文档原文）："the model's **reference body rate is fed forward**
    /// to the rate setpoint, **removing the pure-P law's steady-state tracking lag**".
    prev_yaw: Option<f32>,
    /// 本拍设定偏航（rad）。由 `control()` 在调用 `control_attitude` 前写入 ——
    /// 因为 `control_attitude` 只收到 `q_des`，拿不到 `Setpoint`。
    sp_yaw: f32,
    // 阶段 11-A 诊断：最近一次控制律内部量（供 host 侧打印，绕开 no_std 无 eprintln）
    dbg_raw_d: f32,
    dbg_raw_vd: f32,
    dbg_filt_d: f32,
    dbg_filt_vd: f32,
    dbg_ez: f32,
    dbg_izv: f32,
    dbg_des_vz: f32,
    dbg_acc_d: f32,
    dbg_des_thr: f32,
    /// ★§5.183：宿主注入的**测量/估计世界系加速度**（NED，m/s²），供速度环 D 项，
    /// 与轨迹前馈 `Setpoint.acc` **分离**（PX4 `states.acceleration` 同源 ✓）。
    /// 全 0 ⇒ 退化为“速度微分”旧法（向后兼容 ✓）。
    world_accel_meas: [f32; 3],
}

impl PidController {
    /// 读取最近一次内环调试快照（误差向量、期望机体角速度、机体角速度）。
    pub fn dbg_last(&self) -> ([f32; 3], [f32; 3], [f32; 3]) {
        (self.dbg_err, self.dbg_pqr, self.dbg_omega)
    }

    /// 调试：返回垂向位置积分累积值。
    pub fn debug_iz(&self) -> f32 {
        self.iz
    }

    /// 阶段 11-A 诊断：返回最近一次控制律内部量元组
    /// (raw_d, raw_vd, filt_d, filt_vd, ez, iz, des_vz, acc_d, des_thr)。
    pub fn debug_pid_internal(&self) -> (f32, f32, f32, f32, f32, f32, f32, f32, f32) {
        (
            self.dbg_raw_d,
            self.dbg_raw_vd,
            self.dbg_filt_d,
            self.dbg_filt_vd,
            self.dbg_ez,
            self.dbg_izv,
            self.dbg_des_vz,
            self.dbg_acc_d,
            self.dbg_des_thr,
        )
    }

    /// 从地面站参数表（[KpXY, KpZ, KvXY, KvZ, HoverThrust]）应用增益。
    /// 仅覆盖这 5 个字段，其余（vmax/tilt/att_kd/gravity）保持出厂默认，
    /// 避免地面站误改导致控制律发散。调用方需保证 `g` 长度 ≥ 5。
    pub fn apply_gains(&mut self, g: &[f32]) {
        if g.len() < 5 { return; }
        self.kp_xy = g[0];
        self.kp_z = g[1];
        self.kv_xy = g[2];
        self.kv_z = g[3];
        self.hover_thrust = g[4];
    }

    /// 切换水平速率模式（位置外环旁路，期望速度 = sp.vel）。
    /// 非 HIL 演示固件开启（大疆手感）；HIL 轨迹模式保持位置环。
    /// ★§5.166 诊断：设置水平速度低通 τ（隔离相位滞后 ✓；测试用 ✓）
    pub fn set_vel_lpf_tau_for_test(&mut self, vd: f32, vh: f32) {
        self.vel_lpf_tau = vd;
        self.vel_lpf_h_tau = vh;
    }

    pub fn set_rate_mode_xy(&mut self, on: bool) {
        self.rate_mode_xy = on;
    }

    /// 典型 450mm X 四旋翼参数（后续可移到机型配置）。
    /// 标准串级：pos_err -> 期望速度(限幅) -> vel_err -> 期望加速度 -> 期望姿态(四元数) -> 角速度。
    pub fn default_quad() -> Self {
        Self {
            kp_xy: 0.5,
            kp_z: 0.5,
            // ★★§5.195【速度环 kv_xy **0.8 → 3.0**（外环抗噪/悬停精度优化，两步收口 ✓）】：
            //   §5.194 先到 1.5（−43%），§5.195 再推到 **3.0**（复现器游走 1.62m → **0.875m** ✓）。
            //   快复现器扫描（`SCAN_INIT_N=0` ⇒ 峰值 = **纯游走**，不被初始偏差饱和 ✓）：
            //     | kv | 0.8 | 1.2 | 1.5 | 2.5 | **3.0** | 4.0 | 6.0 | 8.0 | 12.0 |
            //     | 游走(m) | 3.94 | 2.09 | 1.62 | 1.08 | **0.875** | 0.49 | 0.37 | 0.83 | 2.28(失稳) |
            //   ★**位置环 kp 与位置积分 ki_xy 都不是杠杆**（实测）：
            //     kp ∈ [0.2,2.0] ⇒ 游走 1.48~1.83m（无明显趋势 ✗）；ki_xy=0.1 仅 −23%（kv=4 下**无增益** ✗）
            //     ⇒ 即**当前外环结构本身没问题**，瓶颈是速度环增益偏低 ⇒ 抑制估计噪声能力不足 ✓
            //   ★**上界由轨迹可行性定**（非 hover）：`guidance_track` 的 `sat_ratio<5%` ——
            //     kv=4.0 **饱和占比超限 ✗**；3.2/3.5 ✓ ⇒ 取 **3.0 留裕度** ✓
            //   收益（H 场：复现器 + §5.193 PHY 镜像，产品口径）：位置游走 −78%、长跑速度 −41%、
            //     陀螺零偏末态倾角 −66%、max_tilt 亦更低 ✓
            //   注意：**不是**靠滤波——`vel_lpf_h_tau` 实测**有害**（0.1⇒4.59m ✗）
            //     ⇒ 印证 §5.188「§5.176 的 LPF/SLEW/KVD 整定是围着坏估计器调出来的」✓
            kv_xy: 3.0,
            kv_z: 1.5,
            vmax_xy: 3.5, // 速率模式（摇杆→速度）上限：满杆 3.0m/s 不被 clamp（8 字半径 ~3m）
            vmax_z: 2.0,
            // 倾角指令上限：**25°**（原 20°/0.35rad）。
            //
            // 依据（H 场实测，位置环 60s）：20° 时 5.4m/s（蒲福 3 级上限）下倾角指令
            // 顶满（末|pitch|=20.99°）、位置环失去纠偏 ⇒ 漂移 5.59m；抬到 25° 解除饱和
            // ⇒ 3.45m（改善 38%），且 30°/40° 无进一步收益 ⇒ **25° 已足够**。
            // 代价：所需推力 ∝1/cosθ ⇒ 25° 比 20° 仅多 3.7%。
            // 回归守卫：`wind_turb_scan::tilt_max_attribution_scan`。
            tilt_max: 0.4363, // 25°
            rate_mode_xy: false,
            att_kp: 3.0,
            att_kd: 0.3,
            hover_thrust: 0.5,
            gravity: 9.81,
            ki_z: 0.3, // 原 0.6：积分零点从 ωz=1.2 降到 0.6 rad/s（低于增益穿越 ωc≈0.68 rad/s），
                      // 使相位裕度从 19.7° 提升到 36.0°（GM 保持 inf）；实测垂直抗扰峰值偏差
                      // 反而更小（0.094m vs 0.131m），稳态偏差均≈0，未牺牲抗风性能
            iz: 0.0,
            ki_xy: 0.0, // 默认关：既有行为逐位不变；由 wind_turb_scan 扫出后再定
            // **水平速度环积分默认开启（=1.0）**：2026-09-21 定值。
            // 根因：速度环 P-only ⇒ 平飞巡航须非零倾角（平衡阻力）⇒ acc≠0 ⇒ des_v≠v
            // ⇒ 固有速度偏置 ⇒ 位置斜坡。实测扫描（直线 2m/s / 可行切向偏航圆 R=7m）：
            //   ki_v_xy=0   : 7.741m / 5.681m
            //   ki_v_xy=1.0 : 1.440m / 1.124m  <- 逼近固定偏航基线 1.082m
            //   ki_v_xy=3.0 : 1.131m / 1.387m
            // ⇒ 取 1.0（最优区）。与成熟飞控一致（PX4 速度环 = PID 含 I）。
            // ✅ **已翻为 1.0**（2026-09-21，阶段 7 收口）：
            // 依据 ①：增益扫描 —— 巡航段速度误差直流（= 稳态漂移）验收 <0.15 m/s
            //   只有 ki_v_xy=1.0 达标（0.0→0.5483 / 0.5→0.2342 ✗ / 1.0→0.1359 ✓）；
            //   且位置 RMS 单调改善（2.484 → 0.913m ✓）。
            // 依据 ②：当时"1.0 破坏位置阶跃"的归因**已被实验推翻** ✗ ——
            //   真因是【重力锚定被大水平加速度污染的比力参考带歪】✓（roadmap §9.6/§9.7），
            //   而该共同病因已通过"闭环对齐 SIL 约定 att_alpha=0"消除 ✓
            //   ⇒ 4B（估计反馈）阶跃现与 4A（真值）**完全一致** ✓（tail_peak 0.169 两者相同）。
            ki_v_xy: 1.0,
            // （见 G_KI_V_XY 的说明与下方实测），按流程须先重导那 2 项再翻默认。
            i_xy: [0.0; 2],
            i_v_xy: [0.0; 2],
            vel_lpf_tau: 0.15,
            rate_lpf_tau: 0.02, // 50Hz：抑噪为主，相位滞后小
            filt_w: [0.0; 3],
            rate_filt_init: false,
            filt_vd: 0.0,
            filt_d: 0.0,
            vel_lpf_h_tau: 0.0,
            prev_vel_n: 0.0,
            prev_vel_e: 0.0,
            vel_dot_lpf: [0.0; 2],
            prev_des_vx: 0.0,
            prev_des_vy: 0.0, // 默认关：既有行为逐位不变；取值由 A/B 扫描定
            filt_h: [0.0; 4],
            filt_h_init: false,
            filt_init: false,
            dbg_err: [0.0; 3],
            dbg_pqr: [0.0; 3],
            dbg_omega: [0.0; 3],
            dbg_step: 0,
            prev_yaw: None,
            sp_yaw: 0.0,
            dbg_raw_d: 0.0,
            dbg_raw_vd: 0.0,
            dbg_filt_d: 0.0,
            dbg_filt_vd: 0.0,
            dbg_ez: 0.0,
            dbg_izv: 0.0,
            dbg_des_vz: 0.0,
            dbg_acc_d: 0.0,
            dbg_des_thr: 0.0,
            world_accel_meas: [0.0; 3],
        }
    }

    /// 从机型配置构造。
    pub fn from_config(c: &crate::config::CtrlParams) -> Self {
        let mut s = Self::default_quad();
        s.tilt_max = c.tilt_max;
        s.hover_thrust = c.hover_thrust;
        s.gravity = c.gravity;
        s.vmax_xy = c.vmax_xy;
        s.vmax_z = c.vmax_z;
        s.att_kp = c.att_kp;
        s.att_kd = c.att_kd;
        s.kp_xy = c.kp_xy;
        s.kv_xy = c.kv_xy;
        s.vel_lpf_tau = c.vel_lpf_tau;
        s
    }
}

impl Controller for PidController {
    /// ★§5.183：测量/估计世界系加速度（NED）→ D 项专用通道（与 `Setpoint.acc` 前馈分离）。
    fn set_world_accel(&mut self, a: [MeterPerSecondSquared; 3]) {
        self.world_accel_meas = [a[0].0, a[1].0, a[2].0];
    }

    fn control(&mut self, _dt: Second, sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        // ★§5.161 调用计数探针（唯一名 ✓ 查重名 ✓）
        unsafe {
            let c = core::ptr::read_volatile(core::ptr::addr_of!(G_PID_CALLS));
            core::ptr::write_volatile(core::ptr::addr_of_mut!(G_PID_CALLS), c.wrapping_add(1));
        }
        // 标定旋钮（易失读：由外部写入；0 是有效值故哨兵取 <0）
        {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_KI_XY)) };
            if ov >= 0.0 {
                self.ki_xy = ov;
            }
            let ot = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_VEL_LPF_H_TAU)) };
            if ot >= 0.0 {
                self.vel_lpf_h_tau = ot;
            }
        }
        let g = self.gravity;
        let dt = _dt.0;

        // --- 阶段 11-A：垂直通道 EMA 低通（滤除 IMU 高频噪声） ---
        // IMU 抖动经 EKF 估计后直接驱动油门，会导致悬停向上发散（见 PLAN 阶段 11-A）。
        // 对垂直位置/速度估计做一阶低通，时间常数 vel_lpf_tau（0=不过滤，保持历史行为）。
        // 首帧直接赋值，避免启动瞬态。
        // ★§5.165：水平速度低通 τ 可被旋钮覆盖（默认 -1 ⇒ 用字段 ✓）
        let vel_lpf_h_tau = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_VEL_LPF_TAU_K)) };
            if ov >= 0.0 { ov } else { self.vel_lpf_h_tau }
        };
        let (est_d, est_vd) = if self.vel_lpf_tau > 0.0 {
            let alpha = (dt / (self.vel_lpf_tau + dt)).clamp(0.0, 1.0);
            if !self.filt_init {
                self.filt_d = est.pos[2].0;
                self.filt_vd = est.vel[2].0;
                self.filt_init = true;
            } else {
                self.filt_d += alpha * (est.pos[2].0 - self.filt_d);
                self.filt_vd += alpha * (est.vel[2].0 - self.filt_vd);
            }
            (self.filt_d, self.filt_vd)
        } else {
            (est.pos[2].0, est.vel[2].0)
        };
        // 水平低通：**与垂向完全同形**（位置+速度都滤、首帧直接赋值）。
        let (est_n, est_e, est_vn, est_ve) = if vel_lpf_h_tau > 0.0 {
            let alpha = (dt / (vel_lpf_h_tau + dt)).clamp(0.0, 1.0);
            if !self.filt_h_init {
                self.filt_h = [est.pos[0].0, est.pos[1].0, est.vel[0].0, est.vel[1].0];
                self.filt_h_init = true;
            } else {
                self.filt_h[0] += alpha * (est.pos[0].0 - self.filt_h[0]);
                self.filt_h[1] += alpha * (est.pos[1].0 - self.filt_h[1]);
                self.filt_h[2] += alpha * (est.vel[0].0 - self.filt_h[2]);
                self.filt_h[3] += alpha * (est.vel[1].0 - self.filt_h[3]);
            }
            (self.filt_h[0], self.filt_h[1], self.filt_h[2], self.filt_h[3])
        } else {
            (est.pos[0].0, est.pos[1].0, est.vel[0].0, est.vel[1].0)
        };

        // ⚠️ 旋钮读取必须在**使用之前**（初版插在使用之后 ⇒ 旋钮无效、扫描逐位相同 ✗）
        let vmax_xy_eff = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_VMAX_XY)) };
            if ov > 0.0 { ov } else { self.vmax_xy }
        };
        // --- 外环：位置误差 -> 期望速度（限幅，避免饱和） ---
        // 加入设定点速度前馈：轨迹跟踪时直接把 sp.vel 叠加到期望速度，
        // 减少相位滞后（square/circle 场景 RMS 显著下降）。
        // ★§5.166d：位置环增益旋钮（默认 -1 ⇒ 字段值 ✓）
        {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_KP_XY)) };
            if ov >= 0.0 {
                self.kp_xy = ov;
            }
        }
        let ex = sp.pos[0].0 - est_n;
        let ey = sp.pos[1].0 - est_e;
        let ez = sp.pos[2].0 - est_d;
        unsafe {
            let d = core::ptr::addr_of_mut!(G_CTRL_DBG);
            (*d)[0] = ex; (*d)[1] = ey; (*d)[2] = ez;
            (*d)[6] = if self.rate_mode_xy { 1.0 } else { 0.0 };
        }
        // 垂向位置积分（抗稳态下沉）：iz 累积位置误差，作为期望速度的积分分量。
        // 标准 PI 配条件积分（clamping 抗 windup，PLAN 阶段 11-A）：
        // 用回算（back-calculation）抗积分饱和：当 des_vz 将饱和时，把 iz 直接置为
        // “恰好使 des_vz 抵达饱和边界”的值（并夹在 ±2 内），既不继续 windup、也不反向。
        // 注意：下行饱和（需最大爬升）时 iz 应取正值（与 pre_iz 反向，二者相加恰为 -vmax_z），
        // 旧实现把 clamp 上下界写反且夹了错误变量，导致 iz 朝错误方向 windup 到限幅之外、
        // 抵消 PD 爬升指令、悬停无法恢复。
        // ★§5.196：垂向三增益运行时旋钮（默认 -1 ⇒ 编译期值 ✓ 逐位不变）
        let (kp_z_e, kv_z_e, ki_z_e) = unsafe {
            let kp = core::ptr::read_volatile(core::ptr::addr_of!(G_KP_Z));
            let kv = core::ptr::read_volatile(core::ptr::addr_of!(G_KV_Z));
            let ki = core::ptr::read_volatile(core::ptr::addr_of!(G_KI_Z));
            (
                if kp >= 0.0 { kp } else { self.kp_z },
                if kv >= 0.0 { kv } else { self.kv_z },
                if ki >= 0.0 { ki } else { self.ki_z },
            )
        };
        let pre_iz = kp_z_e * ez + sp.vel[2].0; // P 项 + 速度前馈（不含积分）
        let iz_tent = clampf(self.iz + ki_z_e * ez * dt, -2.0, 2.0);
        let des_vz_tent = pre_iz + iz_tent;
        let mut iz_final = iz_tent;
        if des_vz_tent > self.vmax_z {
            // 向上饱和：iz 回算到使 des_vz 恰为 +vmax_z 的值
            iz_final = clampf(self.vmax_z - pre_iz, -2.0, 2.0);
        } else if des_vz_tent < -self.vmax_z {
            // 向下饱和（需最大爬升）：iz 回算到使 des_vz 恰为 -vmax_z 的值（正值，正确方向）
            iz_final = clampf(-self.vmax_z - pre_iz, -2.0, 2.0);
        }
        self.iz = iz_final;
        // 水平位置积分（抗恒定扰动下的稳态偏移，如侧风）—— 与垂向 iz 对称：
        // 同样用**回算**抗饱和（饱和时把积分置为"恰使 des_v 抵达边界"的值）。
        // 积分上限 ±2.0 m/s，与垂向同口径。
        // 积分上限：运行时旋钮（哨兵 <0 = 用编译期默认 2.0）。为何要可调：
        // B3 风下所需稳态速度指令 ≈ 2.26 m/s > 2.0 ⇒ **积分撞上限**，
        // 残余稳态偏移 = (des_v_need - I_XY_MAX)/kp_xy 被这个夹子决定。
        let i_xy_max = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_I_XY_MAX)) };
            if ov >= 0.0 {
                ov
            } else {
                2.0
            }
        };
        let pre_ix = self.kp_xy * ex + sp.vel[0].0; // P 项 + 速度前馈（不含积分）
        let pre_iy = self.kp_xy * ey + sp.vel[1].0;
        let mut ix_final = clampf(self.i_xy[0] + self.ki_xy * ex * dt, -i_xy_max, i_xy_max);
        let mut iy_final = clampf(self.i_xy[1] + self.ki_xy * ey * dt, -i_xy_max, i_xy_max);
        // ★§5.139 诊断旋钮覆盖（默认 -1 ⇒ 不覆盖 ✓）
        unsafe {
            let kp = core::ptr::read_volatile(core::ptr::addr_of!(G_ATT_KP));
            let kd = core::ptr::read_volatile(core::ptr::addr_of!(G_ATT_KD));
            if kp > 0.0 { self.att_kp = kp; }
            if kd > 0.0 { self.att_kd = kd; }
        }
        if !self.rate_mode_xy {
            let dx = pre_ix + ix_final;
            if dx > vmax_xy_eff {
                ix_final = clampf(vmax_xy_eff - pre_ix, -i_xy_max, i_xy_max);
            } else if dx < -vmax_xy_eff {
                ix_final = clampf(-vmax_xy_eff - pre_ix, -i_xy_max, i_xy_max);
            }
            let dy = pre_iy + iy_final;
            if dy > vmax_xy_eff {
                iy_final = clampf(vmax_xy_eff - pre_iy, -i_xy_max, i_xy_max);
            } else if dy < -vmax_xy_eff {
                iy_final = clampf(-vmax_xy_eff - pre_iy, -i_xy_max, i_xy_max);
            }
        }
        self.i_xy = [ix_final, iy_final];
        // ★★§5.181【对齐 PX4 一手 `PositionControl.cpp:138` 的限幅语义 ✓✓】：
        //   PX4 用 **`ControlMath::constrainXY(vel_sp_position, ff, lim)`** —— 优先保留
        //   **位置环方向**，且按**合成模长**缩放（**保方向** ✓）；本仓原为 `clampf(·, ±vmax)`
        //   **逐轴限幅** ✗ ⇒ 当 `ex`/`ey` 同时很大时，逐轴限幅会**改变期望速度的方向** ✗✓
        //   ⇒ 即"纠偏方向错误" ⇒ 与 §5.179 的"权限"现象自洽 ✓
        //   旋钮 `G_CONSTRAIN_XY=2` ⇒ 启用合成限幅（默认 `0` ⇒ 原逐轴行为 ✓ 逐位不变 ✓）
        let use_xy_constrain = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(G_CONSTRAIN_XY))
        } == 2.0;
        let (raw_vx, raw_vy) = if self.rate_mode_xy {
            (sp.vel[0].0, sp.vel[1].0)
        } else {
            (pre_ix + ix_final, pre_iy + iy_final)
        };
        let (des_vx, des_vy_tmp) = if use_xy_constrain {
            // 合成限幅（保方向 ✓）：`|v| > vmax ⇒ 等比缩放` ✓（等价 PX4 `constrainXY` ✓）
            let mag = crate::math::sqrt(raw_vx * raw_vx + raw_vy * raw_vy);
            if mag > vmax_xy_eff && mag > 1e-9 {
                let k = vmax_xy_eff / mag;
                (raw_vx * k, raw_vy * k)
            } else {
                (raw_vx, raw_vy)
            }
        } else {
            (
                clampf(raw_vx, -vmax_xy_eff, vmax_xy_eff),
                clampf(raw_vy, -vmax_xy_eff, vmax_xy_eff),
            )
        };
        unsafe {
            let d = core::ptr::addr_of_mut!(G_CTRL_DBG);
            (*d)[3] = des_vx;
        }
        // ★§5.174：期望速度**速率限制**（参考模型整形 ✓）
        let (mut des_vx_l, mut des_vy_l) = (des_vx, des_vy_tmp);
        {
            let amax = unsafe {
                core::ptr::read_volatile(core::ptr::addr_of!(G_VEL_SLEW))
            };
            if amax > 0.0 {
                let dmax = amax * dt;
                let dx = (des_vx - self.prev_des_vx).clamp(-dmax, dmax);
                let dy = (des_vy_tmp - self.prev_des_vy).clamp(-dmax, dmax);
                des_vx_l = self.prev_des_vx + dx;
                des_vy_l = self.prev_des_vy + dy;
                self.prev_des_vx = des_vx_l;
                self.prev_des_vy = des_vy_l;
            } else {
                self.prev_des_vx = des_vx;
                self.prev_des_vy = des_vy_tmp;
            }
        }
        let (des_vx, des_vy) = (des_vx_l, des_vy_l);
        let des_vz = clampf(pre_iz + self.iz, -self.vmax_z, self.vmax_z);

        // --- 中环：速度误差 -> 期望世界系加速度 ---
        // P3-A1 轨迹跟踪：在速度误差 P 项之上叠加设定点加速度前馈 `sp.acc`，
        // 使转弯/机动时控制器直接按期望加速度预倾（而非等位置/速度误差积累），
        // 减小轨迹跟踪相位滞后。
        // ---- 水平**速度环积分**（消除恒定阻力下的速度偏置）----
        // 读数：标定旋钮（易失读；0 是有效值 ⇒ 哨兵取 <0）
        let ki_v_eff = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_KI_V_XY)) };
            if ov >= 0.0 { ov } else { self.ki_v_xy }
        };
        // ★§5.180：上限可被旋钮覆盖（默认 2.0 ✓）
        let I_V_MAX: f32 = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_IV_MAX)) };
            if ov >= 0.0 { ov } else { 2.0 }
        };
        let ev_n = des_vx - est_vn;
        let ev_e = des_vy - est_ve;
        unsafe {
            let d = core::ptr::addr_of_mut!(G_IV_DBG);
            (*d)[0] = self.i_v_xy[0];
            (*d)[1] = self.i_v_xy[1];
        }
        // ★§5.171：**ARW 启用时**积分更新移到饱和判定**之后**（PX4 同序 ✓）⇒ 此处先不算
        let arw_on = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(G_ARW))
        } == 2.0;
        // ★§5.182【对齐 PX4 一手**时序** ✓】：PX4 的 `_vel_int` 在**饱和处理之后**累积
        //   （`PositionControl.cpp:190-199` 位于 `_accelerationControl` 与推力饱和之后 ✓）；
        //   本仓原在**之前** ✗ ⇒ 积分"看到"的是**未饱和**的 `acc` ⇒ 与饱和相互作用 ✓
        //   ⇒ 旋钮 `G_IV_AFTER_SAT=2` ⇒ 移到饱和之后累积（默认 `0` ⇒ 原时序 ✓ 逐位不变 ✓）
        let iv_after_sat = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(G_IV_AFTER_SAT))
        } == 2.0;
        if !arw_on && !iv_after_sat {
            // 原行为 ✓（回算抗饱和：饱和时把积分置为"恰使 acc 抵达边界"的值 ✓）
            self.i_v_xy[0] = clampf(self.i_v_xy[0] + ki_v_eff * ev_n * dt, -I_V_MAX, I_V_MAX);
            self.i_v_xy[1] = clampf(self.i_v_xy[1] + ki_v_eff * ev_e * dt, -I_V_MAX, I_V_MAX);
        }

        // 运行时旋钮（阶段 7 阻尼整定）：<0 => 用编译期值 ✓
        let kv = unsafe {
            let g = core::ptr::read_volatile(core::ptr::addr_of!(G_KV_XY));
            if g >= 0.0 { g } else { self.kv_xy }
        };
        // ★§5.167【对齐 PX4 一手：速度环 D 项 `−Kd·vel_dot` ✓】
        //   PX4 `_vel_dot = states.acceleration`（世界系加速度 ✓）；本仓用 `est.accel`
        //   （**机体**系加速度 ✓）旋到世界系（用姿态 ✓；悬停小倾角下近似 ✓）
        let kv_d = unsafe {
            let k = core::ptr::read_volatile(core::ptr::addr_of!(G_KV_D));
            if k >= 0.0 { k } else { 0.0 }
        };
        let (mut acc_d_n, mut acc_d_e) = (0.0f32, 0.0f32);
        if kv_d > 0.0 {
            // ★§5.168【对齐 PX4 `_vel_dot = states.acceleration` ✓】：用**由宿主注入**
            //   的世界系加速度（`set_world_accel` 专用通道 ✓；§5.183 与 `Setpoint.acc`
            //   前馈**分离** ✓）——它经 EKF 比力+重力+低通 ⇒ **平滑**（PX4 同源 ✓）。
            //   未注入（全 0）时**退化为速度微分**（旧法 ✓ 噪声大）。
            let a_lpf = {
                let inj = [self.world_accel_meas[0], self.world_accel_meas[1]];
                if inj[0] != 0.0 || inj[1] != 0.0 {
                    inj
                } else {
                    let vd = [
                        (est_vn - self.prev_vel_n) / dt,
                        (est_ve - self.prev_vel_e) / dt,
                    ];
                    self.prev_vel_n = est_vn;
                    self.prev_vel_e = est_ve;
                    let tau = 0.05f32;
                    let al = dt / (tau + dt);
                    self.vel_dot_lpf[0] += al * (vd[0] - self.vel_dot_lpf[0]);
                    self.vel_dot_lpf[1] += al * (vd[1] - self.vel_dot_lpf[1]);
                    self.vel_dot_lpf
                }
            };
            // ★§5.167 A/B（实测 ✓）：PX4 为 `−Kd·vel_dot`；本仓实测**正向**加 D 项
            //   单调**恶化**（0.2→−40.9、0.5→−62.3、1.0→−79.8 ✗）⇒ 本仓信号链符号
            //   （估计速度微分）与 PX4 相反 ⇒ 需**反向**；但反向后 `K_V_D` 语义须改为
            //   "正值=反向"（当前旋钮 `-1`=关闭、`>=0`=启用且负号 ✓）⇒ 已实测"负值=同默认"
            //   （因 `kv_d < 0` 走关闭分支 ✓）⇒ **下一步**用独立旋钮 `G_KV_D_SIGN` 定号 ✓
            // ★§5.167 定号（A/B ✓）：`G_KV_D_FLIP=2` ⇒ 反向
            let flip = unsafe {
                core::ptr::read_volatile(core::ptr::addr_of!(G_KV_D_FLIP))
            } == 2.0;
            let sgn = if flip { 1.0 } else { -1.0 };
            acc_d_n = sgn * kv_d * a_lpf[0];
            acc_d_e = sgn * kv_d * a_lpf[1];
        }
        let acc_n = kv * (des_vx - est_vn) + sp.acc[0].0 + self.i_v_xy[0] + acc_d_n; // 北向
        unsafe {
            // §5.155 扩展：[4]=acc_n [7]=kv
            let d = core::ptr::addr_of_mut!(G_CTRL_DBG);
            (*d)[4] = acc_n;
            (*d)[7] = kv;
        }
        let acc_e = kv * (des_vy - est_ve) + sp.acc[1].0 + self.i_v_xy[1] + acc_d_e; // 东向
        unsafe {
            // §5.183 扩展：[5]=acc_e（原未记录 ⇒ 东向指令不可观测）
            let d = core::ptr::addr_of_mut!(G_CTRL_DBG);
            (*d)[5] = acc_e;
        }
        let acc_d = kv_z_e * (des_vz - est_vd) + sp.acc[2].0; // 下垂方向（NED），用滤波后垂直速度（★§5.196 旋钮 ✓）

        // 阶段 11-A 诊断：把控制律内部量存进调试字段，供 host 侧打印（绕开 no_std 无 eprintln）。
        self.dbg_raw_d = est.pos[2].0;
        self.dbg_raw_vd = est.vel[2].0;
        self.dbg_filt_d = est_d;
        self.dbg_filt_vd = est_vd;
        self.dbg_ez = ez;
        self.dbg_izv = self.iz;
        self.dbg_des_vz = des_vz;
        self.dbg_acc_d = acc_d;
        // des_thrust 在下方计算，此处先留 0，计算后回填

        // 高度推力：悬停 + 垂直加速度项（acc_d>0 表示要向下加速，减推力）。
        // 期望机体倾角（小角）：北向加速度 -> 俯仰，东向加速度 -> 横滚。
        // 采用四元数误差内环（见下），这里把世界系期望加速度转换为期望姿态四元数。
        // 倾角上限旋钮（易失读；哨兵 <0 = 用编译期值）
        let tilt_max_eff = {
            let ov = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_TILT_MAX)) };
            if ov > 0.0 { ov } else { self.tilt_max }
        };
        // ★§5.169【PX4 `limitTilt` 语义（合成倾角限制 ✓）】
        let (tilt_n, tilt_e) = {
            let use_synth = unsafe {
                core::ptr::read_volatile(core::ptr::addr_of!(G_TILT_LIMIT_SYNTH))
            } == 2.0;
            let (mut tn, mut te) = (acc_n / g, acc_e / g);
            if use_synth {
                // PX4：`body_z` 归一化后 `limitTilt` ⇒ 等价于把 (tilt_n, tilt_e) 按**合成模长**等比缩放 ✓
                let mag = crate::math::sqrt(tn * tn + te * te);
                if mag > tilt_max_eff && mag > 1e-9 {
                    let k = tilt_max_eff / mag;
                    tn *= k;
                    te *= k;
                }
            } else {
                tn = clampf(tn, -tilt_max_eff, tilt_max_eff);
                te = clampf(te, -tilt_max_eff, tilt_max_eff);
            }
            (tn, te)
        };
        // ⚠️ **有条件写**（重要）：每拍无条件写一个 16B 静态会把控制任务推过 4ms 预算
        // ——实测同一固件仅加这条 store，`x_hover_noise` 就从 20.00°/28.64°（有界极限环）
        // 变成 141.90°/87.38°（40s 后发散）。固件里已有 `DBG_PID`(48B)/`DBG_MOTOR`(16B)
        // 等多个每拍诊断量，本条是压垮的那一根。
        // 而 H2 专项真正要问的是"**倾角是否顶满**"，所以只在**顶满时**记录 ⇒ 正常情况
        // 几乎零成本（一个可预测分支），且保留关键信息。
        if tilt_n.abs() >= tilt_max_eff - 1e-6 || tilt_e.abs() >= tilt_max_eff - 1e-6 {
            unsafe { DBG_TILT = [acc_n, acc_e, tilt_n, tilt_e]; }
        }

        // ★§5.171【ARW（PX4 `:190-199` ✓）】：水平饱和时用"**实际产出加速度**"反推积分 ✓
        //   `acc_produced ≈ g·tilt_cmd`（小角 ✓，与 PX4 `_thr_sp·(g/hover_thrust)` 同量纲 ✓）
        //   `vel_error ← vel_error − arw_gain·(acc_sp − acc_produced)`（`arw_gain = 2/kv` ✓ 同 PX4）
        // ★★§5.178【按 PX4 一手精确实现 ARW ✓✓】（`PositionControl.cpp:190-199`）：
        //   `acc_sp_xy_produced = _thr_sp.xy() · (g / _hover_thrust)`（**实际产出**加速度 ✓）
        //   `if |acc_sp.xy| > |acc_sp_xy_produced|: vel_error −= arw_gain·(acc_sp − produced)`
        //   （`arw_gain = 2/gain_vel_p` ✓）
        //   本仓等价量 ✓：`acc_produced` = 由**实际倾角实现的能力**得到 —— 取"**被限幅后**的
        //   `acc`"（`g·tilt_n`/`g·tilt_e` ✓ 即倾角限制**实际允许**的水平加速度 ✓）
        //   此前两次实现均失效 ✗：① `acc_n − g·tilt_n` **恒等 0**（`tilt_n=acc_n/g` 定义 ✓）；
        //   ②"拒绝累积"过保守（更差 ✗）⇒ 本版按一手用**差值反推** ✓
        if arw_on {
            let arw_gain = 2.0 / kv.max(1e-3);
            let prod_n = g * tilt_n; // 倾角限制后**实际可产出**的北向加速度 ✓
            let prod_e = g * tilt_e;
            let ev_n2 = ev_n - arw_gain * (acc_n - prod_n);
            let ev_e2 = ev_e - arw_gain * (acc_e - prod_e);
            self.i_v_xy[0] = clampf(self.i_v_xy[0] + ki_v_eff * ev_n2 * dt, -I_V_MAX, I_V_MAX);
            self.i_v_xy[1] = clampf(self.i_v_xy[1] + ki_v_eff * ev_e2 * dt, -I_V_MAX, I_V_MAX);
        }

        // ★§5.182：`iv_after_sat` 路径（PX4 时序 ✓）——在饱和判定（倾角已算 ✓）之后累积
        if iv_after_sat && !arw_on {
            let sat_n = (acc_n.abs() >= g * tilt_max_eff - 1e-3) || (self.i_v_xy[0].abs() >= I_V_MAX - 1e-3);
            let sat_e = (acc_e.abs() >= g * tilt_max_eff - 1e-3) || (self.i_v_xy[1].abs() >= I_V_MAX - 1e-3);
            // 未饱和 ⇒ 正常累积；饱和 ⇒ 若累积方向会加深饱和则**跳过**（ARW 的简化 ✓）
            let d_n = ki_v_eff * ev_n * dt;
            let d_e = ki_v_eff * ev_e * dt;
            if !(sat_n && d_n * acc_n > 0.0) {
                self.i_v_xy[0] = clampf(self.i_v_xy[0] + d_n, -I_V_MAX, I_V_MAX);
            }
            if !(sat_e && d_e * acc_e > 0.0) {
                self.i_v_xy[1] = clampf(self.i_v_xy[1] + d_e, -I_V_MAX, I_V_MAX);
            }
        }

        // 关键：机体倾斜后推力竖直分量 = T·cos(φ)，必须按 1/cos(φ) 放大总推力，
        // 否则一倾斜就掉高 -> 高度环进一步减推力 -> 死亡螺旋翻滚。
        // ⚠️ **修正（2026-09-21，按轴拆解暴露）**：`cos_tilt` 原先由 **clamp 后的
        // `tilt_n/tilt_e`** 反算 ⇒ 与**实际构造出的姿态**（现由 `f_w` 得出）**不一致** ✗。
        // 实测后果：切向偏航下**高度偏差达 10.003m** ✗（标量范数下完全看不出）。
        //
        // 正解：与姿态构造**共用同一个 `f_w`** —— 推力竖直分量占比 = |f_w_z|/|f_w| ✓
        // （悬停时 f_w=(0,0,-g) ⇒ 比值 1 ✓；倾斜时自然给出 cos θ ✓）。
        let f_w_mag = crate::math::sqrt(
            acc_n * acc_n + acc_e * acc_e + (acc_d - g) * (acc_d - g),
        );
        let cos_tilt = if f_w_mag > 1e-3 {
            ((acc_d - g).abs() / f_w_mag).clamp(0.2, 1.0)
        } else {
            1.0
        };
        let tilt_mag = crate::math::atan2(
            crate::math::sqrt(acc_n * acc_n + acc_e * acc_e),
            (acc_d - g).abs(),
        );
        let dbg_pre = (self.hover_thrust - acc_d / g) / cos_tilt;
        unsafe { DBG_PRE = dbg_pre; }
        let des_thrust = clampf(dbg_pre, 0.1, 1.0);
        unsafe { DBG_THR = des_thrust; }
        self.dbg_des_thr = des_thrust;
        // [联调诊断] 记录内部量（含 gravity 与 clamp 前原始值）
        unsafe {
            DBG_PID = [
                self.hover_thrust, acc_d, des_vz, ez, des_thrust, cos_tilt,
                self.kv_z, est_vd, g,
                (self.hover_thrust - acc_d / g) / cos_tilt,
                self.kp_z, self.ki_z,
            ];
        }

        // ================= 期望姿态：**推力矢量构造**（PX4/ArduPilot 做法）=================
        //
        // ⚠️ **为何换成这个**（2026-09-21，阶段 5 的切向偏航实验暴露）：
        // 旧做法 `from_euler(roll, pitch, yaw)` 的 roll/pitch 是**机体系**欧拉角，而
        // `acc_n/acc_e` 是**世界系** ⇒ 二者仅在 `yaw = 0` 时相同 ✗（我已修过其中一处
        // 实例：手工按 yaw 旋转倾角）。但**手工旋转换是补丁**：它忽略了高阶耦合项，
        // 且每加一处引用都要记得转 ✗。
        //
        // 成熟飞控（PX4/ArduPilot）不用欧拉角：它们把**期望姿态直接由推力矢量构造** ——
        // 让机体 -Z（推力轴）对准"所需比力方向"，再把偏航绕该方向独立施加 ⇒
        // **构造上就坐标系正确**，与偏航无关，无需任何手工旋转 ✓。
        //
        // 推导：推力须同时提供重力与期望加速度 ⇒ 期望比力（世界系 NED）
        //       `f_w = (acc_n, acc_e, acc_d + g)`
        //       推力沿机体 **-Z** ⇒ 机体 -Z 在世界系 = f_w/|f_w|
        //       ⇒ 机体 +Z（NED 下朝下）世界系 = -f_w/|f_w| = zb
        //       把水平姿的 +Z（即 NED 的 (0,0,1)）旋到 zb，再施加偏航即可。
        // ⚠️ **符号（第三次失败的根因）**：`f_w` 是"所需**比力**"（世界系 NED）。
        // 悬停时推力须**向上** = NED 的 (0,0,-1) ⇒ 比力也向上 ⇒ z 分量应为 **-g** ✗
        // （曾误用 `+g`：那给出朝下的比力 ⇒ z_b 反了 ⇒ 命令的姿态上下颠倒 ⇒ 跟踪 10.2m ✗）
        // 对照本仓既有约定：`az_w = a_world[2] + g` 是"加速度"；比力 = 加速度 − g_vec，
        // 而 g_vec = (0,0,+g)（NED 向下为正）⇒ 比力 z = (acc_d + g) − g = acc_d …
        // 更直接地：悬停 acc_d=0 时必须得 (0,0,-g) ⇒ 取 `acc_d - g`。
        // ★§5.158 A/B：水平符号翻转（排查"外环→姿态"符号约定 ✓）
        let flip = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of!(G_ACC_FLIP))
        } == 2.0;
        let (acc_n, acc_e) = if flip { (-acc_n, -acc_e) } else { (acc_n, acc_e) };
        let f_w = [acc_n, acc_e, acc_d - g];
        let q_des_thrust = crate::vehicle::thrust_to_attitude(f_w, sp.yaw);
        // 旧路径（保留为对照：`G_MAG3D_ALPHA` 式旋钮可切换 —— 此处直接返回推力矢量版）
        #[allow(unreachable_code)]
        let q_des_legacy = {
        // 期望姿态四元数：由（roll=+tilt_e, pitch=-tilt_n, yaw=sp.yaw）构成。
        // 飞控机体(经 X-180 实为前-左-下)：推力沿机体 -Z_body。绕 +Y 正转(+pitch) 把推力
        // 旋到 -X(南)，故北向(+X)加速需 -pitch；东向(+Y)则需 +roll（绕 +X 正转把 -Z 旋到 +Y，
        // 实测见下）。
        let yaw = sp.yaw;
        // 期望 roll 取 +tilt_e（实测 2026-08-21）：NED 中绕 +X(前向) 正转(右滚)把机体 -Z
        // 旋到 +Y(东) -> 东向推力，故东向加速度需 +roll；旧代码用 -tilt_e 恰好反向，
        // 导致东向速度指令产生西向推力、东向持续漂移发散（Hover 逐秒诊断 y: 0→-65m）。
        // ⚠️ **坐标系修正（2026-09-21，阶段 5 的切向偏航实验暴露）**：
        // `tilt_n`/`tilt_e` 由**世界系**水平加速度 `acc_n`/`acc_e` 经 `/g` 得到，
        // 而 `from_euler(roll, pitch, yaw)` 里的 roll/pitch 是**机体系**欧拉角 ——
        // 二者只有在 **`yaw = 0`**（机体系与世界系对齐）时才相同。
        //
        // 旧代码直接把 `tilt_e`/`-tilt_n` 当 roll/pitch ⇒ **偏航非零时期望倾角方向错**
        // ⇒ 位置误差纠不回来 ⇒ 发散。实测（阶段 5 分解实验，圆轨迹 r=2m ω=1rad/s）：
        //   yaw=0（偏航固定）：跟踪 err_max **1.305m** ✓
        //   切向偏航（1 rad/s）：跟踪 err_max **22.221m** ✗（17× 劣化）
        //
        // 推导：期望推力水平分量在世界系为 `(tilt_n, tilt_e)`；转到机体系即左乘
        // `Rz(-yaw)`（忽略一阶以下的高阶耦合）：
        //   机体系前向 =  tilt_n·cosψ + tilt_e·sinψ
        //   机体系右向 = -tilt_n·sinψ + tilt_e·cosψ
        // 而本实现的欧拉映射是 `roll = 机体系右向`、`pitch = -机体系前向`
        // （见下方 445~447 行的符号推导）。`yaw=0` 时退化为旧行为 ⇒ 既有判据不变 ✓。
        let (sy, cy) = crate::math::sin_cos(yaw.0);
        let tilt_fwd = tilt_n * cy + tilt_e * sy;
        let tilt_right = -tilt_n * sy + tilt_e * cy;
        Quaternion::from_euler(Radian(tilt_right), Radian(-tilt_fwd), yaw)
        }; // end 旧路径对照块
        // ⚠️ **2026-09-21 回退**：推力矢量版（`q_des_thrust`）首次实现**数值有误** ——
        // 实测姿态误差跳到 **355.37°**（四元数约定/符号错误的特征 ✗）、圆轨迹跟踪由
        // 1.305m 劣化到 **10.119m** ✗。约定未核实前**不能启用**，故回退到已验证的旧路径。
        //
        // **注意**：这不否定该架构（PX4/ArduPilot 确实用推力矢量构造期望姿态 ✓），
        // 只说明**我的实现需要先做约定核对**（`Quaternion` 的乘法序、`from_axis_angle`
        // 的旋转方向、以及 `att` 是"世界→机体"还是"机体→世界"）—— 这正是本会话反复
        // 教训的"仪表/约定先自检"。
        // ⚠️ **2026-09-21 二次回退**：约定自检确实抓到了**乘法序**问题（`A*B` 在本仓
        // 是"先 A 后 B"，与标准相反）并已修，但**修后结果逐位不变** ✗ ⇒ 该实现仍有
        // 其它错误，且**当前指标无法定位**（见下）。
        // 另：本测例的"姿态误差 355°"极可能是**指标自身的缠绕假象** —— roll/pitch 由
        // atan2 求得，在 ±180 附近会跳变，两曲线符号相反即算出 ~360° ✗（非真实误差）。
        // ⇒ **再回退**；待 ①推力矢量实现逐项与自检对齐 ②姿态指标改为**四元数夹角**
        // （无缠绕，如 2·acos|⟨q1,q2⟩|）之后，再启用。
        // ⚠️ **2026-09-21 第三次回退**：约定自检**已全绿** ✓（`thrust_to_attitude` 的三条
        // 契约：①机体 (0,0,1)→-f_w/|f_w| ②机体 -Z→f_w ③偏航被保留），姿态估计误差也从
        // 355° 降到 **3.03°**（甚至优于旧路径的 4.29° ✓）。
        // **但跟踪仍 10.195m vs 旧路径 1.305m（8× 劣化）** ✗
        // ⇒ 还有一层**本仓特有的轴约定**未对齐 —— 代码注释自述：
        //   "飞控机体(**经 X-180 实为前-左-下**)：推力沿机体 -Z_body …
        //    绕 +Y 正转(+pitch) 把推力旋到 -X(南)… 东向则需 +roll"
        //   即**本仓机体系不是标准 FRD**，故"标准"三轴构造不匹配 ✗。
        // ⇒ 回退。下一步须**先从 mixer/plant 反解出本仓的真实机体轴约定**，
        //   再据此改写构造（而不是照搬教科书 FRD）。
        // ★§5.158 A/B：切到 legacy（注释记载：thrust 版 8× 劣化 ✗；legacy 版实测 1.305m ✓）
        let q_des = if unsafe { core::ptr::read_volatile(core::ptr::addr_of!(G_ATT_LEGACY)) } == 2.0 {
            q_des_legacy
        } else {
            q_des_thrust
        };

        self.sp_yaw = sp.yaw.0; // 供内环做偏航速率前馈（见 prev_yaw 的说明）
        self.control_attitude(_dt, q_des, des_thrust, est)
    }

    fn reset(&mut self) {
        // 四元数误差内环无状态积分；清除垂向位置积分项防 windup 残留
        self.iz = 0.0;
        // 阶段 11-A：重置 EMA 滤波状态，避免跨任务/重启残留
        self.filt_vd = 0.0;
        self.filt_d = 0.0;
        self.filt_w = [0.0; 3];
        self.rate_filt_init = false;
        self.filt_init = false;
        self.dbg_step = 0;
    }
}

#[inline]
fn clampf(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

/// 姿态内环的独立入口（固有方法，不属于 `Controller` trait）。
impl PidController {
    /// **姿态内环**：期望姿态 + 总推力 → 执行器指令（**不含**位置/速度外环）。
    ///
    /// 从 [`control`](Controller::control) 尾部提取，使姿态内环**可独立驱动**。
    /// 动机（阶段 2）：姿态环原先**没有外部入口** —— `q_des` 只由外环在
    /// `control()` 内部生成（`tilt = clamp(acc/g, ±tilt_max)`），而阶段 2 的核心
    /// 需求是“先用**真值姿态**验证控制器本身，再接入估计姿态”。
    ///
    /// `control()` 内部调用本方法，**行为逐位不变**（纯提取，无逻辑改动）。
    ///
    /// 链路：`omega` 一阶低通 → `attitude_rates`（四元数误差 P-D → 期望机体角速率）
    /// → `x4_mix` → 限幅。注意此处**无独立速率 PID**：角速率指令直接进混控。
    pub fn control_attitude(
        &mut self,
        _dt: Second,
        q_des: Quaternion,
        des_thrust: f32,
        est: &VehicleState,
    ) -> ActuatorCmd {
        let dt = _dt.0;
        // --- 内环：四元数姿态误差 -> 期望机体角速度（标准鲁棒写法，无欧拉角奇点） ---
        // 复用共享姿态内环 `attitude::attitude_rates`（P3-A3 提取，与 TECS 完全一致）。
        // 含：q_err = q_est^-1 ⊗ q_des、误差旋转向量 ≈ 2·sign(w)·(x,y,z)、
        //     期望机体角速度 = Kp_att·误差向量 - Kd_att·当前角速度（阻尼）。
        // 速率环低通：omega 直接来自 EKF(gyro-bias)，噪声直达内环 D 项会自激。
        // 一阶低通 rate_lpf_tau（0=不过滤）。首帧直接赋值避免启动瞬态。
        let omega_f = if self.rate_lpf_tau > 0.0 {
            let a = (dt / (self.rate_lpf_tau + dt)).clamp(0.0, 1.0);
            if !self.rate_filt_init {
                self.filt_w = [est.omega[0].0, est.omega[1].0, est.omega[2].0];
                self.rate_filt_init = true;
            } else {
                for k in 0..3 {
                    self.filt_w[k] += a * (est.omega[k].0 - self.filt_w[k]);
                }
            }
            self.filt_w
        } else {
            [est.omega[0].0, est.omega[1].0, est.omega[2].0]
        };
        let mut att_out = super::attitude::attitude_rates(
            est.att,
            q_des,
            self.att_kp,
            self.att_kd,
            omega_f,
        );
        // ---- **参考机体角速度前馈**（PX4 `MC_REF_FF` 同构）----------------------
        //
        // 期望姿态随时间旋转时（偏航速率 `ψ̇`），机体**本就该**以 `R^T·(0,0,ψ̇)` 的角速度
        // 旋转 ⇒ 这部分应由**前馈**给出，而不是留给 P 项"追" ✗。
        // 不加前馈时（实测）：切向偏航（1 rad/s）下姿态环持续要求大幅差动 ⇒ 单电机贴边
        // **99.5%** 的时间 ⇒ 高度掉 10.002m、水平掉 10~13m ✗。
        let yaw_sp = self.sp_yaw;
        let yaw_rate = match self.prev_yaw {
            Some(py) if dt > 1e-6 => {
                let mut d = yaw_sp - py;
                // 归一到 [-π, π]，避免跨 ±π 的假大速率
                while d > core::f32::consts::PI { d -= 2.0 * core::f32::consts::PI; }
                while d < -core::f32::consts::PI { d += 2.0 * core::f32::consts::PI; }
                d / dt
            }
            _ => 0.0,
        };
        self.prev_yaw = Some(yaw_sp);
        // ⚠️ **默认关闭（2026-09-21）**：本次实现使结果**更差** ✗
        //   切向偏航稳态 17.850m -> **23.449m**、饱和 99.5% -> **100.0%** ✗
        // ⇒ **符号/轴又错了**（本区域第 4 个约定问题 ✗）。
        // 按纪律：不留更差的实现，故做成开关并默认关。
        // **下一步**：为"参考角速度前馈"写**零件级自检**（与姿态构造那次同法 ✓——
        // 那次迭代 3 轮才成功 ✓）：给定已知偏航速率，断言前馈向量在**机体轴**上的
        // 方向与量级（本仓机体系非标准 FRD，不能照搬教科书的 R^T·(0,0,ψ̇)）。
        // ⚠️ **两次实现均未改善，默认关闭**（2026-09-21）：
        //   ① 加到输出（裸 rad/s）：17.850 → 23.449m、饱和 99.5% → 100.0% ✗
        //   ② 并入 D 项输入（×att_kd，量纲正确）：19.031m、饱和 99.8% ✗
        // ⇒ 即便量纲修对了也不改善 ⇒ 说明**饱和不是来自"速率环抵抗被命令旋转"** ✗，
        //   或 `ω_ff` 的**方向/符号本身是错的** ✗（从未验证 —— 我又跳过了自检 ✗）。
        //
        // **教训（本会话第 N 次）**：姿态构造那次**先写零件级自检**，3 轮就成功 ✓；
        // 这两次**跳过自检直接改闭环**，两次都失败且**无从判断** ✗。
        // ⇒ 下一步必须先写自检：给定已知 ψ̇，断言 `ω_ff` 在**机体三轴**上的分量
        //   （符号与量级），且**独立于实现推导**（否则会再犯"自洽地一起错" ✗）。
        //
        // **正确位置（供自检通过后参考）：并入 D 项的输入，不是加到输出**
        //
        // 量纲分析：`att_out.rates = att_kp·err − att_kd·ω` 直接送混控 ⇒ 是"力矩量纲" ✗。
        // ① 裸加 rad/s 是量纲错 ✗（上一轮实测：17.850m → 23.449m、饱和 99.5% → 100.0%）；
        // ② 且**恒速旋转本来不需要力矩** ⇒ 往输出端加前馈物理上就不对 ✗。
        //
        // 真正的病：D 项 `−att_kd·ω` **对抗被命令的旋转** —— 偏航以 1 rad/s 旋转 ⇒ `ω` 大
        // ⇒ D 项持续索要大差动 ⇒ **单电机贴边 99.5%** ✗。
        // 正解：D 项应阻尼**相对被命令速率**的偏差 ⇒ `−att_kd·(ω − ω_ff)`，
        // 等价于 `rates += att_kd · ω_ff` ✓ —— **量纲正确**（Kd 的量纲 × rad/s ✓），
        // 且"零姿态误差 + 零角速度"时输出仍为零 ✓（不破坏基本性质）。
        const ENABLE_YAW_RATE_FF: bool = false;
        if ENABLE_YAW_RATE_FF && yaw_rate.abs() > 1e-6 {
            let w_ff = crate::vehicle::rotate_vec_by_quat_inverse(est.att, [0.0, 0.0, yaw_rate]);
            for k in 0..3 {
                att_out.rates[k] += self.att_kd * w_ff[k];
            }
        }
        self.dbg_err = att_out.err;
        self.dbg_pqr = att_out.rates;
        unsafe {
            // ★§5.161 探针写入（唯一名 ✓）：[0..3)=err(机体系) [3..6)=rates [6]=thrust
            let d = core::ptr::addr_of_mut!(G_PID_ATT_DBG);
            for k in 0..3 {
                (*d)[k] = att_out.err[k];
                (*d)[3 + k] = att_out.rates[k];
            }
            (*d)[6] = des_thrust;
        }
        unsafe {
            // ★§5.159 探针：[0..3)=err（机体轴） [3..6)=rates(p,q,r) [6]=des_thrust
            let d = core::ptr::addr_of_mut!(G_PID_ATT_DBG);
            for k in 0..3 {
                (*d)[k] = att_out.err[k];
                (*d)[3 + k] = att_out.rates[k];
            }
            (*d)[6] = des_thrust;
        }
        self.dbg_omega = [est.omega[0].0, est.omega[1].0, est.omega[2].0];

        // --- 混控：X 型四旋翼（0=前右 1=后左 2=前左 3=后右） ---
        // 复用共享混控 `attitude::x4_mix`（P3-A3 提取，与 TECS 完全一致）。
        // 布局与符号（含 yaw 取 +r_cmd 的符号修正）见 attitude.rs 混控注释；
        // 控制器命令 (p_cmd,q_cmd,r_cmd) 定义在飞控机体轴（NED/FRD：前-X 右-Y 下-Z）。
        let motors = super::attitude::x4_mix(des_thrust, att_out.rates);

        ActuatorCmd {
            motor: [
                clampf(motors[0], 0.0, 1.0),
                clampf(motors[1], 0.0, 1.0),
                clampf(motors[2], 0.0, 1.0),
                clampf(motors[3], 0.0, 1.0),
            ],
        }
    }
}
