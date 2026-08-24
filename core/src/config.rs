//! 机型配置抽象（airframe config）。
//!
//! 把"这台飞机是什么"与"物理仿真/控制律参数"解耦：所有算法与仿真都从同一个
//! [`VehicleConfig`] 派生各自所需的参数。新增机型（六旋翼、垂起 VTOL、不同轴距）
//! 只需提供一个 [`VehicleConfig`] 实例，不必改动物理/控制代码。
//!
//! 派生关系：
//! - [`VehicleConfig::dyn_params`] -> 仿真侧 [`flyctrl_sim::physics::PhysicsParams`]（经 `From`）
//! - [`VehicleConfig::ctrl_params`] -> 控制律侧 [`CtrlParams`]

/// 机型配置：描述一架多旋翼/垂起飞机的物理与几何参数。
#[derive(Debug, Clone, Copy)]
pub struct VehicleConfig {
    pub name: &'static str,
    pub mass: f32,             // kg
    pub arm_length: f32,       // m（机臂长度，X 型四旋翼中心到电机）
    pub thrust_coeff: f32,     // N per 单位推力指令（满油门单电机推力）
    pub torque_coeff: f32,     // N·m 力矩系数（偏航混控用）
    pub inertia: [f32; 3],     // 主惯量 Ixx Iyy Izz (kg·m^2)
    pub motor_tau: f32,        // 电机一阶响应时间常数 (s)
    // ---- 阶段 8 动力系统（电池 + 电机 + 螺旋桨）----
    /// 电池标称电压（V），如 4S LiPo ≈ 14.8。
    pub battery_v_nom: f32,
    /// 电池内阻（Ω），决定大电流下电压跌落幅度。
    pub battery_r: f32,
    /// 电机转速常数（rad/s per V），由 KV（rpm/V）换算 KV·2π/60。
    pub motor_kv: f32,
    /// 电机绕组电阻（Ω），影响电流与发热。
    pub motor_r: f32,
    /// 电机+螺旋桨转子绕轴转动惯量（kg·m²），用于陀螺进动效应。
    pub rotor_inertia: f32,
    // ---- P3-C1 叶素理论（BET）/ 桨叶挥舞 / 桨尖失速（仿真侧高保真用）----
    /// 桨叶数（默认 2，双叶桨）。
    pub rotor_blades: f32,
    /// 桨叶实度 σ = B·c/(π·R)（默认 0.08，典型小型多旋翼桨盘）。
    pub rotor_solidity: f32,
    /// 桨叶升力线斜率 a（rad⁻¹，默认 2π≈6.283，薄翼理论）。
    pub rotor_cl_alpha: f32,
    /// 桨叶剖面零升阻力系数 Cd0（默认 0.012）。
    pub rotor_cd0: f32,
    /// 桨叶失速攻角（rad，默认 0.24 ≈ 14°，后行侧叶尖触失速边界）。
    pub rotor_stall_alpha: f32,
    // ---- P3-C2 桨盘干扰（相邻桨下洗耦合修正项）----
    /// 桨盘干扰耦合系数（无量纲，默认 0.35）。前飞时上游桨的滑流（下洗 vi）被自由流
    /// 吹向后，部分射入下游桨盘 → 下游桨入流比增大、推力下降、反扭矩增大（前/后桨
    /// 不对称，产生前飞俯仰干扰力矩）。悬停（v_xy=0）时滑流垂直向下、桨盘共面互不
    /// 干扰，耦合为 0（保持 P3-C1 悬停标定）。0 关闭该修正项。
    pub rotor_downwash_coupling: f32,
    /// 机体阻力系数（前/右/下三轴，N/(m/s)^2），描述机身/机臂外露气动阻力。
    pub drag_coeff: [f32; 3],
    /// 诱导阻力系数（无量纲），旋翼向下诱导速度产生的附加阻力。
    pub induced_drag_coeff: f32,
    /// 桨盘面积 (m^2)，动量理论诱导速度计算用。
    pub disk_area: f32,
    /// 滑流拖曳系数（无量纲），旋翼下洗气流冲击机体产生的附加下拉力。
    /// 动量理论下洗速度 vi = sqrt(T/(2·ρ·A))，滑流作用在机体投影面积上的
    /// 下拉力 ≈ slipstream_drag_coeff · 0.5·ρ·A·vi²。
    pub slipstream_drag_coeff: f32,
    /// 空气密度 (kg/m^3)。
    pub air_density: f32,
    pub gravity: f32,          // m/s^2（NED 下为正）
    /// 最大可指令倾角（rad），用于位置环限幅（防饱和、保稳定）。
    pub tilt_max: f32,
    /// 悬停归一化油门（总推力基值）。
    pub hover_thrust: f32,
    /// 最大速度（m/s），位置外环输出限幅。
    pub vmax_xy: f32,
    pub vmax_z: f32,
    /// 姿态内环比例增益（四元数误差 -> 期望机体角速度）。
    pub att_kp: f32,
    /// 姿态内环角速度阻尼增益。
    pub att_kd: f32,
    /// 位置外环比例增益（位置误差 -> 期望速度）。
    pub kp_xy: f32,
    /// 速度中环比例增益（速度误差 -> 期望加速度）。
    pub kv_xy: f32,
    /// 垂直速度/位置进入控制律前的一阶低通时间常数（秒）。
    /// IMU 高频噪声经 EKF 估计后直接驱动油门，会导致悬停发散（PLAN 阶段 11-A）。
    /// 设 >0 时启用 EMA 滤波（截止频率 ≈ 1/τ）；设 0 时不过滤（保持历史行为）。
    pub vel_lpf_tau: f32,
    /// 空速拖拽前馈系数（m/s² per (m/s)²）：TECS 用真空速直接预补偿机体气动型阻。
    /// 型阻加速度 ≈ 0.5·ρ·Cd_h/m·v_rel²，default_quad 量级 ≈ 0.09。
    /// 0 表示关闭前馈（退化纯反馈，用于对照测试）。
    pub drag_fwd: f32,
}

impl VehicleConfig {
    /// 典型 450mm X 四旋翼（~1.2kg，满油门 ~2.4 倍重力）。基线机型。
    pub fn default_quad() -> Self {
        Self {
            name: "450quad-X",
            mass: 1.2,
            arm_length: 0.225,
            thrust_coeff: 2.4 * 9.81 / 4.0,
            torque_coeff: 0.02,
            inertia: [0.02, 0.02, 0.04],
            motor_tau: 0.05,
            // 动力系统：4S LiPo（~14.8V，内阻 0.015Ω，悬停掉压小、大油门掉压明显）
            // + 980KV 电机（~102.6 rad/s/V，绕组 0.12Ω）
            battery_v_nom: 14.8,
            battery_r: 0.015,
            motor_kv: 102.6,
            motor_r: 0.12,
            rotor_inertia: 1.2e-5, // 小型旋翼+电机转子惯量 ≈ 1.2e-5 kg·m²
            // P3-C1 叶素理论参数：双叶桨、实度 0.08、薄翼 Cl 斜率 2π、
            // 剖面阻力 0.012、失速攻角 14°（默认典型值，可经 airframe TOML 覆盖）。
            rotor_blades: 2.0,
            rotor_solidity: 0.08,
            rotor_cl_alpha: 6.2832,
            rotor_cd0: 0.012,
            rotor_stall_alpha: 0.24,
            // P3-C2 桨盘干扰：前飞同侧前→后桨滑流耦合系数 0.25（悬停自动为 0）。
            rotor_downwash_coupling: 0.25,
            // 机身/机臂外露阻力（前向略大，向下最小）：N/(m/s)^2 × v^2
            drag_coeff: [0.18, 0.18, 0.10],
            // 诱导阻力：与总推力平方根成正比（动量理论），无量纲标定
            induced_drag_coeff: 0.12,
            // 450mm 四旋翼等效桨盘面积（4× 桨盘），约 0.19 m^2
            disk_area: 0.19,
            // 滑流拖曳：下洗冲击机体，等效约 6% 的桨盘动量通量耦合到机身。
            slipstream_drag_coeff: 0.06,
            air_density: 1.225,
            gravity: 9.81,
            tilt_max: 0.35,
            hover_thrust: 0.5,
            vmax_xy: 2.0,
            vmax_z: 2.0,
            att_kp: 3.0,
            att_kd: 0.3,
            kp_xy: 0.3,
            kv_xy: 0.8,
            vel_lpf_tau: 0.15,
            // 0.5·ρ·Cd_h/m ≈ 0.5·1.225·0.18/1.2 ≈ 0.092（default_quad 机体水平型阻）
            // + P3-C1 BET 桨盘阻力（H 力 + 挥舞后倾，5 m/s 约 0.8 m/s²，≈0.033 平均到 v²）
            // + P3-C2 下洗耦合前飞效率惩罚（≈0.015 平均到 v²）
            // ≈ 0.14（TECS 空速拖拽前馈按此补偿总寄生阻力）
            drag_fwd: 0.14,
        }
    }

    /// 派生仿真动力学参数（core 内定义，sim 侧提供 `From` 转换）。
    pub fn dyn_params(&self) -> DynParams {
        DynParams {
            mass: self.mass,
            arm_length: self.arm_length,
            thrust_coeff: self.thrust_coeff,
            torque_coeff: self.torque_coeff,
            inertia: self.inertia,
            motor_tau: self.motor_tau,
            gravity: self.gravity,
            drag_coeff: self.drag_coeff,
            induced_drag_coeff: self.induced_drag_coeff,
            disk_area: self.disk_area,
            slipstream_drag_coeff: self.slipstream_drag_coeff,
            air_density: self.air_density,
            battery_v_nom: self.battery_v_nom,
            battery_r: self.battery_r,
            motor_kv: self.motor_kv,
            motor_r: self.motor_r,
            rotor_inertia: self.rotor_inertia,
        }
    }

    /// 派生控制律共享参数（位置环/姿态环通用）。
    pub fn ctrl_params(&self) -> CtrlParams {
        CtrlParams {
            mass: self.mass,
            gravity: self.gravity,
            tilt_max: self.tilt_max,
            hover_thrust: self.hover_thrust,
            vmax_xy: self.vmax_xy,
            vmax_z: self.vmax_z,
            att_kp: self.att_kp,
            att_kd: self.att_kd,
            kp_xy: self.kp_xy,
            kv_xy: self.kv_xy,
            vel_lpf_tau: self.vel_lpf_tau,
            drag_fwd: self.drag_fwd,
        }
    }
}

/// 便捷构造：用给定姿态/位置环增益覆盖默认机型配置（调试/调参用）。
impl VehicleConfig {
    pub fn with_gains(mut self, att_kp: f32, att_kd: f32, kp_xy: f32, kv_xy: f32) -> Self {
        self.att_kp = att_kp;
        self.att_kd = att_kd;
        self.kp_xy = kp_xy;
        self.kv_xy = kv_xy;
        self
    }
}

/// 仿真动力学纯数据参数（与 `flyctrl_sim::physics::PhysicsParams` 字段一致）。
/// 定义在 core 以避免 core→sim 反向依赖；sim 侧实现 `From<DynParams>`。
#[derive(Debug, Clone, Copy)]
pub struct DynParams {
    pub mass: f32,
    pub arm_length: f32,
    pub thrust_coeff: f32,
    pub torque_coeff: f32,
    pub inertia: [f32; 3],
    pub motor_tau: f32,
    pub gravity: f32,
    /// 机体阻力系数（前/右/下三轴，N/(m/s)^2），描述机身/机臂外露气动阻力。
    pub drag_coeff: [f32; 3],
    /// 诱导阻力系数（无量纲）：旋翼向下诱导速度产生的附加阻力，
    /// 与总推力平方根成正比（动量理论：v_ind = sqrt(T/(2·ρ·A))）。
    pub induced_drag_coeff: f32,
    /// 桨盘面积 (m^2)，用于诱导速度计算（动量理论）。
    pub disk_area: f32,
    /// 滑流拖曳系数（无量纲），旋翼下洗气流冲击机体产生的附加下拉力。
    pub slipstream_drag_coeff: f32,
    /// 空气密度 (kg/m^3)，标准海平面 1.225。
    pub air_density: f32,
    // ---- 阶段 8 动力系统 ----
    /// 电池标称电压（V）。
    pub battery_v_nom: f32,
    /// 电池内阻（Ω）。
    pub battery_r: f32,
    /// 电机转速常数（rad/s per V）。
    pub motor_kv: f32,
    /// 电机绕组电阻（Ω）。
    pub motor_r: f32,
    /// 电机+螺旋桨转子绕轴转动惯量（kg·m²）。
    pub rotor_inertia: f32,
}

/// 控制律共享参数子集（从 [`VehicleConfig`] 派生）。
#[derive(Debug, Clone, Copy)]
pub struct CtrlParams {
    pub mass: f32,
    pub gravity: f32,
    pub tilt_max: f32,
    pub hover_thrust: f32,
    pub vmax_xy: f32,
    pub vmax_z: f32,
    /// 姿态内环比例增益（四元数误差 -> 期望机体角速度）。
    pub att_kp: f32,
    /// 姿态内环角速度阻尼增益。
    pub att_kd: f32,
    /// 位置外环比例增益（位置误差 -> 期望速度）。
    pub kp_xy: f32,
    /// 速度中环比例增益（速度误差 -> 期望加速度）。
    pub kv_xy: f32,
    /// 垂直速度/位置进入控制律前的一阶低通时间常数（秒）。
    /// IMU 高频噪声经 EKF 估计后直接驱动油门，会导致悬停发散（PLAN 阶段 11-A）。
    /// 设 >0 时启用 EMA 滤波（截止频率 ≈ 1/τ）；设 0 时不过滤（保持历史行为）。
    pub vel_lpf_tau: f32,
    /// 空速拖拽前馈系数（m/s² per (m/s)²），TECS 用（见 VehicleConfig::drag_fwd）。
    pub drag_fwd: f32,
}

impl Default for CtrlParams {
    fn default() -> Self {
        VehicleConfig::default_quad().ctrl_params()
    }
}
