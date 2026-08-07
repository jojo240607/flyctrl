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
    /// 机体阻力系数（前/右/下三轴，N/(m/s)^2），描述机身/机臂外露气动阻力。
    pub drag_coeff: [f32; 3],
    /// 诱导阻力系数（无量纲），旋翼向下诱导速度产生的附加阻力。
    pub induced_drag_coeff: f32,
    /// 桨盘面积 (m^2)，动量理论诱导速度计算用。
    pub disk_area: f32,
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
            // 机身/机臂外露阻力（前向略大，向下最小）：N/(m/s)^2 × v^2
            drag_coeff: [0.18, 0.18, 0.10],
            // 诱导阻力：与总推力平方根成正比（动量理论），无量纲标定
            induced_drag_coeff: 0.12,
            // 450mm 四旋翼等效桨盘面积（4× 桨盘），约 0.19 m^2
            disk_area: 0.19,
            air_density: 1.225,
            gravity: 9.81,
            tilt_max: 0.35,
            hover_thrust: 0.5,
            vmax_xy: 2.0,
            vmax_z: 2.0,
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
            air_density: self.air_density,
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
        }
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
    /// 空气密度 (kg/m^3)，标准海平面 1.225。
    pub air_density: f32,
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
}

impl Default for CtrlParams {
    fn default() -> Self {
        VehicleConfig::default_quad().ctrl_params()
    }
}
