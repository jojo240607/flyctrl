//! 六自由度刚体动力学 + 电机推力/力矩模型 + 气动阻力。
//!
//! 坐标系：世界 NED（北-X，东-Y，下-Z，重力沿 +Z 向下）。
//! 机体 FRD（前-X，右-Y，下-Z）。
//!
//! 模型层次：
//!   1. 电机：归一化推力指令 [0,1] -> 实际推力/力矩（一阶响应 + 推力系数）。
//!   2. 混控：4 电机推力/力矩 -> 机体总推力 F_b 与力矩 M_b。
//!   3. 刚体：姿态四元数 + 角速度动力学；平移受重力 + 推力 + 气动阻力。
//!   4. 气动：机体速度平方阻力（近似多旋翼气动，含诱导阻力项）。

use flyctrl_core::units::*;
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, Quaternion, VehicleState};

pub struct PhysicsParams {
    pub mass: f32,            // kg
    pub arm_length: f32,      // m（机臂长度）
    pub thrust_coeff: f32,    // N per 单位推力指令（满油门推力）
    pub torque_coeff: f32,    // N·m 力矩系数（偏航混控用）
    pub inertia: [f32; 3],    // 主惯量 Ixx Iyy Izz
    pub motor_tau: f32,       // 电机一阶响应时间常数 (s)
    pub drag_coeff: f32,      // 平移气动阻力系数 (N/(m/s)^2)
    pub gravity: f32,         // m/s^2（NED 下为正，沿 +Z）
}

impl Default for PhysicsParams {
    /// 典型 450mm 四旋翼（~1.2kg，满油门~2.4kgf 推力）。
    fn default() -> Self {
        Self {
            mass: 1.2,
            arm_length: 0.225,
            thrust_coeff: 2.4 * 9.81 / 4.0, // 4 电机合计满油门 ~2.4 倍重力
            torque_coeff: 0.02,
            inertia: [0.02, 0.02, 0.04],
            motor_tau: 0.05,
            drag_coeff: 0.15,
            gravity: 9.81,
        }
    }
}

pub struct Physics {
    params: PhysicsParams,
    state: VehicleState,
    /// 电机实际推力（带一阶滞后），用于动力学积分
    motor_force: [f32; 4],
}

impl Physics {
    pub fn new(params: PhysicsParams) -> Self {
        Self {
            params,
            state: VehicleState::zero(),
            motor_force: [0.0; 4],
        }
    }

    pub fn state(&self) -> VehicleState { self.state }

    /// 推进一个控制周期。
    /// `cmd` 为控制器输出，`dt` 为周期。返回该周期内的"理想 IMU 测量"
    /// （噪声/偏差由 world 层注入）。
    pub fn step(&mut self, dt: Second, cmd: ActuatorCmd) -> ImuSample {
        let dt = dt.0;
        let p = &self.params;

        // 1) 电机一阶响应：实际推力趋近指令推力
        for i in 0..4 {
            let target = cmd.motor[i] * p.thrust_coeff;
            let k = if p.motor_tau > 1e-6 { dt / p.motor_tau } else { 1.0 };
            self.motor_force[i] += (target - self.motor_force[i]) * k.clamp(0.0, 1.0);
        }

        // 2) 混控 -> 机体总推力(沿 -Z 机体) 与力矩
        //    电机布局 X 型：0=前右(CCW) 1=后左(CCW) 2=前左(CW) 3=后右(CW)
        //    约定（与控制器/姿态提取一致）：前右电机(0)更高 -> 右滚(+) + 上仰(+)。
        let f_total: f32 = self.motor_force.iter().sum();
        // 俯仰力矩：前(0,2)高 -> 机头上仰(+pitch，绕 +Y)
        let m_pitch = p.arm_length * (self.motor_force[0] + self.motor_force[2]
                                      - self.motor_force[1] - self.motor_force[3]);
        // 横滚力矩：右(0,3)高 -> 右滚(+roll，绕 +X)
        let m_roll = p.arm_length * (self.motor_force[0] + self.motor_force[3]
                                     - self.motor_force[1] - self.motor_force[2]);
        // 偏航力矩：CW/CCW 反扭差（绕 Z 轴）；CCW=(0,1) 反转矩 -> +yaw
        let m_yaw = p.torque_coeff * (self.motor_force[0] + self.motor_force[1]
                                     - self.motor_force[2] - self.motor_force[3]);

        // 机体推力向量（沿机体 -Z）
        let f_body = [0.0, 0.0, -f_total];

        // 3) 平移动力学：
        //    a_world = R * (f_body/m) + 重力(0,0,+g) - 气动阻力/m
        let r = self.state.att;
        let a_body = [f_body[0] / p.mass, f_body[1] / p.mass, f_body[2] / p.mass];
        let a_world = rotate_by_quat(r, a_body);
        let v = self.state.vel;
        // 气动阻力（机体速度平方，近似）：取世界速度平方的反向
        let speed2 = v[0].0 * v[0].0 + v[1].0 * v[1].0 + v[2].0 * v[2].0;
        let speed = speed2.sqrt();
        let drag = if speed > 1e-6 {
            let d = p.drag_coeff * speed2 / p.mass;
            [-v[0].0 / speed * d, -v[1].0 / speed * d, -v[2].0 / speed * d]
        } else { [0.0; 3] };

        let ax = a_world[0] + drag[0];
        let ay = a_world[1] + drag[1];
        let az = a_world[2] + p.gravity + drag[2];

        // 半隐式欧拉积分（位置用更新后速度）
        let vn = [
            MeterPerSecond(v[0].0 + ax * dt),
            MeterPerSecond(v[1].0 + ay * dt),
            MeterPerSecond(v[2].0 + az * dt),
        ];
        let pn = [
            Meter(self.state.pos[0].0 + vn[0].0 * dt),
            Meter(self.state.pos[1].0 + vn[1].0 * dt),
            Meter(self.state.pos[2].0 + vn[2].0 * dt),
        ];

        // 4) 旋转动力学：
        //    I * dw/dt = M_body - w × (I w)
        let w = self.state.omega;
        let m_body = [m_roll, m_pitch, m_yaw];
        let i = p.inertia;
        // 陀螺力矩 w × (I w)
        let iw_x = i[0] * w[0].0;
        let iw_y = i[1] * w[1].0;
        let iw_z = i[2] * w[2].0;
        let gyro_x = w[1].0 * iw_z - w[2].0 * iw_y;
        let gyro_y = w[2].0 * iw_x - w[0].0 * iw_z;
        let gyro_z = w[0].0 * iw_y - w[1].0 * iw_x;
        let dw = [
            (m_body[0] - gyro_x) / i[0],
            (m_body[1] - gyro_y) / i[1],
            (m_body[2] - gyro_z) / i[2],
        ];
        let wn = [
            RadianPerSecond(w[0].0 + dw[0] * dt),
            RadianPerSecond(w[1].0 + dw[1] * dt),
            RadianPerSecond(w[2].0 + dw[2] * dt),
        ];
        let attn = r.integrate(wn[0].0, wn[1].0, wn[2].0, dt);

        self.state = VehicleState { pos: pn, vel: vn, att: attn, omega: wn };

        // 5) 构造"理想 IMU"：机体加速度（含重力分量补偿后的比力）+ 角速度
        //    比力 f = a_world - g_world；再旋到机体
        let g_world = [0.0, 0.0, p.gravity];
        let f_world = [ax - g_world[0], ay - g_world[1], az - g_world[2]];
        let f_body_meas = rotate_by_quat_inverse(attn, f_world);
        ImuSample {
            accel: [MeterPerSecondSquared(f_body_meas[0]),
                    MeterPerSecondSquared(f_body_meas[1]),
                    MeterPerSecondSquared(f_body_meas[2])],
            gyro: wn,
        }
    }

    pub fn reset(&mut self) {
        self.state = VehicleState::zero();
        self.motor_force = [0.0; 4];
    }
}

/// 四元数旋转向量 q * v * q^-1（v 为纯四元数）。
fn rotate_by_quat(q: Quaternion, v: [f32; 3]) -> [f32; 3] {
    // 用旋转矩阵元素直接从四元数取（避免临时四元数分配）
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

fn rotate_by_quat_inverse(q: Quaternion, v: [f32; 3]) -> [f32; 3] {
    // 共轭旋转（机体->世界逆）
    let c = Quaternion { w: q.w, x: -q.x, y: -q.y, z: -q.z };
    rotate_by_quat(c, v)
}
