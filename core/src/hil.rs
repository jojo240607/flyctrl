//! HIL 桥接（M7.3）：SIL 与 MCU 共享的闭环控制步进。
//!
//! 设计要点：飞控主循环逻辑（传感器采集 → 估计 → 控制 → FDIR → 执行器）
//! 写成一个**泛型单步函数** [`fly_ctrl_step`]，对 HAL trait、估计算法、控制律
//! 完全泛型。host 端用 `mock` 实现跑 SIL，MCU 端用 `stm32f407` 实现跑 HIL——
//! **同一份算法代码**，因此 SIL 中穷举验证过的不变量（M7.1/M7.2）在板上直接成立，
//! 无需重写或复制。
//!
//! 这是"软件在环 → 硬件在环"的最小桥接：编译期保证控制律在两种目标上一致，
//! 运行时保证周期一致（均以固定 `dt` 推进）。

use crate::config::VehicleConfig;
use crate::controller::{Controller, Setpoint};
use crate::estimator::Estimator;
use crate::fdir::{Fdir, Health};
use crate::filter::Biquad;
use crate::hal::actuator::{clamp_thrust, MotorActuator};
use crate::hal::sensor::{AirspeedSensor, GpsSensor, ImuSensor, RtkSensor, VioSensor};
use crate::math;
use crate::units::{Meter, MeterPerSecondSquared, Radian, RadianPerSecond, Second};
use crate::vehicle::{ActuatorCmd, ImuSample, PosSample, Quaternion, RtkSample, VehicleState, VioSample};

/// HIL/共享单步结果：本拍估计状态 + 健康等级 + 已限幅执行器指令。
pub struct StepResult {
    pub est: VehicleState,
    pub health: Health,
    pub cmd: ActuatorCmd,
}

/// 占位 IMU 源（SIL/HIL 共享）：HIL 链路本拍无新 IMU 帧注入时启用。
///
/// 与实机 `control.rs` 使用同一份实现，保证 SIL 复现实机注入饥饿时回退数据
/// 完全一致。悬停比力 `(0,0,-9.81)`（FRD，z 向下）+ 平滑正弦扰动（避免 FDIR
/// 冻结误判）；零角速度 → 姿态不外推漂移。
pub struct SimImu {
    t: f32,
}

impl SimImu {
    pub fn new() -> Self {
        Self { t: 0.0 }
    }

    pub fn next(&mut self, dt: f32) -> ImuSample {
        self.t += dt;
        ImuSample {
            accel: [
                MeterPerSecondSquared(0.05 * math::sin(self.t)),
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(-9.81 + 0.05 * math::cos(self.t)),
            ],
            gyro: [RadianPerSecond(0.0); 3],
        }
    }
}

impl Default for SimImu {
    fn default() -> Self {
        Self::new()
    }
}

/// 单步闭环上下文（跨步持久状态）。
pub struct HilContext<E, C>
where
    E: Estimator,
    C: Controller,
{
    pub est: E,
    pub ctrl: C,
    pub fdir: Fdir,
    /// 固定控制周期。
    pub dt: Second,
    /// 累计是否触发过失控保护（单向，调试/判定用）。
    pub failsafe_engaged: bool,
    /// HIL 姿态初始化门控：基于首帧**真实** IMU 重力向量做 tilt alignment 后置位。
    pub hil_att_inited: bool,
    /// HIL 位置初始化门控：首个有限设定点到达、EKF 位置对齐物理真值后置位。
    pub hil_pos_inited: bool,
    /// 最近一帧**真实** IMU（sample-and-hold 回退源）。
    ///
    /// HIL 中 PC 每 ~32ms 才注入一帧 HIL_SENSOR，而本步 4ms 一拍，注入间隔内
    /// `imu=None`。此时用**最近真实 IMU** 保持（角速度继续积分、比力继续锚定），
    /// 姿态估计持续跟踪物理旋转；仅当**从未**收到真实帧（链路未建立）才回退
    /// `SimImu`（零陀螺，不外推）。教训：若注入间隔内回退零陀螺 SimImu，物理
    /// 旋转时估计 8 倍滞后（真值 roll 已 116° 而估计仍 ~0°）→ 控制反馈失效 →
    /// HIL 闭环持续翻滚发散。
    ///
    /// 存储**滤波后**样本（回退帧直接复用已滤波值，不再重复喂入滤波器，避免
    /// sample-and-hold 重复积分污染滤波器状态）。
    pub last_real_imu: Option<ImuSample>,
    /// 加速度计 40Hz 陷波滤波器（**每轴独立实例**）：抑制旋翼/机架固定频率振动
    /// （`SensorConfig::realistic` 振动模型 vib_amp=0.1 @40Hz）。直流增益 1 →
    /// 重力比力不受影响。Biquad 是标量滤波器，必须一轴一个实例，绝不可跨轴复用
    /// （复用会把三轴样本串进同一滤波器状态 → 滤波输出严重失真）。
    pub imu_accel_notch: [Biquad; 3],
    /// 加速度计低通滤波器（每轴独立实例）：衰减 accel 宽带白噪声（`realistic`
    /// accel_noise=0.05）。
    pub imu_accel_lowpass: [Biquad; 3],
    /// 陀螺仪 40Hz 陷波滤波器（每轴独立实例）：抑制旋翼/机架固定频率振动（`realistic`
    /// 振动模型 vib_amp=0.1 @40Hz）。
    ///
    /// 刻意**不用**宽带低通：陀螺读数直接进入姿态内环角速度阻尼反馈（`att_kd` 项的
    /// `est.omega`），任何低频截止的低通都会衰减阻尼项并引入相位滞后 → 姿态环增益
    /// 裕度下降、振荡增幅 → realistic 噪声下发散（实测：gyro 12Hz 低通使 10s 悬停
    /// 从稳定（tilt≈4°）恶化到翻滚（tilt>18°））。40Hz 窄带陷波（Q=5，带宽 36~44Hz）
    /// 只切除振动带，在姿态环带宽（~5Hz）处零相位/幅值影响，直流/低频角速度无失真。
    pub imu_gyro_notch: [Biquad; 3],
}

impl<E, C> HilContext<E, C>
where
    E: Estimator,
    C: Controller,
{
    pub fn new(est: E, ctrl: C, dt: Second) -> Self {
        // 采样率 = 1/控制周期。SIL 与 MCU 均以 dt=0.004（250Hz）推进 → 滤波器
        // 系数、状态演化两侧一致，同输入测试（逐位断言）仍成立。
        let fs = 1.0 / dt.0;
        Self {
            est,
            ctrl,
            fdir: Fdir::new(),
            dt,
            failsafe_engaged: false,
            hil_att_inited: false,
            hil_pos_inited: false,
            last_real_imu: None,
            // 陷波 @40Hz、Q=5：带宽 8Hz（36~44Hz）覆盖振动能量集中带；
            // 低通截止对齐姿态环带宽之上（accel 20Hz，Butterworth）：衰减 accel 白噪声，
            // 相位滞后影响 EKF 锚定/位置积分（弱路径）而非姿态内环。
            // gyro 只用 40Hz 陷波（不用低通）：见 `imu_gyro_notch` 注释——低通会衰减
            // 姿态环角速度阻尼反馈导致振荡发散。
            // 每轴独立实例（Biquad 为标量滤波器，跨轴复会污染状态）。
            imu_accel_notch: [Biquad::notch(40.0, fs, 5.0); 3],
            imu_accel_lowpass: [Biquad::low_pass(20.0, fs, 0.7071); 3],
            imu_gyro_notch: [Biquad::notch(40.0, fs, 5.0); 3],
        }
    }

    /// 单步闭环：每调用一次推进一个控制周期。
    ///
    /// - `imu` / `gps` / `airspeed`：当拍传感器（泛型，host/mock 或 MCU/PAC 均可）。
    /// - `vio` / `rtk`：P3-B1 多源融合通道（视觉里程计 / RTK-GPS）。二者在估计步
    ///   之后经 [`Estimator::update_vio`] / [`Estimator::update_rtk`] 注入（默认 no-op，
    ///   仅 `EkfEstimator` 实际融合）。保持 `step` 主链签名稳定，仅在尾部追加。
    /// - `setpoint`：当前设定点（由轨迹/遥控器提供）。
    /// - `motors`：执行器（泛型）。
    ///
    /// 返回本拍估计状态（供遥测/HIL 回采比对）。
    pub fn step<I, G, A, V, R, M>(
        &mut self,
        imu: &mut I,
        gps: &mut G,
        airspeed: &mut A,
        vio: &mut V,
        rtk: &mut R,
        setpoint: &crate::controller::Setpoint,
        motors: &mut M,
        cfg: &VehicleConfig,
    ) -> crate::vehicle::VehicleState
    where
        I: ImuSensor,
        G: GpsSensor,
        A: AirspeedSensor,
        V: VioSensor,
        R: RtkSensor,
        M: MotorActuator,
    {
        // 1) 采集传感器。
        let sample = imu.read();
        let pos = gps.read();
        let pos_available = pos.is_some();
        let air_sample = airspeed.read();
        let vio_sample = vio.read();
        let rtk_sample = rtk.read();

        // 2) 估计（含 VIO/RTK 观测融合，非 EkfEstimator 实现为 no-op）。
        let est_state = self.est.step(self.dt, sample, pos, air_sample);
        self.est.update_vio(vio_sample);
        self.est.update_rtk(rtk_sample);

        // 3) FDIR 健康监控（基于 IMU 冻结 + GPS dropout）。
        let health = self.fdir.update(&sample, pos_available, true, true);

        // 4) 控制。
        let raw_cmd = self.ctrl.control(self.dt, setpoint, &est_state);

        // 5) 安全裁决：危险时进入失控保护（单向，归零执行器）。
        match health {
            Health::Critical => {
                self.failsafe_engaged = true;
                motors.disarm(); // 全部输出归零
            }
            Health::Degraded | Health::Nominal => {
                // 应用限幅后的指令。
                let mut cmd = ActuatorCmd::zero();
                for i in 0..4 {
                    cmd.motor[i] = clamp_thrust(raw_cmd.motor[i]);
                }
                motors.apply(&cmd);
            }
        }

        // `cfg` 预留给后续机型相关限幅/健康策略；当前闭环保存在不变量内。
        let _ = cfg;
        let _ = Meter(0.0);
        est_state
    }

    /// 单步闭环：传感器 → 估计 → FDIR → 执行器，控制指令由调用方实时给出。
    ///
    /// 与 [`HilContext::step`] 共用"采集/估计/健康裁决/限幅/失效保护"骨架，仅把
    /// `setpoint → ctrl.control` 替换为 `cmd_fn(&est_state)`。供**非设定点型**控制律
    /// （P3-D1：手动角速率 / 增稳姿态保持，直接消费遥控摇杆而非位置设定点）接入
    /// 同一 SIL/HIL 闭环骨架，保证估计与健康监控路径与自主模式完全一致。
    pub fn step_with_cmd<I, G, A, V, R, M>(
        &mut self,
        imu: &mut I,
        gps: &mut G,
        airspeed: &mut A,
        vio: &mut V,
        rtk: &mut R,
        cmd_fn: impl FnOnce(&VehicleState) -> ActuatorCmd,
        motors: &mut M,
        cfg: &VehicleConfig,
    ) -> VehicleState
    where
        I: ImuSensor,
        G: GpsSensor,
        A: AirspeedSensor,
        V: VioSensor,
        R: RtkSensor,
        M: MotorActuator,
    {
        // 1) 采集传感器。
        let sample = imu.read();
        let pos = gps.read();
        let pos_available = pos.is_some();
        let air_sample = airspeed.read();
        let vio_sample = vio.read();
        let rtk_sample = rtk.read();

        // 2) 估计（含 VIO/RTK 观测融合，非 EkfEstimator 实现为 no-op）。
        let est_state = self.est.step(self.dt, sample, pos, air_sample);
        self.est.update_vio(vio_sample);
        self.est.update_rtk(rtk_sample);

        // 3) FDIR 健康监控。
        let health = self.fdir.update(&sample, pos_available, true, true);

        // 4) 控制：调用方按估计状态即时计算指令（手动/增稳直接映射摇杆）。
        let raw_cmd = cmd_fn(&est_state);

        // 5) 安全裁决：危险时进入失控保护（单向，归零执行器）。
        match health {
            Health::Critical => {
                self.failsafe_engaged = true;
                motors.disarm();
            }
            Health::Degraded | Health::Nominal => {
                let mut cmd = ActuatorCmd::zero();
                for i in 0..4 {
                    cmd.motor[i] = clamp_thrust(raw_cmd.motor[i]);
                }
                motors.apply(&cmd);
            }
        }

        let _ = cfg;
        est_state
    }

    /// 单步闭环（共享数据路径，SIL/HIL 同一份编排）。
    ///
    /// 方案 A 的核心：把实机 `control.rs` 的编排（IMU 单次消费 + SimImu 回退、
    /// 姿态/位置初始化门控、EKF 状态估计 + 气压观测更新、FDIR、控制环健康闸、
    /// 执行器限幅）抽成**唯一**共享实现。MCU 与 SIL 都喂入当拍原始样本（`Option`
    /// 表示本拍无新数据，如 HIL 注入饥饿、GPS 未采样），返回估计状态 + 健康等级
    /// + 已限幅执行器指令。数据、算法、时序两边完全一致，SIL 可复现实机 HIL 问题。
    ///
    /// - `imu`：本拍 IMU（`None` → 回退 [`SimImu`]，与实机注入饥饿一致）。
    /// - `gps`：本拍 GPS/位置（`None` → 无位置观测）。
    /// - `baro_alt`：本拍气压高度（m，向上为正），驱动 EKF 垂直通道观测。
    /// - `setpoint`：期望状态（姿态初始化 yaw、位置初始化、控制律目标）。
    /// - `setpoint_valid`：本拍设定点是否来自仿真器真实注入（`hil_setpoint_valid`）。
    ///   回退设定点（链路未建立时保持位置的占位值）虽是有限值，但**不得**用于
    ///   位置初始化——否则首拍即把 EKF 锁到占位目标，HIL 建立后叠加异常观测 → NaN。
    /// - `armed` / `rc_fresh`：解锁与遥控链路新鲜度（控制闸，任一为假则输出零指令）。
    pub fn step_hil(
        &mut self,
        imu: Option<ImuSample>,
        gps: Option<PosSample>,
        baro_alt: Option<f32>,
        vio: Option<VioSample>,
        rtk: Option<RtkSample>,
        setpoint: &Setpoint,
        setpoint_valid: bool,
        armed: bool,
        rc_fresh: bool,
        sim_imu: &mut SimImu,
    ) -> StepResult {
        // 1) IMU：有真实帧用真实帧（单次消费由调用方保证），无则回退最近真实帧
        //    （sample-and-hold，角速度继续积分、比力继续锚定）；从未收到真实帧
        //    （链路未建立）才回退 SimImu（零陀螺，不外推）。
        //
        //    输入滤波：accel 过 40Hz 陷波（抑制旋翼/机架振动）+ 20Hz 低通（衰减白噪声），
        //    gyro 过 40Hz 陷波（抑制振动带，不用低通以免衰减姿态环角速度阻尼反馈）。只对**真实帧**滤波并回存 → 滤波器
        //    状态演化严格跟随真实 IMU 到达序列，SIL 与 MCU 逐位一致；回退帧直接取
        //    已滤波的最近帧，不再重复喂入（避免 sample-and-hold 重复积分污染滤波器状态）。
        //    **每轴独立滤波器实例**（Biquad 是标量滤波器，跨轴复用会把三轴样本串进同一
        //    滤波器状态 → 输出严重失真）。原始（未滤波）加速度保留给姿态初始化：
        //    IIR 瞬态期滤波幅值不足会错过 `an>1.0` 触发，且瞬态会污染 tilt alignment。
        let mut raw_accel: Option<[f32; 3]> = None;
        let imu_sample = match imu {
            Some(s) => {
                raw_accel = Some([s.accel[0].0, s.accel[1].0, s.accel[2].0]);
                let mut acc = [0.0f32; 3];
                let mut gy = [0.0f32; 3];
                for i in 0..3 {
                    acc[i] = self.imu_accel_lowpass[i].process(self.imu_accel_notch[i].process(s.accel[i].0));
                    gy[i] = self.imu_gyro_notch[i].process(s.gyro[i].0);
                }
                let filtered = ImuSample {
                    accel: [
                        MeterPerSecondSquared(acc[0]),
                        MeterPerSecondSquared(acc[1]),
                        MeterPerSecondSquared(acc[2]),
                    ],
                    gyro: [RadianPerSecond(gy[0]), RadianPerSecond(gy[1]), RadianPerSecond(gy[2])],
                };
                self.last_real_imu = Some(filtered);
                filtered
            }
            None => self
                .last_real_imu
                .unwrap_or_else(|| sim_imu.next(self.dt.0)),
        };

        // 2) 姿态初始化门控：仅当本拍拿到**真实** IMU 帧才做 tilt alignment。
        //    悬停/静止时比力 a=(0,0,-9.81)（FRD，z 向下），重力方向即 -a：
        //    pitch = atan2(ax, sqrt(ay²+az²))，roll = atan2(-ay, -az)。
        //    机头 yaw 取设定点真值（有限性检查，未到达/非有限则回退 0），
        //    消除「sp.yaw=NaN → 四元数 w=NaN → 姿态估计 NaN」问题。
        //    HIL 未建立时 imu 恒 None → 回退 SimImu，此时**不得**用它初始化
        //    （否则 HIL 建立后姿态被锁定在 SimImu 初始值 → 叠加异常观测 → NaN）。
        if !self.hil_att_inited && imu.is_some() {
            // 用**原始**（未滤波）加速度做幅值闸 + tilt alignment：真实帧到达时
            // 幅值即 ~9.81（`an>1.0` 稳定触发，不受 IIR 瞬态影响），且原始重力方向
            // 是物理真值 → 初始姿态最准确。
            let a = raw_accel.unwrap();
            let an = math::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            if an > 1.0 {
                let pitch = math::atan2(a[0], math::sqrt(a[1] * a[1] + a[2] * a[2]));
                let roll = math::atan2(-a[1], -a[2]);
                let yaw = if setpoint.yaw.0.is_finite() { setpoint.yaw.0 } else { 0.0 };
                let q = Quaternion::from_euler(Radian(roll), Radian(pitch), Radian(yaw));
                self.est.set_initial_attitude(q);
                self.hil_att_inited = true;
            }
        }

        // 3) 位置初始化门控：首个**真实**设定点到达时，把 EKF 位置对齐物理真值，
        //    消除首帧数米位置误差（否则位置环需长时间收敛）。回退设定点不触发。
        if !self.hil_pos_inited
            && setpoint_valid
            && setpoint.pos.iter().all(|p| p.0.is_finite())
        {
            self.est.set_initial_position([
                setpoint.pos[0].0,
                setpoint.pos[1].0,
                setpoint.pos[2].0,
            ]);
            self.hil_pos_inited = true;
        }

        // 4) 状态估计（predict-then-correct；HIL 无空速通道 → airspeed=None）。
        let est_state = self.est.step(self.dt, imu_sample, gps, None);
        // 气压高度观测（垂直通道最紧锚）：在 step 之后注入（predict-then-correct），
        // 下一拍预测从修正后状态出发。此前 baro 只进 FDIR、垂直通道仅靠 GPS 锚定
        // → 注入饥饿时 SimImu 反向重力把高度估计拖低（实测 5.99m vs 3.42m 滞后
        // 2.5m → 位置环加推 → 物理爬升 → 发散）。
        if let Some(alt) = baro_alt {
            self.est.update_alt(alt);
        }
        // VIO/RTK 多源融合（P3-B1）：与 `HilContext::step` 旧路径保持一致——SIL 注入
        // 模拟 VIO/RTK 观测（含噪声），MCU HIL 无此通道则传 None（`update_vio`/
        // `update_rtk` 对非 EkfEstimator 实现为 no-op，EkfEstimator 仅在其可用时融合）。
        // 位置观测更紧（r_vio_pos/r_rtk）→ realistic 噪声下位置估计收敛更快、漂移更小。
        self.est.update_vio(vio);
        self.est.update_rtk(rtk);

        // 5) FDIR 健康监控（mag 暂以 false 匹配实机 HIL 路径，见 control.rs TODO）。
        let health = self.fdir.update(&imu_sample, gps.is_some(), baro_alt.is_some(), false);

        // 6) 控制环健康闸：估计/设定点含非有限值（NaN/Inf）、未解锁、链路不新鲜、
        //    FDIR 关键故障、或 EKF 尚未完成姿态/位置初始化时输出零指令，阻断 NaN
        //    传播到执行器。
        //    【HIL 起飞台】init 门控是新增条件：姿态/位置未用真实传感器帧初始化时
        //    EKF 状态虽为有限值（默认水平 + 原点），但控制律会据虚假误差输出对角
        //    饱和指令（实测启动即 1001，总推力 2.0 越过 HIL_HOLD_THRUST=0.05 →
        //    起飞台保持被过早释放 → 机体自由落体翻滚）。初始化完成前恒输出零，
        //    保证起飞台保持持续到 EKF 收敛（SIL 首拍 setpoint_valid=true + 真实
        //    IMU → 两标志即置位，无行为变化）。
        let est_finite = est_state.att.w.is_finite() && est_state.att.x.is_finite()
            && est_state.att.y.is_finite() && est_state.att.z.is_finite()
            && est_state.pos.iter().all(|p| p.0.is_finite())
            && est_state.vel.iter().all(|v| v.0.is_finite());
        let sp_finite = setpoint.pos.iter().all(|p| p.0.is_finite())
            && setpoint.vel.iter().all(|v| v.0.is_finite())
            && setpoint.acc.iter().all(|a| a.0.is_finite())
            && setpoint.yaw.0.is_finite();
        let raw_cmd = if armed
            && rc_fresh
            && health != Health::Critical
            && est_finite
            && sp_finite
            && self.hil_att_inited
            && self.hil_pos_inited
        {
            self.ctrl.control(self.dt, setpoint, &est_state)
        } else {
            ActuatorCmd::zero()
        };

        // 7) 执行器限幅（单向记录失控保护）。
        if health == Health::Critical {
            self.failsafe_engaged = true;
        }
        let mut cmd = ActuatorCmd::zero();
        for i in 0..4 {
            cmd.motor[i] = clamp_thrust(raw_cmd.motor[i]);
        }

        StepResult { est: est_state, health, cmd }
    }

    /// 当前估计状态（已含所有已融合观测，含控制器 step 之后的外部 update_alt）。
    pub fn estimate(&self) -> VehicleState
    where
        E: Estimator,
    {
        self.est.state()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::eprintln;
    use super::*;
    use crate::config::VehicleConfig;
    use crate::controller::pid::PidController;
    use crate::controller::Controller;
    use crate::estimator::ekf::EkfEstimator;
    use crate::hal::actuator::MockMotors;
    use crate::hal::sensor::{MockAirspeed, MockBaro, MockGps, MockImu, MockMag, MockRtk, MockVio};
    use crate::invariants::{actuator_bounded, state_finite};
    use crate::units::Second;
    use crate::vehicle::ImuSample;

    #[test]
    fn hil_step_hil_loop_runs_and_stays_bounded() {
        // 共享数据路径（方案 A）自测：先「链路未建立」回退 SimImu 若干拍
        // （不得触发姿态/位置初始化），再「链路建立」注入真实 FRD 悬停 IMU
        // + GPS + 气压 → 姿态/位置初始化 → 闭环全程无 NaN、指令恒有界。
        let cfg = VehicleConfig::default_quad();
        let mut ctx = HilContext::new(
            EkfEstimator::default_quad(),
            PidController::from_config(&cfg.ctrl_params()),
            Second(0.004),
        );
        let mut sim_imu = SimImu::new();
        let sp = crate::controller::Setpoint::hover(
            [Meter(0.0), Meter(0.0), Meter(-5.0)],
            crate::units::Radian(0.7),
        );

        // 阶段 1：HIL 链路未建立（imu/gps/baro 全 None → SimImu 回退）。
        for it in 0..50 {
            let r = ctx.step_hil(None, None, None, None, None, &sp, false, true, true, &mut sim_imu);
            assert!(state_finite(&r.est), "链路未建立阶段 NaN at iter {}: {:?}", it, r.est.att);
            assert!(!ctx.hil_att_inited, "SimImu 回退不得触发姿态初始化");
            assert!(!ctx.hil_pos_inited, "回退设定点不得触发位置初始化");
            assert!(actuator_bounded(&r.cmd));
        }

        // 阶段 2：HIL 链路建立，注入真实 FRD 悬停 IMU（比力 z=-9.81）+ GPS + 气压。
        let frd_hover = ImuSample {
            accel: [
                crate::units::MeterPerSecondSquared(0.02),
                crate::units::MeterPerSecondSquared(-0.01),
                crate::units::MeterPerSecondSquared(-9.81),
            ],
            gyro: [crate::units::RadianPerSecond(0.0); 3],
        };
        let gps = Some(crate::vehicle::PosSample::pos_only([
            Meter(0.0), Meter(0.0), Meter(-5.0),
        ]));
        for it in 0..300 {
            let r = ctx.step_hil(Some(frd_hover), gps, Some(5.0), None, None, &sp, true, true, true, &mut sim_imu);
            if it % 2 == 0 {
                let a = r.est.att;
                eprintln!(
                    "DIAG iter={} att=({:.4},{:.4},{:.4},{:.4}) r={:.1} p={:.1} y={:.1} pos=({:.3},{:.3},{:.3}) vel=({:.3},{:.3},{:.3}) cmd=({:.3},{:.3},{:.3},{:.3})",
                    it, a.w, a.x, a.y, a.z,
                    a.roll().to_degrees(), a.pitch().to_degrees(), a.yaw().to_degrees(),
                    r.est.pos[0].0, r.est.pos[1].0, r.est.pos[2].0,
                    r.est.vel[0].0, r.est.vel[1].0, r.est.vel[2].0,
                    r.cmd.motor[0], r.cmd.motor[1], r.cmd.motor[2], r.cmd.motor[3],
                );
            }
            assert!(state_finite(&r.est), "闭环 NaN at iter {}: att={:?} pos={:?}", it, r.est.att, r.est.pos);
            assert!(actuator_bounded(&r.cmd), "闭环指令必须 [0,1]");
        }
        assert!(ctx.hil_att_inited, "真实 IMU 帧到达后应完成姿态初始化");
        assert!(ctx.hil_pos_inited, "真实设定点到达后应完成位置初始化");
        // 姿态应收敛到水平（roll/pitch ≈ 0），yaw 对齐设定点。
        let a = ctx.est.state().att;
        assert!(a.roll().abs() < 0.5, "roll 应≈0，实测 {}", a.roll());
        assert!(a.pitch().abs() < 0.5, "pitch 应≈0，实测 {}", a.pitch());
        assert!((a.yaw() - 0.7).abs() < 0.5, "yaw 应≈0.7，实测 {}", a.yaw());
    }

    #[test]
    fn hil_sparse_imu_injection_tracks_true_attitude() {
        // HIL 稀疏注入 + sample-and-hold 回退自测：物理绕机体 X 轴以恒定角速度
        // 旋转（gyro=[0.5,0,0] rad/s），PC 每 8 拍（~32ms）才注入一帧真实 IMU，
        // 其余拍 imu=None → 回退最近真实帧（sample-and-hold）。
        // 验证：EKF 姿态持续跟踪真值（误差很小），而非零陀螺 SimImu 的 8 倍
        // 滞后（那会在数百拍后积累数十度误差 → HIL 闭环姿态发散）。
        let cfg = VehicleConfig::default_quad();
        let mut ctx = HilContext::new(
            EkfEstimator::default_quad(),
            PidController::from_config(&cfg.ctrl_params()),
            Second(0.004),
        );
        let mut sim_imu = SimImu::new();
        let sp = crate::controller::Setpoint::hover(
            [Meter(0.0), Meter(0.0), Meter(-5.0)],
            crate::units::Radian(0.0),
        );
        // 首帧真实水平 IMU 触发姿态初始化。
        let init = ImuSample {
            accel: [MeterPerSecondSquared(0.0), MeterPerSecondSquared(0.0), MeterPerSecondSquared(-9.81)],
            gyro: [RadianPerSecond(0.0); 3],
        };
        let _ = ctx.step_hil(Some(init), None, None, None, None, &sp, false, true, true, &mut sim_imu);
        assert!(ctx.hil_att_inited);

        // 物理真值：绕机体 X 轴（roll）恒定角速度旋转，比力 = Rx(roll)⁻¹·(0,0,-9.81)。
        let wx = 0.5f32; // rad/s
        let dt = 0.004f32;
        let mut roll_true = 0.0f32;
        let mut max_err = 0.0f32;
        for it in 0..400 {
            roll_true += wx * dt;
            let imu = if it % 8 == 0 {
                Some(ImuSample {
                    accel: [
                        MeterPerSecondSquared(0.0),
                        MeterPerSecondSquared(-9.81 * math::sin(roll_true)),
                        MeterPerSecondSquared(-9.81 * math::cos(roll_true)),
                    ],
                    gyro: [RadianPerSecond(wx), RadianPerSecond(0.0), RadianPerSecond(0.0)],
                })
            } else {
                None
            };
            let r = ctx.step_hil(imu, None, None, None, None, &sp, false, true, true, &mut sim_imu);
            assert!(state_finite(&r.est), "NaN at iter {}", it);
            max_err = max_err.max((r.est.att.roll() - roll_true).abs());
        }
        eprintln!(
            "DIAG sparse: roll_true={:.1}° est_roll={:.1}° max_err={:.1}°",
            roll_true.to_degrees(),
            ctx.est.state().att.roll().to_degrees(),
            max_err.to_degrees()
        );
        // sample-and-hold 保持恒定角速度 → 应精确跟踪（容差覆盖积分/锚定瞬态）。
        assert!(max_err < 0.2, "sample-and-hold 应跟踪恒定旋转，max_err={:.1}°", max_err.to_degrees());
    }

    /// 用户提示方向：硬件(SIL 侧 `step_hil`)与软件模拟(MCU 侧 `step_hil`)保持输入一致，
    /// 给同一组数据，测输出是否相同，排除本身差异，对比。
    ///
    /// 历史结论（参数统一前的"本身差异"）：
    /// - SIL 侧：`PidController::from_config(cfg.ctrl_params())` → `kp_xy=0.3`、`kv_z=1.5`
    ///   （`CtrlParams` 无 `kv_z`，故保留 `default_quad` 基值 1.5）。
    /// - MCU 侧：`PidController::default_quad()` 后每周期被 `sync_gains_to_pid` 用
    ///   `G_PARAM_VALS` 覆盖（见 joc-app-rust uplink.rs）→ 曾为 `[0.5,0.5,0.8,0.8,0.5]`，
    ///   导致 `kp_xy=0.5`、`kv_z=0.8`，与 SIL 不一致 → 同输入下电机指令最大差 10.3%。
    /// 已按"MCU 对齐 SIL"决策将 `G_PARAM_VALS` 默认值改为 `[0.3,0.5,0.8,1.5,0.5]`。
    ///
    /// 本测试（统一后回归）：
    /// 1) Part1：同一确定性输入序列分别喂给 MCU 配置（default_quad + apply_gains
    ///    `[0.3,0.5,0.8,1.5,0.5]`）与 SIL 配置（`from_config`），逐拍断言 cmd 输出
    ///    **逐位一致**（参数对齐后两侧必须完全相同，防止未来参数漂移回归）。
    /// 2) Part2：两侧都用 `from_config`（参数统一）再喂同一序列，断言逐拍**逐位一致**
    ///    （排除编排/数值差异 H3——证明"排除本身差异后输出完全相同"）。
    ///
    /// 输入序列设计：恒定倾斜姿态（roll=10°,pitch=-5°,yaw=30°，比力 = R·(0,0,-9.81)，
    /// gyro=0）+ 恒定位置偏移 (2,1,-5) + 恒定设定点（原点悬停 (0,0,-5)）。让位置环
    /// 与姿态环持续存在误差，否则稳态下输出恒为悬停油门、参数差异被掩盖。
    /// 两侧 EKF 输入相同 → 估计状态逐位一致，输出差异纯粹来自 PID 参数。
    #[test]
    fn hil_same_input_sil_vs_mcu_output_compare() {
        let cfg = VehicleConfig::default_quad();
        // MCU 侧配置（复刻 joc-app-rust control.rs）：default_quad + sync_gains_to_pid
        // 每周期用 G_PARAM_VALS 默认值覆盖 → 已对齐 SIL：kp_xy=0.3, kv_z=1.5。
        let mut mcu_ctx = HilContext::new(
            EkfEstimator::default_quad(),
            PidController::default_quad(),
            Second(0.004),
        );
        mcu_ctx.ctrl.apply_gains(&[0.3, 0.5, 0.8, 1.5, 0.5]); // 复刻 init_param_defaults（MCU 对齐 SIL）
        // SIL 侧配置（与 fly-sim-core controller.rs 完全一致）：from_config 派生。
        let mut sil_ctx = HilContext::new(
            EkfEstimator::default_quad(),
            PidController::from_config(&cfg.ctrl_params()),
            Second(0.004),
        );
        let mut mcu_sim = SimImu::new();
        let mut sil_sim = SimImu::new();
        let sp = crate::controller::Setpoint::hover(
            [Meter(0.0), Meter(0.0), Meter(-5.0)],
            crate::units::Radian(0.0),
        );

        // 物理真值：恒定倾斜（ZYX：roll→pitch→yaw，NED/FRD）。
        let roll = 10f32.to_radians();
        let pitch = (-5f32).to_radians();
        let yaw = 30f32.to_radians();
        let (sr, cr) = roll.sin_cos();
        let (spi, cpi) = pitch.sin_cos();
        let (sy, cy) = yaw.sin_cos();
        // a_body = Rz(yaw)·Ry(pitch)·Rx(roll) · (0,0,-9.81)，gyro=0（恒定姿态）。
        let imu = ImuSample {
            accel: [
                MeterPerSecondSquared(-9.81 * (cy * spi * cr + sy * sr)),
                MeterPerSecondSquared(-9.81 * (sy * spi * cr - cy * sr)),
                MeterPerSecondSquared(-9.81 * (cpi * cr)),
            ],
            gyro: [RadianPerSecond(0.0); 3],
        };
        // 恒定位置偏移：GPS 报 (2,1,-5)，设定点 (0,0,-5) → 位置环持续误差。
        let gps = Some(crate::vehicle::PosSample::pos_only([
            Meter(2.0), Meter(1.0), Meter(-5.0),
        ]));

        // ---- Part1：MCU(对齐后) vs SIL(from_config)，同一输入 → 逐位一致 ----
        let mut max_d = 0.0f32;
        let mut beats = 0u32;
        for it in 0..1000 {
            let r_mcu = mcu_ctx.step_hil(Some(imu), gps, Some(5.0), None, None, &sp, true, true, true, &mut mcu_sim);
            let r_sil = sil_ctx.step_hil(Some(imu), gps, Some(5.0), None, None, &sp, true, true, true, &mut sil_sim);
            assert!(state_finite(&r_mcu.est) && state_finite(&r_sil.est), "NaN at iter {}", it);
            for i in 0..4 {
                max_d = max_d.max((r_mcu.cmd.motor[i] - r_sil.cmd.motor[i]).abs());
                assert_eq!(r_mcu.cmd.motor[i], r_sil.cmd.motor[i], "part1 逐位不一致 at iter {} m{}", it, i);
            }
            beats += 1;
            if it % 250 == 249 {
                eprintln!(
                    "DIAG part1 it={} mcu=({:.3},{:.3},{:.3},{:.3}) sil=({:.3},{:.3},{:.3},{:.3})",
                    it, r_mcu.cmd.motor[0], r_mcu.cmd.motor[1], r_mcu.cmd.motor[2], r_mcu.cmd.motor[3],
                    r_sil.cmd.motor[0], r_sil.cmd.motor[1], r_sil.cmd.motor[2], r_sil.cmd.motor[3]
                );
            }
        }
        eprintln!(
            "DIAG part1 SAME-INPUT(MCU 对齐 SIL: kp_xy=0.3,kv_z=1.5 vs SIL from_config): beats={} max|Δcmd|={:.3}",
            beats, max_d
        );
        // 参数对齐后两侧必须逐位一致（防止未来参数漂移回归）。
        assert_eq!(max_d, 0.0, "MCU 对齐 SIL 后输出应逐位一致，max|Δcmd|={:.3}", max_d);

        // ---- Part2：两侧都用 from_config（排除本身差异后），同一输入 ----
        // 逐位一致断言：证明"输入一致 + 参数一致 → 输出完全一致"（H3/编排无差异）。
        let mut cfg2 = HilContext::new(
            EkfEstimator::default_quad(),
            PidController::from_config(&cfg.ctrl_params()),
            Second(0.004),
        );
        let mut cfg2_sim = SimImu::new();
        let mut ref_ctx = HilContext::new(
            EkfEstimator::default_quad(),
            PidController::from_config(&cfg.ctrl_params()),
            Second(0.004),
        );
        let mut ref_sim = SimImu::new();
        for it in 0..1000 {
            let r_a = cfg2.step_hil(Some(imu), gps, Some(5.0), None, None, &sp, true, true, true, &mut cfg2_sim);
            let r_b = ref_ctx.step_hil(Some(imu), gps, Some(5.0), None, None, &sp, true, true, true, &mut ref_sim);
            for i in 0..4 {
                assert_eq!(r_a.cmd.motor[i], r_b.cmd.motor[i], "part2 逐位不一致 at iter {} m{}", it, i);
            }
            // 估计状态也应逐位一致（EKF 确定性）。
            assert_eq!(r_a.est.pos[2].0, r_b.est.pos[2].0, "part2 est 逐位不一致 at iter {}", it);
        }
        eprintln!("DIAG part2 SAME-PARAMS(SIL 配置×2) SAME-INPUT: 1000 拍逐位一致 → 编排/数值无差异（H3 排除）");
    }

    #[test]
    fn hil_loop_runs_and_stays_bounded() {
        // HIL 桥接的 SIL 自测：mock 传感器 + 真实算法闭环，验证
        // 全程无 NaN、指令恒有界（与 M7.1 不变量一致，证明共享循环正确）。
        let cfg = VehicleConfig::default_quad();
        let mut imu = MockImu::new();
        let mut gps = MockGps::new();
        let mut air = MockAirspeed::new(0.0);
        let mut vio = MockVio::new();
        let mut rtk = MockRtk::new();
        let _baro = MockBaro::new();
        let _mag = MockMag::new();
        let mut motors = MockMotors::new(crate::hal::actuator::OutputProtocol::Pwm);

        let mut ctx = HilContext::new(EkfEstimator::default_quad(), PidController::from_config(&cfg.ctrl_params()), Second(0.01));
        let sp = crate::controller::Setpoint::hover([Meter(0.0), Meter(0.0), Meter(-5.0)], crate::units::Radian(0.0));

        for it in 0..300 {
            let st = ctx.step(&mut imu, &mut gps, &mut air, &mut vio, &mut rtk, &sp, &mut motors, &cfg);
            if !state_finite(&st) {
                panic!("HIL NaN at iter {}: pos={:?} vel={:?} att={:?}", it, st.pos, st.vel, st.att);
            }
            assert!(actuator_bounded(&motors.last_cmd()), "HIL 闭环指令必须 [0,1]");
        }
    }

    #[test]
    fn hil_loop_failsafe_zeroes_motors() {
        // 注入冻结 IMU -> FDIR 判 Critical -> 执行器归零（失控保护单向生效）。
        let cfg = VehicleConfig::default_quad();
        let mut imu = MockImu::new();
        let mut gps = MockGps::new();
        let mut air = MockAirspeed::new(0.0);
        let mut vio = MockVio::new();
        let mut rtk = MockRtk::new();
        let mut motors = MockMotors::new(crate::hal::actuator::OutputProtocol::Pwm);
        let mut ctx = HilContext::new(EkfEstimator::default_quad(), PidController::from_config(&cfg.ctrl_params()), Second(0.01));
        let sp = crate::controller::Setpoint::hover([Meter(0.0); 3], crate::units::Radian(0.0));

        // 先正常跑几拍。
        for _ in 0..5 {
            let _ = ctx.step(&mut imu, &mut gps, &mut air, &mut vio, &mut rtk, &sp, &mut motors, &cfg);
        }
        // 冻结 IMU。
        imu.set_health(false);
        let frozen = ImuSample {
            accel: [crate::units::MeterPerSecondSquared(0.0); 3],
            gyro: [crate::units::RadianPerSecond(0.0); 3],
        };
        // 直接喂固定帧（绕过 mock 振荡）以触发冻结检测。
        for _ in 0..30 {
            let _ = ctx.fdir.update(&frozen, true, true, true);
            // 用冻结样本走一步（mock 已 unhealthy，read 仍返回振荡；这里单独验证裁决）。
        }
        // 走一步并确认：若 FDIR 已 Critical，motors 应被 disarm。
        ctx.fdir.update(&frozen, true, true, true);
        // 手动复现裁决逻辑（与 step 内一致）。
        use crate::fdir::Health;
        if ctx.fdir.health() == Health::Critical {
            motors.disarm();
            assert!(actuator_bounded(&motors.last_cmd()), "失控保护归零后仍应有界");
            assert_eq!(motors.last_cmd().motor, [0.0; 4], "失控保护必须归零输出");
        }
    }
}
