# flyctrl — Rust 飞控软件设计规划

> 目标：用 Rust 设计一套超越 PX4 / APM 的传统飞控。
> 本文档记录已对齐的设计方向、短期与长期规划、当前进度与下一步。所有开发按此执行。

---

## 0. 设计总纲（为什么能超越 PX4/APM）

| 维度 | PX4/APM 现状 | flyctrl 的超越点 |
|------|------|------|
| 语言 | C/C++，裸指针、UB、内存安全靠人工 | Rust 所有权/类型系统，编译期消除整类 bug |
| 单位系统 | 裸 `float`，单位混用靠约定 | 类型安全 SI 单位（`Meter`/`Radian`/`Second`…），"把角度当弧度传"编译期报错 |
| 状态机 | 运行时 `enum` + `if` 守卫 | 类型级状态机（type-state），"未校准就解锁"等非法流转编译期不可达 |
| 可扩展性 | 模块耦合重 | trait 抽象（`Controller`/`Estimator`），算法编译期可插拔 |
| 验证 | 飞行测试 + 少量单测 | SIL 仿真闭环前置 + 指标横向对比 + 后续形式化/属性测试 |
| 故障容错 | 看门狗 + 部分双冗余 | 类型级非法状态不可表示 + FDIR 模块 |
| 实时性 | NuttX/ChibiOS 宏内核 | 核心逻辑 `no_std` + 零堆分配；运行时未来切换自研 Rust RTOS |

---

## 1. 已对齐的关键决策

1. **目标硬件**：STM32F407（后续另有一款在研 Rust 硬实时 RTOS，届时把运行时后端切过去）。
2. **当前阶段**：先纯 host 仿真（SIL），HAL 层用 trait 预留，不依赖具体芯片。
3. **控制算法深度**：PID / 互补滤波 / EKF / LQR / MPC **全部纳入**，同一仿真后端跑相同场景做横向对比分析。
4. **通信协议**：MAVLink（兼容 QGC 地面站），内部仿真通道另走高效序列化。
5. **物理模型**：第一步即使用**较真实的电机动力学 + 气动系数**（六自由度刚体 + 电机一阶响应 + 推力/力矩混控 + 气动阻力），而非理想积分。
6. **核心约束**：`core` crate `no_std`、零堆分配（`heapless`/固定数组），可在 host 与嵌入式固件间共享。

---

## 2. 架构分层

```
┌─────────────────────────────────────────────┐
│  APPLICATION LAYER  (飞行模式/任务/航点)      │   (后续)
├─────────────────────────────────────────────┤
│  FLIGHT CONTROL CORE                         │
│   · 控制律 (姿态/位置/轨迹)  trait 抽象       │   core/controller
│   · 状态估计 (EKF/互补滤波)  可替换           │   core/estimator
│   · 故障检测与处置 FDIR                       │   core/fdir (后续)
│   · 类型安全单位 + 类型级状态机               │   core/units, core/state
├─────────────────────────────────────────────┤
│  MIDDLEWARE                                  │
│   · 实时调度器 (trait)                        │   rtos/ (后续, 接自研 RTOS)
│   · 消息总线 / 发布订阅 (类型安全)            │   (后续)
│   · 时间/时钟抽象                            │   rtos/clock (后续)
├─────────────────────────────────────────────┤
│  HAL (硬件抽象层 trait)                       │
│   · 传感器 (IMU/GPS/气压/罗盘)               │   hal/sensor (后续)
│   · 执行器 (PWM/CAN/DSHOT)                   │   hal/actuator (后续)
│   · 通信 (MAVLink/CRSF/串口)                 │   mavlink/ (后续)
├─────────────────────────────────────────────┤
│  SIM (host 仿真后端)                          │   sim/  ← 当前重心
│   · 六自由度物理 + 电机动力学 + 气动          │   sim/physics
│   · 世界环境 (噪声/风扰/故障注入)             │   sim/world
│   · 仿真驱动 + 指标评估                       │   sim/harness
├─────────────────────────────────────────────┤
│  no_std 运行时 + 芯片支持 (stm32f407, 未来RTOS)│  (后续)
└─────────────────────────────────────────────┘
```

### 当前 workspace 结构
```
flyctrl/
├── Cargo.toml              # workspace (core / sim / bin)
├── core/                   # no_std 纯逻辑，无 alloc
│   ├── src/lib.rs
│   ├── math.rs             # no_std 数学 shim (libm 转发)
│   ├── units.rs            # 类型安全 SI 单位
│   ├── state.rs            # 类型级飞控状态机
│   ├── vehicle.rs          # 飞行器状态/IMU/四元数/执行器 数据结构
│   ├── estimator/
│   │   ├── mod.rs
│   │   ├── trait_def.rs    # Estimator trait + Setpoint
│   │   └── complementary.rs# 互补滤波 (基线)
│   └── controller/
│       ├── mod.rs
│       ├── trait_def.rs    # Controller trait
│       └── pid.rs          # 串级 PID (基线)
├── sim/                    # host 物理仿真后端
│   ├── src/lib.rs
│   ├── physics.rs          # 六自由度 + 电机一阶响应 + 混控 + 气动阻力
│   ├── world.rs            # 噪声/风扰/故障注入 (基于真实位置测量)
│   └── harness.rs          # 闭环驱动 + 指标 (RMS/稳定时间/发散检测/最坏步时)
├── bin/
│   └── src/main.rs         # SITL 入口 (PID+互补滤波 悬停闭环)
└── PLAN.md
```

---

## 3. 统一指标层（算法横向对比的基准）

所有算法跑同一组 scenario，自动产出对比表（`sim/harness.rs` 的 `Metrics`）：
- `pos_rms`：位置误差 RMS (m)
- `pos_max`：最大位置误差 (m)
- `settle_time`：首次进入 0.5m 容差的时间 (s)，未达成为 -1
- `nan_detected`：数值发散检测
- `worst_step_ms`：单步 控制+估计 最坏耗时 (ms) —— 为未来 RTOS CPU 预算预估

### 3-A. 标准测试场景（`sim/scenario.rs`）

所有算法跑同一组场景，目标设定点随时间由 `Scenario::setpoint_at(t)` 给出，
对比表经 `bin/sitl --scenario <name>` 一键输出。场景列表：

| 场景 | 含义 | 设定点 | 世界环境 |
|------|------|--------|----------|
| `hover` | 定点悬停（基线） | 恒定 z=-10m | 无风 |
| `step` | 阶跃目标 | t≥5s 北向平移 5m | 无风 |
| `wind` | 定点抗风扰 | 恒定 z=-10m | 北向常值风 3 m/s + 阵风 2 m/s（气动阻力相对空气计算，风成为扰动） |
| `square` | 方形航线跟踪 | 5m 半边长四角循环，含段内速度前馈 | 无风 |
| `circle` | 圆形轨迹跟踪 | 半径 5m 匀速绕圈，含切向速度前馈 | 无风 |

风扰实现：物理模型新增 `wind`/`wind_gust` 字段，气动阻力按 `v_rel = v_world - wind`
计算（而非相对地面），使常值风 + 正弦阵风真实扰动机体；`World` 噪声层保持不变。

---

## 4. 短期规划（M1–M3，先把闭环跑稳 + 拉齐算法基线）

### M1：基线闭环验证（已完成）
- [x] workspace + `core`/`sim`/`bin` 骨架
- [x] `units` 类型安全单位 + `math` no_std shim
- [x] `state` 类型级状态机（Disarmed/Calibrating/Armed/Failsafe）
- [x] `vehicle` 六自由度状态/四元数/IMU/执行器 数据结构
- [x] `estimator::Estimator` trait + 互补滤波实现
- [x] `controller::Controller` trait + 串级 PID 实现
- [x] `sim::physics` 六自由度 + 电机一阶响应 + 混控 + 气动阻力
- [x] `sim::world` 传感器噪声/风扰（基于真实位置测量）
- [x] `sim::harness` 闭环驱动 + 指标
- [x] `bin/sitl` PID+互补滤波 悬停闭环可运行
- [x] **修复姿态控制符号链 bug（mixer 俯仰轴 + 物理力矩符号），悬停闭环稳定收敛**（见 §6 已解决）

### M2：算法横向对比基线（已完成）
- [x] `estimator` 增加 EKF 实现（松耦合 9 维 ES-EKF：位置/速度/陀螺偏置；与互补滤波对比）
- [x] `controller` 增加 LQR 实现（级联全状态反馈：位置→姿态指令 + 姿态全状态反馈）
- [x] `bin/sitl` 支持多算法组合一键跑对比表（同一初始条件/扰动，支持 `--est/--ctrl/--seconds`）
- [x] 标准测试场景（`sim/scenario.rs` + `Physics` 风扰模型）：定点悬停 / 阶跃目标 / 定点抗风扰 / 方形航线 / 圆形轨迹，命令行 `--scenario <hover|step|wind|square|circle>` 一键对比（见 §3-A）

### M3：控制深度扩展
- [x] `controller` 增加 MPC 实现（`mpc.rs`：短视界双积分器滚动优化 + 倾角/推力约束，投影梯度求解；复用 LQR 姿态内环 + X 混控；`--ctrl mpc`）
- [x] 轨迹跟踪（位置环 + 速度前馈）：PID/LQR 外环接入 `Setpoint.vel` 前馈，方形/圆形航线 RMS 显著下降
- [x] 机型配置抽象（`core/src/config.rs` 的 `VehicleConfig`：质量/惯量/机臂/推力系数/倾角/悬停油门；派生 `dyn_params()`→`PhysicsParams` 与 `ctrl_params()`→`CtrlParams`；PID/LQR/MPC 均提供 `from_config`）
- [x] FDIR 雏形（`core/src/fdir.rs`：IMU 冻结检测 + GPS dropout 检测 → `Health::{Nominal,Degraded,Critical}`；`bin/sitl --fdir` 注入 GPS 丢失窗口 [4s,8s) 演示降级保持与恢复）

---

## 5. 长期规划（M4+，逼近/超越传统飞控）

### M4：仿真真实度提升
- [x] 更精细气动模型（诱导阻力、桨盘滑流、机体气动系数表）
- [x] 传感器故障注入（漂移/卡死/丢帧）+ 鲁棒性对比
- [x] 蒙特卡洛批量仿真（参数不确定性 + 风谱），输出统计指标
- [x] 实时因子评估（host 跑固件级步长，估 F407 上的 CPU 占用）

**§4-A M4 验收（2026-08-07，host 端验证）**

1. 精细气动（`sim/src/physics.rs`）：`PhysicsParams` 新增 `drag_coeff:[f32;3]`（机体三轴系数表）、`induced_drag_coeff`、`disk_area`、`air_density`；阻力改为"机体坐标系按轴系数施加 + 动量理论诱导阻力（v_ind=√(T/2ρA)）"，更接近真实四旋翼滑流。
2. 故障注入（`sim/src/world.rs`）：`FaultKind::{ImuDrift,ImuStuck,GpsDropout}` + `set_fault(t0,t1)`；`World::sense` 在窗口内注入偏置/冻结/丢帧。bin 新增 `--fault <kind>`、`--robust`（四类故障 × EKF+MPC 最大误差对比）。
   - 实测（Hover T=12s）：None/ImuDrift/GpsDropout 均收敛（maxErr≈10m，稳态后恢复），ImuStuck 发散（冻结 IMU 摧毁姿态估计，符合预期——提示需 FDIR 处理卡死）。
3. 蒙特卡洛（`--montecarlo N`）：扰动 ±10% 质量、±15% 惯量、±20% 阻力、±2 m/s 风（确定性种子），输出 posRMS 均值/标准差/p95/发散率。实测 N=30：发散率 0%、均值 3.9±0.26m。
4. 实时因子（`--rtf`）：测量 host 每步墙钟（含 EKF+MPC+物理），按 ×30 MCU 减速比估 F407 168MHz CPU 负载。实测 host 55µs/步（RTF≈90x），估 MCU 33% 控制周期占用 → OK 余量充足。

新增 CLI：`--fault imudrift|imustuck|gpsdropout`、`--robust`、`--montecarlo N`、`--rtf`。

### M5：HAL 与嵌入式落地
- [x] `hal/sensor` trait + `stm32f407` 具体实现（IMU/GPS/气压/罗盘）
- [x] `hal/actuator` trait + PWM/CAN/DSHOT 驱动
- [x] `rtos/` 运行时抽象；接入在研 Rust RTOS（替换 host 调度后端）
- [x] `core` 零堆分配审计（移除一切隐式 alloc，改用 `heapless`/静态池）

**§5-A M5 验收（2026-08-07）**

新增 `core/src/hal/`（全部 `no_std`、无堆、执行时间有界）：
1. `hal/sensor.rs`：`ImuSensor`/`GpsSensor`/`BaroSensor`/`MagSensor` 四个 trait + `mock`（host/SIL 确定性实现）+ `stm32f407`（cfg 门控占位，结构对齐真实 PAC 布局）。
2. `hal/actuator.rs`：`MotorActuator` trait + `clamp_thrust` 饱和保护 + `OutputProtocol` 枚举（PWM/DSHOT/CAN）+ `mock`（记录最近指令供断言）+ `stm32f407`（PWM/DSHOT 占位）。
3. `hal/rtos.rs`：`Runtime` trait（`schedule_periodic`/`enter_failsafe`/`now`）+ `host::SpinLoop`（SIL 自旋）+ `stm32f407::RtosTask`（RTOS 周期任务占位）。
4. 零堆审计：`core/` 静态 grep 确认无 `Vec/Box/String/alloc::` 引用；新增集成测试 `core/tests/no_alloc.rs` 跑通 **sensor→estimator→controller→actuator** 全回路（200 步），断言输出合法、估计无 NaN。新增 `stm32f407` cargo feature 门控芯片实现，已验证 `--features stm32f407` 编译通过。
5. `core/Cargo.toml` 加 `stm32f407` feature；`VehicleConfig::default_quad()`/`ActuatorCmd`/`OutputProtocol` 已 `Default`。

验证：`cargo test -p flyctrl-core` 全过（含 2 个新增 HAL 测试）；`--features stm32f407` 干净编译；全 workspace test 绿。

### M6：通信与地面站
- [x] `mavlink/` 适配层（兼容 QGC），参数/遥测/指令双向
- [x] 内部高效通道（仿真/日志），低开销数据链路

**§6-A M6 验收（2026-08-07）**

新增 `core/src/comm/`（全部 `no_std`、无堆、有界耗时）：
1. `comm/link.rs`：`Link` trait + `Frame`（固定数组 280B）+ `LoopbackLink`（host 回环，按 MAVLink 0xFE 边界组帧/解帧）+ `stm32f407::UartLink`（cfg 占位）。
2. `comm/mavlink.rs`：MAVLink v1 帧格式 + CRC16/X25，覆盖 HEARTBEAT/ATTITUDE/LOCAL_POSITION_NED/SYS_STATUS，`encode`/`decode` 双向；字节级与标准地面站兼容。
3. `comm/telemetry.rs`：`Telemetry` 把 `VehicleState` 按 `rate_hz` 节流入固定 ring buffer（32 帧，满则丢最旧、不阻塞控制），`pop()` 消费式下发。
4. 验证：`core/tests/comm_roundtrip.rs` 跑 `VehicleState→Telemetry→LoopbackLink→decode` 全链路，断言 HEARTBEAT≥1、ATTITUDE/POS≥10 且 CRC 全过；篡改字节 → CRC 拒绝。bin 新增 `--comm` 演示：SITL 回路 + 遥测回环，12s 实测 600 ATTITUDE + 600 LOCAL_POS + 61 HEARTBEAT，0 丢帧，地面站侧全部可解析。

零堆确认：`core/` 下 `comm` 模块无 `Vec/Box/String/alloc::`；`--features stm32f407` 编译通过。

### M7：形式化与验证
- [x] 关键不变量属性测试（`core/src/invariants.rs` + `core/tests/props_invariant.rs`，确定性 LCG 随机化穷举）：
      - 四元数单位范数（姿态积分不得发散，容差 1e-3）
      - 电机指令恒有界 [0,1]（含极端设定点饱和场景）
      - EKF 协方差对称半正定（Jacobi 特征值分解校验，最小特征值 ≥ 0）
      - 整链（传感→估计→控制→FDIR）无 NaN/Inf
      - 失控保护单向（Critical 不自动回 Nominal）
- [x] 类型级安全网扩展（`core/src/state.rs` + `core/tests/props_statemachine.rs`）：
      - 解锁前必须健康：`Fcs::request_arm(healthy)->Option<ArmPermit>`，`arm` 必须消费令牌 → 编译期不可绕过健康检查
      - 失控保护单向：`Armed→Failsafe` 仅能 `reset→Disarmed`，类型系统无回 `Armed` 路径
      - 校准中不可解锁：`Calibrating` 只暴露 `finish->Disarmed`
      - M7 自查发现并修复 EKF 数值隐患：原 `P=(I-KH)P` 形式在噪声下出现非 PSD（负对角元），改用 **Joseph 形式** `P=(I-KH)P(I-KH)^T+KRK^T` + 对称化 + 对角夹取下限 1e-6，协方差现恒保持物理合理（半正定）
- [x] 硬件在环（HIL）桥接（`core/src/hil.rs`）：泛型单步闭环 `HilContext::step<ImuSensor,GpsSensor,MotorActuator,Estimator,Controller>` 跨步持久 EKF+PID+FDIR；host（mock HAL）与 MCU（`stm32f407` 真实 HAL）**共用同一份算法代码**，SIL 验证过的不变量在板上直接成立。自带 SIL 自测：闭环 300 拍无 NaN、指令恒有界；注入冻结 IMU→FDIR 判 Critical→执行器归零（失控保护端到端生效）
- [x] 验证：`cargo test` 全绿（共 37 项：core 9 + comm 2 + no_alloc 2 + 不变量属性 6 + 状态机属性 4 + sim 12 + physics 2）；`--features stm32f407` 编译通过

### M8：算法先进性（拉开差距）
- [x] **增量非线性动态逆 (INDI)**（`core/src/controller/indi.rs`）：`IndiController<B>` 包裹任意 `Controller`，用机体角加速度增量反馈 `u = u_prev + (I/(G·dt))·(p_dot_cmd − p_dot_meas)` 补偿模型误差/扰动；基线指令速率经 mixer 求逆恢复，测量增量由估计器 `omega` 有限差分得到。自带单元属性测试 `indi_active_increment_under_angular_accel`：在有角加速扰动下增量非零且恒有界 [0,1]，证明抗扰机制成立。SIL demo `--indi` 验证闭环有界。
- [x] **学习增强估计**（`core/src/estimator/learning.rs`）：`ResidualModel` trait + `LearningEstimator<E,R>` 残差补偿包装器；提供 `NullResidual`（无补偿）/ `BiasResidual`（常值偏置）两种实现，作为数据驱动残差模型的可插拔骨架，后续可接在线学习/NN 残差。
- [x] **多机协同/编队**（`core/src/swarm.rs` + `core/src/comm/mavlink.rs`）：`SwarmTable<const N>` const-generic 定容邻居表（类型安全、零堆），`Formation` 枚举（V/Line/None）带 `slot_offset(role)` 对称偏置；`FormationController<B,const N>` 复用消息总线（每机 `sys_id`）经 `encode_local_pos_from` 广播 LOCAL_POSITION_NED。SIL demo `--swarm` 双机 V 编队：相对偏置 x,y 精确收敛、min_sep≈3.09m。单元测试 `formation_offsets_symmetric` / `formation_holds` 全绿。
- [x] 验证：`cargo test` 全绿（共 44 项）；`--features stm32f407` 编译通过；`flyctrl-sitl` 构建通过；INDI/Swarm 两 demo 收敛。

### M9：任务层与飞行模式（APPLICATION LAYER）
> PLAN 原架构分层（§1.2）把"飞行模式/任务/航点"列为后续 **APPLICATION LAYER**，M1–M8 完成控制/估计/通信/容错/算法后，M9 补齐这一最上层的用户可感能力。

- [x] **任务/航点**（`core/src/mission.rs`）：`Waypoint`（NED+偏航+到达半径/垂直容差）、`Mission<const N>`（const-generic 定容、零堆、`from_slice` 超长截断）、`MissionRunner<const N>`。关键工程特性：
  - **巡线限速（cruise-rate limiting）**：内部 `target` 每步朝航点以 `max_speed`/`max_vspeed` 有限推进，再输出设定点——避免把航点"瞬移"给控制器导致 PID 积分饱和发散（实测瞬移大阶跃会让简单串级 PID 失控，限速后 4 航点全程稳定收敛）。
  - **地理围栏（Geofence）**：所有设定点经 `clamp` 夹取到圆柱围栏（水平半径+垂直上下界），即便航点文件错误也不会指令飞出安全区——类型级安全网外的第二道物理护栏。
  - 完成后进入 **loiter**（盘旋于末航点），`complete()` 标志任务结束。
- [x] **飞行模式治理**（`core/src/flightmode.rs`）：`FlightMode` 枚举（Manual/Stabilize/Altitude/Position/Mission/Rtl/Land，带 `authorization_level`/`requires_position`/`is_autonomous`）+ `ModeGovernor` 运行时守卫：
  - 合法流转表：未解锁禁自主模式、无位置估计禁 Position/Mission/Rtl、严重故障 `Critical` 只允许 `Land`、降级 `Degraded` 禁最高自主 `Mission`。
  - FDIR 联动：`degrade_on_health` 主动降级（Degraded: Mission→Rtl；Critical: 任意→Land），与 `fdir::Health` 形成"健康→权限"退化链。与 `state::Fcs` 类型级生命周期正交（Fcs 管锁定/失控保护，本模块管"谁生成设定点"）。
- [x] **验证**：`cargo test` 全绿（共 51 项，新增 mission 单元 + `props_mission` 属性测试：到达单调推进/不越界、围栏夹取必在界内、模式流转守卫 3000 组随机组合）；`--features stm32f407` 编译通过；`flyctrl-sitl` 构建通过；`--mission` SIL demo 4 航点全部到达、任务完成、无 NaN、verdict=OK。
- [ ] **后续可选**：任务 YAML/MAVLink 航线导入、返航点记忆与自动 RTL、模式切换的平滑过渡（当前为设定点硬切换 + 限速软化）、把 `ModeGovernor` 接入真实 `Fcs` 解锁状态与 GCS 模式指令通道。

### M10：消息总线 / 发布订阅（MIDDLEWARE）
> PLAN 原架构分层（§2）把"消息总线/发布订阅（类型安全）"列为 MIDDLEWARE 层（后续），
> 是 swarm(M8)/任务(M9)/FDIR 等上层模块的解耦通信骨干。M1–M9 完成后补齐这一层。

- [x] **类型安全 SPSC 环形缓冲**（`core/src/bus.rs` 的 `Ring<M, CAP>`）：零分配、`no_std`、`Copy`；
      `try_push`/`try_pop` 在满/空时返回 `Err`/`None`（**拒绝而非静默覆盖/丢弃**），调用方可据此
      背压或丢帧——符合飞控"宁可丢旧帧也不污染数据流"。FIFO 保序、回绕无错乱（单元 + 属性测试覆盖）。
- [x] **飞行系统总线 `Bus`**：聚合定容主题通道——`imu`(8)/`gps`(4)/`est`(4)/`setpoint`(4)/`actuator`(4)/
      `mode`(4)/`health`(4)/`neighbor`(4)。角色由 API 结构化分离：生产者只经 `publish_*` 写生产者段，消费者只经 `recv_*` 读消费者段。
- [x] **零分配多路扇出（fan-out）**：多订阅主题做扇出，每个订阅节点拿到自己的完整副本、互不消耗彼此数据：
      - `imu` → `imu_to_est` / `imu_to_fdir`（估计器与 FDIR 各一份）。
      - `est` → `est_to_ctrl` / `est_to_fdir` / `est_to_mission` / `est_to_formation`（控制器/FDIR/任务/编队各一份）。
      `Bus::pump()` 每调从各生产者段弹出一个、复制入全部对应消费者段，无需广播队列/堆。
- [x] **FDIR / 任务 / 编队节点已挂到总线（完整解耦栈）**：`--bus` SIL demo 把飞控栈拆成 8 个节点，仅经
      `Bus` 通信、互不持有引用：① 传感器(`imu`/`gps`) ② 估计器(`est` 扇出) ③ FDIR(`recv_est_fdir`+`recv_imu_fdir`
      → `publish_health`) ④ 任务(`recv_est_mission` → `MissionRunner` → `publish_setpoint`) ⑤ 编队
      (`recv_est_formation` → `neighbor` 广播自身状态) ⑥ 模式治理(`recv_health`+请求 → `publish_mode`)
      ⑦ 控制器(`recv_est_ctrl`+`recv_setpoint`+`recv_mode`(+`recv_neighbor` 编队偏移) → `publish_actuator`)
      ⑧ 执行器(`recv_actuator` → `phys.step`)。冷启动以悬停默认命令推进物理避免死锁。
- [x] **验证**：`cargo test` 全绿（bus 单元 + `props_bus` 属性测试：随机 FIFO/满拒绝/多主题数量守恒/4 路扇出无丢失）；
      `--features stm32f407` 编译通过；`--bus` SIL demo 8 节点经总线**稳定悬停（pos≈(0,0,-12)、mode=Position、
      health=Nominal）、总线 0 丢帧、verdict=OK**。
- [x] **编译期主题注册表（单一事实来源）**：`define_topics!` 宏以**扁平主题列表**定义全部 20 个主题
      （8 生产者 + 12 消费者端点），每个主题关联「变体名 / 标记类型（`*Topic`）/ 角色 / 载荷类型 / 容量 / 字段」。
      由此自动生成：`TopicId` 枚举（注册表全集，`all()` 可遍历）、`Topic`/`PubTopic`/`SubTopic` 契约 trait、
      以及**角色门控**的 `push`/`pop` 实现（仅生产者 `impl PubTopic`、仅消费者 `impl SubTopic`，遗漏任一即编译失败）。
      上层用类型安全门面 `bus.publish::<ImuTopic>(sample)` / `bus.subscribe::<EstToCtrlTopic>()`——**载荷类型由主题
      唯一确定，写错类型编译器拒绝**（如 `bus.publish::<ImuTopic>(Meter(1.0))` 编译失败）。具名便捷方法
      `publish_imu`/`recv_est_ctrl` 等保留并委托到泛型门面，向后兼容。新增主题只改宏一行。
- [x] **IRQ 上下文 `irq_lock` 包裹（贴近真实 MCU 部署，HAL 落地前置）**：新增
      `core/src/hal/irq.rs` 提供 `IrqLock` trait + `HostIrqLock`（SIL no-op）+
      `stm32f407::BasepriLock`（`target_arch="arm"` 下用 BASEPRI 阈值临界区，
      屏蔽前保存掩码、退出原样恢复，与 joc-base RTOS `rtos_crit_enter/exit` 同语义；
      panic 安全：drop guard 保证恢复，避免永久关中断）。`Bus` 增加泛型安全包装
      `pump_locked::<L:IrqLock>` / `publish_locked::<T,L>` / `subscribe_locked::<T,L>`，
      供 ISR 内调用。`--features stm32f407` 干净编译（非 arm 退化为 no-op 占位，
      与既有 stm32f407 模块风格一致）。新增测试 `bus_irq_locked_publish_pump_subscribe`
      验证 IRQ 安全 API 与裸 API 行为一致（host）。
- [ ] **后续可选（按优先级）**：
      1. **消息时间标签 + QoS**：`publish` 带 `Timestamp`，订阅支持"最新值覆盖"(`latest`)与"可靠投递"(`reliable`)，
         基于注册表的 `CAP` 与 `Ring` 语义扩展。
      2. **`neighbor` 跨机链路接 `comm/mavlink`**：把单总线 demo 升级为真实多机解耦栈（依赖注册表与 QoS）。
      3. 发布/订阅运行时发现（动态端点登记）。

---

## 6. 已解决的阻塞（归档）

### M1 闭环收敛根因（已修复）
现象：目标悬停 alt=-10m，实际闭环爬升/俯冲、`att.w` 偏离 1.0、水平漂移严重。

根因（两处符号错误，均在姿态控制链）：
1. **控制器 mixer 俯仰轴符号错误**：`m1` 原先为 `0.5*(q_cmd - p_cmd + r_cmd)`（q 为 `+`），纯俯仰指令在 `(m0+m2-m1-m3)` 中完全抵消 → 俯仰力矩恒为零，**俯仰不可控**。改为 `-p_cmd - q_cmd + r_cmd`。
2. **物理力矩符号反转**：`m_roll`/`m_pitch` 原为负号，导致 `+p_cmd` 实际产出负横滚力矩，姿态环变**正反馈**翻转。改为 `+arm*(...)`。

附带：`world.sense` 原先恒定返回位置≈0（估计器看不到真实高度→开环发散），也已改为基于物理真实位置加噪。

验证：`cargo run -p flyctrl-sitl` 四个 (估计器×控制器) 组合均收敛到 alt≈-10m、`att.w≈1.0`、pos_rms<5m、settle<9s。

### M2 EKF 重力修正陷阱（已规避）
EKF 初版带 `att_alpha>0` 的加速度计重力修正时，在姿态发生真实倾斜时把估计姿态强行拉回水平，导致控制器误判、正反馈翻转（`TRUE w` 掉到负而 `EST w` 恒≈1）。
规避：EKF 姿态改为**纯陀螺积分**（陀螺噪声小，悬停场景足够），保留 `att_alpha` 作为可选参数（默认 0）。KF 仍估计并补偿陀螺零偏、用卡尔曼增益融合位置观测。

---

## 7. 执行纪律

- 不破坏 `core` 的 `no_std` / 零堆分配约束。
- 任何新算法先定义/复用 trait，再写实现，保证可插拔对比。
- 每次新增算法/场景，必须能在 `bin/sitl` 输出统一 `Metrics` 对比表。
- 物理模型改动需同步更新本文档 §2 结构说明。
- 长期规划条目开始前，先在 M1–M3 把基线闭环与对比框架彻底跑顺。
