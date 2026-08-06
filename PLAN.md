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
- [ ] 更精细气动模型（诱导阻力、桨盘滑流、机体气动系数表）
- [ ] 传感器故障注入（漂移/卡死/丢帧）+ 鲁棒性对比
- [ ] 蒙特卡洛批量仿真（参数不确定性 + 风谱），输出统计指标
- [ ] 实时因子评估（host 跑固件级步长，估 F407 上的 CPU 占用）

### M5：HAL 与嵌入式落地
- [ ] `hal/sensor` trait + `stm32f407` 具体实现（IMU/GPS/气压/罗盘）
- [ ] `hal/actuator` trait + PWM/CAN/DSHOT 驱动
- [ ] `rtos/` 运行时抽象；接入在研 Rust RTOS（替换 host 调度后端）
- [ ] `core` 零堆分配审计（移除一切隐式 alloc，改用 `heapless`/静态池）

### M6：通信与地面站
- [ ] `mavlink/` 适配层（兼容 QGC），参数/遥测/指令双向
- [ ] 内部高效通道（仿真/日志），低开销数据链路

### M7：形式化与验证
- [ ] 关键不变量属性测试（如"估计协方差正定"、"控制输出有界"）
- [ ] 类型级安全网扩展（如"解锁前必须健康"、"失控保护单向"）
- [ ] 硬件在环（HIL）桥接，SIL 用例直接复用

### M8：算法先进性（拉开差距）
- [ ] 自适应控制 / 增量非线性动态逆 (INDI)
- [ ] 学习增强估计（数据驱动残差补偿）
- [ ] 多机协同/编队（复用消息总线 + 类型安全状态共享）

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
