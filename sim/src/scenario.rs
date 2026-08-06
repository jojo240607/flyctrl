//! 标准测试场景。
//!
//! 场景定义"目标设定点如何随时间变化"(`Scenario::setpoint_at(t)`) 以及对世界
//! 环境（风扰/故障）的配置，使同一套闭环可对同一组场景做横向对比。
//!
//! 所有场景均产出 [`Setpoint`]：位置环给定 NED 位置 + 期望速度；偏航保持默认。
//! 控制器据此驱动；指标由 [`crate::harness`] 收集。

use flyctrl_core::controller::trait_def::Setpoint;
use flyctrl_core::units::*;
use flyctrl_core::vehicle::{Meter, MeterPerSecond, Radian};

/// 场景类型枚举（命令行 / 程序内选择）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScenarioKind {
    /// 定点保持（默认基线）：恒定目标悬停。
    Hover,
    /// 阶跃目标：在固定时刻将目标位置平移一段距离，考察阶跃响应。
    Step,
    /// 定点抗风扰：目标不变但世界施加常值风 + 阵风。
    Wind,
    /// 方形航线跟踪：四角航点循环，考察轨迹跟踪。
    Square,
    /// 圆形轨迹跟踪：匀速绕圈，考察连续轨迹跟踪 RMS。
    Circle,
}

impl ScenarioKind {
    pub fn all() -> &'static [ScenarioKind] {
        &[
            ScenarioKind::Hover,
            ScenarioKind::Step,
            ScenarioKind::Wind,
            ScenarioKind::Square,
            ScenarioKind::Circle,
        ]
    }

    pub fn name(self) -> &'static str {
        match self {
            ScenarioKind::Hover => "Hover",
            ScenarioKind::Step => "Step",
            ScenarioKind::Wind => "Wind",
            ScenarioKind::Square => "Square",
            ScenarioKind::Circle => "Circle",
        }
    }

    /// 解析命令行字符串 -> 场景（默认 Hover）。
    pub fn parse(s: &str) -> ScenarioKind {
        match s.to_lowercase().as_str() {
            "hover" | "h" => ScenarioKind::Hover,
            "step" | "s" => ScenarioKind::Step,
            "wind" | "w" => ScenarioKind::Wind,
            "square" | "sq" => ScenarioKind::Square,
            "circle" | "c" | "circ" => ScenarioKind::Circle,
            _ => ScenarioKind::Hover,
        }
    }
}

/// 一个场景实例：封装目标轨迹函数与世界环境覆盖。
pub struct Scenario {
    kind: ScenarioKind,
    /// 初始/基准悬停高度（NED D 向下为正，故为负）。
    base_alt: f32,
    /// 阶跃发生时刻 (s) 与目标偏移（NED）。
    step_at: f32,
    step_offset: [f32; 3],
    /// 方形航线半边长 (m)。
    square_half: f32,
    /// 方形航线周期 (s)。
    square_period: f32,
    /// 圆形轨迹半径 (m)。
    circle_radius: f32,
    /// 圆形轨迹角速度 (rad/s)（=2π/周期）。
    circle_omega: f32,
}

impl Scenario {
    pub fn new(kind: ScenarioKind) -> Self {
        Self {
            kind,
            base_alt: -10.0,
            step_at: 5.0,
            step_offset: [5.0, 0.0, 0.0], // 北向平移 5m
            square_half: 5.0,
            square_period: 16.0,          // 每边 4s
            circle_radius: 5.0,
            circle_omega: 2.0 * core::f32::consts::PI / 12.0, // 12s 一圈
        }
    }

    /// 场景类型。
    pub fn kind(&self) -> ScenarioKind { self.kind }

    /// 阶跃发生时刻 (s)。仅 Step 场景有意义。
    pub fn step_time(&self) -> f32 { self.step_at }

    /// 场景名称。
    pub fn name(&self) -> &'static str { self.kind.name() }

    /// 在该场景下，给定时间 `t` 返回期望 setpoint。
    pub fn setpoint_at(&self, t: Second) -> Setpoint {
        let t = t.0;
        match self.kind {
            ScenarioKind::Hover => Setpoint::hover(
                [Meter(0.0), Meter(0.0), Meter(self.base_alt)],
                Radian(0.0),
            ),

            ScenarioKind::Step => {
                let shifted = if t >= self.step_at { 1.0 } else { 0.0 };
                Setpoint::hover(
                    [
                        Meter(self.step_offset[0] * shifted),
                        Meter(self.step_offset[1] * shifted),
                        Meter(self.base_alt + self.step_offset[2] * shifted),
                    ],
                    Radian(0.0),
                )
            }

            ScenarioKind::Wind => Setpoint::hover(
                [Meter(0.0), Meter(0.0), Meter(self.base_alt)],
                Radian(0.0),
            ),

            ScenarioKind::Square => {
                // 四角：(+h,+h)(-h,+h)(-h,-h)(+h,-h) 循环，t=0 在角点
                let period = self.square_period;
                let phase = (t / period).fract(); // [0,1)
                // 在每个 period/4 内沿边长方向移动；这里简化为角点切换 + 线性插值
                let h = self.square_half;
                let corners = [
                    [h, h],
                    [-h, h],
                    [-h, -h],
                    [h, -h],
                ];
                let (cx, cy, vx, vy) = corner_lerp(&corners, phase, period);
                Setpoint {
                    pos: [Meter(cx), Meter(cy), Meter(self.base_alt)],
                    yaw: Radian(0.0),
                    vel: [MeterPerSecond(vx), MeterPerSecond(vy), MeterPerSecond::ZERO],
                }
            }

            ScenarioKind::Circle => {
                let ang = self.circle_omega * t;
                let r = self.circle_radius;
                let (s, c) = flyctrl_core::math::sin_cos(ang);
                let vx = -r * self.circle_omega * s;
                let vy = r * self.circle_omega * c;
                Setpoint {
                    pos: [Meter(r * c), Meter(r * s), Meter(self.base_alt)],
                    yaw: Radian(0.0),
                    vel: [MeterPerSecond(vx), MeterPerSecond(vy), MeterPerSecond::ZERO],
                }
            }
        }
    }

    /// 返回该场景建议的世界环境参数（风扰等）。
    /// 默认无风，Wind 场景叠加常值风 + 阵风。
    pub fn world_params(&self) -> crate::world::WorldParams {
        let mut p = crate::world::WorldParams::default();
        if self.kind == ScenarioKind::Wind {
            // 北向常值风 3 m/s + 阵风幅度 2 m/s
            p.wind = [3.0, 0.0, 0.0];
            p.wind_gust = 2.0;
        }
        p
    }
}

/// 方形航线：给定 4 角点 + 归一化相位 phase∈[0,1)，返回插值位置与速度。
/// corners 顺序为 NED 的 (x,y)；每段匀速移动。
fn corner_lerp(
    corners: &[[f32; 2]; 4],
    phase: f32,
    period: f32,
) -> (f32, f32, f32, f32) {
    let seg_f = phase * 4.0;
    let seg = seg_f.floor() as usize % 4;
    let frac = seg_f - seg_f.floor(); // 段内进度 [0,1)
    let a = corners[seg];
    let b = corners[(seg + 1) % 4];
    let cx = a[0] + (b[0] - a[0]) * frac;
    let cy = a[1] + (b[1] - a[1]) * frac;
    // 速度 = （段向量）/（段时长），段时长 = period/4
    let dt = period / 4.0;
    let vx = (b[0] - a[0]) / dt;
    let vy = (b[1] - a[1]) / dt;
    (cx, cy, vx, vy)
}
