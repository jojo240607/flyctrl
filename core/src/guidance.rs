//! **制导层**（阶段 5）：把"轨迹"转成控制器能吃的 [`Setpoint`]。
//!
//! # 定位（为什么是这个形状）
//!
//! `Setpoint` **早已含速度/加速度前馈**（`pos`/`yaw`/`vel`/`acc`，见 P3-A1 提交
//! `6d79086`），`PidController` 也已实现前馈 + 倾斜补偿（`cos` 补偿）。因此阶段 5
//! **不需要重建控制链路** —— 缺的只是"**连续轨迹 → Setpoint**"这一层生成器，
//! 以及随之而来的机动类型与判据。
//!
//! # 为什么用「采样 + trait」而不是直接吃 `Trajectory`
//!
//! `Trajectory` 定义在 `fly-sim-core`（仿真侧，含物理引擎依赖），而本模块在
//! `flyctrl-core`（**固件侧、no_std**）—— 固件不可能依赖仿真库。
//! ⇒ 本模块只认一个**纯数据采样** [`TrajectorySample`] 与一个 [`TrajectorySource`]
//! trait；仿真侧的 `Trajectory` 由调用方（SIL harness / 测试）适配进来，或直接实现
//! 该 trait（如一个解析式圆/八字，连仿真库都不需要）。
//!
//! # 与「航点」的关系
//!
//! `mission.rs` 是**航点式**（离散目标 + 到达半径），本模块是**连续轨迹式**
//! （每拍都有位/速/加前馈）。两者互补：航点适合任务层，连续轨迹适合跟踪判据。

use crate::controller::Setpoint;
use crate::units::{Meter, MeterPerSecond, MeterPerSecondSquared, Radian, Second};

/// 轨迹在某一时刻的**期望状态**（世界系 NED）。
///
/// 三个量都给：位置供位置环、速度/加速度供**前馈**（`Setpoint` 已支持）。
/// 只给位置会让外环"等误差积累再纠"，跟踪相位滞后 —— 这正是 `mission.rs` 注释里
/// 记录的 P3-A1 之前的症状（`cruise_v/kp_xy ≈ 6.7m` 的稳态跟随误差）。
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrajectorySample {
    /// 期望位置（NED，m）
    pub pos: [Meter; 3],
    /// 期望速度（NED，m/s）—— 前馈
    pub vel: [MeterPerSecond; 3],
    /// 期望加速度（NED，m/s²）—— 前馈（转弯/机动预倾）
    pub acc: [MeterPerSecondSquared; 3],
    /// 期望偏航（rad）
    pub yaw: Radian,
}

impl TrajectorySample {
    /// 定点悬停采样（零速度/加速度前馈）。
    pub fn hover(pos: [Meter; 3], yaw: Radian) -> Self {
        Self {
            pos,
            vel: [MeterPerSecond::ZERO; 3],
            acc: [MeterPerSecondSquared::ZERO; 3],
            yaw,
        }
    }
}

/// 轨迹源：给时刻，取采样。**纯函数式**，便于单测与解析式轨迹。
pub trait TrajectorySource {
    /// 轨迹总时长（s）；用于判断"是否已飞完"。
    fn duration(&self) -> Second;
    /// 取 `t` 时刻的期望状态。**实现须对 `t > duration` 做保持**（飞完后停在终点），
    /// 不得外推（否则飞完瞬间会产生虚假的大前馈）。
    fn at(&self, t: Second) -> TrajectorySample;
}

/// **制导**：按时间推进轨迹源，产出带前馈的 [`Setpoint`]。
///
/// 用法：`let mut g = Guidance::new(src); loop { let sp = g.step(dt); ctrl.step(&sp); }`
///
/// 注意本层**不做反馈/限幅**：它只把"期望状态"翻译成 `Setpoint`；
/// 限幅（`vmax_xy`/`tilt_max`）与反馈在控制器内 ✓ 保持职责单一，便于单独判据。
pub struct Guidance<S: TrajectorySource> {
    src: S,
    /// 当前轨迹时间（s）。由 `step` 按 `dt` 推进。
    t: Second,
    /// 固定时间步（s）：调用方传入的 dt 应与之相符，否则时间会漂。
    dt: Second,
    /// 是否已飞完（`t >= duration`）。
    done: bool,
}

impl<S: TrajectorySource> Guidance<S> {
    /// 构造。`dt` 为控制拍（s）—— 用于内部时间推进的自洽校验。
    pub fn new(src: S, dt: Second) -> Self {
        Self {
            src,
            t: Second::ZERO,
            dt,
            done: false,
        }
    }

    /// 当前轨迹时间（s）。
    pub fn time(&self) -> Second {
        self.t
    }

    /// 是否已飞完。
    pub fn done(&self) -> bool {
        self.done
    }

    /// 取**当前**时刻的 Setpoint（不推进时间）。
    pub fn setpoint(&self) -> Setpoint {
        let s = self.src.at(self.t);
        Setpoint {
            pos: s.pos,
            yaw: s.yaw,
            vel: s.vel,
            acc: s.acc,
        }
    }

    /// 推进 `dt` 并返回**下一拍**的 Setpoint。
    ///
    /// 语义：返回的 Setpoint 对应 `t + dt`（即"本拍应到达的期望状态"），
    /// 再推进内部时间 —— 与控制器"用当前 Setpoint 算出本拍指令"的时序一致。
    pub fn step(&mut self) -> Setpoint {
        let t_next = Second(self.t.0 + self.dt.0);
        let s = self.src.at(t_next);
        if t_next.0 >= self.src.duration().0 {
            self.done = true;
            self.t = self.src.duration();
        } else {
            self.t = t_next;
        }
        Setpoint {
            pos: s.pos,
            yaw: s.yaw,
            vel: s.vel,
            acc: s.acc,
        }
    }
}

// ---------------------------------------------------------------- 解析式轨迹（无仿真依赖）

/// **圆轨迹**（水平面、恒定高度、恒定速率、机头切向）。
///
/// 最简的"连续轨迹"：恒定速率 ⇒ 有常值向心加速度 ⇒ 天然测试**加速度前馈**
/// （无前馈时外环必须靠位置误差换倾角 ⇒ 恒定滞后）。
/// 判据用途：阶段 5 的"**估计误差是否被制导放大**"用它与阶段 3 的单模块误差对比。
#[derive(Clone, Copy, Debug)]
pub struct Circle {
    /// 圆心（NED，m）
    pub center: [Meter; 3],
    /// 半径（m）
    pub radius: Meter,
    /// 角速率（rad/s）：`v = ω·r`、`a = ω²·r`
    pub omega: f32,
    /// 圈数
    pub laps: f32,
    /// 起始相位（rad）
    pub phase0: f32,
    /// 是否机头切向（否则保持 `yaw0`）
    pub nose_tangent: bool,
}

impl Circle {
    /// 常用构造：圆心 + 半径 + 角速率 + 圈数。
    pub fn new(center: [Meter; 3], radius: Meter, omega: f32, laps: f32) -> Self {
        Self {
            center,
            radius,
            omega,
            laps,
            phase0: 0.0,
            nose_tangent: true,
        }
    }
}

impl TrajectorySource for Circle {
    fn duration(&self) -> Second {
        Second(self.laps * 2.0 * core::f32::consts::PI / self.omega.abs().max(1e-6))
    }

    fn at(&self, t: Second) -> TrajectorySample {
        // 飞完后**保持终点**（不外推 —— 否则终点处会冒出虚假前馈）。
        let tt = t.0.min(self.duration().0);
        let ph = self.phase0 + self.omega * tt;
        let (s, c) = ph.sin_cos();
        let r = self.radius.0;
        let w = self.omega;
        // 位置：圆心 + r·(cosφ, sinφ)（北/东）；高度取圆心的高
        let pos = [
            Meter(self.center[0].0 + r * c),
            Meter(self.center[1].0 + r * s),
            self.center[2],
        ];
        // 速度 = d/dt：(-r·ω·sinφ, r·ω·cosφ, 0)
        let vel = [
            MeterPerSecond(-r * w * s),
            MeterPerSecond(r * w * c),
            MeterPerSecond(0.0),
        ];
        // 加速度 = d²/dt² = (-r·ω²·cosφ, -r·ω²·sinφ, 0)（向心，恒指向圆心）
        let acc = [
            MeterPerSecondSquared(-r * w * w * c),
            MeterPerSecondSquared(-r * w * w * s),
            MeterPerSecondSquared(0.0),
        ];
        // 机头切向：atan2(ve, vn)
        let yaw = if self.nose_tangent {
            Radian(f32::atan2(vel[1].0, vel[0].0))
        } else {
            Radian(0.0)
        };
        TrajectorySample {
            pos,
            vel,
            acc,
            yaw,
        }
    }
}

/// **八字轨迹**（Lissajous 1:2）——阶段 5 点名的机动之一。
///
/// `n = A·sin(ωt)`、`e = B·sin(2ωt)/2` ⇒ 水平"8"字；高度恒定。
/// 比圆更苛刻：曲率**变号** ⇒ 向心加速度方向翻转 ⇒ 前馈必须双向都对。
#[derive(Clone, Copy, Debug)]
pub struct Figure8 {
    /// 中心（NED，m）
    pub center: [Meter; 3],
    /// 北向幅值（m）
    pub amp_n: Meter,
    /// 东向幅值（m）—— 取 `amp_n/2` 时得到标准 8 字
    pub amp_e: Meter,
    /// 基础角速率（rad/s）；东向用 2ω
    pub omega: f32,
    /// 周期数
    pub laps: f32,
}

impl Figure8 {
    /// 标准 8 字：`amp_e = amp_n/2`。
    pub fn new(center: [Meter; 3], amp_n: Meter, omega: f32, laps: f32) -> Self {
        Self {
            center,
            amp_n,
            amp_e: Meter(amp_n.0 * 0.5),
            omega,
            laps,
        }
    }
}

impl TrajectorySource for Figure8 {
    fn duration(&self) -> Second {
        Second(self.laps * 2.0 * core::f32::consts::PI / self.omega.abs().max(1e-6))
    }

    fn at(&self, t: Second) -> TrajectorySample {
        let tt = t.0.min(self.duration().0);
        let ph = self.omega * tt;
        let (s1, c1) = ph.sin_cos();
        let (s2, c2) = (2.0 * ph).sin_cos();
        let (an, ae, w) = (self.amp_n.0, self.amp_e.0, self.omega);
        let pos = [
            Meter(self.center[0].0 + an * s1),
            Meter(self.center[1].0 + ae * s2),
            self.center[2],
        ];
        let vel = [
            MeterPerSecond(an * w * c1),
            MeterPerSecond(2.0 * ae * w * c2),
            MeterPerSecond(0.0),
        ];
        let acc = [
            MeterPerSecondSquared(-an * w * w * s1),
            MeterPerSecondSquared(-4.0 * ae * w * w * s2),
            MeterPerSecondSquared(0.0),
        ];
        let yaw = Radian(f32::atan2(vel[1].0, vel[0].0));
        TrajectorySample {
            pos,
            vel,
            acc,
            yaw,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 圆轨迹的运动学自洽：速度 = d(pos)/dt、加速度 = d(vel)/dt（数值微分核对）。
    /// 这条守的是**解析式本身**（前后馈若与位置不自洽，跟踪误差会凭空出现）。
    #[test]
    fn circle_kinematics_self_consistent() {
        let c = Circle::new(
            [Meter(0.0), Meter(0.0), Meter(-5.0)],
            Meter(2.0),
            1.0,
            1.0,
        );
        // ⚠️ 容差依据：f32 下中心/前向差分的舍入误差 ≈ ε·|f|/h。
        // 取 h=1e-4、|v|≈2 m/s ⇒ ε·|f|/h = 1.2e-7×2/1e-4 ≈ **0.002** —— 故 1e-3 过紧
        // （实测正是 0.002 量级的偏差）。此处取 5e-3：仍足以抓住真正的"前馈与位置
        // 不自洽"（那会是 0.1 量级），但不再被 f32 舍入误伤。
        let h = 1e-4f32;
        for &t in &[0.0f32, 1.0, 3.0, 5.0] {
            let a = c.at(Second(t));
            let b = c.at(Second(t + h));
            // 数值微分位置 -> 速度
            for i in 0..3 {
                let v_num = (b.pos[i].0 - a.pos[i].0) / h;
                assert!(
                    (v_num - a.vel[i].0).abs() < 5e-3,
                    "t={t} 轴{i}: 速度前馈 {:.4} vs 数值微分 {:.4}",
                    a.vel[i].0,
                    v_num
                );
            }
            // 数值微分速度 -> 加速度
            for i in 0..3 {
                // 同样受 f32 舍入限制（此处 |a|≈2 m/s²，差分误差 ≈ 0.2 量级边缘）
                let a_num = (b.vel[i].0 - a.vel[i].0) / h;
                assert!(
                    (a_num - a.acc[i].0).abs() < 5e-2,
                    "t={t} 轴{i}: 加速度前馈 {:.4} vs 数值微分 {:.4}",
                    a.acc[i].0,
                    a_num
                );
            }
        }
    }

    /// 飞完后**保持终点**（不得外推）——否则终点瞬间会产生虚假大前馈。
    #[test]
    fn holds_end_after_duration() {
        let c = Circle::new([Meter(0.0), Meter(0.0), Meter(-5.0)], Meter(1.0), 2.0, 1.0);
        let d = c.duration().0;
        let e1 = c.at(Second(d));
        let e2 = c.at(Second(d + 10.0));
        assert_eq!(e1, e2, "飞完后应保持终点采样（不外推）");
    }

    /// 八字轨迹同样自洽 + 标准 8 字（东向幅值为北向一半）。
    #[test]
    fn figure8_self_consistent() {
        let f = Figure8::new([Meter(0.0), Meter(0.0), Meter(-5.0)], Meter(2.0), 0.6, 1.0);
        let h = 1e-4f32;
        for &t in &[0.0f32, 2.0, 7.0] {
            let a = f.at(Second(t));
            let b = f.at(Second(t + h));
            for i in 0..3 {
                let v_num = (b.pos[i].0 - a.pos[i].0) / h;
                assert!((v_num - a.vel[i].0).abs() < 5e-2, "8字 t={t} 轴{i} 速度不自洽");
            }
        }
    }

    /// `Guidance::step` 的时间语义：返回"下一拍"的采样，且飞完后置 `done`。
    #[test]
    fn guidance_advances_and_finishes() {
        let c = Circle::new([Meter(0.0), Meter(0.0), Meter(-5.0)], Meter(1.0), 1.0, 1.0);
        let dur = c.duration().0;
        let mut g = Guidance::new(c, Second(0.004));
        let mut n = 0;
        while !g.done() && n < 100_000 {
            let _ = g.step();
            n += 1;
        }
        assert!(g.done(), "应能飞完（duration={dur}s）");
        assert!(
            (g.time().0 - dur).abs() < 0.01,
            "飞完后时间应停在 duration，实际 {}",
            g.time().0
        );
    }
}
