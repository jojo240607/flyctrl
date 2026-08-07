//! 任务层：航点（Waypoint）/任务（Mission）/任务执行器（MissionRunner）。
//!
//! 这是架构分层里的 **APPLICATION LAYER**（飞行模式/任务/航点），建立在
//! [`crate::controller::Setpoint`] 与 [`crate::vehicle::VehicleState`] 之上。
//!
//! 设计目标（与既有 `core` 一致的约束）：
//! - `no_std`、零堆分配：任务表用 const-generic 固定容量 `Mission<const N>`。
//! - 到达判定纯几何（水平半径 + 垂直容差），不引入时序依赖。
//! - **地理围栏（geofence）**：所有生成的设定点位置被夹取到圆柱/球围栏内，
//!   即便航点文件错误也不会指令飞出安全区——类型级安全网之外的第二道物理护栏。
//! - 任务完成后进入**盘旋（loiter）**，不会因"无目标"而失控。

use crate::controller::trait_def::Setpoint;
use crate::math;
use crate::units::*;
use crate::vehicle::VehicleState;

/// 单个航点：NED 位置 + 期望偏航 + 到达判据。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Waypoint {
    pub pos: [Meter; 3],
    pub yaw: Radian,
    /// 水平到达半径（m）。估计水平位置进入此半径即判到达。
    pub radius: Meter,
    /// 垂直到达容差（m）。高度差小于此值才算到达（避免水平到位但高度差大）。
    pub alt_tol: Meter,
}

impl Waypoint {
    /// 构造悬停式航点（零速度、给定水平半径与垂直容差）。
    pub fn new(pos: [Meter; 3], yaw: Radian, radius: Meter, alt_tol: Meter) -> Self {
        Self { pos, yaw, radius, alt_tol }
    }

    /// 当前估计状态是否"到达"本航点（水平 + 垂直同时满足）。
    pub fn reached(&self, est: &VehicleState) -> bool {
        let dx = est.pos[0].0 - self.pos[0].0;
        let dy = est.pos[1].0 - self.pos[1].0;
        let horiz = math::sqrt(dx * dx + dy * dy);
        let vert = (est.pos[2].0 - self.pos[2].0).abs();
        horiz <= self.radius.0 && vert <= self.alt_tol.0
    }
}

/// 地理围栏：以 `center` 为轴、半径 `radius`、相对高度上下界 `ceil/floor` 的
/// 圆柱围栏（NED：D 向下为正）。所有任务设定点经 [`Geofence::clamp`] 夹取。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geofence {
    pub center: [Meter; 3],
    pub radius: Meter,
    pub ceil: Meter,  // 最高允许高度（D 最小，越接近 0 越高）
    pub floor: Meter, // 最低允许高度（D 最大）
}

impl Geofence {
    /// 标准悬停围栏：原点上方 120 m、下方 5 m、水平半径 80 m。
    pub fn default_quad() -> Self {
        Self {
            center: [Meter::ZERO, Meter::ZERO, Meter(-10.0)],
            radius: Meter(80.0),
            ceil: Meter(-120.0),
            floor: Meter(5.0),
        }
    }

    /// 把任意世界系位置夹取到围栏内，返回夹取后的位置。
    ///
    /// 水平超界时沿径向投影回边界；垂直越界直接夹到 ceil/floor。
    /// 返回 `(pos, clamped)`：`clamped=true` 表示原位置确实越界、被修改。
    pub fn clamp(&self, pos: [Meter; 3]) -> ([Meter; 3], bool) {
        let dx = pos[0].0 - self.center[0].0;
        let dy = pos[1].0 - self.center[1].0;
        let horiz = math::sqrt(dx * dx + dy * dy);
        let mut nx = pos[0].0;
        let mut ny = pos[1].0;
        let mut clamped = false;

        if horiz > self.radius.0 && horiz > 1e-6 {
            let k = self.radius.0 / horiz;
            nx = self.center[0].0 + dx * k;
            ny = self.center[1].0 + dy * k;
            clamped = true;
        }
        let mut nz = pos[2].0;
        // D 向下为正：ceil 是更小的值（更高），floor 是更大的值（更低）。
        if nz < self.ceil.0 {
            nz = self.ceil.0;
            clamped = true;
        }
        if nz > self.floor.0 {
            nz = self.floor.0;
            clamped = true;
        }
        ([Meter(nx), Meter(ny), Meter(nz)], clamped)
    }
}

/// 任务：定容航点序列（零堆）。
///
/// `count` 为有效航点数；超过 `N` 的航点被静默忽略（与既有 `SwarmTable`
/// 一致的有界语义）。遍历只到 `count`，未用槽位不参与逻辑。
#[derive(Debug, Clone, Copy)]
pub struct Mission<const N: usize> {
    wp: [Waypoint; N],
    count: usize,
}

impl<const N: usize> Mission<N> {
    /// 空任务。
    pub fn new() -> Self {
        // 用默认航点填充（不会被访问，count=0）。
        let wp = [Waypoint::new([Meter::ZERO; 3], Radian::ZERO, Meter(1.0), Meter(1.0)); N];
        Self { wp, count: 0 }
    }

    /// 追加航点；容量满则返回 `false`（调用方应感知截断）。
    pub fn push(&mut self, w: Waypoint) -> bool {
        if self.count >= N {
            return false;
        }
        self.wp[self.count] = w;
        self.count += 1;
        true
    }

    /// 由切片构造（超长截断到 N）。
    pub fn from_slice(s: &[Waypoint]) -> Self {
        let mut m = Self::new();
        for w in s.iter() {
            if !m.push(*w) {
                break;
            }
        }
        m
    }

    pub fn len(&self) -> usize { self.count }
    pub fn is_empty(&self) -> bool { self.count == 0 }
    pub fn capacity(&self) -> usize { N }

    /// 取第 `i` 个航点；越界返回 `None`（不 panic，符合嵌入式安全偏好）。
    pub fn get(&self, i: usize) -> Option<&Waypoint> {
        if i < self.count { Some(&self.wp[i]) } else { None }
    }
}

impl<const N: usize> Default for Mission<N> {
    fn default() -> Self { Self::new() }
}

/// 任务执行器：跟踪当前目标航点索引，按到达判定推进；
/// 自带地理围栏，所有输出设定点都经过围栏夹取。
///
/// 关键工程特性：**巡线限速（cruise-rate limiting）**。任务设定点不是把目标
/// 航点直接"瞬移"给控制器（瞬移大阶跃会让 PID 积分饱和、机体发散），而是内部
/// 维护一个 `target`，每步朝当前航点以 `max_speed` / `max_vspeed` 有限推进，
/// 再夹取进地理围栏后输出。这既符合真实航点飞行（定速逼近），又避免控制器发散。
///
/// 完成后进入 **loiter**（盘旋于最后一个航点），`complete()` 返回 `true`。
pub struct MissionRunner<const N: usize> {
    mission: Mission<N>,
    idx: usize,
    fence: Geofence,
    loiter_alt: Meter,
    fence_hit: bool, // 最近一次 setpoint 是否触发了围栏夹取
    target: [Meter; 3], // 内部限速跟踪点
    max_speed: MeterPerSecond, // 水平巡线速度上限
    max_vspeed: MeterPerSecond, // 垂直速度上限
}

impl<const N: usize> MissionRunner<N> {
    /// 构造；`max_speed`/`max_vspeed` 缺省为 3.0 / 1.5 m/s（常规四旋翼航点速度）。
    pub fn new(mission: Mission<N>, fence: Geofence) -> Self {
        let loiter_alt = mission
            .get(0)
            .map(|w| w.pos[2])
            .unwrap_or(Meter(-10.0));
        let target = mission
            .get(0)
            .map(|w| w.pos)
            .unwrap_or([Meter::ZERO; 3]);
        Self {
            mission,
            idx: 0,
            fence,
            loiter_alt,
            fence_hit: false,
            target,
            max_speed: MeterPerSecond(3.0),
            max_vspeed: MeterPerSecond(1.5),
        }
    }

    /// 自定义巡线速度。
    pub fn with_speeds(mut self, max_speed: MeterPerSecond, max_vspeed: MeterPerSecond) -> Self {
        self.max_speed = max_speed;
        self.max_vspeed = max_vspeed;
        self
    }

    /// 当前目标航点索引（任务完成后停在 `len()`，表示无有效目标）。
    pub fn current_index(&self) -> usize { self.idx }

    /// 任务是否已完成（所有航点到达）。
    pub fn complete(&self) -> bool {
        self.mission.is_empty() || self.idx >= self.mission.len()
    }

    /// 最近一次 [`Self::update`] 是否触发了围栏夹取（用于遥测/告警）。
    pub fn fence_hit(&self) -> bool { self.fence_hit }

    /// 复位到第 0 个航点（重新执行任务），并复位内部 `target`。
    pub fn restart(&mut self) {
        self.idx = 0;
        self.fence_hit = false;
        self.target = self
            .mission
            .get(0)
            .map(|w| w.pos)
            .unwrap_or([Meter::ZERO; 3]);
    }

    /// 由估计状态推进任务进度，并以限速朝当前航点推进 `target`，返回设定点。
    ///
    /// - 若当前航点到达，则 `idx` 前进；若已是最后一个，则保持 `idx=len()`
    ///   （完成后盘旋于最后航点）。
    /// - `target` 每步朝目标最多移动 `max_speed*dt`（水平）/ `max_vspeed*dt`（垂直）。
    /// - 输出设定点位置经 [`Geofence::clamp`] 夹取，保证物理安全。
    pub fn update(&mut self, est: &VehicleState, dt: Second) -> Setpoint {
        // 推进逻辑：仅在未越界时推进（越界时不应"判定到达"误推进）。
        if !self.complete() {
            if let Some(w) = self.mission.get(self.idx) {
                if w.reached(est) {
                    self.idx += 1;
                }
            }
        }

        // 目标生成：完成后盘旋于最近一个有效航点（或围栏中心）。
        let (goal, yaw) = if self.complete() {
            let base = self
                .mission
                .get(self.mission.len().saturating_sub(1))
                .map(|w| w.pos)
                .unwrap_or(self.fence.center);
            ([base[0], base[1], self.loiter_alt], Radian::ZERO)
        } else {
            let w = self.mission.get(self.idx).unwrap();
            (w.pos, w.yaw)
        };

        // 限速推进 target → goal（水平/垂直分别限速）。
        let step_h = self.max_speed.0 * dt.0;
        let step_v = self.max_vspeed.0 * dt.0;
        for ax in 0..2 {
            let d = goal[ax].0 - self.target[ax].0;
            let ad = d.abs();
            if ad <= step_h {
                self.target[ax].0 = goal[ax].0;
            } else {
                self.target[ax].0 += d.signum() * step_h;
            }
        }
        let dv = goal[2].0 - self.target[2].0;
        let adv = dv.abs();
        if adv <= step_v {
            self.target[2].0 = goal[2].0;
        } else {
            self.target[2].0 += dv.signum() * step_v;
        }

        let (clamped, hit) = self.fence.clamp(self.target);
        self.fence_hit = hit;
        Setpoint {
            pos: clamped,
            yaw,
            vel: [MeterPerSecond::ZERO; 3],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn waypoint_reached_horizontal_and_vertical() {
        let w = Waypoint::new([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian::ZERO, Meter(2.0), Meter(1.0));
        let mut s = VehicleState::zero();
        s.pos = [Meter(1.0), Meter(0.0), Meter(-10.0)]; // 水平 1m，垂直 0
        assert!(w.reached(&s));
        s.pos = [Meter(1.0), Meter(0.0), Meter(-8.0)]; // 垂直 2m > 容差
        assert!(!w.reached(&s));
        s.pos = [Meter(3.0), Meter(0.0), Meter(-10.0)]; // 水平 3m > 半径
        assert!(!w.reached(&s));
    }

    #[test]
    fn geofence_clamps_out_of_bounds() {
        let f = Geofence::default_quad();
        // 水平超出半径 80：应被夹回边界（距中心 80）
        let (c, hit) = f.clamp([Meter(200.0), Meter(0.0), Meter(-10.0)]);
        assert!(hit);
        let dx = c[0].0 - f.center[0].0;
        let dy = c[1].0 - f.center[1].0;
        let horiz = (dx * dx + dy * dy).sqrt();
        assert!((horiz - f.radius.0).abs() < 1e-3);
        // 高于 ceil（-120）：应夹到 -120
        let (c2, hit2) = f.clamp([Meter(0.0), Meter(0.0), Meter(-200.0)]);
        assert!(hit2);
        assert!((c2[2].0 - f.ceil.0).abs() < 1e-3);
        // 低于 floor（5）：应夹到 5
        let (c3, hit3) = f.clamp([Meter(0.0), Meter(0.0), Meter(50.0)]);
        assert!(hit3);
        assert!((c3[2].0 - f.floor.0).abs() < 1e-3);
        // 围栏内不夹取
        let (_, hit4) = f.clamp([Meter(0.0), Meter(0.0), Meter(-10.0)]);
        assert!(!hit4);
    }

    #[test]
    fn mission_runner_advances_on_arrival() {
        let wps = [
            Waypoint::new([Meter(0.0), Meter(0.0), Meter(-10.0)], Radian::ZERO, Meter(1.0), Meter(1.0)),
            Waypoint::new([Meter(10.0), Meter(0.0), Meter(-10.0)], Radian::ZERO, Meter(1.0), Meter(1.0)),
        ];
        let mission = Mission::<4>::from_slice(&wps);
        let mut runner = MissionRunner::new(mission, Geofence::default_quad());
        let mut s = VehicleState::zero();
        s.pos = [Meter(0.0), Meter(0.0), Meter(-10.0)];
        // 第一步：在 wp0，应已到达并推进到 wp1
        let _ = runner.update(&s, Second(0.02));
        assert_eq!(runner.current_index(), 1);
        // 推进到 wp1 位置
        s.pos = [Meter(10.0), Meter(0.0), Meter(-10.0)];
        let _ = runner.update(&s, Second(0.02));
        assert!(runner.complete());
        // 完成后盘旋设定点应基于最后一个航点高度
        let sp = runner.update(&s, Second(0.02));
        assert!((sp.pos[2].0 - (-10.0)).abs() < 1e-3);
    }

    #[test]
    fn mission_capacity_truncation() {
        // 容量 2，塞 5 个只保留前 2
        let wps: [Waypoint; 5] = [0, 1, 2, 3, 4].map(|i| {
            Waypoint::new([Meter(i as f32), Meter(0.0), Meter(-10.0)], Radian::ZERO, Meter(1.0), Meter(1.0))
        });
        let m = Mission::<2>::from_slice(&wps);
        assert_eq!(m.len(), 2);
        assert_eq!(m.get(0).unwrap().pos[0].0, 0.0);
        assert_eq!(m.get(1).unwrap().pos[0].0, 1.0);
        assert!(m.get(2).is_none());
    }
}
