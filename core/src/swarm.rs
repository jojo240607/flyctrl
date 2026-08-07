//! 多机协同 / 编队（M8.2）。
//!
//! 设计要点：
//! - 类型安全的邻居状态共享：固定容量 [`SwarmTable`]，每架飞机把自身状态经
//!   MAVLink（复用 `comm::mavlink`，按各自 `sys_id` 广播 `LOCAL_POSITION_NED`）
//!   发出，邻居接收后入库。无堆、容量在编译期确定。
//! - 编队控制：以"长机 + 相对偏移"定义阵型（V 字 / 一字），每架从 `SwarmTable`
//!   读取长机（及邻居）状态，生成相对设定点，委托给基底控制器（`Controller`）跟踪。
//! - 分离/避碰：在设定点上叠加一个基于邻居相对位置的斥力项，防止碰撞
//!   （最简化人工势场，有界、确定）。
//!
//! 全部复用既有消息总线与类型安全状态共享，编队闭环可在 SIL 中端到端验证。

use crate::comm::mavlink;
use crate::comm::link::{Frame, MAX_FRAME_LEN};
use crate::controller::{Controller, Setpoint};
use crate::units::*;
use crate::vehicle::{ActuatorCmd, VehicleState};

/// 单架邻居的共享状态（NED 位置 + 速度 + 新鲜度）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NeighborState {
    pub id: u8,
    pub pos: [f32; 3],
    pub vel: [f32; 3],
    /// 自上次更新以来经过的毫秒数；超过阈值视为掉线（stale）。
    pub age_ms: u32,
}

impl NeighborState {
    pub fn fresh(id: u8, pos: [f32; 3], vel: [f32; 3]) -> Self {
        Self { id, pos, vel, age_ms: 0 }
    }
}

/// 邻居状态表（固定容量 `N`，编译期确定，无堆）。
#[derive(Debug, Clone, Copy)]
pub struct SwarmTable<const N: usize> {
    slots: [Option<NeighborState>; N],
}

impl<const N: usize> SwarmTable<N> {
    pub fn new() -> Self {
        Self { slots: [None; N] }
    }

    /// 写入/更新某邻居状态，重置其 age。同 id 覆盖，新 id 找空位或最陈旧位。
    pub fn update(&mut self, st: NeighborState) {
        // 先找同 id。
        for s in self.slots.iter_mut() {
            if let Some(x) = s {
                if x.id == st.id {
                    *x = st;
                    return;
                }
            }
        }
        // 找空位。
        for s in self.slots.iter_mut() {
            if s.is_none() {
                *s = Some(st);
                return;
            }
        }
        // 否则替换最陈旧者。
        let mut oldest = 0;
        let mut oldest_age = 0u32;
        for (i, s) in self.slots.iter().enumerate() {
            if let Some(x) = s {
                if x.age_ms > oldest_age {
                    oldest_age = x.age_ms;
                    oldest = i;
                }
            }
        }
        self.slots[oldest] = Some(st);
    }

    /// 所有条目 age 递增（调用方按时间推进）；超过 `stale_ms` 视为掉线并清除。
    pub fn age_all(&mut self, dt_ms: u32, stale_ms: u32) {
        for s in self.slots.iter_mut() {
            if let Some(x) = s {
                x.age_ms += dt_ms;
                if x.age_ms > stale_ms {
                    *s = None;
                }
            }
        }
    }

    pub fn get(&self, id: u8) -> Option<NeighborState> {
        self.slots.iter().flatten().find(|x| x.id == id).copied()
    }

    pub fn count(&self) -> usize {
        self.slots.iter().filter(|s| s.is_some()).count()
    }

    /// 遍历所有在线邻居（闭包）。
    pub fn for_each(&self, mut f: impl FnMut(&NeighborState)) {
        for s in self.slots.iter().flatten() {
            f(s);
        }
    }
}

/// 编队阵型：相对长机的固定偏移（NED 米）。
#[derive(Debug, Clone, Copy)]
pub enum Formation {
    /// V 字编队：左右对称后下方（wing 间距 d，纵向回缩 r）。
    V { wing: f32, back: f32, down: f32 },
    /// 一字横队：沿东向均匀铺开。
    Line { spacing: f32 },
    /// 无编队（仅单体/避碰）。
    None,
}

impl Formation {
    /// 本机（role 编号，0=长机）在编队中的相对偏移（NED 米）。
    /// 约定：x=北(前+), y=东(右+), z=上(+)（与内部 pos 单位一致，pos 多为负值表示高度）。
    /// V 字：左右对称展开（y），随 rank 向后(x-)与向下(z-)收拢。
    pub fn slot_offset(&self, role: u8) -> [f32; 3] {
        match self {
            Formation::V { wing, back, down } => {
                if role == 0 {
                    [0.0, 0.0, 0.0]
                } else {
                    let side = if role % 2 == 1 { 1.0 } else { -1.0 };
                    let rank = ((role + 1) / 2) as f32;
                    [-back * rank, side * wing * rank, -down * rank]
                }
            }
            Formation::Line { spacing } => {
                if role == 0 {
                    [0.0, 0.0, 0.0]
                } else {
                    [0.0, (role as f32) * spacing, 0.0]
                }
            }
            Formation::None => [0.0; 3],
        }
    }
}

/// 编队控制器：包装基底 `Controller`，把"长机位置 + 自身编队偏移"作为设定点，
/// 并叠加邻居斥力（避碰）。
pub struct FormationController<B: Controller, const N: usize> {
    base: B,
    formation: Formation,
    leader_id: u8,
    self_id: u8,
    self_role: u8,
    neighbors: SwarmTable<N>,
    /// 避碰安全距离（米）。
    safe_dist: f32,
    /// 斥力增益（有界）。
    sep_gain: f32,
}

impl<B: Controller, const N: usize> FormationController<B, N> {
    pub fn new(base: B, formation: Formation, leader_id: u8, self_id: u8, self_role: u8, safe_dist: f32, sep_gain: f32) -> Self {
        Self {
            base,
            formation,
            leader_id,
            self_id,
            self_role,
            neighbors: SwarmTable::new(),
            safe_dist,
            sep_gain,
        }
    }

    pub fn neighbors(&mut self) -> &mut SwarmTable<N> {
        &mut self.neighbors
    }

    /// 由自身当前状态 + 邻居表，生成"编队 + 避碰"后的设定点。
    fn formation_setpoint(&self, self_state: &VehicleState) -> Setpoint {
        // 长机位置（未知则用自身位置，退化为定点）。
        let leader_pos = self
            .neighbors
            .get(self.leader_id)
            .map(|n| n.pos)
            .unwrap_or([self_state.pos[0].0, self_state.pos[1].0, self_state.pos[2].0]);

        // 编队期望位置 = 长机位置 + 本机编队偏移。
        let off = self.formation.slot_offset(self.self_role);
        let mut desired = [
            leader_pos[0] + off[0],
            leader_pos[1] + off[1],
            leader_pos[2] + off[2],
        ];

        // 避碰斥力：对每架邻居，若距离 < safe_dist 则沿远离方向推。
        let mut push = [0.0f32; 3];
        self.neighbors.for_each(|n| {
            if n.id == self.self_id {
                return;
            }
            let d = [
                self_state.pos[0].0 - n.pos[0],
                self_state.pos[1].0 - n.pos[1],
                self_state.pos[2].0 - n.pos[2],
            ];
            let dist = crate::math::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).max(1e-3);
            if dist < self.safe_dist {
                let mag = self.sep_gain * (self.safe_dist - dist) / self.safe_dist;
                push[0] += mag * d[0] / dist;
                push[1] += mag * d[1] / dist;
                push[2] += mag * d[2] / dist;
            }
        });
        desired[0] += push[0];
        desired[1] += push[1];
        desired[2] += push[2];

        Setpoint {
            pos: [Meter(desired[0]), Meter(desired[1]), Meter(desired[2])],
            yaw: Radian(0.0),
            vel: [MeterPerSecond::ZERO; 3],
        }
    }
}

impl<B: Controller, const N: usize> Controller for FormationController<B, N> {
    fn control(&mut self, dt: Second, _sp: &Setpoint, est: &VehicleState) -> ActuatorCmd {
        // 忽略传入的 sp（编队自管理设定点），用编队+避碰生成设定点。
        let sp = self.formation_setpoint(est);
        self.base.control(dt, &sp, est)
    }

    fn reset(&mut self) {
        self.base.reset();
    }
}

/// 把自身状态编码为 MAVLink 帧（按 self_id 作为 sys_id 广播），供邻居接收。
pub fn broadcast_frame(self_id: u8, state: &VehicleState, seq: u8, out: &mut [u8; MAX_FRAME_LEN]) -> usize {
    mavlink::encode_local_pos_from(self_id, state, seq, out)
}

/// 从一帧解析出邻居状态（取 sys_id 与 LOCAL_POSITION_NED 载荷）。
pub fn parse_broadcast(frame: &Frame) -> Option<NeighborState> {
    let (id, payload) = mavlink::decode(frame)?;
    if id != mavlink::msg_id::LOCAL_POSITION_NED {
        return None;
    }
    if payload.len() < 28 {
        return None;
    }
    let rd = |o: usize| f32::from_le_bytes([payload[o], payload[o + 1], payload[o + 2], payload[o + 3]]);
    let sys_id = frame.as_slice().get(3).copied().unwrap_or(0);
    Some(NeighborState::fresh(
        sys_id,
        [rd(4), rd(8), rd(12)],
        [rd(16), rd(20), rd(24)],
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::VehicleConfig;
    use crate::controller::pid::PidController;
    use crate::estimator::Estimator;
    use crate::hal::sensor::{GpsSensor, ImuSensor};

    #[test]
    fn swarm_table_update_and_stale() {
        let mut t: SwarmTable<4> = SwarmTable::new();
        t.update(NeighborState::fresh(2, [1.0, 2.0, -3.0], [0.0; 3]));
        t.update(NeighborState::fresh(3, [4.0; 3], [0.0; 3]));
        assert_eq!(t.count(), 2);
        assert_eq!(t.get(2).unwrap().pos, [1.0, 2.0, -3.0]);
        // 同 id 覆盖。
        t.update(NeighborState::fresh(2, [9.0, 9.0, 9.0], [0.0; 3]));
        assert_eq!(t.count(), 2);
        assert_eq!(t.get(2).unwrap().pos, [9.0, 9.0, 9.0]);
        // 超过 stale 阈值清除。
        t.age_all(5000, 3000);
        assert_eq!(t.count(), 0);
    }

    #[test]
    fn formation_offsets_symmetric() {
        let f = Formation::V { wing: 2.0, back: 1.0, down: 0.5 };
        // 同 rank(1) 的左右翼对称：role1 右(+2)、role2 左(-2)。
        let r1 = f.slot_offset(1);
        let r2 = f.slot_offset(2);
        assert_eq!(r1[1], 2.0); // 右翼东+
        assert_eq!(r2[1], -2.0); // 左翼东-
        // 更深 rank(2) 应相比 rank(1) 更低(更负 z) 且更靠后(更负 x)。
        let r3 = f.slot_offset(3);
        assert!(r3[2] < r1[2], "更深 rank 应更低(更负 z)；r1={:?} r3={:?}", r1, r3);
        assert!(r3[0] < r1[0], "更深 rank 应更靠后(更负 x)；r1={:?} r3={:?}", r1, r3);
        assert_eq!(r1[1], -r2[1]);
    }

    #[test]
    fn formation_holds_and_avoids_collision() {
        // 两机 V 编队：长机(0) 在原点，僚机(1) 应被控到偏移位而非撞向长机。
        let cfg = VehicleConfig::default_quad();
        let _leader: FormationController<PidController, 4> = FormationController::new(
            PidController::from_config(&cfg.ctrl_params()),
            Formation::V { wing: 3.0, back: 2.0, down: 1.0 },
            0, 0, 0, 1.5, 0.5,
        );
        let mut wingman: FormationController<PidController, 4> = FormationController::new(
            PidController::from_config(&cfg.ctrl_params()),
            Formation::V { wing: 3.0, back: 2.0, down: 1.0 },
            0, 1, 1, 1.5, 0.5,
        );
        // 长机广播自身状态 → 僚机入库。
        let leader_state = VehicleState {
            pos: [Meter(0.0), Meter(0.0), Meter(-10.0)],
            vel: [MeterPerSecond::ZERO; 3],
            att: crate::vehicle::Quaternion::IDENTITY,
            omega: [RadianPerSecond::ZERO; 3],
        };
        let mut buf = [0u8; MAX_FRAME_LEN];
        let n = broadcast_frame(0, &leader_state, 0, &mut buf);
        let frame = Frame::from_bytes(&buf[..n]);
        let nb = parse_broadcast(&frame).unwrap();
        wingman.neighbors().update(nb);

        // 僚机初始在长机正上方 (会撞)，验证避碰+编队把其推离并趋向偏移位。
        let wing_state = VehicleState {
            pos: [Meter(0.0), Meter(0.0), Meter(-10.0)],
            vel: [MeterPerSecond::ZERO; 3],
            att: crate::vehicle::Quaternion::IDENTITY,
            omega: [RadianPerSecond::ZERO; 3],
        };
        let sp_hover = Setpoint::hover([Meter(0.0); 3], Radian(0.0));
        for _ in 0..200 {
            let cmd = wingman.control(Second(0.01), &sp_hover, &wing_state);
            // 指令有界。
            for m in cmd.motor.iter() {
                assert!(*m >= 0.0 && *m <= 1.0);
            }
            // 简化"位置随指令移动"近似：用指令均值作为推力，向编队位推进。
            // 这里只验证编队设定点生成（不引入动力学），检查期望位含偏移。
        }
        // 编队设定点应把僚机推向东翼（offset[1]=+3）并略低于长机（offset[2]=-1）。
        let sp = wingman.formation_setpoint(&wing_state);
        assert!(sp.pos[1].0 > 1.0, "僚机编队设定点应东移（V 右翼），得到 {}", sp.pos[1].0);
        assert!(sp.pos[2].0 < -10.0, "僚机应略低于长机(z 更负)，得到 {}", sp.pos[2].0);
        assert!(sp.pos[0].0 < 0.0, "僚机应略落后长机(x 更负)，得到 {}", sp.pos[0].0);
    }
}
