//! 飞行模式（Flight Modes）与模式治理（Mode Governance）。
//!
//! 与 [`crate::state::Fcs`] 的类型级状态机正交：Fcs 管"锁定/解锁/失控保护"
//! 这种**安全生命周期**，本模块管"当前由谁生成设定点"这种**运行模态**
//! （Manual/Stabilize/Altitude/Position/Mission/Rtl/Land）。
//!
//! 模式选择通常是地面站实时切换的（运行时值），无法用类型参数编码，因此这里
//! 用 `FlightMode` 枚举 + [`ModeGovernor`] 的**运行时守卫**来实现：
//! - 合法流转表（如未解锁不能进 Position/Mission；未定位不能进 Position/Mission）。
//! - FDIR 联动：降级时禁止高风险自主模式（Mission/Position），严重故障时强制
//!   RTL/Land（与 [`crate::fdir::Health`] 配合，形成"健康→权限"退化链）。

use crate::fdir::Health;

/// 飞行模式。
///
/// 权限等级（`authorization_level`）随自主性升高：Manual 最低，Mission 最高。
/// 这是 [`ModeGovernor::can_enter`] 的核心判据之一。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightMode {
    /// 手动角速率/姿态（RC 直通），不依赖任何估计。
    Manual,
    /// 自稳（姿态保持），需姿态估计。
    Stabilize,
    /// 定高（高度保持 + 手动水平），需姿态 + 高度估计。
    Altitude,
    /// 定点（水平位置保持），需完整位置估计（GPS/光流）。
    Position,
    /// 任务（自动航点），需位置估计 + 已加载任务。
    Mission,
    /// 返航（回到起飞点并盘旋/降落）。
    Rtl,
    /// 降落（自动降高至地面）。
    Land,
}

impl FlightMode {
    /// 自主性授权等级（0=最低，6=最高）。用于"降权"比较。
    pub fn authorization_level(&self) -> u8 {
        match self {
            FlightMode::Manual => 0,
            FlightMode::Stabilize => 1,
            FlightMode::Altitude => 2,
            FlightMode::Position => 3,
            FlightMode::Mission => 5,
            FlightMode::Rtl => 4,
            FlightMode::Land => 3,
        }
    }

    /// 是否需要完整位置估计（GPS/光流）。
    pub fn requires_position(&self) -> bool {
        matches!(self, FlightMode::Position | FlightMode::Mission | FlightMode::Rtl)
    }

    /// 是否需要已解锁（Armed）。
    pub fn requires_armed(&self) -> bool {
        !matches!(self, FlightMode::Manual) // 手动可在锁定态用于测试，但语义上其余都需解锁
    }

    /// 是否为自主（非人工直接操控）模式——FDIR 严重故障时应强制退出。
    pub fn is_autonomous(&self) -> bool {
        matches!(self, FlightMode::Mission | FlightMode::Rtl | FlightMode::Land)
    }
}

/// 模式治理上下文：当前健康、是否已解锁、是否具备位置估计能力。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ModeContext {
    pub armed: bool,
    pub health: Health,
    pub position_available: bool,
}

impl ModeContext {
    pub fn new(armed: bool, health: Health, position_available: bool) -> Self {
        Self { armed, health, position_available }
    }
}

/// 模式治理器：持有当前模式，按 [`ModeContext`] 校验切换请求，
/// 并在健康恶化时主动降级（degrade）。
#[derive(Debug, Clone, Copy)]
pub struct ModeGovernor {
    mode: FlightMode,
}

impl ModeGovernor {
    pub fn new(initial: FlightMode) -> Self {
        Self { mode: initial }
    }

    /// 当前模式。
    pub fn mode(&self) -> FlightMode { self.mode }

    /// 判断在给定上下文中能否进入 `target` 模式。
    ///
    /// 规则：
    /// 1. 未解锁且 `target.requires_armed()` → 拒绝。
    /// 2. 需要位置但无位置估计 → 拒绝（除非目标是更安全的 Manual/Stabilize/Altitude/Land）。
    /// 3. 健康为 `Critical` → 只允许 `Land`（或保持当前，若已是 Land）。
    /// 4. 健康为 `Degraded` → 禁止最高自主 `Mission`（退到 Rtl/Position 等）。
    pub fn can_enter(&self, target: FlightMode, ctx: &ModeContext) -> bool {
        // 严重故障：只能降落。
        if ctx.health == Health::Critical {
            return target == FlightMode::Land;
        }
        // 未解锁拒绝需要解锁的模式。
        if target.requires_armed() && !ctx.armed {
            return false;
        }
        // 需要位置但无位置估计：拒绝高风险位置模式。
        if target.requires_position() && !ctx.position_available {
            return false;
        }
        // 降级：禁最高自主 Mission（避免带着不确定状态跑复杂任务）。
        if ctx.health == Health::Degraded && target == FlightMode::Mission {
            return false;
        }
        true
    }

    /// 尝试切换到 `target`；非法则返回 `false` 且保持当前模式不变。
    pub fn request(&mut self, target: FlightMode, ctx: &ModeContext) -> bool {
        if self.can_enter(target, ctx) {
            self.mode = target;
            true
        } else {
            false
        }
    }

    /// 健康恶化时的**主动降级**：返回应转入的安全模式，并就地应用。
    ///
    /// - `Critical` → `Land`（立即降落）。
    /// - `Degraded` 且当前为 `Mission` → `Rtl`（返航，比继续任务安全）。
    /// - 其余 → 维持。
    ///
    /// 返回是否发生了降级。
    pub fn degrade_on_health(&mut self, ctx: &ModeContext) -> bool {
        match ctx.health {
            Health::Critical => {
                if self.mode != FlightMode::Land {
                    self.mode = FlightMode::Land;
                    true
                } else {
                    false
                }
            }
            Health::Degraded => {
                if self.mode == FlightMode::Mission {
                    self.mode = FlightMode::Rtl;
                    true
                } else {
                    false
                }
            }
            Health::Nominal => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fdir::Health;

    #[test]
    fn cannot_enter_position_without_fix() {
        let mut g = ModeGovernor::new(FlightMode::Stabilize);
        let ctx = ModeContext::new(true, Health::Nominal, false); // 已解锁但无位置
        assert!(!g.request(FlightMode::Position, &ctx));
        assert_eq!(g.mode(), FlightMode::Stabilize);
        // 有位置后可以
        let ctx2 = ModeContext::new(true, Health::Nominal, true);
        assert!(g.request(FlightMode::Position, &ctx2));
        assert_eq!(g.mode(), FlightMode::Position);
    }

    #[test]
    fn cannot_enter_armed_mode_when_disarmed() {
        let mut g = ModeGovernor::new(FlightMode::Manual);
        let ctx = ModeContext::new(false, Health::Nominal, true);
        assert!(!g.request(FlightMode::Mission, &ctx));
        assert!(!g.request(FlightMode::Altitude, &ctx));
        // 手动允许（requires_armed=false）
        assert!(g.request(FlightMode::Manual, &ctx));
    }

    #[test]
    fn critical_forces_land_only() {
        let mut g = ModeGovernor::new(FlightMode::Mission);
        let ctx = ModeContext::new(true, Health::Critical, true);
        // 任何非 Land 申请都被拒
        assert!(!g.request(FlightMode::Position, &ctx));
        assert!(!g.request(FlightMode::Rtl, &ctx));
        assert!(g.request(FlightMode::Land, &ctx));
        // 主动降级也应把 Mission→Land
        let mut g2 = ModeGovernor::new(FlightMode::Mission);
        assert!(g2.degrade_on_health(&ctx));
        assert_eq!(g2.mode(), FlightMode::Land);
    }

    #[test]
    fn degraded_blocks_mission_but_allows_rtl() {
        let mut g = ModeGovernor::new(FlightMode::Position);
        let ctx = ModeContext::new(true, Health::Degraded, true);
        assert!(!g.request(FlightMode::Mission, &ctx));
        assert!(g.request(FlightMode::Rtl, &ctx));
        // 降级把 Mission→Rtl
        let mut g2 = ModeGovernor::new(FlightMode::Mission);
        assert!(g2.degrade_on_health(&ctx));
        assert_eq!(g2.mode(), FlightMode::Rtl);
    }

    #[test]
    fn authorization_levels_monotonic() {
        assert!(FlightMode::Manual.authorization_level() < FlightMode::Position.authorization_level());
        assert!(FlightMode::Position.authorization_level() < FlightMode::Mission.authorization_level());
    }
}
