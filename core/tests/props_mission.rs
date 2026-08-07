//! M9 任务层 / 飞行模式属性测试（host 端）。
//!
//! 用确定性 LCG 随机化穷举，验证三类性质：
//! - 任务执行器：到达即推进、索引单调不减、且永不越过容量。
//! - 地理围栏：任何输入夹取后必在围栏内（水平≤radius、垂直∈[ceil,floor]）。
//! - 模式治理：非法切换被拒且当前模式不变；严重故障只能转 Land。
//!
//! 不引入 proptest（保持零外部依赖），自实现小型 LCG。

use flyctrl_core::fdir::Health;
use flyctrl_core::flightmode::{FlightMode, ModeContext, ModeGovernor};
use flyctrl_core::mission::{Geofence, Mission, MissionRunner, Waypoint};
use flyctrl_core::units::*;
use flyctrl_core::vehicle::VehicleState;

/// 确定性 LCG（与 props_invariant.rs 同款可复现随机源）。
struct Lcg(u64);
impl Lcg {
    fn new(seed: u64) -> Self {
        Lcg(seed.max(1))
    }
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.0
    }
    fn f(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32
    }
    fn spread(&mut self, mag: f32) -> f32 {
        (self.f() * 2.0 - 1.0) * mag
    }
}

#[test]
fn prop_mission_advances_monotonic_and_bounded() {
    // 随机生成一串航点，喂随机但"逐点逼近"的估计状态，验证索引单调不减、不越界。
    let mut rng = Lcg::new(0x51a9_1357);
    for _ in 0..200 {
        let n = 1 + (rng.next() % 6) as usize; // 1..6 个航点
        let mut wps = Vec::new();
        for i in 0..n {
            wps.push(Waypoint::new(
                [Meter(rng.spread(40.0)), Meter(rng.spread(40.0)), Meter(-10.0 + rng.spread(5.0))],
                Radian(rng.spread(3.14)),
                Meter(0.5 + rng.f() * 2.0),
                Meter(0.5 + rng.f() * 2.0),
            ));
            let _ = i;
        }
        let mission = Mission::<8>::from_slice(&wps);
        let mut runner = MissionRunner::new(mission, Geofence::default_quad());

        let mut prev_idx = runner.current_index();
        // 模拟飞行：每个时间步随机抖动估计位置，但偶尔精确落在当前航点。
        for step in 0..50 {
            let mut s = VehicleState::zero();
            let cur = runner.current_index();
            if cur < mission.len() {
                let w = mission.get(cur).unwrap();
                if step % 3 == 0 {
                    // 精确到达
                    s.pos = w.pos;
                } else {
                    // 附近抖动
                    s.pos = [
                        Meter(w.pos[0].0 + rng.spread(w.radius.0)),
                        Meter(w.pos[1].0 + rng.spread(w.radius.0)),
                        Meter(w.pos[2].0 + rng.spread(w.alt_tol.0)),
                    ];
                }
            } else {
                s.pos = [Meter(0.0), Meter(0.0), Meter(-10.0)];
            }
            let _ = runner.update(&s, Second(0.02));
            let now = runner.current_index();
            assert!(now >= prev_idx, "任务索引必须单调不减");
            assert!(now <= mission.len(), "任务索引不得越过容量");
            prev_idx = now;
        }
    }
}

#[test]
fn prop_geofence_clamp_always_inside() {
    let mut rng = Lcg::new(0x7e0a_91c3);
    let f = Geofence::default_quad();
    for _ in 0..5000 {
        let raw = [
            Meter(rng.spread(500.0)),
            Meter(rng.spread(500.0)),
            Meter(rng.spread(500.0)),
        ];
        let (c, _) = f.clamp(raw);
        let dx = c[0].0 - f.center[0].0;
        let dy = c[1].0 - f.center[1].0;
        let horiz = (dx * dx + dy * dy).sqrt();
        assert!(horiz <= f.radius.0 + 1e-3, "夹取后水平距离必须 ≤ 围栏半径");
        // D 向下为正：ceil 是更小的值（更高），floor 是更大的值（更低）。
        assert!(c[2].0 >= f.ceil.0 - 1e-3, "夹取后高度不得低于 ceil（上限）");
        assert!(c[2].0 <= f.floor.0 + 1e-3, "夹取后高度不得高于 floor（下限）");
    }
}

#[test]
fn prop_mode_governor_rejects_illegal_and_keeps_mode() {
    let mut rng = Lcg::new(0x2b3c_4d5e);
    // 覆盖所有 (armed, health, pos) 组合的随机切换请求。
    for _ in 0..3000 {
        let armed = rng.next() & 1 == 1;
        let health = match rng.next() % 3 {
            0 => Health::Nominal,
            1 => Health::Degraded,
            _ => Health::Critical,
        };
        let pos = rng.next() & 1 == 1;
        let ctx = ModeContext::new(armed, health, pos);

        // 随机初始模式
        let init = match rng.next() % 7 {
            0 => FlightMode::Manual,
            1 => FlightMode::Stabilize,
            2 => FlightMode::Altitude,
            3 => FlightMode::Position,
            4 => FlightMode::Mission,
            5 => FlightMode::Rtl,
            _ => FlightMode::Land,
        };
        let mut g = ModeGovernor::new(init);
        let before = g.mode();

        // 随机目标模式
        let target = match rng.next() % 7 {
            0 => FlightMode::Manual,
            1 => FlightMode::Stabilize,
            2 => FlightMode::Altitude,
            3 => FlightMode::Position,
            4 => FlightMode::Mission,
            5 => FlightMode::Rtl,
            _ => FlightMode::Land,
        };

        let allowed = g.can_enter(target, &ctx);
        let req_ok = g.request(target, &ctx);

        if allowed {
            assert!(req_ok, "can_enter 批准则应 request 成功");
            assert_eq!(g.mode(), target, "批准后模式应更新为目标");
        } else {
            assert!(!req_ok, "can_enter 拒绝则应 request 失败");
            assert_eq!(g.mode(), before, "拒绝后模式必须保持不变");
        }

        // 严重故障：若当前是 Land，保持 Land；否则拒绝任何非 Land 申请。
        if health == Health::Critical {
            if g.mode() != FlightMode::Land {
                assert!(
                    !g.can_enter(FlightMode::Position, &ctx),
                    "Critical 下不得进入 Position"
                );
                assert!(
                    g.can_enter(FlightMode::Land, &ctx),
                    "Critical 下必须允许 Land"
                );
            }
        }
    }
}

#[test]
fn prop_mode_degrade_is_safe() {
    // Degraded 把 Mission→Rtl；Critical 把任意→Land；Nominal 不变。
    let mut rng = Lcg::new(0x0bad_c0de);
    for _ in 0..1000 {
        let health = match rng.next() % 3 {
            0 => Health::Nominal,
            1 => Health::Degraded,
            _ => Health::Critical,
        };
        let cur = match rng.next() % 7 {
            0 => FlightMode::Manual,
            1 => FlightMode::Stabilize,
            2 => FlightMode::Altitude,
            3 => FlightMode::Position,
            4 => FlightMode::Mission,
            5 => FlightMode::Rtl,
            _ => FlightMode::Land,
        };
        let mut g = ModeGovernor::new(cur);
        let ctx = ModeContext::new(true, health, true);
        let changed = g.degrade_on_health(&ctx);
        match health {
            Health::Nominal => {
                assert!(!changed);
                assert_eq!(g.mode(), cur);
            }
            Health::Degraded => {
                if cur == FlightMode::Mission {
                    assert!(changed);
                    assert_eq!(g.mode(), FlightMode::Rtl);
                }
            }
            Health::Critical => {
                assert_eq!(g.mode(), FlightMode::Land);
            }
        }
    }
}
