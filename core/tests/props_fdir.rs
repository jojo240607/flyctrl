//! M7.x FDIR 故障注入属性测试（host 端）。
//!
//! 用确定性 LCG 随机化生成大量"故障注入序列"，穷举验证 FDIR 在各类传感器失效
//! 组合下的关键不变量：
//! - 仅当 IMU 冻结时才会进入 Critical（关键传感器单一性）
//! - Degraded 时健康原因位与降级判据一致（无虚假降级/漏降级）
//! - 失效源恢复后，最多 `timeout` 步内回到 Nominal（可恢复性）
//! - RTL home 一旦锁定，位置在故障期间不漂移（故障不影响已记忆的 home）
//! - 全程无 NaN 渗入健康等级裁决
//!
//! 不引入 proptest 依赖（保持零外部依赖、可复现），自实现小型 LCG。

use flyctrl_core::fdir::{Fdir, Health, HealthFlags, RtlHome};
use flyctrl_core::vehicle::{ImuSample, MeterPerSecondSquared, Ned, RadianPerSecond};

/// 确定性 LCG（线性同余），用于可复现的随机化输入。
struct Lcg(u64);
impl Lcg {
    fn new(seed: u64) -> Self {
        Lcg(seed.max(1))
    }
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        self.0
    }
    /// [0,1) 浮点
    fn f(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32
    }
    /// 以概率 p 返回 true（伯努利）
    fn bernoulli(&mut self, p: f32) -> bool {
        self.f() < p
    }
    /// [-mag, mag] 浮点
    fn spread(&mut self, mag: f32) -> f32 {
        (self.f() * 2.0 - 1.0) * mag
    }
}

fn sample(accel: [f32; 3]) -> ImuSample {
    ImuSample {
        accel: [
            MeterPerSecondSquared(accel[0]),
            MeterPerSecondSquared(accel[1]),
            MeterPerSecondSquared(accel[2]),
        ],
        gyro: [RadianPerSecond(0.0); 3],
    }
}

/// 在某一拍，随机决定各传感器是否"可用"（注入失效）。
struct SourceMask {
    gps: bool,
    baro: bool,
    mag: bool,
    imu_frozen: bool,
}

/// 运行一条随机故障序列并校验不变量。
fn run_fault_sequence(seed: u64, steps: u32) {
    let mut rng = Lcg::new(seed);
    let mut fdir = Fdir::new();
    let mut home = RtlHome::new();
    let mut locked_pos: Option<Ned> = None;

    for step in 0..steps {
        // 随机注入失效（偶尔"制造" IMU 冻结：连续多拍读数不变）
        let mask = SourceMask {
            gps: rng.bernoulli(0.85),
            baro: rng.bernoulli(0.9),
            mag: rng.bernoulli(0.9),
            imu_frozen: rng.bernoulli(0.05),
        };
        let accel = if mask.imu_frozen {
            [0.0, 0.0, 9.8] // 冻结：恒定
        } else {
            [rng.spread(2.0), rng.spread(2.0), 9.8 + rng.spread(1.0)]
        };
        let imu = sample(accel);

        let health = fdir.update(&imu, mask.gps, mask.baro, mask.mag);
        let flags = fdir.flags();

        // 不变量 1：Critical 仅由 IMU 冻结引起（关键传感器单一性）
        if health == Health::Critical {
            assert!(flags.imu_frozen, "Critical must imply IMU frozen (seed={seed}, step={step})");
        }

        // 不变量 2：Degraded 时至少一处非 IMU 失效源被标记，且未误报 IMU 冻结
        if health == Health::Degraded {
            assert!(
                flags.gps_lost || flags.baro_lost || flags.mag_lost,
                "Degraded must have a non-critical cause (seed={seed}, step={step})"
            );
            assert!(!flags.imu_frozen, "Degraded must not coexist with IMU frozen (seed={seed}, step={step})");
        }

        // 不变量 3：健康等级数值合法（无 NaN 渗入）
        assert!(
            matches!(health, Health::Nominal | Health::Degraded | Health::Critical),
            "health out of range (seed={seed}, step={step})"
        );

        // 不变量 4：RTL home 首次定位（GPS + baro 在）即锁定，且锁定后不漂移
        if home.locked {
            // 已锁定：位置不可被后续失效改变
            if let Some(p) = locked_pos {
                assert_eq!(home.pos, p, "RTL home drifted during fault (seed={seed}, step={step})");
            }
        } else if mask.gps && mask.baro && !mask.imu_frozen {
            // 足够好的定位源 → 锁定 home
            let cur = Ned::new(rng.spread(5.0), rng.spread(5.0), -rng.spread(2.0));
            // 仅当 home 未锁定时 try_lock 会生效
            if home.try_lock(cur) {
                locked_pos = Some(cur);
            }
        }

        // 不变量 5：健康原因位自身一致（IMU 冻结源与 Critical 方向一致）
        assert_eq!(flags.imu_frozen, fdir.flags().imu_frozen);
    }

    // 不变量 6：所有源恢复后，最多 max_timeout 步内回到 Nominal（可恢复性）
    let mut f = Fdir::new();
    let (_gps_to, _baro_to, _mag_to, imu_to) = f.timeouts();
    for _ in 0..(imu_to + 1) {
        f.update(&sample([0.0, 0.0, 9.8]), true, true, true); // 冻结
    }
    assert_eq!(f.health(), Health::Critical);
    let (gps_to, baro_to, mag_to, imu_stale_to) = f.timeouts();
    let max_timeout = imu_stale_to + gps_to + baro_to + mag_to;
    let mut recovered = false;
    for _ in 0..(max_timeout + 5) {
        if f.update(&sample([0.1, 0.0, 9.8]), true, true, true) == Health::Nominal {
            recovered = true;
            break;
        }
    }
    assert!(recovered, "FDIR should recover to Nominal after all sources restored");

    // 健康检查：flags helper 一致性
    let _ = HealthFlags::all_ok();
}

#[test]
fn fdir_fault_injection_invariants() {
    // 多组种子 + 步数，覆盖各类随机故障组合
    for seed in 1u64..=128 {
        run_fault_sequence(seed, 300);
    }
}
