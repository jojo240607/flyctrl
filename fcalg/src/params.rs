//! L14 · 参数表（带**单位标签**与**必填出处**）
//! # 它对症的是"约定散落"的第三种形态
//! 旧栈的旋钮满天飞、默认值藏在各处、没人说得清每个值从哪来 ——
//! 最典型的一次缺陷是两个默认值不一致（`vmax_xy` 2.0 vs 3.5），任何扫参都发现不了。
//! # 三条硬规则（本模块的可判据之处）
//! 1. **单位是标签且必填** —— `Unit` 里**没有**"未知"变体 ⇒ 结构上不可能漏；
//! 2. **出处是必填字段**（不是注释）—— 且**不许编**：给不出一手来源/实测的值，
//!    必须显式标成 [`Source::Chosen`]（"本重建选定，待实测/一手来源替换"），
//!    于是**欠债是可数的**（见 `ChosenCount`），不会悄悄增长；
//! 3. **声明区间** —— `lo <= value <= hi`，且 `lo/hi` 自身有限、`lo < hi`。
//! # 与代码的关系
//! 本表**不是**运行时唯一真值源（那样会引入间接层），而是**审计面**：
//! 判据 `cross_module_consistency` 断言各模块的默认值**必须与本表一致** ——
//! 使"表"与"代码"不能各说一套。
/// 单位标签（**没有"未知"变体** ⇒ 漏标在类型上不可能）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Unit {
    Sec,
    Hertz,
    Meters,
    Mps,
    Rad,
    Rps,
    /// rad/s²
    RpsPerS,
    /// 1/s（如姿态环 P 增益）
    PerSec,
    /// 无量纲比
    Ratio,
    /// 计数
    Count,
}
/// 出处（必填）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Source {
    /// 一手来源（源码/规范行号）
    Primary(&'static str),
    /// 实测记录（台账条目）
    Measured(&'static str),
    /// 推导（给出依据）
    Derived(&'static str),
    /// **本重建选定，尚未经一手来源或实测确认** —— 欠债，必须显式
    Chosen(&'static str),
}
impl Source {
    pub fn note(&self) -> &'static str {
        match self {
            Source::Primary(s) | Source::Measured(s) | Source::Derived(s) | Source::Chosen(s) => s,
        }
    }
    pub fn is_debt(&self) -> bool {
        matches!(self, Source::Chosen(_))
    }
}
/// 一条参数。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Meta {
    pub name: &'static str,
    pub value: f32,
    pub unit: Unit,
    pub lo: f32,
    pub hi: f32,
    pub source: Source,
}
/// 参数表（**审计面**；值必须与各模块默认一致 —— 由判据守住）。
pub const PARAMS: &[Meta] = &[
    Meta {
        name: "imu.notch_f0",
        value: 40.0,
        unit: Unit::Hertz,
        lo: 10.0,
        hi: 200.0,
        source: Source::Derived("旧栈 hil.rs 滤波链注释：accel 过 40Hz 陷波"),
    },
    Meta {
        name: "imu.notch_q",
        value: 2.0,
        unit: Unit::Ratio,
        lo: 0.5,
        hi: 10.0,
        source: Source::Chosen("本重建选定；待振动实测标定"),
    },
    Meta {
        name: "imu.accel_lp_fc",
        value: 20.0,
        unit: Unit::Hertz,
        lo: 5.0,
        hi: 100.0,
        source: Source::Derived("旧栈 hil.rs 滤波链注释：accel 20Hz 低通"),
    },
    Meta {
        name: "imu.lp_q",
        value: 0.7071,
        unit: Unit::Ratio,
        lo: 0.5,
        hi: 1.0,
        source: Source::Derived("Butterworth（Q=0.7071）⇒ 阶跃过冲 ≈4.3%，解析可验"),
    },
    Meta {
        name: "delta.max_dt",
        value: 0.02,
        unit: Unit::Sec,
        lo: 0.001,
        hi: 0.5,
        source: Source::Chosen("本重建选定（取名义周期的数倍）"),
    },
    Meta {
        name: "align.g_tol_frac",
        value: 0.06,
        unit: Unit::Ratio,
        lo: 0.01,
        hi: 0.3,
        source: Source::Chosen("本重建选定；旧栈只用\"比力>1.0\"的松判据（已证会拉飞姿态）"),
    },
    Meta {
        name: "obs.sigma_baro",
        value: 0.3,
        unit: Unit::Meters,
        lo: 0.01,
        hi: 10.0,
        source: Source::Derived("旧栈 set_observation_noise(_,_,0.09) 的量级；待实测替换"),
    },
    Meta {
        name: "obs.sigma_gps_p",
        value: 0.5,
        unit: Unit::Meters,
        lo: 0.05,
        hi: 20.0,
        source: Source::Derived("旧栈 set_observation_noise(0.25,_,_) 的量级；待实测替换"),
    },
    Meta {
        name: "obs.sigma_gps_v",
        value: 0.1,
        unit: Unit::Mps,
        lo: 0.01,
        hi: 5.0,
        source: Source::Derived("旧栈 set_observation_noise(_,0.01,_) 的量级；待实测替换"),
    },
    Meta {
        name: "obs.sigma_mag",
        value: 0.3,
        unit: Unit::Ratio,
        lo: 0.01,
        hi: 3.0,
        source: Source::Chosen("本重建选定（航向新息的门限σ）"),
    },
    Meta {
        name: "gate.nis_sigma",
        value: 6.0,
        unit: Unit::Ratio,
        lo: 1.0,
        hi: 12.0,
        source: Source::Chosen("本重建选定；接真传感器后须按 NIS 一致性重标"),
    },
    Meta {
        name: "gate.max_consecutive_rejects",
        value: 10.0,
        unit: Unit::Count,
        lo: 2.0,
        hi: 200.0,
        source: Source::Chosen("本重建选定（旧栈缺失该机制）"),
    },
    Meta {
        name: "gate.reflate_floor",
        value: 1.0,
        unit: Unit::Ratio,
        lo: 0.01,
        hi: 100.0,
        source: Source::Chosen("本重建选定；须与观测噪声量级自洽"),
    },
];
/// 按名查参数。
pub fn param(name: &str) -> Option<&'static Meta> {
    PARAMS.iter().find(|m| m.name == name)
}
/// 「本重建选定」型参数的计数（**欠债指标**：只应随实测替换而下降）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChosenCount {
    pub total: usize,
    pub debt: usize,
}
/// 统计出处分布。
pub fn chosen_count() -> ChosenCount {
    let debt = PARAMS.iter().filter(|m| m.source.is_debt()).count();
    ChosenCount { total: PARAMS.len(), debt }
}
