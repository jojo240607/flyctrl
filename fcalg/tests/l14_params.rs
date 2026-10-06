//! L14 验收 —— 出处必填且**不许编**（欠债可数）、区间自洽、**无重名**（旧栈 vmax_xy 那类
//! 缺陷的判据）、以及**表与代码默认值必须一致**（否则表就成了并行的第二份真值）。
use fcalg::align::AlignConfig;
use fcalg::observe::ObsParams;
use fcalg::params::{chosen_count, param, Source, PARAMS};
/// 出处必填且**不许编**：每条都必须有非空出处说明。
#[test]
fn every_param_has_non_empty_provenance() {
    for m in PARAMS {
        assert!(!m.name.is_empty(), "参数名不得为空");
        assert!(
            !m.source.note().is_empty(),
            "{} 的出处说明不得为空（出处是必填字段，不是注释）",
            m.name
        );
    }
    // 出处分布必须可统计；「本重建选定」必须显式标出（不许冒充一手/实测）
    let c = chosen_count();
    assert_eq!(c.total, PARAMS.len());
    assert!(c.debt > 0, "尚未经实测替换的值必须显式计为欠债");
    assert!(c.debt < c.total, "不应全部都是欠债（至少有些来自旧栈注释/推导）");
    for m in PARAMS {
        if matches!(m.source, Source::Chosen(_)) {
            assert!(m.source.is_debt());
        } else {
            assert!(!m.source.is_debt());
        }
    }
}
/// 区间自洽：值必须在 [lo, hi] 内，且 lo/hi 有限、lo < hi。
#[test]
fn every_param_is_finite_and_within_declared_range() {
    for m in PARAMS {
        assert!(m.value.is_finite(), "{} 的值必须有限", m.name);
        assert!(m.lo.is_finite() && m.hi.is_finite(), "{} 的区间必须有限", m.name);
        assert!(m.lo < m.hi, "{} 的区间必须是 lo<hi", m.name);
        assert!(
            m.value >= m.lo && m.value <= m.hi,
            "{} = {} 越界 [{}, {}]",
            m.name,
            m.value,
            m.lo,
            m.hi
        );
    }
}
/// **无重名**：同名两条不同值 = 旧栈 `vmax_xy`（默认值不一致）那类缺陷的温床。
#[test]
fn table_has_no_duplicate_names() {
    for (i, a) in PARAMS.iter().enumerate() {
        for b in PARAMS.iter().skip(i + 1) {
            assert_ne!(a.name, b.name, "参数名重复：{}", a.name);
        }
    }
}
/// **表与代码必须一致**：各模块的默认值不得与之各说一套（使本表成为可审计的真值面）。
#[test]
fn table_agrees_with_module_defaults() {
    let o = ObsParams::default();
    assert_eq!(param("obs.sigma_baro").unwrap().value, o.sigma_baro);
    assert_eq!(param("obs.sigma_gps_p").unwrap().value, o.sigma_gps_p);
    assert_eq!(param("obs.sigma_gps_v").unwrap().value, o.sigma_gps_v);
    // ★这一条本该在 L17 之前就存在：表里有 obs.sigma_mag，而结构体里没有对应字段
    //   ⇒ 表与代码"各说一套"而判据放过去了。补上，使这类缺口不可能再漏。
    assert_eq!(param("obs.sigma_mag").unwrap().value, o.sigma_mag);
    let a = AlignConfig::default();
    assert_eq!(param("align.g_tol_frac").unwrap().value, a.g_tol_frac);
    // 过程噪声也必须"表=代码"（否则 Q 会变成第二个真值源）
    let q = fcalg::filter::ProcessNoise::default();
    assert_eq!(param("q.att").unwrap().value, q.q_att);
    assert_eq!(param("q.vel").unwrap().value, q.q_vel);
    assert_eq!(param("q.pos").unwrap().value, q.q_pos);
    assert_eq!(param("q.bg").unwrap().value, q.q_bg);
    assert_eq!(param("q.ba").unwrap().value, q.q_ba);
    assert_eq!(param("q.mag_i").unwrap().value, q.q_mag_i);
    assert_eq!(param("q.mag_b").unwrap().value, q.q_mag_b);
}
/// 查表接口 + 单位必填（`Unit` 无"未知"变体 ⇒ 漏标在类型上不可能，这里只验查得到）。
#[test]
fn lookup_works_and_unit_is_always_tagged() {
    assert!(param("delta.max_dt").is_some());
    assert!(param("does.not.exist").is_none());
    for m in PARAMS {
        // 单位标签是枚举（无"未知"）⇒ 结构上必填；这里断言已进入匹配（不会 panic）
        match m.unit {
            fcalg::params::Unit::Sec
            | fcalg::params::Unit::Hertz
            | fcalg::params::Unit::Meters
            | fcalg::params::Unit::Mps
            | fcalg::params::Unit::Rad
            | fcalg::params::Unit::Rps
            | fcalg::params::Unit::RpsPerS
            | fcalg::params::Unit::PerSec
            | fcalg::params::Unit::Ratio
            | fcalg::params::Unit::Count
            | fcalg::params::Unit::VarPerSec => {}
        }
    }
}
/// **欠债棘轮**：`Chosen`（本重建选定）的条数**只应下降**。
/// 修完一条就把下面的常数改小（并说明为何能兑现）；**上升则必须显式改大并写理由**。
/// 目的：让“没依据的值”的数字既**诚实**、又**不会悄悄增长**。
const RECORDED_DEBT: usize = 4;
#[test]
fn debt_is_ratcheted() {
    let c = chosen_count();
    assert!(
        c.debt <= RECORDED_DEBT,
        "欠债增加了：{} > 记录值 {RECORDED_DEBT}（若要增加必须显式改这个常数并写理由）",
        c.debt
    );
    // 本轮兑现了 3 条（均为本仓一手物证），债务由 7 降到 4
    assert_eq!(c.debt, RECORDED_DEBT, "兑现后请同步下调常数（已兑现就必须记账）");
    assert_eq!(c.total, PARAMS.len());
}
/// **非欠债条目必须指到具体物证**（文件/行、台账条目、或可解析推导）——
/// 不让"有依据"退化成一句模糊的说法。
#[test]
fn repaid_entries_cite_a_concrete_artifact() {
    const MARKERS: [&str; 7] = [".rs", ".c", ".yaml", "§", "台账", "解析", "源码"];
    for m in PARAMS {
        if m.source.is_debt() {
            continue;
        }
        let n = m.source.note();
        assert!(
            MARKERS.iter().any(|k| n.contains(k)),
            "{} 声称有依据，但出处未指到具体物证（文件/行、台账、或解析推导）：{n}",
            m.name
        );
    }
}
/// **欠债必须可行动**：每条 `Chosen` 都要写清"需要什么才能兑现"，不能只写"待定"。
#[test]
fn every_chosen_entry_states_its_blocker() {
    const BLOCKERS: [&str; 3] = ["需", "待", "不可"];
    for m in PARAMS {
        if !m.source.is_debt() {
            continue;
        }
        let n = m.source.note();
        assert!(
            BLOCKERS.iter().any(|k| n.contains(k)),
            "{} 是欠债，但没写清兑现条件（需/待/不可）：{n}",
            m.name
        );
    }
}
