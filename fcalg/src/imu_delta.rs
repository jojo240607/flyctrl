//! L3 · `ImuDelta` 与时间戳（契约 §3）
//!
//! # 模块契约
//!
//! **输入**：原始样本 + **硬件时间戳**（u32 tick 计数）+ tick 率（构造时给定）。
//! **输出**：`ImuDelta` = 本拍增量（Δang/Δvel，机体系）+ **双 dt**（`dt_ang`/`dt_vel`）+ 戳。
//!
//! # 三条硬性质（本模块存在的理由）
//!
//! 1. **dt 来自相邻硬件戳之差**，**不是名义周期** —— 调度抖动不得改变积分用的 dt；
//! 2. **双口径同时给出**（本例同源，故相等；保留两字段使将来"陀螺/加计不同率"时不必改契约）；
//! 3. **任何"可疑的时间基"一律拒绝 + 计数，绝不静默使用**：
//!    - 首样本：无前戳 ⇒ 只建时间基（**不计入 dropped**，它不是丢弃而是初始化）；
//!    - `ts == last`：`ZeroDt`；
//!    - `ts < last`：**`WrapAmbiguous`** —— 32 位计数回绕与"时间戳倒退"在数值上不可分辨，
//!      故一律拒绝并要求上层重建（**不做"假设是回绕就加 2³²"的静默修补** ✗）；
//!    - `dt > max_dt`：`Stale`（链路停顿/迟到）；
//!    - 非有限输入：`NonFinite`，且**不推进时间基**（坏样本不得污染 `last`）。
//!
//! **`Stale` 会自动重锚时间基**（`last = ts`）并计入 `resyncs`：前向推进是**可知**的
//! ⇒ 只丢本拍增量，不丢时间基。否则一次停顿会让构造器**永久卡死**
//! （后续每拍都超阈 ⇒ 全被拒 ⇒ `accepted` 恒 0），实测复现过。
//! 而 `WrapAmbiguous`（戳倒退/回绕，不可分辨）**不自动重锚** —— 由上层用 [`DeltaBuilder::rebuild`]
//! 显式重建（上层才知道计数是否真的回绕了）。

#[allow(unused_imports)]
use crate::math::F32Ext;

use crate::finite::{gate_all, Stage, Violation};

/// 单拍增量（机体系）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuDelta {
    /// 角增量（rad，机体系）。
    pub delta_ang: [f32; 3],
    /// 速度增量（m/s，机体系；= 比力 × dt）。
    pub delta_vel: [f32; 3],
    /// 角增量对应的实际 dt（s）—— 由硬件戳差得到。
    pub dt_ang: f32,
    /// 速度增量对应的实际 dt（s）。
    pub dt_vel: f32,
    /// 本样本的硬件戳。
    pub ts_ticks: u32,
}

/// 拒绝原因（**显式**，不静默）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reject {
    /// 首样本：建立时间基（不计入 dropped）。
    First,
    /// 与上一戳相同 ⇒ dt = 0。
    ZeroDt,
    /// 戳倒退或 32 位回绕（不可分辨）⇒ 拒绝并要求上层重建。
    WrapAmbiguous,
    /// dt 超过 `max_dt`（链路停顿/迟到）。
    Stale,
    /// 输入非有限。
    NonFinite(Violation),
}

/// 增量构造器（持有时间基与计数）。
pub struct DeltaBuilder {
    ticks_per_second: u32,
    max_dt: f32,
    last: Option<u32>,
    dropped: u32,
    accepted: u64,
    resyncs: u32,
}

impl DeltaBuilder {
    /// `max_dt_s`：允许的最大 dt；超过视为链路停顿（按实际传感器率取，如 5×名义周期）。
    pub fn new(ticks_per_second: u32, max_dt_s: f32) -> Self {
        Self {
            ticks_per_second: ticks_per_second.max(1),
            max_dt: max_dt_s,
            last: None,
            dropped: 0,
            accepted: 0,
            resyncs: 0,
        }
    }

    /// 投入一个样本。成功返回增量；否则返回**显式的**拒绝原因。
    pub fn push(
        &mut self,
        ts_ticks: u32,
        gyro: [f32; 3],
        accel: [f32; 3],
    ) -> Result<ImuDelta, Reject> {
        // 契约 §4：先过有限性门。坏样本 **不推进 last**（时间基不得被污染）。
        let fin = gate_all(Stage::L3ImuDelta, &gyro).and_then(|_| gate_all(Stage::L3ImuDelta, &accel));
        if let Err(v) = fin {
            self.dropped += 1;
            return Err(Reject::NonFinite(v));
        }

        let Some(last) = self.last else {
            self.last = Some(ts_ticks);
            return Err(Reject::First); // 初始化，不算丢弃
        };

        if ts_ticks == last {
            self.dropped += 1;
            return Err(Reject::ZeroDt);
        }
        if ts_ticks < last {
            // 回绕 vs 倒退：数值上不可分辨 ⇒ 拒绝（不猜、不修）
            self.dropped += 1;
            return Err(Reject::WrapAmbiguous);
        }

        let dt = (ts_ticks - last) as f32 / self.ticks_per_second as f32;
        if !(dt.is_finite() && dt <= self.max_dt) {
            self.dropped += 1;
            // 重锚：见模块头 —— 不重锚会永久卡死
            self.last = Some(ts_ticks);
            self.resyncs += 1;
            return Err(Reject::Stale);
        }

        self.last = Some(ts_ticks);
        self.accepted += 1;

        let mut d = ImuDelta {
            delta_ang: [0.0; 3],
            delta_vel: [0.0; 3],
            dt_ang: dt,
            dt_vel: dt,
            ts_ticks,
        };
        for k in 0..3 {
            d.delta_ang[k] = gyro[k] * dt;
            d.delta_vel[k] = accel[k] * dt;
        }
        Ok(d)
    }

    /// 被拒绝的样本数（不含首样本）。
    pub fn dropped(&self) -> u32 {
        self.dropped
    }

    /// 被接受的样本数。
    pub fn accepted(&self) -> u64 {
        self.accepted
    }

    /// 时间基被重锚的次数（`Stale` 自动重锚 + 显式 `rebuild`）。
    pub fn resyncs(&self) -> u32 {
        self.resyncs
    }

    /// **显式**重锚时间基（回绕/重建链路后由上层调用；上层才知道是否真的回绕了）。
    pub fn rebuild(&mut self, ts_ticks: u32) {
        self.last = Some(ts_ticks);
        self.resyncs += 1;
    }

    /// 是否已建立时间基。
    pub fn primed(&self) -> bool {
        self.last.is_some()
    }
}
