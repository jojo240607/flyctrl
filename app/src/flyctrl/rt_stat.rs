//! ★design.md §9【可观测性】：每任务**执行时间**统计（min/avg/max）。
//!
//! 设计要点（对齐 design.md："每个任务在线统计执行时间 min/max/avg"）：
//!   · **零浮点**（整数 cycles→µs，避开 `flt2dec` 的高开销，见本仓日志纪律）；
//!   · 每拍只做 3 次比较 + 1 次加法（<1µs），不干扰被观测任务；
//!   · 任务循环内：`let t0 = st.tick();` … `st.sample(st.tick().wrapping_sub(t0));`
//!     每 N 拍 `st.report_and_reset();`。
//!
//! 与 P0-2（内核违约计数）互补：内核给 deadline/wcet **违约**，本模块给每任务
//! **实际耗时分布**（定位"哪一段慢"）。

use rtos_app_sdk::rtos::cycle_now;

/// 168MHz 下 cycles→µs 除数。
/// ★design.md §5：cycles→µs 换算**唯一来源 = 内核实测**（不再硬编码 168 ✗——
/// 实测仿真器 DWT 速率约 84/µs，硬编码会让显示的 µs 偏乐观 2×）。
#[inline]
pub fn cycles_per_us() -> u32 {
    (rtos_app_sdk::abi::slot().cycles_per_ms)
        .and_then(|f| Some(f()))
        .unwrap_or(168_000)
        / 1000
}

pub struct RtStat {
    name: &'static str,
    n: u32,
    exec_min: u32,
    exec_max: u32,
    exec_sum: u64,
    // ★design.md §9：**周期抖动** |实测周期 − 名义周期|（cycles）
    jit_min: u32,
    jit_max: u32,
    jit_sum: u64,
    /// 最近一次 exec（cycles）——供 L1 CPU 预算汇总（§4）。
    last_exec: u32,
}

impl RtStat {
    pub const fn new(name: &'static str) -> Self {
        Self { name, n: 0, exec_min: u32::MAX, exec_max: 0, exec_sum: 0, jit_min: u32::MAX, jit_max: 0, jit_sum: 0, last_exec: 0 }
    }

    /// 取当前 cycle 计数（168MHz DWT CYCCNT）。
    #[inline]
    pub fn tick(&self) -> u32 {
        cycle_now()
    }

    /// 记录一次执行耗时（cycles）。
    #[inline]
    pub fn sample(&mut self, exec_cyc: u32, period_cyc: u32, nominal_cyc: u32) {
        self.n = self.n.wrapping_add(1);
        self.last_exec = exec_cyc;
        if exec_cyc < self.exec_min { self.exec_min = exec_cyc; }
        if exec_cyc > self.exec_max { self.exec_max = exec_cyc; }
        self.exec_sum += exec_cyc as u64;
        // 周期抖动（§9）
        let j = if period_cyc > nominal_cyc { period_cyc - nominal_cyc } else { nominal_cyc - period_cyc };
        if j < self.jit_min { self.jit_min = j; }
        if j > self.jit_max { self.jit_max = j; }
        self.jit_sum += j as u64;
    }

    /// 打印 min/avg/max（µs）并清零。
    /// 最近一次 exec（cycles）。
    pub fn last_exec(&self) -> u32 { self.last_exec }

    pub fn report_and_reset(&mut self) {
        if self.n == 0 {
            return;
        }
        let avg = (self.exec_sum / self.n as u64) as u32;
        let javg = (self.jit_sum / self.n as u64) as u32;
        rtos_app_sdk::info!(tag: "stat", "{} n={} exec_us={}/{}/{} jit_us={}/{}/{}",
            self.name, self.n,
            self.exec_min / cycles_per_us(), avg / cycles_per_us(), self.exec_max / cycles_per_us(),
            self.jit_min / cycles_per_us(), javg / cycles_per_us(), self.jit_max / cycles_per_us());
        self.n = 0;
        self.exec_min = u32::MAX;
        self.exec_max = 0;
        self.exec_sum = 0;
        self.jit_min = u32::MAX;
        self.jit_max = 0;
        self.jit_sum = 0;
    }
}
