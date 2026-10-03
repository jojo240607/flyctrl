//! ★design.md：**传感器 WorkItem**（L2 `wq:sensors`）+ IMU 采样（§3/§7，过渡：仍在此采样，
//! 目标迁 L0 data-ready ISR）。
//!
//! 本文件导出的 `sensors_init()`（装配）+ `sensors_step()`（**一拍**，由 L2 队列 worker 调用）；
//! 状态放模块级静态（WorkItem 无自己的栈）。**不再有独立 sensors 线程** ✓。

use core::ffi::c_void;
use core::mem::MaybeUninit;

use rtos_app_sdk::info;

#[cfg(not(feature = "hil"))]
use crate::flyctrl::{SENSOR_FRAME, SENSOR_SEQ};
#[cfg(not(feature = "hil"))]
use crate::sensors::stack::SensorStack;
#[cfg(not(feature = "hil"))]
use flyctrl_core::vehicle::{ImuSample, PosSample, RcInput};

const WRITE_FRAME: bool = true;
/// 采样周期（IMU 1kHz；design.md §3/§7）。
pub const SAMPLE_DT: f32 = 0.001;
// ★架构变更重标（子代理 §B 建议）：GPS 保持超时改为 **时间基准**（tick 差），
//   不再用"item 调用次数"——否则 item 频率一变（500Hz→250Hz）阈值时间语义就漂。
const GPS_VALID_MS: u32 = 500; // 500 ticks × 1ms = 500ms 无新帧 ⇒ 判真丢失

#[cfg(not(feature = "hil"))]
static mut SENS_STACK: MaybeUninit<SensorStack> = MaybeUninit::uninit();
static mut SENS_FILTERS: MaybeUninit<flyctrl_core::imu_filters::ImuFilters> = MaybeUninit::uninit();
static mut SENS_FIRST: bool = false;
static mut SENS_LOOP_CNT: u32 = 0;
static mut SENS_IMU_PREV_TS: u32 = 0;
static mut SENS_IMU_PREV_HAS: bool = false;
static mut SENS_TOPIC_AF: [f32; 3] = [0.0; 3];
static mut SENS_TOPIC_GV: [f32; 3] = [0.0; 3];
static mut SENS_TOPIC_RG: [f32; 3] = [0.0; 3];
static mut SENS_TOPIC_TS: u32 = 0;
static mut SENS_GPS_STALE: u32 = 0;
/// 最近一次收到有效 GPS 帧的 tick（时间基准超时用）。
static mut SENS_GPS_LAST_TICK: u32 = 0;
#[cfg(not(feature = "hil"))]
static mut SENS_LAST_IMU: ImuSample = ImuSample { accel: [flyctrl_core::units::MeterPerSecondSquared(0.0); 3], gyro: [flyctrl_core::units::RadianPerSecond(0.0); 3] };
static mut SENS_HAVE_IMU: bool = false;
/// ★design.md §7：单调性检查丢弃的异常帧计数（诊断）。
static mut SENS_IMU_DROP: u32 = 0;

/// ★装配（在 setup 任务里调用一次）。
pub fn sensors_init() {
    #[cfg(not(feature = "hil"))]
    unsafe {
        SENS_STACK.write(SensorStack::new());
        SENS_FILTERS.write(flyctrl_core::imu_filters::ImuFilters::new(1.0 / SAMPLE_DT));
        SENS_FIRST = true;
        SENS_LOOP_CNT = 0;
        SENS_IMU_PREV_HAS = false;
        SENS_GPS_STALE = 0;
    }
    info!(tag: "sensor", "sensors item init (real={}, hil={})",
          cfg!(feature = "real-sensors") as u32, cfg!(feature = "hil") as u32);
}


/// ★design.md §3 **L0**：data-ready ISR 内调 —— 读 IMU → 预处理 → `ImuRing` + IMU topic。
/// ⚠️在 ISR 上下文执行（只做一次 SPI 事务；不阻塞）。
pub fn imu_sample_step() {
    #[cfg(not(feature = "hil"))]
    unsafe {
        let stack = SENS_STACK.assume_init_mut();
        let imu_filters = SENS_FILTERS.assume_init_mut();
        let s = stack.read_imu();
        let raw_a = [s.accel[0].0, s.accel[1].0, s.accel[2].0];
        let raw_g = [s.gyro[0].0, s.gyro[1].0, s.gyro[2].0];
        let ts = rtos_app_sdk::rtos::cycle_now();
        let raw_dt = (ts.wrapping_sub(SENS_IMU_PREV_TS)) as f32
            / (crate::flyctrl::rt_stat::cycles_per_us() as f32 * 1000.0);
        // ★design.md §7：**单调性检查** —— dt≤0（戳翻转/乱序）或超阈值 ⇒ **丢弃该帧**（不夹紧 ✗）+ 计数。
        if SENS_IMU_PREV_HAS && !(raw_dt > 1e-5 && raw_dt < 0.02) {
            SENS_IMU_PREV_TS = ts; // 重同步（避免持续丢弃）
            SENS_IMU_DROP = SENS_IMU_DROP.wrapping_add(1);
            return;
        }
        let dt = if SENS_IMU_PREV_HAS { raw_dt } else { 0.001 };
        SENS_IMU_PREV_HAS = true;
        SENS_IMU_PREV_TS = ts;
        // ★design.md §3：ISR **只做搬运+打戳**（滤波是"复杂计算" ⇒ 移到 L2 `sensors` item ✗）
        let _ = (raw_a, raw_g, imu_filters);
        SENS_LAST_IMU = s;
        SENS_TOPIC_TS = ts;
        SENS_HAVE_IMU = true;
        let r = &mut *core::ptr::addr_of_mut!(crate::flyctrl::IMU_RING);
        r.push(flyctrl_core::imu_ring::ImuDelta {
            delta_ang: [s.gyro[0].0 * dt, s.gyro[1].0 * dt, s.gyro[2].0 * dt],
            delta_vel: [s.accel[0].0 * dt, s.accel[1].0 * dt, s.accel[2].0 * dt],
            dt,
            ts_cyc: ts,
        });
    }
}

/// ★L2 `wq:sensors` **WorkItem**：**一拍**（由工作队列 worker 调用）。
pub fn sensors_step() {
    #[cfg(not(feature = "hil"))]
    {
        let stack = unsafe { SENS_STACK.assume_init_mut() };
        let imu_filters = unsafe { SENS_FILTERS.assume_init_mut() };
        let sample_dt = SAMPLE_DT;
        let (mut first, mut loop_cnt, mut imu_prev_ts, mut imu_prev_has) = unsafe {
            (SENS_FIRST, SENS_LOOP_CNT, SENS_IMU_PREV_TS, SENS_IMU_PREV_HAS)
        };
        let (mut topic_af, mut topic_gv, mut topic_rg, mut topic_ts) =
            unsafe { (SENS_TOPIC_AF, SENS_TOPIC_GV, SENS_TOPIC_RG, SENS_TOPIC_TS) };
        let mut gps_stale = unsafe { SENS_GPS_STALE };
        let mut gps_last_tick = unsafe { SENS_GPS_LAST_TICK };

            // 非 HIL：采样 + 写共享帧（seqlock：sensors 单写、control 单读，control 优先级更高）。
            #[cfg(not(feature = "hil"))]
            {
                // 虚拟回放模式下，每个采样周期推进一次全局读指针（四类数据同步对齐）。
                #[cfg(not(feature = "real-sensors"))]
                unsafe {
                    crate::sensors::sim::dataset::PLAYBACK.advance(sample_dt);
                }

                // ---- IMU 采样已在 L0 ISR；**滤波在此（L2 `sensors`，design.md §5）** ----
                let imu_sample: Option<ImuSample> =
                    if unsafe { SENS_HAVE_IMU } { Some(unsafe { SENS_LAST_IMU }) } else { None };
                if let Some(s) = imu_sample.as_ref() {
                    let (af, gv) = imu_filters.process(
                        [s.accel[0].0, s.accel[1].0, s.accel[2].0],
                        [s.gyro[0].0, s.gyro[1].0, s.gyro[2].0],
                    );
                    topic_af = af; topic_gv = gv;
                    topic_rg = [s.gyro[0].0, s.gyro[1].0, s.gyro[2].0];
                }
                let _ = &stack; // 慢传感器由本 item 读；IMU 不在此
                // ★design.md §7：逐样本**打硬件时间戳**并入 `IMU_RING`（后续 `rate`/`ekf` 消费）。
                let baro_sample: Option<f32> = Some(stack.read_altitude());
                let gps_sample: Option<PosSample> = stack.read_gps();
                // GPS 样本保持（见循环外注释）：无新帧时保持最近有效样本
                // （超时清理由下方写段按 gps_stale 处理，这里只维护过期计数）。
                if gps_sample.is_some() {
                    gps_stale = 0;
                    gps_last_tick = rtos_app_sdk::rtos::tick_count();
                } else {
                    gps_stale += 1;
                }
                let rc_input: RcInput = stack.read_rc();
                // 磁力计：real-sensors 经 I2C 读 QMC5883L（0x0D），虚拟源直出。
                // read() 内部失败会置 unhealthy；读后 health 反映链路真实状态。
                let mag_sample: [f32; 3] = stack.read_mag();
                let mag_ok = stack.health().mag;

                if WRITE_FRAME {
                    unsafe {
                        SENSOR_SEQ = SENSOR_SEQ.wrapping_add(1); // 奇：写入中
                        let f = &mut *core::ptr::addr_of_mut!(SENSOR_FRAME);
                        f.imu = imu_sample;
                        // ★design.md §5：发布的 IMU topic（带硬件戳）
                        f.accel_filt = topic_af;
                        f.gyro_vel = topic_gv;
                        f.gyro_raw = topic_rg;
                        f.ts_cyc = topic_ts;
                        f.baro_alt = baro_sample;
                        // GPS 样本保持：有新帧更新；无新帧且未超时保持旧样本
                        // （避免 control 拍错过 2ms 窗口导致 pos_available 大面积
                        // false → FDIR 误判 GPS lost）；超时（500ms 无帧）置 None。
                        if gps_sample.is_some() {
                            f.gps = gps_sample; // 新帧 ✓（stale=false ✓）
                        } else if rtos_app_sdk::rtos::tick_count().wrapping_sub(gps_last_tick)
                            > GPS_VALID_MS
                        {
                            f.gps = None; // 超时（时间基准，500ms）⇒ 真丢失 ✓
                        }
                        // ⚠️ 曾在此给保持样本打 `stale` 标记 —— **已回退** ✗：
                        //   该字段使 `PosSample` 变大 ⇒ 破坏【固件↔宿主共享帧 ABI】✗（§5.39 ✓）。
                        //   修复方向见 §5.39：staleness 走【带外】或测试侧按符号读 ✓。
                        f.mag = if mag_ok { Some(mag_sample) } else { None };
                        f.rc = rc_input;
                        f.armed = rc_input.armed;
                        f.imu_ok = imu_sample.is_some();
                        f.baro_ok = baro_sample.is_some();
                        f.gps_ok = f.gps.is_some();
                        f.mag_ok = mag_ok;
                        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
                        SENSOR_SEQ = SENSOR_SEQ.wrapping_add(1); // 偶：写入完成
                    }
                }

                if loop_cnt % 500 == 0 {
                    // ★design.md §9：暴露工作队列统计（提交/软超时/硬超时）
                    let mut w = [0u32; 6];
                    if let Some(f) = rtos_app_sdk::abi::slot().work_stats {
                        f(w.as_mut_ptr());
                    }
                    // ★design.md §9：可观测性 —— 队列统计 + **队列利用率(L2 已用 cycles)**
                    let l1 = unsafe { crate::flyctrl::rate_task::RATE_EXEC_CYC } as u64 * 1000
                        + unsafe { crate::flyctrl::alloc_task::ALLOC_EXEC_CYC } as u64 * 1000
                        + unsafe { crate::flyctrl::safety_task::SAFETY_EXEC_CYC } as u64 * 500;
                    let cpu_ms = crate::flyctrl::rt_stat::cycles_per_us() as u64 * 1000;
                    let l1_permille = (l1 * 1000 / (cpu_ms * 1000)) as u32;
                    let ovl = crate::flyctrl::safety_task::OVERLOAD_LEVEL
                        .load(core::sync::atomic::Ordering::Relaxed);
                    info!(tag: "sensor", "loop {} drdy={} wq={}/{}/{}/{}/{} l2used={} l1={}permille ovl={}",
                          loop_cnt, unsafe { crate::flyctrl::IMU_DRDY_CNT },
                          w[0], w[1], w[2], w[3], w[4], w[5], l1_permille, ovl);
                }
            }

            if first {
                info!(tag: "sensor", "first loop done");
                first = false;
            }
            loop_cnt += 1;

        unsafe {
            SENS_FIRST = first; SENS_LOOP_CNT = loop_cnt;
            SENS_IMU_PREV_TS = imu_prev_ts; SENS_IMU_PREV_HAS = imu_prev_has;
            SENS_TOPIC_AF = topic_af; SENS_TOPIC_GV = topic_gv;
            SENS_TOPIC_RG = topic_rg; SENS_TOPIC_TS = topic_ts;
            SENS_GPS_STALE = gps_stale;
            SENS_GPS_LAST_TICK = gps_last_tick;
        }
    }
    #[cfg(feature = "hil")]
    { let _ = WRITE_FRAME; }
}
