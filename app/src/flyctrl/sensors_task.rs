//! 传感器采集任务：周期性读取 IMU / 气压 / GPS / RC，写入 `SENSOR_FRAME`（经 seqlock，见 `SENSOR_SEQ`）。
//!
//! 数据源由 `sensors::stack::SensorStack` 统一封装，底层是虚拟回放（`VirtualXxx`）
//! 还是真实驱动（`ImuMpu6050` / `BaroBmp280` / `GpsUblox` / `RcSbus`）由编译期
//! `cfg(feature = "real-sensors")` 决定，本任务**不感知**差异，只调用 trait 方法。
//! 调试用虚拟源（默认），接真实硬件时只需开启 feature，算法/控制/遥测层零改动。

use core::ffi::c_void;

use rtos_app_sdk::info;
use rtos_app_sdk::abi::g_app_slot;

// 采样路径仅在非 HIL 模式编译（HIL 模式下仿真真值由 uplink 任务注入，
// 见 uplink.rs 的 HIL_SENSOR / SET_POSITION_TARGET_LOCAL_NED 处理）。
#[cfg(not(feature = "hil"))]
use crate::flyctrl::{SENSOR_FRAME, SENSOR_SEQ};
#[cfg(not(feature = "hil"))]
use crate::sensors::stack::SensorStack;
#[cfg(not(feature = "hil"))]
use flyctrl_core::vehicle::{ImuSample, PosSample, RcInput};

/// 诊断：是否把采集数据写入共享 SENSOR_FRAME。false 时控制环回退到内部虚拟源，
/// 用于区分“写入共享帧后被控制环处理”与“传感器任务自身”两类问题。
const WRITE_FRAME: bool = true;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Sensors;

impl Sensors {
    pub fn new(_name: &[u8]) -> Option<Self> {
        Some(Self)
    }

    pub extern "C" fn entry(_arg: *mut c_void) {
        info!(tag: "sensor", "task started (WRITE_FRAME={}, real={}, hil={})",
              WRITE_FRAME as u32, cfg!(feature = "real-sensors") as u32, cfg!(feature = "hil") as u32);

        // 采样周期（秒），与回放推进一致。
        let sample_dt: f32 = 0.002; // 500Hz 采样

        // ---- 统一数据源：虚拟或真实由编译期 feature 决定 ----
        // HIL 模式下真值由 uplink 任务（usb0 唯一读者）经 HIL_SENSOR /
        // SET_POSITION_TARGET_LOCAL_NED 注入 SENSOR_FRAME，本任务不再采样，
        // 避免与 uplink 双写 SENSOR_FRAME 破坏 seqlock 单写者不变式。
        #[cfg(not(feature = "hil"))]
        let mut stack = SensorStack::new();
        #[cfg(not(feature = "hil"))]
        let h = stack.health();
        #[cfg(not(feature = "hil"))]
        info!(tag: "sensor", "stack init imu={} baro={} gps={} rc={} mag={}",
              h.imu, h.baro, h.gps, h.rc, h.mag);

        let mut first = true;
        let mut loop_cnt: u32 = 0;
        // 绝对节拍基准：周期恒为 sample_dt，不被 control 抢占"吸附"。
        let mut wake_tick = rtos_app_sdk::rtos::tick_count();
        // GPS 样本保持：真实 GPS 20Hz 帧，两次帧之间 read_gps 返回 None；若每拍
        // 直接把 None 写进 SENSOR_FRAME，control(4ms) 只在 2ms Some 窗口内读到样本
        // （约一半拍），FDIR 的 pos_available 大面积 false → gps_lost_steps 累积
        // → 误判 GPS 丢失降级（虚拟时钟校准后实测 baro_step health=1）。
        // 修复：无新帧时保持最近有效样本，超过 GPS_VALID_STEPS（500ms）无新帧才
        // 置 None（真实飞控 GPS 短暂丢帧不降级语义）。
        const GPS_VALID_STEPS: u32 = 250; // 250 × 2ms = 500ms
        let mut gps_stale: u32 = 0;
        loop {
            // 非 HIL：采样 + 写共享帧（seqlock：sensors 单写、control 单读，control 优先级更高）。
            #[cfg(not(feature = "hil"))]
            {
                // 虚拟回放模式下，每个采样周期推进一次全局读指针（四类数据同步对齐）。
                #[cfg(not(feature = "real-sensors"))]
                unsafe {
                    crate::sensors::sim::dataset::PLAYBACK.advance(sample_dt);
                }

                // ---- 读取各类传感器（统一 trait 接口，不区分虚拟/真实）----
                flyctrl_core::perf::probe(30); // 段1 起（sensors 循环顶 ✓）
                let imu_sample: Option<ImuSample> = Some(stack.read_imu());
                flyctrl_core::perf::probe(31); // 段2 起（IMU 读完 ✓）
                let baro_sample: Option<f32> = Some(stack.read_altitude());
                flyctrl_core::perf::probe(32); // 段3 起（气压读完 ✓）
                let gps_sample: Option<PosSample> = stack.read_gps();
                flyctrl_core::perf::probe(33); // 段4 起（GPS 读完 ✓）
                // GPS 样本保持（见循环外注释）：无新帧时保持最近有效样本
                // （超时清理由下方写段按 gps_stale 处理，这里只维护过期计数）。
                if gps_sample.is_some() {
                    gps_stale = 0;
                } else {
                    gps_stale += 1;
                }
                let rc_input: RcInput = stack.read_rc();
                flyctrl_core::perf::probe(34); // 段5 起（RC 读完 ✓）
                // 磁力计：real-sensors 经 I2C 读 QMC5883L（0x0D），虚拟源直出。
                // read() 内部失败会置 unhealthy；读后 health 反映链路真实状态。
                let mag_sample: [f32; 3] = stack.read_mag();
                let mag_ok = stack.health().mag;

                if WRITE_FRAME {
                    unsafe {
                        SENSOR_SEQ = SENSOR_SEQ.wrapping_add(1); // 奇：写入中
                        let f = &mut *core::ptr::addr_of_mut!(SENSOR_FRAME);
                        f.imu = imu_sample;
                        f.baro_alt = baro_sample;
                        // GPS 样本保持：有新帧更新；无新帧且未超时保持旧样本
                        // （避免 control 拍错过 2ms 窗口导致 pos_available 大面积
                        // false → FDIR 误判 GPS lost）；超时（500ms 无帧）置 None。
                        if gps_sample.is_some() {
                            f.gps = gps_sample; // 新帧 ✓（stale=false ✓）
                        } else if gps_stale > GPS_VALID_STEPS {
                            f.gps = None; // 超时 ⇒ 真丢失 ✓
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
                    info!(tag: "sensor", "loop {} gps_w={} imu_w={} baro_w={} mag_w={}",
                          loop_cnt, gps_sample.is_some(), imu_sample.is_some(), baro_sample.is_some(), mag_ok);
                }
            }

            if first {
                info!(tag: "sensor", "first loop done");
                first = false;
            }
            loop_cnt += 1;

            rtos_app_sdk::rtos::delay_until(&mut wake_tick, (sample_dt * 1000.0) as u32);
        }
    }
}

/// 模块级入口（供 `mod.rs::spawn_flyctrl` 经 `spawn_rt` 注册）。
pub extern "C" fn sensors_entry(arg: *mut c_void) {
    Sensors::entry(arg);
}
