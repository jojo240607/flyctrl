//! HIL 共享内存直连（SRAM3 @0x2002_0000）：PC 物理仿真器 ↔ MCU 的零协议闭环通道。
//!
//! 与 USB(MAVLink HIL_SENSOR/SET_POSITION) 路径互补：无 USB / 无 MAVLink，
//! PC 每 4ms 物理步把传感器真值/设定点/解锁写入 SRAM3 共享区，本模块在
//! `uplink` 任务内 1ms 轮询 `pc_seq` 变化后全量注入 `SENSOR_FRAME` 并唤醒
//! control（`HIL_EVT.give()`）；`telemetry` 每 20ms 把执行器/诊断写回共享区，
//! PC 读回驱动 plant。与 USB 路径**并存**：共享区魔数不匹配（仿真器未接共享
//! 内存）时本模块完全静默，USB 路径不受影响。
//!
//! 共享区布局（与 `mcu_simulater/tests/x_shmem_mcusim.rs` 硬编码一致——
//! **任何一侧改动必须同步另一侧**）：
//!
//! ```text
//! 0x2002_0000  MAGIC  u32     0x48494C31（PC 侧 shm_init 写入，探测用）
//! 0x2002_0004  pc_seq u32     PC 每物理步 +1（增量注入触发）
//! 0x2002_0008  IMU accel f32×3（机体系比力，FRD，m/s²）
//! 0x2002_0014  IMU gyro  f32×3（rad/s）
//! 0x2002_0020  baro     f32    气压高度（向上为正，m）
//! 0x2002_0024  gps_valid u32   非 0 = GPS 有效
//! 0x2002_0028  gps pos  f32×3  NED（m）
//! 0x2002_0034  gps vel  f32×3  NED（m/s）
//! 0x2002_0040  sp_valid u32   非 0 = 设定点有效
//! 0x2002_0044  sp pos   f32×3  NED（m）
//! 0x2002_0050  sp vel   f32×3  NED（m/s）
//! 0x2002_005C  sp acc   f32×3  NED（m/s²）
//! 0x2002_0068  sp yaw   f32    rad
//! 0x2002_006C  armed    u32    非 0 = 解锁
//! 0x2002_0070  (保留 16B)
//! 0x2002_0080  mcu_seq  u32    MCU 回写序号（每次 telemetry 回写 +1）
//! 0x2002_0084  motor    f32×4  归一化推力 [0,1]
//! 0x2002_0094  diag     f32×16 诊断区（见 [`DiagIdx`]）
//! ```
//!
//! 访问方式：SRAM3 不在固件链接脚本 MEMORY 内（主 SRAM 止于 0x2002_0000），
//! 无 MPU 配置，`read_volatile`/`write_volatile` 裸指针访问安全。仅在
//! `hil` feature 下编译（与 USB HIL 同一数据源语义：真值由 uplink 单写
//! `SENSOR_FRAME`，不破坏 seqlock 单写者不变式）。

use core::ptr::{read_volatile, write_volatile};

use flyctrl_core::comm::mavlink::SetPositionTargetLocalNed;
use flyctrl_core::fdir::Health;
use flyctrl_core::units::{Meter, MeterPerSecond, MeterPerSecondSquared, RadianPerSecond};
use flyctrl_core::vehicle::{ImuSample, PosSample, RcInput, VehicleState};

use crate::flyctrl::{uplink, HIL_EVT, SENSOR_FRAME, SENSOR_SEQ};

/// 共享区基址（SRAM3；固件链接脚本未占用，mcu_simulater 额外映射）。
const SHM_BASE: usize = 0x2002_0000;
/// 共享区魔数：PC 侧 `shm_init` 写入，用于探测"仿真器经共享内存连接"。
const MAGIC: u32 = 0x4849_4C31;

// ── 偏移（与 mcu_simulater/tests/x_shmem_mcusim.rs 硬编码一致）──────────
const O_MAGIC: usize = 0x00;
const O_PC_SEQ: usize = 0x04;
const O_IMU_ACC: usize = 0x08;
const O_IMU_GYR: usize = 0x14;
const O_BARO: usize = 0x20;
const O_GPS_VALID: usize = 0x24;
const O_GPS_POS: usize = 0x28;
const O_GPS_VEL: usize = 0x34;
const O_SP_VALID: usize = 0x40;
const O_SP_POS: usize = 0x44;
const O_SP_VEL: usize = 0x50;
const O_SP_ACC: usize = 0x5C;
const O_SP_YAW: usize = 0x68;
const O_ARMED: usize = 0x6C;
const O_MCU_SEQ: usize = 0x80;
const O_MOTOR: usize = 0x84;
const O_DIAG: usize = 0x94;

/// diag 区（16 个 f32）布局索引——测试/联调诊断约定，非断言契约。
pub mod diag_idx {
    /// 四路电机指令（[0,1]）。
    pub const MOTORS: usize = 0;
    /// 平均油门（[0,1]）。
    pub const THROTTLE: usize = 4;
    /// 健康等级（0=Nominal, 1=Degraded, 2=Critical）。
    pub const HEALTH: usize = 5;
    /// 估计位置 NED（3 个）。
    pub const EST_POS: usize = 6;
    /// 估计速度 NED（3 个）。
    pub const EST_VEL: usize = 9;
    /// 注入计数（SENSOR_SEQ，本任务每注入 +2）。
    pub const INJECT_CNT: usize = 12;
    /// 估计高度（NED z）。
    pub const EST_ALT: usize = 13;
    /// 最近注入加速度 z（m/s²）。
    pub const ACCEL_Z: usize = 14;
    /// 解锁标志（0/1）。
    pub const ARMED: usize = 15;
}

/// 上次已消费的 pc_seq（本任务（uplink）单写者，无竞争）。
static mut LAST_PC_SEQ: u32 = 0;
/// 最近注入的加速度 z（供 diag 区回显）。
static mut LAST_ACCEL_Z: f32 = 0.0;

fn rd_u32(off: usize) -> u32 {
    unsafe { read_volatile((SHM_BASE + off) as *const u32) }
}
fn rd_f32(off: usize) -> f32 {
    unsafe { read_volatile((SHM_BASE + off) as *const f32) }
}
fn wr_u32(off: usize, v: u32) {
    unsafe { write_volatile((SHM_BASE + off) as *mut u32, v) }
}
fn wr_f32(off: usize, v: f32) {
    unsafe { write_volatile((SHM_BASE + off) as *mut f32, v) }
}

/// 共享内存主机是否在线（magic 匹配）。SRAM3 上电内容不确定，与常量
/// `0x48494C31` 恰好相等的概率可忽略；且即使误判在线，pc_seq 不变也不会注入。
pub fn shmem_available() -> bool {
    rd_u32(O_MAGIC) == MAGIC
}

/// uplink 每 1ms 轮询一次：检测到 `pc_seq` 变化即全量注入。
///
/// 注入内容（一次同步点）：IMU（真实帧，触发 EKF 姿态初始化）/ 气压 / GPS
/// 位置速度 / 设定点（写 `G_HIL_SETPOINT`，control 任务零改动只读）/ 解锁
/// （写 `SENSOR_FRAME.armed`）。完成后 `HIL_EVT.give()` 唤醒 control——与
/// USB 路径共用同一事件驱动闭环（一输入一输出、不依赖 control 自身时钟）。
pub fn shmem_poll_once() {
    if !shmem_available() {
        return;
    }
    let pc_seq = rd_u32(O_PC_SEQ);
    if pc_seq == unsafe { LAST_PC_SEQ } {
        return; // 无新帧：保持静默（USB 路径不受影响）
    }
    unsafe { LAST_PC_SEQ = pc_seq; }

    // ── 读 PC 真值（一次读全，尽量贴近同拍快照）──
    let mut acc = [0.0f32; 3];
    let mut gyr = [0.0f32; 3];
    for i in 0..3 {
        acc[i] = rd_f32(O_IMU_ACC + 4 * i);
        gyr[i] = rd_f32(O_IMU_GYR + 4 * i);
    }
    unsafe { LAST_ACCEL_Z = acc[2]; }
    let baro = rd_f32(O_BARO);
    let gps_valid = rd_u32(O_GPS_VALID) != 0;
    let mut gps_pos = [0.0f32; 3];
    let mut gps_vel = [0.0f32; 3];
    for i in 0..3 {
        gps_pos[i] = rd_f32(O_GPS_POS + 4 * i);
        gps_vel[i] = rd_f32(O_GPS_VEL + 4 * i);
    }
    let sp_valid = rd_u32(O_SP_VALID) != 0;
    let mut sp_pos = [0.0f32; 3];
    let mut sp_vel = [0.0f32; 3];
    let mut sp_acc = [0.0f32; 3];
    for i in 0..3 {
        sp_pos[i] = rd_f32(O_SP_POS + 4 * i);
        sp_vel[i] = rd_f32(O_SP_VEL + 4 * i);
        sp_acc[i] = rd_f32(O_SP_ACC + 4 * i);
    }
    let sp_yaw = rd_f32(O_SP_YAW);
    let armed = rd_u32(O_ARMED) != 0;

    // ── 设定点 → G_HIL_SETPOINT（control 任务只读 G_HIL_SETPOINT/VALID）──
    uplink::set_hil_setpoint(
        SetPositionTargetLocalNed {
            time_boot_ms: pc_seq,
            type_mask: 0,
            x: sp_pos[0], y: sp_pos[1], z: sp_pos[2],
            vx: sp_vel[0], vy: sp_vel[1], vz: sp_vel[2],
            afx: sp_acc[0], afy: sp_acc[1], afz: sp_acc[2],
            yaw: sp_yaw, yaw_rate: 0.0,
        },
        sp_valid,
    );

    // ── SENSOR_FRAME（seqlock；本任务为单写者，与 USB 注入同任务）──
    unsafe {
        SENSOR_SEQ = SENSOR_SEQ.wrapping_add(1); // 奇：写入中
        let f = &mut *core::ptr::addr_of_mut!(SENSOR_FRAME);
        f.imu = Some(ImuSample {
            accel: [
                MeterPerSecondSquared(acc[0]),
                MeterPerSecondSquared(acc[1]),
                MeterPerSecondSquared(acc[2]),
            ],
            gyro: [
                RadianPerSecond(gyr[0]),
                RadianPerSecond(gyr[1]),
                RadianPerSecond(gyr[2]),
            ],
        });
        f.baro_alt = Some(baro);
        f.gps = if gps_valid {
            Some(PosSample::with_vel(
                [
                    Meter(gps_pos[0]),
                    Meter(gps_pos[1]),
                    Meter(gps_pos[2]),
                ],
                [
                    MeterPerSecond(gps_vel[0]),
                    MeterPerSecond(gps_vel[1]),
                    MeterPerSecond(gps_vel[2]),
                ],
            ))
        } else {
            None
        };
        // RC：共享内存直连下遥控接收机不接；仅需 `fresh=true` 使控制环输出
        // （armed 来自共享区，与 USB 路径 COMMAND_LONG 置位语义一致）。
        f.rc = RcInput { roll: 0.0, pitch: 0.0, yaw: 0.0, throttle: 0.0, armed, mode: 0, fresh: true };
        f.armed = armed;
        f.imu_ok = true;
        f.baro_ok = true;
        f.gps_ok = gps_valid;
        f.mag_ok = false; // 共享区无磁力计通道（与 x_shmem_mcusim 一致）
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        SENSOR_SEQ = SENSOR_SEQ.wrapping_add(1); // 偶：写入完成
    }
    // 【事件驱动】唤醒 control：与 USB 注入同语义（一输入一输出闭环）。
    unsafe { HIL_EVT.give(); }
}

/// telemetry 每 20ms 回写：mcu_seq + 执行器 + 诊断区，PC 读回驱动 plant。
///
/// `est` / `health` / `armed` 由 telemetry 任务已在 `EST_MTX` 临界区内读到，
/// 直接传入避免二次加锁。
pub fn shmem_write_back(est: &VehicleState, health: Health, armed: bool) {
    if !shmem_available() {
        return;
    }
    // 递增 MCU 序号（每次回写 +1；PC 据此判断"固件活着且执行器已回传"）。
    wr_u32(O_MCU_SEQ, rd_u32(O_MCU_SEQ).wrapping_add(1));

    let motors = uplink::actuator_cmd();
    for i in 0..4 {
        wr_f32(O_MOTOR + 4 * i, motors[i]);
    }

    let mut diag = [0.0f32; 16];
    diag[diag_idx::MOTORS..diag_idx::MOTORS + 4].copy_from_slice(&motors);
    diag[diag_idx::THROTTLE] = (motors[0] + motors[1] + motors[2] + motors[3]) / 4.0;
    diag[diag_idx::HEALTH] = match health {
        Health::Nominal => 0.0,
        Health::Degraded => 1.0,
        Health::Critical => 2.0,
    };
    for i in 0..3 {
        diag[diag_idx::EST_POS + i] = est.pos[i].0;
        diag[diag_idx::EST_VEL + i] = est.vel[i].0;
    }
    diag[diag_idx::INJECT_CNT] = unsafe { SENSOR_SEQ } as f32;
    diag[diag_idx::EST_ALT] = est.pos[2].0;
    diag[diag_idx::ACCEL_Z] = unsafe { LAST_ACCEL_Z };
    diag[diag_idx::ARMED] = armed as u32 as f32;
    for (i, v) in diag.iter().enumerate() {
        wr_f32(O_DIAG + 4 * i, *v);
    }
}
