//! flyctrl-fw：STM32F407 真实固件入口。
//!
//! 把 M10 的发布/订阅中间件接到真实外设：
//!   - USART2 (PA2/PA3, 921600) ↔ MAVLink/遥测链路（实现 `comm::Link`）
//!   - TIM3  (PB4/5/0/1)        ↔ 四路电机 PWM（实现 `hal::PwmEsc`）
//!   - 裸机忙等节拍              ↔ 控制环路（等价于 hal::Rtos::run）
//!
//! 控制流：UART 收帧 → 注入总线 → 控制律算出 ActuatorCmd → PwmEsc 输出。
//! 此处用最简闭环演示真实驱动接通（编译/链接通过即可，烧录后见 PWM 与串口活动）。

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use core::panic::PanicInfo;
use stm32f4::stm32f407::{interrupt, Peripherals};

use flyctrl_core::comm::link::Link;
use flyctrl_core::comm::mavlink::{self, msg_id};
use flyctrl_core::hal::stm32f407::{clock_init, usart2_isr, UartLink, PwmEsc};
use flyctrl_core::bus::Bus;
use flyctrl_core::fdir::{Fdir, RtlHome};
use flyctrl_core::vehicle::{ActuatorCmd, ImuSample, MeterPerSecondSquared, Ned, RadianPerSecond, VehicleState};
use flyctrl_core::units::Second;

/// 控制环路周期（ms）。400Hz 与 PWM 同频。
const LOOP_MS: u32 = 4;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

/// 外设单例：裸机下只取一次，存进 static 以拿到 `'static` 引用供驱动长期持有。
static mut PERIPHS: Option<Peripherals> = None;

/// `critical-section` 的单核实现：用 PRIMASK 开关中断（cortex-m 无 SMP）。
struct SingleCoreImpl;
unsafe impl critical_section::Impl for SingleCoreImpl {
    unsafe fn acquire() -> u8 {
        let primask: u8;
        core::arch::asm!("mrs {0}, PRIMASK", out(reg) primask);
        core::arch::asm!("cpsid i");
        primask
    }
    unsafe fn release(token: u8) {
        if token & 0x1 == 0 {
            core::arch::asm!("cpsie i");
        }
    }
}
critical_section::set_impl!(SingleCoreImpl);

#[entry]
fn main() -> ! {
    // 外设唯一实例，存进 static 供驱动长期持有引用（避免引入分配器）。
    unsafe {
        PERIPHS = Some(Peripherals::take().expect("Peripherals already taken"));
    }
    let dp = unsafe { &*core::ptr::addr_of!(PERIPHS) }.as_ref().unwrap();

    // 1) 系统时钟 168MHz
    clock_init(&dp.RCC);

    // 2) 真实外设：USART2 链路 + TIM3 四路 PWM
    let mut link = UartLink::new(&dp.USART2, &dp.RCC, 921_600);
    let pwm = PwmEsc::new(&dp.TIM3, &dp.RCC);

    // 3) 发布/订阅总线（板内静态拓扑 + 运行时发现）
    let mut bus = Bus::new();
    let _registry = Bus::discover();
    let _ = &_registry;

    // 4) 板载飞行控制闭环（不依赖 RTOS；RTOS 就绪后此循环整体搬入任务即可）：
    //    FDIR 健康监控 + RTL home 记忆 + MAVLink 遥测下行 + 指令解析。
    let mut fdir = Fdir::new();
    let mut rtl_home = RtlHome::new();
    let state = VehicleState::zero();
    let mut seq: u8 = 0;
    let mut armed = false;
    let mut mode: u8 = 0; // 自定义飞行模式码（与 flightmode::FlightMode 对齐）
    let mut frame_buf = [0u8; flyctrl_core::comm::link::MAX_FRAME_LEN];

    loop {
        // 4a) 收链路帧 → 解析 MAVLink 指令（解锁 / 模式切换 / 参数等）
        let rx = link.recv_frame();
        if let Some((id, payload)) = mavlink::decode(&rx) {
            match id {
                msg_id::COMMAND_LONG => {
                    if let Some(cmd) = mavlink::decode_command_long(payload) {
                        use flyctrl_core::comm::mavlink::enums;
                        match cmd.command {
                            // 解锁 / 上锁（param1: 1=arm, 0=disarm）
                            c if c == enums::MAV_CMD_COMPONENT_ARM_DISARM => {
                                armed = cmd.params[0] as i32 == 1;
                            }
                            // SET_MODE：param1 为目标自定义模式
                            c if c == enums::MAV_CMD_DO_SET_MODE => {
                                mode = cmd.params[0] as u8;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }

        // 4b) 占位 IMU 样本（真实传感器接入后替换）：用零加速度 + 平滑姿态导数，
        //     每拍注入微小变化避免 FDIR 误判 IMU 冻结（真实 IMU 含噪声）。
        let t = (seq as f32) * (LOOP_MS as f32) / 1000.0;
        let imu = ImuSample {
            accel: [
                MeterPerSecondSquared(0.01 * libm::sinf(t)),
                MeterPerSecondSquared(0.0),
                MeterPerSecondSquared(9.8 + 0.01 * libm::cosf(t)),
            ],
            gyro: [RadianPerSecond(0.0); 3],
        };
        // GPS 可用性占位：始终可用（真实为 GNSS 驱动 / 总线 est 位置质量）。
        let gps_ok = true;
        let baro_ok = true;
        let mag_ok = true;
        let health = fdir.update(&imu, gps_ok, baro_ok, mag_ok);

        // 4c) RTL home 记忆：首次定位即锁定起飞点
        if gps_ok && !rtl_home.locked {
            let home = Ned::new(state.pos[0].0, state.pos[1].0, state.pos[2].0);
            let _ = rtl_home.try_lock(home);
        }

        // 4d) 推进总线时间并抽取消息
        bus.tick(Second((LOOP_MS as f32) / 1000.0));
        bus.pump();

        // 4e) 控制律（最简：零油门演示，真实为 EKF+LQR/INDI + FDIR 降级处理）
        let cmd = ActuatorCmd::zero();
        let _ = bus.publish_actuator(cmd);

        // 4f) 输出 PWM（演示：四路最低油门，真实应写 cmd.motor）
        pwm.write_norm([0.0; 4]);

        // 4g) 遥测下行：HEARTBEAT + LOCAL_POSITION_NED（标准 MAVLink，QGC 可解析）
        let n = mavlink::encode_heartbeat(mode, armed, seq, &mut frame_buf);
        link.send_frame(&flyctrl_core::comm::link::Frame::from_bytes(&frame_buf[..n]));
        let n = mavlink::encode_local_pos_from(mavlink::SYS_ID, &state, seq, &mut frame_buf);
        link.send_frame(&flyctrl_core::comm::link::Frame::from_bytes(&frame_buf[..n]));
        // 健康异常时额外上报 SYS_STATUS（占位：全传感器 OK）
        let n = mavlink::encode_sys_status(health != flyctrl_core::fdir::Health::Critical, seq, &mut frame_buf);
        link.send_frame(&flyctrl_core::comm::link::Frame::from_bytes(&frame_buf[..n]));

        seq = seq.wrapping_add(1);

        // 4h) 节拍延时（裸机忙等，避免引入完整 SysTick 中断处理）
        delay_ms(LOOP_MS);
    }
}

/// 毫秒忙等（168MHz 下粗略循环，演示用）。
fn delay_ms(ms: u32) {
    let cycles = ms as u32 * 168_000 / 4;
    for _ in 0..cycles {
        cortex_m::asm::nop();
    }
}

/// USART2 中断：转交真实驱动处理。
#[interrupt]
fn USART2() {
    let usart = unsafe { &*stm32f4::stm32f407::USART2::ptr() };
    usart2_isr(usart);
}
