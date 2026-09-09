//! 虚拟 RC 驱动：从全局 `PLAYBACK` 读取回放数据集，伪装成真实遥控接收机。
//!
//! 与 `rc::sbus::RcSbus` 实现同一 `RcReceiver` trait，使 `sensors_task`
//! 只需切换数据源即可，无需改动采集逻辑。

use flyctrl_core::hal::sensor::RcReceiver;
use flyctrl_core::vehicle::RcInput;

use crate::sensors::sim::dataset::{Frame, PLAYBACK};

/// 虚拟 RC 的 armed 默认。
///
/// 必须为 `false`：否则虚拟 RC 强行 `armed=true` 会让控制律的解锁逻辑
/// `armed_eff = rc_armed || cmd_armed` 永远为 true，地面站经 COMMAND_LONG
/// (ARM/DISARM) 发出的 DISARM 被 OR 掉而失效，无法通过地面站解锁/上锁。
///
/// 解锁改由地面站指令 `G_CMD_ARMED` 唯一决定（虚拟/联调环境下 RC 不应强制 arm）。
/// 纯虚拟数据集演示整链路闭环时，通过地面站发送 ARM 即可获得推力输出。
const RC_FORCE_ARM: bool = false;

pub struct VirtualRc;

impl VirtualRc {
    pub fn new() -> Option<Self> {
        Some(Self)
    }
}

impl RcReceiver for VirtualRc {
    fn read(&mut self) -> RcInput {
        let f: Frame = unsafe { PLAYBACK.current() };
        let r = f.rc;
        RcInput {
            throttle: r[0],
            roll: r[1],
            pitch: r[2],
            yaw: r[3],
            armed: RC_FORCE_ARM,
            mode: (r[4].clamp(0.0, 1.0) * 255.0) as u8,
            fresh: true,
        }
    }

    fn healthy(&self) -> bool {
        true
    }
}
