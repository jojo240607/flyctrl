//! 通信与地面站接口（M6）。
//!
//! 分层：
//! - [`link`]: 物理链路抽象（`Link` trait + `Frame` + host mock），与具体总线解耦。
//! - [`mavlink`]: MAVLink 兼容消息层（固定大小帧、无堆），可被 QGC 类地面站解析。
//! - [`telemetry`]: 内部遥测/日志通道，把 [`VehicleState`] 序列化为 MAVLink 帧，
//!   经共享 ring buffer 异步送出（不阻塞控制回路）。
//!
//! 全部 `no_std`、无堆分配、执行时间有界。

pub mod link;
pub mod mavlink;
pub mod telemetry;
