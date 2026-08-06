//! flyctrl-sim: host 端物理仿真后端。
//!
//! 提供较真实的六自由度刚体动力学 + 电机推力/力矩模型 + 气动系数，
//! 让控制律在"接近真机"的环境下被验证，而不仅是理想积分。
//!
//! 仿真与核心控制逻辑解耦：仿真只消费 [`ActuatorCmd`]、产出 [`ImuSample`]，
//! 不关心用的是 PID 还是 LQR。

pub mod physics;
pub mod world;
pub mod harness;
