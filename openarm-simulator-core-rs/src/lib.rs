//! Shared simulator administration models. This crate has no engine or transport
//! dependencies. Motor control still goes through CAN.
pub use damiao_can_rs::{MappingRanges, MitCommand as MotorCommand, MotorStatus};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Arm {
    Left,
    Right,
}

pub type Pose = [f64; 8];
pub type JointTorques = [f64; 7];
pub type Solref = [f64; 2];
pub type Solimp = [f64; 5];

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct Arms<T> {
    pub left: T,
    pub right: T,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ArmOptions<T> {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub left: Option<T>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub right: Option<T>,
}

/// Persistent external torques in Nm. Omitted arms receive zero torque.
pub type Push = ArmOptions<JointTorques>;

/// Patch a motor's fault. None leaves the field unchanged; status 0 clears the
/// fault and disables the motor. Enabling a motor remains a CAN operation.
#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Fault {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub status: Option<MotorStatus>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub silent: Option<bool>,
}
pub type FaultRequest = (Arm, usize, Fault);

/// Elapsed simulated nanoseconds. Requires a paused clock; fractional update
/// intervals carry into later advances. Completion is not a CAN consumer barrier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Advance {
    pub duration_ns: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct MotorState {
    pub joint: usize,
    pub command: MotorCommand,
    pub q: f64,
    pub dq: f64,
    pub torque: f64,
    pub status: MotorStatus,
    pub mos_temperature: u8,
    pub rotor_temperature: u8,
    pub silent: bool,
    pub ranges: MappingRanges,
}
pub type ArmStates = Arms<[MotorState; 8]>;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Deserialize, Serialize)]
pub struct Statistics {
    pub commands: u64,
    pub replies: u64,
    pub dropped: u64,
    pub steps: u64,
    pub max_lag_ns: u64,
    pub max_catchup_steps: u64,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AngleModulation {
    /// Fractional dry-friction modulation in [0, 1).
    pub amplitude: f64,
    /// Cycles per physical joint revolution, a positive integer.
    pub harmonic: u32,
    pub phase_rad: f64,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Stribeck {
    pub breakaway_nm: f64,
    pub velocity_rad_s: f64,
    #[serde(default)]
    pub direction_asymmetry: f64,
    #[serde(default)]
    pub angle: Option<AngleModulation>,
}
#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct JointParameters {
    pub frictionloss: Option<f64>,
    pub damping: Option<f64>,
    pub stiffness: Option<f64>,
    pub springref: Option<f64>,
    pub stribeck: Option<Stribeck>,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BodyParameters {
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 3],
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct Plant {
    pub friction_model: String,
    pub joints: BTreeMap<String, JointParameters>,
    pub applied_torque_nm: Arms<JointTorques>,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct State {
    pub state: ArmStates,
    pub statistics: Statistics,
    pub time_ns: u64,
    pub paused: bool,
    pub advancing: bool,
    pub mujoco_version: String,
    pub timestep_ns: u64,
    pub plant: Plant,
    pub joint_stop_solref: Solref,
    pub joint_stop_solimp: Solimp,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct PhysicsConfiguration {
    pub friction_model: String,
    pub mujoco_version: String,
    pub timestep_ns: u64,
    pub integrator: String,
    pub gravity_m_s2: [f64; 3],
    pub joint_stop_solref: Solref,
    pub joint_stop_solimp: Solimp,
    pub enhanced_friction_solref: Solref,
    pub enhanced_friction_solimp: Solimp,
    pub gripper_radius_m: f64,
    pub joints: BTreeMap<String, JointParameters>,
    pub bodies: BTreeMap<String, BodyParameters>,
    pub encoder_offsets_rad: Arms<Pose>,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct Configuration {
    pub configuration: PhysicsConfiguration,
}
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
pub struct ErrorResponse {
    pub error: String,
}
