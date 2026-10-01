//! Shared simulator administration models. This crate has no engine or transport
//! dependencies. Motor control still goes through CAN.
pub use damiao_can_rs::{MappingRanges, MitCommand as MotorCommand, MotorStatus};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub type Solref = [f64; 2];
pub type Solimp = [f64; 5];

/// Persistent torques in Nm, keyed by scene hinge-joint name. Omitted joints receive zero.
pub type Push = BTreeMap<String, f64>;

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
pub type FaultRequest = (String, Fault);

/// Elapsed simulated nanoseconds. Requires a paused clock; fractional update
/// intervals carry into later advances. Completion is not a CAN consumer barrier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Advance {
    pub duration_ns: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct MotorState {
    pub id: u16,
    pub reply_id: u16,
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
pub type MotorStates = BTreeMap<String, MotorState>;

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
    pub applied_torque_nm: Push,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct State {
    pub state: MotorStates,
    pub statistics: Statistics,
    pub time_ns: u64,
    pub paused: bool,
    pub advancing: bool,
    pub mujoco_version: String,
    pub timestep_ns: u64,
    pub plant: Plant,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct PhysicsConfiguration {
    pub friction_model: String,
    pub mujoco_version: String,
    pub timestep_ns: u64,
    pub integrator: String,
    pub gravity_m_s2: [f64; 3],
    pub enhanced_friction_solref: Solref,
    pub enhanced_friction_solimp: Solimp,
    pub joints: BTreeMap<String, JointParameters>,
    pub bodies: BTreeMap<String, BodyParameters>,
    pub encoder_offsets_rad: BTreeMap<String, f64>,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct Configuration {
    pub configuration: PhysicsConfiguration,
}
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
pub struct ErrorResponse {
    pub error: String,
}
