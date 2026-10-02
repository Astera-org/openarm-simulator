#![deny(unsafe_code)]
//! Shared simulator administration models. This crate has no engine or transport
//! dependencies. Motor control still goes through CAN.
pub use damiao_can::MotorStatus;
pub use mint;
pub use mujoco_core::{
    ActuatorIndex, BodyIndex, GeomIndex, Integrator, JointIndex, JointKind, SiteIndex,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::ops::Div;
pub use uom;
mod scene;
use mint::{Point3, Vector3};
pub use scene::*;
use uom::si::{
    angle::radian,
    angular_velocity::radian_per_second,
    f64::{
        Acceleration, Angle, AngularVelocity, Force, Length, Mass, MomentOfInertia,
        ThermodynamicTemperature, Torque,
    },
    torque::newton_meter,
};

pub type AngularStiffness = <Torque as Div<Angle>>::Output;
pub type AngularDamping = <Torque as Div<AngularVelocity>>::Output;

/// Targets and gains last received by a motor in MIT mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Deserialize, Serialize)]
pub struct MotorCommand {
    #[serde(rename = "kp_nm_per_rad")]
    pub kp: AngularStiffness,
    #[serde(rename = "kd_nm_s_per_rad")]
    pub kd: AngularDamping,
    #[serde(rename = "q_rad")]
    pub q: Angle,
    #[serde(rename = "dq_rad_s")]
    pub dq: AngularVelocity,
    #[serde(rename = "tau_nm")]
    pub tau: Torque,
}
impl From<damiao_can::MitCommand> for MotorCommand {
    fn from(command: damiao_can::MitCommand) -> Self {
        Self {
            kp: Torque::new::<newton_meter>(command.kp) / Angle::new::<radian>(1.),
            kd: Torque::new::<newton_meter>(command.kd)
                / AngularVelocity::new::<radian_per_second>(1.),
            q: Angle::new::<radian>(command.q),
            dq: AngularVelocity::new::<radian_per_second>(command.dq),
            tau: Torque::new::<newton_meter>(command.tau),
        }
    }
}
impl From<MotorCommand> for damiao_can::MitCommand {
    fn from(command: MotorCommand) -> Self {
        Self {
            kp: command.kp.value,
            kd: command.kd.value,
            q: command.q.get::<radian>(),
            dq: command.dq.get::<radian_per_second>(),
            tau: command.tau.get::<newton_meter>(),
        }
    }
}

/// PMAX/VMAX/TMAX register settings used to interpret motor messages.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub struct MappingRanges {
    #[serde(rename = "pmax_rad")]
    pub pmax: uom::si::f32::Angle,
    #[serde(rename = "vmax_rad_s")]
    pub vmax: uom::si::f32::AngularVelocity,
    #[serde(rename = "tmax_nm")]
    pub tmax: uom::si::f32::Torque,
}
impl From<damiao_can::MappingRanges> for MappingRanges {
    fn from(ranges: damiao_can::MappingRanges) -> Self {
        Self {
            pmax: uom::si::f32::Angle::new::<radian>(ranges.pmax),
            vmax: uom::si::f32::AngularVelocity::new::<radian_per_second>(ranges.vmax),
            tmax: uom::si::f32::Torque::new::<newton_meter>(ranges.tmax),
        }
    }
}
impl From<MappingRanges> for damiao_can::MappingRanges {
    fn from(ranges: MappingRanges) -> Self {
        Self {
            pmax: ranges.pmax.get::<radian>(),
            vmax: ranges.vmax.get::<radian_per_second>(),
            tmax: ranges.tmax.get::<newton_meter>(),
        }
    }
}

/// MuJoCo constraint reference parameters in its native `solref` format.
pub type Solref = [f64; 2];
/// MuJoCo constraint impedance parameters in its native `solimp` format.
pub type Solimp = [f64; 5];

/// Persistent torques in Nm, keyed by MuJoCo hinge-joint index. Omitted joints receive zero.
pub type Push = BTreeMap<JointIndex, Torque>;

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct PushRequest {
    #[serde(rename = "torques_nm")]
    pub torques: Push,
}

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
    #[serde(rename = "q_rad")]
    pub q: Angle,
    #[serde(rename = "dq_rad_s")]
    pub dq: AngularVelocity,
    #[serde(rename = "torque_nm")]
    pub torque: Torque,
    pub status: MotorStatus,
    #[serde(rename = "mos_temperature_k")]
    pub mos_temperature: ThermodynamicTemperature,
    #[serde(rename = "rotor_temperature_k")]
    pub rotor_temperature: ThermodynamicTemperature,
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
    #[serde(rename = "phase_rad")]
    pub phase: Angle,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Stribeck {
    #[serde(rename = "breakaway_nm")]
    pub breakaway: Torque,
    #[serde(rename = "velocity_rad_s")]
    pub velocity: AngularVelocity,
    #[serde(default)]
    pub direction_asymmetry: f64,
    #[serde(default)]
    pub angle: Option<AngleModulation>,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum JointParameters {
    Hinge(HingeJointParameters),
    Slide(SlideJointParameters),
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HingeJointParameters {
    #[serde(rename = "frictionloss_nm")]
    pub frictionloss: Option<Torque>,
    #[serde(rename = "damping_nm_s_per_rad")]
    pub damping: Option<AngularDamping>,
    #[serde(rename = "stiffness_nm_per_rad")]
    pub stiffness: Option<AngularStiffness>,
    #[serde(rename = "springref_rad")]
    pub springref: Option<Angle>,
    pub stribeck: Option<Stribeck>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SlideJointParameters {
    #[serde(rename = "frictionloss_n")]
    pub frictionloss: Option<Force>,
    #[serde(rename = "damping_n_s_per_m")]
    pub damping: Option<SpringDamping>,
    #[serde(rename = "stiffness_n_per_m")]
    pub stiffness: Option<SpringStiffness>,
    #[serde(rename = "springref_m")]
    pub springref: Option<Length>,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct BodyParameters {
    #[serde(rename = "mass_kg")]
    pub mass: Mass,
    #[serde(rename = "com_m")]
    pub com: Point3<Length>,
    #[serde(rename = "inertia_kg_m2")]
    pub inertia: Vector3<MomentOfInertia>,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct Plant {
    pub friction_model: String,
    pub joints: BTreeMap<JointIndex, JointParameters>,
    #[serde(rename = "applied_torque_nm")]
    pub applied_torque: Push,
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
    /// Body states in MuJoCo index order, including unnamed bodies and world at index zero.
    pub bodies: Vec<BodyState>,
    pub springs: BTreeMap<String, SpringState>,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct PhysicsConfiguration {
    pub friction_model: String,
    pub mujoco_version: String,
    pub timestep_ns: u64,
    pub integrator: Integrator,
    #[serde(rename = "gravity_m_s2")]
    pub gravity: Vector3<Acceleration>,
    pub enhanced_friction_solref: Solref,
    pub enhanced_friction_solimp: Solimp,
    pub joints: BTreeMap<JointIndex, JointParameters>,
    pub bodies: Vec<BodyParameters>,
    #[serde(rename = "encoder_offsets_rad")]
    pub encoder_offsets: BTreeMap<ActuatorIndex, Angle>,
}
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
pub struct Configuration {
    pub configuration: PhysicsConfiguration,
}
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
pub struct ErrorResponse {
    pub error: String,
}
