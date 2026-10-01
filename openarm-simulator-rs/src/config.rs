use damiao_simulator::MotorConfig;
use openarm_simulator_core::uom::si::f64::Angle;
use openarm_simulator_core::{BodyParameters, JointParameters};
use serde::Deserialize;
use std::collections::BTreeMap;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct MotorBinding {
    pub bus: String,
    pub actuator: String,
    pub controller: MotorConfig,
    #[serde(default, rename = "encoder_offset_rad")]
    pub encoder_offset: Angle,
}

pub const DEFAULT_TIMESTEP_NS: u64 = 500_000;

#[derive(Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    pub buses: BTreeMap<String, String>,
    pub motors: BTreeMap<String, MotorBinding>,
    pub timestep_ns: u64,
    pub bodies: BTreeMap<String, BodyParameters>,
    pub joints: BTreeMap<String, JointParameters>,
    pub friction_scale: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            buses: BTreeMap::new(),
            motors: BTreeMap::new(),
            timestep_ns: DEFAULT_TIMESTEP_NS,
            bodies: BTreeMap::new(),
            joints: BTreeMap::new(),
            friction_scale: 1.,
        }
    }
}
