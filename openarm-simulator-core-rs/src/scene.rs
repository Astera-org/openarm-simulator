use crate::{ActuatorIndex, BodyIndex, GeomIndex, JointIndex, SiteIndex};
use mint::{Point3, Quaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, ops::Div};
use uom::si::f64::{Force, Length, Torque, Velocity};

pub type SpringStiffness = <Force as Div<Length>>::Output;
pub type SpringDamping = <Force as Div<Velocity>>::Output;

/// A spring and axial damper between two MJCF sites, acting in tension and compression.
/// Coincident endpoints exert no force. These springs are separate from MJCF tendons.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Spring {
    pub sites: [SiteIndex; 2],
    #[serde(rename = "rest_length_m")]
    pub rest_length: Length,
    #[serde(rename = "stiffness_n_per_m")]
    pub stiffness: SpringStiffness,
    #[serde(rename = "damping_n_s_per_m")]
    pub damping: SpringDamping,
}

/// A persistent load at an MJCF site. Force and torque are expressed in world axes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AppliedForce {
    pub site: SiteIndex,
    #[serde(rename = "force_world_n")]
    pub force: Vector3<Force>,
    #[serde(rename = "torque_world_nm")]
    pub torque: Vector3<Torque>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SiteState {
    #[serde(rename = "position_world_m")]
    pub position: Point3<Length>,
    #[serde(rename = "orientation_world_xyzw")]
    pub orientation: Quaternion<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyState {
    #[serde(rename = "position_world_m")]
    pub position: Point3<Length>,
    #[serde(rename = "orientation_world_xyzw")]
    pub orientation: Quaternion<f64>,
    #[serde(rename = "com_world_m")]
    pub com: Point3<Length>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpringState {
    #[serde(rename = "length_m")]
    pub length: Length,
    #[serde(rename = "velocity_m_s")]
    pub velocity: Velocity,
}

/// Named scene objects and their MuJoCo indices, stable for the loaded model across resets.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SceneNames {
    pub bodies: BTreeMap<String, BodyIndex>,
    pub joints: BTreeMap<String, JointIndex>,
    pub actuators: BTreeMap<String, ActuatorIndex>,
    pub geoms: BTreeMap<String, GeomIndex>,
    pub sites: BTreeMap<String, SiteIndex>,
}
