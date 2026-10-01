//! OpenArm plant configuration and motor dynamics on top of the MuJoCo wrapper.
use crate::{friction, motor::Motor};
use anyhow::{Context, Result, ensure};
use mujoco_rs::{Data, JOINT_HINGE, JOINT_SLIDE, Model, NBIAS, NIMP, NREF, Object, Spec};
use damiao_can_rs::{MitCommand, MotorStatus};
use openarm_simulator_core_rs::{
    ArmOptions as Arms, ArmStates, Arms as ArmValues, BodyParameters, JointParameters,
    PhysicsConfiguration, Plant, Pose, Stribeck,
};
use serde::Deserialize;
use std::{collections::BTreeMap, path::Path, time::Duration};

pub const SIDES: [&str; 2] = ["right", "left"];
pub const DEFAULT_TIMESTEP_NS: u64 = 500_000;
#[cfg(test)]
const STEP: f64 = DEFAULT_TIMESTEP_NS as f64 / 1_000_000_000.;
// Motor-to-finger conversion from enactic/openarm_ros2@4e837e1d0dae69,
// openarm_hardware/include/openarm_hardware/openarm_simple_hardware.hpp:
// GRIPPER_JOINT_0_POSITION / -GRIPPER_MOTOR_1_RADIANS. Preserve its rounding.
#[allow(clippy::approx_constant)]
pub const RADIUS: f64 = 0.044 / 1.0472;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    /// Fixed integration interval; the scheduler and API use integer nanoseconds.
    #[serde(default = "default_timestep")]
    pub timestep_ns: u64,
    #[serde(default)]
    pub poses: Arms<Pose>,
    #[serde(default)]
    pub offsets: Arms<Pose>,
    #[serde(default)]
    pub bodies: BTreeMap<String, BodyParameters>,
    #[serde(default)]
    pub joints: BTreeMap<String, JointParameters>,
    #[serde(default = "one")]
    pub friction_scale: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            timestep_ns: DEFAULT_TIMESTEP_NS,
            poses: Arms::default(),
            offsets: Arms::default(),
            bodies: BTreeMap::new(),
            joints: BTreeMap::new(),
            friction_scale: 1.,
        }
    }
}

fn default_timestep() -> u64 {
    DEFAULT_TIMESTEP_NS
}

fn one() -> f64 {
    1.
}

struct VariableFriction {
    qpos: usize,
    dof: usize,
    sliding_nm: f64,
    law: Stribeck,
}

#[derive(Clone, Copy, Default)]
struct Index {
    qpos: [usize; 9],
    dof: [usize; 9],
    actuator: [usize; 8],
    ctrl: [usize; 8],
    force: [usize; 8],
}

pub struct Physics {
    model: Model,
    data: Data,
    index: [Index; 2],
    offsets: [Pose; 2],
    initial_poses: [Pose; 2],
    pub timestep_ns: u64,
    friction: Vec<VariableFriction>,
    joint_parameters: BTreeMap<String, JointParameters>,
    applied_torque: [[f64; 7]; 2],
    pub motors: [[Motor; 8]; 2],
}

impl Physics {
    pub fn load(path: &Path, config: Config) -> Result<Self> {
        ensure!(config.timestep_ns > 0, "timestep_ns must be positive");
        let mut spec = Spec::from_xml(path)?;
        for side in SIDES {
            spec.exclude_contact(
                &format!("openarm_{side}_right_finger"),
                &format!("openarm_{side}_left_finger"),
            )?;
            let force_range = spec.actuator_force_range(&format!("{side}_joint7_ctrl"))?;
            spec.set_actuator_tendon(
                &format!("{side}_finger1_ctrl"),
                &format!("split_{side}"),
                -1. / RADIUS,
                force_range,
            )?;
            spec.remove_actuator(&format!("{side}_finger2_ctrl"))?;
        }
        let mut model = spec.compile()?;
        model.set_timestep(Duration::from_nanos(config.timestep_ns))?;
        model.use_implicit_fast();
        model.use_affine_actuators();
        {
            let m = model.view_mut();
            m.jnt_solref
                .chunks_mut(NREF)
                .for_each(|a| a.copy_from_slice(&[0.002, 1.]));
            m.jnt_solimp
                .chunks_mut(NIMP)
                .for_each(|a| a[..3].copy_from_slice(&[0.99, 0.999, 0.001]));
        }
        let mut index = [Index::default(); 2];
        {
            let m = model.view();
            for (side, name) in SIDES.iter().enumerate() {
                for joint in 0..9 {
                    let label = if joint < 7 {
                        format!("openarm_{name}_joint{}", joint + 1)
                    } else {
                        format!("openarm_{name}_finger_joint{}", joint - 6)
                    };
                    let j = model.id(Object::Joint, &label)?;
                    ensure!(
                        matches!(m.jnt_type[j], JOINT_HINGE | JOINT_SLIDE),
                        "motor joint must have one scalar degree of freedom"
                    );
                    index[side].qpos[joint] = m.jnt_qposadr[j] as usize;
                    index[side].dof[joint] = m.jnt_dofadr[j] as usize;
                    ensure!(
                        index[side].qpos[joint] < m.nq && index[side].dof[joint] < m.nv,
                        "invalid joint address"
                    );
                }
                for joint in 0..8 {
                    let label = if joint < 7 {
                        format!("{name}_joint{}_ctrl", joint + 1)
                    } else {
                        format!("{name}_finger1_ctrl")
                    };
                    let a = model.id(Object::Actuator, &label)?;
                    ensure!(
                        m.actuator_ctrlnum[a] == 1 && m.actuator_outnum[a] == 1,
                        "MIT requires scalar actuator control/force"
                    );
                    index[side].actuator[joint] = a;
                    index[side].ctrl[joint] = m.actuator_ctrladr[a] as usize;
                    index[side].force[joint] = m.actuator_outadr[a] as usize;
                    ensure!(
                        index[side].ctrl[joint] < m.nu && index[side].force[joint] < m.nout,
                        "invalid actuator control/output address"
                    );
                }
            }
        }
        let mut friction = Vec::new();
        let mut joint_parameters = BTreeMap::new();
        // Experimental assembly changes live only in this simulator's model.
        // Never edit the upstream XML or the estimator's nominal description.
        {
            ensure!(
                config.friction_scale.is_finite() && config.friction_scale >= 0.,
                "invalid friction scale"
            );
            for (name, body) in &config.bodies {
                let id = model.id(Object::Body, name)?;
                let m = model.view_mut();
                ensure!(
                    id > 0
                        && (name.starts_with("openarm_right_")
                            || name.starts_with("openarm_left_")),
                    "invalid perturbed arm body {name}"
                );
                ensure!(
                    body.mass.is_finite()
                        && body.mass > 0.
                        && body.com.iter().all(|x| x.is_finite())
                        && body.inertia.iter().all(|x| x.is_finite() && *x > 0.),
                    "invalid inertial parameters for {name}"
                );
                let sum: f64 = body.inertia.iter().sum();
                ensure!(
                    body.inertia.iter().all(|x| 2. * x <= sum + 1e-12),
                    "inertia triangle inequality for {name}"
                );
                m.body_mass[id] = body.mass;
                m.body_ipos[3 * id..3 * id + 3].copy_from_slice(&body.com);
                m.body_inertia[3 * id..3 * id + 3].copy_from_slice(&body.inertia);
            }
            let m = model.view_mut();
            for ix in &index {
                for dof in &ix.dof[..7] {
                    m.dof_frictionloss[*dof] *= config.friction_scale;
                    m.dof_damping[*dof] *= config.friction_scale;
                }
            }
            for (name, parameters) in &config.joints {
                let id = model.id(Object::Joint, name)?;
                let m = model.view_mut();
                let joint = id;
                let dof = m.jnt_dofadr[joint] as usize;
                let qpos = m.jnt_qposadr[joint] as usize;
                ensure!(
                    m.jnt_type[joint] == JOINT_HINGE
                        && index.iter().any(|ix| ix.dof[..7].contains(&dof)),
                    "only arm hinge joints can be perturbed: {name}"
                );
                for (value, field, address) in [
                    (
                        parameters.frictionloss,
                        "frictionloss",
                        &mut m.dof_frictionloss[dof],
                    ),
                    (parameters.damping, "damping", &mut m.dof_damping[dof]),
                    (
                        parameters.stiffness,
                        "stiffness",
                        &mut m.jnt_stiffness[joint],
                    ),
                ] {
                    if let Some(value) = value {
                        ensure!(
                            value.is_finite() && value >= 0.,
                            "invalid {field} for {name}"
                        );
                        *address = value;
                    }
                }
                if let Some(value) = parameters.springref {
                    ensure!(value.is_finite(), "invalid springref for {name}");
                    m.qpos_spring[qpos] = value;
                }
            }
            for (side, ix) in index.iter().enumerate() {
                for j in 0..7 {
                    let name = format!("openarm_{}_joint{}", SIDES[side], j + 1);
                    let id = model.id(Object::Joint, &name)?;
                    let m = model.view_mut();
                    let sliding_nm = m.dof_frictionloss[ix.dof[j]];
                    let damping = m.dof_damping[ix.dof[j]];
                    ensure!(
                        sliding_nm.is_finite() && damping.is_finite(),
                        "friction scaling overflow for {name}"
                    );
                    let law = config.joints.get(&name).and_then(|p| p.stribeck.clone());
                    if let Some(law) = &law {
                        friction::validate(law, sliding_nm)
                            .with_context(|| format!("invalid friction for {name}"))?;
                        // Default soft friction can creep across the Stribeck
                        // band under a sub-breakaway load on small wrist inertia.
                        // Enhanced joints use a firmer, still regularized native
                        // constraint. Basic joints keep their upstream settings.
                        m.dof_solref[ix.dof[j] * NREF..ix.dof[j] * NREF + NREF]
                            .copy_from_slice(&[0.002, 1.]);
                        m.dof_solimp[ix.dof[j] * NIMP..ix.dof[j] * NIMP + NIMP]
                            .copy_from_slice(&[0.999, 0.999, 0.001, 0.5, 2.]);
                        friction.push(VariableFriction {
                            qpos: ix.qpos[j],
                            dof: ix.dof[j],
                            sliding_nm,
                            law: law.clone(),
                        });
                    }
                    joint_parameters.insert(
                        name,
                        JointParameters {
                            frictionloss: Some(sliding_nm),
                            damping: Some(damping),
                            stiffness: Some(m.jnt_stiffness[id]),
                            springref: Some(m.qpos_spring[ix.qpos[j]]),
                            stribeck: law,
                        },
                    );
                }
            }
        }
        let mut data = Data::new(&model)?;
        model.set_constants(&mut data);
        let offsets = [config.offsets.right, config.offsets.left].map(|a| a.unwrap_or([0.; 8]));
        let mut zero = [0.; 8];
        zero[7] = -10f64.to_radians();
        let initial_poses = [config.poses.right, config.poses.left].map(|a| a.unwrap_or(zero));
        let mut world = Self {
            model,
            data,
            index,
            offsets,
            initial_poses,
            timestep_ns: config.timestep_ns,
            friction,
            joint_parameters,
            applied_torque: [[0.; 7]; 2],
            motors: std::array::from_fn(|_| std::array::from_fn(|i| Motor::new(i + 1))),
        };
        world.reset()?;
        Ok(world)
    }

    pub fn reset(&mut self) -> Result<()> {
        let poses = self.initial_poses;
        ensure!(
            poses
                .iter()
                .chain(&self.offsets)
                .flatten()
                .all(|v| v.is_finite()),
            "poses/offsets must be finite"
        );
        self.model.reset_data(&mut self.data);
        {
            let d = self.data.view_mut();
            for (side, pose) in poses.iter().enumerate() {
                for (j, q) in pose[..7].iter().enumerate() {
                    d.qpos[self.index[side].qpos[j]] = *q;
                }
                for j in 7..9 {
                    d.qpos[self.index[side].qpos[j]] = -pose[7] * RADIUS;
                }
                self.motors[side] = std::array::from_fn(|i| Motor::new(i + 1));
            }
        }
        self.applied_torque = [[0.; 7]; 2];
        self.configure();
        self.configure_friction();
        self.model.forward(&mut self.data);
        self.feedback();
        Ok(())
    }

    fn configure(&mut self) {
        {
            let (m, d) = (self.model.view_mut(), self.data.view_mut());
            for side in 0..2 {
                for j in 0..8 {
                    let motor = self.motors[side][j];
                    let a = self.index[side].actuator[j];
                    let c = if motor.status == MotorStatus::ENABLED {
                        motor.command
                    } else {
                        MitCommand::default()
                    };
                    d.ctrl[self.index[side].ctrl[j]] =
                        c.kp * (c.q - self.offsets[side][j]) + c.kd * c.dq + c.tau;
                    m.actuator_biasprm[a * NBIAS + 1] = -c.kp;
                    m.actuator_biasprm[a * NBIAS + 2] = -c.kd;
                }
            }
        }
    }

    fn configure_friction(&mut self) {
        // Freeze a positive friction bound over this implicit step. MuJoCo's
        // native constraint chooses the opposing force and handles stiction;
        // no explicit sign(v) torque, hidden integrator or bristle state is added.
        {
            let (m, d) = (self.model.view_mut(), self.data.view_mut());
            for joint in &self.friction {
                m.dof_frictionloss[joint.dof] = friction::bound_nm(
                    &joint.law,
                    joint.sliding_nm,
                    d.qpos[joint.qpos],
                    d.qvel[joint.dof],
                );
            }
        }
    }

    pub fn step(&mut self, count: u64) -> Result<()> {
        self.configure();
        for _ in 0..count {
            self.configure_friction();
            self.model.step(&mut self.data);
        }
        ensure!(
            self.data.warnings().iter().all(|w| w.number == 0)
                && self.data.view().qpos.iter().all(|q| q.is_finite()),
            "MuJoCo numerical warning; simulation stopped"
        );
        self.feedback();
        Ok(())
    }

    fn feedback(&mut self) {
        {
            let d = self.data.view();
            for side in 0..2 {
                for j in 0..8 {
                    let ix = self.index[side];
                    let (q, dq) = if j < 7 {
                        (d.qpos[ix.qpos[j]], d.qvel[ix.dof[j]])
                    } else {
                        (
                            -(d.qpos[ix.qpos[7]] + d.qpos[ix.qpos[8]]) / (2. * RADIUS),
                            -(d.qvel[ix.dof[7]] + d.qvel[ix.dof[8]]) / (2. * RADIUS),
                        )
                    };
                    let motor = &mut self.motors[side][j];
                    motor.q = q + self.offsets[side][j];
                    motor.dq = dq;
                    motor.torque = if motor.status == MotorStatus::ENABLED {
                        d.actuator_force[ix.force[j]]
                    } else {
                        0.
                    };
                }
            }
        }
    }

    pub fn snapshot(&self) -> ArmStates {
        ArmValues {
            right: self.motors[0].map(|m| m.snapshot()),
            left: self.motors[1].map(|m| m.snapshot()),
        }
    }
    pub fn version() -> String {
        mujoco_rs::version()
    }

    pub fn push(&mut self, forces: [[f64; 7]; 2]) -> Result<()> {
        ensure!(
            forces.iter().flatten().all(|v| v.is_finite()),
            "invalid applied torque"
        );
        {
            let d = self.data.view_mut();
            for (side, arm) in forces.iter().enumerate() {
                for (j, force) in arm.iter().enumerate() {
                    d.qfrc_applied[self.index[side].dof[j]] = *force;
                }
            }
        }
        self.applied_torque = forces;
        Ok(())
    }

    pub fn parameters(&self) -> Plant {
        Plant {
            friction_model: "mujoco-dry-viscous-with-optional-stribeck-v1".into(),
            joints: self.joint_parameters.clone(),
            applied_torque_nm: ArmValues {
                right: self.applied_torque[0],
                left: self.applied_torque[1],
            },
        }
    }

    pub fn configuration(&self) -> PhysicsConfiguration {
        PhysicsConfiguration {
            friction_model: "mujoco-dry-viscous-with-optional-stribeck-v1".into(),
            mujoco_version: Self::version(),
            timestep_ns: self.timestep_ns,
            integrator: "implicitfast".into(),
            gravity_m_s2: self.model.gravity(),
            joint_stop_solref: [0.002, 1.],
            joint_stop_solimp: [0.99, 0.999, 0.001, 0.5, 2.],
            enhanced_friction_solref: [0.002, 1.],
            enhanced_friction_solimp: [0.999, 0.999, 0.001, 0.5, 2.],
            gripper_radius_m: RADIUS,
            joints: self.joint_parameters.clone(),
            bodies: self.body_parameters(),
            encoder_offsets_rad: ArmValues {
                right: self.offsets[0],
                left: self.offsets[1],
            },
        }
    }

    fn body_parameters(&self) -> BTreeMap<String, BodyParameters> {
        let model = &self.model;
        let m = model.view();
        (1..m.nbody)
            .filter_map(|i| {
                let name = model.name(Object::Body, i)?;
                if !name.starts_with("openarm_right_") && !name.starts_with("openarm_left_") {
                    return None;
                }
                Some((
                    name.to_owned(),
                    BodyParameters {
                        mass: m.body_mass[i],
                        com: m.body_ipos[3 * i..3 * i + 3].try_into().unwrap(),
                        inertia: m.body_inertia[3 * i..3 * i + 3].try_into().unwrap(),
                    },
                ))
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn world(config: Config) -> Physics {
        Physics::load(std::path::Path::new(openarm_test_model::SCENE), config).unwrap()
    }

    #[test]
    fn nonfinite_state_is_rejected_until_reset() {
        let mut p = world(Config::default());
        p.data.view_mut().qpos[0] = f64::NAN;
        assert_eq!(
            p.step(0).unwrap_err().to_string(),
            "MuJoCo numerical warning; simulation stopped"
        );
        p.reset().unwrap();
        p.step(1).unwrap();
    }

    #[test]
    fn gravity_and_holding() {
        let mut pose = [0.; 8];
        pose[3] = 45f64.to_radians();
        pose[7] = -0.2;
        let mut p = world(Config {
            poses: Arms {
                right: Some(pose),
                left: Some(pose),
            },
            ..Config::default()
        });
        p.step(200).unwrap();
        assert!((p.motors[0][3].q - pose[3]).abs() > 0.001);
        p.reset().unwrap();
        for arm in &mut p.motors {
            for m in arm {
                m.status = MotorStatus::ENABLED;
                m.command = MitCommand {
                    kp: 150.,
                    kd: 2.,
                    q: m.q,
                    ..MitCommand::default()
                };
            }
        }
        p.step(1000).unwrap();
        assert!((p.motors[0][3].q - pose[3]).abs() < 0.08);
        assert!(p.motors[0][3].torque.abs() > 0.1);
    }

    #[test]
    fn gripper_transmission_loaded_stop_and_no_teleport() {
        let mut p = world(Config::default());
        p.motors[0][7].status = MotorStatus::ENABLED;
        p.motors[0][7].command = MitCommand {
            kp: 10.,
            kd: 0.9,
            q: -0.5,
            ..MitCommand::default()
        };
        p.step(1).unwrap();
        assert!((p.motors[0][7].q + 0.5).abs() > 0.1);
        p.step(1000).unwrap();
        assert!((p.motors[0][7].q + 0.5).abs() < 0.01);
        {
            for i in &p.index[0].qpos[7..] {
                assert!((p.data.view().qpos[*i] - 0.5 * RADIUS).abs() < 0.001);
            }
        }
        p.motors[0][7].command = MitCommand {
            kp: 45.,
            kd: 1.2,
            q: 0.1,
            ..MitCommand::default()
        };
        p.step(2000).unwrap();
        let m = p.motors[0][7];
        assert!(m.dq.abs() < 0.01 && m.torque > 0.3 && m.q > 0. && m.q < 0.2f64.to_radians());
    }

    #[test]
    fn joint_target_is_physics_driven_and_force_is_bounded() {
        let mut p = world(Config::default());
        p.motors[0][6].status = MotorStatus::ENABLED;
        p.motors[0][6].command = MitCommand {
            kp: 10.,
            kd: 0.5,
            q: 0.3,
            ..MitCommand::default()
        };
        p.step(1).unwrap();
        assert!(p.motors[0][6].q < 0.1);
        p.step(1000).unwrap();
        assert!((p.motors[0][6].q - 0.3).abs() < 0.05);
        assert!(p.motors[1][6].q.abs() < 0.05);
        p.motors[0][6].command.q = 10.;
        p.step(1).unwrap();
        assert!((p.motors[0][6].torque - 7.).abs() < 1e-6);
        p.motors[0][6].status = MotorStatus::DISABLED;
        p.step(1).unwrap();
        assert_eq!(p.motors[0][6].torque, 0.);
    }

    #[test]
    fn encoder_bias_and_reset() {
        let mut p = world(Config {
            offsets: Arms {
                right: Some([0.01; 8]),
                left: None,
            },
            ..Config::default()
        });
        assert_eq!(p.motors[0][6].q, 0.01);
        p.motors[0][6].status = MotorStatus::ENABLED;
        p.motors[0][6].command = MitCommand {
            kp: 30.,
            kd: 0.8,
            q: 0.3,
            ..MitCommand::default()
        };
        p.step(1000).unwrap();
        assert!((p.motors[0][6].q - 0.3).abs() < 0.02);
        p.motors[0][6].silent = true;
        p.reset().unwrap();
        assert!(
            !p.motors[0][6].silent
                && p.motors[0][6].status == MotorStatus::DISABLED
                && p.motors[0][6].command.kp == 0.
        );
    }

    #[test]
    fn per_joint_passive_parameters_override_scaling_and_survive_reset() {
        let parameters = JointParameters {
            frictionloss: Some(0.23),
            damping: Some(0.61),
            stiffness: Some(0.47),
            springref: Some(0.19),
            ..JointParameters::default()
        };
        let mut p = world(Config {
            friction_scale: 0.5,
            joints: BTreeMap::from([("openarm_right_joint4".into(), parameters)]),
            ..Config::default()
        });
        p.reset().unwrap();
        {
            let (m, d) = (p.model.view(), p.data.view());
            let dof = p.index[0].dof[3];
            assert_eq!(m.dof_frictionloss[dof], 0.23);
            assert_eq!(m.dof_damping[dof], 0.61);
            assert!((d.qfrc_passive[dof] - 0.47 * 0.19).abs() < 1e-12);
            assert!((m.dof_frictionloss[p.index[1].dof[3]] - 0.05).abs() < 1e-12);
        }
    }

    #[test]
    fn reject_invalid_joint_parameters() {
        let path = std::path::Path::new(openarm_test_model::SCENE);
        for (name, value) in [
            ("openarm_right_joint4", -0.1),
            ("openarm_right_finger_joint1", 0.1),
        ] {
            let config = Config {
                joints: BTreeMap::from([(
                    name.into(),
                    JointParameters {
                        frictionloss: Some(value),
                        ..JointParameters::default()
                    },
                )]),
                ..Config::default()
            };
            assert!(Physics::load(path, config).is_err());
        }
    }

    fn friction_world(law: Option<Stribeck>, step: f64) -> Physics {
        let mut p = world(Config {
            friction_scale: 0.,
            joints: BTreeMap::from([(
                "openarm_right_joint7".into(),
                JointParameters {
                    frictionloss: Some(0.2),
                    damping: Some(0.05),
                    stribeck: law,
                    ..JointParameters::default()
                },
            )]),
            ..Config::default()
        });
        // Isolate passive dissipation from gravity, contacts and joint stops.
        // This remains the full coupled robot mass matrix, not a signal delay.
        p.model.set_gravity([0.; 3]).unwrap();
        p.model.set_timestep(Duration::from_secs_f64(step)).unwrap();
        p.model.disable_contacts_and_limits();
        p.model.forward(&mut p.data);

        p
    }

    fn test_law() -> Stribeck {
        Stribeck {
            breakaway_nm: 0.6,
            velocity_rad_s: 0.04,
            direction_asymmetry: 0.,
            angle: None,
        }
    }

    #[test]
    fn enhanced_friction_opposes_slip_and_dissipates_energy_in_coupled_physics() {
        for sign in [-1., 1.] {
            let law = Stribeck {
                direction_asymmetry: 0.25,
                angle: Some(openarm_simulator_core_rs::AngleModulation {
                    amplitude: 0.3,
                    harmonic: 2,
                    phase_rad: 0.4,
                }),
                ..test_law()
            };
            let mut p = friction_world(Some(law), STEP);
            let dof = p.index[0].dof[6];
            {
                p.data.view_mut().qvel[dof] = sign * 2.;
                p.model.forward(&mut p.data);
            }
            let initial_energy = p.model.kinetic_energy(&mut p.data);
            let mut friction_work = 0.;
            for _ in 0..1000 {
                let before = p.motors[0][6].q;
                p.step(1).unwrap();
                {
                    let d = p.data.view();
                    let force = d.qfrc_constraint[dof];
                    friction_work += force * (p.motors[0][6].q - before);
                    assert!(force.abs() <= p.model.view().dof_frictionloss[dof] + 1e-10);

                    assert!(p.model.kinetic_energy(&mut p.data) <= initial_energy * 1.0001);
                }
            }
            // Other unresisted joints can retain energy transferred through the
            // coupled mass matrix; this test asserts dissipation, not all-arm rest.
            assert!(friction_work < 0.);
            assert!(p.model.kinetic_energy(&mut p.data) < initial_energy);
        }
    }

    #[test]
    fn breakaway_resists_small_push_and_batched_steps_are_deterministic() {
        let mut basic = friction_world(None, STEP);
        let mut rich = friction_world(Some(test_law()), STEP);
        let mut repeated = friction_world(Some(test_law()), STEP);
        let mut force = [[0.; 7]; 2];
        force[0][6] = 0.3; // Above sliding friction, below enhanced breakaway.
        for p in [&mut basic, &mut rich, &mut repeated] {
            p.push(force).unwrap();
        }
        basic.step(400).unwrap();
        rich.step(400).unwrap();
        for _ in 0..400 {
            repeated.step(1).unwrap();
        }
        assert!(
            basic.motors[0][6].q > 5. * rich.motors[0][6].q,
            "basic={} enhanced={}",
            basic.motors[0][6].q,
            rich.motors[0][6].q
        );
        assert_eq!(rich.motors[0][6].q, repeated.motors[0][6].q);
        assert_eq!(rich.motors[0][6].dq, repeated.motors[0][6].dq);
        assert_eq!(rich.motors[0][6].torque, 0.); // External push is not motor torque.
        force[0][6] = 0.9;
        rich.push(force).unwrap();
        let before = rich.motors[0][6].q;
        rich.step(200).unwrap();
        assert!(rich.motors[0][6].q - before > 0.01);
    }

    #[test]
    fn enhanced_friction_is_stable_under_time_step_variation() {
        let mut endpoints = Vec::new();
        for step in [0.00025, STEP, 0.001] {
            let mut p = friction_world(
                Some(Stribeck {
                    direction_asymmetry: -0.2,
                    angle: Some(openarm_simulator_core_rs::AngleModulation {
                        amplitude: 0.25,
                        harmonic: 2,
                        phase_rad: -0.3,
                    }),
                    ..test_law()
                }),
                step,
            );
            for torque in [0.2, 0.9, 0., -0.9, 0.] {
                let mut force = [[0.; 7]; 2];
                force[0][6] = torque;
                p.push(force).unwrap();
                p.step((0.1 / step).round() as u64).unwrap();
            }
            endpoints.push(p.motors[0][6].q);
            assert!(p.motors[0][6].dq.is_finite());
        }
        for q in &endpoints[1..] {
            // Severe square torque pulses and stick/slip are first-order here;
            // require agreement within 5% over the fourfold step-size range.
            assert!(
                (q - endpoints[0]).abs() < 0.05 * endpoints[0].abs(),
                "step-sensitive endpoints {endpoints:?}"
            );
        }
    }

    #[test]
    fn reset_retains_resistance_but_clears_external_push_atomically() {
        let mut p = friction_world(Some(test_law()), STEP);
        let mut force = [[0.; 7]; 2];
        force[0][6] = 0.75;
        p.push(force).unwrap();
        let previous = p.parameters();
        force[1][0] = f64::NAN;
        assert!(p.push(force).is_err());
        assert_eq!(p.parameters(), previous);
        p.step(100).unwrap();
        p.reset().unwrap();
        assert_eq!(p.applied_torque, [[0.; 7]; 2]);
        assert_eq!(p.parameters().joints, previous.joints);
        {
            assert_eq!(p.data.view().qfrc_applied[p.index[0].dof[6]], 0.);
            assert_eq!(p.model.view().dof_frictionloss[p.index[0].dof[6]], 0.6);
        }
    }

    #[test]
    fn arms_share_contact_world() {
        let right = [-26.625f64, 1.814, -56.701, 121.279, 0., 0., 0., -10.].map(f64::to_radians);
        let left = [-69.152f64, -4.847, -72.3, 115.939, 0., 0., 0., -10.].map(f64::to_radians);
        let p = world(Config {
            poses: Arms {
                right: Some(right),
                left: Some(left),
            },
            ..Config::default()
        });
        let hit = p.data.contacts().iter().any(|c| {
            let a = p.model.name(Object::Geom, c.geom[0] as usize).unwrap_or("");
            let b = p.model.name(Object::Geom, c.geom[1] as usize).unwrap_or("");
            c.dist < -0.001
                && c.efc_address >= 0
                && ((a.contains("openarm_left") && b.contains("openarm_right"))
                    || (a.contains("openarm_right") && b.contains("openarm_left")))
        });
        assert!(hit);
    }
}
