//! Mechanical scene integration and configured actuator bindings.
use crate::{config::Config, friction};
use anyhow::{Context, Result, ensure};
use damiao_simulator_rs::{Drive, ShaftState};
use mujoco_rs::{Data, JOINT_HINGE, JOINT_SLIDE, Model, NBIAS, NIMP, NREF, Object};
use openarm_simulator_core_rs::{
    BodyParameters, JointParameters, PhysicsConfiguration, Plant, Push, Stribeck,
};
use std::{collections::BTreeMap, path::Path, time::Duration};

struct VariableFriction {
    qpos: usize,
    dof: usize,
    sliding_nm: f64,
    law: Stribeck,
}

struct Actuator {
    name: String,
    id: usize,
    ctrl: usize,
    force: usize,
    offset: f64,
}

pub struct Physics {
    model: Model,
    data: Data,
    actuators: Vec<Actuator>,
    initial_positions: Vec<(usize, f64)>,
    pub timestep_ns: u64,
    friction: Vec<VariableFriction>,
    joint_parameters: BTreeMap<String, JointParameters>,
    applied_torque: Push,
}

fn scalar_joint(model: &Model, name: &str) -> Result<(usize, usize, usize)> {
    let id = model.id(Object::Joint, name)?;
    let m = model.view();
    ensure!(
        matches!(m.jnt_type[id], JOINT_HINGE | JOINT_SLIDE),
        "expected a hinge or slide joint: {name}"
    );
    Ok((id, m.jnt_qposadr[id] as usize, m.jnt_dofadr[id] as usize))
}

impl Physics {
    pub fn load(path: &Path, config: &Config) -> Result<Self> {
        ensure!(config.timestep_ns > 0, "timestep_ns must be positive");
        let mut model = Model::from_xml(path)?;
        model.set_timestep(Duration::from_nanos(config.timestep_ns))?;
        let mut actuators = Vec::new();
        for binding in config.motors.values() {
            ensure!(
                binding.encoder_offset_rad.is_finite(),
                "encoder offset must be finite"
            );
            let id = model.id(Object::Actuator, &binding.actuator)?;
            model.use_affine_actuator(id)?;
            let m = model.view();
            actuators.push(Actuator {
                name: binding.actuator.clone(),
                id,
                ctrl: m.actuator_ctrladr[id] as usize,
                force: m.actuator_outadr[id] as usize,
                offset: binding.encoder_offset_rad,
            });
        }
        for (name, body) in &config.bodies {
            let id = model.id(Object::Body, name)?;
            ensure!(id > 0, "cannot change world body inertia");
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
            let m = model.view_mut();
            m.body_mass[id] = body.mass;
            m.body_ipos[3 * id..3 * id + 3].copy_from_slice(&body.com);
            m.body_inertia[3 * id..3 * id + 3].copy_from_slice(&body.inertia);
        }
        ensure!(
            config.friction_scale.is_finite() && config.friction_scale >= 0.,
            "invalid friction scale"
        );
        {
            let m = model.view_mut();
            for (id, kind) in m.jnt_type.iter().enumerate() {
                if *kind == JOINT_HINGE {
                    let dof = m.jnt_dofadr[id] as usize;
                    m.dof_frictionloss[dof] *= config.friction_scale;
                    m.dof_damping[dof] *= config.friction_scale;
                }
            }
        }
        for (name, parameters) in &config.joints {
            let (id, qpos, dof) = scalar_joint(&model, name)?;
            let m = model.view_mut();
            ensure!(
                parameters.stribeck.is_none() || m.jnt_type[id] == JOINT_HINGE,
                "Stribeck friction requires a hinge joint: {name}"
            );
            for (value, field, address) in [
                (
                    parameters.frictionloss,
                    "frictionloss",
                    &mut m.dof_frictionloss[dof],
                ),
                (parameters.damping, "damping", &mut m.dof_damping[dof]),
                (parameters.stiffness, "stiffness", &mut m.jnt_stiffness[id]),
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
        let mut friction = Vec::new();
        let mut joint_parameters = BTreeMap::new();
        for id in 0..model.view().jnt_type.len() {
            if !matches!(model.view().jnt_type[id], JOINT_HINGE | JOINT_SLIDE) {
                continue;
            }
            let Some(name) = model.name(Object::Joint, id).map(str::to_owned) else {
                continue;
            };
            let (_, qpos, dof) = scalar_joint(&model, &name)?;
            let m = model.view_mut();
            let sliding_nm = m.dof_frictionloss[dof];
            let damping = m.dof_damping[dof];
            ensure!(
                sliding_nm.is_finite() && damping.is_finite(),
                "friction scaling overflow for {name}"
            );
            let law = config.joints.get(&name).and_then(|p| p.stribeck.clone());
            if let Some(law) = &law {
                friction::validate(law, sliding_nm)
                    .with_context(|| format!("invalid friction for {name}"))?;
                m.dof_solref[dof * NREF..dof * NREF + NREF].copy_from_slice(&[0.002, 1.]);
                m.dof_solimp[dof * NIMP..dof * NIMP + NIMP]
                    .copy_from_slice(&[0.999, 0.999, 0.001, 0.5, 2.]);
                friction.push(VariableFriction {
                    qpos,
                    dof,
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
                    springref: Some(m.qpos_spring[qpos]),
                    stribeck: law,
                },
            );
        }
        let mut initial_positions = Vec::new();
        for (name, value) in &config.positions {
            ensure!(value.is_finite(), "startup position must be finite: {name}");
            let (_, qpos, _) = scalar_joint(&model, name)?;
            initial_positions.push((qpos, *value));
        }
        let mut data = Data::new(&model)?;
        model.set_constants(&mut data);
        let mut physics = Self {
            model,
            data,
            actuators,
            initial_positions,
            timestep_ns: config.timestep_ns,
            friction,
            joint_parameters,
            applied_torque: Push::new(),
        };
        physics.reset()?;
        Ok(physics)
    }

    pub fn reset(&mut self) -> Result<()> {
        self.model.reset_data(&mut self.data);
        for (qpos, position) in &self.initial_positions {
            self.data.view_mut().qpos[*qpos] = *position;
        }
        self.applied_torque.clear();
        self.configure(&vec![Drive::default(); self.actuators.len()])?;
        self.configure_friction();
        self.model.forward(&mut self.data);
        self.validate_state()
    }

    fn configure(&mut self, drives: &[Drive]) -> Result<()> {
        ensure!(
            drives.len() == self.actuators.len(),
            "actuator drive count mismatch"
        );
        let (m, d) = (self.model.view_mut(), self.data.view_mut());
        for (a, drive) in self.actuators.iter().zip(drives) {
            let control = drive.feedforward - drive.stiffness * a.offset;
            ensure!(
                [control, drive.stiffness, drive.damping]
                    .iter()
                    .all(|v| v.is_finite()),
                "nonfinite actuator drive"
            );
            d.ctrl[a.ctrl] = control;
            m.actuator_biasprm[a.id * NBIAS + 1] = -drive.stiffness;
            m.actuator_biasprm[a.id * NBIAS + 2] = -drive.damping;
        }
        Ok(())
    }

    fn configure_friction(&mut self) {
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

    pub fn step(&mut self, count: u64, drives: &[Drive]) -> Result<()> {
        self.configure(drives)?;
        for _ in 0..count {
            self.configure_friction();
            self.model.step(&mut self.data);
        }
        self.validate_state()?;
        self.model.forward(&mut self.data);
        self.validate_state()
    }

    fn validate_state(&self) -> Result<()> {
        let d = self.data.view();
        ensure!(
            self.data.warnings().iter().all(|w| w.number == 0)
                && [
                    d.qpos,
                    d.qvel,
                    d.actuator_length,
                    d.actuator_velocity,
                    d.actuator_force
                ]
                .into_iter()
                .flatten()
                .all(|v| v.is_finite()),
            "MuJoCo numerical warning; simulation stopped"
        );
        Ok(())
    }

    pub fn observations(&self) -> Vec<ShaftState> {
        let d = self.data.view();
        self.actuators
            .iter()
            .map(|a| ShaftState {
                position: d.actuator_length[a.force] + a.offset,
                velocity: d.actuator_velocity[a.force],
                torque: d.actuator_force[a.force],
            })
            .collect()
    }

    pub fn version() -> String {
        mujoco_rs::version()
    }

    pub fn push(&mut self, forces: Push) -> Result<()> {
        let mut resolved = Vec::new();
        for (name, force) in &forces {
            ensure!(force.is_finite(), "invalid applied torque");
            let (id, _, dof) = scalar_joint(&self.model, name)?;
            ensure!(
                self.model.view().jnt_type[id] == JOINT_HINGE,
                "joint torque requires a hinge: {name}"
            );
            resolved.push((dof, *force));
        }
        let d = self.data.view_mut();
        d.qfrc_applied.fill(0.);
        for (dof, force) in resolved {
            d.qfrc_applied[dof] = force;
        }
        self.applied_torque = forces;
        Ok(())
    }

    pub fn parameters(&self) -> Plant {
        Plant {
            friction_model: "mujoco-dry-viscous-with-optional-stribeck-v1".into(),
            joints: self.joint_parameters.clone(),
            applied_torque_nm: self.applied_torque.clone(),
        }
    }

    pub fn configuration(&self) -> PhysicsConfiguration {
        PhysicsConfiguration {
            friction_model: "mujoco-dry-viscous-with-optional-stribeck-v1".into(),
            mujoco_version: Self::version(),
            timestep_ns: self.timestep_ns,
            integrator: self.model.integrator().into(),
            gravity_m_s2: self.model.gravity(),
            enhanced_friction_solref: [0.002, 1.],
            enhanced_friction_solimp: [0.999, 0.999, 0.001, 0.5, 2.],
            joints: self.joint_parameters.clone(),
            bodies: self.body_parameters(),
            encoder_offsets_rad: self
                .actuators
                .iter()
                .map(|a| (a.name.clone(), a.offset))
                .collect(),
        }
    }

    fn body_parameters(&self) -> BTreeMap<String, BodyParameters> {
        let m = self.model.view();
        (1..m.nbody)
            .filter_map(|id| {
                self.model.name(Object::Body, id).map(|name| {
                    (
                        name.into(),
                        BodyParameters {
                            mass: m.body_mass[id],
                            com: m.body_ipos[3 * id..3 * id + 3].try_into().unwrap(),
                            inertia: m.body_inertia[3 * id..3 * id + 3].try_into().unwrap(),
                        },
                    )
                })
            })
            .collect()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::DEFAULT_TIMESTEP_NS;
    const STEP: f64 = DEFAULT_TIMESTEP_NS as f64 / 1_000_000_000.;
    #[allow(clippy::approx_constant)]
    const RADIUS: f64 = 0.044 / 1.0472;
    fn profile() -> Config {
        serde_json::from_str(include_str!("../config/openarm-v1.json")).unwrap()
    }
    fn positions(right: [f64; 8], left: [f64; 8]) -> BTreeMap<String, f64> {
        let mut positions = BTreeMap::new();
        for (side, pose) in [("right", right), ("left", left)] {
            for (j, q) in pose[..7].iter().enumerate() {
                positions.insert(format!("openarm_{side}_joint{}", j + 1), *q);
            }
            for j in 1..=2 {
                positions.insert(format!("openarm_{side}_finger_joint{j}"), -pose[7] * RADIUS);
            }
        }
        positions
    }
    fn torques(values: [[f64; 7]; 2]) -> Push {
        ["right", "left"]
            .into_iter()
            .zip(values)
            .flat_map(|(side, values)| {
                values
                    .into_iter()
                    .enumerate()
                    .map(move |(j, value)| (format!("openarm_{side}_joint{}", j + 1), value))
            })
            .collect()
    }

    use crate::simulation::Simulation;
    use damiao_can_rs::{MitCommand, MotorStatus};
    #[test]
    fn arbitrary_scene_names_gearing_and_unmapped_actuators() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("scene.xml");
        std::fs::write(&path, r#"<mujoco>
          <option gravity="0 0 0" integrator="implicitfast"/>
          <worldbody>
            <body name="tool"><joint name="hinge" damping="0.2"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="1"/></body>
            <body name="extra" pos="2 0 0"><joint name="passive"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="1"/></body>
            <body pos="4 0 0"><freejoint name="floating"/><geom type="sphere" size="0.1" mass="1"/></body>
          </worldbody>
          <actuator>
            <position name="untouched" joint="passive" kp="7" ctrlrange="-1 1"/>
            <motor name="shaft" joint="hinge" gear="3" forcelimited="true" forcerange="-2 2"/>
          </actuator>
        </mujoco>"#).unwrap();
        let config: Config = serde_json::from_value(serde_json::json!({
            "buses":{"bench":"vcan9"},
            "positions":{"hinge":0.2},
            "bodies":{"tool":{"mass":2.,"com":[0.1,0.,0.],"inertia":[0.001,0.01,0.01]}},
            "motors":{"tool_motor":{"bus":"bench","actuator":"shaft","encoder_offset_rad":0.4,
                "controller":{"id":75,"reply_id":150,"ranges":{"pmax":12.5,"vmax":30.,"tmax":10.}}}}
        }))
        .unwrap();
        let original = Model::from_xml(&path).unwrap();
        let mut sim = Simulation::load(&path, config).unwrap();
        let initial = sim.snapshot();
        assert_eq!(sim.motors.len(), 1);
        assert!((initial["tool_motor"].q - 1.).abs() < 1e-12);
        assert_eq!(sim.physics.configuration().bodies["tool"].mass, 2.);
        assert_eq!(
            &sim.physics.model.view().actuator_biasprm[..NBIAS],
            &original.view().actuator_biasprm[..NBIAS]
        );
        sim.physics.data.view_mut().ctrl[0] = 0.3;
        sim.motors[0].status = MotorStatus::ENABLED;
        sim.motors[0].command = MitCommand {
            q: 1.6,
            kp: 4.,
            kd: 0.5,
            ..MitCommand::default()
        };
        sim.step(200).unwrap();
        assert!(sim.motors[0].q > 1.1);
        assert!(sim.motors[0].torque.abs() <= 2.);
        let (_, qpos, _) = scalar_joint(&sim.physics.model, "hinge").unwrap();
        assert!((sim.motors[0].q - (3. * sim.physics.data.view().qpos[qpos] + 0.4)).abs() < 1e-12);
        let (_, passive, _) = scalar_joint(&sim.physics.model, "passive").unwrap();
        assert!(sim.physics.data.view().qpos[passive] > 0.);
        assert_eq!(sim.physics.data.view().ctrl[0], 0.3);
        sim.reset().unwrap();
        assert_eq!(sim.snapshot(), initial);
    }

    fn world(config: Config) -> Simulation {
        Simulation::load(std::path::Path::new(openarm_test_model::SCENE), config).unwrap()
    }

    #[test]
    fn nonfinite_state_is_rejected_until_reset() {
        let mut p = world(profile());
        p.physics.data.view_mut().qpos[0] = f64::NAN;
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
            positions: positions(pose, pose),
            ..profile()
        });
        p.step(200).unwrap();
        assert!((p.motors[11].q - pose[3]).abs() > 0.001);
        p.reset().unwrap();
        for m in &mut p.motors {
            m.status = MotorStatus::ENABLED;
            m.command = MitCommand {
                kp: 150.,
                kd: 2.,
                q: m.q,
                ..MitCommand::default()
            };
        }
        p.step(1000).unwrap();
        assert!((p.motors[11].q - pose[3]).abs() < 0.08);
        assert!(p.motors[11].torque.abs() > 0.1);
    }

    #[test]
    fn gripper_transmission_loaded_stop_and_no_teleport() {
        let mut p = world(profile());
        p.motors[15].status = MotorStatus::ENABLED;
        p.motors[15].command = MitCommand {
            kp: 10.,
            kd: 0.9,
            q: -0.5,
            ..MitCommand::default()
        };
        p.step(1).unwrap();
        assert!((p.motors[15].q + 0.5).abs() > 0.1);
        p.step(1000).unwrap();
        assert!((p.motors[15].q + 0.5).abs() < 0.01);
        {
            for name in ["openarm_right_finger_joint1", "openarm_right_finger_joint2"] {
                let i = scalar_joint(&p.physics.model, name).unwrap().1;
                assert!((p.physics.data.view().qpos[i] - 0.5 * RADIUS).abs() < 0.001);
            }
        }
        p.motors[15].command = MitCommand {
            kp: 45.,
            kd: 1.2,
            q: 0.1,
            ..MitCommand::default()
        };
        p.step(2000).unwrap();
        let m = p.motors[15];
        assert!(m.dq.abs() < 0.01 && m.torque > 0.3 && m.q > 0. && m.q < 0.2f64.to_radians());
    }

    #[test]
    fn joint_target_is_physics_driven_and_force_is_bounded() {
        let mut p = world(profile());
        p.motors[14].status = MotorStatus::ENABLED;
        p.motors[14].command = MitCommand {
            kp: 10.,
            kd: 0.5,
            q: 0.3,
            ..MitCommand::default()
        };
        p.step(1).unwrap();
        assert!(p.motors[14].q < 0.1);
        p.step(1000).unwrap();
        assert!((p.motors[14].q - 0.3).abs() < 0.05);
        assert!(p.motors[6].q.abs() < 0.05);
        p.motors[14].command.q = 10.;
        p.step(1).unwrap();
        assert!((p.motors[14].torque - 7.).abs() < 1e-6);
        p.motors[14].status = MotorStatus::DISABLED;
        p.step(1).unwrap();
        assert_eq!(p.motors[14].torque, 0.);
    }

    #[test]
    fn encoder_bias_and_reset() {
        let mut config = profile();
        for (name, motor) in &mut config.motors {
            if name.starts_with("right_") {
                motor.encoder_offset_rad = 0.01;
            }
        }
        let mut p = world(config);
        assert_eq!(p.motors[14].q, 0.01);
        p.motors[14].status = MotorStatus::ENABLED;
        p.motors[14].command = MitCommand {
            kp: 30.,
            kd: 0.8,
            q: 0.3,
            ..MitCommand::default()
        };
        p.step(1000).unwrap();
        assert!((p.motors[14].q - 0.3).abs() < 0.02);
        p.motors[14].silent = true;
        p.reset().unwrap();
        assert!(
            !p.motors[14].silent
                && p.motors[14].status == MotorStatus::DISABLED
                && p.motors[14].command.kp == 0.
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
            ..profile()
        });
        p.reset().unwrap();
        {
            let (m, d) = (p.physics.model.view(), p.physics.data.view());
            let dof = scalar_joint(&p.physics.model, "openarm_right_joint4")
                .unwrap()
                .2;
            assert_eq!(m.dof_frictionloss[dof], 0.23);
            assert_eq!(m.dof_damping[dof], 0.61);
            assert!((d.qfrc_passive[dof] - 0.47 * 0.19).abs() < 1e-12);
            assert!(
                (m.dof_frictionloss[scalar_joint(&p.physics.model, "openarm_left_joint4")
                    .unwrap()
                    .2]
                    - 0.05)
                    .abs()
                    < 1e-12
            );
        }
    }

    #[test]
    fn reject_invalid_joint_parameters() {
        let path = std::path::Path::new(openarm_test_model::SCENE);
        for (name, value) in [("openarm_right_joint4", -0.1), ("missing_joint", 0.1)] {
            let config = Config {
                joints: BTreeMap::from([(
                    name.into(),
                    JointParameters {
                        frictionloss: Some(value),
                        ..JointParameters::default()
                    },
                )]),
                ..profile()
            };
            assert!(Simulation::load(path, config).is_err());
        }
    }

    fn friction_world(law: Option<Stribeck>, step: f64) -> Simulation {
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
            ..profile()
        });
        // Isolate passive dissipation from gravity, contacts and joint stops.
        // This remains the full coupled robot mass matrix, not a signal delay.
        p.physics.model.set_gravity([0.; 3]).unwrap();
        p.physics
            .model
            .set_timestep(Duration::from_secs_f64(step))
            .unwrap();
        p.physics.model.disable_contacts_and_limits();
        p.physics.model.forward(&mut p.physics.data);

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
            let dof = scalar_joint(&p.physics.model, "openarm_right_joint7")
                .unwrap()
                .2;
            {
                p.physics.data.view_mut().qvel[dof] = sign * 2.;
                p.physics.model.forward(&mut p.physics.data);
            }
            let initial_energy = p.physics.model.kinetic_energy(&mut p.physics.data);
            let mut friction_work = 0.;
            for _ in 0..1000 {
                let before = p.motors[14].q;
                p.step(1).unwrap();
                {
                    let d = p.physics.data.view();
                    let force = d.qfrc_constraint[dof];
                    friction_work += force * (p.motors[14].q - before);
                    assert!(force.abs() <= p.physics.model.view().dof_frictionloss[dof] + 1e-10);

                    assert!(
                        p.physics.model.kinetic_energy(&mut p.physics.data)
                            <= initial_energy * 1.0001
                    );
                }
            }
            // Other unresisted joints can retain energy transferred through the
            // coupled mass matrix; this test asserts dissipation, not all-arm rest.
            assert!(friction_work < 0.);
            assert!(p.physics.model.kinetic_energy(&mut p.physics.data) < initial_energy);
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
            p.push(torques(force)).unwrap();
        }
        basic.step(400).unwrap();
        rich.step(400).unwrap();
        for _ in 0..400 {
            repeated.step(1).unwrap();
        }
        assert!(
            basic.motors[14].q > 5. * rich.motors[14].q,
            "basic={} enhanced={}",
            basic.motors[14].q,
            rich.motors[14].q
        );
        assert_eq!(rich.motors[14].q, repeated.motors[14].q);
        assert_eq!(rich.motors[14].dq, repeated.motors[14].dq);
        assert_eq!(rich.motors[14].torque, 0.); // External push is not motor torque.
        force[0][6] = 0.9;
        rich.push(torques(force)).unwrap();
        let before = rich.motors[14].q;
        rich.step(200).unwrap();
        assert!(rich.motors[14].q - before > 0.01);
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
                p.push(torques(force)).unwrap();
                p.step((0.1 / step).round() as u64).unwrap();
            }
            endpoints.push(p.motors[14].q);
            assert!(p.motors[14].dq.is_finite());
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
        p.push(torques(force)).unwrap();
        let previous = p.physics.parameters();
        force[1][0] = f64::NAN;
        assert!(p.push(torques(force)).is_err());
        assert_eq!(p.physics.parameters(), previous);
        p.step(100).unwrap();
        p.reset().unwrap();
        assert!(p.physics.applied_torque.is_empty());
        assert_eq!(p.physics.parameters().joints, previous.joints);
        {
            assert_eq!(
                p.physics.data.view().qfrc_applied[scalar_joint(
                    &p.physics.model,
                    "openarm_right_joint7"
                )
                .unwrap()
                .2],
                0.
            );
            assert_eq!(
                p.physics.model.view().dof_frictionloss[scalar_joint(
                    &p.physics.model,
                    "openarm_right_joint7"
                )
                .unwrap()
                .2],
                0.6
            );
        }
    }

    #[test]
    fn arms_share_contact_world() {
        let right = [-26.625f64, 1.814, -56.701, 121.279, 0., 0., 0., -10.].map(f64::to_radians);
        let left = [-69.152f64, -4.847, -72.3, 115.939, 0., 0., 0., -10.].map(f64::to_radians);
        let p = world(Config {
            positions: positions(right, left),
            ..profile()
        });
        let hit = p.physics.data.contacts().iter().any(|c| {
            let a = p
                .physics
                .model
                .name(Object::Geom, c.geom[0] as usize)
                .unwrap_or("");
            let b = p
                .physics
                .model
                .name(Object::Geom, c.geom[1] as usize)
                .unwrap_or("");
            c.dist < -0.001
                && c.efc_address >= 0
                && ((a.contains("openarm_left") && b.contains("openarm_right"))
                    || (a.contains("openarm_right") && b.contains("openarm_left")))
        });
        assert!(hit);
    }
}
