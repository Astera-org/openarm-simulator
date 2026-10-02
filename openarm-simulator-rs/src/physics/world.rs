//! Mechanical scene integration and configured actuator bindings.
use super::{Loads, VariableFriction, friction, spring_geometry};
use crate::config::Config;
use anyhow::{Context, Result, ensure};
use damiao_simulator::{Drive, ShaftState};
use mujoco::{
    ActuatorIndex, BodyIndex, Data, GeomIndex, JointIndex, JointKind, Model, NBIAS, NIMP, NREF,
    SiteIndex,
};
use openarm_simulator_core::{
    AppliedForce, Attachment, BodyPoint, BodyState, SceneNames, Spring, SpringState,
    mint::Quaternion,
    uom::si::{
        acceleration::meter_per_second_squared,
        angle::radian,
        angular_velocity::radian_per_second,
        f64::{
            Acceleration, Angle, AngularVelocity, Force, Length, Mass, MomentOfInertia, Torque,
            Velocity,
        },
        force::newton,
        length::meter,
        mass::kilogram,
        moment_of_inertia::kilogram_square_meter,
        torque::newton_meter,
        velocity::meter_per_second,
    },
};
use openarm_simulator_core::{
    BodyParameters, HingeJointParameters, JointParameters, PhysicsConfiguration, Plant, Push,
    SlideJointParameters,
};
use std::{collections::BTreeMap, time::Duration};

struct Actuator {
    id: ActuatorIndex,
    ctrl: usize,
    force: usize,
    offset: f64,
}

pub struct Physics {
    model: Model,
    data: Data,
    loads: Loads,
    actuators: Vec<Actuator>,
    pub timestep_ns: u64,
    friction: Vec<VariableFriction>,
    joint_parameters: BTreeMap<JointIndex, JointParameters>,
    applied_torque: Push,
}

fn scalar_joint(model: &Model, name: &str) -> Result<(usize, usize, usize)> {
    let id = model.id::<JointIndex>(name)?.0;
    let m = model.view();
    ensure!(
        matches!(
            model.joint_kind(JointIndex(id)),
            JointKind::Hinge | JointKind::Slide
        ),
        "expected a hinge or slide joint: {name}"
    );
    Ok((id, m.jnt_qposadr[id] as usize, m.jnt_dofadr[id] as usize))
}

impl Physics {
    pub fn new(mut model: Model, config: &Config) -> Result<Self> {
        ensure!(config.timestep_ns > 0, "timestep_ns must be positive");
        model.disable_sleep();
        model.set_timestep(Duration::from_nanos(config.timestep_ns))?;
        let mut actuators = Vec::new();
        for binding in config.motors.values() {
            ensure!(
                binding.encoder_offset.value.is_finite(),
                "encoder offset must be finite"
            );
            let id = model.id::<ActuatorIndex>(&binding.actuator)?;
            model.use_affine_actuator(id)?;
            let m = model.view();
            actuators.push(Actuator {
                id,
                ctrl: m.actuator_ctrladr[id.0] as usize,
                force: m.actuator_outadr[id.0] as usize,
                offset: binding.encoder_offset.get::<radian>(),
            });
        }
        for (name, body) in &config.bodies {
            let mass = body.mass.get::<kilogram>();
            let com = <[Length; 3]>::from(body.com).map(|v| v.get::<meter>());
            let inertia = <[MomentOfInertia; 3]>::from(body.inertia)
                .map(|v| v.get::<kilogram_square_meter>());
            let id = model.id::<BodyIndex>(name)?.0;
            ensure!(id > 0, "cannot change world body inertia");
            ensure!(
                mass.is_finite()
                    && mass > 0.
                    && com.iter().all(|x| x.is_finite())
                    && inertia.iter().all(|x| x.is_finite() && *x > 0.),
                "invalid inertial parameters for {name}"
            );
            let sum: f64 = inertia.iter().sum();
            ensure!(
                inertia.iter().all(|x| 2. * x <= sum + 1e-12),
                "inertia triangle inequality for {name}"
            );
            let m = model.view_mut();
            m.body_mass[id] = mass;
            m.body_ipos[3 * id..3 * id + 3].copy_from_slice(&com);
            m.body_inertia[3 * id..3 * id + 3].copy_from_slice(&inertia);
        }
        ensure!(
            config.friction_scale.is_finite() && config.friction_scale >= 0.,
            "invalid friction scale"
        );
        for id in 0..model.count::<JointIndex>() {
            if model.joint_kind(JointIndex(id)) == JointKind::Hinge {
                let m = model.view_mut();
                let dof = m.jnt_dofadr[id] as usize;
                m.dof_frictionloss[dof] *= config.friction_scale;
                m.dof_damping[dof] *= config.friction_scale;
            }
        }
        for (name, parameters) in &config.joints {
            let (id, qpos, dof) = scalar_joint(&model, name)?;
            let kind = model.joint_kind(JointIndex(id));
            let m = model.view_mut();
            let (frictionloss, damping, stiffness, springref) = match parameters {
                JointParameters::Hinge(p) => {
                    ensure!(
                        kind == JointKind::Hinge,
                        "hinge parameters require a hinge joint: {name}"
                    );
                    (
                        p.frictionloss.map(|v| v.get::<newton_meter>()),
                        p.damping.map(|v| v.value),
                        p.stiffness.map(|v| v.value),
                        p.springref.map(|v| v.get::<radian>()),
                    )
                }
                JointParameters::Slide(p) => {
                    ensure!(
                        kind == JointKind::Slide,
                        "slide parameters require a slide joint: {name}"
                    );
                    (
                        p.frictionloss.map(|v| v.get::<newton>()),
                        p.damping.map(|v| v.value),
                        p.stiffness.map(|v| v.value),
                        p.springref.map(|v| v.get::<meter>()),
                    )
                }
            };
            for (value, field, address) in [
                (frictionloss, "frictionloss", &mut m.dof_frictionloss[dof]),
                (damping, "damping", &mut m.dof_damping[dof]),
                (stiffness, "stiffness", &mut m.jnt_stiffness[id]),
            ] {
                if let Some(value) = value {
                    ensure!(
                        value.is_finite() && value >= 0.,
                        "invalid {field} for {name}"
                    );
                    *address = value;
                }
            }
            if let Some(value) = springref {
                ensure!(value.is_finite(), "invalid springref for {name}");
                m.qpos_spring[qpos] = value;
            }
        }
        let mut friction = Vec::new();
        let mut joint_parameters = BTreeMap::new();
        for id in 0..model.count::<JointIndex>() {
            let kind = model.joint_kind(JointIndex(id));
            if !matches!(kind, JointKind::Hinge | JointKind::Slide) {
                continue;
            }
            let parameters = model
                .name(JointIndex(id))
                .and_then(|name| config.joints.get(name));
            let (qpos, dof) = (
                model.view().jnt_qposadr[id] as usize,
                model.view().jnt_dofadr[id] as usize,
            );
            let m = model.view_mut();
            let frictionloss = m.dof_frictionloss[dof];
            let damping = m.dof_damping[dof];
            ensure!(
                frictionloss.is_finite() && damping.is_finite(),
                "friction scaling overflow for joint {id}"
            );
            let law = parameters.and_then(|p| match p {
                JointParameters::Hinge(p) => p.stribeck.clone(),
                JointParameters::Slide(_) => None,
            });
            if let Some(law) = &law {
                friction::validate(law, frictionloss)
                    .with_context(|| format!("invalid friction for joint {id}"))?;
                m.dof_solref[dof * NREF..dof * NREF + NREF].copy_from_slice(&[0.002, 1.]);
                m.dof_solimp[dof * NIMP..dof * NIMP + NIMP]
                    .copy_from_slice(&[0.999, 0.999, 0.001, 0.5, 2.]);
                friction.push(VariableFriction {
                    qpos,
                    dof,
                    sliding_nm: frictionloss,
                    law: law.clone(),
                });
            }
            let parameters = if kind == JointKind::Hinge {
                JointParameters::Hinge(HingeJointParameters {
                    frictionloss: Some(Torque::new::<newton_meter>(frictionloss)),
                    damping: Some(
                        Torque::new::<newton_meter>(damping)
                            / AngularVelocity::new::<radian_per_second>(1.),
                    ),
                    stiffness: Some(
                        Torque::new::<newton_meter>(m.jnt_stiffness[id]) / Angle::new::<radian>(1.),
                    ),
                    springref: Some(Angle::new::<radian>(m.qpos_spring[qpos])),
                    stribeck: law,
                })
            } else {
                JointParameters::Slide(SlideJointParameters {
                    frictionloss: Some(Force::new::<newton>(frictionloss)),
                    damping: Some(
                        Force::new::<newton>(damping) / Velocity::new::<meter_per_second>(1.),
                    ),
                    stiffness: Some(
                        Force::new::<newton>(m.jnt_stiffness[id]) / Length::new::<meter>(1.),
                    ),
                    springref: Some(Length::new::<meter>(m.qpos_spring[qpos])),
                })
            };
            joint_parameters.insert(JointIndex(id), parameters);
        }
        let mut data = Data::new(&model)?;
        model.set_constants(&mut data);
        let mut physics = Self {
            loads: Loads::default(),
            model,
            data,
            actuators,
            timestep_ns: config.timestep_ns,
            friction,
            joint_parameters,
            applied_torque: Push::new(),
        };
        physics.reset()?;
        Ok(physics)
    }

    pub fn reset(&mut self) -> Result<()> {
        self.loads.reset();
        self.model.reset_data(&mut self.data);
        self.applied_torque.clear();
        self.configure(&vec![Drive::default(); self.actuators.len()])?;
        self.configure_friction();
        self.forward();
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
            m.actuator_biasprm[a.id.0 * NBIAS + 1] = -drive.stiffness;
            m.actuator_biasprm[a.id.0 * NBIAS + 2] = -drive.damping;
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
            self.model
                .step_with_forces(&mut self.data, |context| self.loads.apply(context));
        }
        self.validate_state()?;
        self.forward();
        self.validate_state()
    }

    fn forward(&mut self) {
        self.model
            .forward_with_forces(&mut self.data, |context| self.loads.apply(context));
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
        mujoco::version()
    }

    pub fn push(&mut self, forces: Push) -> Result<()> {
        let mut resolved = Vec::new();
        let model = self.model.view();
        for (&index, force) in &forces {
            ensure!(force.value.is_finite(), "invalid applied torque");
            ensure!(
                index.0 < self.model.count::<JointIndex>()
                    && self.model.joint_kind(index) == JointKind::Hinge,
                "joint torque requires a valid hinge-joint index: {index}"
            );
            resolved.push((
                model.jnt_dofadr[index.0] as usize,
                force.get::<newton_meter>(),
            ));
        }
        self.applied_torque = forces;
        self.loads.joint_torques = resolved;
        self.forward();
        Ok(())
    }

    pub fn parameters(&self) -> Plant {
        Plant {
            friction_model: "mujoco-dry-viscous-with-optional-stribeck-v1".into(),
            joints: self.joint_parameters.clone(),
            applied_torque: self.applied_torque.clone(),
        }
    }

    pub fn configuration(&self) -> PhysicsConfiguration {
        PhysicsConfiguration {
            friction_model: "mujoco-dry-viscous-with-optional-stribeck-v1".into(),
            mujoco_version: Self::version(),
            timestep_ns: self.timestep_ns,
            integrator: self.model.integrator(),
            gravity: self
                .model
                .gravity()
                .map(Acceleration::new::<meter_per_second_squared>)
                .into(),
            enhanced_friction_solref: [0.002, 1.],
            enhanced_friction_solimp: [0.999, 0.999, 0.001, 0.5, 2.],
            joints: self.joint_parameters.clone(),
            bodies: self.body_parameters(),
            encoder_offsets: self
                .actuators
                .iter()
                .map(|a| (a.id, Angle::new::<radian>(a.offset)))
                .collect(),
        }
    }

    fn body_parameters(&self) -> Vec<BodyParameters> {
        let m = self.model.view();
        (0..m.nbody)
            .map(|id| BodyParameters {
                mass: Mass::new::<kilogram>(m.body_mass[id]),
                com: <[f64; 3]>::try_from(&m.body_ipos[3 * id..3 * id + 3])
                    .unwrap()
                    .map(Length::new::<meter>)
                    .into(),
                inertia: <[f64; 3]>::try_from(&m.body_inertia[3 * id..3 * id + 3])
                    .unwrap()
                    .map(MomentOfInertia::new::<kilogram_square_meter>)
                    .into(),
            })
            .collect()
    }

    pub fn names(&self) -> SceneNames {
        SceneNames {
            bodies: self
                .model
                .names::<BodyIndex>()
                .map(|(name, index)| (name.to_owned(), index))
                .collect(),
            joints: self
                .model
                .names::<JointIndex>()
                .map(|(name, index)| (name.to_owned(), index))
                .collect(),
            actuators: self
                .model
                .names::<ActuatorIndex>()
                .map(|(name, index)| (name.to_owned(), index))
                .collect(),
            geoms: self
                .model
                .names::<GeomIndex>()
                .map(|(name, index)| (name.to_owned(), index))
                .collect(),
            sites: self
                .model
                .names::<SiteIndex>()
                .map(|(name, index)| (name.to_owned(), index))
                .collect(),
        }
    }

    pub fn springs(&self) -> &BTreeMap<String, Spring<BodyPoint>> {
        &self.loads.springs
    }
    pub fn forces(&self) -> &BTreeMap<String, AppliedForce<BodyPoint>> {
        &self.loads.forces
    }

    pub fn put_spring(&mut self, name: String, spring: Spring) -> Result<bool> {
        name_valid(&name)?;
        let [a, b] = spring.endpoints;
        let spring = Spring {
            endpoints: [self.resolve_point(a)?, self.resolve_point(b)?],
            rest_length: spring.rest_length,
            stiffness: spring.stiffness,
            damping: spring.damping,
        };
        ensure!(
            [
                spring.rest_length.get::<meter>(),
                spring.stiffness.value,
                spring.damping.value
            ]
            .iter()
            .all(|v| v.is_finite() && *v >= 0.),
            "spring length, stiffness and damping must be finite and nonnegative"
        );
        let created = self.loads.springs.insert(name, spring).is_none();
        self.forward();
        Ok(created)
    }

    pub fn delete_spring(&mut self, name: &str) {
        self.loads.springs.remove(name);
        self.forward();
    }

    pub fn put_force(&mut self, name: String, force: AppliedForce) -> Result<bool> {
        name_valid(&name)?;
        let force = AppliedForce {
            point: self.resolve_point(force.point)?,
            force: force.force,
            torque: force.torque,
        };
        ensure!(
            <[Force; 3]>::from(force.force)
                .iter()
                .all(|v| v.value.is_finite())
                && <[Torque; 3]>::from(force.torque)
                    .iter()
                    .all(|v| v.value.is_finite()),
            "force and torque must be finite"
        );
        let created = self.loads.forces.insert(name, force).is_none();
        self.forward();
        Ok(created)
    }

    pub fn delete_force(&mut self, name: &str) {
        self.loads.forces.remove(name);
        self.forward();
    }

    fn resolve_point(&self, attachment: Attachment) -> Result<BodyPoint> {
        let point = match attachment {
            Attachment::Body(point) => point,
            Attachment::Site(index) => {
                ensure!(
                    index.0 < self.model.count::<SiteIndex>(),
                    "site index out of range: {index}"
                );
                let (body, position) = self.model.site_point(index);
                BodyPoint {
                    body,
                    position: position.map(Length::new::<meter>).into(),
                }
            }
        };
        ensure!(
            point.body.0 < self.model.count::<BodyIndex>(),
            "body index out of range: {}",
            point.body
        );
        ensure!(
            <[Length; 3]>::from(point.position)
                .iter()
                .all(|v| v.value.is_finite()),
            "attachment position must be finite"
        );
        Ok(point)
    }

    pub fn body_states(&self) -> Vec<BodyState> {
        (0..self.model.count::<BodyIndex>())
            .map(|index| {
                let body = self.model.body(&self.data, BodyIndex(index));
                BodyState {
                    position: body.position().map(Length::new::<meter>).into(),
                    orientation: quaternion(*body.orientation()),
                    com: body.com().map(Length::new::<meter>).into(),
                }
            })
            .collect()
    }

    pub fn spring_states(&self) -> BTreeMap<String, SpringState> {
        self.loads
            .springs
            .iter()
            .map(|(name, spring)| {
                let bodies = spring
                    .endpoints
                    .map(|point| self.model.body(&self.data, point.body));
                let (length, velocity, _) = spring_geometry(&bodies, &spring.endpoints);
                (
                    name.clone(),
                    SpringState {
                        length: Length::new::<meter>(length),
                        velocity: Velocity::new::<meter_per_second>(velocity),
                    },
                )
            })
            .collect()
    }
}

fn name_valid(name: &str) -> Result<()> {
    ensure!(
        !name.is_empty() && !name.contains('\0'),
        "name must be nonempty and contain no NUL"
    );
    Ok(())
}
fn quaternion(q: [f64; 4]) -> Quaternion<f64> {
    [q[1], q[2], q[3], q[0]].into()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::DEFAULT_TIMESTEP_NS;
    use openarm_simulator_core::Stribeck;
    const STEP: f64 = DEFAULT_TIMESTEP_NS as f64 / 1_000_000_000.;
    #[allow(clippy::approx_constant)]
    const RADIUS: f64 = 0.044 / 1.0472;
    fn profile() -> Config {
        serde_json::from_str(include_str!("../../config/openarm-v1.json")).unwrap()
    }
    fn set_pose(sim: &mut Simulation, right: [f64; 8], left: [f64; 8]) {
        for (side, pose) in [("right", right), ("left", left)] {
            for (j, q) in pose[..7].iter().enumerate() {
                let name = format!("openarm_{side}_joint{}", j + 1);
                let (_, qpos, _) = scalar_joint(&sim.physics.model, &name).unwrap();
                sim.physics.data.view_mut().qpos[qpos] = *q;
            }
            for j in 1..=2 {
                let name = format!("openarm_{side}_finger_joint{j}");
                let (_, qpos, _) = scalar_joint(&sim.physics.model, &name).unwrap();
                sim.physics.data.view_mut().qpos[qpos] = -pose[7] * RADIUS;
            }
        }
        sim.step(0).unwrap();
    }
    fn torques(model: &Model, values: [[f64; 7]; 2]) -> Push {
        ["right", "left"]
            .into_iter()
            .zip(values)
            .flat_map(|(side, values)| {
                values.into_iter().enumerate().map(move |(j, value)| {
                    (
                        model
                            .id::<JointIndex>(&format!("openarm_{side}_joint{}", j + 1))
                            .unwrap(),
                        Torque::new::<newton_meter>(value),
                    )
                })
            })
            .collect()
    }

    use crate::simulation::Simulation;
    use damiao_can::{MitCommand, MotorStatus};
    #[test]
    fn arbitrary_scene_names_gearing_and_unmapped_actuators() {
        let xml = br#"<mujoco>
          <compiler angle="radian"/>
          <option gravity="0 0 0" integrator="implicitfast"/>
          <worldbody>
            <body name="tool"><joint name="hinge" ref="0.2" damping="0.2"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="1"/></body>
            <body name="extra" pos="2 0 0"><joint name="passive"/><geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="1"/></body>
            <body pos="4 0 0"><freejoint name="floating"/><geom type="sphere" size="0.1" mass="1"/></body>
          </worldbody>
          <actuator>
            <position name="untouched" joint="passive" kp="7" ctrlrange="-1 1"/>
            <motor name="shaft" joint="hinge" gear="3" forcelimited="true" forcerange="-2 2"/>
          </actuator>
        </mujoco>"#;
        let config: Config = serde_json::from_value(serde_json::json!({
            "buses":{"bench":"vcan9"},
            "bodies":{"tool":{"mass_kg":2.,"com_m":[0.1,0.,0.],"inertia_kg_m2":[0.001,0.01,0.01]}},
            "motors":{"tool_motor":{"bus":"bench","actuator":"shaft","encoder_offset_rad":0.4,
                "controller":{"id":75,"reply_id":150,"ranges":{"pmax":12.5,"vmax":30.,"tmax":10.}}}}
        }))
        .unwrap();
        let original = Model::from_xml_bytes(xml).unwrap();
        let mut sim = Simulation::new(Model::from_xml_bytes(xml).unwrap(), config).unwrap();
        let initial = sim.snapshot();
        assert_eq!(sim.motors.len(), 1);
        assert!((initial["tool_motor"].q.get::<radian>() - 1.).abs() < 1e-12);
        assert_eq!(
            sim.physics.configuration().bodies
                [sim.physics.model.id::<BodyIndex>("tool").unwrap().0]
                .mass
                .get::<kilogram>(),
            2.
        );
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
        let mut p = world(profile());
        set_pose(&mut p, pose, pose);
        p.step(200).unwrap();
        assert!((p.motors[11].q - pose[3]).abs() > 0.001);
        p.reset().unwrap();
        set_pose(&mut p, pose, pose);
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
                motor.encoder_offset = Angle::new::<radian>(0.01);
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
        let parameters = JointParameters::Hinge(HingeJointParameters {
            frictionloss: Some(Torque::new::<newton_meter>(0.23)),
            damping: Some(
                Torque::new::<newton_meter>(0.61) / AngularVelocity::new::<radian_per_second>(1.),
            ),
            stiffness: Some(Torque::new::<newton_meter>(0.47) / Angle::new::<radian>(1.)),
            springref: Some(Angle::new::<radian>(0.19)),
            ..HingeJointParameters::default()
        });
        let mut p = world(Config {
            friction_scale: 0.5,
            joints: BTreeMap::from([
                ("openarm_right_joint4".into(), parameters),
                (
                    "openarm_right_finger_joint1".into(),
                    JointParameters::Slide(SlideJointParameters {
                        frictionloss: Some(Force::new::<newton>(0.06)),
                        damping: Some(
                            Force::new::<newton>(7.) / Velocity::new::<meter_per_second>(1.),
                        ),
                        stiffness: Some(Force::new::<newton>(2.) / Length::new::<meter>(1.)),
                        springref: Some(Length::new::<
                            openarm_simulator_core::uom::si::length::centimeter,
                        >(1.)),
                    }),
                ),
            ]),
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
            let (id, qpos, dof) =
                scalar_joint(&p.physics.model, "openarm_right_finger_joint1").unwrap();
            assert_eq!(m.dof_frictionloss[dof], 0.06);
            assert_eq!(m.dof_damping[dof], 7.);
            assert_eq!(m.jnt_stiffness[id], 2.);
            assert_eq!(m.qpos_spring[qpos], 0.01);
            let wire = serde_json::to_value(p.physics.parameters()).unwrap();
            assert_eq!(wire["joints"][id.to_string()]["kind"], "slide");
            assert_eq!(wire["joints"][id.to_string()]["springref_m"], 0.01);
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
                    JointParameters::Hinge(HingeJointParameters {
                        frictionloss: Some(Torque::new::<newton_meter>(value)),
                        ..HingeJointParameters::default()
                    }),
                )]),
                ..profile()
            };
            assert!(Simulation::load(path, config).is_err());
        }
        for (name, parameters) in [
            (
                "openarm_right_joint4",
                serde_json::json!({"kind":"slide","frictionloss_n":1.}),
            ),
            (
                "openarm_right_finger_joint1",
                serde_json::json!({"kind":"hinge","frictionloss_nm":1.}),
            ),
        ] {
            let config = Config {
                joints: BTreeMap::from([(
                    name.into(),
                    serde_json::from_value(parameters).unwrap(),
                )]),
                ..profile()
            };
            assert!(Simulation::load(path, config).is_err());
        }
        assert!(
            serde_json::from_value::<JointParameters>(serde_json::json!({
                "kind":"hinge", "frictionloss_n":1.
            }))
            .is_err()
        );
    }

    fn friction_world(law: Option<Stribeck>, step: f64) -> Simulation {
        let mut p = world(Config {
            friction_scale: 0.,
            joints: BTreeMap::from([(
                "openarm_right_joint7".into(),
                JointParameters::Hinge(HingeJointParameters {
                    frictionloss: Some(Torque::new::<newton_meter>(0.2)),
                    damping: Some(
                        Torque::new::<newton_meter>(0.05)
                            / AngularVelocity::new::<radian_per_second>(1.),
                    ),
                    stribeck: law,
                    ..HingeJointParameters::default()
                }),
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
            breakaway: Torque::new::<newton_meter>(0.6),
            velocity: AngularVelocity::new::<radian_per_second>(0.04),
            direction_asymmetry: 0.,
            angle: None,
        }
    }

    #[test]
    fn enhanced_friction_opposes_slip_and_dissipates_energy_in_coupled_physics() {
        for sign in [-1., 1.] {
            let law = Stribeck {
                direction_asymmetry: 0.25,
                angle: Some(openarm_simulator_core::AngleModulation {
                    amplitude: 0.3,
                    harmonic: 2,
                    phase: Angle::new::<radian>(0.4),
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
            p.push(torques(&p.physics.model, force)).unwrap();
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
        rich.push(torques(&rich.physics.model, force)).unwrap();
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
                    angle: Some(openarm_simulator_core::AngleModulation {
                        amplitude: 0.25,
                        harmonic: 2,
                        phase: Angle::new::<radian>(-0.3),
                    }),
                    ..test_law()
                }),
                step,
            );
            for torque in [0.2, 0.9, 0., -0.9, 0.] {
                let mut force = [[0.; 7]; 2];
                force[0][6] = torque;
                p.push(torques(&p.physics.model, force)).unwrap();
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
        p.push(torques(&p.physics.model, force)).unwrap();
        let previous = p.physics.parameters();
        force[1][0] = f64::NAN;
        assert!(p.push(torques(&p.physics.model, force)).is_err());
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
        let mut p = world(profile());
        set_pose(&mut p, right, left);
        let hit = p.physics.data.contacts().iter().any(|c| {
            let a = p
                .physics
                .model
                .name(GeomIndex(c.geom[0] as usize))
                .unwrap_or("");
            let b = p
                .physics
                .model
                .name(GeomIndex(c.geom[1] as usize))
                .unwrap_or("");
            c.dist < -0.001
                && c.efc_address >= 0
                && ((a.contains("openarm_left") && b.contains("openarm_right"))
                    || (a.contains("openarm_right") && b.contains("openarm_left")))
        });
        assert!(hit);
    }

    fn scene(integrator: &str, native_spring: bool) -> Physics {
        let (left_site, right_site, tendon) = if native_spring {
            (
                "<site name='left_site' pos='0 .2 0'/>",
                "<site name='right_site' pos='0 -.1 0'/>",
                "<tendon><spatial name='native' stiffness='40' damping='3' springlength='1'><site site='left_site'/><site site='right_site'/></spatial></tendon>",
            )
        } else {
            ("", "", "")
        };
        let model = Model::from_xml_bytes(format!(r#"<mujoco>
            <option gravity="0 0 0" integrator="{integrator}"><flag contact="disable"/></option>
            <worldbody>
              <body name="left"><freejoint/><inertial pos=".1 0 0" mass="1" diaginertia="1 1 1"/>
                {left_site}
              </body>
              <body name="right" pos="2 0 0"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                {right_site}
              </body>
            </worldbody>{tendon}</mujoco>"#).as_bytes()).unwrap();
        Physics::new(model, &Config::default()).unwrap()
    }
    fn point(body: BodyIndex, position: [f64; 3]) -> BodyPoint {
        BodyPoint {
            body,
            position: position.map(Length::new::<meter>).into(),
        }
    }
    fn spring(a: BodyPoint, b: BodyPoint) -> Spring {
        Spring {
            endpoints: [Attachment::Body(a), Attachment::Body(b)],
            rest_length: Length::new::<meter>(1.),
            stiffness: Force::new::<newton>(40.) / Length::new::<meter>(1.),
            damping: Force::new::<newton>(3.) / Velocity::new::<meter_per_second>(1.),
        }
    }
    fn force(point: BodyPoint) -> AppliedForce {
        AppliedForce {
            point: Attachment::Body(point),
            force: [1., 0., 0.].map(Force::new::<newton>).into(),
            torque: [0., 0., 2.].map(Torque::new::<newton_meter>).into(),
        }
    }
    fn close(a: &[f64], b: &[f64]) {
        for (a, b) in a.iter().zip(b) {
            assert!((a - b).abs() < 1e-11, "{a} != {b}");
        }
    }

    #[test]
    fn callback_spring_matches_native_spring_including_rk4_stages() {
        for integrator in ["Euler", "RK4"] {
            let mut reference = scene(integrator, true);
            let mut custom = scene(integrator, false);
            let left = point(custom.names().bodies["left"], [0., 0.2, 0.]);
            let right = point(custom.names().bodies["right"], [0., -0.1, 0.]);
            let mut indexed = spring(left, right);
            indexed.endpoints = ["left_site", "right_site"]
                .map(|name| Attachment::Site(reference.names().sites[name]));
            reference.put_spring("lookup".into(), indexed).unwrap();
            assert_eq!(reference.springs()["lookup"].endpoints, [left, right]);
            reference.delete_spring("lookup");
            custom
                .put_spring("spring".into(), spring(left, right))
                .unwrap();
            for physics in [&mut reference, &mut custom] {
                physics.data.view_mut().qvel[0] = -0.4;
                physics.data.view_mut().qvel[5] = 0.7;
                physics.data.view_mut().qvel[6] = 0.2;
                physics.data.view_mut().qvel[11] = -0.3;
                physics.forward();
            }
            close(
                custom.data.view().qfrc_applied,
                reference.data.view().qfrc_passive,
            );
            for _ in 0..20 {
                reference.step(10, &[]).unwrap();
                custom.step(10, &[]).unwrap();
                close(custom.data.view().qpos, reference.data.view().qpos);
                close(custom.data.view().qvel, reference.data.view().qvel);
            }
        }
    }

    #[test]
    fn forces_and_springs_update_body_points_without_sites() {
        let mut p = scene("implicitfast", false);
        let left = point(p.names().bodies["left"], [0., 0.2, 0.]);
        let right = point(p.names().bodies["right"], [0., -0.1, 0.]);
        let anchor = point(BodyIndex(0), [2., -0.1, 0.]);
        let stretch = point(BodyIndex(0), [4., -0.1, 0.]);
        let compress = point(BodyIndex(0), [2.5, -0.1, 0.]);
        let missing = point(BodyIndex(p.model.count::<BodyIndex>()), [0.; 3]);
        let initial_bodies = p.body_states();
        let names = p.names();
        assert_eq!(p.model.count::<SiteIndex>(), 0);
        p.put_force("load".into(), force(left)).unwrap();
        // F=(1,0,0) at y=.2 contributes -.2 Nm, plus the explicit +2 Nm torque.
        close(
            p.data.view().qfrc_applied,
            &[1., 0., 0., 0., 0., 1.8, 0., 0., 0., 0., 0., 0.],
        );
        let moved = point(left.body, [0., 0.3, 0.]);
        assert!(!p.put_force("load".into(), force(moved)).unwrap());
        close(
            p.data.view().qfrc_applied,
            &[1., 0., 0., 0., 0., 1.7, 0., 0., 0., 0., 0., 0.],
        );
        let mut invalid = force(left);
        invalid.force.x = Force::new::<newton>(f64::NAN);
        assert!(p.put_force("load".into(), invalid).is_err());
        assert!(p.put_force("load".into(), force(missing)).is_err());
        assert_eq!(p.forces()["load"].point, moved);
        assert!(!p.put_force("load".into(), force(right)).unwrap());
        close(
            p.data.view().qfrc_applied,
            &[0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 2.1],
        );
        p.delete_force("load");
        assert!(p.data.view().qfrc_applied.iter().all(|v| *v == 0.));
        p.put_spring("spring".into(), spring(right, anchor))
            .unwrap();
        assert_eq!(p.spring_states()["spring"].length.get::<meter>(), 0.);
        assert!(p.data.view().qfrc_applied.iter().all(|v| *v == 0.));
        p.put_spring("spring".into(), spring(right, stretch))
            .unwrap();
        assert_eq!(p.data.view().qfrc_applied[6], 40.);
        p.put_spring("spring".into(), spring(right, compress))
            .unwrap();
        assert_eq!(p.data.view().qfrc_applied[6], -20.);
        assert!(
            p.put_spring("spring".into(), spring(right, missing))
                .is_err()
        );
        let mut invalid = spring(left, anchor);
        invalid.damping.value = -1.;
        assert!(p.put_spring("spring".into(), invalid).is_err());
        let invalid = point(left.body, [f64::NAN, 0., 0.]);
        assert!(p.put_force("load".into(), force(invalid)).is_err());
        for endpoints in [[invalid, right], [left, invalid]] {
            assert!(
                p.put_spring("spring".into(), spring(endpoints[0], endpoints[1]))
                    .is_err()
            );
        }
        assert!(p.forces().is_empty());
        assert_eq!(p.springs()["spring"].endpoints, [right, compress]);
        p.delete_spring("spring");
        assert_eq!(p.body_states(), initial_bodies);
        p.put_force("load".into(), force(left)).unwrap();
        p.put_spring("spring".into(), spring(left, anchor)).unwrap();
        p.step(100, &[]).unwrap();
        p.reset().unwrap();
        assert_eq!(p.names(), names);
        assert_eq!(p.body_states(), initial_bodies);
        assert!(p.forces().is_empty() && p.springs().is_empty());
        assert!(p.data.view().qfrc_applied.iter().all(|v| *v == 0.));
    }

    #[test]
    fn spring_damping_dissipates_energy_at_configured_step() {
        let mut p = scene("implicitfast", false);
        let left = point(p.names().bodies["left"], [0., 0.2, 0.]);
        let right = point(p.names().bodies["right"], [0., -0.1, 0.]);
        p.put_spring("spring".into(), spring(left, right)).unwrap();
        let energy = |p: &mut Physics| {
            let extension = p.spring_states()["spring"].length.get::<meter>() - 1.;
            p.model.kinetic_energy(&mut p.data) + 0.5 * 40. * extension * extension
        };
        let initial = energy(&mut p);
        p.step(4000, &[]).unwrap();
        assert!(energy(&mut p) < initial * 0.01);
    }
}
