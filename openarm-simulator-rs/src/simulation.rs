use crate::{config::Config, physics::Physics};
use anyhow::{Context, Result, ensure};
use damiao_simulator_rs::Motor;
use mujoco_rs::Model;
use openarm_simulator_core_rs::{
    MotorState, MotorStates,
    uom::si::{
        angle::radian,
        angular_velocity::radian_per_second,
        f64::{Angle, AngularVelocity, ThermodynamicTemperature, Torque},
        thermodynamic_temperature::degree_celsius,
        torque::newton_meter,
    },
};
use std::{collections::BTreeSet, path::Path};

pub struct Binding {
    pub name: String,
    pub bus: usize,
}

pub struct Simulation {
    pub physics: Physics,
    pub motors: Vec<Motor>,
    pub bindings: Vec<Binding>,
}

impl Simulation {
    pub fn load(path: &Path, config: Config) -> Result<Self> {
        Self::new(Model::from_xml(path)?, config)
    }

    pub fn new(model: Model, config: Config) -> Result<Self> {
        let mut motors = Vec::new();
        let mut bindings = Vec::new();
        let mut addresses = BTreeSet::new();
        let mut actuators = BTreeSet::new();
        for (name, binding) in &config.motors {
            ensure!(!name.is_empty(), "motor name must not be empty");
            let bus = config
                .buses
                .keys()
                .position(|bus| bus == &binding.bus)
                .with_context(|| format!("unknown bus {} for {name}", binding.bus))?;
            let motor =
                Motor::new(binding.controller).with_context(|| format!("invalid motor {name}"))?;
            for id in [motor.id(), motor.reply_id()] {
                ensure!(
                    addresses.insert((bus, id)),
                    "CAN address conflict on {}: {id}",
                    binding.bus
                );
            }
            ensure!(
                actuators.insert(&binding.actuator),
                "actuator {} is bound more than once",
                binding.actuator
            );
            motors.push(motor);
            bindings.push(Binding {
                name: name.clone(),
                bus,
            });
        }
        let physics = Physics::new(model, &config)?;
        let mut simulation = Self {
            physics,
            motors,
            bindings,
        };
        simulation.observe();
        Ok(simulation)
    }

    pub fn reset(&mut self) -> Result<()> {
        self.physics.reset()?;
        for motor in &mut self.motors {
            motor.reset();
        }
        self.observe();
        Ok(())
    }

    pub fn step(&mut self, count: u64) -> Result<()> {
        let drives: Vec<_> = self.motors.iter().map(Motor::drive).collect();
        self.physics.step(count, &drives)?;
        self.observe();
        Ok(())
    }

    fn observe(&mut self) {
        for (motor, observation) in self.motors.iter_mut().zip(self.physics.observations()) {
            motor.observe(observation);
        }
    }

    pub fn snapshot(&self) -> MotorStates {
        self.bindings
            .iter()
            .zip(&self.motors)
            .map(|(binding, motor)| (binding.name.clone(), motor_snapshot(motor)))
            .collect()
    }

    pub fn motor_mut(&mut self, name: &str) -> Result<&mut Motor> {
        let index = self
            .bindings
            .iter()
            .position(|b| b.name == name)
            .with_context(|| format!("unknown motor {name}"))?;
        Ok(&mut self.motors[index])
    }

    pub fn push(&mut self, torques: openarm_simulator_core_rs::Push) -> Result<()> {
        self.physics.push(torques)
    }
}

fn motor_snapshot(motor: &Motor) -> MotorState {
    MotorState {
        id: motor.id(),
        reply_id: motor.reply_id(),
        command: motor.command.into(),
        q: Angle::new::<radian>(motor.q),
        dq: AngularVelocity::new::<radian_per_second>(motor.dq),
        torque: Torque::new::<newton_meter>(motor.torque),
        status: motor.status,
        mos_temperature: ThermodynamicTemperature::new::<degree_celsius>(
            motor.mos_temperature.into(),
        ),
        rotor_temperature: ThermodynamicTemperature::new::<degree_celsius>(
            motor.rotor_temperature.into(),
        ),
        silent: motor.silent,
        ranges: motor.ranges.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use damiao_can_rs::{Feedback, MappingRanges, MitCommand, MotorStatus};
    use damiao_simulator_rs::MotorConfig;
    #[test]
    fn snapshot_and_can_feedback_preserve_motor_fields() {
        let mut motor = Motor::new(MotorConfig {
            id: 1,
            reply_id: 17,
            ranges: MappingRanges {
                pmax: 12.5,
                vmax: 45.,
                tmax: 54.,
            },
        })
        .unwrap();
        motor.command = MitCommand {
            kp: 120.5,
            kd: 1.5,
            q: -1.25,
            dq: 2.5,
            tau: -3.75,
        };
        motor.ranges = MappingRanges {
            pmax: 7.5,
            vmax: 21.,
            tmax: 6.5,
        };
        motor.q = -7.5;
        motor.dq = 21.;
        motor.torque = 6.5;
        motor.status = MotorStatus(2);
        motor.mos_temperature = 31;
        motor.rotor_temperature = 47;
        motor.silent = true;

        let snapshot = motor_snapshot(&motor);
        let json = serde_json::to_value(snapshot).unwrap();
        assert_eq!(
            json,
            serde_json::json!({
                "id": 1, "reply_id": 17,
                "command": {"kp_nm_per_rad": 120.5, "kd_nm_s_per_rad": 1.5, "q_rad": -1.25, "dq_rad_s": 2.5, "tau_nm": -3.75},
                "q_rad": -7.5, "dq_rad_s": 21.0, "torque_nm": 6.5,
                "status": 2, "mos_temperature_k": 304.15, "rotor_temperature_k": 320.15, "silent": true,
                "ranges": {"pmax_rad": 7.5, "vmax_rad_s": 21.0, "tmax_nm": 6.5}
            })
        );
        let restored: MotorState = serde_json::from_value(json).unwrap();
        assert_eq!(
            damiao_can_rs::MitCommand::from(restored.command),
            motor.command
        );
        assert_eq!(restored, snapshot);
        let feedback = Feedback::decode(&motor.state().unwrap(), restored.ranges.into()).unwrap();
        assert_eq!(feedback.reported_id, restored.id as u8);
        assert_eq!(
            (feedback.q, feedback.dq, feedback.torque),
            (
                restored.q.get::<radian>(),
                restored.dq.get::<radian_per_second>(),
                restored.torque.get::<newton_meter>()
            )
        );
        assert_eq!(feedback.status, restored.status);
        assert_eq!(
            f64::from(feedback.mos_temperature),
            restored.mos_temperature.get::<degree_celsius>()
        );
        assert_eq!(
            f64::from(feedback.rotor_temperature),
            restored.rotor_temperature.get::<degree_celsius>()
        );
    }
}
