use crate::{config::Config, motor, physics::Physics};
use anyhow::{Context, Result, ensure};
use damiao_simulator_rs::Motor;
use openarm_simulator_core_rs::MotorStates;
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
        let physics = Physics::load(path, &config)?;
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
            .map(|(binding, motor)| (binding.name.clone(), motor::snapshot(motor)))
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
