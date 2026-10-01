use crate::{
    motor,
    physics::{Config, Physics},
};
use anyhow::{Context, Result, ensure};
use damiao_simulator_rs::{Drive, Motor};
use openarm_simulator_core_rs::MotorStates;
use std::{collections::BTreeSet, path::Path};

pub struct Binding {
    pub name: String,
    pub bus: usize,
    pub port: (usize, usize),
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
                port: (0, 0),
            });
        }
        let names: Vec<_> = config.motors.values().map(|b| b.actuator.clone()).collect();
        let physics = Physics::load(path, config)?;
        for (binding, name) in bindings.iter_mut().zip(names) {
            binding.port = physics.port(&name)?;
        }
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
        let mut drives = [[Drive::default(); 8]; 2];
        for (binding, motor) in self.bindings.iter().zip(&self.motors) {
            drives[binding.port.0][binding.port.1] = motor.drive();
        }
        self.physics.step(count, &drives)?;
        self.observe();
        Ok(())
    }

    fn observe(&mut self) {
        let observations = self.physics.observations();
        for (binding, motor) in self.bindings.iter().zip(&mut self.motors) {
            motor.observe(observations[binding.port.0][binding.port.1]);
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

    pub fn push(&mut self, torques: [[f64; 7]; 2]) -> Result<()> {
        self.physics.push(torques)
    }
}
