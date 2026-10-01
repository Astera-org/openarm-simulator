use crate::{
    motor,
    physics::{Config, Physics},
};
use anyhow::Result;
use damiao_simulator_rs::Motor;
use openarm_simulator_core_rs::{ArmStates, Arms};
use std::path::Path;

pub struct Simulation {
    pub physics: Physics,
    pub motors: [[Motor; 8]; 2],
}

impl Simulation {
    pub fn load(path: &Path, config: Config) -> Result<Self> {
        let physics = Physics::load(path, config)?;
        let motors = std::array::from_fn(|_| std::array::from_fn(|i| motor::v1_motor(i + 1)));
        let mut simulation = Self { physics, motors };
        simulation.observe();
        Ok(simulation)
    }

    pub fn reset(&mut self) -> Result<()> {
        self.physics.reset()?;
        for motor in self.motors.iter_mut().flatten() {
            motor.reset();
        }
        self.observe();
        Ok(())
    }

    pub fn step(&mut self, count: u64) -> Result<()> {
        let drives = self.motors.map(|arm| arm.map(|motor| motor.drive()));
        self.physics.step(count, &drives)?;
        self.observe();
        Ok(())
    }

    fn observe(&mut self) {
        for (motor, state) in self
            .motors
            .iter_mut()
            .flatten()
            .zip(self.physics.observations().into_iter().flatten())
        {
            motor.observe(state);
        }
    }

    pub fn snapshot(&self) -> ArmStates {
        Arms {
            right: self.motors[0].map(|m| motor::snapshot(&m)),
            left: self.motors[1].map(|m| motor::snapshot(&m)),
        }
    }

    pub fn push(&mut self, torques: [[f64; 7]; 2]) -> Result<()> {
        self.physics.push(torques)
    }
}
