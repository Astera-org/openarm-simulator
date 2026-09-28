//! Accelerated, deterministic experiments using the SAME physics and wire codec.
//! This entry point opens no CAN/network socket and has no real-time deadline.
use crate::physics::{Arms, Physics, Pose};
use anyhow::{Result, ensure};
use serde::Deserialize;
use serde_json::{Value, json};
use std::io::{self, BufRead, Write};

#[derive(Deserialize)]
#[serde(tag = "action", rename_all = "snake_case", deny_unknown_fields)]
enum Request {
    Step {
        packets: Vec<(usize, u32, [u8; 8])>,
        steps: u64,
    },
    Reset {
        poses: Arms<Pose>,
    },
    Push {
        forces: [[f64; 7]; 2],
    },
    Inspect,
    Configuration,
    Truth,
}

fn request(physics: &mut Physics, message: Request) -> Result<Value> {
    match message {
        Request::Step { packets, steps } => {
            ensure!(
                steps <= 2000 && packets.len() <= 64,
                "experiment batch too large"
            );
            for (side, id, packet) in packets {
                ensure!(
                    side < 2 && (1..=8).contains(&id),
                    "invalid experiment motor"
                );
                physics.motors[side][id as usize - 1].receive(id, &packet)?;
            }
            physics.step(steps)?;
        }
        Request::Reset { poses } => physics.reset(poses)?,
        Request::Push { forces } => physics.push(forces)?,
        Request::Inspect => (),
        Request::Configuration => return Ok(physics.configuration()),
        Request::Truth => return Ok(physics.truth()),
    }
    let packets: Vec<_> = physics
        .motors
        .iter()
        .map(|arm| arm.iter().map(|motor| motor.state()).collect::<Vec<_>>())
        .collect();
    Ok(json!({"time": physics.time(), "packets": packets, "contacts": physics.contacts()}))
}

pub fn run(mut physics: Physics) -> Result<()> {
    let mut stdout = io::stdout().lock();
    for line in io::stdin().lock().lines() {
        let value = match serde_json::from_str::<Request>(&line?)
            .map_err(anyhow::Error::from)
            .and_then(|r| request(&mut physics, r))
        {
            Ok(value) => value,
            Err(error) => json!({"error": error.to_string()}),
        };
        serde_json::to_writer(&mut stdout, &value)?;
        writeln!(&mut stdout)?;
        stdout.flush()?;
    }
    Ok(())
}
