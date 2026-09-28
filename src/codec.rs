//! Hardware-free codec harness for conformance with the official C++ driver.
use crate::protocol::Motor;
use anyhow::{Result, ensure};
use serde::Deserialize;
use serde_json::json;
use std::io;

#[derive(Deserialize)]
struct Case {
    joint: usize,
    packet: Option<(u32, Vec<u8>)>,
    state: Option<(f64, f64, f64, u8)>,
}

pub fn run() -> Result<()> {
    let cases: Vec<Case> = serde_json::from_reader(io::stdin().lock())?;
    let mut results = Vec::with_capacity(cases.len());
    for case in cases {
        ensure!((1..=8).contains(&case.joint), "invalid motor index");
        let mut motor = Motor::new(case.joint);
        if let Some((q, dq, torque, status)) = case.state {
            ensure!(
                [q, dq, torque].iter().all(|v| v.is_finite()) && status <= 15,
                "invalid motor state"
            );
            motor.q = q;
            motor.dq = dq;
            motor.torque = torque;
            motor.status = status;
        }
        let reply = case
            .packet
            .map(|(id, data)| motor.receive(id, &data))
            .transpose();
        results.push(match reply {
            Ok(reply) => json!({"motor": motor, "state": motor.state(), "reply": reply.flatten()}),
            Err(e) => json!({"error": e.to_string(), "motor": motor}),
        });
    }
    serde_json::to_writer(io::stdout().lock(), &results)?;
    Ok(())
}
