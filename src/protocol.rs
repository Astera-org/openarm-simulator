//! Damiao MIT wire behavior. Pure data; no sockets or physics in this module.
use anyhow::{Result, bail, ensure};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
pub struct Command {
    pub kp: f64,
    pub kd: f64,
    pub q: f64,
    pub dq: f64,
    pub tau: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct Limits {
    pub position: f64,
    pub velocity: f64,
    pub torque: f64,
}

// Pinned openarm_can MOTOR_LIMIT_PARAMS: DM8009, DM4340, DM4310.
// Golden vectors below were captured from the official 1.4.0 encoder.
pub fn limits(joint: usize) -> Limits {
    let (velocity, torque) = match joint {
        1 | 2 => (45., 54.),
        3 | 4 => (10., 28.),
        5..=8 => (30., 10.),
        _ => panic!("v1 motor index must be 1..8"),
    };
    Limits {
        position: 12.5,
        velocity,
        torque,
    }
}

pub fn unpack(value: u32, maximum: f64, bits: u32) -> f64 {
    (2. * value as f64 / ((1u32 << bits) - 1) as f64 - 1.) * maximum
}

pub fn pack(value: f64, maximum: f64, bits: u32) -> u32 {
    ((value.clamp(-maximum, maximum) + maximum) / (2. * maximum) * ((1u32 << bits) - 1) as f64)
        as u32
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct Motor {
    pub joint: usize,
    pub command: Command,
    pub q: f64,
    pub dq: f64,
    pub torque: f64,
    pub status: u8,
    pub temperature: u8,
    pub silent: bool,
    pub limits: Limits,
}

impl Motor {
    pub fn new(joint: usize) -> Self {
        Self {
            joint,
            command: Command::default(),
            q: 0.,
            dq: 0.,
            torque: 0.,
            status: 0,
            temperature: 25,
            silent: false,
            limits: limits(joint),
        }
    }

    pub fn state(&self) -> [u8; 8] {
        let q = pack(self.q, self.limits.position, 16);
        let v = pack(self.dq, self.limits.velocity, 12);
        let t = pack(self.torque, self.limits.torque, 12);
        [
            self.joint as u8 | self.status << 4,
            (q >> 8) as u8,
            q as u8,
            (v >> 4) as u8,
            ((v & 15) << 4 | t >> 8) as u8,
            t as u8,
            self.temperature,
            self.temperature,
        ]
    }

    pub fn receive(&mut self, id: u32, data: &[u8]) -> Result<Option<[u8; 8]>> {
        ensure!(data.len() == 8, "motor packet must contain eight bytes");
        if id == 0x7ff {
            if u16::from_le_bytes([data[0], data[1]]) as usize != self.joint {
                return Ok(None);
            }
            let (operation, rid) = (data[2], data[3]);
            if operation == 0xcc {
                return Ok(Some(self.state()));
            }
            match operation {
                0x55 => {
                    ensure!(
                        rid == 10 && u32::from_le_bytes(data[4..].try_into()?) == 1,
                        "only CTRL_MODE=MIT writes are supported"
                    );
                    self.command = Command::default();
                }
                0x33 => (),
                _ => bail!("unsupported register operation {operation:#x}"),
            }
            let value = match rid {
                7 => (self.joint as u32 + 16).to_le_bytes(),
                8 => (self.joint as u32).to_le_bytes(),
                9 => 0u32.to_le_bytes(), // No firmware watchdog model.
                10 => 1u32.to_le_bytes(),
                21 => (self.limits.position as f32).to_le_bytes(),
                22 => (self.limits.velocity as f32).to_le_bytes(),
                23 => (self.limits.torque as f32).to_le_bytes(),
                _ => bail!("unsupported simulated register {rid}"),
            };
            let mut reply: [u8; 8] = data.try_into()?;
            reply[4..].copy_from_slice(&value);
            return Ok(Some(reply));
        }
        if id != self.joint as u32 {
            return Ok(None);
        }
        if data[..7] == [0xff; 7] {
            match data[7] {
                0xfc => {
                    if self.status < 8 {
                        self.status = 1;
                    }
                }
                0xfd => {
                    if self.status < 8 {
                        self.status = 0;
                    }
                    self.torque = 0.;
                }
                0xfb => {
                    self.status = 0;
                    self.command = Command::default();
                    self.torque = 0.;
                }
                _ => bail!("zero/save commands are not supported"),
            }
        } else {
            let b: [u32; 8] = std::array::from_fn(|i| data[i] as u32);
            self.command = Command {
                q: unpack(b[0] << 8 | b[1], self.limits.position, 16),
                dq: unpack(b[2] << 4 | b[3] >> 4, self.limits.velocity, 12),
                kp: ((b[3] & 15) << 8 | b[4]) as f64 * 500. / 4095.,
                kd: (b[5] << 4 | b[6] >> 4) as f64 * 5. / 4095.,
                tau: unpack((b[6] & 15) << 8 | b[7], self.limits.torque, 12),
            };
        }
        Ok(Some(self.state()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn official_encoder_vectors_and_registers() {
        #[derive(Deserialize)]
        struct Vector {
            joint: usize,
            packet: [u8; 8],
            command: Command,
        }
        #[derive(Deserialize)]
        struct Vectors {
            vectors: Vec<Vector>,
        }
        let vectors: Vectors =
            serde_json::from_str(include_str!("../tests/data/mit_commands.json")).unwrap();
        for vector in vectors.vectors {
            let mut motor = Motor::new(vector.joint);
            motor.receive(vector.joint as u32, &vector.packet).unwrap();
            let (got, want) = (motor.command, vector.command);
            for (got, want, step) in [
                (got.q, want.q, 25. / 65535.),
                (got.dq, want.dq, 2. * motor.limits.velocity / 4095.),
                (got.tau, want.tau, 2. * motor.limits.torque / 4095.),
                (got.kp, want.kp, 500. / 4095.),
                (got.kd, want.kd, 5. / 4095.),
            ] {
                assert!((got - want).abs() <= step + 1e-5, "{got} != {want}");
            }
        }
        let mut motor = Motor::new(7);
        for (register, value) in [
            (7, 23u32.to_le_bytes()),
            (8, 7u32.to_le_bytes()),
            (9, 0u32.to_le_bytes()),
            (10, 1u32.to_le_bytes()),
            (21, 12.5f32.to_le_bytes()),
            (22, 30f32.to_le_bytes()),
            (23, 10f32.to_le_bytes()),
        ] {
            let reply = motor
                .receive(0x7ff, &[7, 0, 0x33, register, 0, 0, 0, 0])
                .unwrap()
                .unwrap();
            assert_eq!(&reply[4..], &value);
        }
        for packet in [
            [7, 0, 0x55, 54, 0, 0, 0, 0],
            [7, 0, 0x55, 10, 2, 0, 0, 0],
            [7, 0, 0x33, 255, 0, 0, 0, 0],
            [7, 0, 0xaa, 0, 0, 0, 0, 0],
        ] {
            assert!(motor.receive(0x7ff, &packet).is_err());
            assert_eq!(motor.status, 0);
        }
    }

    #[test]
    fn golden_packet_and_latched_fault() {
        let mut motor = Motor::new(1);
        motor
            .receive(1, &[0x7f, 0xff, 0x7f, 0xf8, 0, 0x80, 7, 0xff])
            .unwrap();
        assert!((motor.command.q + 12.5 / 65535.).abs() < 1e-12);
        assert!((motor.command.dq + 45. / 4095.).abs() < 1e-12);
        assert_eq!(motor.state(), [1, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 25, 25]);
        motor.status = 10;
        motor
            .receive(1, &[255, 255, 255, 255, 255, 255, 255, 252])
            .unwrap();
        motor
            .receive(1, &[255, 255, 255, 255, 255, 255, 255, 253])
            .unwrap();
        assert_eq!(motor.status, 10);
        motor
            .receive(1, &[255, 255, 255, 255, 255, 255, 255, 251])
            .unwrap();
        assert_eq!(motor.status, 0);
        assert_eq!(motor.command.kp, 0.);
        assert!(
            motor
                .receive(1, &[255, 255, 255, 255, 255, 255, 255, 254])
                .is_err()
        );
    }
}
