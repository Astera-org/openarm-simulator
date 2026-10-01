//! Damiao motor-controller emulation. Mechanical observations are supplied by the caller.
use damiao_can_rs::{
    ControlMode, Feedback, MappingRanges, MitCommand, MotorStatus, REGISTER_CAN_ID,
    RegisterAddress, Request,
};
use serde::{Deserialize, Serialize};

type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, PartialEq, thiserror::Error)]
pub enum Error {
    #[error(transparent)]
    Protocol(#[from] damiao_can_rs::Error),
    #[error("reply ID must be a standard CAN ID distinct from the motor and register IDs")]
    InvalidReplyId,
    #[error("mapping ranges must be positive finite float32 register values")]
    InvalidMappingRange,
    #[error("only CTRL_MODE=MIT is supported")]
    UnsupportedMode,
    #[error("setting the motor zero is not simulated")]
    SetZero,
    #[error("unsupported simulated register write {0}")]
    UnsupportedRegisterWrite(u8),
    #[error("unsupported simulated register read {0}")]
    UnsupportedRegisterRead(u8),
}

/// Startup settings restored by a controller reset.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct MotorConfig {
    pub id: u16,
    pub reply_id: u16,
    pub ranges: MappingRanges,
}

impl MotorConfig {
    pub fn validate(&self) -> Result<()> {
        ControlMode::Mit.command_id(self.id)?;
        if self.reply_id >= REGISTER_CAN_ID as u16 || self.reply_id == self.id {
            return Err(Error::InvalidReplyId);
        }
        if ![self.ranges.pmax, self.ranges.vmax, self.ranges.tmax]
            .into_iter()
            .all(|v| v.is_finite() && v > 0. && (v as f32).is_finite() && v as f32 > 0.)
        {
            return Err(Error::InvalidMappingRange);
        }
        Ok(())
    }
}

const SIMULATED_TEMPERATURE_C: u8 = 25; // No thermal model.
const SIMULATED_TIMEOUT: u32 = 0; // No firmware watchdog model.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motor {
    config: MotorConfig,
    pub command: MitCommand,
    pub q: f64,
    pub dq: f64,
    pub torque: f64,
    pub status: MotorStatus,
    pub mos_temperature: u8,
    pub rotor_temperature: u8,
    pub silent: bool,
    pub ranges: MappingRanges,
}

impl Motor {
    pub fn new(config: MotorConfig) -> Result<Self> {
        config.validate()?;
        Ok(Self {
            config,
            command: MitCommand::default(),
            q: 0.,
            dq: 0.,
            torque: 0.,
            status: MotorStatus::DISABLED,
            mos_temperature: SIMULATED_TEMPERATURE_C,
            rotor_temperature: SIMULATED_TEMPERATURE_C,
            silent: false,
            ranges: config.ranges,
        })
    }

    pub fn id(&self) -> u16 {
        self.config.id
    }
    pub fn reply_id(&self) -> u16 {
        self.config.reply_id
    }

    /// Restore the controller to its startup configuration.
    pub fn reset(&mut self) {
        *self = Self::new(self.config).expect("validated motor configuration");
    }

    pub fn state(&self) -> Result<[u8; 8]> {
        Ok(Feedback {
            reported_id: (self.id() & 0xf) as u8,
            q: self.q,
            dq: self.dq,
            torque: self.torque,
            status: self.status,
            mos_temperature: self.mos_temperature,
            rotor_temperature: self.rotor_temperature,
        }
        .encode(self.ranges)?)
    }

    pub fn receive(&mut self, id: u32, data: &[u8]) -> Result<Option<[u8; 8]>> {
        let Some(request) = Request::decode(id, self.id(), data, self.ranges)? else {
            return Ok(None);
        };
        match request {
            Request::Mit(command) => self.command = command,
            Request::Enable(ControlMode::Mit) => {
                if matches!(self.status, MotorStatus::DISABLED | MotorStatus::ENABLED) {
                    self.status = MotorStatus::ENABLED;
                }
            }
            Request::Disable(ControlMode::Mit) => {
                if matches!(self.status, MotorStatus::DISABLED | MotorStatus::ENABLED) {
                    self.status = MotorStatus::DISABLED;
                }
                self.torque = 0.;
            }
            Request::ClearError(ControlMode::Mit) => {
                self.status = MotorStatus::DISABLED;
                self.command = MitCommand::default();
                self.torque = 0.;
            }
            Request::Feedback => (),
            Request::PositionVelocity(_)
            | Request::Velocity(_)
            | Request::PositionForce(_)
            | Request::Enable(_)
            | Request::Disable(_)
            | Request::ClearError(_) => return Err(Error::UnsupportedMode),
            Request::SetZero(_) => return Err(Error::SetZero),
            Request::ReadRegister(rid) | Request::WriteRegister { register: rid, .. } => {
                if let Request::WriteRegister { value, .. } = request {
                    match rid {
                        RegisterAddress::CTRL_MODE => {
                            if u32::from_le_bytes(value) != ControlMode::Mit as u32 {
                                return Err(Error::UnsupportedMode);
                            }
                            // Damiao Mode switching: changing mode clears command values.
                            self.command = MitCommand::default();
                        }
                        RegisterAddress::PMAX | RegisterAddress::VMAX | RegisterAddress::TMAX => {
                            let value = f32::from_le_bytes(value);
                            if !value.is_finite() || value <= 0. {
                                return Err(Error::InvalidMappingRange);
                            }
                            let range = match rid {
                                RegisterAddress::PMAX => &mut self.ranges.pmax,
                                RegisterAddress::VMAX => &mut self.ranges.vmax,
                                _ => &mut self.ranges.tmax,
                            };
                            // Damiao Write parameters: RAM writes take effect immediately.
                            *range = f64::from(value);
                        }
                        _ => return Err(Error::UnsupportedRegisterWrite(rid.0)),
                    }
                }
                let value = match rid {
                    RegisterAddress::MST_ID => u32::from(self.reply_id()).to_le_bytes(),
                    RegisterAddress::ESC_ID => u32::from(self.id()).to_le_bytes(),
                    RegisterAddress::TIMEOUT => SIMULATED_TIMEOUT.to_le_bytes(),
                    RegisterAddress::CTRL_MODE => (ControlMode::Mit as u32).to_le_bytes(),
                    RegisterAddress::PMAX => (self.ranges.pmax as f32).to_le_bytes(),
                    RegisterAddress::VMAX => (self.ranges.vmax as f32).to_le_bytes(),
                    RegisterAddress::TMAX => (self.ranges.tmax as f32).to_le_bytes(),
                    _ => return Err(Error::UnsupportedRegisterRead(rid.0)),
                };
                // The reply is always eight bytes, even for a four-byte read request.
                let mut reply = [0; 8];
                reply[..4].copy_from_slice(&data[..4]);
                reply[4..].copy_from_slice(&value);
                return Ok(Some(reply));
            }
        }
        Ok(Some(self.state()?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn startup_configuration_addresses_and_reset() {
        let config = MotorConfig {
            id: 0x123,
            reply_id: 0x456,
            ranges: MappingRanges {
                pmax: 8.,
                vmax: 20.,
                tmax: 5.,
            },
        };
        let mut motor = Motor::new(config).unwrap();
        for (register, expected) in [
            (RegisterAddress::ESC_ID, 0x123u32),
            (RegisterAddress::MST_ID, 0x456),
        ] {
            let request = Request::ReadRegister(register)
                .encode(config.id, config.ranges)
                .unwrap();
            let response = motor.receive(request.id, request.data()).unwrap().unwrap();
            assert_eq!(&response[4..], &expected.to_le_bytes());
        }
        assert_eq!(
            Feedback::decode(&motor.state().unwrap(), config.ranges)
                .unwrap()
                .reported_id,
            3
        );
        motor.ranges.pmax = 10.;
        motor.status = MotorStatus::ENABLED;
        motor.command.tau = 2.;
        motor.silent = true;
        motor.reset();
        assert_eq!(motor, Motor::new(config).unwrap());
        for invalid in [
            MotorConfig {
                id: 0x7ff,
                ..config
            },
            MotorConfig {
                reply_id: 0x800,
                ..config
            },
            MotorConfig {
                reply_id: config.id,
                ..config
            },
            MotorConfig {
                ranges: MappingRanges {
                    pmax: f64::MAX,
                    ..config.ranges
                },
                ..config
            },
        ] {
            assert!(Motor::new(invalid).is_err());
        }
    }

    fn motor(id: u16, vmax: f64, tmax: f64) -> Motor {
        Motor::new(MotorConfig {
            id,
            reply_id: id + 0x10,
            ranges: MappingRanges {
                pmax: 12.5,
                vmax,
                tmax,
            },
        })
        .unwrap()
    }

    #[test]
    fn mapping_register_writes_change_command_and_feedback_scaling() {
        let mut motor = motor(7, 30., 10.);
        for (rid, value) in [
            (RegisterAddress::PMAX, 7.5f32),
            (RegisterAddress::VMAX, 21.),
            (RegisterAddress::TMAX, 6.5),
        ] {
            let packet = Request::WriteRegister {
                register: rid,
                value: value.to_le_bytes(),
            }
            .encode(7, motor.ranges)
            .unwrap();
            assert_eq!(
                motor.receive(packet.id, packet.data()).unwrap(),
                Some(packet.data().try_into().unwrap())
            );
            let query = Request::ReadRegister(rid).encode(7, motor.ranges).unwrap();
            let reply = motor
                .receive(query.id, &query.data()[..4])
                .unwrap()
                .unwrap();
            assert_eq!(&reply[..4], &query.data()[..4]);
            assert_eq!(&reply[4..], &value.to_le_bytes());
            for invalid in [0., -1., f32::NAN, f32::INFINITY] {
                let before = motor;
                let packet = Request::WriteRegister {
                    register: rid,
                    value: invalid.to_le_bytes(),
                }
                .encode(7, motor.ranges)
                .unwrap();
                assert!(motor.receive(packet.id, packet.data()).is_err());
                assert_eq!(motor, before);
            }
        }
        assert_eq!(
            motor.ranges,
            MappingRanges {
                pmax: 7.5,
                vmax: 21.,
                tmax: 6.5
            }
        );
        let command = MitCommand {
            q: motor.ranges.pmax,
            dq: motor.ranges.vmax,
            tau: motor.ranges.tmax,
            ..MitCommand::default()
        };
        let packet = Request::Mit(command).encode(7, motor.ranges).unwrap();
        motor.receive(packet.id, packet.data()).unwrap();
        assert_eq!(motor.command, command);
        motor.q = command.q;
        motor.dq = command.dq;
        motor.torque = command.tau;
        let feedback = Feedback::decode(&motor.state().unwrap(), motor.ranges).unwrap();
        assert_eq!(
            (feedback.q, feedback.dq, feedback.torque),
            (command.q, command.dq, command.tau)
        );
        assert_eq!(feedback.status, MotorStatus::DISABLED);
    }

    #[test]
    fn registers_and_unsupported_requests() {
        let mut motor = motor(7, 30., 10.);
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
            assert_eq!(motor.status, MotorStatus::DISABLED);
        }
    }

    #[test]
    fn unsupported_modes_leave_the_motor_unchanged() {
        use damiao_can_rs::{PositionForceCommand, PositionVelocityCommand, VelocityCommand};
        let mut motor = motor(7, 30., 10.);
        motor.command = MitCommand {
            kp: 4.,
            q: 1.,
            tau: 3.,
            ..MitCommand::default()
        };
        motor.status = MotorStatus::ENABLED;
        motor.torque = 3.;
        let before = motor;
        for (mode, command) in [
            (
                ControlMode::PositionVelocity,
                Request::PositionVelocity(PositionVelocityCommand::default()),
            ),
            (
                ControlMode::Velocity,
                Request::Velocity(VelocityCommand::default()),
            ),
            (
                ControlMode::PositionForce,
                Request::PositionForce(PositionForceCommand::default()),
            ),
        ] {
            for request in [
                command,
                Request::Enable(mode),
                Request::Disable(mode),
                Request::ClearError(mode),
                Request::SetZero(mode),
                Request::WriteRegister {
                    register: RegisterAddress::CTRL_MODE,
                    value: (mode as u32).to_le_bytes(),
                },
            ] {
                let packet = request.encode(7, motor.ranges).unwrap();
                assert!(
                    motor.receive(packet.id, packet.data()).is_err(),
                    "{request:?}"
                );
                assert_eq!(motor, before);
            }
        }
    }

    #[test]
    fn golden_packet_and_latched_fault_or_unknown_status() {
        let mut motor = motor(1, 45., 54.);
        motor
            .receive(1, &[0x7f, 0xff, 0x7f, 0xf8, 0, 0x80, 7, 0xff])
            .unwrap();
        assert!((motor.command.q + 12.5 / 65535.).abs() < 1e-12);
        assert!((motor.command.dq + 45. / 4095.).abs() < 1e-12);
        assert_eq!(
            motor.state().unwrap(),
            [1, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 25, 25]
        );
        for raw in 2..=15 {
            motor.status = MotorStatus(raw);
            for opcode in [0xfc, 0xfd] {
                let reply = motor
                    .receive(1, &[255, 255, 255, 255, 255, 255, 255, opcode])
                    .unwrap()
                    .unwrap();
                assert_eq!(reply[0] >> 4, raw);
                assert_eq!(motor.status, MotorStatus(raw));
            }
            motor
                .receive(1, &[255, 255, 255, 255, 255, 255, 255, 251])
                .unwrap();
            assert_eq!(motor.status, MotorStatus::DISABLED);
        }
        assert_eq!(motor.command.kp, 0.);
        assert!(
            motor
                .receive(1, &[255, 255, 255, 255, 255, 255, 255, 254])
                .is_err()
        );
    }
}
