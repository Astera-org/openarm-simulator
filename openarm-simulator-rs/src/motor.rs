//! Simulated motor registers, faults and command latching. Wire codecs live in openarm-can-rs.
use anyhow::{Result, bail, ensure};
use openarm_can_rs::{
    ControlMode, DM4310_DEFAULT_MAPPING_RANGES, DM4340_DEFAULT_MAPPING_RANGES,
    DM8009_DEFAULT_MAPPING_RANGES, Feedback, MappingRanges, MitCommand, MotorStatus,
    RegisterAddress, Request,
};
use openarm_simulator_core_rs::MotorState;

// v1 joint/motor and command/reply-ID mapping obtained from enactic/openarm_ros2:
// 4e837e1d0dae692ff67b560b69d8d281d7a8d4ed,
// openarm_hardware/include/openarm_hardware/openarm_simple_hardware.hpp.
pub const V1_REPLY_ID_OFFSET: u16 = 0x10;
const SIMULATED_TEMPERATURE_C: u8 = 25; // Simulator default; no thermal model.
const SIMULATED_TIMEOUT: u32 = 0; // Simulator default; firmware watchdog is not modeled.

fn default_ranges(joint: usize) -> MappingRanges {
    match joint {
        1 | 2 => DM8009_DEFAULT_MAPPING_RANGES,
        3 | 4 => DM4340_DEFAULT_MAPPING_RANGES,
        5..=8 => DM4310_DEFAULT_MAPPING_RANGES,
        _ => panic!("v1 motor index must be 1..8"),
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Motor {
    pub joint: usize,
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
    pub fn new(joint: usize) -> Self {
        Self {
            joint,
            command: MitCommand::default(),
            q: 0.,
            dq: 0.,
            torque: 0.,
            status: MotorStatus::DISABLED,
            mos_temperature: SIMULATED_TEMPERATURE_C,
            rotor_temperature: SIMULATED_TEMPERATURE_C,
            silent: false,
            ranges: default_ranges(joint),
        }
    }

    pub fn snapshot(&self) -> MotorState {
        MotorState {
            joint: self.joint,
            command: self.command,
            q: self.q,
            dq: self.dq,
            torque: self.torque,
            status: self.status,
            mos_temperature: self.mos_temperature,
            rotor_temperature: self.rotor_temperature,
            silent: self.silent,
            ranges: self.ranges,
        }
    }

    pub fn state(&self) -> Result<[u8; 8]> {
        Ok(Feedback {
            reported_id: self.joint as u8,
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
        let Some(request) = Request::decode(id, self.joint as u16, data, self.ranges)? else {
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
            | Request::ClearError(_) => {
                bail!("only CTRL_MODE=MIT is supported")
            }
            Request::SetZero(_) => bail!("setting the motor zero is not simulated"),
            Request::ReadRegister(rid) | Request::WriteRegister { register: rid, .. } => {
                if let Request::WriteRegister { value, .. } = request {
                    match rid {
                        RegisterAddress::CTRL_MODE => {
                            ensure!(
                                u32::from_le_bytes(value) == ControlMode::Mit as u32,
                                "only CTRL_MODE=MIT is supported"
                            );
                            // Damiao Mode switching: changing mode clears command values.
                            self.command = MitCommand::default();
                        }
                        RegisterAddress::PMAX | RegisterAddress::VMAX | RegisterAddress::TMAX => {
                            let value = f32::from_le_bytes(value);
                            ensure!(value.is_finite() && value > 0., "invalid mapping range");
                            let range = match rid {
                                RegisterAddress::PMAX => &mut self.ranges.pmax,
                                RegisterAddress::VMAX => &mut self.ranges.vmax,
                                _ => &mut self.ranges.tmax,
                            };
                            // Damiao Write parameters: RAM writes take effect immediately.
                            *range = f64::from(value);
                        }
                        _ => bail!("unsupported simulated register write {}", rid.0),
                    }
                }
                let value = match rid {
                    RegisterAddress::MST_ID => {
                        (self.joint as u32 + u32::from(V1_REPLY_ID_OFFSET)).to_le_bytes()
                    }
                    RegisterAddress::ESC_ID => (self.joint as u32).to_le_bytes(),
                    RegisterAddress::TIMEOUT => SIMULATED_TIMEOUT.to_le_bytes(),
                    RegisterAddress::CTRL_MODE => (ControlMode::Mit as u32).to_le_bytes(),
                    RegisterAddress::PMAX => (self.ranges.pmax as f32).to_le_bytes(),
                    RegisterAddress::VMAX => (self.ranges.vmax as f32).to_le_bytes(),
                    RegisterAddress::TMAX => (self.ranges.tmax as f32).to_le_bytes(),
                    _ => bail!("unsupported simulated register {}", rid.0),
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
    fn snapshot_and_can_feedback_preserve_motor_fields() {
        let mut motor = Motor::new(1);
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

        let snapshot = motor.snapshot();
        let json = serde_json::to_value(snapshot).unwrap();
        assert_eq!(
            json,
            serde_json::json!({
                "joint": 1,
                "command": {"kp": 120.5, "kd": 1.5, "q": -1.25, "dq": 2.5, "tau": -3.75},
                "q": -7.5, "dq": 21.0, "torque": 6.5,
                "status": 2, "mos_temperature": 31, "rotor_temperature": 47, "silent": true,
                "ranges": {"pmax": 7.5, "vmax": 21.0, "tmax": 6.5}
            })
        );
        let restored: MotorState = serde_json::from_value(json).unwrap();
        assert_eq!(restored, snapshot);
        let feedback = Feedback::decode(&motor.state().unwrap(), restored.ranges).unwrap();
        assert_eq!(feedback.reported_id, restored.joint as u8);
        assert_eq!(
            (feedback.q, feedback.dq, feedback.torque),
            (restored.q, restored.dq, restored.torque)
        );
        assert_eq!(feedback.status, restored.status);
        assert_eq!(feedback.mos_temperature, restored.mos_temperature);
        assert_eq!(feedback.rotor_temperature, restored.rotor_temperature);
    }

    #[test]
    fn mapping_register_writes_change_command_and_feedback_scaling() {
        let mut motor = Motor::new(7);
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
                let before = motor.snapshot();
                let packet = Request::WriteRegister {
                    register: rid,
                    value: invalid.to_le_bytes(),
                }
                .encode(7, motor.ranges)
                .unwrap();
                assert!(motor.receive(packet.id, packet.data()).is_err());
                assert_eq!(motor.snapshot(), before);
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
            assert_eq!(motor.status, MotorStatus::DISABLED);
        }
    }

    #[test]
    fn unsupported_modes_leave_the_motor_unchanged() {
        use openarm_can_rs::{PositionForceCommand, PositionVelocityCommand, VelocityCommand};
        let mut motor = Motor::new(7);
        motor.command = MitCommand {
            kp: 4.,
            q: 1.,
            tau: 3.,
            ..MitCommand::default()
        };
        motor.status = MotorStatus::ENABLED;
        motor.torque = 3.;
        let before = motor.snapshot();
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
                assert_eq!(motor.snapshot(), before);
            }
        }
    }

    #[test]
    fn golden_packet_and_latched_fault_or_unknown_status() {
        let mut motor = Motor::new(1);
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
                assert_eq!(motor.snapshot().status, MotorStatus(raw));
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
