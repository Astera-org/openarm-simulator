// Motor manual (English translation): MIT, status codes, modes, and registers.
// https://damiao.enactic.ai/en/products/hardware/dm-j4340p-2ec-v1.0/
// Damiao Drive Control Protocol V1.4 (Chinese): §2.4 (ranges), §4 (CAN commands).
// https://raw.githubusercontent.com/dmBots/damiao-document/master/%E8%B0%83%E8%AF%95%E5%8A%A9%E6%89%8B%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6%EF%BC%88%E8%BE%BE%E5%A6%99%E9%A9%B1%E5%8A%A8%E6%8E%A7%E5%88%B6%E5%8D%8F%E8%AE%AE%EF%BC%89V1.4.pdf
// OpenArm range presets, 0xCC feedback request, and reference codec:
// https://github.com/enactic/openarm_can/tree/f340d4b808fb177e1f297af54eb55fd51c6c7c10

#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("invalid mapping ranges")]
    InvalidRanges,
    #[error("non-finite motor command")]
    NonFiniteCommand,
    #[error("motor command exceeds float32 range")]
    CommandOutOfRange,
    #[error("invalid motor feedback")]
    InvalidFeedback,
    #[error("invalid motor packet length")]
    InvalidLength,
    #[error("invalid motor CAN id")]
    InvalidMotorId,
    #[error("unsupported register operation {0:#x}")]
    UnsupportedRegisterOperation(u8),
    #[error("unknown control mode {0}")]
    UnknownControlMode(u32),
    #[error("reserved motor operation in MIT command")]
    ReservedMotorOperation,
}

const POSITION_BITS: u32 = 16;
const MIT_FIELD_BITS: u32 = 12;
pub const MIT_KP_MAX: f64 = 500.;
pub const MIT_KD_MAX: f64 = 5.;

// OpenArm PMAX/VMAX/TMAX presets.
pub const DM8009_DEFAULT_MAPPING_RANGES: MappingRanges = MappingRanges {
    pmax: 12.5,
    vmax: 45.,
    tmax: 54.,
};
pub const DM4340_DEFAULT_MAPPING_RANGES: MappingRanges = MappingRanges {
    pmax: 12.5,
    vmax: 10.,
    tmax: 28.,
};
pub const DM4310_DEFAULT_MAPPING_RANGES: MappingRanges = MappingRanges {
    pmax: 12.5,
    vmax: 30.,
    tmax: 10.,
};

/// Identifies a motor setting to read or change. Unrecognized addresses are preserved.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RegisterAddress(pub u8);

impl RegisterAddress {
    pub const MST_ID: Self = Self(7);
    pub const ESC_ID: Self = Self(8);
    pub const TIMEOUT: Self = Self(9);
    pub const CTRL_MODE: Self = Self(10);
    pub const PMAX: Self = Self(21);
    pub const VMAX: Self = Self(22);
    pub const TMAX: Self = Self(23);
}
/// Selects the motor's controller through the CTRL_MODE register.
#[repr(u32)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlMode {
    Mit = 1,
    PositionVelocity = 2,
    Velocity = 3,
    PositionForce = 4,
}

impl TryFrom<u32> for ControlMode {
    type Error = Error;

    fn try_from(value: u32) -> Result<Self, Error> {
        match value {
            1 => Ok(Self::Mit),
            2 => Ok(Self::PositionVelocity),
            3 => Ok(Self::Velocity),
            4 => Ok(Self::PositionForce),
            _ => Err(Error::UnknownControlMode(value)),
        }
    }
}

impl ControlMode {
    /// Finds the command address for a motor using this mode.
    /// Rejects addresses that collide with register traffic or exceed standard CAN IDs.
    pub fn command_id(self, motor_id: u16) -> Result<u32, Error> {
        validate_motor_id(motor_id)?;
        // Damiao control-frame tables: base ID + 0x000/0x100/0x200/0x300.
        let id = u32::from(motor_id) + (self as u32 - 1) * 0x100;
        if id >= REGISTER_CAN_ID {
            return Err(Error::InvalidMotorId);
        }
        Ok(id)
    }
}

/// A motor's reported operating state or fault. Unrecognized codes are preserved.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct MotorStatus(pub u8);

impl MotorStatus {
    pub const DISABLED: Self = Self(0x0);
    pub const ENABLED: Self = Self(0x1);
    pub const OVERVOLTAGE: Self = Self(0x8);
    pub const UNDERVOLTAGE: Self = Self(0x9);
    pub const OVERCURRENT: Self = Self(0xa);
    pub const MOS_OVERHEAT: Self = Self(0xb);
    pub const COIL_OVERHEAT: Self = Self(0xc);
    pub const COMMUNICATION_LOST: Self = Self(0xd);
    pub const OVERLOAD: Self = Self(0xe);
}

pub const REGISTER_CAN_ID: u32 = 0x7ff;
const READ_REGISTER: u8 = 0x33;
const WRITE_REGISTER: u8 = 0x55;
const REFRESH_FEEDBACK: u8 = 0xcc;
const ENABLE: u8 = 0xfc;
const DISABLE: u8 = 0xfd;
const SET_ZERO: u8 = 0xfe;
const CLEAR_ERROR: u8 = 0xfb;

/// Targets and gains for controlling a motor in MIT mode. All values must be finite.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct MitCommand {
    pub kp: f64,
    pub kd: f64,
    pub q: f64,
    pub dq: f64,
    pub tau: f64,
}

/// Scaling settings for MIT commands and feedback in every control mode.
/// Must match the motor's PMAX/VMAX/TMAX registers; each value must be positive
/// and at most `f64::MAX / 2`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct MappingRanges {
    /// PMAX setting, in radians.
    pub pmax: f64,
    /// VMAX setting, in radians/second.
    pub vmax: f64,
    /// TMAX setting, in newton-metres.
    pub tmax: f64,
}

impl MappingRanges {
    fn validate(self) -> Result<(), Error> {
        if [self.pmax, self.vmax, self.tmax]
            .iter()
            .all(|x| x.is_finite() && *x > 0. && (2. * x).is_finite())
        {
            Ok(())
        } else {
            Err(Error::InvalidRanges)
        }
    }
}

// Floating-point operation order affects quantization at code boundaries.
fn unpack(value: u32, minimum: f64, maximum: f64, bits: u32) -> f64 {
    value as f64 / ((1u32 << bits) - 1) as f64 * (maximum - minimum) + minimum
}
fn pack(value: f64, minimum: f64, maximum: f64, bits: u32) -> u32 {
    ((value.clamp(minimum, maximum) - minimum) / (maximum - minimum) * ((1u32 << bits) - 1) as f64)
        as u32
}

impl MitCommand {
    /// Prepares a motion command to send to a motor in MIT mode.
    ///
    /// Values outside the configured limits are reduced to those limits.
    /// Rejects commands that would trigger a different motor operation.
    pub fn encode(self, ranges: MappingRanges) -> Result<[u8; 8], Error> {
        ranges.validate()?;
        if ![self.q, self.dq, self.tau, self.kp, self.kd]
            .iter()
            .all(|x| x.is_finite())
        {
            return Err(Error::NonFiniteCommand);
        }
        let q = pack(self.q, -ranges.pmax, ranges.pmax, POSITION_BITS);
        let dq = pack(self.dq, -ranges.vmax, ranges.vmax, MIT_FIELD_BITS);
        let tau = pack(self.tau, -ranges.tmax, ranges.tmax, MIT_FIELD_BITS);
        let kp = pack(self.kp, 0., MIT_KP_MAX, MIT_FIELD_BITS);
        let kd = pack(self.kd, 0., MIT_KD_MAX, MIT_FIELD_BITS);
        let data = [
            (q >> 8) as u8,
            q as u8,
            (dq >> 4) as u8,
            ((dq & 15) << 4 | kp >> 8) as u8,
            kp as u8,
            (kd >> 4) as u8,
            ((kd & 15) << 4 | tau >> 8) as u8,
            tau as u8,
        ];
        if special_operation(&data).is_some() {
            return Err(Error::ReservedMotorOperation);
        }
        Ok(data)
    }
    /// Reads a motion command received by a motor in MIT mode.
    ///
    /// Requires eight bytes. Rejects other motor operations.
    pub fn decode(data: &[u8], ranges: MappingRanges) -> Result<Self, Error> {
        if data.len() != 8 {
            return Err(Error::InvalidLength);
        }
        if special_operation(data).is_some() {
            return Err(Error::ReservedMotorOperation);
        }
        ranges.validate()?;
        let b: [u32; 8] = std::array::from_fn(|i| data[i] as u32);
        Ok(Self {
            q: unpack(b[0] << 8 | b[1], -ranges.pmax, ranges.pmax, POSITION_BITS),
            dq: unpack(
                b[2] << 4 | b[3] >> 4,
                -ranges.vmax,
                ranges.vmax,
                MIT_FIELD_BITS,
            ),
            kp: unpack((b[3] & 15) << 8 | b[4], 0., MIT_KP_MAX, MIT_FIELD_BITS),
            kd: unpack(b[5] << 4 | b[6] >> 4, 0., MIT_KD_MAX, MIT_FIELD_BITS),
            tau: unpack(
                (b[6] & 15) << 8 | b[7],
                -ranges.tmax,
                ranges.tmax,
                MIT_FIELD_BITS,
            ),
        })
    }
}

fn command_float(value: f64) -> Result<[u8; 4], Error> {
    if !value.is_finite() {
        return Err(Error::NonFiniteCommand);
    }
    let value = value as f32;
    if !value.is_finite() {
        return Err(Error::CommandOutOfRange);
    }
    Ok(value.to_le_bytes())
}

fn read_command_float(bytes: &[u8]) -> Result<f64, Error> {
    let value = f32::from_le_bytes(bytes.try_into().map_err(|_| Error::InvalidLength)?);
    if !value.is_finite() {
        return Err(Error::NonFiniteCommand);
    }
    Ok(value.into())
}

/// Moves to a position using the motor's position controller.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct PositionVelocityCommand {
    /// Target position, in radians.
    pub q: f64,
    /// Maximum speed during the move, in radians/second.
    pub velocity_limit: f64,
}

impl PositionVelocityCommand {
    /// Prepares a position move to send to a motor. Values must fit finite float32.
    pub fn encode(self) -> Result<[u8; 8], Error> {
        let mut bytes = [0; 8];
        bytes[..4].copy_from_slice(&command_float(self.q)?);
        bytes[4..].copy_from_slice(&command_float(self.velocity_limit)?);
        Ok(bytes)
    }

    /// Reads a position move received by a motor. Requires eight bytes with finite values.
    pub fn decode(bytes: &[u8]) -> Result<Self, Error> {
        if bytes.len() != 8 {
            return Err(Error::InvalidLength);
        }
        Ok(Self {
            q: read_command_float(&bytes[..4])?,
            velocity_limit: read_command_float(&bytes[4..])?,
        })
    }
}

/// Runs the motor's velocity controller at a target speed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct VelocityCommand {
    /// Target velocity, in radians/second.
    pub dq: f64,
}

impl VelocityCommand {
    /// Prepares a velocity command to send to a motor. The target must fit finite float32.
    pub fn encode(self) -> Result<[u8; 4], Error> {
        command_float(self.dq)
    }

    /// Reads a velocity command received by a motor. Requires four bytes with a finite value.
    pub fn decode(bytes: &[u8]) -> Result<Self, Error> {
        Ok(Self {
            dq: read_command_float(bytes)?,
        })
    }
}

// Damiao force-position control-frame table: speed * 100 and current fraction * 10000.
const POSITION_FORCE_SPEED_SCALE: f64 = 100.;
const POSITION_FORCE_CURRENT_SCALE: f64 = 10000.;
const POSITION_FORCE_MAX_CODE: u16 = 10000;

/// Moves to a position with speed and current limits.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct PositionForceCommand {
    /// Target position, in radians.
    pub q: f64,
    /// Speed limit, in radians/second.
    pub velocity_limit: f64,
    /// Current limit as a fraction of the motor's maximum current.
    pub current_limit: f64,
}

impl PositionForceCommand {
    /// Prepares a current-limited position move. Position must fit finite float32;
    /// limits must be finite and are clamped to 0–100 rad/s and 0–1 of maximum current.
    pub fn encode(self) -> Result<[u8; 8], Error> {
        let mut bytes = [0; 8];
        bytes[..4].copy_from_slice(&command_float(self.q)?);
        if !self.velocity_limit.is_finite() || !self.current_limit.is_finite() {
            return Err(Error::NonFiniteCommand);
        }
        let speed = (self.velocity_limit * POSITION_FORCE_SPEED_SCALE)
            .clamp(0., f64::from(POSITION_FORCE_MAX_CODE)) as u16;
        let current = (self.current_limit.clamp(0., 1.) * POSITION_FORCE_CURRENT_SCALE) as u16;
        bytes[4..6].copy_from_slice(&speed.to_le_bytes());
        bytes[6..].copy_from_slice(&current.to_le_bytes());
        Ok(bytes)
    }

    /// Reads a current-limited position move. Requires eight bytes and a finite position.
    /// Limits above the protocol maximum are reduced to that maximum.
    pub fn decode(bytes: &[u8]) -> Result<Self, Error> {
        if bytes.len() != 8 {
            return Err(Error::InvalidLength);
        }
        Ok(Self {
            q: read_command_float(&bytes[..4])?,
            velocity_limit: f64::from(
                u16::from_le_bytes([bytes[4], bytes[5]]).min(POSITION_FORCE_MAX_CODE),
            ) / POSITION_FORCE_SPEED_SCALE,
            current_limit: f64::from(
                u16::from_le_bytes([bytes[6], bytes[7]]).min(POSITION_FORCE_MAX_CODE),
            ) / POSITION_FORCE_CURRENT_SCALE,
        })
    }
}

/// A request ready to pass to a CAN socket.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncodedRequest {
    pub id: u32,
    data: [u8; 8],
    len: usize,
}

impl EncodedRequest {
    fn new<const N: usize>(id: u32, data: [u8; N]) -> Self {
        let mut packet = Self {
            id,
            data: [0; 8],
            len: N,
        };
        packet.data[..N].copy_from_slice(&data);
        packet
    }

    pub fn data(&self) -> &[u8] {
        &self.data[..self.len]
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Request {
    Mit(MitCommand),
    PositionVelocity(PositionVelocityCommand),
    Velocity(VelocityCommand),
    PositionForce(PositionForceCommand),
    Enable(ControlMode),
    Disable(ControlMode),
    ClearError(ControlMode),
    SetZero(ControlMode),
    Feedback,
    ReadRegister(RegisterAddress),
    WriteRegister {
        register: RegisterAddress,
        /// New register value: little-endian f32 for PMAX/VMAX/TMAX,
        /// little-endian u32 for MST_ID/ESC_ID/TIMEOUT/CTRL_MODE.
        value: [u8; 4],
    },
}

fn validate_motor_id(motor_id: u16) -> Result<(), Error> {
    // 0x7ff is reserved for register requests.
    if u32::from(motor_id) >= REGISTER_CAN_ID {
        return Err(Error::InvalidMotorId);
    }
    Ok(())
}

impl Request {
    /// Prepares a control or configuration request to send to a motor.
    ///
    /// Requires the motor ID plus the command's mode offset to be in `0..=0x7fe`.
    /// Only MIT commands use `ranges`.
    pub fn encode(self, motor_id: u16, ranges: MappingRanges) -> Result<EncodedRequest, Error> {
        validate_motor_id(motor_id)?;
        let mut data = [0; 8];
        data[..2].copy_from_slice(&motor_id.to_le_bytes());
        match self {
            Self::Mit(command) => {
                return Ok(EncodedRequest::new(
                    motor_id.into(),
                    command.encode(ranges)?,
                ));
            }
            Self::PositionVelocity(command) => {
                return Ok(EncodedRequest::new(
                    ControlMode::PositionVelocity.command_id(motor_id)?,
                    command.encode()?,
                ));
            }
            Self::Velocity(command) => {
                return Ok(EncodedRequest::new(
                    ControlMode::Velocity.command_id(motor_id)?,
                    command.encode()?,
                ));
            }
            Self::PositionForce(command) => {
                return Ok(EncodedRequest::new(
                    ControlMode::PositionForce.command_id(motor_id)?,
                    command.encode()?,
                ));
            }
            Self::Enable(mode)
            | Self::Disable(mode)
            | Self::ClearError(mode)
            | Self::SetZero(mode) => {
                data.fill(0xff);
                data[7] = match self {
                    Self::Enable(_) => ENABLE,
                    Self::Disable(_) => DISABLE,
                    Self::SetZero(_) => SET_ZERO,
                    _ => CLEAR_ERROR,
                };
                return Ok(EncodedRequest::new(mode.command_id(motor_id)?, data));
            }
            Self::Feedback => data[2] = REFRESH_FEEDBACK,
            Self::ReadRegister(register) => {
                data[2] = READ_REGISTER;
                data[3] = register.0;
            }
            Self::WriteRegister { register, value } => {
                data[2] = WRITE_REGISTER;
                data[3] = register.0;
                data[4..].copy_from_slice(&value);
            }
        }
        Ok(EncodedRequest::new(REGISTER_CAN_ID, data))
    }
    /// Reads incoming control and configuration requests for a motor.
    ///
    /// Requires `motor_id` in `0..=0x7fe`.
    /// Velocity commands require four bytes; register reads accept four or eight;
    /// other requests require eight.
    /// Returns `None` for requests addressed elsewhere, or an error for malformed
    /// requests and unsupported operations.
    pub fn decode(
        id: u32,
        motor_id: u16,
        data: &[u8],
        ranges: MappingRanges,
    ) -> Result<Option<Self>, Error> {
        validate_motor_id(motor_id)?;
        if id == REGISTER_CAN_ID {
            if data.len() != 4 && data.len() != 8 {
                return Err(Error::InvalidLength);
            }
            if u16::from_le_bytes([data[0], data[1]]) != motor_id {
                return Ok(None);
            }
            return Ok(Some(match data[2] {
                REFRESH_FEEDBACK if data.len() == 8 => Self::Feedback,
                READ_REGISTER => Self::ReadRegister(RegisterAddress(data[3])),
                WRITE_REGISTER if data.len() == 8 => Self::WriteRegister {
                    register: RegisterAddress(data[3]),
                    value: data[4..].try_into().unwrap(),
                },
                REFRESH_FEEDBACK | WRITE_REGISTER => return Err(Error::InvalidLength),
                opcode => return Err(Error::UnsupportedRegisterOperation(opcode)),
            }));
        }
        let mode = match id.checked_sub(motor_id.into()) {
            Some(0) => ControlMode::Mit,
            Some(0x100) => ControlMode::PositionVelocity,
            Some(0x200) => ControlMode::Velocity,
            Some(0x300) => ControlMode::PositionForce,
            _ => return Ok(None),
        };
        mode.command_id(motor_id)?;
        Ok(Some(match special_operation(data) {
            Some(ENABLE) => Self::Enable(mode),
            Some(DISABLE) => Self::Disable(mode),
            Some(CLEAR_ERROR) => Self::ClearError(mode),
            Some(SET_ZERO) => Self::SetZero(mode),
            Some(_) => unreachable!(),
            None => match mode {
                ControlMode::Mit => Self::Mit(MitCommand::decode(data, ranges)?),
                ControlMode::PositionVelocity => {
                    Self::PositionVelocity(PositionVelocityCommand::decode(data)?)
                }
                ControlMode::Velocity => Self::Velocity(VelocityCommand::decode(data)?),
                ControlMode::PositionForce => {
                    Self::PositionForce(PositionForceCommand::decode(data)?)
                }
            },
        }))
    }
}

fn special_operation(data: &[u8]) -> Option<u8> {
    (data.len() == 8
        && data[..7] == [0xff; 7]
        && matches!(data[7], CLEAR_ERROR | ENABLE | DISABLE | SET_ZERO))
    .then(|| data[7])
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Feedback {
    /// Motor ID nibble reported in the payload, separate from the CAN frame ID.
    pub reported_id: u8,
    pub q: f64,
    pub dq: f64,
    pub torque: f64,
    pub status: MotorStatus,
    pub mos_temperature: u8,
    pub rotor_temperature: u8,
}
impl Feedback {
    /// Prepares a motor status reply to send to a controller.
    ///
    /// Measurements must be finite; `reported_id` and the status code must be in `0..=15`.
    /// Measurements outside the configured limits are reported at those limits.
    pub fn encode(self, ranges: MappingRanges) -> Result<[u8; 8], Error> {
        ranges.validate()?;
        if self.reported_id >= 16
            || self.status.0 >= 16
            || ![self.q, self.dq, self.torque].iter().all(|x| x.is_finite())
        {
            return Err(Error::InvalidFeedback);
        }
        let q = pack(self.q, -ranges.pmax, ranges.pmax, POSITION_BITS);
        let v = pack(self.dq, -ranges.vmax, ranges.vmax, MIT_FIELD_BITS);
        let t = pack(self.torque, -ranges.tmax, ranges.tmax, MIT_FIELD_BITS);
        Ok([
            self.reported_id | self.status.0 << 4,
            (q >> 8) as u8,
            q as u8,
            (v >> 4) as u8,
            ((v & 15) << 4 | t >> 8) as u8,
            t as u8,
            self.mos_temperature,
            self.rotor_temperature,
        ])
    }
    /// Reads a motor's reported state from a received reply.
    ///
    /// Requires an eight-byte motor feedback payload.
    pub fn decode(data: &[u8], ranges: MappingRanges) -> Result<Self, Error> {
        if data.len() != 8 {
            return Err(Error::InvalidLength);
        }
        ranges.validate()?;
        Ok(Self {
            reported_id: data[0] & 15,
            q: unpack(
                u16::from_be_bytes([data[1], data[2]]).into(),
                -ranges.pmax,
                ranges.pmax,
                POSITION_BITS,
            ),
            dq: unpack(
                (data[3] as u32) << 4 | (data[4] as u32) >> 4,
                -ranges.vmax,
                ranges.vmax,
                MIT_FIELD_BITS,
            ),
            torque: unpack(
                ((data[4] & 15) as u32) << 8 | data[5] as u32,
                -ranges.tmax,
                ranges.tmax,
                MIT_FIELD_BITS,
            ),
            status: MotorStatus(data[0] >> 4),
            mos_temperature: data[6],
            rotor_temperature: data[7],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn position_velocity_command_matches_wire_bytes_in_both_directions() {
        let command = PositionVelocityCommand {
            q: -1.25,
            velocity_limit: 12.5,
        };
        let bytes = [0x00, 0x00, 0xa0, 0xbf, 0x00, 0x00, 0x48, 0x41];
        assert_eq!(command.encode(), Ok(bytes));
        assert_eq!(PositionVelocityCommand::decode(&bytes), Ok(command));
        let request = Request::PositionVelocity(command);
        let packet = request.encode(7, DM4310_DEFAULT_MAPPING_RANGES).unwrap();
        assert_eq!(packet.id, 0x107);
        assert_eq!(packet.data(), bytes);
        assert_eq!(
            Request::decode(0x107, 7, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
            Ok(Some(request))
        );
    }

    #[test]
    fn velocity_command_has_a_four_byte_payload_in_both_directions() {
        let command = VelocityCommand { dq: -2.5 };
        let bytes = [0x00, 0x00, 0x20, 0xc0];
        assert_eq!(command.encode(), Ok(bytes));
        assert_eq!(VelocityCommand::decode(&bytes), Ok(command));
        let request = Request::Velocity(command);
        let packet = request.encode(7, DM4310_DEFAULT_MAPPING_RANGES).unwrap();
        assert_eq!(packet.id, 0x207);
        assert_eq!(packet.data(), bytes);
        assert_eq!(
            Request::decode(0x207, 7, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
            Ok(Some(request))
        );
    }

    #[test]
    fn position_force_command_matches_wire_bytes_in_both_directions() {
        let command = PositionForceCommand {
            q: -1.25,
            velocity_limit: 12.5,
            current_limit: 0.25,
        };
        let bytes = [0x00, 0x00, 0xa0, 0xbf, 0xe2, 0x04, 0xc4, 0x09];
        assert_eq!(command.encode(), Ok(bytes));
        assert_eq!(PositionForceCommand::decode(&bytes), Ok(command));
        let request = Request::PositionForce(command);
        let packet = request.encode(7, DM4310_DEFAULT_MAPPING_RANGES).unwrap();
        assert_eq!(packet.id, 0x307);
        assert_eq!(packet.data(), bytes);
        assert_eq!(
            Request::decode(0x307, 7, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
            Ok(Some(request))
        );
    }

    #[test]
    fn position_force_limits_clamp_and_truncate() {
        for (speed, current, limits) in [
            (-1., -1., [0, 0, 0, 0]),
            (100., 1., [0x10, 0x27, 0x10, 0x27]),
            (f64::MAX, f64::MAX, [0x10, 0x27, 0x10, 0x27]),
            (1.234, 0.12345, [0x7b, 0x00, 0xd2, 0x04]),
        ] {
            let command = PositionForceCommand {
                q: 0.,
                velocity_limit: speed,
                current_limit: current,
            };
            assert_eq!(
                command.encode().unwrap(),
                [0, 0, 0, 0, limits[0], limits[1], limits[2], limits[3]]
            );
        }
        assert_eq!(
            PositionForceCommand::decode(&[0, 0, 0, 0, 255, 255, 255, 255]),
            Ok(PositionForceCommand {
                q: 0.,
                velocity_limit: 100.,
                current_limit: 1.
            })
        );
    }

    #[test]
    fn float_commands_reject_nonfinite_and_unrepresentable_values() {
        for (value, error) in [
            (f64::NAN, Error::NonFiniteCommand),
            (f64::INFINITY, Error::NonFiniteCommand),
            (f64::NEG_INFINITY, Error::NonFiniteCommand),
            (f64::MAX, Error::CommandOutOfRange),
            (-f64::MAX, Error::CommandOutOfRange),
        ] {
            assert_eq!(VelocityCommand { dq: value }.encode(), Err(error));
            assert_eq!(
                PositionVelocityCommand {
                    q: value,
                    velocity_limit: 1.
                }
                .encode(),
                Err(error)
            );
            assert_eq!(
                PositionVelocityCommand {
                    q: 0.,
                    velocity_limit: value
                }
                .encode(),
                Err(error)
            );
            assert_eq!(
                PositionForceCommand {
                    q: value,
                    ..PositionForceCommand::default()
                }
                .encode(),
                Err(error)
            );
            if !value.is_finite() {
                for command in [
                    PositionForceCommand {
                        velocity_limit: value,
                        ..PositionForceCommand::default()
                    },
                    PositionForceCommand {
                        current_limit: value,
                        ..PositionForceCommand::default()
                    },
                ] {
                    assert_eq!(command.encode(), Err(error));
                }
            }
        }
        for bytes in [[0, 0, 0x80, 0x7f], [0, 0, 0x80, 0xff], [0, 0, 0xc0, 0x7f]] {
            assert_eq!(
                VelocityCommand::decode(&bytes),
                Err(Error::NonFiniteCommand)
            );
            let mut position = [0; 8];
            position[..4].copy_from_slice(&bytes);
            assert_eq!(
                PositionVelocityCommand::decode(&position),
                Err(Error::NonFiniteCommand)
            );
            assert_eq!(
                PositionForceCommand::decode(&position),
                Err(Error::NonFiniteCommand)
            );
            position.rotate_left(4);
            assert_eq!(
                PositionVelocityCommand::decode(&position),
                Err(Error::NonFiniteCommand)
            );
        }
    }

    #[test]
    fn float_command_round_trips_preserve_float32_extremes_and_signed_zero() {
        for value in [
            f32::MAX,
            f32::MIN,
            f32::MIN_POSITIVE,
            f32::from_bits(1),
            -0.0,
            0.0,
        ] {
            let command = VelocityCommand {
                dq: f64::from(value),
            };
            let decoded = VelocityCommand::decode(&command.encode().unwrap()).unwrap();
            assert_eq!(decoded.dq.to_bits(), command.dq.to_bits());
        }
    }

    #[test]
    fn mode_specific_decoders_require_the_exact_payload_length() {
        let ranges = DM4310_DEFAULT_MAPPING_RANGES;
        for length in 0..=9 {
            let bytes = &[0; 9][..length];
            if length != 4 {
                assert_eq!(VelocityCommand::decode(bytes), Err(Error::InvalidLength));
                assert_eq!(
                    Request::decode(0x207, 7, bytes, ranges),
                    Err(Error::InvalidLength)
                );
            }
            if length != 8 {
                assert_eq!(
                    PositionVelocityCommand::decode(bytes),
                    Err(Error::InvalidLength)
                );
                assert_eq!(
                    PositionForceCommand::decode(bytes),
                    Err(Error::InvalidLength)
                );
                for id in [0x107, 0x307] {
                    assert_eq!(
                        Request::decode(id, 7, bytes, ranges),
                        Err(Error::InvalidLength)
                    );
                }
            }
        }
    }

    #[test]
    fn non_mit_commands_do_not_depend_on_mapping_ranges() {
        let invalid_ranges = MappingRanges {
            pmax: f64::NAN,
            vmax: 0.,
            tmax: -1.,
        };
        for request in [
            Request::PositionVelocity(PositionVelocityCommand::default()),
            Request::Velocity(VelocityCommand::default()),
            Request::PositionForce(PositionForceCommand::default()),
        ] {
            let packet = request.encode(7, invalid_ranges).unwrap();
            assert_eq!(
                Request::decode(packet.id, 7, packet.data(), invalid_ranges),
                Ok(Some(request))
            );
        }
    }

    #[test]
    fn special_requests_use_each_modes_address_and_eight_byte_payload() {
        for (mode, id) in [
            (ControlMode::Mit, 0x123),
            (ControlMode::PositionVelocity, 0x223),
            (ControlMode::Velocity, 0x323),
            (ControlMode::PositionForce, 0x423),
        ] {
            for (request, opcode) in [
                (Request::Enable(mode), 0xfc),
                (Request::Disable(mode), 0xfd),
                (Request::ClearError(mode), 0xfb),
                (Request::SetZero(mode), 0xfe),
            ] {
                let bytes = [255, 255, 255, 255, 255, 255, 255, opcode];
                let packet = request
                    .encode(0x123, DM4310_DEFAULT_MAPPING_RANGES)
                    .unwrap();
                assert_eq!(packet.id, id);
                assert_eq!(packet.data(), bytes);
                assert_eq!(
                    Request::decode(id, 0x123, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
                    Ok(Some(request))
                );
            }
        }
    }

    #[test]
    fn mode_offsets_cannot_produce_reserved_or_extended_can_ids() {
        for (mode, max_motor_id, command) in [
            (ControlMode::Mit, 0x7fe, Request::Mit(MitCommand::default())),
            (
                ControlMode::PositionVelocity,
                0x6fe,
                Request::PositionVelocity(PositionVelocityCommand::default()),
            ),
            (
                ControlMode::Velocity,
                0x5fe,
                Request::Velocity(VelocityCommand::default()),
            ),
            (
                ControlMode::PositionForce,
                0x4fe,
                Request::PositionForce(PositionForceCommand::default()),
            ),
        ] {
            assert_eq!(mode.command_id(max_motor_id), Ok(0x7fe));
            let packet = command
                .encode(max_motor_id, DM4310_DEFAULT_MAPPING_RANGES)
                .unwrap();
            assert_eq!(packet.id, 0x7fe);
            assert!(
                Request::decode(
                    packet.id,
                    max_motor_id,
                    packet.data(),
                    DM4310_DEFAULT_MAPPING_RANGES
                )
                .unwrap()
                .is_some()
            );
            for motor_id in [max_motor_id + 1, max_motor_id + 2, u16::MAX] {
                assert_eq!(mode.command_id(motor_id), Err(Error::InvalidMotorId));
                assert_eq!(
                    command.encode(motor_id, DM4310_DEFAULT_MAPPING_RANGES),
                    Err(Error::InvalidMotorId)
                );
            }
            assert_eq!(
                Request::decode(
                    0x800,
                    max_motor_id + 2,
                    &[0; 8],
                    DM4310_DEFAULT_MAPPING_RANGES
                ),
                Err(Error::InvalidMotorId)
            );
        }
    }

    #[test]
    fn control_modes_use_documented_register_values() {
        for (value, mode) in [
            (1, ControlMode::Mit),
            (2, ControlMode::PositionVelocity),
            (3, ControlMode::Velocity),
            (4, ControlMode::PositionForce),
        ] {
            assert_eq!(ControlMode::try_from(value), Ok(mode));
            let packet = Request::WriteRegister {
                register: RegisterAddress::CTRL_MODE,
                value: (mode as u32).to_le_bytes(),
            }
            .encode(7, DM4310_DEFAULT_MAPPING_RANGES)
            .unwrap();
            assert_eq!(packet.id, 0x7ff);
            assert_eq!(packet.data(), [7, 0, 0x55, 10, value as u8, 0, 0, 0]);
        }
        for value in [0, 5, u32::MAX] {
            assert_eq!(
                ControlMode::try_from(value),
                Err(Error::UnknownControlMode(value))
            );
        }
    }

    #[test]
    fn motor_status_patterns_and_feedback_preserve_known_and_unknown_codes() {
        assert_eq!(std::mem::size_of::<MotorStatus>(), 1);
        assert_eq!(std::mem::align_of::<MotorStatus>(), 1);
        for raw in 0..=15 {
            let bytes = [raw << 4 | 7, 0, 0, 0, 0, 0, 0, 0];
            let feedback = Feedback::decode(&bytes, DM4310_DEFAULT_MAPPING_RANGES).unwrap();
            let matched = match feedback.status {
                MotorStatus::DISABLED => 0x0,
                MotorStatus::ENABLED => 0x1,
                MotorStatus::OVERVOLTAGE => 0x8,
                MotorStatus::UNDERVOLTAGE => 0x9,
                MotorStatus::OVERCURRENT => 0xa,
                MotorStatus::MOS_OVERHEAT => 0xb,
                MotorStatus::COIL_OVERHEAT => 0xc,
                MotorStatus::COMMUNICATION_LOST => 0xd,
                MotorStatus::OVERLOAD => 0xe,
                MotorStatus(unknown) => {
                    assert!(matches!(unknown, 2..=7 | 15));
                    unknown
                }
            };
            assert_eq!(matched, raw);
            assert_eq!(feedback.status.0, raw);
            assert_eq!(feedback.encode(DM4310_DEFAULT_MAPPING_RANGES), Ok(bytes));
        }
    }

    #[test]
    fn command_encode_matches_upstream_for_each_motor_preset() {
        // tools/generate_vectors.cpp reproduces these bytes with the upstream C++ codec.
        let command = MitCommand {
            kp: 123.4,
            kd: 1.2,
            q: 1.23,
            dq: -0.7,
            tau: 0.9,
        };
        for (ranges, bytes) in [
            (
                DM8009_DEFAULT_MAPPING_RANGES,
                [0x8c, 0x97, 0x7d, 0xf3, 0xf2, 0x3d, 0x68, 0x21],
            ),
            (
                DM4340_DEFAULT_MAPPING_RANGES,
                [0x8c, 0x97, 0x77, 0x03, 0xf2, 0x3d, 0x68, 0x41],
            ),
            (
                DM4310_DEFAULT_MAPPING_RANGES,
                [0x8c, 0x97, 0x7c, 0xf3, 0xf2, 0x3d, 0x68, 0xb7],
            ),
        ] {
            assert_eq!(command.encode(ranges), Ok(bytes), "{ranges:?}");
        }
    }

    #[test]
    fn command_decode_extracts_big_endian_position_and_shared_nibbles() {
        // Raw codes: q=0x1234, dq=0x567, kp=0x89a, kd=0xbcd, tau=0xef0.
        let bytes = [0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0];
        let command = MitCommand::decode(&bytes, DM4310_DEFAULT_MAPPING_RANGES).unwrap();
        // Tolerance covers floating-point arithmetic, not a whole wire-code step.
        assert!((command.q - -10.722323949034866).abs() < 1e-12);
        assert!((command.dq - -9.736263736263734).abs() < 1e-12);
        assert!((command.kp - 268.86446886446885).abs() < 1e-12);
        assert!((command.kd - 3.688644688644689).abs() < 1e-12);
        assert!((command.tau - 8.676434676434677).abs() < 1e-12);
    }

    #[test]
    fn command_encode_clamps_each_field_to_its_limits() {
        let ranges = DM4310_DEFAULT_MAPPING_RANGES;
        for (factor, bytes) in [(-2., [0; 8]), (2., [0xff; 8])] {
            let command = MitCommand {
                q: factor * ranges.pmax,
                dq: factor * ranges.vmax,
                tau: factor * ranges.tmax,
                kp: factor * MIT_KP_MAX,
                kd: factor * MIT_KD_MAX,
            };
            assert_eq!(command.encode(ranges), Ok(bytes));
        }
    }

    #[test]
    fn command_uses_custom_mapping_ranges() {
        // Settings a caller could read from PMAX/VMAX/TMAX, not a motor preset.
        let ranges = MappingRanges {
            pmax: 7.,
            vmax: 19.,
            tmax: 3.,
        };
        let command = MitCommand {
            q: -ranges.pmax,
            dq: ranges.vmax,
            tau: -ranges.tmax,
            ..MitCommand::default()
        };
        let bytes = [0x00, 0x00, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00];
        assert_eq!(command.encode(ranges), Ok(bytes));
        assert_eq!(MitCommand::decode(&bytes, ranges), Ok(command));
    }

    #[test]
    fn zero_command_truncates_signed_fields_to_the_lower_middle_code() {
        // Zero maps to 32767.5 (16 bits) or 2047.5 (12 bits). Truncate, don't round:
        // changing this changes the bytes sent to real motors, even for zero targets.
        assert_eq!(
            MitCommand::default().encode(DM4310_DEFAULT_MAPPING_RANGES),
            Ok([0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x00, 0x07, 0xff])
        );
    }

    #[test]
    fn decoded_zero_targets_are_half_a_step_below_zero() {
        let ranges = DM4310_DEFAULT_MAPPING_RANGES;
        let bytes = [0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x00, 0x07, 0xff];
        let command = MitCommand::decode(&bytes, ranges).unwrap();
        // Symmetric signed ranges have no exact zero code. These tiny negative
        // values are expected resolution loss, not a sign or offset bug.
        assert!((command.q + ranges.pmax / 65535.).abs() < 1e-12);
        assert!((command.dq + ranges.vmax / 4095.).abs() < 1e-12);
        assert!((command.tau + ranges.tmax / 4095.).abs() < 1e-12);
    }

    #[test]
    fn command_quantization_matches_upstream_at_a_code_boundary() {
        // This kp encodes as 65 in C++. Multiplying by 4095 before dividing by
        // MIT_KP_MAX instead yields 64 due to roundoff. Keep wire compatibility.
        let command = MitCommand {
            kp: 7.936507936507936,
            ..MitCommand::default()
        };
        assert_eq!(
            command.encode(DM4310_DEFAULT_MAPPING_RANGES),
            Ok([0x7f, 0xff, 0x7f, 0xf0, 0x41, 0x00, 0x07, 0xff])
        );
    }

    #[test]
    fn command_encode_rejects_nonfinite_values_in_every_field() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for command in [
                MitCommand {
                    q: value,
                    ..MitCommand::default()
                },
                MitCommand {
                    dq: value,
                    ..MitCommand::default()
                },
                MitCommand {
                    tau: value,
                    ..MitCommand::default()
                },
                MitCommand {
                    kp: value,
                    ..MitCommand::default()
                },
                MitCommand {
                    kd: value,
                    ..MitCommand::default()
                },
            ] {
                assert_eq!(
                    command.encode(DM4310_DEFAULT_MAPPING_RANGES),
                    Err(Error::NonFiniteCommand),
                    "{command:?}"
                );
            }
        }
    }

    #[test]
    fn feedback_encode_packs_measurements_and_metadata() {
        let feedback = Feedback {
            reported_id: 7,
            q: 1.23,
            dq: -0.7,
            torque: 0.9,
            status: MotorStatus::OVERCURRENT,
            mos_temperature: 0,
            rotor_temperature: 255,
        };
        // Motor ID 7 and status 10 occupy separate nibbles.
        assert_eq!(
            feedback.encode(DM4310_DEFAULT_MAPPING_RANGES),
            Ok([0xa7, 0x8c, 0x97, 0x7c, 0xf8, 0xb7, 0x00, 0xff])
        );
    }

    #[test]
    fn feedback_decode_extracts_measurements_and_metadata() {
        // Distinct bytes/nibbles expose field overlap and byte-order mistakes.
        // Physical values can also be reproduced with tools/generate_vectors.cpp.
        let bytes = [0xf7, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xff, 0x00];
        let feedback = Feedback::decode(&bytes, DM4310_DEFAULT_MAPPING_RANGES).unwrap();
        assert!((feedback.q - -10.722323949034866).abs() < 1e-12);
        assert!((feedback.dq - -9.736263736263734).abs() < 1e-12);
        assert!((feedback.torque - 0.7545787545787537).abs() < 1e-12);
        // Preserve even unknown status 15; the low ID nibble is not the status.
        assert_eq!(feedback.status, MotorStatus(15));
        assert_eq!(feedback.reported_id, 7);
        assert_eq!(
            (feedback.mos_temperature, feedback.rotor_temperature),
            (255, 0)
        );
    }

    #[test]
    fn feedback_keeps_reported_id_and_status_separate() {
        for (first_byte, reported_id, status) in [
            (0xf0, 0, MotorStatus(15)),
            (0x0f, 15, MotorStatus::DISABLED),
            (0xa7, 7, MotorStatus::OVERCURRENT),
        ] {
            let bytes = [first_byte, 0, 0, 0, 0, 0, 0, 0];
            let feedback = Feedback::decode(&bytes, DM4310_DEFAULT_MAPPING_RANGES).unwrap();
            assert_eq!(feedback.reported_id, reported_id);
            assert_eq!(feedback.status, status);
            assert_eq!(feedback.encode(DM4310_DEFAULT_MAPPING_RANGES), Ok(bytes));
        }
    }

    #[test]
    fn feedback_encode_clamps_each_measurement_to_its_limits() {
        let ranges = DM4310_DEFAULT_MAPPING_RANGES;
        for (factor, bytes) in [
            (-2., [0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            (2., [0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00]),
        ] {
            let feedback = Feedback {
                reported_id: 7,
                q: factor * ranges.pmax,
                dq: factor * ranges.vmax,
                torque: factor * ranges.tmax,
                status: MotorStatus::DISABLED,
                mos_temperature: 0,
                rotor_temperature: 0,
            };
            assert_eq!(feedback.encode(ranges), Ok(bytes));
        }
    }

    #[test]
    fn feedback_uses_custom_mapping_ranges() {
        let ranges = MappingRanges {
            pmax: 7.,
            vmax: 19.,
            tmax: 3.,
        };
        let feedback = Feedback {
            reported_id: 7,
            q: ranges.pmax,
            dq: -ranges.vmax,
            torque: ranges.tmax,
            status: MotorStatus::DISABLED,
            mos_temperature: 0,
            rotor_temperature: 0,
        };
        let bytes = [0x07, 0xff, 0xff, 0x00, 0x0f, 0xff, 0x00, 0x00];
        assert_eq!(feedback.encode(ranges), Ok(bytes));
        assert_eq!(Feedback::decode(&bytes, ranges), Ok(feedback));
    }

    #[test]
    fn feedback_encode_rejects_nonfinite_measurements() {
        let valid = Feedback {
            reported_id: 7,
            q: 0.,
            dq: 0.,
            torque: 0.,
            status: MotorStatus::DISABLED,
            mos_temperature: 0,
            rotor_temperature: 0,
        };
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for feedback in [
                Feedback { q: value, ..valid },
                Feedback { dq: value, ..valid },
                Feedback {
                    torque: value,
                    ..valid
                },
            ] {
                assert_eq!(
                    feedback.encode(DM4310_DEFAULT_MAPPING_RANGES),
                    Err(Error::InvalidFeedback),
                    "{feedback:?}"
                );
            }
        }
    }

    #[test]
    fn feedback_encode_rejects_status_that_does_not_fit_in_a_nibble() {
        let feedback = Feedback {
            reported_id: 7,
            q: 0.,
            dq: 0.,
            torque: 0.,
            status: MotorStatus(16),
            mos_temperature: 0,
            rotor_temperature: 0,
        };
        assert_eq!(
            feedback.encode(DM4310_DEFAULT_MAPPING_RANGES),
            Err(Error::InvalidFeedback)
        );
    }

    #[test]
    fn feedback_encode_rejects_reported_ids_that_would_overlap_status() {
        for reported_id in [16, 0x47, u8::MAX] {
            let feedback = Feedback {
                reported_id,
                q: 0.,
                dq: 0.,
                torque: 0.,
                status: MotorStatus::OVERCURRENT,
                mos_temperature: 0,
                rotor_temperature: 0,
            };
            assert_eq!(
                feedback.encode(DM4310_DEFAULT_MAPPING_RANGES),
                Err(Error::InvalidFeedback)
            );
        }
    }

    #[test]
    fn mapping_ranges_require_positive_finite_spans_in_every_field() {
        let valid = DM4310_DEFAULT_MAPPING_RANGES;
        // f64::MAX itself is finite, but the signed span (-max..max) overflows.
        for value in [
            0.,
            -1.,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::MAX,
        ] {
            for ranges in [
                MappingRanges {
                    pmax: value,
                    ..valid
                },
                MappingRanges {
                    vmax: value,
                    ..valid
                },
                MappingRanges {
                    tmax: value,
                    ..valid
                },
            ] {
                assert_eq!(ranges.validate(), Err(Error::InvalidRanges), "{ranges:?}");
            }
        }
    }

    #[test]
    fn all_command_and_feedback_codecs_validate_mapping_ranges() {
        let ranges = MappingRanges {
            tmax: 0.,
            ..DM4310_DEFAULT_MAPPING_RANGES
        };
        let feedback = Feedback {
            reported_id: 7,
            q: 0.,
            dq: 0.,
            torque: 0.,
            status: MotorStatus::DISABLED,
            mos_temperature: 0,
            rotor_temperature: 0,
        };
        assert_eq!(
            MitCommand::default().encode(ranges),
            Err(Error::InvalidRanges)
        );
        assert_eq!(
            MitCommand::decode(&[0; 8], ranges),
            Err(Error::InvalidRanges)
        );
        assert_eq!(feedback.encode(ranges), Err(Error::InvalidRanges));
        assert_eq!(Feedback::decode(&[0; 8], ranges), Err(Error::InvalidRanges));
    }

    #[test]
    fn motion_and_feedback_decoders_require_exactly_eight_bytes() {
        let ranges = DM4310_DEFAULT_MAPPING_RANGES;
        for len in [0, 7, 9] {
            let bytes = &[0; 9][..len];
            assert_eq!(MitCommand::decode(bytes, ranges), Err(Error::InvalidLength));
            assert_eq!(Feedback::decode(bytes, ranges), Err(Error::InvalidLength));
            assert_eq!(
                Request::decode(7, 7, bytes, ranges),
                Err(Error::InvalidLength)
            );
        }
    }

    #[test]
    fn mit_request_encode_uses_the_motor_can_id() {
        let ranges = DM4310_DEFAULT_MAPPING_RANGES;
        let command = MitCommand {
            q: -ranges.pmax,
            dq: -ranges.vmax,
            tau: -ranges.tmax,
            ..MitCommand::default()
        };
        assert_eq!(
            Request::Mit(command).encode(7, ranges),
            Ok(EncodedRequest::new(7, [0; 8]))
        );
    }

    #[test]
    fn mit_request_decode_returns_the_motion_command() {
        let ranges = DM4310_DEFAULT_MAPPING_RANGES;
        let command = MitCommand {
            q: -ranges.pmax,
            dq: -ranges.vmax,
            tau: -ranges.tmax,
            ..MitCommand::default()
        };
        assert_eq!(
            Request::decode(7, 7, &[0; 8], ranges),
            Ok(Some(Request::Mit(command)))
        );
    }

    #[test]
    fn special_request_encode_uses_seven_ff_bytes_and_an_opcode() {
        for (request, opcode) in [
            (Request::Enable(ControlMode::Mit), 0xfc),
            (Request::Disable(ControlMode::Mit), 0xfd),
            (Request::ClearError(ControlMode::Mit), 0xfb),
        ] {
            assert_eq!(
                request.encode(7, DM4310_DEFAULT_MAPPING_RANGES),
                Ok(EncodedRequest::new(
                    7,
                    [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, opcode]
                ))
            );
        }
    }

    #[test]
    fn special_request_decode_recognizes_supported_opcodes() {
        for (opcode, request) in [
            (0xfc, Request::Enable(ControlMode::Mit)),
            (0xfd, Request::Disable(ControlMode::Mit)),
            (0xfb, Request::ClearError(ControlMode::Mit)),
        ] {
            let bytes = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, opcode];
            assert_eq!(
                Request::decode(7, 7, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
                Ok(Some(request))
            );
        }
    }

    #[test]
    fn set_zero_request_decode_recognizes_the_opcode() {
        let bytes = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe];
        assert_eq!(
            Request::decode(7, 7, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
            Ok(Some(Request::SetZero(ControlMode::Mit)))
        );
    }

    #[test]
    fn special_operation_requires_all_seven_prefix_bytes_to_be_ff() {
        for index in 0..7 {
            let mut bytes = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc];
            bytes[index] = 0xfe;
            assert!(MitCommand::decode(&bytes, DM4310_DEFAULT_MAPPING_RANGES).is_ok());
            assert!(
                matches!(
                    Request::decode(7, 7, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
                    Ok(Some(Request::Mit(_)))
                ),
                "prefix byte {index}"
            );
        }
    }

    #[test]
    fn command_and_request_encode_reject_commands_that_become_motor_operations() {
        let ranges = DM4310_DEFAULT_MAPPING_RANGES;
        for opcode in [0xfbu8, 0xfc, 0xfd, 0xfe] {
            // Seven ff bytes plus one of these torque tails means a motor operation.
            // Use the middle of that torque bin so roundoff cannot change its code.
            let command = MitCommand {
                q: ranges.pmax,
                dq: ranges.vmax,
                kp: MIT_KP_MAX,
                kd: MIT_KD_MAX,
                tau: (f64::from(0xf00 + u16::from(opcode)) + 0.5) / 4095. * (2. * ranges.tmax)
                    - ranges.tmax,
            };
            assert_eq!(command.encode(ranges), Err(Error::ReservedMotorOperation));
            assert_eq!(
                Request::Mit(command).encode(7, ranges),
                Err(Error::ReservedMotorOperation)
            );
        }
    }

    #[test]
    fn command_decode_rejects_motor_operations() {
        for opcode in [0xfb, 0xfc, 0xfd, 0xfe] {
            let bytes = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, opcode];
            assert_eq!(
                MitCommand::decode(&bytes, DM4310_DEFAULT_MAPPING_RANGES),
                Err(Error::ReservedMotorOperation)
            );
        }
    }

    #[test]
    fn all_ff_is_a_valid_maximum_motion_command() {
        let ranges = DM4310_DEFAULT_MAPPING_RANGES;
        let command = MitCommand {
            q: ranges.pmax,
            dq: ranges.vmax,
            tau: ranges.tmax,
            kp: MIT_KP_MAX,
            kd: MIT_KD_MAX,
        };
        assert_eq!(command.encode(ranges), Ok([0xff; 8]));
        assert_eq!(MitCommand::decode(&[0xff; 8], ranges), Ok(command));
        assert_eq!(
            Request::Mit(command).encode(7, ranges),
            Ok(EncodedRequest::new(7, [0xff; 8]))
        );
        assert_eq!(
            Request::decode(7, 7, &[0xff; 8], ranges),
            Ok(Some(Request::Mit(command)))
        );
    }

    #[test]
    fn feedback_request_encode_uses_the_register_can_id() {
        assert_eq!(
            Request::Feedback.encode(0x123, DM4310_DEFAULT_MAPPING_RANGES),
            Ok(EncodedRequest::new(
                0x7ff,
                [0x23, 0x01, 0xcc, 0, 0, 0, 0, 0]
            ))
        );
    }

    #[test]
    fn feedback_request_decode_reads_a_little_endian_motor_id() {
        let bytes = [0x23, 0x01, 0xcc, 0, 0, 0, 0, 0];
        assert_eq!(
            Request::decode(0x7ff, 0x123, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
            Ok(Some(Request::Feedback))
        );
    }

    #[test]
    fn read_register_encode_includes_the_register_and_zero_padding() {
        assert_eq!(
            Request::ReadRegister(RegisterAddress::PMAX)
                .encode(0x123, DM4310_DEFAULT_MAPPING_RANGES),
            Ok(EncodedRequest::new(
                0x7ff,
                [0x23, 0x01, 0x33, 21, 0, 0, 0, 0]
            ))
        );
    }

    #[test]
    fn read_register_decode_accepts_four_or_eight_bytes() {
        let bytes = [0x23, 0x01, 0x33, 21, 0, 0, 0, 0];
        for len in [4, 8] {
            assert_eq!(
                Request::decode(0x7ff, 0x123, &bytes[..len], DM4310_DEFAULT_MAPPING_RANGES),
                Ok(Some(Request::ReadRegister(RegisterAddress::PMAX)))
            );
        }
    }

    #[test]
    fn write_register_encode_preserves_the_four_value_bytes() {
        // Register values are opaque to the codec; their numeric type is the caller's concern.
        let request = Request::WriteRegister {
            register: RegisterAddress::TMAX,
            value: [0x12, 0x34, 0x56, 0x78],
        };
        assert_eq!(
            request.encode(0x123, DM4310_DEFAULT_MAPPING_RANGES),
            Ok(EncodedRequest::new(
                0x7ff,
                [0x23, 0x01, 0x55, 23, 0x12, 0x34, 0x56, 0x78]
            ))
        );
    }

    #[test]
    fn write_register_decode_preserves_the_four_value_bytes() {
        let bytes = [0x23, 0x01, 0x55, 23, 0x12, 0x34, 0x56, 0x78];
        assert_eq!(
            Request::decode(0x7ff, 0x123, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
            Ok(Some(Request::WriteRegister {
                register: RegisterAddress::TMAX,
                value: [0x12, 0x34, 0x56, 0x78]
            }))
        );
    }

    #[test]
    fn register_requests_preserve_unknown_addresses() {
        for address in [0, 255] {
            for (request, bytes) in [
                (
                    Request::ReadRegister(RegisterAddress(address)),
                    [7, 0, 0x33, address, 0, 0, 0, 0],
                ),
                (
                    Request::WriteRegister {
                        register: RegisterAddress(address),
                        value: [0x12, 0x34, 0x56, 0x78],
                    },
                    [7, 0, 0x55, address, 0x12, 0x34, 0x56, 0x78],
                ),
            ] {
                assert_eq!(
                    request.encode(7, DM4310_DEFAULT_MAPPING_RANGES),
                    Ok(EncodedRequest::new(0x7ff, bytes))
                );
                assert_eq!(
                    Request::decode(0x7ff, 7, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
                    Ok(Some(request))
                );
            }
        }
    }

    #[test]
    fn request_codecs_accept_the_motor_id_endpoints() {
        for motor_id in [0, 0x7fe] {
            assert_eq!(
                Request::Enable(ControlMode::Mit)
                    .encode(motor_id, DM4310_DEFAULT_MAPPING_RANGES)
                    .unwrap()
                    .id,
                u32::from(motor_id)
            );
            assert_eq!(
                Request::decode(
                    u32::from(motor_id),
                    motor_id,
                    &[0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc],
                    DM4310_DEFAULT_MAPPING_RANGES,
                ),
                Ok(Some(Request::Enable(ControlMode::Mit)))
            );
        }
    }

    #[test]
    fn request_codecs_reject_reserved_or_nonstandard_motor_ids() {
        // 0x7ff is reserved for register traffic; larger IDs do not fit standard CAN.
        for motor_id in [0x7ff, 0x800, u16::MAX] {
            assert_eq!(
                Request::Enable(ControlMode::Mit).encode(motor_id, DM4310_DEFAULT_MAPPING_RANGES),
                Err(Error::InvalidMotorId)
            );
            // Invalid configuration is an error even when a frame is addressed elsewhere.
            for frame_id in [0, 0x7ff, u32::from(motor_id)] {
                assert_eq!(
                    Request::decode(frame_id, motor_id, &[0; 8], DM4310_DEFAULT_MAPPING_RANGES),
                    Err(Error::InvalidMotorId)
                );
            }
        }
    }

    #[test]
    fn request_decode_ignores_other_can_ids_before_parsing_payloads() {
        assert_eq!(
            Request::decode(8, 7, &[], DM4310_DEFAULT_MAPPING_RANGES),
            Ok(None)
        );
    }

    #[test]
    fn register_request_decode_ignores_other_motor_ids() {
        // Same low byte, different high byte. Even the unknown opcode is irrelevant.
        let bytes = [0x23, 0x02, 0xaa, 0];
        assert_eq!(
            Request::decode(0x7ff, 0x123, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
            Ok(None)
        );
    }

    #[test]
    fn register_request_decode_rejects_lengths_other_than_four_or_eight() {
        for len in [0, 3, 5, 7, 9] {
            let bytes = [7, 0, 0x33, 21, 0, 0, 0, 0, 0];
            assert_eq!(
                Request::decode(0x7ff, 7, &bytes[..len], DM4310_DEFAULT_MAPPING_RANGES),
                Err(Error::InvalidLength)
            );
        }
    }

    #[test]
    fn write_and_feedback_requests_require_eight_bytes() {
        // Only register reads have the four-byte exception.
        for opcode in [0x55, 0xcc] {
            let bytes = [7, 0, opcode, 21];
            assert_eq!(
                Request::decode(0x7ff, 7, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
                Err(Error::InvalidLength)
            );
        }
    }

    #[test]
    fn unknown_register_opcode_is_returned_in_the_error() {
        let bytes = [7, 0, 0xaa, 0];
        assert_eq!(
            Request::decode(0x7ff, 7, &bytes, DM4310_DEFAULT_MAPPING_RANGES),
            Err(Error::UnsupportedRegisterOperation(0xaa))
        );
    }
}
