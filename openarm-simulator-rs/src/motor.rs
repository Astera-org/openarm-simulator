//! OpenArm v1 controller configuration and administration snapshots.
use damiao_can_rs::MappingRanges;
pub use damiao_simulator_rs::{Motor, MotorConfig};
use openarm_simulator_core_rs::MotorState;

// enactic/openarm_ros2@4e837e1d0dae692ff67b560b69d8d281d7a8d4ed,
// openarm_hardware/include/openarm_hardware/openarm_simple_hardware.hpp.
pub const V1_REPLY_ID_OFFSET: u16 = 0x10;
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

pub fn v1_motor(joint: usize) -> Motor {
    let ranges = match joint {
        1 | 2 => DM8009_DEFAULT_MAPPING_RANGES,
        3 | 4 => DM4340_DEFAULT_MAPPING_RANGES,
        5..=8 => DM4310_DEFAULT_MAPPING_RANGES,
        _ => panic!("v1 motor index must be 1..8"),
    };
    Motor::new(MotorConfig {
        id: joint as u16,
        reply_id: joint as u16 + V1_REPLY_ID_OFFSET,
        ranges,
    })
    .unwrap()
}

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct MotorBinding {
    pub bus: String,
    pub actuator: String,
    pub controller: MotorConfig,
}

pub fn v1_buses() -> std::collections::BTreeMap<String, String> {
    [
        ("left".into(), "can1".into()),
        ("right".into(), "can0".into()),
    ]
    .into()
}

pub fn v1_bindings() -> std::collections::BTreeMap<String, MotorBinding> {
    ["left", "right"]
        .into_iter()
        .flat_map(|side| {
            (1..=8).map(move |joint| {
                let motor = v1_motor(joint);
                (
                    format!("{side}_joint{joint}"),
                    MotorBinding {
                        bus: side.into(),
                        actuator: if joint == 8 {
                            format!("{side}_finger1_ctrl")
                        } else {
                            format!("{side}_joint{joint}_ctrl")
                        },
                        controller: MotorConfig {
                            id: motor.id(),
                            reply_id: motor.reply_id(),
                            ranges: motor.ranges,
                        },
                    },
                )
            })
        })
        .collect()
}

pub fn snapshot(motor: &Motor) -> MotorState {
    MotorState {
        id: motor.id(),
        reply_id: motor.reply_id(),
        command: motor.command,
        q: motor.q,
        dq: motor.dq,
        torque: motor.torque,
        status: motor.status,
        mos_temperature: motor.mos_temperature,
        rotor_temperature: motor.rotor_temperature,
        silent: motor.silent,
        ranges: motor.ranges,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use damiao_can_rs::{Feedback, MitCommand, MotorStatus};
    #[test]
    fn snapshot_and_can_feedback_preserve_motor_fields() {
        let mut motor = v1_motor(1);
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

        let snapshot = snapshot(&motor);
        let json = serde_json::to_value(snapshot).unwrap();
        assert_eq!(
            json,
            serde_json::json!({
                "id": 1, "reply_id": 17,
                "command": {"kp": 120.5, "kd": 1.5, "q": -1.25, "dq": 2.5, "tau": -3.75},
                "q": -7.5, "dq": 21.0, "torque": 6.5,
                "status": 2, "mos_temperature": 31, "rotor_temperature": 47, "silent": true,
                "ranges": {"pmax": 7.5, "vmax": 21.0, "tmax": 6.5}
            })
        );
        let restored: MotorState = serde_json::from_value(json).unwrap();
        assert_eq!(restored, snapshot);
        let feedback = Feedback::decode(&motor.state().unwrap(), restored.ranges).unwrap();
        assert_eq!(feedback.reported_id, restored.id as u8);
        assert_eq!(
            (feedback.q, feedback.dq, feedback.torque),
            (restored.q, restored.dq, restored.torque)
        );
        assert_eq!(feedback.status, restored.status);
        assert_eq!(feedback.mos_temperature, restored.mos_temperature);
        assert_eq!(feedback.rotor_temperature, restored.rotor_temperature);
    }
}
