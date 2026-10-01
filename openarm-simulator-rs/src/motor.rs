//! Application bindings and administration snapshots for emulated motors.
pub use damiao_simulator_rs::{Motor, MotorConfig};
use openarm_simulator_core_rs::MotorState;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize)]
#[serde(deny_unknown_fields)]
pub struct MotorBinding {
    pub bus: String,
    pub actuator: String,
    pub controller: MotorConfig,
    #[serde(default)]
    pub encoder_offset_rad: f64,
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
    use damiao_can_rs::{Feedback, MappingRanges, MitCommand, MotorStatus};
    #[test]
    fn snapshot_and_can_feedback_preserve_motor_fields() {
        let mut motor = Motor::new(MotorConfig {
            id: 1,
            reply_id: 17,
            ranges: MappingRanges {
                pmax: 12.5,
                vmax: 45.,
                tmax: 54.,
            },
        })
        .unwrap();
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
