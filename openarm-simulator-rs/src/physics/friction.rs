use anyhow::{Result, ensure};
use openarm_simulator_core::{
    Stribeck,
    uom::si::{angle::radian, angular_velocity::radian_per_second, torque::newton_meter},
};

pub(super) struct VariableFriction {
    pub(super) qpos: usize,
    pub(super) dof: usize,
    pub(super) sliding_nm: f64,
    pub(super) law: Stribeck,
}

pub(super) fn validate(law: &Stribeck, sliding_nm: f64) -> Result<()> {
    ensure!(
        sliding_nm.is_finite()
            && sliding_nm >= 0.
            && law.breakaway.get::<newton_meter>().is_finite()
            && law.breakaway.get::<newton_meter>() >= sliding_nm,
        "stribeck requires finite breakaway_nm >= sliding frictionloss >= 0"
    );
    ensure!(
        law.velocity.get::<radian_per_second>().is_finite()
            && law.velocity.get::<radian_per_second>() > 0.,
        "stribeck velocity_rad_s must be finite and positive"
    );
    ensure!(
        law.direction_asymmetry.is_finite() && law.direction_asymmetry.abs() < 1.,
        "stribeck direction_asymmetry must be finite and between -1 and 1"
    );
    if let Some(angle) = &law.angle {
        ensure!(
            angle.amplitude.is_finite()
                && (0. ..1.).contains(&angle.amplitude)
                && angle.harmonic > 0
                && angle.phase.get::<radian>().is_finite(),
            "stribeck angle needs amplitude in [0,1), positive harmonic and finite phase_rad"
        );
    }
    ensure!(
        max_bound_nm(law).is_finite(),
        "stribeck maximum friction overflows"
    );
    Ok(())
}

pub(super) fn max_bound_nm(law: &Stribeck) -> f64 {
    law.breakaway.get::<newton_meter>()
        * (1. + law.direction_asymmetry.abs())
        * (1. + law.angle.as_ref().map_or(0., |a| a.amplitude))
}

pub(super) fn bound_nm(law: &Stribeck, sliding_nm: f64, physical_q: f64, velocity: f64) -> f64 {
    let speed = velocity / law.velocity.get::<radian_per_second>();
    let stribeck =
        sliding_nm + (law.breakaway.get::<newton_meter>() - sliding_nm) * (-speed * speed).exp();
    let direction = 1. + law.direction_asymmetry * speed.tanh();
    let angle = law.angle.as_ref().map_or(1., |a| {
        1. + a.amplitude * (f64::from(a.harmonic) * physical_q + a.phase.get::<radian>()).cos()
    });
    stribeck * direction * angle
}

#[cfg(test)]
mod tests {
    use super::*;
    use openarm_simulator_core::{
        AngleModulation,
        uom::si::f64::{Angle, AngularVelocity, Torque},
    };

    fn example() -> Stribeck {
        Stribeck {
            breakaway: Torque::new::<newton_meter>(0.6),
            velocity: AngularVelocity::new::<radian_per_second>(0.04),
            direction_asymmetry: 0.,
            angle: None,
        }
    }

    #[test]
    fn symmetric_curve_has_correct_units_limits_and_dissipative_sliding_force() {
        let model = example();
        validate(&model, 0.2).unwrap();
        assert_eq!(bound_nm(&model, 0.2, 0., 0.), 0.6);
        assert!(
            (bound_nm(&model, 0.2, 0., 0.04) - (0.2 + 0.4 / std::f64::consts::E)).abs() < 1e-14
        );
        assert!((bound_nm(&model, 0.2, 0., 1.) - 0.2).abs() < 1e-14);
        for speed in -1000..=1000 {
            let v = f64::from(speed) * 0.001;
            let bound = bound_nm(&model, 0.2, 0.7, v);
            assert!((0.2..=0.6).contains(&bound));
            assert_eq!(bound, bound_nm(&model, 0.2, 0.7, -v));
            assert!(-bound * v.signum() * v <= 0.);
        }
    }

    #[test]
    fn angle_and_direction_modulate_positive_bounded_resistance_only() {
        let model = Stribeck {
            direction_asymmetry: 0.3,
            angle: Some(AngleModulation {
                amplitude: 0.4,
                harmonic: 2,
                phase: Angle::new::<radian>(0.2),
            }),
            ..example()
        };
        validate(&model, 0.2).unwrap();
        for q in -30..=30 {
            for speed in -100..=100 {
                let q = f64::from(q) * 0.1;
                let v = f64::from(speed) * 0.01;
                let bound = bound_nm(&model, 0.2, q, v);
                assert!(bound > 0. && bound <= max_bound_nm(&model));
                assert!(-bound * v.signum() * v <= 0.);
                assert!((bound - bound_nm(&model, 0.2, q + std::f64::consts::PI, v)).abs() < 1e-14);
            }
        }
        assert!(bound_nm(&model, 0.2, 0.1, 0.1) > bound_nm(&model, 0.2, 0.1, -0.1));
    }

    #[test]
    fn reject_unphysical_or_nonfinite_configuration() {
        for model in [
            Stribeck {
                breakaway: Torque::new::<newton_meter>(0.1),
                ..example()
            },
            Stribeck {
                breakaway: Torque::new::<newton_meter>(f64::INFINITY),
                ..example()
            },
            Stribeck {
                velocity: AngularVelocity::new::<radian_per_second>(0.),
                ..example()
            },
            Stribeck {
                velocity: AngularVelocity::new::<radian_per_second>(f64::NAN),
                ..example()
            },
            Stribeck {
                direction_asymmetry: 1.,
                ..example()
            },
            Stribeck {
                angle: Some(AngleModulation {
                    amplitude: 1.,
                    harmonic: 1,
                    phase: Angle::new::<radian>(0.),
                }),
                ..example()
            },
            Stribeck {
                angle: Some(AngleModulation {
                    amplitude: 0.1,
                    harmonic: 0,
                    phase: Angle::new::<radian>(0.),
                }),
                ..example()
            },
        ] {
            assert!(validate(&model, 0.2).is_err());
        }
    }
}
