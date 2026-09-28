//! Optional synthetic dry-friction envelope; MuJoCo still solves the constraint.
//!
//! Positive envelopes oppose slip without an explicit sign(v) force. Modulating
//! the bound is a quasi-static approximation, not a bristle/hysteresis model.
use anyhow::{Result, ensure};
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize, Clone, Debug)]
#[serde(deny_unknown_fields)]
pub struct AngleModulation {
    /// Fractional dry-friction modulation, in [0, 1).
    pub amplitude: f64,
    /// Cycles per physical joint revolution, a positive integer.
    pub harmonic: u32,
    pub phase_rad: f64,
}

#[derive(Deserialize, Serialize, Clone, Debug)]
#[serde(deny_unknown_fields)]
pub struct Stribeck {
    /// Dry-friction bound at zero speed, before angle modulation, in Nm.
    pub breakaway_nm: f64,
    /// Stribeck transition speed in rad/s; also blends directional asymmetry.
    pub velocity_rad_s: f64,
    /// Fractional sliding asymmetry in (-1, 1); positive means more resistance
    /// at positive velocity. Exact zero speed uses the mean breakaway value.
    #[serde(default)]
    pub direction_asymmetry: f64,
    #[serde(default)]
    pub angle: Option<AngleModulation>,
}

impl Stribeck {
    pub fn validate(&self, sliding_nm: f64) -> Result<()> {
        ensure!(
            sliding_nm.is_finite()
                && sliding_nm >= 0.
                && self.breakaway_nm.is_finite()
                && self.breakaway_nm >= sliding_nm,
            "stribeck requires finite breakaway_nm >= sliding frictionloss >= 0"
        );
        ensure!(
            self.velocity_rad_s.is_finite() && self.velocity_rad_s > 0.,
            "stribeck velocity_rad_s must be finite and positive"
        );
        ensure!(
            self.direction_asymmetry.is_finite() && self.direction_asymmetry.abs() < 1.,
            "stribeck direction_asymmetry must be finite and between -1 and 1"
        );
        if let Some(angle) = &self.angle {
            ensure!(
                angle.amplitude.is_finite()
                    && (0. ..1.).contains(&angle.amplitude)
                    && angle.harmonic > 0
                    && angle.phase_rad.is_finite(),
                "stribeck angle needs amplitude in [0,1), positive harmonic and finite phase_rad"
            );
        }
        ensure!(
            self.max_bound_nm().is_finite(),
            "stribeck maximum friction overflows"
        );
        Ok(())
    }

    pub fn max_bound_nm(&self) -> f64 {
        self.breakaway_nm
            * (1. + self.direction_asymmetry.abs())
            * (1. + self.angle.as_ref().map_or(0., |a| a.amplitude))
    }

    pub fn bound_nm(&self, sliding_nm: f64, physical_q: f64, velocity: f64) -> f64 {
        let speed = velocity / self.velocity_rad_s;
        let stribeck = sliding_nm + (self.breakaway_nm - sliding_nm) * (-speed * speed).exp();
        let direction = 1. + self.direction_asymmetry * speed.tanh();
        let angle = self.angle.as_ref().map_or(1., |a| {
            1. + a.amplitude * (f64::from(a.harmonic) * physical_q + a.phase_rad).cos()
        });
        stribeck * direction * angle
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn example() -> Stribeck {
        Stribeck {
            breakaway_nm: 0.6,
            velocity_rad_s: 0.04,
            direction_asymmetry: 0.,
            angle: None,
        }
    }

    #[test]
    fn symmetric_curve_has_correct_units_limits_and_dissipative_sliding_force() {
        let model = example();
        model.validate(0.2).unwrap();
        assert_eq!(model.bound_nm(0.2, 0., 0.), 0.6);
        assert!((model.bound_nm(0.2, 0., 0.04) - (0.2 + 0.4 / std::f64::consts::E)).abs() < 1e-14);
        assert!((model.bound_nm(0.2, 0., 1.) - 0.2).abs() < 1e-14);
        for speed in -1000..=1000 {
            let v = f64::from(speed) * 0.001;
            let bound = model.bound_nm(0.2, 0.7, v);
            assert!((0.2..=0.6).contains(&bound));
            assert_eq!(bound, model.bound_nm(0.2, 0.7, -v));
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
                phase_rad: 0.2,
            }),
            ..example()
        };
        model.validate(0.2).unwrap();
        for q in -30..=30 {
            for speed in -100..=100 {
                let q = f64::from(q) * 0.1;
                let v = f64::from(speed) * 0.01;
                let bound = model.bound_nm(0.2, q, v);
                assert!(bound > 0. && bound <= model.max_bound_nm());
                assert!(-bound * v.signum() * v <= 0.);
                assert!((bound - model.bound_nm(0.2, q + std::f64::consts::PI, v)).abs() < 1e-14);
            }
        }
        assert!(model.bound_nm(0.2, 0.1, 0.1) > model.bound_nm(0.2, 0.1, -0.1));
    }

    #[test]
    fn reject_unphysical_or_nonfinite_configuration() {
        for model in [
            Stribeck {
                breakaway_nm: 0.1,
                ..example()
            },
            Stribeck {
                breakaway_nm: f64::INFINITY,
                ..example()
            },
            Stribeck {
                velocity_rad_s: 0.,
                ..example()
            },
            Stribeck {
                velocity_rad_s: f64::NAN,
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
                    phase_rad: 0.,
                }),
                ..example()
            },
            Stribeck {
                angle: Some(AngleModulation {
                    amplitude: 0.1,
                    harmonic: 0,
                    phase_rad: 0.,
                }),
                ..example()
            },
        ] {
            assert!(model.validate(0.2).is_err());
        }
    }
}
