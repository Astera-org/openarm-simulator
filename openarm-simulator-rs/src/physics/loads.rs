use mujoco::{AppliedForces, Body};
use openarm_simulator_core::{
    AppliedForce, BodyPoint, Spring,
    uom::si::{
        f64::{Force, Length, Torque},
        force::newton,
        length::meter,
        torque::newton_meter,
    },
};
use std::collections::BTreeMap;

#[derive(Default)]
pub(super) struct Loads {
    pub(super) springs: BTreeMap<String, Spring<BodyPoint>>,
    pub(super) forces: BTreeMap<String, AppliedForce<BodyPoint>>,
    pub(super) joint_torques: Vec<(usize, f64)>,
}
impl Loads {
    pub(super) fn reset(&mut self) {
        self.springs.clear();
        self.forces.clear();
        self.joint_torques.clear();
    }

    pub(super) fn apply(&self, context: &mut AppliedForces<'_>) {
        context.generalized().fill(0.);
        for &(dof, torque) in &self.joint_torques {
            context.generalized()[dof] = torque;
        }
        for force in self.forces.values() {
            let position = context
                .body(force.point.body)
                .point_position(local_position(&force.point));
            context.apply(
                force.point.body,
                position,
                <[Force; 3]>::from(force.force).map(|v| v.get::<newton>()),
                <[Torque; 3]>::from(force.torque).map(|v| v.get::<newton_meter>()),
            );
        }
        // ponytail: callback forces integrate explicitly; native MJCF tendons are
        // the upgrade path when stiffness/damping need implicit treatment.
        for spring in self.springs.values() {
            let bodies = spring.endpoints.map(|point| context.body(point.body));
            let (length, velocity, direction) = spring_geometry(&bodies, &spring.endpoints);
            if length == 0. {
                continue;
            }
            let magnitude = spring.stiffness.value * (length - spring.rest_length.get::<meter>())
                + spring.damping.value * velocity;
            let force = direction.map(|n| n * magnitude);
            let [point_a, point_b] = std::array::from_fn(|i| {
                bodies[i].point_position(local_position(&spring.endpoints[i]))
            });
            context.apply(spring.endpoints[0].body, point_a, force, [0.; 3]);
            context.apply(
                spring.endpoints[1].body,
                point_b,
                force.map(|v| -v),
                [0.; 3],
            );
        }
    }
}

fn local_position(point: &BodyPoint) -> [f64; 3] {
    <[Length; 3]>::from(point.position).map(|v| v.get::<meter>())
}

pub(super) fn spring_geometry(
    bodies: &[Body<'_>; 2],
    endpoints: &[BodyPoint; 2],
) -> (f64, f64, [f64; 3]) {
    let points: [_; 2] =
        std::array::from_fn(|i| bodies[i].point_position(local_position(&endpoints[i])));
    let delta: [f64; 3] = std::array::from_fn(|i| points[1][i] - points[0][i]);
    let length = delta[0].hypot(delta[1]).hypot(delta[2]);
    if length == 0. {
        return (0., 0., [0.; 3]);
    }
    let direction = delta.map(|v| v / length);
    let velocities: [_; 2] =
        std::array::from_fn(|i| bodies[i].point_velocity(local_position(&endpoints[i])));
    let velocity = (0..3)
        .map(|i| direction[i] * (velocities[1][i] - velocities[0][i]))
        .sum();
    (length, velocity, direction)
}
