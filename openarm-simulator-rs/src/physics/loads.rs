use mujoco::{AppliedForces, Site};
use openarm_simulator_core::{
    AppliedForce, Spring,
    uom::si::{
        f64::{Force, Torque},
        force::newton,
        length::meter,
        torque::newton_meter,
    },
};
use std::collections::BTreeMap;

#[derive(Default)]
pub(super) struct Loads {
    pub(super) springs: BTreeMap<String, Spring>,
    pub(super) forces: BTreeMap<String, AppliedForce>,
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
            let state = context.site(force.site);
            let body = state.body();
            let position = *state.position();
            context.apply(
                body,
                position,
                <[Force; 3]>::from(force.force).map(|v| v.get::<newton>()),
                <[Torque; 3]>::from(force.torque).map(|v| v.get::<newton_meter>()),
            );
        }
        // ponytail: callback forces integrate explicitly; native MJCF tendons are
        // the upgrade path when stiffness/damping need implicit treatment.
        for spring in self.springs.values() {
            let states = spring.sites.map(|index| context.site(index));
            let (length, velocity, direction) = spring_geometry(&states);
            if length == 0. {
                continue;
            }
            let magnitude = spring.stiffness.value * (length - spring.rest_length.get::<meter>())
                + spring.damping.value * velocity;
            let force = direction.map(|n| n * magnitude);
            let [(body_a, point_a), (body_b, point_b)] =
                states.map(|site| (site.body(), *site.position()));
            context.apply(body_a, point_a, force, [0.; 3]);
            context.apply(body_b, point_b, force.map(|v| -v), [0.; 3]);
        }
    }
}

pub(super) fn spring_geometry(states: &[Site<'_>; 2]) -> (f64, f64, [f64; 3]) {
    let delta: [f64; 3] =
        std::array::from_fn(|i| states[1].position()[i] - states[0].position()[i]);
    let length = delta[0].hypot(delta[1]).hypot(delta[2]);
    if length == 0. {
        return (0., 0., [0.; 3]);
    }
    let direction = delta.map(|v| v / length);
    let velocities = states.each_ref().map(|site| site.velocity());
    let velocity = (0..3)
        .map(|i| direction[i] * (velocities[1][i] - velocities[0][i]))
        .sum();
    (length, velocity, direction)
}
