"""Nominal collision checks for selecting independent experiment poses."""
import math

import mujoco
import numpy as np

from .model import model_path

CLEARANCE = .005  # Model clearance, metres; not a bound on model/calibration error.


def path_poses(starts, selected, targets=None):
    """Same straight joint-space path as the smooth execution, sampled at <=1° steps."""
    if set(starts) != {'right', 'left'} or not selected or not set(selected) <= set(starts):
        raise ValueError('Zero path needs both measured arm poses and at least one selected arm')
    if any(len(q) != 8 or not all(math.isfinite(v) for v in q) for q in starts.values()):
        raise ValueError('Zero path needs eight finite encoder positions per arm')
    targets = {s: [0.]*8 for s in selected} if targets is None else targets
    if not set(selected) <= set(targets) or any(
            len(targets[s]) != 8 or not all(math.isfinite(v) for v in targets[s]) for s in selected):
        raise ValueError('Target path needs eight finite encoder positions per selected arm')
    largest = max(abs(math.degrees(b-a)) for s in selected for a, b in zip(starts[s], targets[s]))
    samples = max(2, math.ceil(largest) + 1)
    for fraction in np.linspace(0., 1., samples):
        yield {s: np.asarray(q) + fraction*(np.asarray(targets[s])-q) if s in selected else np.asarray(q)
               for s, q in starts.items()}


class CollisionBlocked(RuntimeError):
    def __init__(self, hit):
        self.details = dict(geoms=list(hit[:2]), clearance_mm=hit[2]*1000)
        super().__init__(f'Collision blocked: {hit[0]} / {hit[1]} clearance {hit[2]*1000:.1f} mm')


class CollisionChecker:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(model_path()))
        self.data = mujoco.MjData(self.model)
        # MuJoCo combines geom margins by maximum, so each needs the full clearance.
        self.model.geom_margin[:] = CLEARANCE
        self.joints = {s: [self.model.joint(f'openarm_{s}_joint{i}').qposadr[0] for i in range(1, 8)]
                       for s in ('right', 'left')}
        self.fingers = {s: [self.model.joint(f'openarm_{s}_finger_joint{i}').qposadr[0] for i in (1, 2)]
                        for s in self.joints}
        # These finger pairs overlap in upstream's closed pose; closure is intentional.
        self.closure_pairs = {frozenset((self.model.geom(f'openarm_{s}_right_finger_collision').id,
                                        self.model.geom(f'openarm_{s}_left_finger_collision').id)) for s in self.joints}

    def contacts(self, poses):
        for side in self.joints:
            angles = poses[side]
            self.data.qpos[self.joints[side]] = angles[:7]
            self.data.qpos[self.fingers[side]] = .044 * max(0., min(1., angles[7] / -1.0472))
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        mujoco.mj_collision(self.model, self.data)
        hits = {}
        for contact in self.data.contact:
            if contact.dist < CLEARANCE and frozenset(contact.geom) not in self.closure_pairs:
                pair = tuple(sorted((self.model.geom(contact.geom1).name, self.model.geom(contact.geom2).name)))
                hits[pair] = min(hits.get(pair, CLEARANCE), float(contact.dist))
        return hits

    def collision(self, poses):
        return next(((*pair, distance) for pair, distance in self.contacts(poses).items()), None)

    def check(self, poses):
        hit = self.collision(poses)
        if hit:
            raise CollisionBlocked(hit)

    def path(self, starts, selected, targets=None):
        samples = 0
        for poses in path_poses(starts, selected, targets):
            self.check(poses)
            samples += 1
        return samples
