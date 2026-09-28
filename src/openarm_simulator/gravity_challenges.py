"""Independent, deterministic plant fixtures for the blind gravity audit.

Owned by the simulator/evaluation task, not by estimator development. These are
synthetic assembly variations, not measured tolerances for an actual OpenArm.
No calibration or collector code is imported. Freeze this file before scoring;
changing a fixture after seeing its score starts a new benchmark version.
"""
from dataclasses import dataclass
import copy

import mujoco
import numpy as np

from .model import model_path

VERSION = 'independent-assemblies-v1'
SIDES = ('right', 'left')


@dataclass(frozen=True)
class Challenge:
    name: str
    category: str
    description: str
    config: dict


def assembly(seed, *, severity=1.):
    """Vary mass, CoM, and shape with strictly physical principal inertias.

    Principal second moments S = (sum(I)/2 - I) remain positive. Changing their
    three lengths independently then rebuilding I preserves triangle inequalities
    instead of independently perturbing principal inertias into invalid tensors.
    The upstream principal-axis orientations and collision geometry stay fixed.
    """
    model = mujoco.MjModel.from_xml_path(str(model_path()))
    rng = np.random.default_rng(seed)
    arm_density = dict(zip(SIDES, rng.uniform(1-.09*severity, 1+.09*severity, 2)))
    bodies = {}
    for index in range(model.nbody):
        name = model.body(index).name
        side = next((s for s in SIDES if name.startswith(f'openarm_{s}_')), None)
        if side is None or name.endswith('link0') or model.body_mass[index] <= 0:
            continue
        scale = arm_density[side]*rng.uniform(1-.11*severity, 1+.11*severity)
        inertia = model.body_inertia[index]
        second = np.maximum(inertia.sum()/2-inertia, 1e-15)
        lengths = rng.uniform(1-.17*severity, 1+.17*severity, 3)
        second = second*scale*lengths**2
        com = model.body_ipos[index]+rng.uniform(-.005*severity, .005*severity, 3)
        bodies[name] = dict(mass=float(model.body_mass[index]*scale),
                            com=com.tolist(), inertia=(second.sum()-second).tolist())
    joints = {}
    # Friction is independently sampled per joint and per arm, rather than a
    # single scale that silently gives both arms identical nuisance parameters.
    for side in SIDES:
        for joint in range(1, 8):
            joints[f'openarm_{side}_joint{joint}'] = dict(
                frictionloss=float(rng.uniform(.045, .17)),
                damping=float(rng.uniform(.17, .74)))
    return dict(bodies=bodies, joints=joints)


def challenges():
    first, second = assembly(620187), assembly(944063, severity=1.3)
    low = copy.deepcopy(second)
    for joint in low['joints'].values():
        joint['frictionloss'] *= .06
        joint['damping'] *= .3
    heavy = assembly(379013, severity=1.5)
    for body in heavy['bodies'].values():
        body['inertia'] = (np.asarray(body['inertia'])*2.7).tolist()
    for index, joint in enumerate(heavy['joints'].values()):
        joint['frictionloss'] *= 2.4 if index % 7 < 4 else 1.4
        joint['damping'] *= 1.8
    elastic = copy.deepcopy(first)
    for side in SIDES:
        for joint, stiffness, reference in ((2, .36, .15), (4, .27, .6), (6, .12, -.1)):
            elastic['joints'][f'openarm_{side}_joint{joint}'].update(
                stiffness=stiffness, springref=reference if side == 'right' else -reference)
    return [
        Challenge('assembly-a', 'ordinary', 'Correlated density and independent body/drive variation.', first),
        Challenge('assembly-b', 'ordinary', 'A separate, moderately larger assembly variation.', second),
        Challenge('low-friction', 'stress', 'Assembly B with reduced stiction; exposes uncompensated drift.', low),
        Challenge('inertia-friction', 'stress', 'Larger rotational inertias and higher dry/viscous friction.', heavy),
        Challenge('elastic-load', 'model-mismatch', 'Unmodeled passive joint springs resembling cable restoring loads.', elastic),
    ]


def challenge_poses(count, *, validation=False):
    """Independent whole-robot poses, selected without seeing hidden dynamics.

    Geometric checks use the public nominal collision model. Training and scoring
    use disjoint fixed random streams; physical calibration truth never selects a
    pose. Sweeps and the home-hub transitions are checked before any simulation.
    """
    from .collision import CollisionChecker

    home = {s: np.radians(q).tolist() for s, q in dict(
        right=[20, 35, 0, 65, 0, 0, 0, -25], left=[-20, -35, 0, 65, 0, 0, 0, -25]).items()}
    rng = np.random.default_rng(248087 if validation else 108791)
    low = np.array([-35, 18, -60, 24, -62, -28, -60, -39])
    high = np.array([92, 93, 60, 108, 62, 28, 60, -16])
    checker, poses = CollisionChecker(), []
    for _ in range(200*count):
        center = {s: np.radians(rng.uniform(low, high)) for s in SIDES}
        center['left'][:2] *= -1
        direction = {s: rng.choice([-1., 1.], 7) for s in SIDES}
        endpoints = [{s: (q+np.r_[direction[s]*sign*np.radians(3.5), 0.]).tolist()
                      for s, q in center.items()} for sign in (-1, 1)]
        try:
            checker.path(home, SIDES, endpoints[0])
            checker.path(endpoints[0], SIDES, endpoints[1])
            checker.path(endpoints[1], SIDES, home)
        except RuntimeError:
            continue
        poses.append(dict(center={s: q.tolist() for s, q in center.items()}, endpoints=endpoints))
        if len(poses) == count:
            return poses
    raise RuntimeError('Insufficient collision-free challenge poses')


def diagnostic_low_friction(config):
    """Same hidden assembly with nearly absent friction, for drift scoring only."""
    result = copy.deepcopy(config)
    for joint in result['joints'].values():
        joint['frictionloss'] *= .03
        joint['damping'] *= .03
    return result
