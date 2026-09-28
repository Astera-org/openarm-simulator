"""Independent synthetic resistance fixtures, frozen before controller scoring.

These ranges illustrate robot-joint effects; none are measured OpenArm values.
Do not tune a fixture after observing a controller result. Change VERSION for
any changed challenge. The original gravity_challenges benchmark stays intact.
"""
import copy

import numpy as np

from .gravity_challenges import Challenge, SIDES, assembly

VERSION = 'independent-friction-v1'


def challenges():
    """Compatible, richer, and deliberately mismatched passive plants.

    Dry sliding friction: 0.045–0.17 Nm from the independent assembly generator;
    viscous damping: 0.17–0.74 Nm*s/rad. Stribeck breakaway is 1.6–3.1 times dry
    sliding friction at 0.018–0.075 rad/s. Modulated cases add 18–42% angle ripple
    and 12–32% direction asymmetry; strong stiction multiplies breakaway by 1.8.
    Passive cable springs are separate energy-storing elements, not friction.
    """
    base = assembly(738901, severity=1.15)
    rng = np.random.default_rng(273841)
    stribeck = copy.deepcopy(base)
    for joint in stribeck['joints'].values():
        joint['stribeck'] = dict(
            breakaway_nm=float(joint['frictionloss']*rng.uniform(1.6, 3.1)),
            velocity_rad_s=float(rng.uniform(.018, .075)))

    modulated = copy.deepcopy(stribeck)
    for joint in modulated['joints'].values():
        joint['stribeck'].update(
            direction_asymmetry=float(rng.choice([-1., 1.])*rng.uniform(.12, .32)),
            angle=dict(amplitude=float(rng.uniform(.18, .42)),
                       harmonic=int(rng.choice([1, 2])),
                       phase_rad=float(rng.uniform(-np.pi, np.pi))))

    breakaway = copy.deepcopy(stribeck)
    for joint in breakaway['joints'].values():
        joint['stribeck']['breakaway_nm'] *= 1.8
        joint['stribeck']['velocity_rad_s'] *= .7

    cable = copy.deepcopy(modulated)
    for side in SIDES:
        sign = 1. if side == 'right' else -1.
        for index, stiffness, rest in ((2, .31, .22*sign), (4, .23, .72), (6, .11, -.18)):
            cable['joints'][f'openarm_{side}_joint{index}'].update(
                stiffness=stiffness, springref=rest)

    return [
        Challenge('basic', 'compatible', 'Independent dry/viscous assembly baseline.', base),
        Challenge('stribeck', 'compatible', 'Symmetric speed-dependent breakaway and sliding.', stribeck),
        Challenge('angle-direction', 'model-mismatch',
                  'Position-modulated and asymmetric friction; constant friction models are incomplete.', modulated),
        Challenge('strong-breakaway', 'stress', 'Larger breakaway/sliding contrast at lower speed.', breakaway),
        Challenge('cable-resistance', 'model-mismatch',
                  'Angle/direction friction plus unmodeled passive cable springs.', cable),
    ]
