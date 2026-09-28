"""No-socket accelerated fixture, sharing Rust physics and official CAN codecs."""
import json
import select
import subprocess

import numpy as np
import openarm_can as oa

from .model import model_path
from .native import native_binary, joint_torques, configuration_identity

SIDES = ('right', 'left')
TYPES = [oa.MotorType.DM8009] * 2 + [oa.MotorType.DM4340] * 2 + [oa.MotorType.DM4310] * 4


class PhysicsExperiment:
    """Explicit simulated-time stepping; cannot connect to a physical robot."""
    backend = 'mujoco-lockstep'

    def __init__(self, *, model=None, **config):
        self.model = model_path(model)
        self.process = subprocess.Popen([str(native_binary()), 'experiment', str(self.model),
                                         json.dumps(config, allow_nan=False)],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        text=True, bufsize=1)
        try:
            self.motors = [[oa.Motor(kind, i+1, i+17) for i, kind in enumerate(TYPES)] for _ in SIDES]
            self.observation = self.decode(self.rpc(action='inspect'))
            self.configuration, self.config_sha256 = configuration_identity(self.rpc(action='configuration'), self.model)
        except BaseException:
            self.close()
            raise

    def rpc(self, **message):
        self.process.stdin.write(json.dumps(message, allow_nan=False)+'\n')
        self.process.stdin.flush()
        if not select.select([self.process.stdout], [], [], 15.)[0]:
            raise RuntimeError('Physics experiment timed out')
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError('Physics experiment exited; inspect stderr')
        result = json.loads(line)
        if 'error' in result:
            raise RuntimeError(result['error'])
        return result

    def decode(self, result):
        observation = dict(time=result['time'], contacts=result['contacts'], arms={})
        for side, motors, packets in zip(SIDES, self.motors, result['packets']):
            states = [oa.CanPacketDecoder.parse_motor_state_data(motor, packet)
                      for motor, packet in zip(motors, packets, strict=True)]
            if not all(s.valid for s in states):
                raise RuntimeError('Invalid simulator feedback')
            observation['arms'][side] = dict(q=[s.position for s in states],
                dq=[s.velocity for s in states], torque=[s.torque for s in states],
                status=[p[0] >> 4 for p in packets])
        return observation

    def reset(self, poses):
        self.observation = self.decode(self.rpc(action='reset', poses=poses))
        return self.observation

    def enable(self):
        packets = [[s, i+1, [255]*7+[0xfc]] for s in range(2) for i in range(8)]
        self.observation = self.decode(self.rpc(action='step', packets=packets, steps=0))
        return self.observation

    def step(self, commands, dt=.005):
        count = round(dt/.0005)
        if not 1 <= count <= 2000 or abs(count*.0005-dt) > 1e-10:
            raise ValueError('Experiment period must be an integer number of 0.5ms physics steps')
        packets = []
        if set(commands) != set(SIDES):
            raise ValueError('Experiment requires explicit commands for both arms')
        for s, side in enumerate(SIDES):
            if len(commands[side]) != 8:
                raise ValueError('Eight motor commands required')
            for i, (q, dq, tau, kp, kd) in enumerate(commands[side]):
                values = [q, dq, tau, kp, kd]
                if not np.isfinite(values).all():
                    raise ValueError('Non-finite experiment command')
                packet = oa.CanPacketEncoder.create_mit_control_command(
                    self.motors[s][i], oa.MITParam(kp, kd, q, dq, tau))
                packets.append([s, packet.send_can_id, list(packet.data)])
        self.observation = self.decode(self.rpc(action='step', packets=packets, steps=count))
        return self.observation

    def push(self, forces):
        """Replace persistent external joint torques, in Nm; reset clears them."""
        forces = joint_torques(forces)
        return self.rpc(action='push', forces=[forces[s] for s in SIDES])

    def truth(self):
        """Evaluator-only; the data collector/estimator must never call this."""
        return self.rpc(action='truth')

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process.stdout.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def perturbed_bodies(seed, mass_fraction=.2, com_shift_m=.008, inertia_fraction=.3, *, model=None):
    """Reproducible hidden assembly: scaled SPD inertia, shifted local CoM."""
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(model_path(model)))
    rng = np.random.default_rng(seed)
    result = {}
    for i in range(model.nbody):
        name = model.body(i).name
        if (not any(name.startswith(f'openarm_{s}_') for s in SIDES)
                or model.body_mass[i] <= 0 or name.endswith('link0')):
            continue
        mass_scale = rng.uniform(1-mass_fraction, 1+mass_fraction)
        inertia_scale = mass_scale*rng.uniform(1-inertia_fraction, 1+inertia_fraction)
        result[name] = dict(mass=float(model.body_mass[i]*mass_scale),
            com=(model.body_ipos[i]+rng.uniform(-com_shift_m, com_shift_m, 3)).tolist(),
            inertia=(model.body_inertia[i]*inertia_scale).tolist())
    return result
