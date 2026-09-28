"""Build/start the Rust simulator; Python never participates in its timed loop."""
import functools
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess

from .interfaces import require_virtual_interfaces
from .model import model_path, model_sha256

CRATE = Path(__file__).resolve().parents[2]


def joint_torques(forces):
    """Seven external Nm per arm; reject typos/malformed sidechannel requests."""
    if not isinstance(forces, dict) or set(forces)-{'right', 'left'}:
        raise ValueError('Push expects right/left joint torque arrays')
    result = {}
    for side in ('right', 'left'):
        values = forces.get(side, [0.]*7)
        if (not isinstance(values, (list, tuple)) or len(values) != 7
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise ValueError('Push needs seven finite joint torques in Nm per arm')
        result[side] = list(values)
    return result


def configuration_identity(configuration, model):
    """Canonical resolved plant, excluding poses and transient external torques."""
    effective = dict(configuration, model_sha256=model_sha256(model))
    encoded = json.dumps(effective, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    return effective, hashlib.sha256(encoded).hexdigest()


def cargo(arguments):
    import mujoco
    executable = shutil.which('cargo') or str(Path.home()/'.cargo/bin/cargo')
    if not Path(executable).is_file():
        raise RuntimeError('The simulator needs Rust/Cargo; install the Rust toolchain, then retry.')
    environment = dict(os.environ, MUJOCO_DIR=str(Path(mujoco.__file__).parent.resolve()))
    subprocess.run([executable, *arguments, '--manifest-path', str(CRATE/'Cargo.toml'),
                    '--target-dir', str(CRATE/'target'), '--locked'], env=environment, check=True)


@functools.lru_cache(maxsize=1)
def native_binary():
    cargo(['build', '--release', '--quiet'])
    return CRATE/'target/release/openarm-sim-native'


class NativeSimulator:
    def __init__(self, poses=None, offsets=None, *, model=None, log=None, bodies=None, friction_scale=1., joints=None):
        require_virtual_interfaces()
        self.model = model_path(model)
        binary = native_binary()
        self.process = None
        self.socket, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.socket.settimeout(10.)
        try:
            self.process = subprocess.Popen(
                [str(binary), 'serve', str(child.fileno()), str(self.model),
                 json.dumps(dict(poses=poses or {}, offsets=offsets or {}, bodies=bodies or {},
                                 friction_scale=friction_scale, joints=joints or {}), allow_nan=False)],
                pass_fds=(child.fileno(),), start_new_session=True,
                stdout=log, stderr=subprocess.STDOUT if log else None)
            child.close()
            ready = self._receive()
            if not ready.get('ready'):
                raise RuntimeError('Native simulator did not become ready')
            self.configuration, self.config_sha256 = configuration_identity(ready['configuration'], self.model)
            self.socket.settimeout(3.)
        except BaseException:
            child.close()
            self.close()
            raise

    def _receive(self):
        packet = self.socket.recv(32768)
        if not packet:
            raise RuntimeError('Native simulator disconnected; check its error output')
        return json.loads(packet)

    def check(self):
        if self.process.poll() is not None:
            raise RuntimeError(f'Native simulator exited ({self.process.returncode})')

    def rpc(self, action='inspect', payload=None):
        self.check()
        if action == 'push':
            payload = joint_torques(payload)
        message = dict(action=action)
        if payload is not None:
            message['payload'] = payload
        self.socket.sendall(json.dumps(message, allow_nan=False).encode())
        reply = self._receive()
        if 'error' in reply:
            raise ValueError(reply['error'])
        return dict(reply, config_sha256=self.config_sha256)

    def push(self, forces):
        """Replace external joint torques (7 Nm per arm); omitted arms become zero.

        Torques persist until the next push or simulator restart. This is an
        administrative simulated disturbance, never a motor command.
        """
        return self.rpc('push', forces)

    def close(self):
        # EOF also stops the endpoint after launcher failure, without leaving an
        # orphan. Keep it alive until the controller has disabled the motors.
        self.socket.close()
        if self.process is not None:
            try:
                self.process.wait(5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
