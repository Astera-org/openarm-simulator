"""Run with: uv run python -m unittest discover -s tests -v

Requires Linux user/network namespaces and vcan. Cargo builds the simulator
and provisions its pinned test model using the shared asset cache.
"""

import json
import os
from pathlib import Path
import select
import socket
import subprocess
import unittest

from serde.json import from_json, to_json

from openarm_simulator import APIError, Client
from openarm_simulator.models import Configuration, Fault, MotorCommand, Push, Reset, State


class ClientTest(unittest.TestCase):
    def test_control_api(self):
        repo = Path(__file__).resolve().parents[2]
        build = subprocess.run(
            [
                "cargo", "build", "--locked", "--message-format=json-render-diagnostics",
                "-p", "openarm-simulator", "-p", "openarm-test-model",
            ],
            cwd=repo, stdout=subprocess.PIPE, text=True, check=True,
        )
        binary = model = None
        for line in build.stdout.splitlines():
            if not line.startswith("{"):
                continue
            message = json.loads(line)
            if message.get("target", {}).get("name") == "openarm-simulator":
                binary = message["executable"]
            for key, value in message.get("env", []):
                if key == "OPENARM_TEST_MODEL":
                    model = value
        self.assertIsNotNone(binary, "Cargo did not report the simulator executable")
        self.assertIsNotNone(model, "Cargo did not report the test model")

        env = os.environ.copy()
        for key in (
            "LISTEN_FDS", "LISTEN_PID", "LISTEN_FDS_FIRST_FD", "OPENARM_SIMULATOR_CONFIG",
        ):
            env.pop(key, None)
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            listener.listen()
            url = f"http://127.0.0.1:{listener.getsockname()[1]}"
            process = subprocess.Popen(
                [
                    "unshare", "--user", "--map-root-user", "--net", "sh", "-ec",
                    'ip link set lo up; for bus in can0 can1; do '
                    'ip link add "$bus" type vcan; ip link set "$bus" mtu 72 up; '
                    'done; exec "$@"',
                    "namespace", binary, "--model", model,
                    "--http-fd", str(listener.fileno()),
                ],
                pass_fds=(listener.fileno(),), stdout=subprocess.PIPE, text=True, env=env,
            )
        try:
            with process.stdout:
                self.assertTrue(select.select([process.stdout], [], [], 30)[0], "startup timeout")
                self.assertEqual(process.stdout.readline().strip(), f"HTTP administration: {url}")

                client = Client(url)
                state = client.state()
                configuration = client.configuration()
                self.assertEqual(from_json(State, to_json(state)), state)
                self.assertEqual(from_json(Configuration, to_json(configuration)), configuration)
                self.assertEqual(len(state.state.left), 8)
                self.assertIsInstance(state.state.left[0].command, MotorCommand)
                self.assertIsInstance(configuration.configuration.encoder_offsets_rad.left, tuple)
                self.assertEqual(state.config_sha256, configuration.config_sha256)

                fault = client.fault("left", 1, Fault(status=9, silent=True))
                self.assertEqual(fault.state.left[0].status, 9)
                self.assertTrue(fault.state.left[0].silent)
                self.assertEqual(fault.state.right[0].status, 0)
                cleared = client.fault("left", 1, Fault(status=0, silent=False))
                self.assertEqual(cleared.state.left[0].status, 0)
                self.assertFalse(cleared.state.left[0].silent)

                push = client.push(Push(left=(0.1,) * 7))
                self.assertEqual(push.plant.applied_torque_nm.left, (0.1,) * 7)
                self.assertEqual(push.plant.applied_torque_nm.right, (0.0,) * 7)
                reset = client.reset(Reset(left=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, -0.2)))
                self.assertEqual(reset.time, 0.0)
                self.assertEqual(reset.state.left[6].q, 0.25)
                self.assertEqual(reset.plant.applied_torque_nm.left, (0.0,) * 7)

                with self.assertRaisesRegex(APIError, "joint") as error:
                    client.fault("left", 0, Fault(status=9))
                self.assertEqual(error.exception.status, 400)
                with self.assertRaises(ValueError):
                    client.push(Push(left=(float("nan"),) * 7))
        finally:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    unittest.main()
