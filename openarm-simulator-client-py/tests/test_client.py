"""Run with: uv run python -m unittest discover -s tests -v

Requires Linux user/network namespaces and vcan. Cargo builds the simulator
and provisions its pinned test model using the shared asset cache.
"""

import json
from http import HTTPStatus
import os
from pathlib import Path
import select
import socket
import subprocess
import unittest

from serde.json import from_json, to_json

from openarm_simulator_client import APIError, Client
from openarm_simulator_client.models import AppliedForce, Configuration, Fault, HingeJointParameters, Integrator, MappingRanges, MotorCommand, SiteIndex, SlideJointParameters, Spring, State


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
            "LISTEN_FDS", "LISTEN_PID", "LISTEN_FDS_FIRST_FD",
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
                    "--config", str(repo / "openarm-simulator-rs/config/openarm-v1.json"),
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
                self.assertIs(configuration.configuration.integrator, Integrator.IMPLICIT_FAST)
                names = client.names()
                joint = names.joints["openarm_left_joint7"]
                site = names.sites["world_site"]
                self.assertEqual(from_json(State, to_json(state)), state)
                self.assertEqual(from_json(Configuration, to_json(configuration)), configuration)
                self.assertIsInstance(configuration.configuration.joints[names.joints["openarm_left_joint1"]], HingeJointParameters)
                self.assertIsInstance(configuration.configuration.joints[names.joints["openarm_left_finger_joint1"]], SlideJointParameters)
                self.assertEqual(len(state.state), 16)
                self.assertIsInstance(state.state["left_joint1"].command, MotorCommand)
                self.assertEqual(state.state["left_joint1"].ranges, MappingRanges(12.5, 45.0, 54.0))
                self.assertEqual(state.state["left_joint1"].mos_temperature_k, 298.15)
                self.assertEqual(state.state["left_joint1"].rotor_temperature_k, 298.15)
                self.assertIsInstance(configuration.configuration.encoder_offsets_rad, dict)
                self.assertEqual(state.timestep_ns, configuration.configuration.timestep_ns)

                fault = client.fault("left_joint1", Fault(status=9, silent=True))
                self.assertEqual(fault.state["left_joint1"].status, 9)
                self.assertTrue(fault.state["left_joint1"].silent)
                self.assertEqual(fault.state["right_joint1"].status, 0)
                unknown = client.fault("left_joint1", Fault(status=2))
                self.assertEqual(unknown.state["left_joint1"].status, 2)
                cleared = client.fault("left_joint1", Fault(status=0, silent=False))
                self.assertEqual(cleared.state["left_joint1"].status, 0)
                self.assertFalse(cleared.state["left_joint1"].silent)

                push = client.push({joint: 0.1})
                self.assertEqual(push.plant.applied_torque_nm, {joint: 0.1})
                self.assertEqual(client.reset(), HTTPStatus.OK)
                self.assertEqual(client.names(), names)
                self.assertEqual(client.state(), state)
                self.assertTrue(state.paused)
                self.assertEqual(state.time_ns, 0)
                self.assertEqual(client.pause(), HTTPStatus.NO_CONTENT)
                self.assertEqual(client.advance(state.timestep_ns - 1), HTTPStatus.OK)
                self.assertEqual(client.state().statistics.steps, 0)
                self.assertEqual(client.advance(1), HTTPStatus.OK)
                self.assertEqual(client.state().statistics.steps, 1)
                self.assertEqual(client.state().time_ns, state.timestep_ns)
                self.assertEqual(client.unpause(), HTTPStatus.OK)
                self.assertEqual(client.unpause(), HTTPStatus.NO_CONTENT)
                with self.assertRaises(APIError) as error:
                    client.advance(1)
                self.assertEqual(error.exception.status, 409)
                self.assertEqual(client.pause(), HTTPStatus.OK)
                self.assertEqual(client.pause(), HTTPStatus.NO_CONTENT)
                client.reset()
                self.assertEqual(client.state(), state)
                for duration in [-1, 1.0, True, 2**64]:
                    with self.assertRaises(ValueError):
                        client.advance(duration)

                id = "load / #α"
                spring = Spring((site, site), 0.1, 1.0, 0.2)
                force = AppliedForce(site, (0.1, 0.0, 0.0), (0.0, 0.0, 0.0))
                self.assertEqual(client.put_spring(id, spring), HTTPStatus.CREATED)
                self.assertEqual(client.spring(id), spring)
                self.assertEqual(client.springs()[id], spring)
                self.assertEqual(client.put_force(id, force), HTTPStatus.CREATED)
                self.assertEqual(client.force(id), force)
                self.assertEqual(client.forces()[id], force)
                with self.assertRaises(APIError) as error:
                    client.put_force(id, AppliedForce(SiteIndex(len(state.sites)), force.force_world_n, force.torque_world_nm))
                self.assertEqual(error.exception.status, 400)
                client.unpause()
                self.assertEqual(client.put_force(id, force), HTTPStatus.NO_CONTENT)
                self.assertEqual(client.put_spring(id, spring), HTTPStatus.NO_CONTENT)
                client.pause()
                observed = client.state()
                self.assertEqual(observed.springs[id].length_m, 0.0)
                self.assertEqual(observed.sites[site], state.sites[site])
                self.assertEqual(client.delete_spring(id), HTTPStatus.NO_CONTENT)
                self.assertEqual(client.delete_force(id), HTTPStatus.NO_CONTENT)
                with self.assertRaises(APIError) as error:
                    client.force(id)
                self.assertEqual(error.exception.status, 404)
                client.reset()
                self.assertEqual(client.state(), state)

                with self.assertRaisesRegex(APIError, "motor") as error:
                    client.fault("left_joint0", Fault(status=9))
                self.assertEqual(error.exception.status, 400)
                with self.assertRaises(ValueError):
                    client.push({joint: float("nan")})
        finally:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    unittest.main()
