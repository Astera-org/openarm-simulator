"""Simulator-only configuration, identity, disturbances and frozen challenges."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock

from openarm_simulator.experiment import PhysicsExperiment
from openarm_simulator.friction_challenges import challenges, VERSION
from openarm_simulator.native import NativeSimulator, joint_torques
from openarm_simulator.service import load_configuration


class FrictionTests(unittest.TestCase):
    def test_independent_fixtures_are_frozen_distinct_and_loadable(self):
        import openarm_simulator.friction_challenges as module
        self.assertEqual(VERSION, 'independent-friction-v1')
        self.assertEqual(hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest(),
                         '763607344f8a0820f27a982014358cb31dc6ebe70344f7729e90995be072f054')
        self.assertEqual([case.config for case in challenges()], [case.config for case in challenges()])
        identities = set()
        for case in challenges():
            with self.subTest(case=case.name), PhysicsExperiment(**case.config) as world:
                identities.add(world.config_sha256)
                for name, requested in case.config['joints'].items():
                    effective = world.configuration['joints'][name]
                    self.assertAlmostEqual(effective['frictionloss'], requested['frictionloss'], places=14)
                    if 'stribeck' in requested:
                        self.assertAlmostEqual(effective['stribeck']['breakaway_nm'], requested['stribeck']['breakaway_nm'], places=14)
                self.assertNotIn('plant', world.observation)
                self.assertNotIn('configuration', world.observation)
        self.assertEqual(len(identities), 5)

    def test_identity_uses_effective_parameters_not_initial_pose_or_json_order(self):
        with PhysicsExperiment() as basic:
            expected = basic.config_sha256
            q = {side: list(arm['q']) for side, arm in basic.observation['arms'].items()}
            q['right'][6] += .1
            # Explicit default values resolve to the same physical model.
            joints = copy.deepcopy(basic.configuration['joints'])
        with PhysicsExperiment(poses=q, joints=dict(reversed(list(joints.items())))) as equivalent:
            self.assertEqual(expected, equivalent.config_sha256)
        with PhysicsExperiment(offsets=dict(right=[.01]*8)) as shifted:
            self.assertNotEqual(expected, shifted.config_sha256)

    def test_push_replaces_all_joint_torques_and_reset_clears_it(self):
        with PhysicsExperiment(**challenges()[1].config) as world:
            initial = copy.deepcopy(world.observation)
            identity = world.config_sha256
            world.push(dict(right=[.1]*7, left=[-.2]*7))
            self.assertEqual(world.truth()['plant']['applied_torque_nm'], dict(right=[.1]*7, left=[-.2]*7))
            world.push(dict(left=[.3]*7))
            self.assertEqual(world.truth()['plant']['applied_torque_nm'], dict(right=[0.]*7, left=[.3]*7))
            world.reset({s: arm['q'] for s, arm in initial['arms'].items()})
            self.assertEqual(world.truth()['plant']['applied_torque_nm'], dict(right=[0.]*7, left=[0.]*7))
            self.assertEqual(world.config_sha256, identity)

    def test_invalid_push_is_rejected_before_native_service_traffic(self):
        native = NativeSimulator.__new__(NativeSimulator)
        native.process = Mock()
        native.process.poll.return_value = None
        native.socket = Mock()
        native._receive = Mock(return_value=dict(state={}))
        native.config_sha256 = 'fixture'
        for value in (None, [], dict(typo=[0.]*7), dict(right=[0.]*8), dict(left=[float('nan')]*7)):
            with self.subTest(value=value), self.assertRaises(ValueError):
                native.push(value)
        native.socket.sendall.assert_not_called()
        result = native.push(dict(right=[.2]*7))
        sent = json.loads(native.socket.sendall.call_args.args[0])
        self.assertEqual(sent, dict(action='push', payload=dict(right=[.2]*7, left=[0.]*7)))
        self.assertEqual(result['config_sha256'], 'fixture')
        self.assertEqual(joint_torques({}), dict(right=[0.]*7, left=[0.]*7))

    def test_service_configuration_file_replaces_runtime_object(self):
        config = dict(simulator=dict(friction_scale=.5))
        self.assertEqual(load_configuration(config), dict(friction_scale=.5))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'plant.json'
            path.write_text(json.dumps(dict(joints={})))
            self.assertEqual(load_configuration(config, path), dict(joints={}))
            for value in ([], dict(unknown=1)):
                path.write_text(json.dumps(value))
                with self.assertRaises(ValueError):
                    load_configuration(config, path)


if __name__ == '__main__':
    unittest.main()
