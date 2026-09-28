"""Guard checks must complete before any simulated controller can open CAN."""
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from openarm_simulator.native import NativeSimulator, require_virtual_interfaces


def link(kind='vcan',mtu=72):
    return SimpleNamespace(stdout=json.dumps([dict(mtu=mtu,linkinfo=dict(info_kind=kind))]))


class IsolationTests(unittest.TestCase):
    def test_either_physical_interface_refused_before_socket_open(self):
        for replies in ([link('can'),link()],[link(),link('can')],[link(mtu=16),link()]):
            with self.subTest(replies=replies), patch('openarm_simulator.interfaces.subprocess.run',side_effect=replies), \
                    patch('openarm_simulator.native.subprocess.Popen') as create:
                with self.assertRaisesRegex(RuntimeError,'refusing a physical interface'):
                    NativeSimulator()
                create.assert_not_called()

    def test_both_virtual_interfaces_required(self):
        with patch('openarm_simulator.interfaces.subprocess.run',side_effect=[link(),link()]) as inspect:
            require_virtual_interfaces()
        self.assertEqual([c.args[0][-1] for c in inspect.call_args_list],['can0','can1'])


if __name__=='__main__':
    unittest.main()
