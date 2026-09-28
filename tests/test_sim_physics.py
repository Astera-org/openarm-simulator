"""Native physics/protocol/guard units; no CAN sockets or hardware access."""
import unittest
from openarm_simulator.native import cargo


class NativeUnitTests(unittest.TestCase):
    def test_rust_units(self):
        cargo(['test', '--quiet'])
