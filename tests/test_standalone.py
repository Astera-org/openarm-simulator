"""External model resolution and geometry identity, independent of Lab paths."""
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from openarm_simulator.model import model_path, model_sha256


class ModelTests(unittest.TestCase):
    def test_external_path_and_compiled_identity(self):
        with tempfile.TemporaryDirectory() as directory, patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(ValueError, 'Supply --model'):
                model_path()
            root = Path(directory)
            scene = root/'scene.xml'
            body = root/'body.xml'
            scene.write_text('<mujoco><include file="body.xml"/></mujoco>')
            body.write_text('<mujoco><worldbody><geom type="sphere" size="1"/></worldbody></mujoco>')
            os.environ['OPENARM_SIMULATOR_MODEL'] = str(scene)
            self.assertEqual(model_path(), scene)
            with self.assertRaises(ValueError):
                model_path(root)
            with self.assertRaises(FileNotFoundError):
                model_path(root/'absent.xml')
            identity = model_sha256(model_path())
            self.assertEqual(identity, model_sha256(scene))
            copy = root/'renamed.xml'
            copy.write_bytes(scene.read_bytes())
            self.assertEqual(model_path(copy), copy)
            self.assertEqual(identity, model_sha256(copy))
            body.write_text(body.read_text().replace('size="1"', 'size="2"'))
            self.assertNotEqual(identity, model_sha256(scene))


if __name__ == '__main__':
    unittest.main()
