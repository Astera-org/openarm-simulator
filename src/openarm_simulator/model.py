"""Externally supplied OpenArm v1 MJCF scene and its compiled geometry."""
import hashlib
import os
from pathlib import Path


def model_path(path=None):
    path = path or os.environ.get('OPENARM_SIMULATOR_MODEL')
    if not path:
        raise ValueError('Supply --model PATH or set OPENARM_SIMULATOR_MODEL to an OpenArm v1 scene.xml')
    path = Path(path).expanduser().resolve(strict=True)
    if not path.is_file():
        raise ValueError(f'Model must be an MJCF file: {path}')
    return path


def model_sha256(path):
    # MuJoCo resolves includes and meshes; hashing its binary includes collision
    # geometry without assuming particular XML filenames or directory layouts.
    import mujoco
    import numpy as np
    model = mujoco.MjModel.from_xml_path(str(path))
    buffer = np.empty(mujoco.mj_sizeModel(model), dtype=np.uint8)
    mujoco.mj_saveModel(model, buffer=buffer)
    return hashlib.sha256(buffer).hexdigest()
