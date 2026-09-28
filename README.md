# OpenArm simulator

MuJoCo physics with virtual OpenArm CAN motors. Both arms share one world;
normal motor commands and feedback use SocketCAN. A small HTTP API provides
simulator-only inspection, fault injection, external forces and world reset.

Requires Linux, Python 3.12, Rust, and an externally supplied **OpenArm v1 MJCF
scene with its referenced meshes**. Model assets are not bundled or downloaded.
See [setup](docs/setup.md) for dependencies and virtual CAN configuration.

```sh
uv sync --locked
uv run openarm-simulator --prepare
uv run openarm-simulator --model /path/to/openarm_mujoco/v1/scene.xml
```

The simulator requires CAN-FD-capable `vcan` interfaces named `can0` and `can1`
and refuses physical CAN interfaces. HTTP administration defaults to
`http://127.0.0.1:8080`. Use `--help` for startup configuration and report options.

```sh
curl http://127.0.0.1:8080/state
```

> **Limitation:** The mechanical model is approximate. Passing simulation does
> not establish real-world calibration accuracy or collision/force safety.

[HTTP API](docs/api.md) · [Model and protocol](docs/model.md) ·
[Friction experiments](friction.md) · [Extraction provenance](docs/extraction.md)
