# Setup and checks

Use this source checkout with [uv](https://docs.astral.sh/uv/) and a
[Rust toolchain](https://rustup.rs/). On Debian/Ubuntu:

```sh
sudo apt install build-essential cmake ninja-build libcli11-dev libclang-dev iproute2 util-linux
uv sync --locked
uv run openarm-simulator --prepare
```

The first build downloads Python and Cargo dependencies. The Python MuJoCo
package supplies both headers and the shared library; a separate MuJoCo install
is unnecessary. Run from the checkout; the Python launcher builds the Rust crate
here. Standalone wheel distribution is not supported.

## Supply a model

Pass `--model PATH` or set `OPENARM_SIMULATOR_MODEL` to an OpenArm v1 MJCF scene.
The model must retain the v1 joint, actuator, tendon and body names expected by
the engine. Its includes, collision meshes and visual meshes are resolved by
MuJoCo relative to the supplied model. Changing to a different robot topology
requires engine changes.

```sh
export OPENARM_SIMULATOR_MODEL=/absolute/path/to/openarm_mujoco/v1/scene.xml
```

The original model is `enactic/openarm_mujoco` at revision
`56e846b34d8a5bcea1bcebf93db5dc9da467d3c8`. The simulator never fetches it.
Configuration identity includes a hash of the compiled model, including its
geometry, plus the effective plant overrides and numerical settings.

## Virtual CAN

The kernel needs the `vcan` module (`sudo modprobe vcan` if needed). Both buses
must be virtual CAN with MTU 72. `can0` is the right arm; `can1` is the left.

For an isolated session, open a private network namespace:

```sh
unshare --user --map-root-user --net bash
ip link set lo up
ip link add can0 type vcan && ip link set can0 mtu 72 up
ip link add can1 type vcan && ip link set can1 mtu 72 up
.venv/bin/openarm-simulator --model "$OPENARM_SIMULATOR_MODEL" &
simulator_pid=$!
curl http://127.0.0.1:8080/state
# Run the CAN controller and other clients from this same shell/namespace.
kill "$simulator_pid"
wait "$simulator_pid"
exit
```

User/network namespaces must be enabled. Loopback HTTP is also private to that
namespace. For host-accessible HTTP, run the simulator in the host namespace
with two existing virtual buses, or create them with administrator privileges:

```sh
sudo ip link add can0 type vcan && sudo ip link set can0 mtu 72 up
sudo ip link add can1 type vcan && sudo ip link set can1 mtu 72 up
uv run openarm-simulator --model "$OPENARM_SIMULATOR_MODEL"
```

If either name is occupied by physical CAN, use a private namespace instead.
The simulator does not create, rename, or configure interfaces itself.

## Verification

With the external model environment variable set:

```sh
uv run python -m unittest discover -s tests -v
```

This includes the Rust physics/protocol tests and comparison with the official
Python CAN codec, without opening CAN sockets. Run the end-to-end check inside
the isolated namespace after setting up its two buses:

```sh
.venv/bin/python tests/check_socketcan.py
```

It verifies actual CAN-FD motor commands on both buses, HTTP administration,
invalid requests, reset and shutdown metadata. It refuses physical interfaces.
