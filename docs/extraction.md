# Extraction provenance

Extracted from `mickvangelderen/openarm-lab` commit
`0710d4e9e13f8dbd69ec05ab7e702fba3f9ed30d` into a fresh Git history.
The source checkout was left unchanged.

Included the `openarm-simulator` Rust engine, Python experiment fixtures,
friction documentation and tests. Also extracted the virtual-interface guard
from `openarm-client`, the required local IPC functions from `headquarters-ipc`,
and the geometric path checks used by gravity fixtures from `openarm-core`.
Protocol, isolation and Rust test launchers came from Lab's integration tests.
Controller, viewer and Headquarters applications are separate projects.

Dependencies resolve from package registries, with committed Cargo and uv
lockfiles. Model assets are supplied externally; the original model checkout
was `enactic/openarm_mujoco` at
`56e846b34d8a5bcea1bcebf93db5dc9da467d3c8` (v1 scene).

The extraction adds HTTP administration and explicit model selection. Model
identity hashes the compiled MuJoCo model, including geometry, so its hashes
differ from Lab's former two-XML-file identity. Fixture seeds and friction
challenge definitions are preserved.
