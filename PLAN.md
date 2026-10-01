# Motor and mechanical simulation separation

The Damiao simulator declares its motor configuration. The application supplies
that configuration together with CAN bus assignments and scene bindings. MJCF
describes the mechanical scene. Keep one executable; reusable libraries do not
require separate processes or dynamic plugins.

Each phase must leave a working system and be committed separately. Record
unexpected obstacles in LOG.md with ISSUE, SOLUTION DIRECTIONS, and MOTIVATION
FOR CHOSEN OPTION.

## Phases

- [x] 1. Rename `openarm-can-rs` to `damiao-can-rs`. Update consumers without
  changing packet behavior or existing tests.
- [x] 2. Extract `damiao-simulator-rs`. Declare `MotorConfig` for implemented
  controller settings; remove OpenArm joint-number construction and dependencies
  on administration models. Move OpenArm presets into the application. Controller
  unit tests must run without MuJoCo, model downloads, or sockets.
- [x] 3. Separate motor ownership from mechanics. The runtime owns motors and
  scene independently and exchanges mechanical observations and actuation.
  Preserve implicit proportional/derivative integration, encoder conventions,
  clock ownership, and startup/reset equivalence. Controllers receive logical
  time when their implemented behavior needs it; do not add unused timing APIs.
- [x] 4. Accept named motor configurations through startup configuration. Embed
  the Damiao configuration directly alongside bus and scene bindings. Validate
  controller settings, address conflicts, and bindings at their respective
  boundaries. Reset restores initial controller settings. Motor names, CAN
  addresses, and scene names remain distinct identities.
- [x] 5. Remove fixed arm arrays and scene-name assumptions. Resolve configured
  actuator/transmission bindings; move OpenArm gripper/contact adjustments into
  the supplied model. Only mapped actuators are adapted for motor emulation.
  Test a small scene with different names and motor count, retaining OpenArm
  physical regression coverage.

## Boundaries

| Component | Responsibility |
| --- | --- |
| `damiao-can-rs` | Protocol types and byte codecs |
| `damiao-simulator-rs` | Controller configuration, registers, commands, faults, feedback, and drive behavior |
| `mujoco-sys-rs`, `mujoco-rs` | Native bindings and safe MuJoCo access |
| `openarm-simulator-rs` | Runtime, clock, sockets, HTTP, motor/scene bindings, and configuration |
| `openarm-simulator-core-rs` | Shared administration models |
| Rust and Python clients | Administration clients |

Motor configuration contains no socket/interface or MuJoCo object names.
Mechanical integration contains no CAN registers, addresses, or status codes.
The mechanical boundary preserves the existing drive law rather than silently
replacing it with explicitly sampled torque. Integer durations and whole physics
updates remain the clock contract. Reset takes no arguments and leaves time
paused. Normal motor commands continue to use CAN.

## Validation

Run the relevant package tests after each phase, then the workspace and client
checks after the completed integration. Keep codec tests as unit tests. Verify
controller unit tests build without MuJoCo/model downloads.
Use virtual CAN only for socket integration tests.

## Deferred

Motor-only executable operation and moving socket tests off MuJoCo are not a
priority and are excluded from this implementation. Preserve the ability to use
the motor library independently through its dependency boundary and mechanical
interface; do not add a standalone mode or backend framework for that purpose.

Generic scene inspection, pose editing, disturbances, additional motor protocols,
dynamic plugins, and a general physics-plugin framework are separate work.

## Using the extracted configuration

Supply `--model /path/to/scene.xml --config openarm-simulator-rs/config/openarm-v1.json`
for OpenArm. The supplied `openarm-simulator-rs/models/openarm-v1.xml` expects the
pinned upstream `v1/meshes` directory beside it; copy the XML into that external
model directory and load it directly, or include it from your scene. Tests stage
the XML against cached meshes automatically. No configuration means no emulated
motors or CAN buses.

The OpenArm preset retains the previous motor assignments and mapping ranges
(DM8009: 12.5/45/54, DM4340: 12.5/10/28, DM4310: 12.5/30/10), inherited from
`enactic/openarm_can`'s motor presets. Command/reply IDs follow
`enactic/openarm_ros2@4e837e1d0dae692ff67b560b69d8d281d7a8d4ed`,
`openarm_hardware/include/openarm_hardware/openarm_simple_hardware.hpp`.

Use repeatable `--can-interface BUS=INTERFACE` or `--can-fd BUS=FD` for configured
buses. State entries and `/fault` identify motors by configuration name;
`/push` accepts a map from scene hinge-joint names to torques in Nm. Startup
`positions` likewise uses scene hinge/slide names. Both clients use these models.
