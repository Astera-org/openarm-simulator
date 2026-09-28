# Simulator model and protocol

Both arms share one MuJoCo world. Motors start disabled, with the arms at physical
zero and gripper motors at −10°. Gravity acts while idle; enabling, holding and
disabling use the normal controller. A simulator restart resets the world, while
an administration client disconnect leaves it running.

> **Limitation:** Successful simulation exercises software integration and
> approximate mechanics. It cannot establish real-world calibration accuracy
> or collision/force safety.

## Build and runtime

The physics/protocol endpoint is Rust (this repository); install current Rust/Cargo,
a C compiler and libclang development headers (`libclang-dev` on Debian/Ubuntu).
The uv launcher builds it with the committed Cargo lockfile before starting
the physics process. First build needs registry access; later launches reuse
build outputs.
`bindgen` generates the C interface from uv's installed MuJoCo headers and links
that same shared library. Runtime/header versions must match. No separate physics
engine version is downloaded or installed.

## Processes and isolation

The [HTTP administration API](api.md) runs in Python. Motor commands and feedback
use CAN between a separate controller and the Rust simulator. Both Python and
Rust verify that can0/can1 are CAN-FD-capable vcan before simulator use. See
[setup](setup.md) for an isolated namespace or host virtual buses.

The Rust simulator and Python controller run in separate processes. One Rust
thread owns the complete model, motor state and virtual CAN sockets; neither
Python nor a shared-state mutex participates in its timed loop. Linux `timerfd`
paces physics against monotonic wall time with a **0.5 ms** step and `implicitfast`
integration. `poll` waits for timer, CAN or administrative messages. Each received
MIT packet changes the held motor command. MuJoCo
applies that command on every subsequent physics step; it never replaces joint
positions with commanded values. Incoming work is bounded so packet traffic
cannot indefinitely starve physics. A delay over one second stops the simulator
instead of silently skipping physics time. This is a simulation overload check,
not an added hardware motion limit. Late wakeups run all elapsed steps before
accepting new commands, so new commands are not applied retroactively to those
steps. This remains ordinary Linux scheduling, not a hard real-time guarantee.

The Python launcher handles child-process cleanup and administration. A private
inherited Unix `SEQPACKET` socket carries startup, inspection, reset, pushes and
fault injection; JSON does not carry per-step commands or feedback.
Closing that socket stops the native process, including after a launcher crash.

Stop CAN command senders before intentionally resetting or restarting the world.

## Protocol contract

The reference is the pinned
[official encoder/decoder](https://github.com/enactic/openarm_can/blob/f340d4b808fb177e1f297af54eb55fd51c6c7c10/src/openarm/damiao_motor/dm_motor_control.cpp),
[motor scale definitions](https://github.com/enactic/openarm_can/blob/f340d4b808fb177e1f297af54eb55fd51c6c7c10/include/openarm/damiao_motor/dm_motor_constants.hpp),
and [Damiao manual](https://damiao.enactic.ai/en/products/hardware/dm-j4340p-2ec-v1.0/).
Command/feedback tests compare with the actual installed C++ encoder and decoder,
plus literal known packets; integration tests use real Linux SocketCAN sockets.

| Operation | Behavior |
| --- | --- |
| MIT command, IDs 1–8 | Decode q, dq, Kp, Kd and feedforward torque; retain the command until replaced |
| Enable / disable | `FF FF FF FF FF FF FF FC` / `… FD`; disabled actuators produce no drive torque |
| Clear error | `FF FF FF FF FF FF FF FB`; clear the injected fault and leave the motor disabled |
| Refresh | ID `0x7ff`, motor ID little-endian, `0xcc`; return current state |
| State reply | IDs 17–24; 16-bit position, 12-bit velocity/torque, status nibble and temperatures |
| Read register | `0x33`: master ID (7), motor ID (8), TIMEOUT (9), mode (10), P/V/T scales (21–23) |
| Write register | `0x55`: only mode=MIT (register 10, value 1) |
| Unsupported requests | Counted/rejected; no success reply |

Both classic and FD input frames are accepted; replies use CAN-FD with BRS.
Register integers/floats use little-endian 32-bit values. The driver supplies the
motor families/scales: DM8009 for J1–J2, DM4340 for J3–J4, DM4310 for J5–J8.
Quantization and saturation use the same ranges as `openarm_can`.

Zero-offset writes, persistent saves, unsupported registers, and
POS_VEL/VEL/POS_FORCE are **not implemented**. This supports this
project's MIT diagnostics, not every manufacturer utility. Injected fault codes
remain latched until the controller sends clear-error or the test fixture clears
or resets them; enable/disable do not erase an injected fault. Temperatures are a fixed 25°C. TIMEOUT reads as zero and
no drive watchdog is emulated. Feedback is returned in response to commands or
refreshes; there is no unsolicited periodic publisher.

## Physics assumptions

- MIT drive torque is `Kp*(q_ref-q_encoder) + Kd*(dq_ref-dq) + tau_ff`, implemented
  with MuJoCo affine actuators so damping is integrated implicitly. Torque feedback
  comes from the resulting saturated actuator force, not from a fabricated load or
  contact flag. Gravity, coupled inertias, friction and contacts produce the motion.
- Masses, inertias, joint ranges, friction, meshes and collision exclusions come
  from the externally supplied model. The original v1 scene has arm force
  envelopes of ±40/27/7 Nm. The gripper
  uses ±7 Nm from the modeled DM4310 family. These are simulation envelopes, **not
  measurements or guarantees of the real firmware's current/torque limits**.
- Each gripper uses the upstream symmetric fixed tendon and finger equality.
  `q_motor = -mean(finger displacement)/(0.044/1.0472)`. One actuator drives that
  tendon with reciprocal gearing, preserving virtual work and sending half the
  corresponding linear force to each finger. This shares the existing linear
  motor-to-finger position approximation; actual transmission geometry may differ.
- The same two overlapping finger/finger collision pairs already excluded by the
  zero-path checker are excluded here. Other upstream collision pairs remain active.
- Default XML joint-limit compliance allowed unrealistic stop penetration under
  calibration load. Numerical hard stops use `solref=[0.002,1]` and
  `solimp=[0.99,0.999,0.001,0.5,2]`. Penetration remains finite and is visible in
  feedback. This stiffness is **not fitted to physical stop elasticity**.
- The model has ideal instantaneous current/torque response: no electrical current
  loop, thermal model, gearbox backlash, motor torque ripple, USB latency, CAN
  arbitration/bandwidth model or bus-off dynamics. Host scheduling and driver
  queues are real. Dropped replies and fault codes can be injected in tests.
- Encoder biases can be set independently of physical starting poses through
  `NativeSimulator(poses=..., offsets=...)` in the integration fixture. Arrays
  contain eight radians per arm.

Successful simulation exercises software integration and approximate mechanics.
It cannot establish real-world calibration accuracy or collision/force safety.

## Identification experiments

`PhysicsExperiment` uses this same Rust physics implementation in deterministic
lockstep, without opening CAN sockets. It passes commands and feedback through
the official driver's encoder/decoder and the native motor protocol. Its explicit
`truth()` channel is for scoring only; observation packets contain no mass/CoM or
gravity truth. This allows many simulated poses to run faster than wall time.

Both `PhysicsExperiment` and `NativeSimulator` accept simulator-only `bodies`
overrides (mass, local center of mass and principal inertia), `friction_scale`, and
per-arm-joint `joints` overrides for frictionloss, damping, stiffness and springref.
Overrides are validated and retained across fixture resets. The supplied model
files stay unchanged. The ordinary service uses the same defaults.

Optional per-joint Stribeck, angle-dependent and asymmetric friction, external
joint pushes, managed-service plant configuration and their numerical limits
are described in [friction experiments](../friction.md). These are synthetic test
models, not measured OpenArm properties.

The gravity and friction fixtures remain available for local deterministic
physics checks. Set `OPENARM_SIMULATOR_MODEL` before calling their generators.
`PhysicsExperiment(model=...)` and `NativeSimulator(model=...)` also accept an
explicit scene path. The experiment harness is a local test fixture, not a
network control interface.

## Reports and tests

See [setup and verification](setup.md). The standalone suite includes Rust
physics/protocol/guard units and comparison with the official C++ codec, plus
an isolated HTTP/SocketCAN check. Controller/UI end-to-end audits remain in Lab.

`--report-dir PATH` saves `simulator.json` on shutdown: model identity, MuJoCo
version, effective plant, packet counts, scheduling lag and final motor state.
It is an end-of-run summary, not a continuously saved physics trace.
