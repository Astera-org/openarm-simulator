# Optional joint-friction experiments

The default stays MuJoCo's dry joint friction plus viscous damping. Add
`stribeck` only to joints that need the richer synthetic model:

```json
{
  "joints": {
    "openarm_right_joint4": {
      "frictionloss": 0.15,
      "damping": 0.4,
      "stribeck": {
        "breakaway_nm": 0.4,
        "velocity_rad_s": 0.035,
        "direction_asymmetry": 0.15,
        "angle": {"amplitude": 0.2, "harmonic": 2, "phase_rad": 0.3}
      }
    }
  }
}
```

Omit `angle` and `direction_asymmetry` for symmetric Stribeck friction. Omit
`stribeck` for basic friction. Only J1–J7 arm hinges support these overrides;
gripper transmission friction is unchanged. `frictionloss` is dry sliding torque
in Nm, `damping` is Nm·s/rad, and all angles/velocities use radians. Explicit
values replace `friction_scale`-scaled defaults. Existing `stiffness` (Nm/rad)
and `springref` (rad) describe separate passive springs, which can store energy.

Use the JSON file with the simulator:

```sh
uv run openarm-simulator --model /path/to/v1/scene.xml --config /path/to/plant.json
```

The simulator service also accepts `--config PATH`, or a `simulator` object in
its runtime configuration. A file replaces that object. Configuration is read
only at simulator startup; changing it requires restarting the simulator.
`NativeSimulator(joints=...)` and `PhysicsExperiment(joints=...)` use the same
Rust implementation. Restart creates a new motor/physics state. A lockstep
`reset(poses)` preserves the configured plant but clears commands, motor faults
and external pushes.

Resolved body/joint parameters, encoder offsets, compiled model identity, MuJoCo version
and numerical settings form a canonical configuration hash. In Lab-managed mode the service
atomically publishes `simulator-physics.json` before announcing readiness, and
inspection/report metadata includes the same `config_sha256`. Initial pose,
elapsed time and transient pushes do not change this plant identity.

## Model and numerical interpretation

The dry-friction magnitude bound is

```
[Fc + (Fs−Fc) exp(−(v/vs)²)]
× [1 + a tanh(v/vs)]
× [1 + A cos(h q + phase)]
```

`Fc` is `frictionloss`; `Fs` is `breakaway_nm`; `vs` is `velocity_rad_s`.
Require `Fs ≥ Fc ≥ 0`, `vs > 0`, `|a| < 1`, `0 ≤ A < 1`, finite values, and a
positive integer harmonic. Positive `a` gives greater resistance at positive
velocity. Asymmetry blends toward the mean breakaway at exact rest. The angle
uses the physical hinge coordinate, independent of encoder offsets. Viscous
damping is separate and is not modulated. The dry bound always stays nonnegative
and below `Fs(1+|a|)(1+A)`.

This bound is evaluated at each physics step; MuJoCo determines the opposing
friction torque through its native constraint. There is no explicit discontinuous
sign-force integrator or hidden bristle state. MuJoCo documents dry friction as
a bounded force constraint, including its regularization.
[MuJoCo computation](https://mujoco.readthedocs.io/en/latest/computation/#friction-loss)

Enhanced joints use friction `solref=[0.002,1]` and
`solimp=[0.999,0.999,0.001,0.5,2]`. The upstream softer settings allow tiny wrist
inertias to creep through the Stribeck band under a nominally sub-breakaway load.
The firmer settings make static resistance better resolved, but still allow
finite numerical creep. Basic joints retain upstream friction solver settings.
This numerical difference is part of the recorded model, not hardware calibration.

Tests check positive bounds, friction work/energy dissipation in the coupled
robot, sub-breakaway resistance, sliding, deterministic batching, resets and
0.25/0.5/1 ms steps. With severe square torque pulses, the final wrist angles
must agree within 5% across that fourfold step range; stick/slip transition
timing is not exact. The normal service continues to use 0.5 ms `implicitfast`.

Position dependence and direction asymmetry are motivated by robot-joint
measurements. The bounded multiplicative forms above are deliberately simple
approximations, not reproductions of the fitted models or parameters in these
papers. [Xiao et al., 2018](https://journals.sagepub.com/doi/10.1177/1729881418788992),
[Elhami and Brookfield, 1997](https://www.sciencedirect.com/science/article/pii/S0005109896001835).

No temperature/load-dependent friction, presliding hysteresis, dwell-time aging,
gearbox backlash, torque ripple, elastic transmission or thermal/current-loop
dynamics are added. The model cannot establish actual hardware guiding quality.

## External pushes and independent fixtures

`PhysicsExperiment.push({"right": [seven torques]})` or
`NativeSimulator.push(...)` replaces persistent external torques in Nm.
Omitted arms become zero; `push({})` clears all. Native service sidechannel:
`{"action":"push","payload":{"right":[0,0,0,0,0,0,0.3]}}`.
These are generalized joint torques, not Cartesian hand forces. They affect
physics but never alter the held motor command or fabricate motor torque feedback.

`openarm_simulator.friction_challenges.challenges()` provides five independent
cases: basic, symmetric Stribeck, angle/direction mismatch, strong breakaway,
and passive cable resistance. Ranges are documented in that module and are
synthetic, not measured OpenArm tolerances. The suite was frozen before scoring
the controller: `independent-friction-v1`, source SHA256
`763607344f8a0820f27a982014358cb31dc6ebe70344f7729e90995be072f054`.
Changing a fixture after seeing its score requires a new benchmark version.
The original gravity challenge suite is unchanged.

Simulator-only verification (set `OPENARM_SIMULATOR_MODEL` first):

```sh
uv run python -c 'from openarm_simulator.native import cargo; cargo(["test"])'
uv run python -m unittest discover -s tests -v
```
