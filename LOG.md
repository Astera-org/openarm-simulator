# Implementation obstacles

## Configured mapping ranges and register precision

**ISSUE:** Startup configuration accepts the existing f64 mapping-range type,
but the emulated PMAX/VMAX/TMAX registers return f32. A value such as 0.1 could
otherwise produce different scaling in the controller and a client that reads
its registers.

**SOLUTION DIRECTIONS:** Reject values not exactly representable in f32, change
the reusable codec's types, or normalize initial register values to f32.

**MOTIVATION FOR CHOSEN OPTION:** Normalize once when constructing the controller,
matching CAN register writes. This preserves the codec API and accepts ordinary
configuration values while keeping reported registers and actual scaling equal.

## Supplying the OpenArm mechanical model

**ISSUE:** The pinned upstream scene has two finger actuators per gripper. The
old runtime rewrote that model and its contact/joint-stop settings on every load.
Simply removing the rewrite changes the robot's behavior.

**SOLUTION DIRECTIONS:** Retain an OpenArm-specific runtime branch, transform XML
with a custom patching mechanism, or supply an adapted MJCF definition.

**MOTIVATION FOR CHOSEN OPTION:** Supply `models/openarm-v1.xml`, preserving the
upstream license and revision and expressing the existing mechanical choices in
MJCF. The test-model builder combines it with cached upstream meshes in its own
output directory. Arbitrary runtime models are loaded without robot-specific
surgery, and neither meshes nor cached source files are copied or modified.

## Transmission observations after integration

**ISSUE:** MuJoCo advances positions during `mj_step`, but derived actuator
length/velocity observations need updating before feedback. Reading them directly
would introduce stale feedback when replacing the old joint-index calculations.

**SOLUTION DIRECTIONS:** Reimplement transmission kinematics, expose stale values,
or run MuJoCo's forward calculation before reading observations.

**MOTIVATION FOR CHOSEN OPTION:** Run the existing forward calculation after an
advance batch. MuJoCo remains responsible for geared/coupled transmissions. The
existing gripper, holding, friction, and batched-step regressions pass unchanged
in their physical assertions.
