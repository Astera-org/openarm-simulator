"""Wire models for simulator administration. Motor commands are sent over CAN."""

from typing import Literal

from serde import serde

Arm = Literal["left", "right"]
Vector3 = tuple[float, float, float]
Pose = tuple[float, float, float, float, float, float, float, float]
JointTorques = tuple[float, float, float, float, float, float, float]
Solref = tuple[float, float]
Solimp = tuple[float, float, float, float, float]


@serde
class Advance:
    """Elapsed simulated nanoseconds; the clock must be paused."""

    duration_ns: int


@serde
class Push:
    """Persistent external joint torques in Nm; omitted arms receive zero torque."""

    left: JointTorques | None = None
    right: JointTorques | None = None


@serde
class Fault:
    """Patch a motor's fault state. None leaves a field unchanged.

    status=0 clears the fault and disables the motor; enable it through CAN.
    silent=False restores replies independently of status.
    """

    status: int | None = None
    silent: bool | None = None


@serde
class MotorCommand:
    kp: float
    kd: float
    q: float
    dq: float
    tau: float


@serde
class MappingRanges:
    """Motor PMAX/VMAX/TMAX encoding ranges, not physical joint/effort limits."""

    pmax: float
    vmax: float
    tmax: float


@serde
class MotorState:
    joint: int
    command: MotorCommand
    q: float
    dq: float
    torque: float
    status: int
    mos_temperature: int
    rotor_temperature: int
    silent: bool
    ranges: MappingRanges


@serde
class ArmStates:
    left: list[MotorState]
    right: list[MotorState]


@serde
class ArmTorques:
    left: JointTorques
    right: JointTorques


@serde
class ArmPoses:
    left: Pose
    right: Pose


@serde
class Statistics:
    commands: int
    replies: int
    dropped: int
    steps: int
    max_lag_ns: int
    max_catchup_steps: int


@serde
class AngleModulation:
    amplitude: float
    harmonic: int
    phase_rad: float


@serde
class Stribeck:
    breakaway_nm: float
    velocity_rad_s: float
    direction_asymmetry: float = 0.0
    angle: AngleModulation | None = None


@serde
class JointParameters:
    frictionloss: float | None = None
    damping: float | None = None
    stiffness: float | None = None
    springref: float | None = None
    stribeck: Stribeck | None = None


@serde
class BodyParameters:
    mass: float
    com: Vector3
    inertia: Vector3


@serde
class Plant:
    friction_model: str
    joints: dict[str, JointParameters]
    applied_torque_nm: ArmTorques


@serde
class State:
    state: ArmStates
    statistics: Statistics
    time_ns: int
    paused: bool
    advancing: bool
    mujoco_version: str
    timestep_ns: int
    plant: Plant
    joint_stop_solref: Solref
    joint_stop_solimp: Solimp


@serde
class PhysicsConfiguration:
    friction_model: str
    mujoco_version: str
    timestep_ns: int
    integrator: str
    gravity_m_s2: Vector3
    joint_stop_solref: Solref
    joint_stop_solimp: Solimp
    enhanced_friction_solref: Solref
    enhanced_friction_solimp: Solimp
    gripper_radius_m: float
    joints: dict[str, JointParameters]
    bodies: dict[str, BodyParameters]
    encoder_offsets_rad: ArmPoses


@serde
class Configuration:
    configuration: PhysicsConfiguration


@serde
class ErrorResponse:
    error: str
