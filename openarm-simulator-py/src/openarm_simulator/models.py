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
class Reset:
    """Reset time, motion and faults. Poses are radians; joint 8 is the gripper motor.

    An omitted arm resets to the simulator's default pose, not its current pose.
    """

    left: Pose | None = None
    right: Pose | None = None


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
class MotorLimits:
    position: float
    velocity: float
    torque: float


@serde
class MotorState:
    joint: int
    command: MotorCommand
    q: float
    dq: float
    torque: float
    status: int
    temperature: int
    silent: bool
    limits: MotorLimits


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
    rejected: int
    dropped: int
    steps: int
    max_lag_ms: float
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
    time: float
    mujoco_version: str
    timestep_s: float
    plant: Plant
    config_sha256: str
    joint_stop_solref: Solref
    joint_stop_solimp: Solimp


@serde
class PhysicsConfiguration:
    model_sha256: str
    friction_model: str
    mujoco_version: str
    timestep_s: float
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
    config_sha256: str


@serde
class ErrorResponse:
    error: str
