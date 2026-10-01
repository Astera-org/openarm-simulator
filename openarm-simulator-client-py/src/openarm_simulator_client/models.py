"""Wire models for simulator administration. Motor commands are sent over CAN."""

from enum import StrEnum

from serde import serde, Untagged
from typing import Literal


class Integrator(StrEnum):
    EULER = "Euler"
    RK4 = "RK4"
    IMPLICIT = "implicit"
    IMPLICIT_FAST = "implicitfast"
    DISCRETE = "discrete"


class BodyIndex(int):
    pass


class JointIndex(int):
    pass


class ActuatorIndex(int):
    pass


class GeomIndex(int):
    pass


class SiteIndex(int):
    pass


Vector3 = tuple[float, float, float]
Solref = tuple[float, float]
Solimp = tuple[float, float, float, float, float]
# Persistent hinge-joint torques in Nm; omitted joints receive zero.
Push = dict[JointIndex, float]


@serde
class PushRequest:
    torques_nm: Push


@serde
class Advance:
    """Elapsed simulated nanoseconds; the clock must be paused."""

    duration_ns: int


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
    kp_nm_per_rad: float
    kd_nm_s_per_rad: float
    q_rad: float
    dq_rad_s: float
    tau_nm: float


@serde
class MappingRanges:
    """Motor PMAX/VMAX/TMAX encoding ranges, not physical joint/effort limits."""

    pmax_rad: float
    vmax_rad_s: float
    tmax_nm: float


@serde
class MotorState:
    id: int
    reply_id: int
    command: MotorCommand
    q_rad: float
    dq_rad_s: float
    torque_nm: float
    status: int
    mos_temperature_k: float
    rotor_temperature_k: float
    silent: bool
    ranges: MappingRanges


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
class HingeJointParameters:
    kind: Literal["hinge"] = "hinge"
    frictionloss_nm: float | None = None
    damping_nm_s_per_rad: float | None = None
    stiffness_nm_per_rad: float | None = None
    springref_rad: float | None = None
    stribeck: Stribeck | None = None


@serde
class SlideJointParameters:
    kind: Literal["slide"] = "slide"
    frictionloss_n: float | None = None
    damping_n_s_per_m: float | None = None
    stiffness_n_per_m: float | None = None
    springref_m: float | None = None


JointParameters = HingeJointParameters | SlideJointParameters


@serde
class BodyParameters:
    mass_kg: float
    com_m: Vector3
    inertia_kg_m2: Vector3


@serde
class Spring:
    """Spring and axial damper between two MJCF sites; separate from MJCF tendons.

    Acts in tension and compression. Coincident endpoints exert no force.
    """

    sites: tuple[SiteIndex, SiteIndex]
    rest_length_m: float
    stiffness_n_per_m: float
    damping_n_s_per_m: float


@serde
class AppliedForce:
    """Persistent force and torque in world axes, applied at an MJCF site."""

    site: SiteIndex
    force_world_n: Vector3
    torque_world_nm: Vector3


@serde
class SiteState:
    position_world_m: Vector3
    orientation_world_xyzw: tuple[float, float, float, float]


@serde
class BodyState:
    position_world_m: Vector3
    orientation_world_xyzw: tuple[float, float, float, float]
    com_world_m: Vector3


@serde
class SpringState:
    length_m: float
    velocity_m_s: float


@serde(tagging=Untagged)
class Plant:
    friction_model: str
    joints: dict[JointIndex, JointParameters]
    applied_torque_nm: Push


@serde
class State:
    state: dict[str, MotorState]
    statistics: Statistics
    time_ns: int
    paused: bool
    advancing: bool
    mujoco_version: str
    timestep_ns: int
    plant: Plant
    bodies: list[BodyState]
    sites: list[SiteState]
    springs: dict[str, SpringState]


@serde
class SceneNames:
    bodies: dict[str, BodyIndex]
    joints: dict[str, JointIndex]
    actuators: dict[str, ActuatorIndex]
    geoms: dict[str, GeomIndex]
    sites: dict[str, SiteIndex]


@serde(tagging=Untagged)
class PhysicsConfiguration:
    friction_model: str
    mujoco_version: str
    timestep_ns: int
    integrator: Integrator
    gravity_m_s2: Vector3
    enhanced_friction_solref: Solref
    enhanced_friction_solimp: Solimp
    joints: dict[JointIndex, JointParameters]
    bodies: list[BodyParameters]
    encoder_offsets_rad: dict[ActuatorIndex, float]


@serde
class Configuration:
    configuration: PhysicsConfiguration


@serde
class ErrorResponse:
    error: str
