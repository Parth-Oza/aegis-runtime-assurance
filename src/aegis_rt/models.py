from __future__ import annotations

from dataclasses import asdict, dataclass, field
from math import hypot
from typing import Any


@dataclass(frozen=True)
class Vec2:
    x: float
    y: float

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vec2":
        return Vec2(self.x * scalar, self.y * scalar)

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.y * other.y

    def norm(self) -> float:
        return hypot(self.x, self.y)


@dataclass(frozen=True)
class RobotState:
    position: Vec2
    heading_rad: float
    speed_mps: float


@dataclass(frozen=True)
class Control:
    acceleration_mps2: float
    yaw_rate_rps: float


@dataclass(frozen=True)
class Obstacle:
    obstacle_id: str
    position: Vec2
    velocity: Vec2
    radius_m: float = 0.45
    position_sigma_m: float = 0.15
    acceleration_sigma_mps2: float = 0.2


@dataclass(frozen=True)
class ResidualBin:
    horizon_s: float
    residuals_m: tuple[float, ...]
    quantile_m: float


@dataclass(frozen=True)
class EmpiricalCalibration:
    schema_version: int
    source: str
    target_coverage: float
    empirical_coverage: float
    calibration_error: float
    sample_count: int
    bins: tuple[ResidualBin, ...]


@dataclass(frozen=True)
class SafetyConfig:
    horizon_s: float = 5.0
    step_s: float = 0.1
    risk_budget: float = 0.05
    robot_radius_m: float = 0.45
    safety_margin_m: float = 0.25
    localization_sigma_m: float = 0.08
    max_speed_mps: float = 4.0
    max_braking_mps2: float = 2.5
    max_yaw_rate_rps: float = 1.2
    reaction_time_s: float = 0.2

    def __post_init__(self) -> None:
        if self.horizon_s <= 0 or self.step_s <= 0:
            raise ValueError("horizon_s and step_s must be positive")
        if not 0 < self.risk_budget < 1:
            raise ValueError("risk_budget must be between zero and one")
        if self.max_braking_mps2 <= 0 or self.max_speed_mps <= 0:
            raise ValueError("motion limits must be positive")


@dataclass(frozen=True)
class Scenario:
    name: str
    ego: RobotState
    commanded_control: Control
    obstacles: tuple[Obstacle, ...]
    config: SafetyConfig = field(default_factory=SafetyConfig)
    calibration: EmpiricalCalibration | None = None


@dataclass(frozen=True)
class TrajectoryPoint:
    time_s: float
    state: RobotState


@dataclass(frozen=True)
class RiskSample:
    time_s: float
    obstacle_id: str
    separation_m: float
    clearance_m: float
    uncertainty_sigma_m: float
    collision_probability: float
    uncertainty_model: str


@dataclass(frozen=True)
class CandidateEvaluation:
    control: Control
    max_collision_probability: float
    tail_risk: float
    minimum_clearance_m: float
    earliest_risk_time_s: float | None
    trajectory: tuple[TrajectoryPoint, ...]
    risk_samples: tuple[RiskSample, ...]


@dataclass(frozen=True)
class AssuranceDecision:
    scenario_name: str
    commanded_control: Control
    applied_control: Control
    intervened: bool
    reason: str
    commanded_evaluation: CandidateEvaluation
    applied_evaluation: CandidateEvaluation
    candidates_evaluated: int
    fallback_controller: str
    fallback_status: str
    command_deviation: float
    counterfactual: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

