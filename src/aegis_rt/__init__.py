"""Aegis runtime assurance for mobile robot control."""

from .assurance import RuntimeAssuranceSupervisor
from .models import (
    AssuranceDecision,
    Control,
    EmpiricalCalibration,
    Obstacle,
    RobotState,
    SafetyConfig,
    Scenario,
    Vec2,
)

__all__ = [
    "AssuranceDecision",
    "Control",
    "EmpiricalCalibration",
    "Obstacle",
    "RobotState",
    "RuntimeAssuranceSupervisor",
    "SafetyConfig",
    "Scenario",
    "Vec2",
]

__version__ = "0.1.0"

