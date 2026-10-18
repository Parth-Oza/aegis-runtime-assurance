"""Aegis runtime assurance for mobile robot control."""

from .assurance import RuntimeAssuranceSupervisor
from .command_guard import CommandGuard, CommandGuardConfig, GuardResult, VelocityCommand
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
    "CommandGuard",
    "CommandGuardConfig",
    "EmpiricalCalibration",
    "GuardResult",
    "Obstacle",
    "RobotState",
    "RuntimeAssuranceSupervisor",
    "SafetyConfig",
    "Scenario",
    "Vec2",
    "VelocityCommand",
]

__version__ = "0.1.0"

