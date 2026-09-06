from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from aegis_rt.models import SafetyConfig
from aegis_rt.predictor import collision_probability, rollout
from aegis_rt.scenario import load_scenario


SCENARIO = Path(__file__).parents[1] / "scenarios" / "warehouse_crossing.json"


def test_collision_probability_is_monotonic_in_separation() -> None:
    near = collision_probability(0.5, 1.0, 0.2)
    far = collision_probability(2.0, 1.0, 0.2)
    assert near > far


def test_rollout_respects_speed_bounds() -> None:
    scenario = load_scenario(SCENARIO)
    fast = replace(scenario.commanded_control, acceleration_mps2=100.0)
    points = rollout(scenario.ego, fast, scenario.config)
    assert all(0.0 <= point.state.speed_mps <= scenario.config.max_speed_mps for point in points)


def test_invalid_risk_budget_is_rejected() -> None:
    with pytest.raises(ValueError):
        SafetyConfig(risk_budget=1.2)

