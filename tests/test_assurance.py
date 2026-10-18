from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import pytest

from aegis_rt import Control, RuntimeAssuranceSupervisor
from aegis_rt.evidence import write_html, write_json
from aegis_rt.scenario import load_scenario


SCENARIO = Path(__file__).parents[1] / "scenarios" / "warehouse_crossing.json"


def test_crossing_triggers_lower_risk_control() -> None:
    decision = RuntimeAssuranceSupervisor().evaluate(load_scenario(SCENARIO))
    assert decision.intervened
    assert decision.applied_evaluation.max_collision_probability <= 0.05
    assert decision.applied_evaluation.max_collision_probability < decision.commanded_evaluation.max_collision_probability
    assert decision.candidates_evaluated == 26
    assert decision.fallback_controller == "lattice"
    assert decision.fallback_status == "feasible"


def test_open_space_preserves_planner_command() -> None:
    scenario = load_scenario(SCENARIO)
    scenario = replace(scenario, obstacles=(), commanded_control=Control(0.2, 0.1))
    decision = RuntimeAssuranceSupervisor().evaluate(scenario)
    assert not decision.intervened
    assert decision.applied_control == scenario.commanded_control
    assert decision.fallback_status == "not_required"


def test_cbf_fallback_handles_collision_course() -> None:
    scenario = load_scenario(SCENARIO)
    decision = RuntimeAssuranceSupervisor(fallback_controller="cbf").evaluate(scenario)

    assert decision.intervened
    assert decision.fallback_controller == "cbf"
    assert decision.fallback_status == "feasible"
    assert decision.applied_evaluation.max_collision_probability <= scenario.config.risk_budget
    assert decision.command_deviation >= 0.0


def test_cbf_reports_infeasible_near_boundary() -> None:
    scenario = load_scenario(SCENARIO)
    obstacle = replace(
        scenario.obstacles[0],
        position=replace(scenario.ego.position, x=0.2),
        velocity=replace(scenario.ego.position, x=0.0, y=0.0),
    )
    scenario = replace(scenario, obstacles=(obstacle,))

    decision = RuntimeAssuranceSupervisor(fallback_controller="cbf").evaluate(scenario)

    assert decision.intervened
    assert decision.fallback_status == "infeasible"
    assert "explicitly infeasible" in decision.reason


def test_unknown_fallback_is_rejected() -> None:
    with pytest.raises(ValueError):
        RuntimeAssuranceSupervisor(fallback_controller="unknown")


def test_decision_is_deterministic() -> None:
    scenario = load_scenario(SCENARIO)
    supervisor = RuntimeAssuranceSupervisor()
    first = supervisor.evaluate(scenario).to_dict()
    second = supervisor.evaluate(scenario).to_dict()
    assert first == second


def test_evidence_artifacts_are_written(tmp_path: Path) -> None:
    decision = RuntimeAssuranceSupervisor().evaluate(load_scenario(SCENARIO))
    json_path = write_json(decision, tmp_path / "decision.json")
    html_path = write_html(decision, tmp_path / "evidence.html")
    payload = json.loads(json_path.read_text(encoding="utf-8"))
    assert payload["intervened"] is True
    report = html_path.read_text(encoding="utf-8")
    assert "AEGIS RUNTIME ASSURANCE" in report
    assert "Feasibility" in report

