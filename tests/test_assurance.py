from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

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


def test_open_space_preserves_planner_command() -> None:
    scenario = load_scenario(SCENARIO)
    scenario = replace(scenario, obstacles=(), commanded_control=Control(0.2, 0.1))
    decision = RuntimeAssuranceSupervisor().evaluate(scenario)
    assert not decision.intervened
    assert decision.applied_control == scenario.commanded_control


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
    assert "AEGIS RUNTIME ASSURANCE" in html_path.read_text(encoding="utf-8")

