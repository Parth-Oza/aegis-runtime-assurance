from __future__ import annotations

import html
import json
from pathlib import Path

from .models import AssuranceDecision


def write_json(decision: AssuranceDecision, path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(decision.to_dict(), indent=2, sort_keys=True), encoding="utf-8")
    return target


def _percent(value: float) -> str:
    return f"{100.0 * value:.2f}%"


def write_html(decision: AssuranceDecision, path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    before = decision.commanded_evaluation
    after = decision.applied_evaluation
    status = "INTERVENED" if decision.intervened else "ALLOWED"
    rows = "".join(
        f"<tr><td>{sample.time_s:.1f}</td><td>{html.escape(sample.obstacle_id)}</td>"
        f"<td>{sample.clearance_m:.2f}</td><td>{_percent(sample.collision_probability)}</td></tr>"
        for sample in sorted(
            before.risk_samples,
            key=lambda sample: sample.collision_probability,
            reverse=True,
        )[:10]
    )
    document = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Aegis evidence — {html.escape(decision.scenario_name)}</title>
<style>
:root{{--ink:#eaf2ff;--muted:#91a4bd;--panel:#101a2a;--line:#24344d;--good:#4ade80;--bad:#fb7185;--accent:#67e8f9}}
*{{box-sizing:border-box}}body{{margin:0;background:#07101d;color:var(--ink);font:15px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace}}
main{{max-width:1100px;margin:auto;padding:48px 24px}}h1{{font-size:clamp(28px,5vw,56px);margin:.2em 0}}h2{{font-size:18px;color:var(--accent)}}
.eyebrow{{color:var(--accent);letter-spacing:.15em}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:16px}}
.card{{background:linear-gradient(145deg,#142238,#0c1625);border:1px solid var(--line);border-radius:14px;padding:20px}}
.value{{font-size:30px;font-weight:700}}.bad{{color:var(--bad)}}.good{{color:var(--good)}}.muted{{color:var(--muted)}}
table{{width:100%;border-collapse:collapse}}th,td{{text-align:left;padding:10px;border-bottom:1px solid var(--line)}}code{{color:var(--accent)}}
</style></head><body><main>
<div class="eyebrow">AEGIS RUNTIME ASSURANCE / DECISION EVIDENCE</div>
<h1>{html.escape(decision.scenario_name)}</h1><p class="muted">{html.escape(decision.reason)}</p>
<section class="grid">
<div class="card"><div class="muted">Decision</div><div class="value">{status}</div></div>
<div class="card"><div class="muted">Commanded risk</div><div class="value bad">{_percent(before.max_collision_probability)}</div></div>
<div class="card"><div class="muted">Applied risk</div><div class="value good">{_percent(after.max_collision_probability)}</div></div>
<div class="card"><div class="muted">Candidates evaluated</div><div class="value">{decision.candidates_evaluated}</div></div>
<div class="card"><div class="muted">Fallback</div><div class="value">{html.escape(decision.fallback_controller)}</div></div>
<div class="card"><div class="muted">Feasibility</div><div class="value">{html.escape(decision.fallback_status)}</div></div>
<div class="card"><div class="muted">Command deviation</div><div class="value">{decision.command_deviation:.3f}</div></div>
</section>
<section class="card" style="margin-top:16px"><h2>Counterfactual control</h2>
<p>Planner: <code>a={decision.commanded_control.acceleration_mps2:.2f} m/s²</code>, <code>ω={decision.commanded_control.yaw_rate_rps:.2f} rad/s</code><br>
Applied: <code>a={decision.applied_control.acceleration_mps2:.2f} m/s²</code>, <code>ω={decision.applied_control.yaw_rate_rps:.2f} rad/s</code></p></section>
<section class="card" style="margin-top:16px"><h2>Highest-risk predictions before intervention</h2>
<table><thead><tr><th>t (s)</th><th>Obstacle</th><th>Clearance (m)</th><th>Collision risk</th></tr></thead><tbody>{rows}</tbody></table></section>
</main></body></html>"""
    target.write_text(document, encoding="utf-8")
    return target

