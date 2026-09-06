from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .models import Control, Obstacle, RobotState, SafetyConfig, Scenario, Vec2


def _vec(data: dict[str, Any]) -> Vec2:
    return Vec2(float(data["x"]), float(data["y"]))


def load_scenario(path: str | Path) -> Scenario:
    source = Path(path)
    data = json.loads(source.read_text(encoding="utf-8"))
    ego = data["ego"]
    command = data["commanded_control"]
    obstacles = tuple(
        Obstacle(
            obstacle_id=str(item["id"]),
            position=_vec(item["position"]),
            velocity=_vec(item["velocity"]),
            radius_m=float(item.get("radius_m", 0.45)),
            position_sigma_m=float(item.get("position_sigma_m", 0.15)),
            acceleration_sigma_mps2=float(item.get("acceleration_sigma_mps2", 0.2)),
        )
        for item in data.get("obstacles", [])
    )
    config = SafetyConfig(**{key: float(value) for key, value in data.get("config", {}).items()})
    return Scenario(
        name=str(data.get("name", source.stem)),
        ego=RobotState(
            position=_vec(ego["position"]),
            heading_rad=float(ego["heading_rad"]),
            speed_mps=float(ego["speed_mps"]),
        ),
        commanded_control=Control(
            acceleration_mps2=float(command["acceleration_mps2"]),
            yaw_rate_rps=float(command["yaw_rate_rps"]),
        ),
        obstacles=obstacles,
        config=config,
    )

