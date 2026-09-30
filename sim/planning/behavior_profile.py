"""Indian-traffic behavior profiles: classify perceived agents by *behavior*,
not just object category -- "auto-rickshaw, crossing, erratic" is far more
actionable for a planner than just "auto-rickshaw"."""
from __future__ import annotations
import math
from typing import Optional

from sim.core.geometry import RoadSegment
from sim.prediction.intent import ObservedAgent

PROFILES = ["Stable", "Cautious", "Erratic", "Crossing", "Filtering", "Oncoming", "Stationary", "Occluded"]


def classify(obs: ObservedAgent, road: RoadSegment, surprise_score: float,
             ego_heading: float, was_occluded_recently: bool) -> str:
    if was_occluded_recently:
        return "Occluded"
    if obs.v < 0.15:
        return "Stationary"

    s, lat = road.project((obs.x, obs.y))
    heading_diff = abs((obs.theta - road.heading_at(s) + math.pi) % (2 * math.pi) - math.pi)

    if heading_diff > math.radians(120):
        return "Oncoming"

    lateral_component = abs(obs.v * math.sin(heading_diff))
    if lateral_component > 0.7 and obs.kind in ("pedestrian", "animal", "pushcart"):
        return "Crossing"

    if obs.kind == "two_wheeler" and heading_diff < math.radians(35) and 0.2 < surprise_score < 0.6:
        return "Filtering"

    if surprise_score >= 0.6:
        return "Erratic"

    if surprise_score >= 0.3 or heading_diff > math.radians(20):
        return "Cautious"

    return "Stable"
