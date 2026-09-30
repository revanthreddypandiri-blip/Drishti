"""Dynamic drivable corridor: instead of asking "where is the lane?", ask
"what region of the road is safely drivable right now?" -- computed fresh
every tick from road boundary + static obstacles + nearby agents (with their
uncertainty-widened margins), independent of any painted lane marking.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple, Dict

from sim.core.geometry import RoadSegment, StaticObstacle
from sim.prediction.intent import ObservedAgent

STATION_STEP = 2.0
LOOKAHEAD = 30.0


@dataclass
class CorridorStation:
    s: float
    x: float
    y: float
    left: float    # left bound, lateral offset from centerline (m)
    right: float   # right bound, lateral offset from centerline (m, negative)


def compute_corridor(road: RoadSegment, ego_s: float, obstacles: List[StaticObstacle],
                      observations: List[ObservedAgent],
                      uncertainty_by_id: Dict[str, float]) -> List[CorridorStation]:
    stations = []
    n = int(LOOKAHEAD / STATION_STEP)
    for i in range(n + 1):
        s = min(road.length, ego_s + i * STATION_STEP)
        half_w = road.width_at(s) / 2.0
        left, right = half_w, -half_w

        for o in obstacles:
            os_, olat = road.project(o.position)
            if abs(os_ - s) < 3.0:
                pad = o.radius + 0.6
                if olat >= 0:
                    left = min(left, olat - pad) if olat - pad < left else left
                    # obstacle on the left narrows the left bound down to just past it
                    left = min(left, max(right + 0.5, olat - pad))
                else:
                    right = max(right, min(left - 0.5, olat + pad))

        for o in observations:
            os_, olat = road.project((o.x, o.y))
            if abs(os_ - s) < 4.0:
                extra = uncertainty_by_id.get(o.id, 0.0) * 1.5
                half_agent = 0.9 + extra
                if olat >= 0:
                    left = min(left, max(right + 0.5, olat - half_agent))
                else:
                    right = max(right, min(left - 0.5, olat + half_agent))

        if left < right:
            left = right = (left + right) / 2.0
        p = road.point_at(s)
        stations.append(CorridorStation(s=round(s, 1), x=round(p[0], 2), y=round(p[1], 2),
                                         left=round(left, 2), right=round(right, 2)))
    return stations


def clamp_to_corridor(stations: List[CorridorStation], s: float, desired_lat: float) -> float:
    if not stations:
        return desired_lat
    best = min(stations, key=lambda st: abs(st.s - s))
    return max(best.right + 0.5, min(best.left - 0.5, desired_lat))
