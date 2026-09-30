"""Independent Safety Shield.

Deliberately separate and simpler than the main planner: it does NOT use
intent hypotheses or the ripple graph. It re-checks the planner's chosen
trajectory against a dumb, conservative constant-velocity extrapolation of
every currently perceived agent (i.e. "assume the worst, assume nothing about
intent"). If that check would ever bring ego closer than a hard safety margin,
the Shield vetoes the planner and substitutes an emergency-braking fallback.
This gives Drishti two independent lines of defense, matching the project's
'choose a future-safe trajectory -> independently verify with a Safety Shield'
pipeline step."""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Tuple

from sim.core.kinematics import BicycleState, step_bicycle
from sim.prediction.intent import ObservedAgent
from sim.planning.planner import TrajectoryCandidate, EGO_RADIUS, base_margin, DT, STEPS

HARD_MARGIN_EXTRA = 0.7


@dataclass
class ShieldVerdict:
    approved: bool
    reason: str
    min_clearance: float
    fallback: TrajectoryCandidate = None
    target_id: str = None


def _cv_project(obs: ObservedAgent, horizon: float, dt: float):
    pts = []
    x, y = obs.x, obs.y
    for i in range(int(horizon / dt)):
        x += obs.v * math.cos(obs.theta) * dt
        y += obs.v * math.sin(obs.theta) * dt
        pts.append((round((i + 1) * dt, 2), x, y))
    return pts


def _emergency_brake(ego, road) -> TrajectoryCandidate:
    st = BicycleState(ego.state.x, ego.state.y, ego.state.theta, ego.state.v, ego.state.wheelbase)
    pts = [(0.0, st.x, st.y, st.theta)]
    controls: List[Tuple[float, float]] = []
    for i in range(STEPS):
        s, lat = road.project((st.x, st.y))
        heading_err = (road.heading_at(s) - st.theta + math.pi) % (2 * math.pi) - math.pi
        steer = 0.3 * heading_err
        controls.append((-5.5, steer))
        st = step_bicycle(st, -5.5, steer, DT, max_speed=ego.profile()["max_speed"], min_speed=0.0)
        pts.append((round((i + 1) * DT, 2), st.x, st.y, st.theta))
    return TrajectoryCandidate(name="emergency_brake", label="SAFETY SHIELD: emergency brake", points=pts,
                                controls=controls, cost=0.0, risk=0.0)


def verify(ego, road, chosen: TrajectoryCandidate, observations: List[ObservedAgent]) -> ShieldVerdict:
    min_clearance = float("inf")
    for obs in observations:
        margin = base_margin(obs.kind)
        safe_dist = EGO_RADIUS + margin + HARD_MARGIN_EXTRA
        cv_pts = _cv_project(obs, HORIZON := DT * STEPS, DT)
        for (t, ax, ay), (et, ex, ey, eth) in zip(cv_pts, chosen.points[1:]):
            d = math.hypot(ax - ex, ay - ey)
            min_clearance = min(min_clearance, d)
            if d < safe_dist and t < 1.3:
                fb = _emergency_brake(ego, road)
                return ShieldVerdict(approved=False,
                                      reason=f"Predicted clearance {d:.1f}m < safe {safe_dist:.1f}m "
                                             f"with {obs.kind} '{obs.id}' at t={t:.1f}s (constant-velocity check)",
                                      min_clearance=min_clearance, fallback=fb, target_id=obs.id)
    return ShieldVerdict(approved=True, reason="OK", min_clearance=min_clearance if observations else 999.0)
