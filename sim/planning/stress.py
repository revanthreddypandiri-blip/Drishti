"""Traffic Stress Index (environment state) and Risk Budget (planner's
allowable exposure) -- kept as two distinct quantities on purpose:

  Traffic Stress Index = how chaotic the *environment* is right now
                          (0=calm empty road ... 100=chaotic/occluded/dense)
  Risk Budget           = how much residual risk the *planner* is currently
                          allowed to accept before it must react conservatively

Risk Budget shrinks as stress rises, and is fed back into the planner's risk
weighting -- so the stress gauge is not dashboard decoration, it's part of
the cost function the vehicle actually optimizes.
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Dict, List

from sim.prediction.intent import ObservedAgent

INFLUENCE_RADIUS = 20.0


@dataclass
class StressBreakdown:
    density: float
    ripple_activity: float
    occlusion: float
    planner_risk: float
    total: float


def compute_stress(ego_pos, observations: List[ObservedAgent],
                    active_disturbance_total: float, occluded_count: int,
                    planner_top_risk: float) -> StressBreakdown:
    # density: how many agents are nearby, saturating around ~8 agents
    close = [o for o in observations
             if math.hypot(o.x - ego_pos[0], o.y - ego_pos[1]) < INFLUENCE_RADIUS]
    density = min(1.0, len(close) / 8.0)

    # ripple activity: how much active reactive disturbance is in play nearby
    ripple_activity = min(1.0, active_disturbance_total / 5.0)

    # occlusion: fraction of nearby-but-unresolved detections (hidden agents)
    occlusion = min(1.0, occluded_count / 3.0)

    # planner risk: the top candidate's own risk score, already 0..~0.3 typical
    planner_risk = min(1.0, planner_top_risk / 0.25)

    total = 100.0 * (0.30 * density + 0.25 * ripple_activity +
                      0.25 * occlusion + 0.20 * planner_risk)
    total = max(0.0, min(100.0, total))
    return StressBreakdown(density=round(density * 100, 1), ripple_activity=round(ripple_activity * 100, 1),
                            occlusion=round(occlusion * 100, 1), planner_risk=round(planner_risk * 100, 1),
                            total=round(total, 1))


def risk_budget(stress_total: float) -> float:
    """Allowable exposure remaining, 0..100. Falls faster than linearly as
    stress climbs past the mid-range, so the planner tightens up decisively
    rather than gradually once things get chaotic."""
    headroom = 100.0 - stress_total
    return round(max(0.0, headroom ** 1.15 / (100.0 ** 0.15)), 1)


def risk_weight_multiplier(budget: float) -> float:
    """How much to scale the planner's base risk weight given the current
    budget -- less budget => more risk-averse cost function. Square-root
    scaling (capped) keeps this a meaningful nudge rather than compounding
    with the uncertainty-widened margins into gridlock."""
    budget = max(5.0, budget)
    return min(2.2, math.sqrt(100.0 / budget))
