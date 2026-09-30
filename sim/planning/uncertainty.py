"""Multi-cause uncertainty: instead of one arbitrary safety-margin fudge
factor, track *why* an agent is uncertain, so the planner can react
differently to different causes rather than inflating every buffer equally.

  behavior     -- how spread-out the intent-hypothesis distribution is
                  (entropy) plus recent prediction-error ("surprise")
  perception   -- 1 - sensor confidence for this detection
  occlusion    -- how long since we last actually observed this agent
  environment  -- how locally unclear the road boundary is (width jitter)
"""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Dict


@dataclass
class UncertaintyBreakdown:
    behavior: float
    perception: float
    occlusion: float
    environment: float

    @property
    def total(self) -> float:
        # weighted combination, not a plain average -- occlusion and behavior
        # unpredictability matter more for planning than a slightly noisy
        # position estimate does.
        return max(0.0, min(1.0, 0.35 * self.behavior + 0.20 * self.perception +
                             0.30 * self.occlusion + 0.15 * self.environment))

    @property
    def dominant_cause(self) -> str:
        parts = {"behavior": self.behavior, "perception": self.perception,
                 "occlusion": self.occlusion, "environment": self.environment}
        return max(parts, key=parts.get)


def entropy_of(probs: Dict[str, float]) -> float:
    h = -sum(p * math.log(p + 1e-9) for p in probs.values() if p > 0)
    h_max = math.log(len(probs)) if probs else 1.0
    return max(0.0, min(1.0, h / h_max)) if h_max > 0 else 0.0


def environment_uncertainty(road, s: float) -> float:
    """How locally unclear the road boundary is near arc-length s -- an
    unmarked road with a rapidly changing edge is inherently less certain
    than a clean, constant-width stretch."""
    w0 = road.width_at(max(0.0, s - 3.0))
    w1 = road.width_at(min(road.length, s + 3.0))
    jitter = abs(w1 - w0)
    return max(0.0, min(1.0, jitter / 2.5))


def build_breakdown(intent_probs: Dict[str, float], surprise_score: float,
                     confidence: float, seconds_since_seen: float,
                     road, s: float) -> UncertaintyBreakdown:
    behavior = max(0.0, min(1.0, 0.6 * entropy_of(intent_probs) + 0.4 * min(1.0, surprise_score)))
    perception = max(0.0, min(1.0, 1.0 - confidence))
    occlusion = max(0.0, min(1.0, seconds_since_seen / 2.0))
    environment = environment_uncertainty(road, s)
    return UncertaintyBreakdown(behavior=round(behavior, 3), perception=round(perception, 3),
                                 occlusion=round(occlusion, 3), environment=round(environment, 3))
