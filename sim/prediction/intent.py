"""Probabilistic intent estimation and short-term motion prediction for
*perceived* (not ground-truth) agents. This is the ego's belief model: given
noisy, possibly-occluded detections, estimate what each nearby agent is likely
to do next, and roll that forward into multiple plausible short-term futures.
Crucially, this model also predicts *ripple risk*: if agent A was just seen
swerving/braking, agents near A get an elevated probability of reacting too,
even before we observe them reacting -- this is what lets Drishti anticipate
"the ripple hasn't reached the pedestrian yet, but it's about to"."""
from __future__ import annotations
import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

HYPOTHESES = ["keep", "brake", "swerve_left", "swerve_right", "stop", "cross_path"]

RIPPLE_EVENT_TO_REACTIVE_BOOST = {
    "brake": {"brake": 0.35, "stop": 0.1},
    "swerve": {"swerve_left": 0.2, "swerve_right": 0.2, "brake": 0.15},
    "stop": {"stop": 0.3, "brake": 0.1},
}


@dataclass
class ObservedAgent:
    id: str
    kind: str
    x: float
    y: float
    theta: float
    v: float
    prev_v: Optional[float] = None
    prev_theta: Optional[float] = None
    confidence: float = 1.0     # from perception (occlusion/noise reduces this)


@dataclass
class IntentBelief:
    agent_id: str
    probs: Dict[str, float]


class IntentEstimator:
    def __init__(self, seed: int = 7):
        self.rng = random.Random(seed)
        self._beliefs: Dict[str, Dict[str, float]] = {}

    def _prior(self, obs: ObservedAgent) -> Dict[str, float]:
        if obs.kind in ("pedestrian", "animal", "pushcart"):
            base = {"keep": 0.55, "brake": 0.0, "swerve_left": 0.08, "swerve_right": 0.08,
                    "stop": 0.14, "cross_path": 0.15}
        else:
            base = {"keep": 0.65, "brake": 0.1, "swerve_left": 0.08, "swerve_right": 0.08,
                    "stop": 0.04, "cross_path": 0.05}
        return base

    def update(self, obs: ObservedAgent, nearby_recent_events: List[Tuple[str, float, float]]
               ) -> Dict[str, float]:
        """nearby_recent_events: list of (event_kind, distance_to_this_agent, event_magnitude)
        for ripple-source events observed within the last ~1.5s, used to anticipate
        reactive intent before it's directly visible in this agent's own motion."""
        probs = dict(self._beliefs.get(obs.id, self._prior(obs)))

        if obs.prev_theta is not None:
            heading_rate = abs((obs.theta - obs.prev_theta + math.pi) % (2 * math.pi) - math.pi)
            if heading_rate > 0.25:
                side = "swerve_left" if (obs.theta - obs.prev_theta) > 0 else "swerve_right"
                probs[side] = min(0.9, probs[side] + 0.25)
                probs["keep"] = max(0.02, probs["keep"] - 0.2)
        if obs.prev_v is not None:
            decel = obs.prev_v - obs.v
            if decel > 0.8:
                probs["brake"] = min(0.9, probs["brake"] + 0.3)
                probs["keep"] = max(0.02, probs["keep"] - 0.25)
            if obs.v < 0.2 and obs.prev_v < 0.4:
                probs["stop"] = min(0.85, probs["stop"] + 0.15)

        for kind, edist, mag in nearby_recent_events:
            boosts = RIPPLE_EVENT_TO_REACTIVE_BOOST.get(kind, {})
            decay = math.exp(-edist / 12.0) * mag
            for h, b in boosts.items():
                probs[h] = min(0.9, probs[h] + b * decay)

        total = sum(probs.values()) or 1.0
        probs = {k: v / total for k, v in probs.items()}
        self._beliefs[obs.id] = probs
        return probs

    def forget(self, agent_id: str):
        self._beliefs.pop(agent_id, None)


def rollout_futures(obs: ObservedAgent, probs: Dict[str, float], horizon: float = 3.0,
                     dt: float = 0.3) -> List[Dict]:
    """Produce a small set of plausible future trajectories for one agent, each
    tagged with the hypothesis and its probability, for downstream risk scoring."""
    steps = max(1, int(horizon / dt))
    futures = []
    for hyp, p in probs.items():
        if p < 0.03:
            continue
        x, y, theta, v = obs.x, obs.y, obs.theta, obs.v
        pts = []
        for i in range(steps):
            if hyp == "keep":
                pass
            elif hyp == "brake":
                v = max(0.0, v - 1.8 * dt)
            elif hyp == "stop":
                v = max(0.0, v - 2.6 * dt)
            elif hyp == "swerve_left":
                theta += 0.35 * dt
            elif hyp == "swerve_right":
                theta -= 0.35 * dt
            elif hyp == "cross_path":
                theta = theta  # assume continues toward road at current heading
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            pts.append((round((i + 1) * dt, 2), x, y, theta))
        futures.append({"hypothesis": hyp, "probability": p, "points": pts})
    return futures
