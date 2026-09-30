"""Candidate trajectory generation + risk-based selection for the ego vehicle.

Generates a small, human-interpretable set of maneuver candidates (not a dense
search space -- explainability matters for a hackathon demo), forward-
simulates each with the same kinematic bicycle model used elsewhere, and
scores each against a Monte-Carlo-style set of *predicted* futures for every
currently perceived road user (multiple weighted hypotheses per agent, from
sim.prediction.intent).

This version also wires in the full negotiation-aware architecture:
- the Behavior FSM's allowed-maneuver bias (so NEGOTIATE/YIELD/EMERGENCY
  states actually narrow what the planner may pick, not just label it)
- the Risk Budget's risk-weight multiplier (less budget -> more risk-averse)
- the dynamic drivable corridor (lateral targets are clamped to it, not to
  the raw road width)
- multi-cause uncertainty per perceived agent, widening its effective safety
  margin instead of using one flat number
- a surprise detector (prediction error vs. simple extrapolation) feeding
  both uncertainty and behavior-profile classification
- occlusion-zone caution (slow down before a region that could be hiding an
  undetected road user)

The candidate with lowest total cost is proposed to the Safety Shield for
independent verification before being applied.
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional

from sim.core.kinematics import BicycleState, step_bicycle
from sim.core.geometry import RoadSegment, StaticObstacle, dist
from sim.agents.base import Agent
from sim.prediction.intent import ObservedAgent, IntentEstimator, rollout_futures
from sim.perception.sensors import PerceptionResult, OcclusionZone
from sim.planning import corridor as corridor_mod
from sim.planning import uncertainty as unc_mod
from sim.planning import behavior_profile

DT = 0.3
HORIZON = 3.0
STEPS = int(HORIZON / DT)

MANEUVERS = [
    # name, target_speed_frac_of_current_target, lateral_shift(m), label
    ("keep",        1.00,  0.0, "Maintain course"),
    ("decelerate",  0.55,  0.0, "Slow down"),
    ("hard_brake",  0.10,  0.0, "Hard brake"),
    ("nudge_left",  0.90,  1.4, "Shift left in corridor"),
    ("nudge_right", 0.90, -1.4, "Shift right in corridor"),
    ("creep",       0.30,  0.0, "Creep forward cautiously"),
]

EGO_RADIUS = 2.4
AGENT_MARGIN = {"pedestrian": 1.4, "animal": 1.6, "pushcart": 1.2}
DEFAULT_MARGIN = 1.0


def footprint_radius(kind: str) -> float:
    """Bounding-circle radius of an agent type. Safety margins must scale with
    the agent's real size: an 8 m truck needs far more separation from the ego
    than a pedestrian does, regardless of a fixed per-type buffer."""
    from sim.agents.base import TYPE_PROFILE
    prof = TYPE_PROFILE.get(kind)
    if not prof:
        return 1.5
    return math.hypot(prof["length"], prof["width"]) / 2.0


def base_margin(kind: str) -> float:
    return max(AGENT_MARGIN.get(kind, DEFAULT_MARGIN), footprint_radius(kind) + 0.5)


@dataclass
class TrajectoryCandidate:
    name: str
    label: str
    points: List[Tuple[float, float, float, float]]   # t, x, y, theta
    controls: List[Tuple[float, float]] = field(default_factory=list)  # (accel, steer) per step
    cost: float = 0.0
    risk: float = 0.0
    comfort_cost: float = 0.0
    progress_cost: float = 0.0
    offroad_cost: float = 0.0
    occlusion_cost: float = 0.0
    breakdown: Dict[str, float] = field(default_factory=dict)


class Planner:
    def __init__(self):
        self.intent_estimator = IntentEstimator()
        self._recent_events: List[Tuple[str, float, float, float]] = []  # kind,x,y,t
        self._obs_history: Dict[str, ObservedAgent] = {}
        self._last_nearby_events: Dict[str, list] = {}
        self._last_seen_t: Dict[str, float] = {}
        self._surprise: Dict[str, float] = {}
        self.last_explanation: str = "Initializing…"
        self.last_focus_agent_id = None
        self.last_uncertainty: Dict[str, "unc_mod.UncertaintyBreakdown"] = {}
        self.last_behavior_profile: Dict[str, str] = {}
        self.last_corridor: List = []
        self.last_explain_struct: Dict = {}

    def _update_event_log(self, observations: List[ObservedAgent], t: float):
        for o in observations:
            prev = self._obs_history.get(o.id)
            if prev is not None:
                if prev.v - o.v > 0.8 * DT * 3:
                    self._recent_events.append(("brake", o.x, o.y, t))
                heading_delta = abs((o.theta - prev.theta + math.pi) % (2 * math.pi) - math.pi)
                if heading_delta > 0.25:
                    self._recent_events.append(("swerve", o.x, o.y, t))
            self._obs_history[o.id] = o
        self._recent_events = [e for e in self._recent_events if t - e[3] < 1.5][-40:]

    def _nearby_events_for(self, x: float, y: float, t: float):
        out = []
        for kind, ex, ey, et in self._recent_events:
            d = math.hypot(ex - x, ey - y)
            if d < 20.0:
                out.append((kind, d, max(0.2, 1.0 - (t - et) / 1.5)))
        return out

    def _update_surprise(self, o: ObservedAgent, t: float, dt: float) -> float:
        """Compare actual motion to a naive constant-velocity extrapolation
        of the previous observation -- large, sustained deviation means this
        agent is behaving unexpectedly (erratic), which both widens its
        uncertainty and feeds the 'Erratic' behavior-profile label."""
        prev = self._obs_history.get(o.id)
        last_t = self._last_seen_t.get(o.id, t)
        elapsed = max(1e-3, t - last_t)
        if prev is not None:
            pred_x = prev.x + prev.v * math.cos(prev.theta) * elapsed
            pred_y = prev.y + prev.v * math.sin(prev.theta) * elapsed
            err = math.hypot(o.x - pred_x, o.y - pred_y)
            scale = 0.4 + 0.35 * o.v
            instant = max(0.0, min(1.0, err / scale))
        else:
            instant = 0.0
        prior = self._surprise.get(o.id, 0.0)
        smoothed = 0.65 * prior + 0.35 * instant
        self._surprise[o.id] = smoothed
        self._last_seen_t[o.id] = t
        return smoothed

    def _predict_agents(self, observations: List[ObservedAgent], t: float, dt: float,
                         road: RoadSegment):
        futures = {}
        self._last_nearby_events = {}
        self.last_uncertainty = {}
        self.last_behavior_profile = {}
        for o in observations:
            surprise = self._update_surprise(o, t, dt)
            nearby = self._nearby_events_for(o.x, o.y, t)
            self._last_nearby_events[o.id] = nearby
            probs = self.intent_estimator.update(o, nearby)
            futures[o.id] = rollout_futures(o, probs, horizon=HORIZON, dt=DT)

            s, _ = road.project((o.x, o.y))
            self.last_uncertainty[o.id] = unc_mod.build_breakdown(
                probs, surprise, o.confidence, seconds_since_seen=0.0, road=road, s=s)
            self.last_behavior_profile[o.id] = behavior_profile.classify(
                o, road, surprise, ego_heading=0.0, was_occluded_recently=False)
        return futures

    def _simulate_candidate(self, ego: Agent, road: RoadSegment, target_speed_frac: float,
                             lateral_shift: float, stations) -> Tuple[list, list]:
        st = BicycleState(ego.state.x, ego.state.y, ego.state.theta, ego.state.v, ego.state.wheelbase)
        s0, lat0 = road.project((st.x, st.y))
        desired_lat = lat0 + lateral_shift
        desired_lat = corridor_mod.clamp_to_corridor(stations, s0, desired_lat)
        target_lat = max(-road.width_at(s0) / 2 + 1.0, min(road.width_at(s0) / 2 - 1.0, desired_lat))
        base_target_speed = ego.profile()["max_speed"] * ego.target_speed_frac
        target_speed = base_target_speed * target_speed_frac
        pts = [(0.0, st.x, st.y, st.theta)]
        controls: List[Tuple[float, float]] = []
        for i in range(STEPS):
            s, lat = road.project((st.x, st.y))
            heading_err = (road.heading_at(s) - st.theta + math.pi) % (2 * math.pi) - math.pi
            lat_err = target_lat - lat
            steer = 0.4 * heading_err + 0.1 * lat_err
            accel = 1.6 * (1 - (st.v / max(target_speed, 0.3)) ** 2) if target_speed > 0.05 else -3.0
            accel = max(-6.0, min(2.0, accel))
            controls.append((accel, steer))
            st = step_bicycle(st, accel, steer, DT, max_speed=base_target_speed + 2, min_speed=0.0, max_steer=0.4)
            pts.append((round((i + 1) * DT, 2), st.x, st.y, st.theta))
        return pts, controls

    def _score(self, cand_points, road: RoadSegment, agent_futures: Dict[str, List[Dict]],
               observations: List[ObservedAgent], occlusion_zones: List[OcclusionZone],
               risk_w: float) -> Tuple[float, Dict[str, float]]:
        risk = 0.0
        top_contrib = 0.0
        top_info = None   # (agent_id, hypothesis, t_of_closest_approach)
        by_obs = {o.id: o for o in observations}
        for aid, futures in agent_futures.items():
            obs = by_obs[aid]
            unc = self.last_uncertainty.get(aid)
            margin = base_margin(obs.kind) * (1.0 + (unc.total if unc else 0.0))
            safe_dist = EGO_RADIUS + margin
            for fut in futures:
                p = fut["probability"]
                for (t, ax, ay, _th), (et, ex, ey, _eth) in zip(fut["points"], cand_points[1:]):
                    d = math.hypot(ax - ex, ay - ey)
                    if d < safe_dist * 1.5:
                        severity = max(0.0, (safe_dist * 1.5 - d) / (safe_dist * 1.5))
                        time_discount = math.exp(-t * 0.5)
                        contrib = p * severity * severity * time_discount
                        risk += contrib
                        if contrib > top_contrib:
                            top_contrib = contrib
                            top_info = (aid, fut["hypothesis"], t)

        # occlusion-zone caution: entering a region that could be hiding an
        # undetected road user costs a little, even with nothing detected there
        occlusion_cost = 0.0
        for (t, x, y, th) in cand_points[1:]:
            for z in occlusion_zones:
                d = math.hypot(x - z.x, y - z.y)
                if d < z.radius:
                    occlusion_cost += 0.4 * math.exp(-t * 0.6)

        comfort = 0.0
        for i in range(2, len(cand_points)):
            t0, x0, y0, h0 = cand_points[i - 2]
            t1, x1, y1, h1 = cand_points[i - 1]
            t2, x2, y2, h2 = cand_points[i]
            dh1 = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
            dh2 = (h2 - h1 + math.pi) % (2 * math.pi) - math.pi
            comfort += abs(dh2 - dh1)

        s_start, _ = road.project((cand_points[0][1], cand_points[0][2]))
        s_end, _ = road.project((cand_points[-1][1], cand_points[-1][2]))
        progress_cost = -(s_end - s_start) * 2.0

        offroad = 0.0
        for (t, x, y, th) in cand_points:
            if not road.is_drivable((x, y), margin=0.3):
                offroad += 3.0

        comfort_w, offroad_w, occl_w = 0.8, 1.0, 1.0
        total = risk_w * risk + comfort_w * comfort + progress_cost + offroad_w * offroad + occl_w * occlusion_cost
        return total, {"risk": risk, "comfort": comfort, "progress": progress_cost, "offroad": offroad,
                        "occlusion": occlusion_cost, "top_risk_source": top_info}

    def plan(self, ego: Agent, road: RoadSegment, obstacles: List[StaticObstacle],
             perception: PerceptionResult, t: float, dt: float,
             fsm=None,
             risk_w_multiplier: float = 1.0
             ) -> Tuple[TrajectoryCandidate, List[TrajectoryCandidate]]:
        observations = perception.observations
        self._update_event_log(observations, t)
        agent_futures = self._predict_agents(observations, t, dt, road)

        ego_s, _ = road.project(ego.pos())
        uncertainty_by_id = {aid: (u.total if u else 0.0) for aid, u in self.last_uncertainty.items()}
        stations = corridor_mod.compute_corridor(road, ego_s, obstacles, observations, uncertainty_by_id)
        self.last_corridor = stations

        base_risk_w = 6.0
        risk_w = base_risk_w * risk_w_multiplier

        candidates: List[TrajectoryCandidate] = []
        for name, speed_frac, lat_shift, label in MANEUVERS:
            pts, controls = self._simulate_candidate(ego, road, speed_frac, lat_shift, stations)
            total, breakdown = self._score(pts, road, agent_futures, observations,
                                            perception.occlusion_zones, risk_w)
            if fsm is not None:
                total += fsm.maneuver_penalty(name)
            candidates.append(TrajectoryCandidate(name=name, label=label, points=pts, controls=controls,
                                                    cost=total,
                                                    risk=breakdown["risk"], comfort_cost=breakdown["comfort"],
                                                    progress_cost=breakdown["progress"],
                                                    offroad_cost=breakdown["offroad"],
                                                    occlusion_cost=breakdown["occlusion"], breakdown=breakdown))
        candidates.sort(key=lambda c: c.cost)
        best = candidates[0]
        self.last_explanation = self._build_explanation(best, observations, t, perception.occlusion_zones)
        return best, candidates

    def _build_explanation(self, chosen: "TrajectoryCandidate", observations: List[ObservedAgent],
                            t: float, occlusion_zones: List[OcclusionZone]) -> str:
        struct = dict(reason="Corridor clear", risk_0_100=round(chosen.risk * 100, 1),
                       action=chosen.label, clearance_m=None, next_eval_s=DT)

        if chosen.risk < 0.02 and chosen.occlusion_cost < 0.05:
            self.last_focus_agent_id = None
            struct["reason"] = "No specific nearby risk"
            self.last_explain_struct = struct
            return "Corridor clear — maintaining course." if chosen.name == "keep" else \
                   f"{chosen.label} (precautionary, no specific nearby risk)."

        info = chosen.breakdown.get("top_risk_source")
        if not info:
            if chosen.occlusion_cost >= 0.05 and occlusion_zones:
                self.last_focus_agent_id = None
                z = occlusion_zones[0]
                struct["reason"] = f"Occlusion zone behind '{z.blocker_id}'"
                self.last_explain_struct = struct
                return (f"{chosen.label}: approaching a region behind '{z.blocker_id}' that sensors "
                        f"can't clear — treating it as a possible hidden road user.")
            self.last_focus_agent_id = None
            struct["reason"] = "Diffuse precaution"
            self.last_explain_struct = struct
            return f"{chosen.label} (no single dominant risk source)."

        aid, hyp, t_at = info
        self.last_focus_agent_id = aid
        obs_by_id = {o.id: o for o in observations}
        obs = obs_by_id.get(aid)
        kind = obs.kind if obs else "agent"
        profile = self.last_behavior_profile.get(aid, "")
        hyp_text = {
            "brake": "braking", "stop": "stopping",
            "swerve_left": "veering left" if kind in ("pedestrian", "animal", "pushcart") else "swerving left",
            "swerve_right": "veering right" if kind in ("pedestrian", "animal", "pushcart") else "swerving right",
            "cross_path": "crossing into the corridor",
            "keep": "continuing straight",
        }.get(hyp, hyp)
        chain_note = ""
        nearby = self._last_nearby_events.get(aid, [])
        if nearby:
            src_kind = nearby[0][0]
            chain_note = f" — anticipated as a ripple reaction to a nearby {src_kind} event"
        profile_note = f" [{profile}]" if profile and profile not in ("Stable",) else ""

        struct["reason"] = f"{kind} '{aid}' predicted {hyp_text}{profile_note}"
        struct["clearance_m"] = None
        self.last_explain_struct = struct
        return (f"{chosen.label}: predicted {kind} '{aid}'{profile_note} {hyp_text} in ~{t_at:.1f}s"
                f"{chain_note}.")
