"""Top-level simulation loop: ties together road geometry, agents, the ripple
engine (ground-truth reactive behavior for all non-ego agents), perception,
the negotiation-aware planning stack (behavior FSM, traffic stress index,
risk budget, dynamic drivable corridor, multi-cause uncertainty), and the
safety shield for the ego vehicle. Call step() once per tick; snapshot()
returns a JSON-serializable world state for the frontend."""
from __future__ import annotations
import math
import random
import time
from typing import Dict, List, Optional

from sim.core.geometry import Scene, RoadSegment, StaticObstacle, dist
from sim.core.kinematics import BicycleState, PointMassState, step_bicycle
from sim.agents.base import Agent
from sim.agents.behaviors import update_wheeled, update_soft
from sim.graph.ripple import RippleEngine
from sim.perception.sensors import SensorSuite
from sim.planning.planner import Planner
from sim.planning import behavior_fsm
from sim.planning import stress as stress_mod
from sim.safety_shield import shield as safety_shield


class Simulation:
    def __init__(self, scene: Scene, agents: List[Agent], dt: float = 0.1, seed: int = 1):
        self.scene = scene
        self.road: RoadSegment = scene.road
        self.dt = dt
        self.t = 0.0
        self.seed = seed
        self.rng = random.Random(seed)
        self.ripple = RippleEngine()
        self.sensors = SensorSuite(seed=seed + 1)
        self.planner = Planner()
        self.fsm = behavior_fsm.FSMState()
        self.background_ai_enabled = True
        self.paused = False

        self.agents: Dict[str, Agent] = {}
        self.ego_id: Optional[str] = None
        for a in agents:
            self._spawn(a)
            if a.is_ego:
                self.ego_id = a.id
        assert self.ego_id is not None, "scenario must include exactly one ego agent"

        self.last_candidates = []
        self.last_chosen = None
        self.last_verdict = None
        self.last_observations = []
        self.last_plan_latency_ms = 0.0
        self.last_explanation = "Initializing…"
        self.last_focus_agent_id = None
        self.last_top_risk = 0.0
        self.last_stress = None
        self.last_risk_budget = 100.0
        self.last_occlusion_zones = []
        self.last_sensor_trust = {}

        self.metrics = dict(collisions=0, shield_interventions=0, ticks=0,
                             min_clearance_ever=999.0, sum_heading_change=0.0,
                             ego_progress_frac=0.0, completed=False, total_planner_cost=0.0)

    # -- setup -------------------------------------------------------------
    def _spawn(self, agent: Agent):
        prof = agent.profile()
        p = self.road.point_at(agent.route_s0)
        h = self.road.heading_at(agent.route_s0)
        if agent.is_wheeled():
            nx = p[0] - math.sin(h) * agent.lateral_pref
            ny = p[1] + math.cos(h) * agent.lateral_pref
            agent.state = BicycleState(nx, ny, h, prof["max_speed"] * 0.35)
        else:
            agent.state = PointMassState(p[0], p[1], h, 0.0)
        agent.history.append(agent.pos())
        self.agents[agent.id] = agent

    def _leading_gap(self, agent: Agent) -> Optional[float]:
        s_self, lat_self = self.road.project(agent.pos())
        best = None
        for other in self.agents.values():
            if other.id == agent.id or not other.alive or not other.is_wheeled():
                continue
            s_o, lat_o = self.road.project(other.pos())
            if s_o > s_self and abs(lat_o - lat_self) < 2.2:
                gap = s_o - s_self - (agent.profile()["length"] / 2 + other.profile()["length"] / 2)
                if best is None or gap < best:
                    best = gap
        return best

    def trigger_whatif(self, kind: str = "cross") -> Optional[str]:
        """Live provocation for the demo: immediately force the nearest
        non-ego, non-static agent within range into a crossing/erratic
        reaction, the way a judge might ask 'what if that pedestrian
        suddenly crosses?' -- a lightweight, on-demand counterfactual rather
        than a full recorded-state branch/replay."""
        ego = self.agents[self.ego_id]
        ego_s, _ = self.road.project(ego.pos())
        best, best_d = None, float("inf")
        for a in self.agents.values():
            if a.id == ego.id or not a.alive:
                continue
            s, _ = self.road.project(a.pos())
            ds = s - ego_s
            if 2.0 < ds < 35.0 and ds < best_d:
                best, best_d = a, ds
        if best is None:
            return None
        if not best.is_wheeled():
            gx, gy = best.pos()
            side = -1.0 if best.lateral_pref >= 0 else 1.0
            best.goal_free_roam = (gx + 4.0, gy + side * 6.0)
            best.target_speed_frac = min(1.0, best.target_speed_frac + 0.7)
        else:
            best.lateral_pref = -best.lateral_pref if abs(best.lateral_pref) > 0.1 else 1.5
            best.target_speed_frac = max(0.2, best.target_speed_frac - 0.4)
        return best.id

    # -- main loop -----------------------------------------------------------
    def step(self):
        if self.paused:
            return
        t, dt = self.t, self.dt
        self.ripple.step(self.agents, self.road, t, dt)

        ego = self.agents[self.ego_id]
        t0 = time.perf_counter()
        lighting = self.scene.conditions.get("lighting", "day")
        perception = self.sensors.perceive(ego, self.agents, lighting=lighting)
        self.last_sensor_trust = perception.sensor_trust
        self.last_occlusion_zones = perception.occlusion_zones

        active_disturbance = 0.0
        for a in self.agents.values():
            if a.id == ego.id or not a.alive:
                continue
            if a.disturbances and dist(ego.pos(), a.pos()) < 20.0:
                active_disturbance += len(a.disturbances) * 0.5

        stress = stress_mod.compute_stress(ego.pos(), perception.observations, active_disturbance,
                                            len(perception.occluded_ids), self.last_top_risk)
        self.last_stress = stress
        budget = stress_mod.risk_budget(stress.total)
        self.last_risk_budget = budget
        self.fsm.step(stress.total)
        risk_w_mult = stress_mod.risk_weight_multiplier(budget)

        chosen, candidates = self.planner.plan(ego, self.road, self.scene.obstacles, perception, t, dt,
                                                 fsm=self.fsm,
                                                 risk_w_multiplier=risk_w_mult)
        verdict = safety_shield.verify(ego, self.road, chosen, perception.observations)
        self.last_plan_latency_ms = (time.perf_counter() - t0) * 1000.0
        final = chosen if verdict.approved else verdict.fallback
        self.last_top_risk = chosen.risk

        self.last_candidates = candidates
        self.last_chosen = final
        self.last_verdict = verdict
        self.last_observations = perception.observations
        if verdict.approved:
            self.last_explanation = self.planner.last_explanation
            self.last_focus_agent_id = self.planner.last_focus_agent_id
        else:
            self.last_explanation = f"Safety Shield override: {verdict.reason}"
            self.last_focus_agent_id = verdict.target_id

        accel, steer = final.controls[0] if final.controls else (0.0, 0.0)
        ego.state = step_bicycle(ego.state, accel, steer, dt,
                                  max_speed=ego.profile()["max_speed"] + 2, min_speed=0.0, max_steer=0.5)
        ego.history.append(ego.pos())
        ego.history = ego.history[-250:]

        for a in self.agents.values():
            if a.id == ego.id or not a.alive:
                continue
            a.apply_due_triggers(t)
            if not a.is_wheeled():
                update_soft(a, self.road, dt, self.rng)
            elif self.background_ai_enabled:
                gap = self._leading_gap(a)
                update_wheeled(a, self.road, self.scene.obstacles, gap, dt, self.rng)
            else:
                a.state = step_bicycle(a.state, 0.3 * (a.profile()["max_speed"] * 0.5 - a.state.v), 0.0, dt,
                                        max_speed=a.profile()["max_speed"], min_speed=0.0)
            a.history.append(a.pos())
            a.history = a.history[-250:]

        self._update_metrics(ego, verdict, final)
        self.t += dt

    def _update_metrics(self, ego: Agent, verdict, final):
        m = self.metrics
        m["ticks"] += 1
        if not verdict.approved:
            m["shield_interventions"] += 1
        m["min_clearance_ever"] = min(m["min_clearance_ever"], verdict.min_clearance)
        m["total_planner_cost"] += final.cost if final is not None else 0.0

        for a in self.agents.values():
            if a.id == ego.id or not a.alive:
                continue
            d = dist(ego.pos(), a.pos())
            if d < (ego.bbox_radius() + a.bbox_radius()) * 0.75:
                m["collisions"] += 1

        s_now, _ = self.road.project(ego.pos())
        m["ego_progress_frac"] = max(0.0, min(1.0, s_now / max(self.road.length, 1e-3)))
        if m["ego_progress_frac"] >= 0.985:
            m["completed"] = True

    # -- serialization for the frontend --------------------------------------
    def snapshot(self) -> dict:
        def agent_json(a: Agent):
            unc = self.planner.last_uncertainty.get(a.id)
            return dict(id=a.id, kind=a.kind, x=round(a.state.x, 2), y=round(a.state.y, 2),
                        theta=round(a.state.theta, 3), v=round(a.state.v, 2),
                        length=a.profile()["length"], width=a.profile()["width"],
                        is_ego=a.is_ego, trail=[[round(x, 2), round(y, 2)] for x, y in a.history[-40:]],
                        disturbed=len(a.disturbances) > 0,
                        behavior_profile=self.planner.last_behavior_profile.get(a.id),
                        uncertainty=(dict(behavior=unc.behavior, perception=unc.perception,
                                           occlusion=unc.occlusion, environment=unc.environment,
                                           total=round(unc.total, 3), dominant=unc.dominant_cause)
                                     if unc else None))

        cand_json = []
        if self.last_candidates:
            for c in self.last_candidates:
                cand_json.append(dict(name=c.name, label=c.label, cost=round(c.cost, 3),
                                       risk=round(c.risk, 3),
                                       points=[[round(x, 2), round(y, 2)] for (t, x, y, th) in c.points]))

        chosen_json = None
        if self.last_chosen is not None:
            chosen_json = dict(name=self.last_chosen.name, label=self.last_chosen.label,
                                points=[[round(x, 2), round(y, 2)] for (t, x, y, th) in self.last_chosen.points])

        verdict_json = None
        if self.last_verdict is not None:
            verdict_json = dict(approved=self.last_verdict.approved, reason=self.last_verdict.reason,
                                 min_clearance=round(self.last_verdict.min_clearance, 2))

        events_json = [dict(kind=e.kind, x=round(e.origin_pos[0], 2), y=round(e.origin_pos[1], 2),
                             magnitude=round(e.magnitude, 2), origin=e.origin_agent)
                       for e in self.ripple.recent_events()]

        obs_json = [dict(id=o.id, kind=o.kind, x=round(o.x, 2), y=round(o.y, 2),
                          confidence=o.confidence) for o in self.last_observations]

        corridor_json = [dict(s=st.s, x=st.x, y=st.y, left=st.left, right=st.right)
                          for st in self.planner.last_corridor]

        occlusion_json = [dict(x=z.x, y=z.y, r=z.radius, blocker=z.blocker_id)
                           for z in self.last_occlusion_zones]

        stress_json = None
        if self.last_stress is not None:
            stress_json = dict(total=self.last_stress.total, density=self.last_stress.density,
                                ripple_activity=self.last_stress.ripple_activity,
                                occlusion=self.last_stress.occlusion, planner_risk=self.last_stress.planner_risk)

        return dict(
            t=round(self.t, 2),
            seed=self.seed,
            road=dict(points=self.road.points, width=self.road.width, name=self.road.name),
            obstacles=[dict(x=o.position[0], y=o.position[1], r=o.radius, kind=o.kind) for o in self.scene.obstacles],
            bounds=self.scene.bounds,
            network=self.scene.network,
            conditions=self.scene.conditions,
            agents=[agent_json(a) for a in self.agents.values() if a.alive],
            ego_id=self.ego_id,
            candidates=cand_json,
            chosen=chosen_json,
            verdict=verdict_json,
            ripple_events=events_json,
            ripple_edges=[[u, v, round(w, 2)] for u, v, w in self.ripple.active_edges_for_viz],
            observations=obs_json,
            metrics=dict(self.metrics),
            plan_latency_ms=round(self.last_plan_latency_ms, 2),
            background_ai_enabled=self.background_ai_enabled,
            explanation=self.last_explanation,
            focus_agent_id=self.last_focus_agent_id,
            explain_struct=self.planner.last_explain_struct,
            fsm_state=self.fsm.state,
            fsm_description=self.fsm.description(),
            stress=stress_json,
            risk_budget=self.last_risk_budget,
            corridor=corridor_json,
            occlusion_zones=occlusion_json,
            sensor_trust=self.last_sensor_trust,
        )
