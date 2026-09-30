"""Dynamic interaction graph + traffic ripple propagation.

This is the centerpiece of Drishti: instead of treating road users as
independent obstacles, we (1) connect nearby agents into a proximity-weighted
interaction graph every tick, (2) detect "root events" -- an agent braking
hard or swerving hard -- and (3) propagate a decaying, delayed reaction
(a Disturbance, see agents/base.py) outward through the graph, hop by hop,
so that e.g. a pothole-induced auto-swerve reaches a following motorcyclist a
beat later, and a pedestrian near that motorcyclist becomes "uncertain" a beat
after that. Everything is rule-based and fully inspectable/loggable.
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional
import networkx as nx

from sim.agents.base import Agent, Disturbance
from sim.core.geometry import RoadSegment, dist

INFLUENCE_RADIUS = 16.0        # m, beyond this agents don't directly interact
MAX_HOPS = 3
MIN_MAGNITUDE = 0.08
BRAKE_EVENT_DECEL = 2.2         # m/s^2 drop within one tick window to count as "hard brake"
SWERVE_EVENT_LAT_RATE = 0.9     # m/s lateral-offset change to count as "swerve"


@dataclass
class RippleEvent:
    id: str
    kind: str            # "brake" | "swerve" | "stop" | "cross"
    origin_agent: str
    origin_pos: Tuple[float, float]
    magnitude: float
    t_created: float


@dataclass
class ScheduledDisturbance:
    deliver_t: float
    target_id: str
    disturbance: Disturbance
    hop: int
    event_kind: str
    origin_pos: Tuple[float, float]


class RippleEngine:
    def __init__(self):
        self.graph = nx.DiGraph()
        self._pending: List[ScheduledDisturbance] = []
        self._prev_state: Dict[str, Tuple[float, float]] = {}  # id -> (speed, lateral)
        self._events: List[RippleEvent] = []          # recent events, for viz/logging
        self._event_counter = 0
        self.active_edges_for_viz: List[Tuple[str, str, float]] = []

    def build_graph(self, agents: Dict[str, Agent], road: RoadSegment):
        g = nx.DiGraph()
        for a in agents.values():
            if a.alive:
                g.add_node(a.id)
        ids = [a.id for a in agents.values() if a.alive]
        for i in ids:
            ai = agents[i]
            for j in ids:
                if i == j:
                    continue
                aj = agents[j]
                d = dist(ai.pos(), aj.pos())
                if d < INFLUENCE_RADIUS:
                    w = math.exp(-d / (INFLUENCE_RADIUS * 0.5))
                    g.add_edge(i, j, weight=w, dist=d)
        self.graph = g
        self.active_edges_for_viz = [(u, v, d["weight"]) for u, v, d in g.edges(data=True)]

    def _detect_events(self, agents: Dict[str, Agent], road: RoadSegment, t: float, dt: float):
        for a in agents.values():
            if not a.alive:
                continue
            s, lat = road.project(a.pos())
            prev = self._prev_state.get(a.id)
            if prev is not None:
                prev_speed, prev_lat = prev
                decel = (prev_speed - a.speed()) / max(dt, 1e-3)
                lat_rate = abs(lat - prev_lat) / max(dt, 1e-3)
                if decel > BRAKE_EVENT_DECEL and a.speed() < prev_speed:
                    self._spawn_event(a, "brake", min(1.0, decel / 6.0), t, agents, road)
                elif lat_rate > SWERVE_EVENT_LAT_RATE:
                    self._spawn_event(a, "swerve", min(1.0, lat_rate / 2.5), t, agents, road)
                elif a.kind in ("pedestrian",) and a.speed() < 0.15 and prev_speed > 0.4:
                    self._spawn_event(a, "stop", 0.5, t, agents, road)
            self._prev_state[a.id] = (a.speed(), lat)

    def _spawn_event(self, agent: Agent, kind: str, magnitude: float, t: float,
                      agents: Dict[str, Agent], road: RoadSegment):
        self._event_counter += 1
        ev = RippleEvent(id=f"ev{self._event_counter}", kind=kind, origin_agent=agent.id,
                          origin_pos=agent.pos(), magnitude=magnitude, t_created=t)
        self._events.append(ev)
        self._events = self._events[-60:]
        self._propagate(agent.id, kind, magnitude, t, hop=0, agents=agents, origin_pos=agent.pos())

    def _propagate(self, source_id: str, kind: str, magnitude: float, t: float, hop: int,
                    agents: Dict[str, Agent], origin_pos: Tuple[float, float]):
        if hop >= MAX_HOPS or magnitude < MIN_MAGNITUDE or source_id not in self.graph:
            return
        for nb in self.graph.successors(source_id):
            if nb == source_id:
                continue
            w = self.graph[source_id][nb]["weight"]
            d = self.graph[source_id][nb]["dist"]
            passed_mag = magnitude * w * 0.85
            if passed_mag < MIN_MAGNITUDE:
                continue
            target = agents.get(nb)
            if target is None or not target.alive:
                continue
            reaction_time = target.profile()["reaction"]
            delay = reaction_time + d / 25.0   # perception+reaction delay, distance-scaled
            dist_obj = self._make_disturbance(kind, passed_mag, target)
            self._pending.append(ScheduledDisturbance(
                deliver_t=t + delay, target_id=nb, disturbance=dist_obj, hop=hop + 1,
                event_kind=kind, origin_pos=origin_pos))

    def _make_disturbance(self, kind: str, magnitude: float, target: Agent) -> Disturbance:
        if kind == "brake":
            return Disturbance(source_event=kind, speed_factor=max(0.15, 1.0 - magnitude),
                                lateral_push=0.0, heading_jitter=0.0, time_left=1.2, magnitude=magnitude)
        if kind == "swerve":
            side = 1.0 if target.lateral_pref >= 0 else -1.0
            return Disturbance(source_event=kind, lateral_push=side * magnitude * 1.3,
                                speed_factor=max(0.5, 1.0 - magnitude * 0.5),
                                heading_jitter=magnitude * 0.6, time_left=1.0, magnitude=magnitude)
        if kind == "stop":
            return Disturbance(source_event=kind, speed_factor=max(0.2, 1.0 - magnitude),
                                heading_jitter=magnitude * 1.2, time_left=1.4, magnitude=magnitude)
        return Disturbance(source_event=kind, time_left=0.5, magnitude=magnitude)

    def step(self, agents: Dict[str, Agent], road: RoadSegment, t: float, dt: float):
        self.build_graph(agents, road)
        self._detect_events(agents, road, t, dt)

        still_pending = []
        for sch in self._pending:
            if sch.deliver_t <= t:
                target = agents.get(sch.target_id)
                if target is not None and target.alive:
                    target.add_disturbance(sch.disturbance)
                    self._propagate(sch.target_id, sch.event_kind, sch.disturbance.magnitude, t,
                                     sch.hop, agents, sch.origin_pos)
            else:
                still_pending.append(sch)
        self._pending = still_pending

        for a in agents.values():
            a.tick_disturbances(dt)

    def recent_events(self) -> List[RippleEvent]:
        return list(self._events[-20:])

    def pending_count(self) -> int:
        return len(self._pending)
