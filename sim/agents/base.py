"""Agent representation. Every road user (ego included) is an Agent with a
type-specific kinematic model, a nominal behavior, and a "disturbance" state
that ripple propagation writes into (this is how ripples actually change an
agent's motion: a disturbance nudges target lateral offset / target speed /
adds a heading jitter, on top of the agent's normal following behavior)."""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import math

from sim.core.kinematics import BicycleState, PointMassState

WHEELED = {"ego", "car", "bus", "truck", "auto", "two_wheeler"}
SOFT = {"pedestrian", "pushcart", "animal"}

# Rough per-type physical + behavioral profiles (length,width in meters,
# nominal max speed m/s, lane-discipline 0..1 (1=strict corridor centering,
# 0=highly irregular/erratic lateral wander), reaction time seconds)
TYPE_PROFILE = {
    "ego":         dict(length=4.4, width=1.9, max_speed=16.0, discipline=0.9, reaction=0.6),
    "car":         dict(length=4.3, width=1.8, max_speed=16.0, discipline=0.8, reaction=0.9),
    "bus":         dict(length=10.0, width=2.5, max_speed=12.0, discipline=0.85, reaction=1.2),
    "truck":       dict(length=8.0, width=2.4, max_speed=12.0, discipline=0.85, reaction=1.2),
    "auto":        dict(length=2.6, width=1.4, max_speed=11.0, discipline=0.4, reaction=0.5),
    "two_wheeler": dict(length=1.9, width=0.7, max_speed=14.0, discipline=0.25, reaction=0.35),
    "pedestrian":  dict(length=0.5, width=0.5, max_speed=1.8, discipline=0.1, reaction=0.7),
    "pushcart":    dict(length=1.6, width=0.9, max_speed=1.2, discipline=0.3, reaction=1.0),
    "animal":      dict(length=1.4, width=0.6, max_speed=2.5, discipline=0.0, reaction=1.5),
}


@dataclass
class Disturbance:
    """A transient reaction injected by ripple propagation."""
    source_event: str
    lateral_push: float = 0.0     # desired extra lateral offset (m), sign = left/right
    speed_factor: float = 1.0     # multiplies target speed (e.g. 0.3 = hard slow)
    heading_jitter: float = 0.0   # extra random heading noise (rad/s scale)
    time_left: float = 0.0        # seconds remaining
    magnitude: float = 1.0        # 0..1, used for ripple decay across the graph


@dataclass
class Agent:
    id: str
    kind: str                      # one of TYPE_PROFILE keys
    route_s0: float                # starting arc-length position along the road
    lateral_pref: float = 0.0       # nominal preferred lateral offset from centerline (m)
    target_speed_frac: float = 0.75  # fraction of type max_speed this agent nominally targets
    goal_free_roam: Optional[Tuple[float, float]] = None  # for soft agents: a wander target (x,y)
    is_ego: bool = False
    triggers: List[Tuple[float, dict]] = field(default_factory=list)  # (sim_time, {attr: new_value})

    # runtime state, populated by Simulation.spawn()
    state: Optional[object] = None
    alive: bool = True
    disturbances: List[Disturbance] = field(default_factory=list)
    history: List[Tuple[float, float]] = field(default_factory=list)  # trail for viz

    def profile(self):
        return TYPE_PROFILE[self.kind]

    def is_wheeled(self) -> bool:
        return self.kind in WHEELED

    def bbox_radius(self) -> float:
        p = self.profile()
        return math.hypot(p["length"], p["width"]) / 2.0

    def add_disturbance(self, d: Disturbance):
        self.disturbances.append(d)

    def tick_disturbances(self, dt: float):
        for d in self.disturbances:
            d.time_left -= dt
        self.disturbances = [d for d in self.disturbances if d.time_left > 0]

    def net_disturbance(self) -> Disturbance:
        if not self.disturbances:
            return Disturbance(source_event="none")
        lateral = sum(d.lateral_push * d.magnitude for d in self.disturbances)
        speed_factor = min((d.speed_factor for d in self.disturbances), default=1.0)
        jitter = sum(d.heading_jitter * d.magnitude for d in self.disturbances)
        return Disturbance(source_event="combined", lateral_push=lateral,
                            speed_factor=speed_factor, heading_jitter=jitter,
                            time_left=1.0, magnitude=1.0)

    def apply_due_triggers(self, t: float):
        """One-off scripted state changes (e.g. 'start crossing now', 'begin
        merge into corridor now') used by scenarios to stage specific moments
        (a cattle crossing, a merge) without hand-animating every frame."""
        still = []
        for trigger_t, updates in self.triggers:
            if t >= trigger_t:
                for k, v in updates.items():
                    setattr(self, k, v)
            else:
                still.append((trigger_t, updates))
        self.triggers = still

    def pos(self) -> Tuple[float, float]:
        return (self.state.x, self.state.y)

    def heading(self) -> float:
        return self.state.theta

    def speed(self) -> float:
        return self.state.v
