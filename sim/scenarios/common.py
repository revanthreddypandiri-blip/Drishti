"""Shared helpers for building scenario scenes and agent rosters. Every
scenario builder accepts a `seed`: it's used to procedurally vary agent
counts, spawn offsets, obstacle placement and lighting conditions within
realistic bounds, so the same scenario key produces a genuinely different
(but still representative) situation each time -- nothing here is a single
fixed hardcoded layout."""
from __future__ import annotations
import math
import random
from typing import List, Tuple
from sim.core.geometry import RoadSegment, StaticObstacle, Scene
from sim.agents.base import Agent

_counter = {"n": 0}

LIGHTING_CHOICES = ["day", "day", "day", "low_light", "fog"]


def next_id(prefix: str) -> str:
    _counter["n"] += 1
    return f"{prefix}{_counter['n']}"


def make_rng(seed) -> random.Random:
    return random.Random(seed if seed is not None else random.randrange(1_000_000))


def pick_lighting(rng: random.Random) -> str:
    return rng.choice(LIGHTING_CHOICES)


def jitter(rng: random.Random, base: float, spread: float) -> float:
    return base + rng.uniform(-spread, spread)


def straight_road(start: Tuple[float, float], end: Tuple[float, float], width: float,
                   n: int = 6, name: str = "road", width_jitter: float = 0.0,
                   rng: random.Random = None) -> RoadSegment:
    pts = []
    widths = []
    phase = rng.uniform(0, 6.28) if rng else 0.0
    for i in range(n):
        t = i / (n - 1)
        pts.append((start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t))
        w = width + (width_jitter * math.sin(t * 7.0 + phase))
        widths.append(max(2.5, w))
    return RoadSegment(points=pts, width=widths, name=name)


def make_ego(s0: float, lateral_pref: float = 0.0, target_speed_frac: float = 0.7) -> Agent:
    return Agent(id="ego", kind="ego", route_s0=s0, lateral_pref=lateral_pref,
                 target_speed_frac=target_speed_frac, is_ego=True)


def make_agent(kind: str, s0: float, lateral_pref: float = 0.0, target_speed_frac: float = 0.75,
               agent_id: str = None) -> Agent:
    return Agent(id=agent_id or next_id(kind[:3]), kind=kind, route_s0=s0,
                 lateral_pref=lateral_pref, target_speed_frac=target_speed_frac)


def make_wanderer(kind: str, s0: float, lateral_pref: float, goal: Tuple[float, float],
                   target_speed_frac: float = 0.8, agent_id: str = None) -> Agent:
    a = make_agent(kind, s0, lateral_pref, target_speed_frac, agent_id)
    a.goal_free_roam = goal
    return a


def with_trigger(agent: Agent, t: float, **updates) -> Agent:
    """Schedule a one-off scripted change (e.g. 'start crossing at t=6s')."""
    agent.triggers.append((t, updates))
    return agent
