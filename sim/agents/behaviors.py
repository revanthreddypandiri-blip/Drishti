"""Per-tick motion controllers. Kept intentionally simple and rule-based:
- Wheeled agents: lateral control toward a (disturbance-modulated) preferred
  offset from centerline + IDM-like longitudinal car-following/obstacle
  reaction. Lane "discipline" scales how tightly an agent tracks its offset
  vs. wandering -- this is how autos/two-wheelers look less lane-bound than
  cars/buses, matching the PS's "informal merging" / "limited lane discipline".
- Soft agents (pedestrian/pushcart/animal): goal-biased random walk whose
  jitter and hesitation increase under disturbance (this is literally how
  "the pedestrian becomes uncertain" is implemented).
"""
from __future__ import annotations
import math
import random
from typing import Optional, List

from sim.core.kinematics import BicycleState, PointMassState, step_bicycle, step_point_mass
from sim.core.geometry import RoadSegment, StaticObstacle, dist
from sim.agents.base import Agent


def _nearest_obstacle_ahead(agent: Agent, road: RoadSegment, obstacles: List[StaticObstacle], s: float):
    best = None
    best_d = float("inf")
    for o in obstacles:
        os_, olat = road.project(o.position)
        ds = os_ - s
        if 0 < ds < best_d and abs(olat) < 3.0:
            best_d = ds
            best = (o, ds, olat)
    return best


def update_wheeled(agent: Agent, road: RoadSegment, obstacles: List[StaticObstacle],
                    leading_gap: Optional[float], dt: float, rng: random.Random):
    prof = agent.profile()
    st: BicycleState = agent.state
    dist_field = agent.net_disturbance()

    s, lat = road.project((st.x, st.y))
    max_speed = prof["max_speed"]
    target_speed = max_speed * agent.target_speed_frac * dist_field.speed_factor

    # --- react to a nearby static obstacle (e.g. pothole) by biasing offset ---
    obstacle_bias = 0.0
    ob = _nearest_obstacle_ahead(agent, road, obstacles, s)
    if ob is not None:
        o, ds, olat = ob
        if ds < 12.0 and o.kind == "pothole":
            # steer away from the obstacle's lateral position
            avoid_dir = -1.0 if olat >= 0 else 1.0
            closeness = max(0.0, 1.0 - ds / 12.0)
            obstacle_bias = avoid_dir * closeness * 1.6
            if ds < 6.0:
                target_speed *= max(0.3, 1.0 - closeness * 0.5)

    target_lat = agent.lateral_pref + dist_field.lateral_push + obstacle_bias
    w = road.width_at(s) / 2.0 - prof["width"] / 2.0 * 0.5
    target_lat = max(-w, min(w, target_lat))

    discipline = prof["discipline"]
    # lower discipline -> looser tracking + more wander noise
    lateral_gain = 0.5 + 1.0 * discipline
    wander = (1.0 - discipline) * rng.uniform(-0.35, 0.35)
    lat_error = (target_lat - lat) + wander

    road_heading = road.heading_at(s)
    heading_error = (road_heading - st.theta + math.pi) % (2 * math.pi) - math.pi
    # combine road-alignment with lateral correction as a steering command
    steer = 0.35 * heading_error + lateral_gain * 0.08 * lat_error
    steer += dist_field.heading_jitter * (1.0 - discipline) * rng.uniform(-1, 1)

    # --- longitudinal: simple IDM-style car following ---
    accel_max = 1.8
    comfort_decel = 2.5
    v = st.v
    if leading_gap is not None:
        s0 = 3.0
        T = 1.3
        desired_gap = s0 + max(0.0, v * T)
        gap = max(0.1, leading_gap)
        accel = accel_max * (1 - (v / max(target_speed, 0.5)) ** 4 - (desired_gap / gap) ** 2)
    else:
        accel = accel_max * (1 - (v / max(target_speed, 0.5)) ** 4) if target_speed > 0.1 else -comfort_decel

    if ob is not None and ob[1] < 5.0 and ob[0].kind == "pothole":
        accel = min(accel, -1.0)

    accel = max(-6.0, min(accel_max, accel))
    agent.state = step_bicycle(st, accel, steer, dt, max_speed=max_speed + 1.0, min_speed=0.0, max_steer=0.5)


def update_soft(agent: Agent, road: RoadSegment, dt: float, rng: random.Random):
    prof = agent.profile()
    st: PointMassState = agent.state
    dist_field = agent.net_disturbance()

    base_jitter = (1.0 - prof["discipline"]) * 1.2
    uncertainty = dist_field.heading_jitter  # ripple-driven "becomes uncertain"
    heading_rate = rng.uniform(-1, 1) * (base_jitter + uncertainty)

    target_speed = prof["max_speed"] * agent.target_speed_frac * dist_field.speed_factor
    if uncertainty > 0.5:
        # hesitation: agent slows/stalls when made uncertain by a ripple
        target_speed *= 0.25

    accel = 1.2 * (1 if st.v < target_speed else -1)

    if agent.goal_free_roam is not None:
        gx, gy = agent.goal_free_roam
        desired_heading = math.atan2(gy - st.y, gx - st.x)
        herr = (desired_heading - st.theta + math.pi) % (2 * math.pi) - math.pi
        heading_rate += 0.8 * herr

    agent.state = step_point_mass(st, accel, heading_rate, dt, max_speed=prof["max_speed"], min_speed=0.0)
