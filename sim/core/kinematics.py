"""Kinematic motion models. Wheeled agents (car/bus/truck/ego/auto/two-wheeler)
use a kinematic bicycle model. Soft agents (pedestrian/pushcart/animal) use a
point-mass model with heading smoothing."""
from __future__ import annotations
import math
from dataclasses import dataclass


@dataclass
class BicycleState:
    x: float
    y: float
    theta: float      # heading, radians
    v: float           # longitudinal speed, m/s
    wheelbase: float = 2.6


def step_bicycle(state: BicycleState, accel: float, steer: float, dt: float,
                  max_speed: float = 20.0, min_speed: float = -3.0,
                  max_steer: float = 0.6) -> BicycleState:
    steer = max(-max_steer, min(max_steer, steer))
    v = state.v + accel * dt
    v = max(min_speed, min(max_speed, v))
    x = state.x + v * math.cos(state.theta) * dt
    y = state.y + v * math.sin(state.theta) * dt
    theta = state.theta + (v / max(state.wheelbase, 0.5)) * math.tan(steer) * dt
    theta = (theta + math.pi) % (2 * math.pi) - math.pi
    return BicycleState(x, y, theta, v, state.wheelbase)


@dataclass
class PointMassState:
    x: float
    y: float
    theta: float
    v: float


def step_point_mass(state: PointMassState, accel: float, heading_rate: float, dt: float,
                     max_speed: float = 3.0, min_speed: float = 0.0) -> PointMassState:
    v = max(min_speed, min(max_speed, state.v + accel * dt))
    theta = state.theta + heading_rate * dt
    theta = (theta + math.pi) % (2 * math.pi) - math.pi
    x = state.x + v * math.cos(theta) * dt
    y = state.y + v * math.sin(theta) * dt
    return PointMassState(x, y, theta, v)
