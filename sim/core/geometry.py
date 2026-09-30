"""Road geometry primitives: centerline-defined roads, drivable-area polygons,
and basic vector/geometry helpers used across the simulation."""
from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import List, Tuple

Vec2 = Tuple[float, float]


def dist(a: Vec2, b: Vec2) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def lerp(a: Vec2, b: Vec2, t: float) -> Vec2:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def normalize(v: Vec2) -> Vec2:
    n = math.hypot(*v)
    if n < 1e-9:
        return (0.0, 0.0)
    return (v[0] / n, v[1] / n)


def heading_of(a: Vec2, b: Vec2) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


@dataclass
class RoadSegment:
    """A drivable region defined by an (irregular, possibly unmarked) centerline
    polyline and a width profile. No hard lanes are assumed -- Indian roads
    frequently have no painted lane markings, so we model the road as a single
    negotiable corridor rather than discrete lanes."""
    points: List[Vec2]                # centerline polyline
    width: List[float]                # width (m) at each point, same length as points
    name: str = "road"
    has_markings: bool = False        # whether soft "suggested lane" lines exist
    n_suggested_lanes: int = 1

    def __post_init__(self):
        assert len(self.points) == len(self.width) and len(self.points) >= 2
        self._seg_lengths = [dist(self.points[i], self.points[i + 1]) for i in range(len(self.points) - 1)]
        self._cum = [0.0]
        for L in self._seg_lengths:
            self._cum.append(self._cum[-1] + L)
        self.length = self._cum[-1]

    def _locate(self, s: float):
        s = max(0.0, min(self.length, s))
        for i in range(len(self._seg_lengths)):
            if s <= self._cum[i + 1] or i == len(self._seg_lengths) - 1:
                seg_len = self._seg_lengths[i] or 1e-9
                t = (s - self._cum[i]) / seg_len
                return i, max(0.0, min(1.0, t))
        return len(self._seg_lengths) - 1, 1.0

    def point_at(self, s: float) -> Vec2:
        i, t = self._locate(s)
        return lerp(self.points[i], self.points[i + 1], t)

    def heading_at(self, s: float) -> float:
        i, t = self._locate(s)
        return heading_of(self.points[i], self.points[i + 1])

    def width_at(self, s: float) -> float:
        i, t = self._locate(s)
        return self.width[i] + (self.width[i + 1] - self.width[i]) * t

    def project(self, p: Vec2) -> Tuple[float, float]:
        """Project point p onto the centerline. Returns (s, lateral_offset).
        lateral_offset > 0 means left of the direction of travel."""
        best_s, best_d, best_lat = 0.0, float("inf"), 0.0
        s_acc = 0.0
        for i in range(len(self.points) - 1):
            a, b = self.points[i], self.points[i + 1]
            seg = (b[0] - a[0], b[1] - a[1])
            seg_len = self._seg_lengths[i] or 1e-9
            ux, uy = seg[0] / seg_len, seg[1] / seg_len
            wx, wy = p[0] - a[0], p[1] - a[1]
            proj_len = wx * ux + wy * uy   # projection length in meters along the segment
            t = max(0.0, min(1.0, proj_len / seg_len))
            proj = (a[0] + ux * t * seg_len, a[1] + uy * t * seg_len)
            d = dist(p, proj)
            if d < best_d:
                best_d = d
                best_s = s_acc + t * seg_len
                # signed lateral offset (left positive) via cross product
                cross = ux * wy - uy * wx
                best_lat = cross
            s_acc += seg_len
        return best_s, best_lat

    def is_drivable(self, p: Vec2, margin: float = 0.0) -> bool:
        s, lat = self.project(p)
        w = self.width_at(s)
        return abs(lat) <= (w / 2.0 + margin)


@dataclass
class StaticObstacle:
    position: Vec2
    radius: float
    kind: str = "obstacle"   # e.g. "pothole", "debris", "barrier"


@dataclass
class Scene:
    road: RoadSegment
    obstacles: List[StaticObstacle] = field(default_factory=list)
    bounds: Tuple[float, float, float, float] = (0, 0, 100, 100)  # xmin,ymin,xmax,ymax
    name: str = "scene"
    conditions: dict = field(default_factory=lambda: {"lighting": "day"})
    seed: int = 0
    network: dict = field(default_factory=dict)
