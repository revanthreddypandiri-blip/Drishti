"""Simulated multi-sensor perception (camera + LiDAR + radar), fused into a
single noisy, occlusion-aware detection list for the ego vehicle. We don't
simulate raw pixels/point-clouds -- this is a *simulation-first* prototype --
but we do simulate the failure modes that matter for planning: limited field
of view, range limits, occlusion by closer agents, sensor noise, and
*adaptive sensor trust* (each sensor's reliability shifts with conditions,
e.g. camera confidence drops in low light, radar becomes relatively more
useful for fast movers) -- so the planner works with genuinely imperfect,
condition-dependent information, and we also surface *occlusion zones*:
regions behind large vehicles that could be hiding an undetected road user,
even when nothing is actually there in a given run."""
from __future__ import annotations
import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from sim.agents.base import Agent
from sim.prediction.intent import ObservedAgent

LIDAR_RANGE = 45.0
CAMERA_FOV = math.radians(140)   # forward camera cone
RADAR_RANGE = 60.0
RADAR_FOV = math.radians(90)

LARGE_BLOCKER_KINDS = {"bus", "truck"}

CONDITION_TRUST = {
    # lighting -> (camera_trust, lidar_trust, radar_trust) multipliers
    "day":       (1.00, 1.00, 0.85),
    "low_light": (0.55, 0.95, 0.90),
    "fog":       (0.45, 0.65, 1.00),
}


def _segment_circle_blocked(ego_pos, target_pos, blocker_pos, blocker_r) -> bool:
    ex, ey = ego_pos
    tx, ty = target_pos
    bx, by = blocker_pos
    seg = (tx - ex, ty - ey)
    seg_len = math.hypot(*seg) or 1e-6
    u = (seg[0] / seg_len, seg[1] / seg_len)
    w = (bx - ex, by - ey)
    proj = w[0] * u[0] + w[1] * u[1]
    if proj <= 0.2 or proj >= seg_len - 0.2:
        return False
    closest = (ex + u[0] * proj, ey + u[1] * proj)
    perp_dist = math.hypot(bx - closest[0], by - closest[1])
    return perp_dist < blocker_r * 0.9


@dataclass
class OcclusionZone:
    x: float
    y: float
    radius: float
    blocker_id: str


@dataclass
class PerceptionResult:
    observations: List[ObservedAgent] = field(default_factory=list)
    occluded_ids: List[str] = field(default_factory=list)
    occlusion_zones: List[OcclusionZone] = field(default_factory=list)
    sensor_trust: Dict[str, float] = field(default_factory=dict)


class SensorSuite:
    def __init__(self, seed: int = 3):
        self.rng = random.Random(seed)
        self._prev_obs: Dict[str, ObservedAgent] = {}

    def perceive(self, ego: Agent, all_agents: Dict[str, Agent],
                 lighting: str = "day") -> PerceptionResult:
        ex, ey = ego.pos()
        eh = ego.heading()
        cam_trust, lidar_trust, radar_trust = CONDITION_TRUST.get(lighting, CONDITION_TRUST["day"])
        results: List[ObservedAgent] = []
        occluded_ids: List[str] = []
        occlusion_zones: List[OcclusionZone] = []
        others = [a for a in all_agents.values() if a.id != ego.id and a.alive]

        for a in others:
            ax, ay = a.pos()
            dx, dy = ax - ex, ay - ey
            r = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx) - eh
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi

            lidar_sees = r <= LIDAR_RANGE * (0.75 if lighting == "fog" else 1.0)
            camera_sees = r <= LIDAR_RANGE and abs(bearing) <= CAMERA_FOV / 2
            radar_sees = r <= RADAR_RANGE and abs(bearing) <= RADAR_FOV / 2
            if not (lidar_sees or camera_sees or radar_sees):
                continue

            occluded = False
            blocker = None
            for b in others:
                if b.id == a.id:
                    continue
                bx, by = b.pos()
                br = math.hypot(bx - ex, by - ey)
                if br < r and _segment_circle_blocked((ex, ey), (ax, ay), (bx, by), b.bbox_radius()):
                    occluded = True
                    blocker = b
                    break
            if occluded and r > 6.0:
                occluded_ids.append(a.id)
                continue

            # fuse three sensors with condition-adjusted trust, favoring
            # whichever is currently most reliable rather than a flat blend
            camera_conf = cam_trust if camera_sees else 0.0
            lidar_conf = lidar_trust if lidar_sees else 0.0
            radar_conf = (radar_trust * (1.15 if a.speed() > 4.0 else 0.9)) if radar_sees else 0.0
            fused = max(camera_conf, lidar_conf, radar_conf)
            range_falloff = max(0.15, 1.0 - r / (LIDAR_RANGE * 1.3))
            confidence = max(0.1, min(1.0, fused * range_falloff))
            noise_scale = (1.0 - confidence) * 1.2

            nx = ax + self.rng.gauss(0, noise_scale * 0.4)
            ny = ay + self.rng.gauss(0, noise_scale * 0.4)
            ntheta = a.heading() + self.rng.gauss(0, noise_scale * 0.15)
            nv = max(0.0, a.speed() + self.rng.gauss(0, noise_scale * 0.3))

            prev = self._prev_obs.get(a.id)
            obs = ObservedAgent(
                id=a.id, kind=a.kind if camera_sees else "unknown_agent",
                x=nx, y=ny, theta=ntheta, v=nv,
                prev_v=(prev.v if prev else None), prev_theta=(prev.theta if prev else None),
                confidence=round(confidence, 2),
            )
            results.append(obs)
            self._prev_obs[a.id] = obs

        # phantom occlusion zones: a region behind any large vehicle within
        # forward sensor range could hide an undetected road user, whether or
        # not one actually happens to be there this run
        for a in others:
            if a.kind not in LARGE_BLOCKER_KINDS:
                continue
            ax, ay = a.pos()
            r = math.hypot(ax - ex, ay - ey)
            if r > LIDAR_RANGE * 0.8:
                continue
            bearing = math.atan2(ay - ey, ax - ex) - eh
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
            if abs(bearing) > CAMERA_FOV / 2:
                continue
            away = math.atan2(ay - ey, ax - ex)
            depth = a.bbox_radius() + 3.0
            zx = ax + math.cos(away) * depth
            zy = ay + math.sin(away) * depth
            occlusion_zones.append(OcclusionZone(x=round(zx, 2), y=round(zy, 2),
                                                  radius=round(a.bbox_radius() + 1.5, 2),
                                                  blocker_id=a.id))

        seen_ids = {o.id for o in results}
        for k in list(self._prev_obs.keys()):
            if k not in seen_ids:
                self._prev_obs.pop(k, None)

        avg_conf = sum(o.confidence for o in results) / len(results) if results else 1.0
        sensor_trust = dict(camera=round(cam_trust, 2), lidar=round(lidar_trust * avg_conf, 2),
                             radar=round(radar_trust, 2))

        return PerceptionResult(observations=results, occluded_ids=occluded_ids,
                                 occlusion_zones=occlusion_zones, sensor_trust=sensor_trust)
