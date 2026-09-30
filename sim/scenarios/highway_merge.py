"""Scenario 3: Highway merge involving slow-moving vehicles.
A tractor/truck and a slow auto merge into the main flow from the shoulder,
forcing the ego to negotiate speed differentials and an informal merge."""
from sim.core.geometry import StaticObstacle, Scene
from sim.scenarios.common import (straight_road, make_ego, make_agent, with_trigger,
                                   make_rng, pick_lighting, jitter)

TITLE = "Highway Merge (Slow-Moving Vehicles)"
DESCRIPTION = ("Higher-speed highway corridor. A slow truck and an auto merge in "
               "from the shoulder without signalling, well below highway speed.")


def build(seed=None):
    rng = make_rng(seed)
    road = straight_road((0, 0), (220, 0), width=jitter(rng, 8.0, 0.3), n=6, name=TITLE,
                          width_jitter=jitter(rng, 0.2, 0.1), rng=rng)
    scene = Scene(road=road, obstacles=[], bounds=(-5, -10, 225, 10), name=TITLE,
                  conditions={"lighting": pick_lighting(rng)}, seed=seed or 0)

    ego = make_ego(s0=jitter(rng, 10.0, 2.0), lateral_pref=jitter(rng, 0.3, 0.2),
                    target_speed_frac=jitter(rng, 0.9, 0.05))
    truck = make_agent("truck", s0=jitter(rng, 70.0, 8.0), lateral_pref=jitter(rng, 3.6, 0.3),
                        target_speed_frac=jitter(rng, 0.25, 0.05))
    with_trigger(truck, t=jitter(rng, 4.0, 1.0), lateral_pref=jitter(rng, 0.2, 0.2))
    slow_auto = make_agent("auto", s0=jitter(rng, 95.0, 8.0), lateral_pref=jitter(rng, -3.4, 0.3),
                            target_speed_frac=jitter(rng, 0.3, 0.05))
    with_trigger(slow_auto, t=jitter(rng, 6.5, 1.2), lateral_pref=jitter(rng, -0.4, 0.2))
    fast_car = make_agent("car", s0=jitter(rng, 40.0, 6.0), lateral_pref=jitter(rng, -0.5, 0.2),
                           target_speed_frac=jitter(rng, 1.0, 0.05))
    bike = make_agent("two_wheeler", s0=jitter(rng, 130.0, 8.0), lateral_pref=jitter(rng, 0.5, 0.3),
                       target_speed_frac=jitter(rng, 0.95, 0.05))
    agents = [ego, truck, slow_auto, fast_car, bike]
    if rng.random() < 0.4:
        agents.append(make_agent("car", s0=jitter(rng, 20.0, 4.0), lateral_pref=jitter(rng, 1.0, 0.3),
                                  target_speed_frac=jitter(rng, 0.85, 0.05)))
    return scene, agents
