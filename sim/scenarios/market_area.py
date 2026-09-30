"""Scenario 4: Dense market area with mixed traffic.
Narrow, crowded corridor with pushcarts, pedestrians, autos and two-wheelers
all sharing the same tight space at low speed -- tests planning under heavy,
persistent clutter rather than a single sharp event."""
from sim.core.geometry import StaticObstacle, Scene
from sim.scenarios.common import (straight_road, make_ego, make_agent, make_wanderer,
                                   make_rng, pick_lighting, jitter)

TITLE = "Dense Market Area"
DESCRIPTION = ("Narrow, crowded market street: pushcarts, pedestrians, autos and "
               "two-wheelers packed into a tight corridor at low speed.")


def build(seed=None):
    rng = make_rng(seed)
    road = straight_road((0, 0), (100, 0), width=jitter(rng, 5.5, 0.5), n=6, name=TITLE,
                          width_jitter=jitter(rng, 0.8, 0.25), rng=rng)
    obstacles = [StaticObstacle(position=(jitter(rng, 35.0, 5.0), jitter(rng, 0.5, 0.4)), radius=0.5, kind="debris"),
                 StaticObstacle(position=(jitter(rng, 70.0, 5.0), jitter(rng, -0.8, 0.4)), radius=0.5, kind="debris")]
    scene = Scene(road=road, obstacles=obstacles, bounds=(-5, -9, 105, 9), name=TITLE,
                  conditions={"lighting": pick_lighting(rng)}, seed=seed or 0)

    ego = make_ego(s0=jitter(rng, 3.0, 1.0), lateral_pref=0.0, target_speed_frac=jitter(rng, 0.4, 0.08))
    cart1 = make_agent("pushcart", s0=jitter(rng, 25.0, 4.0), lateral_pref=jitter(rng, 1.2, 0.3),
                        target_speed_frac=jitter(rng, 0.4, 0.1))
    cart2 = make_agent("pushcart", s0=jitter(rng, 55.0, 4.0), lateral_pref=jitter(rng, -1.0, 0.3),
                        target_speed_frac=jitter(rng, 0.4, 0.1))
    auto = make_agent("auto", s0=jitter(rng, 40.0, 4.0), lateral_pref=jitter(rng, -1.5, 0.3),
                       target_speed_frac=jitter(rng, 0.5, 0.1))
    bike1 = make_agent("two_wheeler", s0=jitter(rng, 18.0, 3.0), lateral_pref=jitter(rng, 0.8, 0.3),
                        target_speed_frac=jitter(rng, 0.6, 0.1))
    bike2 = make_agent("two_wheeler", s0=jitter(rng, 48.0, 3.0), lateral_pref=jitter(rng, 1.4, 0.3),
                        target_speed_frac=jitter(rng, 0.55, 0.1))
    n_peds = rng.randint(3, 6)
    peds = [make_wanderer("pedestrian", s0=jitter(rng, 20.0 + i * 12, 3.0),
                           lateral_pref=(1 if i % 2 else -1) * jitter(rng, 1.8, 0.3),
                           goal=(20 + i * 12 + 3, -(1 if i % 2 else -1) * 1.8),
                           target_speed_frac=jitter(rng, 0.5, 0.1))
            for i in range(n_peds)]

    return scene, [ego, cart1, cart2, auto, bike1, bike2] + peds
