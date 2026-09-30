"""Scenario 2: Busy, unsignalled urban intersection.
No traffic lights, no priority rules -- vehicles and pedestrians negotiate the
crossing informally. Modeled as a wide junction corridor with dense mixed
through-traffic plus pedestrians and a two-wheeler cutting in from the side."""
from sim.core.geometry import StaticObstacle, Scene
from sim.scenarios.common import (straight_road, make_ego, make_agent, make_wanderer,
                                   with_trigger, make_rng, pick_lighting, jitter)

TITLE = "Unsignalled Urban Intersection"
DESCRIPTION = ("A junction with no traffic signals or lane discipline: crossing "
               "pedestrians, a two-wheeler cutting in from the side, dense through-traffic.")


def build(seed=None):
    rng = make_rng(seed)
    road = straight_road((0, 0), (130, 0), width=jitter(rng, 9.0, 0.4), n=6, name=TITLE,
                          width_jitter=jitter(rng, 0.6, 0.2), rng=rng)
    scene = Scene(road=road, obstacles=[], bounds=(-5, -12, 135, 12), name=TITLE,
                  conditions={"lighting": pick_lighting(rng)}, seed=seed or 0)

    ego = make_ego(s0=jitter(rng, 5.0, 1.5), lateral_pref=0.0, target_speed_frac=jitter(rng, 0.55, 0.08))
    cross1 = make_agent("car", s0=jitter(rng, 40.0, 5.0), lateral_pref=jitter(rng, 1.5, 0.4),
                         target_speed_frac=jitter(rng, 0.4, 0.1))
    cross2 = make_agent("auto", s0=jitter(rng, 52.0, 5.0), lateral_pref=jitter(rng, -1.0, 0.4),
                         target_speed_frac=jitter(rng, 0.6, 0.1))
    cut_t = jitter(rng, 3.5, 1.0)
    cutter = make_agent("two_wheeler", s0=jitter(rng, 58.0, 5.0), lateral_pref=jitter(rng, 3.2, 0.5),
                         target_speed_frac=jitter(rng, 0.5, 0.1))
    with_trigger(cutter, t=cut_t, lateral_pref=jitter(rng, -0.3, 0.2), target_speed_frac=jitter(rng, 0.9, 0.1))
    ped1 = make_wanderer("pedestrian", s0=jitter(rng, 62.0, 4.0), lateral_pref=jitter(rng, 4.0, 0.4),
                          goal=(63, -4.0), target_speed_frac=jitter(rng, 0.6, 0.1))
    ped2 = make_wanderer("pedestrian", s0=jitter(rng, 66.0, 4.0), lateral_pref=jitter(rng, -4.0, 0.4),
                          goal=(65, 4.0), target_speed_frac=jitter(rng, 0.6, 0.1))
    bus = make_agent("bus", s0=jitter(rng, 80.0, 6.0), lateral_pref=jitter(rng, 0.8, 0.3),
                      target_speed_frac=jitter(rng, 0.35, 0.08))

    agents = [ego, cross1, cross2, cutter, ped1, ped2, bus]
    if rng.random() < 0.5:
        agents.append(make_wanderer("pedestrian", s0=jitter(rng, 70.0, 4.0), lateral_pref=jitter(rng, 3.5, 0.4),
                                     goal=(72, -3.5), target_speed_frac=jitter(rng, 0.55, 0.1)))
    if rng.random() < 0.35:
        agents.append(make_agent("two_wheeler", s0=jitter(rng, 30.0, 4.0), lateral_pref=jitter(rng, -1.8, 0.3),
                                  target_speed_frac=jitter(rng, 0.75, 0.1)))
    return scene, agents
