"""Scenario 5: Sudden cattle-crossing event.
Cattle graze at the roadside and then, abruptly, cross -- a sharp, unpredicted
event with no warning, testing the safety shield's independent verification
as much as the main planner's anticipation."""
from sim.core.geometry import Scene
from sim.scenarios.common import (straight_road, make_ego, make_agent, make_wanderer,
                                   with_trigger, make_rng, pick_lighting, jitter)

TITLE = "Sudden Cattle Crossing"
DESCRIPTION = ("Cattle graze at the roadside then suddenly cross with no warning -- "
               "a sharp unpredicted event stress-testing both planner and safety shield.")


def build(seed=None):
    rng = make_rng(seed)
    road = straight_road((0, 0), (150, 0), width=jitter(rng, 7.5, 0.4), n=6, name=TITLE,
                          width_jitter=jitter(rng, 0.3, 0.15), rng=rng)
    scene = Scene(road=road, obstacles=[], bounds=(-5, -10, 155, 10), name=TITLE,
                  conditions={"lighting": pick_lighting(rng)}, seed=seed or 0)

    ego = make_ego(s0=jitter(rng, 5.0, 1.5), lateral_pref=0.0, target_speed_frac=jitter(rng, 0.75, 0.06))
    cow_s = jitter(rng, 70.0, 8.0)
    cow1 = make_wanderer("animal", s0=cow_s, lateral_pref=jitter(rng, 4.5, 0.3), goal=(cow_s + 2, -4.5),
                          target_speed_frac=0.03)
    with_trigger(cow1, t=jitter(rng, 5.0, 1.2), target_speed_frac=jitter(rng, 0.8, 0.1))
    cow2 = make_wanderer("animal", s0=cow_s + jitter(rng, 4.0, 1.0), lateral_pref=jitter(rng, 4.8, 0.3),
                          goal=(cow_s + 6, -4.5), target_speed_frac=0.03)
    with_trigger(cow2, t=jitter(rng, 5.6, 1.2), target_speed_frac=jitter(rng, 0.7, 0.1))
    slow_car = make_agent("car", s0=jitter(rng, 45.0, 5.0), lateral_pref=jitter(rng, 0.3, 0.3),
                           target_speed_frac=jitter(rng, 0.6, 0.1))
    bike = make_agent("two_wheeler", s0=jitter(rng, 95.0, 6.0), lateral_pref=jitter(rng, -0.5, 0.3),
                       target_speed_frac=jitter(rng, 0.7, 0.1))
    agents = [ego, cow1, cow2, slow_car, bike]
    if rng.random() < 0.5:
        cow3_s = cow_s + jitter(rng, 8.0, 1.5)
        cow3 = make_wanderer("animal", s0=cow3_s, lateral_pref=jitter(rng, 4.6, 0.3), goal=(cow3_s + 2, -4.5),
                              target_speed_frac=0.03)
        with_trigger(cow3, t=jitter(rng, 6.2, 1.2), target_speed_frac=jitter(rng, 0.6, 0.1))
        agents.append(cow3)
    return scene, agents
