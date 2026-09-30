"""Scenario 1: Unmarked village road.
The flagship 'ripple' demo: a pothole causes an auto to swerve, a following
two-wheeler reacts, a nearby pedestrian becomes uncertain, and the ego must
read the emerging ripple and slow down before the conflict fully develops."""
from sim.core.geometry import StaticObstacle, Scene
from sim.scenarios.common import straight_road, make_ego, make_agent, make_wanderer, make_rng, pick_lighting, jitter

TITLE = "Unmarked Village Road"
DESCRIPTION = ("No lane markings, uneven road edges, a pothole mid-road. Watch the "
               "reaction ripple: pothole -> auto swerves -> two-wheeler reacts -> "
               "pedestrian hesitates -> ego slows pre-emptively.")


def build(seed=None):
    rng = make_rng(seed)
    road = straight_road((0, 0), (140, 0), width=jitter(rng, 7.0, 0.6), n=8, name=TITLE,
                          width_jitter=jitter(rng, 1.2, 0.3), rng=rng)
    pothole_s = jitter(rng, 45.0, 6.0)
    pothole_lat = jitter(rng, -1.3, 0.4)
    obstacles = [StaticObstacle(position=(pothole_s, pothole_lat), radius=0.6, kind="pothole")]
    scene = Scene(road=road, obstacles=obstacles, bounds=(-5, -10, 145, 10), name=TITLE,
                  conditions={"lighting": pick_lighting(rng)}, seed=seed or 0)

    ego = make_ego(s0=jitter(rng, 5.0, 1.5), lateral_pref=0.0, target_speed_frac=jitter(rng, 0.6, 0.08))
    auto = make_agent("auto", s0=jitter(rng, 28.0, 4.0), lateral_pref=jitter(rng, -0.9, 0.3),
                       target_speed_frac=jitter(rng, 0.7, 0.1))
    bike = make_agent("two_wheeler", s0=jitter(rng, 20.0, 4.0), lateral_pref=jitter(rng, -1.6, 0.3),
                       target_speed_frac=jitter(rng, 0.85, 0.1))
    car_ahead = make_agent("car", s0=jitter(rng, 60.0, 6.0), lateral_pref=jitter(rng, 0.6, 0.3),
                            target_speed_frac=jitter(rng, 0.45, 0.1))
    ped = make_wanderer("pedestrian", s0=jitter(rng, 48.0, 4.0), lateral_pref=jitter(rng, 3.0, 0.5),
                         goal=(50, -3.2), target_speed_frac=jitter(rng, 0.55, 0.1))
    agents = [ego, auto, bike, car_ahead, ped]

    if rng.random() < 0.7:
        agents.append(make_wanderer("animal", s0=jitter(rng, 95.0, 8.0), lateral_pref=jitter(rng, 2.6, 0.4),
                                     goal=(98, -2.4), target_speed_frac=jitter(rng, 0.25, 0.08)))
    if rng.random() < 0.4:
        agents.append(make_wanderer("pedestrian", s0=jitter(rng, 15.0, 4.0), lateral_pref=jitter(rng, -2.8, 0.4),
                                     goal=(17, 2.8), target_speed_frac=jitter(rng, 0.5, 0.1)))

    return scene, agents
