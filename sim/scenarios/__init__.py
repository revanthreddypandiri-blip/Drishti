from sim.scenarios import village_road, urban_intersection, highway_merge, market_area, cattle_crossing

REGISTRY = {
    "village_road": village_road,
    "urban_intersection": urban_intersection,
    "highway_merge": highway_merge,
    "market_area": market_area,
    "cattle_crossing": cattle_crossing,
}


def build(name: str, seed=None):
    mod = REGISTRY[name]
    scene, agents = mod.build(seed=seed)
    from sim.network import build_network
    scene.network = build_network(name, scene.road, seed)
    return scene, agents


def list_scenarios():
    return [dict(key=k, title=m.TITLE, description=m.DESCRIPTION) for k, m in REGISTRY.items()]
