"""Road-network layer: side roads / T-junctions / 4-way junctions / merging slip roads,
conflict zones, zebra + informal crossings, speed breakers, drains, medians, work zones,
and the Indian roadside (shops, houses, garages, walls, gates, bus stops, carts, poles,
parked cars). Deterministic per (scenario, seed). Sent to the frontend inside every snapshot."""
import random

# arms: (x on main road, side +1/-1, width m, x-drift of the far end -> angled merge)
CFG = {
    "urban_intersection": dict(arms=[(60, 1, 8, 0), (60, -1, 8, 0)], zebra=[50, 70], inf=[102], mark="broken", bld="dense"),
    "village_road": dict(arms=[(90, 1, 5, 0)], zebra=[], inf=[48], mark="none", bld="village", bump=[25], drain=[70, 118]),
    "highway_merge": dict(arms=[(55, -1, 4, 38)], zebra=[], inf=[], mark="solid", bld="sparse", median=True, bump=[]),
    "market_area": dict(arms=[(40, 1, 6, 0), (100, -1, 6, 0)], zebra=[70], inf=[], mark="faded", bld="dense",
                        bus=[78], work=(112, 130), bump=[22], drain=[55]),
    "cattle_crossing": dict(arms=[(70, -1, 4.5, 0)], zebra=[], inf=[95], mark="none", bld="village", drain=[40]),
}
KINDS = {"dense": ["shop", "shop", "commercial", "house", "garage", "shop", "wall"],
         "village": ["house", "hut", "hut", "wall", "gate", "house", "garage"],
         "sparse": ["wall", "tree", "tree", "wall"]}


def build_network(key, road, seed):
    c = CFG.get(key)
    if not c:
        return {}
    r = random.Random((seed or 0) * 31 + len(key))
    L = road.length
    H = road.width_at(L / 2) / 2.0 + 2.0          # road edge + dirt shoulder
    arms = [dict(x=x, d=d, w=w, x0=x + dx, y0=d * H * 0.3, x1=x, y1=d * (H + 45), zebra=w >= 6)
            for x, d, w, dx in c["arms"]]
    for a in arms:                                # angled slip road: far end drifts, joins at x
        a["x0"], a["x1"] = a["x"], a["x"] + (a["x0"] - a["x"])
        a["y0"], a["y1"] = a["d"] * H * 0.3, a["d"] * (H + 45)
    clear = [(a["x"], a["d"], a["w"] / 2 + 5 + abs(a["x1"] - a["x0"]) * 0.4) for a in arms]
    clear += [(b, 1, 7) for b in c.get("bus", [])]
    objs, x = [], 3.0
    kinds = KINDS[c["bld"]]
    while x < L - 3:
        for side in (1, -1):
            if any(sd == side and abs(x - cx) < hw for cx, sd, hw in clear):
                continue
            k = r.choice(kinds)
            w, d = (r.uniform(5, 9), r.uniform(4, 7)) if k not in ("tree", "wall", "hut") else \
                   ((r.uniform(3, 5), 3.2) if k == "hut" else (r.uniform(4, 8), 0.5 if k == "wall" else 2.6))
            y = side * (H + 1.8 + d / 2 + r.uniform(0, 1.6))
            objs.append(dict(k=k, x=x + w / 2, y=y, w=w, d=d, s=side, c=round(r.random(), 3)))
            if r.random() < 0.55:                  # roadside clutter on the shoulder
                pk = r.choice(["cart", "parked", "pole", "tree", "bin", "parked", "cart"])
                objs.append(dict(k=pk, x=x + r.uniform(0, w), y=side * (H - 0.6 - r.random() * 0.4),
                                 w=1.8 if pk in ("parked", "cart") else 0.7, d=1.0, s=side, c=round(r.random(), 3)))
        x += 8.5 + r.uniform(0, 3)
    for px in range(8, int(L), 16):                # street poles
        objs.append(dict(k="pole", x=px + r.uniform(-1, 1), y=-(H + 0.4), w=.5, d=.5, s=-1, c=0))
    work = None
    if c.get("work"):
        x0, x1 = c["work"]; work = dict(x0=x0, x1=x1, y=-(H - 2.0))
        for bx in range(int(x0), int(x1), 3):
            objs.append(dict(k="barrier", x=bx + 1.0, y=-(H - 2.6), w=2.0, d=.5, s=-1, c=0))
        for bx in (x0 - 4, x0 - 8, x1 + 3):
            objs.append(dict(k="cone", x=bx, y=-(H - 2.4), w=.5, d=.5, s=-1, c=0))
    return dict(
        H=round(H, 2), L=L, arms=arms, objs=objs, mark=c["mark"], work=work,
        zebra=[dict(x=z) for z in c.get("zebra", [])], informal=[dict(x=z) for z in c.get("inf", [])],
        bumps=c.get("bump", []), drains=c.get("drain", []), bus=c.get("bus", []), median=bool(c.get("median")),
        conflict=[dict(x=a["x"], w=a["w"] + (abs(a["x1"] - a["x0"]) * 0.3)) for a in arms],
    )
