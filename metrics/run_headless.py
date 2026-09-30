"""Headless multi-seed batch runner: simulates all 5 required scenarios across
many seeds (each seed procedurally generates a different layout) and reports
the metrics the SIH problem statement asks for: collision-free performance,
replanning latency, path smoothness, scenario completion / progress.

    python -m metrics.run_headless            # 10 seeds x 5 scenarios x 25 s
    python -m metrics.run_headless 20 40      # 20 seeds, 40 s each
"""
import json, os, statistics, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from sim.scenarios import build, list_scenarios
from sim.simulation import Simulation

N_SEEDS = int(sys.argv[1]) if len(sys.argv) > 1 else 10
SIM_SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
DT = 0.1


def run_one(key, seed):
    scene, agents = build(key, seed=seed)
    sim = Simulation(scene, agents, dt=DT, seed=seed)
    lat, dth, prev = [], [], None
    fsm_ticks = {}
    for _ in range(int(SIM_SECONDS / DT)):
        sim.step()
        lat.append(sim.last_plan_latency_ms)
        fsm_ticks[sim.fsm.state] = fsm_ticks.get(sim.fsm.state, 0) + 1
        th = sim.agents[sim.ego_id].state.theta
        if prev is not None:
            dth.append(abs((th - prev + 3.14159) % 6.28318 - 3.14159))
        prev = th
    m = sim.metrics
    return dict(seed=seed, progress=m["ego_progress_frac"], completed=m["completed"], collisions=m["collisions"],
                shield=m["shield_interventions"], lat_mean=statistics.mean(lat),
                lat_p95=sorted(lat)[int(len(lat) * .95)], smooth=statistics.mean(dth) if dth else 0.0,
                min_clear=m["min_clearance_ever"], fsm=fsm_ticks)


def main():
    seeds = list(range(1, N_SEEDS + 1))
    report = []
    print(f"{N_SEEDS} seeds x {len(list_scenarios())} scenarios x {SIM_SECONDS:.0f}s simulated\n")
    print(f"{'scenario':20s} {'collision-free':>15s} {'avg progress':>13s} {'latency mean/p95 ms':>21s} {'smoothness rad/tick':>20s} {'shield/run':>11s}")
    for s in list_scenarios():
        runs = [run_one(s["key"], sd) for sd in seeds]
        row = dict(scenario=s["key"], runs=len(runs),
                   collision_free_runs=sum(1 for r in runs if r["collisions"] == 0),
                   total_collision_ticks=sum(r["collisions"] for r in runs),
                   avg_progress=round(statistics.mean(r["progress"] for r in runs), 3),
                   completion_rate=round(sum(1 for r in runs if r["completed"]) / len(runs), 3),
                   lat_mean_ms=round(statistics.mean(r["lat_mean"] for r in runs), 3),
                   lat_p95_ms=round(statistics.mean(r["lat_p95"] for r in runs), 3),
                   smoothness=round(statistics.mean(r["smooth"] for r in runs), 5),
                   shield_per_run=round(statistics.mean(r["shield"] for r in runs), 1),
                   per_seed=runs)
        report.append(row)
        print(f"{s['key']:20s} {row['collision_free_runs']:>7d}/{row['runs']:<7d} {row['avg_progress']*100:>12.1f}% "
              f"{row['lat_mean_ms']:>9.2f}/{row['lat_p95_ms']:<10.2f} {row['smoothness']:>20.5f} {row['shield_per_run']:>11.1f}")
    tot = sum(r["runs"] for r in report); ok = sum(r["collision_free_runs"] for r in report)
    print(f"\nOVERALL: {ok}/{tot} runs collision-free")
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results.json")
    json.dump(report, open(out, "w"), indent=2); print("wrote", out)


if __name__ == "__main__":
    main()
