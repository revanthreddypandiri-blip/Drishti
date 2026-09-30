# DRISHTI
**Dynamic Risk-aware Intent & Situation-aware Trajectory Handling for Indian Roads**

Smart India Hackathon prototype: an adaptive path-planning simulation for autonomous vehicles in unstructured, mixed-traffic Indian road conditions. Built from scratch in Python (simulation-first — not aiming at production AV software).

---

## The Core Idea: Traffic Ripple Prediction
Road users on Indian roads aren't independent obstacles — they react to each other. Drishti models this explicitly:

```
Pothole → Auto swerves → Motorcycle reacts → Pedestrian becomes uncertain
        → Ego's safe corridor narrows → Ego slows down before the conflict develops
```

---

## Current V5 Simulation Architecture & Pipeline

1. **Road Geometry & Network** — Unmarked, variable-width drivable corridor with no hard lane assumption (`sim/core/geometry.py`, `sim/network.py`).
2. **Perception** — Simulated camera + LiDAR + radar sensor suite with field-of-view, range limits, occlusion zones behind heavy vehicles, and environmental noise/lighting adaptation (`sim/perception/sensors.py`).
3. **Intent Estimation** — Multi-hypothesis probabilistic intent estimation per perceived agent (keep / brake / swerve / stop / cross) with surprise detection and 4-cause uncertainty modeling (`sim/prediction/intent.py`, `sim/planning/uncertainty.py`).
4. **Dynamic Interaction Graph & Ripple Propagation** — Proximity-weighted graph rebuilt every tick (`sim/graph/ripple.py`). Detected root events (hard brake / swerve) propagate as delayed, decaying disturbances through the graph (up to 3 hops). This engine drives non-ego agents' reactive behavior, while the planner separately predicts ripple risk from perceived state (`sim/planning/planner.py`).
5. **Adaptive Planning & Decision FSM** — Maneuver candidate scoring (keep, decelerate, hard brake, nudge left/right, creep) evaluated against predicted collision risk, comfort, and progress. Governed by a Traffic Stress Index and 5-state negotiation FSM (`sim/planning/planner.py`, `sim/planning/behavior_fsm.py`).
6. **Safety Shield** — An independent, deliberately simpler verification layer using worst-case constant-velocity extrapolation (no intent reasoning); can veto the planner and substitute an emergency-brake fallback (`sim/safety_shield/shield.py`).
7. **Vehicle Motion** — Kinematic bicycle model for wheeled agents; point-mass model for soft agents (pedestrians, animals, pushcarts) (`sim/core/kinematics.py`).

---

## The 5 Required Validation Scenarios
Defined in `sim/scenarios/`:
* `village_road`: Unmarked narrow road with animals, informal crossings, and speed bumps.
* `urban_intersection`: Unsignalled 4-way conflict zone with multi-directional crossing traffic.
* `highway_merge`: High-speed corridor with slow merging vehicles and heavy trucks.
* `market_area`: Dense mixed-traffic zone with pushcarts, parked cars, roadside shops, and bus stops.
* `cattle_crossing`: Sudden animal entry into ego corridor triggering ripple deceleration.

Each scenario uses procedural seed generation for varied road widths, obstacle placements, agent counts, lighting (day / low-light / fog), and trigger timings.

---

## Development & Feature Branches

The DRISHTI project follows a structured modular architecture:
* **`main`**: Baseline production-ready V5 code. Must remain stable and runnable at all times.
* **`feature/perception`**: Member 1 — Indian Perception integration, dataset pipelines, and `PerceptionResult` interface exposure.
* **`feature/intelligence`**: Member 2 — Temporal prediction extensions, counterfactual evaluation, risk budgeting, and planner upgrades.
* **`feature/simulation`**: Member 3 — Road network expansion (junctions, slip roads, conflict zones), scenario upgrades, UI HUD, and metrics.

For details on team ownership, interface specifications, and component boundaries, see [`TEAM_INTEGRATION.md`](file:///f:/drishti/TEAM_INTEGRATION.md).

---

## Future Indian Dataset & Perception Integration Roadmap
The V5 architecture is designed to integrate real-world perception models and Indian driving datasets (such as IDD / Indian Driving Dataset) via standard data contracts:
* **Current State (V5 Baseline)**: Perception is simulated via realistic FOV, occlusion, and noise models (`sim/perception/sensors.py`).
* **Planned Integration**: Real camera/LiDAR vision models will plug into the pipeline by outputting standardized `PerceptionResult` objects. The downstream prediction (`sim/prediction/intent.py`), ripple graph (`sim/graph/ripple.py`), planner (`sim/planning/planner.py`), and safety shield (`sim/safety_shield/shield.py`) will operate seamlessly with real-world perception feeds.

*Note: MATLAB, Simulink, and RoadRunner are not currently integrated into the codebase; DRISHTI is built natively in Python.*

---

## Running DRISHTI V5

### 1. Web Application & Interactive Simulation
```bash
# Linux/macOS:
./run.sh

# Windows:
run.bat

# Manual Python launch:
pip install -r requirements.txt
uvicorn backend.main:app --port 8000

# Docker launch:
docker build -t drishti .
docker run -p 8000:8000 drishti
```
Open **`http://localhost:8000`** in your browser.

**Pages Available**:
* **`/`** — Home (animated project overview and architecture)
* **`/explain`** — Interactive technical breakdown of ripple propagation and FSM
* **`/math`** — Mathematical formulas for risk budgeting, ripple decay, and maneuver scoring
* **`/simulation`** — Live interactive simulation workbench with Driver HUD, what-if triggers, layer toggles, and scenario controls

### 2. Headless Metrics & Batch Verification
```bash
python -m metrics.run_headless 10 25    # 10 seeds x 5 scenarios x 25 s -> metrics/results.json
```

---

## Known Simplifications & Limitations
* **Simulation-First Prototype**: Sensors model FOV, range, occlusion, and lighting-dependent noise, but do not process raw point clouds or camera pixels directly.
* **Shield Vetoes**: The Safety Shield intervenes in high-density conflict scenes to guarantee collision-free performance when planner candidates exceed safety thresholds.
* **Single Corridor Geometry Baseline**: Cross-traffic in V5 is modeled on single road corridors via lateral-offset triggers; multi-road graph expansions are planned in `feature/simulation`.
* **Bounding Circle Proxy**: Collision checking uses agent-type specific bounding radii.

---

## Technical Documentation & References
* [`TECHNICAL_REPORT.md`](file:///f:/drishti/docs/TECHNICAL_REPORT.md) — In-depth architectural report and benchmark findings.
* [`cost_comparison.md`](file:///f:/drishti/docs/cost_comparison.md) — Retrofit ADAS deployment economics vs full LiDAR stack.
* [`TEAM_INTEGRATION.md`](file:///f:/drishti/TEAM_INTEGRATION.md) — Team architecture, module interfaces, and git guidelines.
