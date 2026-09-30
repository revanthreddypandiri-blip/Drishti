# DRISHTI V5 — Team Integration & Architecture Guide

## 1. Executive Principles

1. **V5 is the Baseline**: The existing V5 architecture is the core baseline for the project. All future development must build directly upon V5.
2. **Do Not Rebuild V5**: Team members must extend existing components rather than rewriting or rebuilding V5 modules from scratch.
3. **Do Not Create Duplicate Planners**: The existing planner (`sim/planning/planner.py`) and decision pipeline serve as the primary planning engine. Do not create parallel or redundant planners.
4. **Preserve Existing Safety Shield**: The Safety Shield (`sim/safety_shield/shield.py`) is an independent safety veto mechanism and must be preserved across all updates.
5. **Preserve and Upgrade 5 Validation Scenarios**: The five core validation scenarios (`village_road`, `urban_intersection`, `highway_merge`, `market_area`, `cattle_crossing`) must remain fully functional and upgraded as perception and road network features evolve.
6. **No Fabricated Tool Integrations**: Do not fabricate or claim MATLAB, Simulink, or RoadRunner integration unless an actual, working implementation exists in the repository.
7. **Raw Datasets Must Not Be Committed**: Large sensor logs, raw camera/LiDAR datasets, and heavy model checkpoints must never be committed to the repository (enforced via `.gitignore`).
8. **Main Branch Stability**: The `main` branch must always remain clean, stable, and runnable. All experimental development must occur on dedicated feature branches.

---

## 2. End-to-End System Architecture

The target pipeline architecture flows strictly as follows:

```
Camera / Sensor
        ↓
Indian Perception
        ↓
PerceptionResult
        ↓
Temporal Prediction
        ↓
Risk + Interaction Analysis
        ↓
Counterfactual Action Evaluation
        ↓
Adaptive Planning
        ↓
Safety Shield
        ↓
Vehicle Motion
        ↓
Simulation / Road Network
        ↓
Next Frame
```

---

## 3. Team Ownership & Responsibilities

### Member 1: Perception & Datasets
* **Responsibilities**:
  * Indian Perception models and sensor integration (Camera / LiDAR / Radar).
  * Dataset curation, annotation pipelines, and model training.
  * Exposing standardized `PerceptionResult` structures for downstream modules.
* **Core Output Interface**: `PerceptionResult` (containing bounding boxes, class labels, confidence scores, distance/velocity estimates, and sensor trust metrics).

### Member 2: Intelligence & Planning Extensions
* **Responsibilities**:
  * Temporal Prediction (intent estimation, multi-hypothesis prediction).
  * Interaction & Risk Analysis (proximity-weighted ripple graph, risk budgeting).
  * Counterfactual Action Evaluation & What-If scenario probing.
  * Adaptive Planning extension (extending V5 intelligence and cost functions).
* **Core Output Interfaces**: `PredictionResult`, `RiskResult`, `PlanningDecision`.

### Member 3: Simulation, Environment & UI
* **Responsibilities**:
  * V5 Core Simulation loop (`sim/simulation.py`) and state management.
  * Road Network expansion (`sim/network.py` & core geometry).
  * Scenario definitions (`sim/scenarios/`) and vehicle kinematics.
  * Web UI frontend (`frontend/`), visualization layers, and HUD.
  * Performance metrics (`metrics/run_headless.py`).
* **Core Output Interface**: `EnvironmentState` (road layout, static obstacles, environment conditions, ground-truth world state).

---

## 4. Interface & Communication Protocols

Modules must communicate strictly via stable, decoupled interfaces:

* **`EnvironmentState`** (Provided by Member 3):
  * Describes current scene geometry, road boundaries, surface conditions, lighting, and ground-truth agent states for the simulation tick.
* **`PerceptionResult`** (Exposed by Member 1):
  * Structured perception outputs derived from simulated or real sensor inputs. Includes detected obstacles, confidence, occlusion zones, and weather/lighting sensor trust scores.
* **`PredictionResult` / `RiskResult` / `PlanningDecision`** (Exposed by Member 2):
  * Probabilistic agent intent distributions, ripple graph disturbances, stress index calculations, trajectory candidate evaluations, and selected ego maneuvers.
* **`Safety Shield Veto`** (Preserved in V5):
  * Independent safety layer evaluating selected `PlanningDecision` against worst-case constant-velocity projections, overriding with emergency braking if necessary.

---

## 5. Road Network Evolution Roadmap

The road network (`sim/network.py` and `sim/core/geometry.py`) must be incrementally extended to support rich Indian road features while keeping V5 backwards compatible:

* 4-Way Intersections & Unsignalled Crossings
* T-Junctions & Angled Side Roads
* Merging Slip Roads & On-Ramps
* Dedicated & Informal Turning Lanes
* Dirt & Paved Shoulders
* Physical & Informal Medians
* High-Conflict Zones (market clutter, bus stops, pedestrian crossing hotspots, work zones)

---

## 6. Git Branching & Workflow Strategy

All development must follow a feature-branch workflow to maintain a working baseline at all times on `main`.

### Active Feature Branches:
* `feature/perception`: Owned by Member 1 (Perception models, dataset connectors, `PerceptionResult` interface).
* `feature/intelligence`: Owned by Member 2 (Prediction, interaction ripple extensions, risk budgeting, planner scoring upgrades).
* `feature/simulation`: Owned by Member 3 (Road network upgrades, scenario additions, UI visualization enhancements, metrics).

### Rules for Pull Requests:
1. Every PR to `main` must pass headless verification (`python -m metrics.run_headless 10 25`).
2. No breaking changes to existing data structures (`PerceptionResult`, `EnvironmentState`, `PlanningDecision`).
3. No binary dataset files or model checkpoints (> 5 MB) may be committed.

---

## 7. Integration Checkpoints

Checkpoint 1 — Branch setup:
All members confirm their branch, baseline execution, and intended files.

Checkpoint 2 — First working modules:
Member 1 demonstrates PerceptionResult.
Member 2 demonstrates Prediction/Risk/PlanningDecision.
Member 3 demonstrates EnvironmentState and road network.

Checkpoint 3 — Pull Request readiness:
Each branch must pass its tests, contain no raw datasets or large weights, and document changes.

Checkpoint 4 — Integration:
Merge one branch at a time and run the complete five-scenario regression after every merge.

Integration order:
1. feature/simulation
2. feature/perception
3. feature/intelligence
