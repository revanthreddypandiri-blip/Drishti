# DRISHTI — Technical Report (short form)

**Dynamic Risk-aware Intent & Situation-aware Trajectory Handling for Indian Roads**
SIH problem: adaptive path planning for AVs in unstructured Indian road conditions.

## 1. Approach in one paragraph
Indian traffic is a negotiation, not an obstacle field. DRISHTI builds a per-tick *interaction graph*
of nearby road users, detects root events (hard brake / swerve), and propagates them as delayed,
decaying reactions ("traffic ripples", up to 3 hops). The ego plans against *predicted,
uncertainty-weighted futures* that already include those ripples, inside a continuously recomputed
*drivable corridor* (no lane assumption), governed by a five-state negotiation FSM driven by a Traffic
Stress Index, and independently verified by a simpler Safety Shield.

## 2. Pipeline (perception → prediction → planning → decision → motion)
| Stage | Module | Notes |
|---|---|---|
| Perception | `sim/perception/sensors.py` | Simulated camera/LiDAR/radar: FOV, range, occlusion, noise; adaptive sensor trust for day / low-light / fog; occlusion zones behind buses & trucks |
| Prediction | `sim/prediction/intent.py`, `sim/planning/uncertainty.py`, `behavior_profile.py` | 6-hypothesis intent belief; surprise detector; 4-cause uncertainty; behavior profiles (Stable/Cautious/Erratic/Crossing/Filtering/Oncoming/Stationary/Occluded) |
| Interaction | `sim/graph/ripple.py` | Proximity graph, multi-hop delayed/decaying propagation. Drives the *real* reactions of background agents |
| Planning | `sim/planning/planner.py`, `corridor.py` | 6 maneuver candidates scored on risk, comfort, progress, off-road, occlusion; lateral targets clamped to the live corridor |
| Decision | `sim/planning/behavior_fsm.py`, `stress.py` | Stress Index → FOLLOW/CAUTIOUS/NEGOTIATE/YIELD/EMERGENCY; Risk Budget scales the risk weight; FSM adds soft per-maneuver cost penalties |
| Verification | `sim/safety_shield/shield.py` | Independent constant-velocity, size-aware check; can veto with emergency brake |
| Vehicle motion | `sim/core/kinematics.py` | Kinematic bicycle (wheeled), point-mass (pedestrians / animals / pushcarts) |

## 3. Scenarios (all seeded, procedurally varied — nothing is a fixed layout)
Unmarked village road · unsignalled urban intersection · highway merge with slow vehicles ·
dense market · sudden cattle crossing. Each seed changes road width and edge irregularity, agent counts,
positions, speeds, trigger timing, obstacle placement and lighting (day / low light / fog).

## 4. Results (`python -m metrics.run_headless 10 25`: 10 seeds × 5 scenarios × 25 s)
| Scenario | Collision-free runs | Avg route progress in 25 s | Replan latency mean / p95 |
|---|---|---|---|
| Village road | 10/10 | 47.7% | 2.12 / 2.66 ms |
| Urban intersection | 10/10 | 47.5% | 2.06 / 2.58 ms |
| Highway merge | 10/10 | 53.9% | 1.42 / 1.59 ms |
| Dense market | 10/10 | 22.2% | 2.22 / 2.63 ms |
| Cattle crossing | 10/10 | 87.5% | 1.55 / 1.95 ms |
| **Overall** | **50/50** | | |

Additionally: 25 runs with three injected "what-if" events each → 0 collisions.
Latency is measured on the sandbox CPU for the perception→plan→verify step (Python, single thread).

## 5. Honest limitations
* **Simulation only.** Sensors are modelled (FOV, range, occlusion, noise, condition-dependent trust),
  not real drivers or point clouds. Real deployment needs sensor integration, calibration, on-vehicle
  latency validation and regulatory certification.
* **The Shield does heavy lifting in dense scenes** (54–175 override ticks per 250-tick run). The planner
  is conservative by design; reducing reliance on the Shield is the main tuning target.
* **Route progress is low in 25 s** in slow/dense scenarios (market ≈22%) — the system prioritises
  collision-free behaviour over speed. Longer runs give proportionally more progress.
* **Single road corridor.** Cross-traffic at junctions is approximated with scripted lateral triggers,
  not a full multi-road network.
* **Collision metric** is a bounding-circle proxy, not a mesh-accurate contact test.
* **What-if** is a live injection (make the nearest road user act unexpectedly), not recorded-state
  branch-and-replay.
* MATLAB / Simulink / RoadRunner were **not** used (the PS "encourages" but does not mandate them).

## 6. Bugs found and fixed during validation
Multi-seed testing exposed collisions in `highway_merge` (always with the truck). Root cause: planner and
Safety Shield treated every road user as a point with a fixed margin, ignoring size. Fixed by scaling
margins with each agent type's bounding radius; re-test: 0 collisions in 40 seeded runs.
An earlier arc-length road-projection bug (metres clamped as a 0–1 fraction) had silently zeroed
forward-progress tracking; fixed and regression-tested.
