# Drishti vs. a full LiDAR-based AV stack — cost & deployability comparison

*Figures below are approximate, sourced from public 2025–2026 pricing
(cited), and meant to establish order-of-magnitude difference for a pitch —
**verify current vendor quotes before citing exact numbers to judges**,
prices move fast in this space.*

## Why this comparison matters
The PS's "expected solution" section points teams toward a full sensor-fusion
stack (camera + LiDAR + radar + Automated Driving/Deep Learning Toolbox).
That's the right reference architecture for full autonomy — but it prices
most Indian fleet operators (bus depots, auto-rickshaw unions, last-mile
delivery fleets) out of adopting it this decade. Drishti is deliberately
scoped as a **retrofit safety layer**, not a full AV stack, so its hardware
budget looks completely different.

## Per-vehicle hardware, prototype/dev-kit scale

| Component | Full LiDAR-based AV stack | Drishti retrofit kit |
|---|---|---|
| Ranging sensor | 1–3× automotive-grade LiDAR units. Dev-kit-scale units have historically run **$1,500–$4,000+ each** at low volume; even at OEM bulk scale, average selling price was still ≈RMB 2,192 (**~$300**) per unit in 2025 [1][2] | None — no LiDAR required |
| Compute | Data-center-class or Jetson **AGX Orin**-class module for real-time deep-learning perception + prediction, ≈**₹2.1 lakh (~$2,500)** for a dev kit in India [3] | Jetson **Orin Nano Super** Developer Kit, ≈**₹21,000–33,000 (~$249–400)** in India [3][4] |
| Cameras | Automotive-grade camera array, calibrated + synced | 1–2 dash-cam-grade cameras (commodity, tens of dollars each) |
| Short-range sensing | Radar array | Commodity ultrasonic/short-range radar modules (commodity, single-digit-to-low-tens of dollars each) |
| HD maps / localization infra | Often required | Not required — Drishti reasons from live perception + road geometry, not pre-mapped infrastructure |
| **Approximate per-vehicle total** | **~$6,000–$15,000+** (dev-kit scale; production/OEM volumes bring this down but still dominated by LiDAR + compute) | **~$300–$650** |

**Headline: Drishti's stack costs roughly a tenth to a twentieth of a
LiDAR-based dev stack**, because the causal ripple-graph approach is
rule-based/probabilistic rather than a GPU-hungry deep perception+prediction
pipeline — it doesn't need the compute tier that justifies an AGX-class
module, let alone LiDAR.

## Why this is the *right* trade-off, not a shortcut
- **Full autonomy needs LiDAR-grade ranging precision; a safety-advisory
  layer that anticipates and warns/intervenes early does not** — it only
  needs to be right enough, early enough, which is a lower bar than
  centimeter-accurate 3D reconstruction.
- **Deployability timeline**: full AV stacks face years of regulatory
  approval in India; a retrofit ADAS kit for existing commercial vehicles
  faces a far shorter path to real pilots (fleet operators can adopt
  voluntarily, the way dash-cams and telematics units already got adopted).
- **This is also why the ripple/intent reasoning is rule-based, not a
  trained deep net** — it runs entirely on the Orin Nano's compute budget,
  with no training data collection pipeline, no retraining/certification
  cycle when the model updates, and a fully auditable decision trail
  (see the explainability angle in the main pitch).

## Sources
[1] RoboSense Technology, Q2 2025 ADAS LiDAR average selling price
    (≈RMB 2,192.5) — iTiger news, Oct 2025.
[2] The Robot Report, on Ouster/Sense Photonics: historical automotive LiDAR
    unit pricing context ("a single LiDAR unit currently sells for $2000+").
[3] ThinkRobotics (India), Jetson Orin Nano Super Developer Kit and Jetson
    AGX Orin Developer Kit India retail listings, accessed 2026.
[4] NVIDIA official Jetson Orin Nano Super Developer Kit price ($249).
