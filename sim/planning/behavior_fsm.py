"""Negotiation-aware behavior state machine.

Real Indian traffic isn't "obstacle -> STOP" -- a human driver negotiates:
slows slightly for a side-approaching auto rather than slamming the brakes,
holds a buffer near a pedestrian while still moving, nudges around a
two-wheeler filtering through. We make this explicit as a discrete state
machine (the Stateflow-style decision logic the PS asks for), driven by the
Traffic Stress Index, so the mapping from "how chaotic is it right now" to
"how should the vehicle behave" is an inspectable rule table, not a black box.
"""
from __future__ import annotations
from dataclasses import dataclass

STATES = ["FOLLOW", "CAUTIOUS", "NEGOTIATE", "YIELD", "EMERGENCY"]

# stress (0-100) -> state, matching the 5 stress bands 1:1
STRESS_BANDS = [
    (20.0, "FOLLOW"),
    (40.0, "CAUTIOUS"),
    (60.0, "NEGOTIATE"),
    (80.0, "YIELD"),
    (101.0, "EMERGENCY"),
]

STATE_DESCRIPTIONS = {
    "FOLLOW":    "Normal trajectory — corridor is clear, maintain course.",
    "CAUTIOUS":  "Increased clearance — widen buffers around nearby agents.",
    "NEGOTIATE": "Reduced speed, actively negotiating shared space (yield a little, "
                 "nudge around, hold a gap) rather than stopping outright.",
    "YIELD":     "Conservative replanning — favor braking/creeping maneuvers, "
                 "give way rather than contest the corridor.",
    "EMERGENCY": "Yield/stop behavior — Safety Shield is the final authority here.",
}

# maneuver-name soft cost bias per state (0 = preferred/no penalty). This is
# what makes the state machine *causal* without causing gridlock: it nudges
# the planner's own cost function rather than hard-excluding options, so a
# genuinely risk-free "keep" can still win if the state's caution turns out
# to be unwarranted this exact tick, but every non-preferred maneuver pays a
# real, visible tax while the state holds.
STATE_MANEUVER_PENALTY = {
    "FOLLOW":    {},
    "CAUTIOUS":  {"hard_brake": 0.3},
    "NEGOTIATE": {"keep": 0.5, "hard_brake": 0.4},
    "YIELD":     {"keep": 1.1, "nudge_left": 0.5, "nudge_right": 0.5},
    "EMERGENCY": {"keep": 1.8, "nudge_left": 1.0, "nudge_right": 1.0, "decelerate": 0.4},
}


@dataclass
class FSMState:
    state: str = "FOLLOW"
    ticks_in_state: int = 0

    def step(self, stress: float) -> "FSMState":
        target = "FOLLOW"
        for threshold, name in STRESS_BANDS:
            if stress < threshold:
                target = name
                break
        else:
            target = "EMERGENCY"

        # hysteresis: don't drop out of a more cautious state after a single
        # low-stress tick -- require a short settle period, matching how a
        # real driver eases off caution rather than snapping back to normal.
        order = STATES.index
        if order(target) < order(self.state) and self.ticks_in_state < 2:
            target = self.state

        if target == self.state:
            self.ticks_in_state += 1
        else:
            self.state = target
            self.ticks_in_state = 0
        return self

    def maneuver_penalty(self, name: str) -> float:
        return STATE_MANEUVER_PENALTY.get(self.state, {}).get(name, 0.0)

    def description(self):
        return STATE_DESCRIPTIONS.get(self.state, "")
