"""
Traffic Classifier.

Only GRIDLOCK vetoes (FAIL). HEAVY is a WARN. FREE / LIGHT only label the
state (PASS, logged).

Gridlock, per non-excluded lane (L9), ALL of:
  * mean speed  < gridlock_speed_mps
  * occupancy   >= gridlock_occupancy_frac_of_jam x lane jam occupancy
  * halting     >= gridlock_fill_frac x capacity
A junction is gridlocked when >= gridlock_lane_fraction of its non-excluded
lanes qualify (one full lane at the end of red is normal).

Why "of jam" occupancy: SUMO occupancy counts vehicle LENGTH only (5 m of a
7.5 m footprint), so a fully jammed lane reads about 0.64-0.67. A raw
">= 90 %" test could never fire. Lane jam occupancy =
capacity x vehicle_length / lane_length (capped at 1).

No jammed lanes: FREE if mean queue fill <= free_max, otherwise LIGHT.
One or two jammed lanes: HEAVY. Three of five: GRIDLOCK.
"""
from __future__ import annotations

import math

from ..config import SupervisorConfig
from ..datatypes import CheckResult, JunctionSnapshot, Proposal, Status

NAME = "traffic_classifier"


def lane_is_gridlocked(r, cc) -> bool:
    jam_occ = min(1.0, r.capacity * cc.vehicle_length_m / r.length) if r.length > 0 else 1.0
    return (r.mean_speed < cc.gridlock_speed_mps
            and r.occupancy >= cc.gridlock_occupancy_frac_of_jam * jam_occ
            and r.halting >= cc.gridlock_fill_frac * r.capacity
            and r.halting > 0)


def mean_fill(snap: JunctionSnapshot) -> float:
    active = [r for r in snap.lanes if not r.excluded]
    if not active:
        return float("nan")
    return sum(r.halting / r.capacity for r in active) / len(active)


def check(snap: JunctionSnapshot, proposal: Proposal,
          cfg: SupervisorConfig) -> CheckResult:
    cc = cfg.classifier
    active = [r for r in snap.lanes if not r.excluded]
    n_active = len(active)
    need = max(1, math.ceil(cc.gridlock_lane_fraction * n_active - 1e-9))
    if n_active == 0:
        return CheckResult(NAME, Status.FAIL, "NO_ACTIVE_LANES", {"label": "UNKNOWN"})
    for r in active:
        values = (r.halting, r.count, r.occupancy, r.mean_speed, r.length, r.capacity)
        if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
                or r.capacity <= 0 or r.length <= 0 or r.count < 0 or r.halting < 0
                or r.halting > r.count or not 0 <= r.occupancy <= 1 or r.mean_speed < 0):
            return CheckResult(NAME, Status.FAIL, "INVALID_READING", {"label": "UNKNOWN", "lane": r.lane_id})
    gridlocked = [r.lane_id for r in active if lane_is_gridlocked(r, cc)]
    fill = mean_fill(snap)
    details = {"mean_fill": fill, "gridlocked_lanes": gridlocked,
               "lanes_needed": need, "n_active_lanes": n_active}

    if len(gridlocked) >= need:
        return CheckResult(NAME, Status.FAIL, "GRIDLOCK",
                           {**details, "label": "GRIDLOCK"})

    if gridlocked:
        return CheckResult(NAME, Status.WARN, "HEAVY", {**details, "label": "HEAVY"})
    label = "FREE" if fill <= cc.free_max else "LIGHT"
    return CheckResult(NAME, Status.PASS, label, {**details, "label": label})
