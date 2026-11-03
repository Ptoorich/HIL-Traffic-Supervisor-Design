"""
Input Validator: is the sensor data physically plausible?

Untrusted sensors mean an untrusted ASTA, so any hard violation is a FAIL.
Soft limits (just outside the normal envelope) are a WARN and never veto.
Pure function of (snapshot, proposal, cfg).
"""
from __future__ import annotations

import math

from ..config import SupervisorConfig
from ..datatypes import CheckResult, JunctionSnapshot, Proposal, Status

NAME = "input_validator"
_MAX_LISTED = 10   # keep log lines bounded when everything is broken


def check(snap: JunctionSnapshot, proposal: Proposal,
          cfg: SupervisorConfig) -> CheckResult:
    v = cfg.validator
    hard: list[dict] = []
    soft: list[dict] = []

    expected = {l.lane_id for l in cfg.lanes_of(snap.agent)}
    seen = [r.lane_id for r in snap.lanes]
    for lane_id in sorted(expected - set(seen)):
        hard.append({"lane": lane_id, "code": "LANE_MISSING"})
    for lane_id in sorted(set(seen) - expected):
        hard.append({"lane": lane_id, "code": "LANE_UNEXPECTED"})
    if len(seen) != len(set(seen)):
        hard.append({"lane": "*", "code": "LANE_DUPLICATED"})
    if not math.isfinite(snap.sim_time) or snap.sim_time < 0:
        hard.append({"lane": "*", "code": "BAD_SIM_TIME", "value": snap.sim_time})

    for r in snap.lanes:
        nums = (r.occupancy, r.mean_speed, r.length, r.max_speed)
        if not all(math.isfinite(x) for x in nums):
            hard.append({"lane": r.lane_id, "code": "NON_FINITE"})
            continue
        if r.halting < 0 or r.count < 0:
            hard.append({"lane": r.lane_id, "code": "NEGATIVE_COUNT",
                         "halting": r.halting, "count": r.count})
        if r.halting > r.count:
            hard.append({"lane": r.lane_id, "code": "HALTING_GT_COUNT",
                         "halting": r.halting, "count": r.count})
        if not (0.0 <= r.occupancy <= 1.0):
            hard.append({"lane": r.lane_id, "code": "OCCUPANCY_RANGE",
                         "occupancy": r.occupancy})
        if r.count > r.capacity + v.count_slack_vehicles:
            hard.append({"lane": r.lane_id, "code": "COUNT_GT_CAPACITY",
                         "count": r.count, "capacity": r.capacity})
        elif r.count > r.capacity:
            soft.append({"lane": r.lane_id, "code": "COUNT_AT_SLACK",
                         "count": r.count, "capacity": r.capacity})
        if r.mean_speed < 0:
            hard.append({"lane": r.lane_id, "code": "NEGATIVE_SPEED",
                         "speed": r.mean_speed})
        elif r.mean_speed > r.max_speed * v.speed_fail_factor:
            hard.append({"lane": r.lane_id, "code": "SPEED_OVER_LIMIT",
                         "speed": r.mean_speed, "limit": r.max_speed})
        elif r.mean_speed > r.max_speed * v.speed_warn_factor:
            soft.append({"lane": r.lane_id, "code": "SPEED_ABOVE_LIMIT",
                         "speed": r.mean_speed, "limit": r.max_speed})

        # Cross-combinations. SUMO reports the speed LIMIT for an empty lane,
        # so speed is only meaningful when count > 0.
        if r.count > 0:
            if (r.occupancy > v.high_speed_max_occupancy
                    and r.mean_speed > v.high_speed_mps):
                hard.append({"lane": r.lane_id, "code": "SPEED_OCCUPANCY_IMPOSSIBLE",
                             "speed": r.mean_speed, "occupancy": r.occupancy})
            if (r.halting == r.count
                    and r.mean_speed > v.all_halting_max_speed_mps):
                hard.append({"lane": r.lane_id, "code": "HALTING_SPEED_MISMATCH",
                             "speed": r.mean_speed, "halting": r.halting})

    if hard:
        status, reason = Status.FAIL, "INVALID_INPUT"
    elif soft:
        status, reason = Status.WARN, "INPUT_NEAR_LIMIT"
    else:
        status, reason = Status.PASS, "OK"
    return CheckResult(
        check=NAME, status=status, reason=reason,
        details={"violations": hard[:_MAX_LISTED], "n_violations": len(hard),
                 "soft": soft[:_MAX_LISTED], "n_soft": len(soft)},
    )
