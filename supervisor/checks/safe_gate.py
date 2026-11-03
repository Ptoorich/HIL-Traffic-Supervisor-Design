"""
Safe Gate (SARTSM minimum greens).

implied Phase B green = cycle - requested Phase A green - lost time (12 s).
BOTH phases are checked, using the UNCLIPPED requested split (D2): the env
clips silently to [15, 63], so checking the clipped value would hide a
policy that asks for something unsafe.

  min(A, B) < 8.0 s  (7.0 hard minimum + 1.0 latency margin)  -> FAIL
  min(A, B) < 11.0 s (preferred minimum)                      -> WARN
"""
from __future__ import annotations

import math

from ..config import SupervisorConfig
from ..datatypes import CheckResult, JunctionSnapshot, Proposal, Status

NAME = "safe_gate"


def check(snap: JunctionSnapshot, proposal: Proposal,
          cfg: SupervisorConfig) -> CheckResult:
    t = cfg.timing
    a = proposal.requested_split
    if not math.isfinite(a):          # NaN would slip through every comparison
        return CheckResult(NAME, Status.FAIL, "NON_FINITE",
                           {"requested_split": a})
    b = t.cycle_s - a - cfg.lost_time_s
    details = {"green_a": a, "green_b": b,
               "current_split": proposal.current_split, "delta": proposal.delta,
               "reject_below": cfg.min_green_reject_s,
               "warn_below": t.pref_min_green_s}
    worst_phase, worst = ("A", a) if a <= b else ("B", b)
    details["worst_phase"] = worst_phase
    if worst < cfg.min_green_reject_s:
        return CheckResult(NAME, Status.FAIL, "GREEN_BELOW_MIN", details)
    if worst < t.pref_min_green_s:
        return CheckResult(NAME, Status.WARN, "GREEN_BELOW_PREFERRED", details)
    return CheckResult(NAME, Status.PASS, "OK", details)
