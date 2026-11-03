"""
Immutable data types shared by every supervisor module.

Everything here is frozen: a snapshot is taken once in the main thread and
then handed read-only to the Fast Gate worker threads (L3).
(Named datatypes.py, not types.py, so it can never shadow Python's `types`.)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class Status(str, Enum):
    PASS = "PASS"
    WARN = "WARN"
    FAIL = "FAIL"


class Overall(str, Enum):
    APPROVE = "APPROVE"
    APPROVE_WARN = "APPROVE_WARN"
    VETO = "VETO"


@dataclass(frozen=True)
class LaneReading:
    lane_id: str
    halting: int            # getLastStepHaltingNumber
    count: int              # getLastStepVehicleNumber
    occupancy: float        # FRACTION 0..1 (normalised in snapshot.py)
    mean_speed: float       # m/s; SUMO reports the speed LIMIT for an empty lane
    length: float           # m
    max_speed: float        # m/s, lane speed limit
    capacity: int           # from config (empirical, or theoretical if excluded)
    excluded: bool          # excluded from Gridlock/classifier (L9)


@dataclass(frozen=True)
class JunctionSnapshot:
    agent: str
    tl_id: str
    sim_time: float
    lanes: tuple                    # tuple[LaneReading, ...]
    live_state: str                 # traci getRedYellowGreenState
    commanded_phase_strings: tuple  # (gA, yA, gB, yB) cached by the env, () if unknown
    current_split: float            # env.phase_splits[agent] BEFORE the inner controller acted


@dataclass(frozen=True)
class Proposal:
    agent: str
    current_split: float
    delta: int                      # -5 / 0 / +5 (seconds on phase A)
    requested_split: float          # current + delta, UNCLIPPED (D2 / section 8)
    source: str                     # e.g. "asta" or "contingency"

    @property
    def is_noop(self) -> bool:
        return self.delta == 0


@dataclass(frozen=True)
class CheckResult:
    check: str
    status: Status
    reason: str                     # short machine-readable code, e.g. "GRIDLOCK"
    details: dict = field(default_factory=dict)
    elapsed_ms: float = 0.0


@dataclass(frozen=True)
class GateResult:
    agent: str
    checks: tuple                   # tuple[CheckResult, ...]
    overall: Overall
    triggering: tuple               # names of FAIL checks
    warnings: tuple                 # names of WARN checks
    classifier_label: str           # informational: FREE / LIGHT / HEAVY / GRIDLOCK / UNKNOWN
    snapshot_ms: float
    decision_ms: float

    def to_log_dict(self) -> dict[str, Any]:
        return {
            "agent": self.agent,
            "overall": self.overall.value,
            "triggering": list(self.triggering),
            "warnings": list(self.warnings),
            "classifier_label": self.classifier_label,
            "snapshot_ms": round(self.snapshot_ms, 3),
            "decision_ms": round(self.decision_ms, 3),
            "checks": [
                {"check": c.check, "status": c.status.value, "reason": c.reason,
                 "details": c.details, "elapsed_ms": round(c.elapsed_ms, 3)}
                for c in self.checks
            ],
        }
