"""Experimental, per-junction recovery bookkeeping; no TraCI or side effects.

All values are provisional simulation parameters, not field-safety limits.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math


def valid_pending(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) and value >= 0 and float(value).is_integer())


@dataclass
class Recovery:
    """Keep evidence since the most recent uninterrupted contingency entry."""
    is_sarl: bool = False
    min_dwell: int = 5
    required_clean: int = 3
    selection_span: int = 10
    required_selections: int = 3
    low_pending_limit: int = 100
    low_pending_cycles: int = 3
    active: bool = False
    causes: set[str] = field(default_factory=set)
    entry_time: float | None = None
    dwell: int = 0
    clean: int = 0
    low_pending: int = 0
    transition_completed: bool = False
    _selections: deque = field(default_factory=deque, repr=False)

    def __post_init__(self):
        self._selections = deque(maxlen=self.selection_span)

    def enter(self, cause, at_time, *, restart=False):
        if not self.active or restart:
            self.active = True
            self.causes = set() if not restart else set(self.causes)
            self.entry_time = at_time
            self.dwell = 0
            self.invalidate()
        self.causes.add(cause)

    def invalidate(self):
        self.clean = 0
        self.low_pending = 0
        self._selections.clear()
        self.transition_completed = False

    def finish_fixed_cycle(self, *, complete, gate_ok, measurements_ok,
                           pending_end, selected):
        if not self.active:
            raise RuntimeError('Cannot record a fixed cycle without an active latch')
        if complete:
            self.dwell += 1
        if (not complete or not gate_ok or not measurements_ok
                or not valid_pending(pending_end)):
            self.invalidate()
            return
        self.clean += 1
        self._selections.append(bool(selected))
        self.low_pending = self.low_pending + 1 if pending_end < self.low_pending_limit else 0

    def ready(self):
        return (self.active and self.dwell >= self.min_dwell
                and self.clean >= self.required_clean
                and (not self.is_sarl or sum(self._selections) >= self.required_selections)
                and ('slow_monitor' not in self.causes
                     or self.low_pending >= self.low_pending_cycles))

    def finish_transition(self, *, complete, gate_ok, measurements_ok,
                          pending_end, selected):
        self.finish_fixed_cycle(complete=complete, gate_ok=gate_ok,
                                measurements_ok=measurements_ok,
                                pending_end=pending_end, selected=selected)
        self.transition_completed = self.ready()

    def leave(self):
        self.active = False
        self.transition_completed = False

    def describe(self):
        return {'active': self.active, 'causes': sorted(self.causes),
                'entry_time': self.entry_time, 'dwell_cycles': self.dwell,
                'clean_cycles': self.clean, 'low_pending_cycles': self.low_pending,
                'selected_in_recent_cycles': sum(self._selections),
                'recent_cycles_observed': len(self._selections),
                'transition_completed': self.transition_completed, 'ready': self.ready()}
