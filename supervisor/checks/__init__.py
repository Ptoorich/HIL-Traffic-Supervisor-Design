"""
The four Fast Gate checks. Each is a pure function

    check(snapshot, proposal, cfg[, matrix]) -> CheckResult

`build_checks` binds the config (and the conflict matrix) so the runner sees
one uniform callable per check: fn(snapshot, proposal) -> CheckResult.
"""
from __future__ import annotations

from collections import namedtuple
from functools import partial

from ..config import SupervisorConfig
from ..conflicts import ConflictMatrix
from . import conflict_monitor, input_validator, safe_gate, traffic_classifier

NamedCheck = namedtuple("NamedCheck", "name fn")


def build_checks(cfg: SupervisorConfig, matrix: ConflictMatrix) -> tuple:
    return (
        NamedCheck(input_validator.NAME, partial(input_validator.check, cfg=cfg)),
        NamedCheck(traffic_classifier.NAME, partial(traffic_classifier.check, cfg=cfg)),
        NamedCheck(safe_gate.NAME, partial(safe_gate.check, cfg=cfg)),
        NamedCheck(conflict_monitor.NAME,
                   partial(conflict_monitor.check, cfg=cfg, matrix=matrix)),
    )
