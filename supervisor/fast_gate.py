"""
Fast Gate runner: all checks, all junctions, in parallel, against a deadline.

Fail-safe rules (L3 + Conflict Resolver contract):
  * a check that has not finished by the deadline   -> FAIL  (TIMEOUT)
  * a check that raises                              -> FAIL  (ERROR)
  * a check that returns something other than a
    CheckResult for its own name                     -> FAIL  (MALFORMED)
  * a junction with no snapshot or no proposal       -> FAIL  (MISSING_INPUT)
  * anything the runner itself cannot do             -> FAIL  (ERROR)
evaluate() never raises; the worst case is a VETO.

Threads and the GIL: the checks are microsecond-scale pure Python, so threads
give NO speed-up. They give fault isolation and a real, enforceable deadline.
A hung check thread cannot be killed; it is counted in `stuck_count`, logged
loudly, and the pool has spare workers. If the pool is ever exhausted, new
checks time out and every cycle vetoes, which is the safe outcome.

The gate is memoryless (3.1): no state is kept between calls except the list
of stuck threads, which is diagnostic only.
"""
from __future__ import annotations

import logging
import time
from concurrent.futures import ThreadPoolExecutor, wait
from dataclasses import replace

from .checks import build_checks
from .config import SupervisorConfig
from .conflicts import ConflictMatrix
from .datatypes import (CheckResult, GateResult, JunctionSnapshot, Overall,
                        Proposal, Status)

log = logging.getLogger("supervisor.fast_gate")

_SPARE_WORKERS = 4


def _fail(name: str, reason: str, **details) -> CheckResult:
    return CheckResult(name, Status.FAIL, reason, details)


def _run_one(fn, name: str, snap: JunctionSnapshot, prop: Proposal,
             clock) -> CheckResult:
    t = clock()
    res = fn(snap, prop)
    ms = (clock() - t) * 1000.0
    if not isinstance(res, CheckResult) or res.check != name:
        return replace(_fail(name, "MALFORMED", got=repr(res)[:200]), elapsed_ms=ms)
    return replace(res, elapsed_ms=ms)


class FastGate:
    def __init__(self, cfg: SupervisorConfig, matrix: ConflictMatrix,
                 checks: tuple | None = None, clock=time.perf_counter):
        self.cfg = cfg
        self.checks = tuple(checks) if checks is not None else build_checks(cfg, matrix)
        self._clock = clock
        n = len(self.checks) * len(cfg.agents) + _SPARE_WORKERS
        self._pool = ThreadPoolExecutor(max_workers=n, thread_name_prefix="fastgate")
        self._stuck: set = set()

    @property
    def stuck_count(self) -> int:
        self._stuck = {f for f in self._stuck if not f.done()}
        return len(self._stuck)

    def close(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ── main entry point ───────────────────────────────────────────────
    def evaluate(self, snapshots: dict, proposals: dict,
                 deadline_s: float | None = None,
                 snapshot_ms: float = 0.0) -> dict:
        """
        snapshots / proposals: {agent: JunctionSnapshot / Proposal}.
        deadline_s: time allowed for the checks themselves (the caller
        subtracts the snapshot time from the stage deadline). Default is the
        whole fast_gate_stage_s.
        Returns {agent: GateResult} for every configured agent.
        """
        clock = self._clock
        t0 = clock()
        budget = self.cfg.timing.fast_gate_stage_s if deadline_s is None else deadline_s
        deadline = t0 + max(0.0, budget)
        results: dict[tuple, CheckResult] = {}
        futures: dict = {}

        for agent in self.cfg.agents:
            snap, prop = snapshots.get(agent), proposals.get(agent)
            if snap is None or prop is None:
                for c in self.checks:
                    results[(agent, c.name)] = _fail(
                        c.name, "MISSING_INPUT",
                        snapshot=snap is not None, proposal=prop is not None)
                continue
            for c in self.checks:
                try:
                    fut = self._pool.submit(_run_one, c.fn, c.name, snap, prop, clock)
                except Exception as e:           # e.g. pool already shut down
                    results[(agent, c.name)] = _fail(c.name, "ERROR", error=repr(e))
                    continue
                futures[fut] = (agent, c.name)

        done, not_done = wait(list(futures), timeout=max(0.0, deadline - clock()))
        for fut in done:
            key = futures[fut]
            exc = fut.exception()
            results[key] = (_fail(key[1], "ERROR", error=repr(exc)) if exc
                            else fut.result())
        for fut in not_done:
            key = futures[fut]
            results[key] = _fail(key[1], "TIMEOUT", budget_s=budget)
            if not fut.cancel():                 # already running: cannot be killed
                self._stuck.add(fut)
        if not_done:
            log.error("Fast Gate: %d check(s) missed the %.3f s deadline: %s; "
                      "%d thread(s) still running", len(not_done), budget,
                      sorted(futures[f] for f in not_done), self.stuck_count)

        decision_ms = (clock() - t0) * 1000.0
        return {agent: self._aggregate(agent, results, snapshot_ms, decision_ms)
                for agent in self.cfg.agents}

    # ── aggregation ────────────────────────────────────────────────────
    def _aggregate(self, agent: str, results: dict, snapshot_ms: float,
                   decision_ms: float) -> GateResult:
        checks = []
        for c in self.checks:
            checks.append(results.get((agent, c.name))
                          or _fail(c.name, "MISSING"))     # belt and braces
        fails = tuple(c.check for c in checks if c.status == Status.FAIL)
        warns = tuple(c.check for c in checks if c.status == Status.WARN)
        overall = (Overall.VETO if fails else
                   Overall.APPROVE_WARN if warns else Overall.APPROVE)
        label = "UNKNOWN"
        for c in checks:
            if c.check == "traffic_classifier" and "label" in c.details:
                label = c.details["label"]
        return GateResult(agent=agent, checks=tuple(checks), overall=overall,
                          triggering=fails, warnings=warns,
                          classifier_label=label, snapshot_ms=snapshot_ms,
                          decision_ms=decision_ms)
