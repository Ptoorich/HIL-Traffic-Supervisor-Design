"""Initial two-junction supervisor. Contingency remains latched until reset.

Use a fresh instance (or reset_episode()) for each new env.reset(). This wrapper
returns actions; the existing evaluation loop remains responsible for env.step().
"""
from __future__ import annotations

import json
import logging
import math
import operator
import time
from dataclasses import asdict
from pathlib import Path

from .config import validate_against_env
from .datatypes import Overall, Proposal
from .fast_gate import FastGate
from .snapshot import SnapshotTaker

LOG = logging.getLogger(__name__)
DELTAS = {0: -5, 1: 0, 2: 5}


def _json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


class SupervisedController:
    """Wrap an existing act(env, obs, t) controller.

    No automatic handback, shadow evaluation, HIL or Slow Monitor is performed.
    Logging is synchronous; an I/O failure raises before env.step can be called.
    """
    def __init__(self, inner, cfg, matrix, log_path=None, *, snapshot_taker=None,
                 gate=None, clock=time.perf_counter, run_id=None):
        self.inner, self.cfg, self.clock = inner, cfg, clock
        self.matrix = matrix
        self.name = f"supervised_{getattr(inner, 'name', 'controller')}"
        self.run_id = run_id or self.name
        self.snapshot_taker = snapshot_taker or SnapshotTaker(cfg)
        self.gate = gate or FastGate(cfg, matrix)
        self._owns_gate = gate is None
        self.log_path = Path(log_path or cfg.logging.jsonl_path)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log = self.log_path.open('a', encoding='utf-8')
        self.modes = {a: 'ASTA' for a in cfg.agents}
        self._validated_env = None
        self.episode_id = None

    def reset_episode(self, episode_id=None):
        """Explicitly clear latches only at an episode boundary after env.reset()."""
        self.modes = {a: 'ASTA' for a in self.cfg.agents}
        self.episode_id = episode_id
        self.snapshot_taker.clear_cache()
        self._validated_env = None

    def close(self):
        if self._owns_gate:
            self.gate.close()
        self._log.close()

    def __enter__(self): return self
    def __exit__(self, *args): self.close()

    def act(self, env, obs, t):
        if self._validated_env is not env:
            validate_against_env(self.cfg, env)
            for agent in self.cfg.agents:
                junction = self.matrix.junctions.get(agent)
                if junction is None or junction.tl_id != env.tl_map[agent]:
                    raise ValueError(f"conflict fixture does not match traffic light for {agent}")
            self._validated_env = env
        agents = self.cfg.agents
        before = {a: float(env.phase_splits[a]) for a in agents}
        # Policy prediction time is excluded from the supervisory timer.
        try:
            raw_actions = self.inner.act(env, obs, t)
            candidate_base = {a: float(env.phase_splits[a]) for a in agents}
            inner_error = None
        except Exception as e:
            raw_actions, candidate_base = {}, dict(before)
            inner_error = f"{type(e).__name__}: {e}"
        start = self.clock()
        # Remove any set_splits side effects before evaluating any proposals.
        env.set_splits(before)
        errors = {}
        if inner_error:
            errors = {a: f"INNER_ERROR: {inner_error}" for a in agents}
        if not isinstance(raw_actions, dict):
            errors = {a: 'INVALID_ACTION_MAP' for a in agents}
            raw_actions = {}
        if set(raw_actions) - set(agents):
            errors = {a: 'UNKNOWN_ACTION_AGENT' for a in agents}
        actions, proposals = {}, {}
        for a in agents:
            try:
                if isinstance(raw_actions.get(a, 1), bool): raise ValueError('boolean')
                action = operator.index(raw_actions.get(a, 1))
                if action not in DELTAS: raise ValueError('outside 0..2')
                if not math.isfinite(candidate_base[a]): raise ValueError('non-finite split')
            except (ValueError, TypeError, OverflowError):
                errors[a] = 'INVALID_PROPOSAL'
                action, candidate_base[a] = 1, before[a]
            actions[a] = action
            proposals[a] = Proposal(a, before[a], DELTAS[action],
                                    candidate_base[a] + DELTAS[action], self.name)
        snapshot_start = self.clock()
        snapshot_error = None
        try:
            snapshots = self.snapshot_taker.take(env, current_splits=before)
        except Exception as e:
            snapshots = {}
            snapshot_error = f"{type(e).__name__}: {e}"
        snapshot_s = self.clock() - snapshot_start
        budget_left = self.cfg.timing.fast_gate_stage_s - snapshot_s
        gate_start = self.clock()
        try:
            results = self.gate.evaluate(snapshots, proposals,
                                         deadline_s=max(0.0, budget_left),
                                         snapshot_ms=snapshot_s * 1000)
        except Exception as e:
            results = {}
            errors.update({a: f"GATE_ERROR: {type(e).__name__}: {e}" for a in agents})
        gate_s = self.clock() - gate_start
        if budget_left <= 0 or snapshot_s + gate_s > self.cfg.timing.fast_gate_stage_s:
            errors.update({a: 'FAST_GATE_STAGE_TIMEOUT' for a in agents})
        if snapshot_error:
            errors.update({a: f'SNAPSHOT_ERROR: {snapshot_error}' for a in agents})
        resolver_start = self.clock()
        decisions, result_actions, base_to_apply = {}, {}, {}
        for a in agents:
            result = results.get(a)
            if result is None or getattr(result, 'overall', None) not in (Overall.APPROVE, Overall.APPROVE_WARN):
                errors.setdefault(a, 'FAST_GATE_VETO_OR_MISSING')
            previous = self.modes[a]
            if a in errors:
                self.modes[a] = 'CONTINGENCY'
            if self.modes[a] == 'CONTINGENCY':
                base_to_apply[a], result_actions[a] = float(self.cfg.contingency[a]), 1
            else:
                base_to_apply[a], result_actions[a] = candidate_base[a], actions[a]
            decisions[a] = {'mode_before': previous, 'mode_after': self.modes[a],
                            'entered_contingency': previous != self.modes[a],
                            'reason': errors.get(a), 'proposal': asdict(proposals[a]),
                            'proposed_action': actions[a], 'returned_action': result_actions[a],
                            'applied_base_split': base_to_apply[a],
                            'gate': result.to_log_dict() if result is not None else None}
        env.set_splits(base_to_apply)
        resolver_s = self.clock() - resolver_start
        total_s = self.clock() - start
        entry = {'run_id': self.run_id, 'episode_id': self.episode_id, 'sim_time': t,
                 'decisions': decisions, 'snapshot_ms': snapshot_s * 1000,
                 'gate_ms': gate_s * 1000, 'resolver_ms': resolver_s * 1000,
                 'supervisor_ms_before_log': total_s * 1000,
                 'fast_gate_stage_timeout': snapshot_s + gate_s > self.cfg.timing.fast_gate_stage_s,
                 'resolver_budget_exceeded': resolver_s > self.cfg.timing.resolver_stage_s,
                 'total_budget_exceeded_before_log': total_s > self.cfg.timing.total_budget_s,
                 'timing_scope': 'post_prediction_through_action_preparation; excludes log, env.step and LED'}
        self._log.write(json.dumps(_json_safe(entry), allow_nan=False) + '\n')
        self._log.flush()
        return result_actions
