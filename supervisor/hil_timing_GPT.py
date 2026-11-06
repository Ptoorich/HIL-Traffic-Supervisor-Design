"""Host timing evidence only: never selects actions, sleeps, or changes a mode.

The two endpoints are the validated ACK after boundary action application and
the validated ACK after the first SUMO step. All timestamps use the same clock
as SerialDisplay. These measurements are not optical latency measurements.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import statistics

SCHEMA_VERSION = 1
BUDGET_METRICS = ('post_policy_to_boundary_ack_ms', 'policy_to_boundary_ack_ms',
                  'policy_to_first_step_ack_ms', 'boundary_to_first_step_ack_ms')


class TimedPolicy:
    """Delegate the existing policy unchanged, including its exception path."""
    def __init__(self, policy, timing):
        self.policy, self.timing = policy, timing

    def __getattr__(self, name):
        return getattr(self.policy, name)

    def act(self, *args, **kwargs):
        self.timing.mark('policy_started_at')
        status = 'raised'
        try:
            value = self.policy.act(*args, **kwargs)
            status = 'returned'
            return value
        finally:
            self.timing.mark('policy_finished_at')
            self.timing.pending['policy_status'] = status


def _stats(values):
    values = sorted(values)
    def percentile(p):
        h = (len(values) - 1) * p
        lo, hi = math.floor(h), math.ceil(h)
        return values[lo] + (values[hi] - values[lo]) * (h - lo)
    return {'n': len(values), 'mean': statistics.fmean(values),
            'median': statistics.median(values), 'p95': percentile(.95),
            'p99': percentile(.99), 'min': values[0], 'max': values[-1]}


class BoundaryTiming:
    def __init__(self, out, *, clock, budget_ms):
        if not math.isfinite(budget_ms) or budget_ms <= 0:
            raise ValueError('Timing budget must be finite and positive')
        self.out, self.clock, self.budget_ms = Path(out), clock, budget_ms
        self.pending, self.rows, self.log = None, [], None
        self.interrupted = None

    def __enter__(self):
        # Refuse to mix fresh telemetry with an old run, including its summary.
        if (self.out / 'timing_summary_GPT.json').exists():
            raise FileExistsError(self.out / 'timing_summary_GPT.json')
        self.log = (self.out / 'timing_GPT.jsonl').open('x', encoding='utf-8')
        return self

    def begin(self, cycle, sim_time):
        self.finish_cycle()
        self.pending = {'schema_version': SCHEMA_VERSION, 'cycle': cycle,
                        'sim_time': sim_time, 'boundary_started_at': self.clock(),
                        'budget_ms': self.budget_ms}

    def mark(self, name):
        self.pending[name] = self.clock()

    def boundary_ack(self, sample):
        self.pending['boundary_output'] = sample

    def before_first_step(self, display):
        self.pending['pacing_requested_ms'] = display.step_sleep_requested_ms
        self.pending['pacing_actual_ms'] = display.step_sleep_actual_ms
        self.mark('first_step_started_at')

    def first_step_ack(self, sample):
        self.pending['first_step_output'] = sample
        row = self.pending
        try:
            first, second = row['boundary_output'], row['first_step_output']
            a, b = first['ack'], second['ack']
            if (a['kind'] != 'S' or b['kind'] != 'S' or a['session'] != b['session']
                    or type(a['sequence']) is not int or type(b['sequence']) is not int
                    or b['sequence'] != a['sequence'] + 1):
                raise ValueError('ACKs do not identify consecutive STATE commands in one session')
            if first['sim_time'] != row['sim_time'] or second['sim_time'] != row['sim_time'] + 1:
                raise ValueError('ACKs are not for this boundary and its first simulated second')
            timeline = [row['boundary_started_at'], row['policy_started_at'],
                        row['policy_finished_at'], row['decision_finished_at'],
                        row['env_step_called_at'], first['sampled_at'], a['sent_at'],
                        a['ack_received_at'], row['first_step_started_at'],
                        second['sampled_at'], b['sent_at'], b['ack_received_at']]
            if (not all(math.isfinite(t) for t in timeline)
                    or any(v < u for u, v in zip(timeline, timeline[1:]))):
                raise ValueError('Missing, nonfinite or out-of-order timing markers')
            wait = row['pacing_actual_ms']
            requested = row['pacing_requested_ms']
            if not all(math.isfinite(t) and t >= 0 for t in (wait, requested)):
                raise ValueError('Invalid pacing measurement')
            if wait > (row['first_step_started_at'] - a['ack_received_at']) * 1000 + 1e-6:
                raise ValueError('Pacing interval exceeds the interval containing it')
            boundary, policy, proposal = (row['boundary_started_at'], row['policy_started_at'],
                                          row['policy_finished_at'])
            first_ack, next_ack = a['ack_received_at'], b['ack_received_at']
            m = {'policy_call_ms': (proposal - policy) * 1000,
                 'post_policy_to_boundary_ack_ms': (first_ack - proposal) * 1000,
                 'policy_to_boundary_ack_ms': (first_ack - policy) * 1000,
                 'boundary_to_boundary_ack_ms': (first_ack - boundary) * 1000,
                 'post_policy_to_first_step_ack_ms': (next_ack - proposal) * 1000,
                 'policy_to_first_step_ack_ms': (next_ack - policy) * 1000,
                 'boundary_to_first_step_ack_ms': (next_ack - boundary) * 1000,
                 'policy_to_first_step_ack_excluding_pacing_ms': (next_ack - policy) * 1000 - wait,
                 'pacing_requested_ms': requested, 'pacing_actual_ms': wait,
                 'decision_to_boundary_ack_ms': (first_ack - row['decision_finished_at']) * 1000,
                 'first_step_and_readback_to_ack_ms': (next_ack - row['first_step_started_at']) * 1000}
            row.update(status='complete', metrics_ms=m,
                       budget_exceeded={k: m[k] > self.budget_ms for k in BUDGET_METRICS})
        except (KeyError, TypeError, ValueError) as exc:
            row.update(status='invalid', reason=f'{type(exc).__name__}: {exc}')
        self._write_pending()

    def _write_pending(self):
        # Unexpected logging errors propagate like the existing evidence logs.
        self.log.write(json.dumps(self.pending, allow_nan=False) + '\n')
        self.log.flush()
        self.rows.append(self.pending)
        self.pending = None

    def finish_cycle(self, reason='No boundary/first-step ACK pair was completed'):
        if self.pending is not None:
            self.pending.update(status='incomplete', reason=reason)
            self._write_pending()

    def summary(self):
        complete = [r for r in self.rows if r['status'] == 'complete']
        incomplete = len(self.rows) - len(complete)
        status = 'INCOMPLETE' if incomplete or self.interrupted else ('COMPLETE' if complete else 'NO_DATA')
        exceedances = {k: sum(r['budget_exceeded'][k] for r in complete) for k in BUDGET_METRICS}
        return {'schema_version': SCHEMA_VERSION, 'measurement_status': status,
                'boundaries_started': len(self.rows), 'boundaries_completed': len(complete),
                'incomplete_or_invalid_boundaries': incomplete, 'budget_ms': self.budget_ms,
                'run_interrupted_by': self.interrupted,
                'budget_status': ('INCOMPLETE' if status != 'COMPLETE' else
                                  'EXCEEDED' if any(exceedances.values()) else 'WITHIN_OBSERVED_BUDGET'),
                'budget_exceedances': exceedances,
                'metrics_ms': {k: _stats([r['metrics_ms'][k] for r in complete])
                               for k in (complete[0]['metrics_ms'] if complete else ())},
                'policy_calls_raised': sum(r.get('policy_status') == 'raised' for r in self.rows),
                'clock': 'host monotonic, shared with serial transport',
                'scope': 'cycle boundary/policy call through validated GPIO ACK; not optical latency',
                'policy_scope': 'StrictPolicy.act includes observation validation and model predictions',
                'pacing_scope': 'first-step totals include actual sleep; excluded-pacing metric is derived',
                'budget_scope': 'observational comparison only; never changes control or watchdogs',
                'percentiles': 'linear interpolation at (n-1)*p; all complete records included'}

    def __exit__(self, typ, value, tb):
        self.interrupted = typ.__name__ if typ is not None else None
        try:
            reason = f'Run interrupted by {typ.__name__}' if typ is not None else 'No boundary/first-step ACK pair was completed'
            self.finish_cycle(reason)
            with (self.out / 'timing_summary_GPT.json').open('x', encoding='utf-8') as target:
                json.dump(self.summary(), target, indent=2, allow_nan=False)
        except Exception:
            if typ is None:
                raise
            # Preserve the original control/serial error if writing diagnostics also fails.
        finally:
            self.log.close()
