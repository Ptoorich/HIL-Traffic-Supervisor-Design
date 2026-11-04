"""Experimental Slow Monitor measurements and provisional trigger rules.

The decision rule is retained from the previously tested trial runner.
"""
import math


def attempt(fn):
    try:
        return fn(), None
    except Exception as exc:
        return None, f'{type(exc).__name__}: {exc}'


def capture_boundary(cfg, traci_mod):
    """Snapshot only; incomplete readings are null, never zero-filled."""
    t, terr = attempt(lambda: float(traci_mod.simulation.getTime()))
    pending, perr = attempt(lambda: len(traci_mod.simulation.getPendingVehicles()))
    worst = {}
    errors = {}
    for agent in cfg.agents:
        lanes = [lc for lc in cfg.lanes_of(agent) if not lc.excluded]
        values = []
        for lc in lanes:
            val, err = attempt(lambda lc=lc: int(traci_mod.lane.getLastStepHaltingNumber(lc.lane_id)))
            if err:
                errors[lc.lane_id] = err
            else:
                values.append({'lane': lc.lane_id, 'halting': val,
                               'capacity': lc.capacity, 'fill': val / lc.capacity})
        # If a lane is missing, a reported worst would be misleading.
        worst[agent] = (max(values, key=lambda x: x['fill'])
                        if len(values) == len(lanes) and values else None)
    return {'sim_time': t, 'pending_insertion': pending, 'worst_lane_fill': worst,
            'errors': {'time': terr, 'pending': perr, 'lanes': errors}}


class StepProbe:
    """Wrap simulationStep only while one existing env.step is executing."""
    def __init__(self, cfg, step_length, traci_mod):
        self.cfg, self.step_length, self.traci = cfg, step_length, traci_mod
        self.steps = 0
        self.totals = {'inserted': 0, 'arrived': 0, 'teleports_started': 0}
        self.errors = {}
        self.end_boundary = None
        self.original = None

    def __enter__(self):
        self.original = self.traci.simulationStep
        self.traci.simulationStep = self.step
        return self

    def __exit__(self, *exc):
        self.traci.simulationStep = self.original

    def step(self, *args, **kwargs):
        result = self.original(*args, **kwargs)
        self.steps += 1
        for key, method in (('inserted', 'getDepartedNumber'),
                            ('arrived', 'getArrivedNumber'),
                            ('teleports_started', 'getStartingTeleportNumber')):
            val, err = attempt(lambda method=method: int(getattr(self.traci.simulation, method)()))
            if err:
                self.errors.setdefault(key, err)
                self.totals[key] = None
            elif self.totals[key] is not None:
                self.totals[key] += val
        if self.steps == self.step_length:
            self.end_boundary = capture_boundary(self.cfg, self.traci)
        return result


def suffix_run(seq):
    if not seq:
        return 0
    n = 1
    while n < len(seq) and seq[-1-n] == seq[-1]:
        n += 1
    return n


def alternating_suffix(seq):
    """Length of exact A,B,A,B suffix; two values are descriptive, not a diagnosis."""
    if len(seq) < 2 or seq[-1] == seq[-2]:
        return 0
    n = 2
    while n < len(seq) and seq[-1-n] == seq[-1-(n % 2)]:
        n += 1
    return n


def behavior_summary(records, agent, lb, ub):
    # Only contiguous RL-controlled cycles with intact action/split readings.
    valid = []
    for record in reversed(records):
        if not record['complete_cycle']:
            break
        x = record['junctions'][agent]
        # The integrated controller logs the true mode before an accepted
        # release. That cycle is nevertheless executed by ASTA and must be
        # the first cycle in the new behaviour history.
        if ((x['mode_before'] != 'ASTA' and not x.get('handback_accepted', False))
                or x['mode_after'] != 'ASTA'
                or x['proposed_action'] not in (0, 1, 2)
                or x['split_after'] is None or x['split_before'] is None):
            break
        valid.append(x)
    valid.reverse()
    actions = [x['proposed_action'] for x in valid]
    splits = [x['split_after'] for x in valid]
    bound_attempts = []
    for x in valid:
        before, action = x['split_before'], x['proposed_action']
        bound_attempts.append((math.isclose(before, lb) and action == 0) or
                              (math.isclose(before, ub) and action == 2))
    return {'cycles_available': len(valid),
            'repeated_action_suffix_cycles': suffix_run(actions),
            'alternating_action_suffix_cycles': alternating_suffix(actions),
            'alternating_applied_split_suffix_cycles': alternating_suffix(splits),
            'outward_bound_attempt_suffix_cycles':
                suffix_run(bound_attempts) if bound_attempts and bound_attempts[-1] else 0}


def aggregate_window(records, cfg):
    rows = list(records)[-3:]
    if len(rows) != 3 or any(not x['complete_cycle'] for x in rows):
        return {'available': False, 'reason': 'fewer than three complete contiguous cycles'}
    if any(rows[i]['end_time'] != rows[i+1]['start_time'] for i in range(2)):
        return {'available': False, 'reason': 'cycle times are not contiguous'}
    start = rows[0]['boundary_start']['pending_insertion']
    end_boundary = rows[-1]['boundary_end']
    end = end_boundary['pending_insertion'] if end_boundary else None
    counts = {}
    for key in ('inserted', 'arrived', 'teleports_started'):
        vals = [x['step_counts'][key] for x in rows]
        counts[key] = sum(vals) if all(x is not None for x in vals) else None
    worst = {}
    for a in cfg.agents:
        boundaries = [rows[0]['boundary_start']] + [x['boundary_end'] for x in rows]
        readings = [b['worst_lane_fill'][a] if b else None for b in boundaries]
        worst[a] = max(readings, key=lambda x: x['fill']) if all(readings) else None
    return {'available': True, 'start_time': rows[0]['start_time'],
            'end_time': rows[-1]['end_time'], 'pending_start': start,
            'pending_end': end, 'pending_change': end-start if start is not None and end is not None else None,
            'counts': counts, 'worst_boundary_lane_fill': worst,
            'sampling_note': 'lane fill sampled at four cycle boundaries, not continuously',
            'modes': [{a: x['junctions'][a]['mode_after'] for a in cfg.agents} for x in rows]}


def evaluate_experimental_rule(row, cfg):
    """Describe provisional candidate; no mutation, unknown inputs never match."""
    window = row.get('traffic_window') or {}
    result = {'status': 'incomplete', 'pending_end': window.get('pending_end'),
              'pending_change': window.get('pending_change'), 'candidates': {},
              'parameters': {'minimum_pending': 150, 'growth_270s': 30,
                             'bound_cycles': 5, 'alternating_split_cycles': 6}}
    if not row.get('complete_cycle') or not window.get('available'):
        return result
    pending, growth = window.get('pending_end'), window.get('pending_change')
    if pending is None or growth is None or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (pending, growth)):
        result['status'] = 'unavailable_traffic_data'
        return result
    traffic_bad = pending >= 150 and growth >= 30
    result['status'] = 'evaluated'
    result['traffic_condition_met'] = traffic_bad
    for a in cfg.agents:
        behavior = row['behavior'][a]
        j = row['junctions'][a]
        bound = behavior['outward_bound_attempt_suffix_cycles'] >= 5
        alternate = behavior['alternating_applied_split_suffix_cycles'] >= 6
        eligible = ((j['mode_before'] == 'ASTA' or j.get('handback_accepted', False))
                    and j['mode_after'] == 'ASTA')
        result['candidates'][a] = {'eligible': eligible, 'bound_pattern': bound,
                                   'alternating_split_pattern': alternate,
                                   'candidate': bool(eligible and traffic_bad and (bound or alternate))}
    return result
