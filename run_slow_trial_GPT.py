"""Opt-in Slow Monitor integration trial. Run from project root.

Without --enable-slow-trial this is observation only. The opt-in rule uses
experimental thresholds and latches matching junctions at the next boundary.
The existing Fast Gate remains active. Not validated for real signal hardware.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import traceback
from collections import deque
from pathlib import Path

import traci
import tod_plan as tp
import tod_eval_common as ec
from sumo_env_marl_ps import SumoMARLEnvPS
from supervisor.config import load_config
from supervisor.conflicts import load_fixture
from supervisor.supervised_controller_GPT import SupervisedController


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


class TracedSarlPolicy(ec.SarlPolicy):
    """Same action mapping as SarlPolicy, retaining the original joint action."""
    def __init__(self, model, name, deterministic=True):
        super().__init__(model, name, deterministic)
        self.last_joint_action = None
        self.last_selected_agent = None

    def act(self, env, obs, t):
        self.last_joint_action = self.last_selected_agent = None
        joint = int(self.model.predict(self.build_obs(env, obs), deterministic=self.det)[0])
        idx, action = joint // 3, joint % 3
        agent = env.agent_ids[idx]
        actions = {a: 1 for a in env.agent_ids}
        actions[agent] = action
        self.last_joint_action, self.last_selected_agent = joint, agent
        return actions


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
        if (x['mode_before'] != 'ASTA' or x['mode_after'] != 'ASTA'
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
        eligible = j['mode_before'] == 'ASTA' and j['mode_after'] == 'ASTA'
        result['candidates'][a] = {'eligible': eligible, 'bound_pattern': bound,
                                   'alternating_split_pattern': alternate,
                                   'candidate': bool(eligible and traffic_bad and (bound or alternate))}
    return result


def latest_decision(handle):
    line = handle.readline()
    if not line:
        return None
    return json.loads(line)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--controller', required=True, choices=['static', 'marl-ppo', 'marl-dqn', 'sarl-ppo', 'sarl-dqn'])
    p.add_argument('--model', help='required for RL controllers')
    p.add_argument('--output-dir', required=True, help='new directory; existing directories are refused')
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--peak-scale', type=float, default=ec.DEFAULT_SCALE)
    p.add_argument('--behavior-cycles', type=int, default=10,
                   help='descriptive history length, not a threshold (default 10)')
    p.add_argument('--enable-slow-trial', action='store_true',
                   help='enable experimental per-junction contingency at the next boundary')
    p.add_argument('--max-cycles', type=int, default=None,
                   help='optional short smoke run; omit for a full episode')
    args = p.parse_args()
    if args.controller != 'static' and not args.model: p.error('--model required for RL controllers')
    if args.behavior_cycles < 6: p.error('--behavior-cycles must be at least 6 for the trial rule')
    if args.max_cycles is not None and args.max_cycles < 1: p.error('--max-cycles must be positive')
    out = Path(args.output_dir).resolve()
    out.mkdir(parents=True, exist_ok=False)
    config, cfg = ec.make_config(), load_config()
    matrix = load_fixture(cfg.conflicts.fixture_path)
    mix = tp.load_route_mix('2int_files/routes.xml')
    peak = args.peak_scale * tp.ORIGINAL_RATE_VPH
    if args.controller == 'static':
        schedule = tp.build_schedule(mix, config['lane_map'], peak, config['cycle_length'],
                                     config['yellow_time'], config['allred_time'], config['s_lb'], config['s_ub'])
        inner = ec.StaticController(schedule)
    else:
        from stable_baselines3 import PPO, DQN
        kind, method = args.controller.split('-')
        algo = PPO if method == 'ppo' else DQN
        model = algo.load(args.model, device='cpu')
        if kind == 'marl':
            obs_size = max(len(v) for v in config['lane_map'].values()) + 2
            actions = 3
        else:
            obs_size = sum(len(v) for v in config['lane_map'].values()) + len(config['agent_ids'])
            actions = 3 * len(config['agent_ids'])
        if model.observation_space.shape != (obs_size,) or getattr(model.action_space, 'n', None) != actions:
            raise ValueError(f'Unexpected {kind.upper()} model spaces: {model.observation_space}, '
                             f'{model.action_space}; expected ({obs_size},), Discrete({actions})')
        policy = TracedSarlPolicy if kind == 'sarl' else ec.MarlPolicy
        inner = policy(model, args.controller, deterministic=True)
    route = out / 'routes_seed42_GPT.xml' if args.seed == 42 else out / f'routes_seed{args.seed}_GPT.xml'
    tp.generate_route_file(str(route), mix, args.seed, peak)
    meta = {'controller': args.controller, 'seed': args.seed, 'peak_scale': args.peak_scale,
            'route_sha256': hashlib.sha256(route.read_bytes()).hexdigest(),
            'model_sha256': hashlib.sha256(Path(args.model).read_bytes()).hexdigest() if args.model else None,
            'behavior_history_cycles': args.behavior_cycles,
            'mode': 'experimental_slow_contingency' if args.enable_slow_trial else 'observation_only', 'traffic_window_cycles': 3,
            'sarl_action_note': 'joint action selects one agent; other agent receives a mapped no-op' if args.controller.startswith('sarl-') else None}
    (out / 'metadata_GPT.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    decisions, monitor = out / 'decisions_GPT.jsonl', out / 'slow_observation_GPT.jsonl'
    env = SumoMARLEnvPS(config)
    history = deque(maxlen=max(3, args.behavior_cycles))
    first = config['agent_ids'][0]
    try:
        with SupervisedController(inner, cfg, matrix, log_path=decisions, run_id=f'observe_{args.controller}') as ctrl:
            obs = env.reset(seed=args.seed, route_file=str(route))
            ctrl.reset_episode(episode_id=args.seed)
            with decisions.open(encoding='utf-8') as decision_reader, monitor.open('x', encoding='utf-8') as output, (out / 'slow_trial_events_GPT.jsonl').open('x', encoding='utf-8') as events:
                cycle = 0
                pending_switches = {}
                while True:
                    start_time = env.current_step
                    applied_slow_switches = []
                    for a, evidence in pending_switches.items():
                        previous = ctrl.modes[a]
                        if previous == 'ASTA':
                            # Trial hook: use the wrapper's existing latch and normal
                            # Fast Gate path. No mid-cycle split change occurs here.
                            ctrl.modes[a] = 'CONTINGENCY'
                            applied_slow_switches.append({'agent': a, 'mode_before': previous,
                                'mode_after': ctrl.modes[a], 'boundary_time': start_time,
                                'trigger_evidence': evidence})
                    pending_switches = {}
                    for event in applied_slow_switches:
                        events.write(json.dumps({'event': 'latch_set_before_act', **event}, allow_nan=False) + '\n')
                        events.flush()
                    before = {a: float(env.phase_splits[a]) for a in cfg.agents}
                    boundary_start = capture_boundary(cfg, traci)
                    actions = ctrl.act(env, obs, start_time)
                    entry, read_err = attempt(lambda: latest_decision(decision_reader))
                    modes_after_act = dict(ctrl.modes)
                    with StepProbe(cfg, env.step_length, traci) as probe:
                        obs, _, terminated = env.step(actions)
                    complete = probe.steps == env.step_length
                    # current_step is overwritten on early stop. Derive end from executed steps.
                    end_time = start_time + probe.steps
                    junctions = {}
                    for a in cfg.agents:
                        d = ((entry or {}).get('decisions') or {}).get(a, {})
                        junctions[a] = {'split_before': before[a],
                            'proposed_action': d.get('proposed_action'),
                            'requested_split': (d.get('proposal') or {}).get('requested_split'),
                            'returned_action': actions.get(a),
                            'split_after': float(env.phase_splits[a]),
                            'mode_before': d.get('mode_before'),
                            'mode_after': d.get('mode_after', modes_after_act[a]),
                            'gate_overall': (d.get('gate') or {}).get('overall')}
                    row = {'cycle': cycle, 'start_time': start_time, 'end_time': end_time,
                           'executed_steps': probe.steps, 'complete_cycle': complete,
                           'terminated': bool(terminated[first]),
                           'boundary_start': boundary_start, 'boundary_end': probe.end_boundary,
                           'step_counts': probe.totals, 'step_count_errors': probe.errors,
                           'decision_read_error': read_err, 'junctions': junctions,
                           'sarl_joint_action': getattr(inner, 'last_joint_action', None),
                           'sarl_selected_agent': getattr(inner, 'last_selected_agent', None)}
                    history.append(row)
                    row['traffic_window'] = aggregate_window(history, cfg)
                    row['behavior'] = {a: behavior_summary(list(history)[-args.behavior_cycles:], a,
                                                           env.s_lb, env._split_ub()) for a in cfg.agents}
                    row['slow_trial_applied'] = applied_slow_switches
                    row['slow_trial_evaluation'] = evaluate_experimental_rule(row, cfg)
                    can_continue = not terminated[first] and (args.max_cycles is None or cycle + 1 < args.max_cycles)
                    if args.enable_slow_trial and can_continue:
                        for a, result in row['slow_trial_evaluation']['candidates'].items():
                            if result['candidate']:
                                pending_switches[a] = {'evaluated_at': end_time,
                                    'pending_end': row['traffic_window']['pending_end'],
                                    'pending_change': row['traffic_window']['pending_change'],
                                    'behavior': dict(row['behavior'][a]),
                                    'matched_patterns': [key for key in ('bound_pattern', 'alternating_split_pattern')
                                                         if result[key]]}
                    row['slow_trial_scheduled_for_next_boundary'] = list(pending_switches)
                    # Keep telemetry separate from the controller's decisions.
                    output.write(json.dumps(row, allow_nan=False) + '\n'); output.flush()
                    cycle += 1
                    if cycle == 1 or cycle % 20 == 0 or terminated[first] or applied_slow_switches or pending_switches:
                        print(f'cycle {cycle}, sim t={end_time}, modes={ctrl.modes}, '
                              f'pending={(probe.end_boundary or {}).get("pending_insertion")}')
                    if terminated[first] or (args.max_cycles is not None and cycle >= args.max_cycles):
                        break
    finally:
        env.close()
    print(f'Finished observation run. Files: {out}')
    print('Experimental slow contingency ENABLED.' if args.enable_slow_trial else 'Observation only; slow contingency disabled.')
    print('Existing Fast Gate remained active. Check slow_trial_applied for actual latch transitions.')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'Observation run failed: {type(exc).__name__}: {exc}', file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)
