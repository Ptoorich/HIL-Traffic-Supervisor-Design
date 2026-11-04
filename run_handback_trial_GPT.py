"""Opt-in SUMO trial of per-junction FTSC-to-RL handback.

This is an experiment, not production integration or a hardware safety system.
Run from the root of the existing two-junction project. The normal supervisor
and its episode-latched behaviour are not modified by this script.
"""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import traceback
import xml.etree.ElementTree as ET

import traci
import tod_plan as tp
import tod_eval_common as ec
from sumo_env_marl_ps import SumoMARLEnvPS
from supervisor.config import load_config
from supervisor.conflicts import load_fixture
from supervisor.supervised_controller_GPT import SupervisedController

from handback_slow_support_GPT import (StepProbe, aggregate_window, behavior_summary,
    capture_boundary, evaluate_experimental_rule, latest_decision)
from handback_trial_adapters_GPT import (InjectedSnapshotTaker, StrictPolicy,
    read_active_programs)
from handback_trial_logic_GPT import Recovery, valid_pending


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def verify_compatible_sources(project):
    """Guard the trial against an unreviewed change in its controller contracts."""
    manifest = Path(__file__).with_name('source_fingerprints_GPT.json')
    fingerprints = json.loads(manifest.read_text(encoding='utf-8'))['files']
    differences = []
    for name, expected in fingerprints.items():
        path = project / name
        if not path.is_file():
            differences.append(f'{name}: missing')
            continue
        content = path.read_bytes().replace(b'\r\n', b'\n')
        actual = hashlib.sha256(content).hexdigest()
        if actual != expected:
            differences.append(f'{name}: differs from reviewed snapshot')
    if differences:
        raise RuntimeError('Project source does not match the snapshot used to prepare this trial. '
                           'Please share these current files for review:\n  ' + '\n  '.join(differences))
    return fingerprints


def _contained_path(base, root, value):
    """Reject explicit file references escaping the copied SUMO work folder."""
    if not value or value in ('-', 'stdout', 'NUL', '/dev/null'):
        return None
    candidate = (base / value).resolve()
    if not candidate.is_relative_to(root.resolve()):
        raise ValueError(f'SUMO file reference escapes isolated work folder: {value!r}')
    return candidate


def isolate_scenario(project, out):
    source = project / '2int_files'
    if out.is_relative_to(source.resolve()):
        raise ValueError('Place the output directory outside 2int_files')
    work = out / 'sumo_work'
    work.mkdir()
    shutil.copytree(source, work / '2int_files')
    cfg = work / '2int_files' / '2int.sumocfg'
    root = ET.parse(cfg).getroot()
    input_node, output_node = root.find('input'), root.find('output')
    additional = []
    if input_node is not None:
        for el in input_node:
            val = el.get('value', '')
            for part in val.split(','):
                p = _contained_path(cfg.parent, work, part.strip())
                if el.tag == 'additional-files' and p is not None:
                    additional.append(p)
    if output_node is not None:
        for el in output_node:
            _contained_path(cfg.parent, work, el.get('value', ''))
    for path in additional:
        if not path.exists():
            raise FileNotFoundError(f'Missing copied SUMO additional file: {path}')
        # All file= references in additional files must remain inside this run.
        opener = __import__('gzip').open if path.suffix == '.gz' else open
        with opener(path, 'rb') as f:
            additional_root = ET.parse(f).getroot()
        for element in additional_root.iter():
            value = element.get('file')
            if value:
                _contained_path(path.parent, work, value)
    return work, cfg


def boundary_ok(boundary, agent, expected_time):
    if not isinstance(boundary, dict) or not valid_pending(boundary.get('pending_insertion')):
        return False
    try:
        tm = float(boundary['sim_time'])
        worst = boundary['worst_lane_fill'][agent]
        fill = float(worst['fill'])
        errors = boundary['errors']
        return (math.isfinite(tm) and math.isclose(tm, expected_time, abs_tol=1e-6)
                and math.isfinite(fill) and fill >= 0
                and errors.get('time') is None and errors.get('pending') is None)
    except (KeyError, TypeError, ValueError):
        return False


def counters_ok(probe):
    return (not probe.errors and all(valid_pending(probe.totals.get(key))
                                     for key in ('inserted', 'arrived', 'teleports_started')))


def gate_ok(decision, policy, agent):
    gate = decision.get('gate') or {}
    return (decision.get('reason') is None
            and gate.get('overall') in ('APPROVE', 'APPROVE_WARN')
            and agent not in policy.last_observation_errors)


def missing_lane_confirmed(decision, lane):
    for check in ((decision.get('gate') or {}).get('checks') or []):
        if check.get('check') == 'input_validator' and check.get('status') == 'FAIL':
            return any(item.get('code') == 'LANE_MISSING' and item.get('lane') == lane
                       for item in (check.get('details') or {}).get('violations', []))
    return False


class EvidenceProbe(StepProbe):
    def __init__(self, cfg, env, traci_mod):
        super().__init__(cfg, env.step_length, traci_mod)
        self.env, self.active_programs = env, None

    def step(self, *args, **kwargs):
        result = super().step(*args, **kwargs)
        if self.steps == 1:
            # Abort the experimental run on missing or mismatched readback.
            self.active_programs = read_active_programs(self.env, self.traci)
        return result


def build_policy(args, config):
    if args.controller == 'demo':
        if args.model:
            raise ValueError('The deterministic demo policy does not use --model')
        return StrictPolicy('demo', None, traci, demo_agent=args.fault_agent), None
    if not args.model:
        raise ValueError('--model is required for an RL controller')
    from stable_baselines3 import DQN, PPO
    kind, algo = args.controller.split('-')
    model_path = Path(args.model).resolve(strict=True)
    model = (PPO if algo == 'ppo' else DQN).load(str(model_path), device='cpu')
    shape = (max(len(v) for v in config['lane_map'].values()) + 2 if kind == 'marl'
             else sum(len(v) for v in config['lane_map'].values()) + len(config['agent_ids']))
    n_actions = 3 if kind == 'marl' else 3 * len(config['agent_ids'])
    if model.observation_space.shape != (shape,) or getattr(model.action_space, 'n', None) != n_actions:
        raise ValueError(f'Unexpected model spaces: {model.observation_space}, {model.action_space}; '
                         f'expected ({shape},), Discrete({n_actions})')
    return StrictPolicy(kind, model, traci, demo_agent=args.fault_agent), model_path


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', required=True,
                        choices=['demo', 'marl-ppo', 'marl-dqn', 'sarl-ppo', 'sarl-dqn'])
    parser.add_argument('--model', help='trained model archive, required except for demo')
    parser.add_argument('--output-dir', required=True, help='new directory; existing paths are refused')
    parser.add_argument('--enable-handback-trial', action='store_true',
                        help='required: enable experimental automatic handback')
    parser.add_argument('--enable-slow-trial', action='store_true',
                        help='also enable the previously approved provisional Slow Monitor trigger')
    parser.add_argument('--fault-demo', action='store_true',
                        help='inject one missing snapshot lane, then another after verified handback')
    parser.add_argument('--fault-agent', default='tl1', choices=['tl1', 'tl2'])
    parser.add_argument('--fault-at-cycle', type=int, default=2, help='zero-based first fault cycle')
    parser.add_argument('--max-cycles', type=int, default=None)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--peak-scale', type=float, default=ec.DEFAULT_SCALE)
    args = parser.parse_args()
    if not args.enable_handback_trial:
        parser.error('this experimental runner requires --enable-handback-trial')
    if args.fault_at_cycle < 0 or (args.max_cycles is not None and args.max_cycles < 1):
        parser.error('fault cycle must be nonnegative and max cycles must be positive')
    if not math.isfinite(args.peak_scale) or args.peak_scale <= 0:
        parser.error('peak scale must be finite and positive')
    return args


def main():
    args = parse_args()
    project = Path.cwd().resolve()
    fingerprints = verify_compatible_sources(project)
    out = Path(args.output_dir).resolve()
    out.mkdir(parents=True, exist_ok=False)
    config, cfg = ec.make_config(), load_config()
    matrix = load_fixture(cfg.conflicts.fixture_path)
    policy, model_path = build_policy(args, config)
    mix = tp.load_route_mix(str(project / '2int_files' / 'routes.xml'))
    peak = args.peak_scale * tp.ORIGINAL_RATE_VPH
    route = out / f'routes_seed{args.seed}_GPT.xml'
    tp.generate_route_file(str(route), mix, args.seed, peak)
    work, copied_config = isolate_scenario(project, out)
    config['sumocfg'] = str(copied_config)
    meta = {'controller': args.controller, 'seed': args.seed, 'peak_scale': args.peak_scale,
            'route_sha256': digest(route), 'model_sha256': digest(model_path) if model_path else None,
            'fault_demo': args.fault_demo, 'fault_agent': args.fault_agent,
            'reviewed_source_fingerprints': fingerprints,
            'slow_trial_enabled': args.enable_slow_trial,
            'parameters': {'fixed_dwell_cycles': 5, 'clean_cycles': 3,
                           'sarl_selections': 3, 'sarl_selection_window': 10,
                           'slow_backlog_below': 100, 'slow_backlog_cycles': 3,
                           'transition_cycles': 1},
            'scope': 'experimental simulation; not production or hardware validation'}
    (out / 'metadata_GPT.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
    decisions = out / 'decisions_GPT.jsonl'
    observations = out / 'handback_observations_GPT.jsonl'
    events_path = out / 'handback_events_GPT.jsonl'
    states = {a: Recovery(is_sarl=policy.kind == 'sarl') for a in cfg.agents}
    history, slow_switches = deque(maxlen=10), {}
    first = config['agent_ids'][0]
    injected_count = 0
    arm_second = False
    summary = {'first_fault_confirmed': False, 'first_fixed_program_confirmed': False,
               'handback_completed': False, 'second_fault_confirmed': False,
               'second_fixed_program_confirmed': False, 'cycles_observed': 0, 'complete_cycles': 0}
    env = None
    try:
        os.chdir(work)
        env = SumoMARLEnvPS(config)
        with SupervisedController(policy, cfg, matrix, log_path=decisions,
                                  run_id=f'handback_{args.controller}') as ctrl:
            injector = InjectedSnapshotTaker(ctrl.snapshot_taker)
            ctrl.snapshot_taker = injector
            obs = env.reset(seed=args.seed, route_file=str(route))
            ctrl.reset_episode(episode_id=args.seed)
            with decisions.open(encoding='utf-8') as decision_reader, \
                 observations.open('x', encoding='utf-8') as log, \
                 events_path.open('x', encoding='utf-8') as events:

                def event(name, **fields):
                    events.write(json.dumps({'event': name, **fields}, allow_nan=False) + '\n')
                    events.flush()

                cycle = 0
                while True:
                    start = env.current_step
                    before = {a: float(env.phase_splits[a]) for a in cfg.agents}
                    programs_before = read_active_programs(env, traci)
                    boundary_start = capture_boundary(cfg, traci)
                    for agent, evidence in slow_switches.items():
                        if ctrl.modes[agent] == 'ASTA':
                            ctrl.modes[agent] = 'CONTINGENCY'
                            states[agent].enter('slow_monitor', start)
                            event('slow_contingency_set', agent=agent, sim_time=start, evidence=evidence)
                    slow_switches = {}
                    fault_number = None
                    if args.fault_demo and ((injected_count == 0 and cycle == args.fault_at_cycle)
                                            or (injected_count == 1 and arm_second)):
                        if ctrl.modes[args.fault_agent] != 'ASTA':
                            raise RuntimeError('Cannot demonstrate a new fault: target junction is already in contingency')
                        lane = cfg.lanes_of(args.fault_agent)[0].lane_id
                        injector.arm(args.fault_agent, lane)
                        injected_count += 1
                        fault_number = injected_count
                        arm_second = False
                        event('synthetic_fault_armed', agent=args.fault_agent, lane=lane,
                              sim_time=start, fault_number=fault_number)
                    release_attempts = set()
                    for agent in cfg.agents:
                        state = states[agent]
                        if not state.active or not state.transition_completed:
                            continue
                        pending = boundary_start.get('pending_insertion')
                        can_release = (ctrl.modes[agent] == 'CONTINGENCY'
                                       and boundary_ok(boundary_start, agent, start)
                                       and math.isclose(before[agent], float(cfg.contingency[agent]), abs_tol=1e-6)
                                       and ('slow_monitor' not in state.causes
                                            or (valid_pending(pending) and pending < state.low_pending_limit)))
                        if can_release:
                            ctrl.modes[agent] = 'ASTA'
                            release_attempts.add(agent)
                        else:
                            state.invalidate()
                            event('handback_cancelled', agent=agent, sim_time=start,
                                  reason='unavailable or changed boundary evidence')
                    actions = ctrl.act(env, obs, start)
                    entry = latest_decision(decision_reader)
                    if not isinstance(entry, dict) or set(entry.get('decisions', {})) != set(cfg.agents):
                        raise RuntimeError('The controller did not produce a complete decision log; no step performed')
                    decision_map = entry['decisions']
                    if fault_number is not None:
                        info = injector.last_injected
                        d_fault = decision_map[args.fault_agent]
                        if (info is None or not missing_lane_confirmed(d_fault, info['lane'])
                                or d_fault.get('mode_before') != 'ASTA'
                                or d_fault.get('mode_after') != 'CONTINGENCY'
                                or not d_fault.get('entered_contingency')):
                            raise RuntimeError('The synthetic missing lane was not confirmed by the Input Validator')
                        event('synthetic_fault_confirmed', **info, fault_number=fault_number,
                              validator='LANE_MISSING',
                              other_junctions={a: {'mode_before': decision_map[a].get('mode_before'),
                                                   'mode_after': decision_map[a].get('mode_after'),
                                                   'reason': decision_map[a].get('reason'),
                                                   'gate_overall': (decision_map[a].get('gate') or {}).get('overall')}
                                               for a in cfg.agents if a != args.fault_agent})
                        summary['first_fault_confirmed' if fault_number == 1 else 'second_fault_confirmed'] = True
                    transitions, released = set(), set()
                    for agent in cfg.agents:
                        d, state = decision_map[agent], states[agent]
                        approved = gate_ok(d, policy, agent)
                        if agent in release_attempts:
                            if approved and ctrl.modes[agent] == 'ASTA':
                                released.add(agent)
                                state.leave()
                            else:
                                if ctrl.modes[agent] != 'CONTINGENCY':
                                    raise RuntimeError('Release error did not restore contingency')
                                state.enter('fast_gate', start, restart=True)
                                event('handback_vetoed', agent=agent, sim_time=start, reason=d.get('reason'))
                        if ctrl.modes[agent] == 'CONTINGENCY':
                            if not state.active:
                                state.enter('fast_gate', start)
                                event('fast_contingency_entered', agent=agent, sim_time=start,
                                      reason=d.get('reason'))
                            elif not approved:
                                state.causes.add('fast_gate')
                            pending = boundary_start.get('pending_insertion')
                            if (state.ready() and approved and boundary_ok(boundary_start, agent, start)
                                    and math.isclose(before[agent], float(cfg.contingency[agent]), abs_tol=1e-6)
                                    and ('slow_monitor' not in state.causes
                                         or (valid_pending(pending) and pending < state.low_pending_limit))):
                                transitions.add(agent)
                                event('transition_started', agent=agent, sim_time=start,
                                      recovery=state.describe())
                    modes_after_act = dict(ctrl.modes)
                    with EvidenceProbe(cfg, env, traci) as probe:
                        obs, _, terminated = env.step(actions)
                    complete = probe.steps == env.step_length
                    end = start + probe.steps
                    programs = probe.active_programs
                    if probe.steps and programs is None:
                        raise RuntimeError('Active signal program was not captured')
                    for agent in cfg.agents:
                        if (modes_after_act[agent] == 'CONTINGENCY' and programs
                                and not math.isclose(programs[agent]['green_a'], float(cfg.contingency[agent]), abs_tol=1e-6)):
                            raise RuntimeError(f'{agent}: active program does not match contingency split')
                    if (fault_number is not None and complete and programs
                            and modes_after_act[args.fault_agent] == 'CONTINGENCY'):
                        key = 'first_fixed_program_confirmed' if fault_number == 1 else 'second_fixed_program_confirmed'
                        summary[key] = True
                        event('fixed_program_confirmed', agent=args.fault_agent, sim_time=start + 1,
                              fault_number=fault_number, program=programs[args.fault_agent])
                    junctions = {}
                    for agent in cfg.agents:
                        d = decision_map[agent]
                        junctions[agent] = {'split_before': before[agent],
                            'proposed_action': d.get('proposed_action'),
                            'requested_split': (d.get('proposal') or {}).get('requested_split'),
                            'returned_action': actions.get(agent), 'split_after': float(env.phase_splits[agent]),
                            'mode_before': d.get('mode_before'), 'mode_after': d.get('mode_after'),
                            'gate_overall': (d.get('gate') or {}).get('overall')}
                        selected = policy.kind != 'sarl' or policy.last_selected_agent == agent
                        if modes_after_act[agent] == 'CONTINGENCY':
                            measurement_ok = (boundary_ok(boundary_start, agent, start)
                                              and boundary_ok(probe.end_boundary, agent, end)
                                              and counters_ok(probe) and programs is not None)
                            fn = states[agent].finish_transition if agent in transitions else states[agent].finish_fixed_cycle
                            fn(complete=complete, gate_ok=gate_ok(d, policy, agent),
                               measurements_ok=measurement_ok,
                               pending_end=(probe.end_boundary or {}).get('pending_insertion'),
                               selected=selected)
                            if agent in transitions:
                                event('transition_finished' if states[agent].transition_completed else 'transition_cancelled',
                                      agent=agent, sim_time=end, recovery=states[agent].describe())
                        if agent in released:
                            event('handback_completed' if complete and programs else 'handback_unverified_partial',
                                  agent=agent, sim_time=end, program=(programs or {}).get(agent))
                            if complete and programs and agent == args.fault_agent:
                                summary['handback_completed'] = True
                                if args.fault_demo and injected_count == 1:
                                    arm_second = True
                    row = {'cycle': cycle, 'start_time': start, 'end_time': end,
                           'executed_steps': probe.steps, 'complete_cycle': complete,
                           'terminated': bool(terminated[first]),
                           'boundary_start': boundary_start, 'boundary_end': probe.end_boundary,
                           'step_counts': probe.totals, 'step_count_errors': probe.errors,
                           'junctions': junctions, 'sarl_joint_action': policy.last_joint_action,
                           'sarl_selected_agent': policy.last_selected_agent,
                           'observation_errors': policy.last_observation_errors,
                           'active_programs_before': programs_before, 'active_programs_after_first_step': programs,
                           'transition_agents': sorted(transitions), 'released_agents': sorted(released),
                           'recovery': {a: states[a].describe() for a in cfg.agents}}
                    history.append(row)
                    row['traffic_window'] = aggregate_window(history, cfg)
                    row['behavior'] = {a: behavior_summary(list(history), a, env.s_lb, env._split_ub())
                                       for a in cfg.agents}
                    row['slow_trial_evaluation'] = evaluate_experimental_rule(row, cfg)
                    can_continue = not terminated[first] and (args.max_cycles is None or cycle + 1 < args.max_cycles)
                    if args.enable_slow_trial and can_continue:
                        for agent, result in row['slow_trial_evaluation']['candidates'].items():
                            if result['candidate']:
                                slow_switches[agent] = {'evaluated_at': end,
                                    'pending_end': row['traffic_window']['pending_end'],
                                    'pending_change': row['traffic_window']['pending_change'],
                                    'behavior': dict(row['behavior'][agent]),
                                    'matched_patterns': [k for k in ('bound_pattern', 'alternating_split_pattern') if result[k]]}
                    row['slow_trial_scheduled_for_next_boundary'] = list(slow_switches)
                    log.write(json.dumps(row, allow_nan=False) + '\n')
                    log.flush()
                    cycle += 1
                    summary['cycles_observed'] = cycle
                    if complete:
                        summary['complete_cycles'] += 1
                    if cycle == 1 or cycle % 20 == 0 or transitions or released or fault_number:
                        print(f'cycle={cycle}, sim_t={end}, modes={ctrl.modes}, '
                              f'pending={(probe.end_boundary or {}).get("pending_insertion")}')
                    if terminated[first] or (args.max_cycles is not None and cycle >= args.max_cycles):
                        break
    except Exception:
        (out / 'run_error_GPT.txt').write_text(traceback.format_exc(), encoding='utf-8')
        raise
    finally:
        if env is not None:
            env.close()
        os.chdir(project)
    summary['fault_demo_complete'] = (args.fault_demo and all(summary[k] for k in (
        'first_fault_confirmed', 'first_fixed_program_confirmed', 'handback_completed',
        'second_fault_confirmed', 'second_fixed_program_confirmed')))
    (out / 'summary_GPT.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print('Trial finished:', json.dumps(summary))
    print(f'Output directory: {out}')


if __name__ == '__main__':
    main()
