"""Opt-in Slow Monitor and handback path for the supervised SUMO runner.

The thresholds are experimental. This is a simulation workflow, not a
validated physical traffic signal controller. The legacy path stays separate.
"""
from __future__ import annotations

from collections import deque
import gzip
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
from sumo_env_marl_ps import SumoMARLEnvPS

from .observation_adapter_GPT import StrictPolicy, read_active_programs
from .recovery_GPT import Recovery, valid_pending
from .slow_monitor_GPT import (StepProbe, aggregate_window, behavior_summary,
                               capture_boundary, evaluate_experimental_rule)
from .supervised_integrated_GPT import IntegratedSupervisedController


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _contained_path(base, root, value):
    if not value or value in ('-', 'stdout', 'NUL', '/dev/null'):
        return None
    path = (base / value).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f'SUMO file reference escapes isolated work folder: {value!r}')
    return path


def isolate_scenario(project, seed_dir):
    """Copy the scenario and refuse explicit references outside the run copy."""
    source = project / '2int_files'
    if seed_dir.is_relative_to(source.resolve()):
        raise ValueError('Place --run-dir outside 2int_files')
    work = seed_dir / 'sumo_work'
    work.mkdir()
    shutil.copytree(source, work / '2int_files')
    copied_config = work / '2int_files' / '2int.sumocfg'
    root = ET.parse(copied_config).getroot()
    additional = []
    input_node, output_node = root.find('input'), root.find('output')
    if input_node is not None:
        for node in input_node:
            for part in node.get('value', '').split(','):
                path = _contained_path(copied_config.parent, work, part.strip())
                if node.tag == 'additional-files' and path is not None:
                    additional.append(path)
    if output_node is not None:
        for node in output_node:
            _contained_path(copied_config.parent, work, node.get('value', ''))
    for path in additional:
        if not path.exists():
            raise FileNotFoundError(f'Missing copied SUMO additional file: {path}')
        opener = gzip.open if path.suffix == '.gz' else open
        with opener(path, 'rb') as stream:
            root = ET.parse(stream).getroot()
        for node in root.iter():
            if node.get('file'):
                _contained_path(path.parent, work, node.get('file'))
    return work, copied_config


def validate_model(model, kind, config):
    shape = (max(len(v) for v in config['lane_map'].values()) + 2 if kind == 'marl'
             else sum(len(v) for v in config['lane_map'].values()) + len(config['agent_ids']))
    n_actions = 3 if kind == 'marl' else 3 * len(config['agent_ids'])
    if model.observation_space.shape != (shape,) or getattr(model.action_space, 'n', None) != n_actions:
        raise ValueError(f'Unexpected {kind.upper()} model spaces: {model.observation_space}, '
                         f'{model.action_space}; expected ({shape},), Discrete({n_actions})')


def boundary_ok(boundary, agent, expected_time):
    if not isinstance(boundary, dict) or not valid_pending(boundary.get('pending_insertion')):
        return False
    try:
        tm = float(boundary['sim_time'])
        fill = float(boundary['worst_lane_fill'][agent]['fill'])
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


class EvidenceProbe(StepProbe):
    """Use the trial's scoped step probe and check the active SUMO programme."""
    def __init__(self, cfg, env, traci_mod):
        super().__init__(cfg, env.step_length, traci_mod)
        self.env, self.active_programs = env, None

    def step(self, *args, **kwargs):
        result = super().step(*args, **kwargs)
        if self.steps == 1:
            self.active_programs = read_active_programs(self.env, self.traci)
        return result


def run_episode(env, policy, cfg, matrix, seed, route, out, *, slow_enabled,
                handback_enabled, max_cycles=None, traci_mod=traci,
                controller_factory=None):
    """Run one episode. An injectable controller factory supports mock tests."""
    if controller_factory is None:
        controller_factory = IntegratedSupervisedController
    agents = tuple(cfg.agents)
    recovery = {a: Recovery(is_sarl=policy.kind == 'sarl') for a in agents}
    history, slow_switches = deque(maxlen=10), {}
    summary = {'seed': seed, 'slow_monitor_enabled': slow_enabled,
               'auto_handback_enabled': handback_enabled,
               'cycles_observed': 0, 'complete_cycles': 0,
               'slow_contingencies': {a: 0 for a in agents},
               'fast_contingencies': {a: 0 for a in agents},
               'handbacks_completed': {a: 0 for a in agents},
               'handbacks_unverified_partial': {a: 0 for a in agents},
               'final_modes': None}
    decisions = out / 'decisions_GPT.jsonl'
    with controller_factory(policy, cfg, matrix, log_path=decisions,
                            run_id=f'integrated_{policy.kind}') as ctrl:
        obs = env.reset(seed=seed, route_file=str(route))
        ctrl.reset_episode(episode_id=seed)
        with (out/'observations_GPT.jsonl').open('x', encoding='utf-8') as log, \
             (out/'events_GPT.jsonl').open('x', encoding='utf-8') as events:
            def event(name, **fields):
                events.write(json.dumps({'event': name, **fields}, allow_nan=False) + '\n')
                events.flush()

            cycle = 0
            while True:
                start = env.current_step
                before = {a: float(env.phase_splits[a]) for a in agents}
                programs_before = read_active_programs(env, traci_mod)
                boundary_start = capture_boundary(cfg, traci_mod)
                entered_boundary = {}
                for agent, evidence in slow_switches.items():
                    if ctrl.enter_contingency(agent, reason='slow_monitor'):
                        entered_boundary[agent] = 'slow_monitor'
                        if handback_enabled:
                            recovery[agent].enter('slow_monitor', start)
                        summary['slow_contingencies'][agent] += 1
                        event('slow_contingency_set', agent=agent, sim_time=start, evidence=evidence)
                slow_switches = {}

                release_candidates = set()
                if handback_enabled:
                    for agent in agents:
                        state = recovery[agent]
                        if not state.active or not state.transition_completed:
                            continue
                        pending = boundary_start.get('pending_insertion')
                        can_release = (ctrl.modes[agent] == 'CONTINGENCY'
                                       and boundary_ok(boundary_start, agent, start)
                                       and math.isclose(before[agent], float(cfg.contingency[agent]), abs_tol=1e-6)
                                       and ('slow_monitor' not in state.causes
                                            or (valid_pending(pending) and pending < state.low_pending_limit)))
                        if can_release:
                            release_candidates.add(agent)
                        else:
                            state.invalidate()
                            event('handback_cancelled', agent=agent, sim_time=start,
                                  reason='unavailable or changed boundary evidence')

                actions = ctrl.act(env, obs, start,
                                   release_candidates=release_candidates if handback_enabled else None)
                entry = ctrl.last_entry
                if not isinstance(entry, dict) or set(entry.get('decisions', {})) != set(agents):
                    raise RuntimeError('Incomplete controller decision; no simulation step performed')
                decisions_by_agent = entry['decisions']
                transitions, released = set(), set()
                for agent in agents:
                    decision, state = decisions_by_agent[agent], recovery[agent]
                    approved = gate_ok(decision, policy, agent)
                    if agent in release_candidates:
                        if approved and decision.get('handback_accepted') and ctrl.modes[agent] == 'ASTA':
                            released.add(agent)
                            state.leave()
                        else:
                            if ctrl.modes[agent] != 'CONTINGENCY':
                                raise RuntimeError('A rejected handback did not retain contingency')
                            state.enter('fast_gate', start, restart=True)
                            event('handback_vetoed', agent=agent, sim_time=start, reason=decision.get('reason'))
                    if ctrl.modes[agent] == 'CONTINGENCY':
                        if decision.get('entered_contingency'):
                            entered_boundary[agent] = 'fast_gate'
                            summary['fast_contingencies'][agent] += 1
                            event('fast_contingency_entered', agent=agent, sim_time=start,
                                  reason=decision.get('reason'))
                        if handback_enabled:
                            if not state.active:
                                state.enter('fast_gate', start)
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
                with EvidenceProbe(cfg, env, traci_mod) as probe:
                    obs, _, terminated = env.step(actions)
                complete = probe.steps == env.step_length
                end = start + probe.steps
                programs = probe.active_programs
                if probe.steps and programs is None:
                    raise RuntimeError('Active SUMO programme readback was not captured')
                for agent in agents:
                    if (modes_after_act[agent] == 'CONTINGENCY' and programs
                            and not math.isclose(programs[agent]['green_a'], float(cfg.contingency[agent]), abs_tol=1e-6)):
                        raise RuntimeError(f'{agent}: active programme does not match contingency split')
                for agent, reason in entered_boundary.items():
                    if programs is not None and modes_after_act[agent] == 'CONTINGENCY':
                        event('contingency_program_observed', agent=agent, source=reason,
                              sim_time=start + 1, complete_cycle=complete, program=programs[agent])
                junctions = {}
                for agent in agents:
                    decision = decisions_by_agent[agent]
                    junctions[agent] = {'split_before': before[agent],
                        'proposed_action': decision.get('proposed_action'),
                        'requested_split': (decision.get('proposal') or {}).get('requested_split'),
                        'returned_action': actions.get(agent), 'split_after': float(env.phase_splits[agent]),
                        'mode_before': decision.get('mode_before'), 'mode_after': decision.get('mode_after'),
                        'handback_accepted': bool(decision.get('handback_accepted', False)),
                        'gate_overall': (decision.get('gate') or {}).get('overall')}
                    selected = policy.kind != 'sarl' or policy.last_selected_agent == agent
                    if handback_enabled and modes_after_act[agent] == 'CONTINGENCY':
                        measurements = (boundary_ok(boundary_start, agent, start)
                                        and boundary_ok(probe.end_boundary, agent, end)
                                        and counters_ok(probe) and programs is not None)
                        fn = recovery[agent].finish_transition if agent in transitions else recovery[agent].finish_fixed_cycle
                        fn(complete=complete, gate_ok=gate_ok(decision, policy, agent),
                           measurements_ok=measurements,
                           pending_end=(probe.end_boundary or {}).get('pending_insertion'),
                           selected=selected)
                        if agent in transitions:
                            name = 'transition_finished' if recovery[agent].transition_completed else 'transition_cancelled'
                            event(name, agent=agent, sim_time=end, recovery=recovery[agent].describe())
                    if agent in released:
                        verified = complete and programs is not None
                        field = 'handbacks_completed' if verified else 'handbacks_unverified_partial'
                        summary[field][agent] += 1
                        event('handback_completed' if verified else 'handback_unverified_partial',
                              agent=agent, sim_time=end, program=(programs or {}).get(agent))
                row = {'cycle': cycle, 'start_time': start, 'end_time': end,
                       'executed_steps': probe.steps, 'complete_cycle': complete,
                       'terminated': bool(terminated[agents[0]]),
                       'boundary_start': boundary_start, 'boundary_end': probe.end_boundary,
                       'step_counts': probe.totals, 'step_count_errors': probe.errors,
                       'junctions': junctions, 'sarl_joint_action': policy.last_joint_action,
                       'sarl_selected_agent': policy.last_selected_agent,
                       'observation_errors': policy.last_observation_errors,
                       'active_programs_before': programs_before,
                       'active_programs_after_first_step': programs,
                       'transition_agents': sorted(transitions), 'released_agents': sorted(released),
                       'recovery': ({a: recovery[a].describe() for a in agents} if handback_enabled else None)}
                history.append(row)
                row['traffic_window'] = aggregate_window(history, cfg)
                row['behavior'] = {a: behavior_summary(list(history), a, env.s_lb, env._split_ub())
                                   for a in agents}
                row['slow_monitor_evaluation'] = (evaluate_experimental_rule(row, cfg) if slow_enabled
                                                  else {'status':'disabled', 'candidates':{}})
                can_continue = not terminated[agents[0]] and (max_cycles is None or cycle + 1 < max_cycles)
                if slow_enabled and can_continue:
                    for agent, result in row['slow_monitor_evaluation']['candidates'].items():
                        if result['candidate']:
                            slow_switches[agent] = {'evaluated_at': end,
                                'pending_end': row['traffic_window']['pending_end'],
                                'pending_change': row['traffic_window']['pending_change'],
                                'behavior': dict(row['behavior'][agent]),
                                'matched_patterns': [k for k in ('bound_pattern', 'alternating_split_pattern') if result[k]]}
                row['slow_monitor_scheduled_for_next_boundary'] = list(slow_switches)
                log.write(json.dumps(row, allow_nan=False) + '\n')
                log.flush()
                cycle += 1
                summary['cycles_observed'] = cycle
                summary['complete_cycles'] += int(complete)
                summary['final_modes'] = dict(ctrl.modes)
                if cycle == 1 or cycle % 20 == 0 or transitions or released:
                    print(f'seed={seed}, cycle={cycle}, sim_t={end}, modes={ctrl.modes}, '
                          f'pending={(probe.end_boundary or {}).get("pending_insertion")}')
                if not can_continue:
                    break
    return summary


def run_integrated(*, config, cfg, matrix, model, kind, model_path, name, seeds,
                   mix, peak, run_dir, slow_enabled, handback_enabled, max_cycles=None):
    """Run independent isolated seed directories; neither flag changes defaults."""
    project = Path.cwd().resolve()
    root = Path(run_dir).resolve()
    if root.is_relative_to((project/'2int_files').resolve()):
        raise ValueError('Place --run-dir outside 2int_files')
    validate_model(model, kind, config)
    root.mkdir(parents=True, exist_ok=False)
    original_cwd = Path.cwd()
    try:
        for seed in seeds:
            out = root / f'seed_{seed}'
            out.mkdir()
            env = None
            try:
                route = out / f'routes_seed{seed}_GPT.xml'
                tp.generate_route_file(str(route), mix, seed, peak)
                work, copied_config = isolate_scenario(project, out)
                seed_config = dict(config)
                seed_config['sumocfg'] = str(copied_config)
                meta = {'controller': name, 'kind': kind, 'seed': seed,
                        'peak_rate_vph': peak, 'route_sha256': sha256(route),
                        'model_sha256': sha256(model_path),
                        'slow_monitor_enabled': slow_enabled, 'auto_handback_enabled': handback_enabled,
                        'parameters': {'traffic_window_cycles': 3, 'slow_backlog_min': 150,
                                       'slow_backlog_growth_min': 30, 'bound_cycles': 5,
                                       'alternating_split_cycles': 6, 'fixed_dwell_cycles': 5,
                                       'clean_cycles': 3, 'sarl_selections': 3,
                                       'sarl_selection_window': 10, 'slow_backlog_below': 100,
                                       'slow_backlog_cycles': 3, 'transition_cycles': 1},
                        'scope': 'experimental simulation; not hardware validation'}
                (out/'metadata_GPT.json').write_text(json.dumps(meta, indent=2), encoding='utf-8')
                os.chdir(work)
                env = SumoMARLEnvPS(seed_config)
                policy = StrictPolicy(kind, model, traci)
                summary = run_episode(env, policy, cfg, matrix, seed, route, out,
                                      slow_enabled=slow_enabled, handback_enabled=handback_enabled,
                                      max_cycles=max_cycles)
                (out/'summary_GPT.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
                print('Finished:', json.dumps(summary))
                print(f'Output directory: {out}')
            except Exception:
                (out/'run_error_GPT.txt').write_text(traceback.format_exc(), encoding='utf-8')
                raise
            finally:
                if env is not None:
                    env.close()
                os.chdir(original_cwd)
    finally:
        os.chdir(original_cwd)
