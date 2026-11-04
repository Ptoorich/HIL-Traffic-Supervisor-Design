"""Controlled two-junction supervisor experiment; run from repository root.

Runs short clean and injected episodes on identical generated demand, then tests
an episode reset. All SUMO input copies and outputs go into a new run folder.
The injected fault exists only in a single immutable supervisor snapshot.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import sys
import traceback
import uuid
from dataclasses import replace
from pathlib import Path, PureWindowsPath
import xml.etree.ElementTree as ET

import traci
import tod_eval_common as ec
import tod_plan as tp
from sumo_env_marl_ps import SumoMARLEnvPS
from supervisor.config import load_config
from supervisor.conflicts import load_fixture
from supervisor.snapshot import SnapshotTaker
from supervisor.supervised_controller_GPT import SupervisedController

SEED = 42
CYCLES = 5
INJECT_CYCLE = 2  # zero based: t = 180 s
REQUIRED = ('2int.sumocfg', 'osm.net.xml.gz', 'routes.xml', 'osm.poly.xml.gz',
            'output.add.xml', 'Jorissen_model_two_intersection.dat.add.xml',
            'osm.view.xml')


class DeterministicController:
    name = 'deterministic_experiment'

    def act(self, env, obs, t):
        # Alternate a valid tl2 request. Keep tl1 different from its 31 s
        # contingency so a transition can be observed.
        tl2 = 39 if (int(t) // 90) % 2 == 0 else 44
        env.set_splits({'tl1': 39, 'tl2': tl2})
        return {'tl1': 1, 'tl2': 1}


class InjectingSnapshot:
    def __init__(self, cfg, enabled):
        self.real = SnapshotTaker(cfg)
        self.enabled = enabled
        self.count = 0
        self.injected_lane = None

    def clear_cache(self):
        self.real.clear_cache()
        self.count = 0
        self.injected_lane = None

    def take(self, env, current_splits=None):
        snapshots = self.real.take(env, current_splits=current_splits)
        if self.enabled and self.count == INJECT_CYCLE:
            snap = snapshots['tl1']
            if not snap.lanes:
                raise RuntimeError('tl1 snapshot unexpectedly has no lanes')
            self.injected_lane = snap.lanes[0].lane_id
            snapshots['tl1'] = replace(snap, lanes=snap.lanes[1:])
        self.count += 1
        return snapshots


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def active_program(env, agent):
    tl = env.tl_map[agent]
    active = traci.trafficlight.getProgram(tl)
    matches = [p for p in traci.trafficlight.getAllProgramLogics(tl)
               if p.programID == active]
    require(len(matches) == 1, f'{agent}: active program {active} not found uniquely')
    phases = matches[0].phases
    require(len(phases) == 6, f'{agent}: expected six phases, found {len(phases)}')
    return {'program': active, 'A': float(phases[0].duration),
            'B': float(phases[3].duration)}


def latest_log(path):
    with path.open(encoding='utf-8') as f:
        entries = [line for line in f if line.strip()]
    require(bool(entries), f'No supervisor decision in {path}')
    return json.loads(entries[-1])


def run_episode(env, cfg, matrix, route, output, name, inject):
    log = output / f'{name}_decisions.jsonl'
    trace_path = output / f'{name}_trace.jsonl'
    taker = InjectingSnapshot(cfg, inject)
    records = []
    with SupervisedController(DeterministicController(), cfg, matrix,
                              log_path=log, snapshot_taker=taker,
                              run_id=name) as ctrl:
        obs = env.reset(seed=SEED, route_file=str(route))
        ctrl.reset_episode(episode_id=f'{name}_{SEED}')
        require(ctrl.modes == {'tl1': 'ASTA', 'tl2': 'ASTA'}, 'Reset did not clear modes')
        for cycle in range(CYCLES):
            t = env.current_step
            require(t == cycle * 90, f'Unexpected time at cycle {cycle}: {t}')
            before = dict(env.phase_splits)
            actions = ctrl.act(env, obs, t)
            decision = latest_log(log)
            intended = dict(env.phase_splits)
            obs, _, terminated, = env.step(actions)
            require(not any(terminated.values()), f'SUMO ended before observation, cycle {cycle}')
            applied = dict(env.phase_splits)
            programs = {a: active_program(env, a) for a in ('tl1', 'tl2')}
            record = {'cycle': cycle, 'time': t, 'fault_injected': inject and cycle == INJECT_CYCLE,
                      'missing_lane': taker.injected_lane if inject and cycle == INJECT_CYCLE else None,
                      'before_A': before, 'returned_actions': actions,
                      'prepared_base_A': intended, 'post_step_A': applied,
                      'observed_SUMO_program': programs, 'decision': decision['decisions']}
            records.append(record)
            with trace_path.open('a', encoding='utf-8') as f:
                f.write(json.dumps(record) + '\n')
            print(f"{name}: cycle {cycle}, t={t}, fault={record['fault_injected']}, "
                  f"modes={ctrl.modes}, SUMO A/B="
                  f"{programs['tl1']['A']:g}/{programs['tl1']['B']:g}, "
                  f"{programs['tl2']['A']:g}/{programs['tl2']['B']:g}")

            d1, d2 = decision['decisions']['tl1'], decision['decisions']['tl2']
            require(d2['mode_after'] == 'ASTA', f'tl2 vetoed at cycle {cycle}: {d2}')
            expected2 = 39 if cycle % 2 == 0 else 44
            require(actions['tl2'] == 1 and applied['tl2'] == expected2,
                    f'tl2 expected {expected2}, got {applied["tl2"]}')
            require(programs['tl2']['A'] == expected2 and programs['tl2']['B'] == 78 - expected2,
                    f'tl2 SUMO program mismatch: {programs["tl2"]}')
            if inject and cycle >= INJECT_CYCLE:
                require(d1['mode_after'] == 'CONTINGENCY', f'tl1 not latched at cycle {cycle}')
                require(actions['tl1'] == 1 and applied['tl1'] == 31,
                        f'tl1 contingency not applied at cycle {cycle}')
                require(programs['tl1']['A'] == 31 and programs['tl1']['B'] == 47,
                        f'tl1 SUMO program mismatch: {programs["tl1"]}')
                if cycle == INJECT_CYCLE:
                    require(before['tl1'] == 39, 'tl1 was not at the pre-fault split')
                    require(d1['entered_contingency'], 'tl1 did not enter contingency')
                    gate = d1.get('gate') or {}
                    checks = gate.get('checks', [])
                    validator = next((x for x in checks if x['check'] == 'input_validator'), None)
                    require(validator is not None and validator['status'] == 'FAIL',
                            f'Input Validator did not fail: {validator}')
                    require(any(v.get('code') == 'LANE_MISSING' and v.get('lane') == taker.injected_lane
                                for v in validator['details'].get('violations', [])),
                            f'Expected injected LANE_MISSING not found: {validator}')
                else:
                    require(not d1['entered_contingency'], 'Latch entered again unexpectedly')
                    validator = next((x for x in (d1.get('gate') or {}).get('checks', [])
                                      if x['check'] == 'input_validator'), None)
                    require(validator is not None and validator['status'] != 'FAIL',
                            f'Validator did not recover after injection: {validator}')
            else:
                require(d1['mode_after'] == 'ASTA' and applied['tl1'] == 39,
                        f'Unexpected tl1 veto/split at cycle {cycle}: {d1}')
                require(programs['tl1']['A'] == 39 and programs['tl1']['B'] == 39,
                        f'tl1 SUMO program mismatch: {programs["tl1"]}')
        if inject:
            # Perform a real environment reset before clearing the latch.
            require(ctrl.modes['tl1'] == 'CONTINGENCY', 'Latch lost before reset')
            env.reset(seed=SEED, route_file=str(route))
            require(ctrl.modes['tl1'] == 'CONTINGENCY', 'Environment reset unexpectedly cleared latch')
            ctrl.reset_episode(episode_id='post_reset_42')
            require(ctrl.modes == {'tl1': 'ASTA', 'tl2': 'ASTA'}, 'Explicit reset did not clear latch')
            print('Explicit episode-boundary reset cleared the tl1 latch.')
    env.close()
    return records


def main():
    root = Path.cwd().resolve()
    source = root / '2int_files'
    missing = [str(source / x) for x in REQUIRED if not (source / x).is_file()]
    require(not missing, 'Missing input files: ' + ', '.join(missing))
    # Resolve config/fixture relative paths before moving SUMO into isolation.
    cfg = load_config()
    matrix = load_fixture(cfg.conflicts.fixture_path)
    run = root / 'supervisor_logs' / ('fault_experiment_GPT_' + uuid.uuid4().hex[:12])
    work = run / 'sumo_work'
    work.mkdir(parents=True, exist_ok=False)
    print(f'Experiment files: {run}')
    for filename in REQUIRED:
        shutil.copy2(source / filename, work / filename)
    # Reject file attributes that could direct additional-file outputs outside
    # the copied working tree. External inputs must be reviewed separately.
    for filename in ('output.add.xml', 'Jorissen_model_two_intersection.dat.add.xml'):
        for element in ET.parse(work / filename).iter():
            value = element.attrib.get('file')
            if value:
                for item in value.split(','):
                    path = PureWindowsPath(item.strip())
                    require(not path.is_absolute() and not path.drive and '..' not in path.parts,
                            f'Unsafe file path in {filename}: {value}')
    # Work on copied inputs. Any SUMO-generated relative outputs remain here.
    os.chdir(work)
    config = ec.make_config()
    config['sumocfg'] = str(work / '2int.sumocfg')
    env = SumoMARLEnvPS(config)
    mix = tp.load_route_mix(str(work / 'routes.xml'))
    route = work / 'routes_experiment_GPT.xml'
    tp.generate_route_file(str(route), mix, SEED, ec.DEFAULT_SCALE * tp.ORIGINAL_RATE_VPH)
    route_hash = hashlib.sha256(route.read_bytes()).hexdigest()
    metadata = {'seed': SEED, 'route_sha256': route_hash, 'cycles_per_run': CYCLES,
                'fault_cycle': INJECT_CYCLE, 'fault': 'one tl1 snapshot missing one lane',
                'purpose': 'integration demonstration; not a hardware safety validation'}
    (run / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    try:
        run_episode(env, cfg, matrix, route, run, 'clean', False)
        run_episode(env, cfg, matrix, route, run, 'injected', True)
    finally:
        env.close()
    print('\nPASS: clean control, injected veto, applied SUMO programs, latch and episode reset.')
    print(f'Route SHA-256: {route_hash}')
    print(f'Please share clean_trace.jsonl and injected_trace.jsonl from: {run}')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'\nEXPERIMENT NOT VERIFIED: {type(exc).__name__}: {exc}', file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)
