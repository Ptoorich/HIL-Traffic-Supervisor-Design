"""Ten-cycle MARL PPO HIL test: one TL1 snapshot fault, FTSC, gated handback.

Run from the project root. The fault removes one lane reading from one copied
supervisor snapshot at t=90; it does not change SUMO's lanes or the RL input.
"""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import traceback

import traci
import tod_eval_common as ec
import tod_plan as tp
from sumo_env_marl_ps import SumoMARLEnvPS
from supervisor.config import load_config
from supervisor.conflicts import load_fixture
from supervisor.hil_display_GPT import PhaseDisplay
from supervisor.hil_serial_GPT import SerialDisplay
from supervisor.integrated_runner_GPT import (
    isolate_scenario, run_episode, sha256, validate_model,
)
from supervisor.observation_adapter_GPT import StrictPolicy
from supervisor.snapshot import SnapshotTaker
from supervisor.supervised_integrated_GPT import IntegratedSupervisedController

SEED, CYCLES, FAULT_TIME = 42, 10, 90


class OneShotFault(SnapshotTaker):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.injection = None

    def clear_cache(self):
        super().clear_cache()
        self.injection = None

    def take(self, env, current_splits=None):
        snapshots = super().take(env, current_splits=current_splits)
        snap = snapshots['tl1']
        if snap.sim_time == FAULT_TIME and self.injection is None:
            lane = snap.lanes[0].lane_id
            snapshots['tl1'] = replace(snap, lanes=snap.lanes[1:])
            self.injection = {'agent': 'tl1', 'sim_time': FAULT_TIME,
                              'missing_lane': lane, 'snapshot_count': 1}
        return snapshots


class ProgressDisplay(PhaseDisplay):
    previous_modes = None

    def sample(self, env, traci_mod, modes, transitions=()):
        super().sample(env, traci_mod, modes, transitions)
        labels = tuple('FTSC transition' if a in transitions else
                       ('FTSC' if modes[a] == 'CONTINGENCY' else 'ASTA')
                       for a in env.agent_ids)
        if labels != self.previous_modes:
            print(f't={self.last_time:g}s: TL1={labels[0]}, TL2={labels[1]}', flush=True)
            self.previous_modes = labels


def read_rows(out, name):
    return [json.loads(line) for line in (out / name).read_text(
        encoding='utf-8').splitlines() if line.strip()]


def verify(out, summary, injection):
    rows = read_rows(out, 'display_GPT.jsonl')
    samples = [r for r in rows if r['event'] == 'sample']
    changes = [s for i, s in enumerate(samples)
               if i == 0 or s['modes'] != samples[i - 1]['modes']]
    fixed = [s for s in samples if s['modes'][0] in (1, 2)]
    events = read_rows(out, 'events_GPT.jsonl')
    transitions = [e for e in events if e['event'] == 'transition_started']
    missing = []
    for row in read_rows(out, 'decisions_GPT.jsonl'):
        for agent, decision in row['decisions'].items():
            for check in (decision.get('gate') or {}).get('checks', []):
                if check['check'] == 'input_validator' and check['status'] == 'FAIL':
                    for item in check.get('details', {}).get('violations', []):
                        if item['code'] == 'LANE_MISSING':
                            missing.append((row['sim_time'], agent, item['lane']))
    expected = [(FAULT_TIME, 'tl1', injection['missing_lane'])] if injection else []
    checks = {
        'one_injected_lane_rejected': injection is not None and missing == expected,
        'ten_complete_cycles': summary['complete_cycles'] == CYCLES,
        'one_tl1_fallback': summary['fast_contingencies'] == {'tl1': 1, 'tl2': 0},
        'no_slow_interventions': summary['slow_contingencies'] == {'tl1': 0, 'tl2': 0},
        'one_verified_handback': summary['handbacks_completed'] == {'tl1': 1, 'tl2': 0}
            and summary['handbacks_unverified_partial'] == {'tl1': 0, 'tl2': 0},
        'both_finish_asta': summary['final_modes'] == {'tl1': 'ASTA', 'tl2': 'ASTA'},
        'display_mode_sequence': [s['modes'] for s in changes]
            == [[0, 0], [1, 0], [2, 0], [0, 0]] and changes[1]['sim_time'] == FAULT_TIME,
        'every_second_and_boundary': [s['sim_time'] for s in samples]
            == [c * 90 + t for c in range(CYCLES) for t in range(91)],
        'all_six_phases': all({s['phases'][i] for s in samples} == set(range(6))
                              for i in (0, 1)),
        'tl1_fixed_program': bool(fixed) and all(
            s['programs']['tl1']['durations'] == [31, 3, 3, 47, 3, 3] for s in fixed),
        'qualified_transition': len(transitions) == 1 and transitions[0]['agent'] == 'tl1'
            and transitions[0]['recovery']['ready']
            and transitions[0]['recovery']['dwell_cycles'] >= 5
            and transitions[0]['recovery']['clean_cycles'] >= 3,
        'normal_stop_ack': bool(rows) and rows[-1]['event'] == 'ack'
            and rows[-1].get('kind') == 'X' and not any(r['event'] == 'fault' for r in rows),
    }
    return {'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks,
            'injection': injection, 'sample_count': len(samples),
            'mode_changes': [{'sim_time': s['sim_time'], 'modes': s['modes']} for s in changes],
            'scope': 'SUMO readback and GPIO ACK; visually check the actual LEDs'}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--speed', type=float, choices=[1, 3], default=1)
    p.add_argument('--gui', action='store_true')
    p.add_argument('--run-dir', required=True, help='new output directory')
    p.add_argument('--model', default='models/marl_ppo_ps_2int_260k.zip')
    args = p.parse_args(argv)
    project = Path.cwd().resolve()
    if project != Path(__file__).resolve().parent:
        p.error('Run from the project root, alongside this script')
    if not args.port.strip():
        p.error('--port cannot be empty')
    root = Path(args.run_dir).resolve()
    if root.is_relative_to(project / '2int_files'):
        p.error('Place --run-dir outside 2int_files')
    if root.exists():
        p.error('--run-dir already exists; choose a fresh name')
    from stable_baselines3 import PPO
    model_path = Path(args.model).resolve()
    model = PPO.load(model_path, device='cpu')
    config, cfg = ec.make_config(), load_config()
    validate_model(model, 'marl', config)
    matrix = load_fixture(cfg.conflicts.fixture_path)
    root.mkdir(parents=True, exist_ok=False)
    out = root / f'seed_{SEED}'
    out.mkdir()
    env = None
    injector = OneShotFault(cfg)
    try:
        route = out / f'routes_seed{SEED}_GPT.xml'
        mix = tp.load_route_mix(str(project / '2int_files/routes.xml'))
        tp.generate_route_file(str(route), mix, SEED, ec.DEFAULT_SCALE * tp.ORIGINAL_RATE_VPH)
        work, copied = isolate_scenario(project, out, gui=args.gui)
        config.update(sumocfg=str(copied), use_gui=args.gui)
        metadata = {'test': 'one_tl1_missing_lane', 'seed': SEED, 'cycles': CYCLES,
                    'fault_time': FAULT_TIME, 'slow_monitor_enabled': False,
                    'auto_handback_enabled': True, 'sumo_gui': args.gui,
                    'port': args.port, 'speed': args.speed, 'model_sha256': sha256(model_path),
                    'route_sha256': sha256(route), 'runner_sha256': sha256(Path(__file__)),
                    'fault_scope': 'one supervisor snapshot; SUMO and RL inputs unchanged'}
        (out / 'metadata_GPT.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
        print('Fault test: 10 cycles; one TL1 missing-lane snapshot at t=90s.', flush=True)
        print(f'Allow about {900 / args.speed / 60:g} minutes plus overhead.', flush=True)
        print(f'Output directory: {out}', flush=True)
        os.chdir(work)
        env = SumoMARLEnvPS(config)
        policy = StrictPolicy('marl', model, traci)

        def factory(inner, cfg, matrix, **kwargs):
            return IntegratedSupervisedController(inner, cfg, matrix,
                snapshot_taker=injector, **kwargs)

        with SerialDisplay(args.port, log_path=out / 'display_GPT.jsonl') as transport:
            display = ProgressDisplay(transport, speed=args.speed)
            summary = run_episode(env, policy, cfg, matrix, SEED, route, out,
                slow_enabled=False, handback_enabled=True, max_cycles=CYCLES,
                controller_factory=factory, display=display)
        (out / 'summary_GPT.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        result = verify(out, summary, injector.injection)
        (out / 'fault_test_result_GPT.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        print('Finished:', json.dumps(summary), flush=True)
        print('Fault test software checks:', result['status'], flush=True)
        for name, passed in result['checks'].items():
            if not passed:
                print('FAILED:', name, flush=True)
        print(f'Result: {out / "fault_test_result_GPT.json"}', flush=True)
        return 0 if result['status'] == 'PASS' else 1
    except BaseException:
        (out / 'run_error_GPT.txt').write_text(traceback.format_exc(), encoding='utf-8')
        raise
    finally:
        try:
            if env is not None:
                env.close()
        finally:
            os.chdir(project)


if __name__ == '__main__':
    raise SystemExit(main())