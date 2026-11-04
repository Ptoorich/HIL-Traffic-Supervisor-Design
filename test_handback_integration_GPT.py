"""Mock-SUMO integration through the real existing wrapper and Fast Gate.

Run from the project root: python -m unittest -v test_handback_integration_GPT
This is not a substitute for running SUMO.
"""
from __future__ import annotations

import importlib
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

from supervisor.config import load_config
from supervisor.conflicts import ConflictMatrix, JunctionConflicts
from supervisor.supervised_controller_GPT import SupervisedController
from supervisor.tests.helpers import COMMANDED, healthy_snapshot


class HandbackLoopTest(unittest.TestCase):
    def test_fault_recovery_handback_and_new_fault_with_real_gate(self):
        cfg = load_config()
        foes = frozenset({(0, 2), (0, 3), (1, 2), (1, 3)})
        matrix = ConflictMatrix({a: JunctionConflicts(f'tl_{a}', 4, foes, COMMANDED)
                                 for a in cfg.agents})
        fake = types.ModuleType('traci')
        fake.env = None

        class Simulation:
            def __init__(self): self.now = 0
            def getTime(self): return self.now
            def getPendingVehicles(self): return []
            def getDepartedNumber(self): return 0
            def getArrivedNumber(self): return 0
            def getStartingTeleportNumber(self): return 0
        fake.simulation = Simulation()
        fake.lane = types.SimpleNamespace(getLastStepHaltingNumber=lambda lane: 0)
        def simulation_step(): fake.simulation.now += 1
        fake.simulationStep = simulation_step

        class Lights:
            def getProgram(self, tl_id):
                agent = next(a for a in cfg.agents if fake.env.tl_map[a] == tl_id)
                return f'tod6_{fake.env.phase_splits[agent]:g}'
            def getAllProgramLogics(self, tl_id):
                agent = next(a for a in cfg.agents if fake.env.tl_map[a] == tl_id)
                env = fake.env; a = env.phase_splits[agent]
                durations = [a, 3, 3, 78-a, 3, 3]
                states = ['GGrr', 'yyrr', 'rrrr', 'rrGG', 'rryy', 'rrrr']
                phases = [types.SimpleNamespace(duration=d, state=s) for d, s in zip(durations, states)]
                return [types.SimpleNamespace(programID=self.getProgram(tl_id), phases=phases)]
        fake.trafficlight = Lights()

        class FakeEnv:
            def __init__(self, config):
                fake.env = self
                self.agent_ids = list(cfg.agents)
                self.lane_map = {a: [lc.lane_id for lc in cfg.lanes_of(a)] for a in cfg.agents}
                self.tl_map = {a: f'tl_{a}' for a in cfg.agents}
                self.phase_splits = {a: 39.0 for a in cfg.agents}
                self.agent_index = {'tl1': 0.0, 'tl2': 0.5}
                self.max_local_obs = max(len(x) + 1 for x in self.lane_map.values())
                self.obs_size = self.max_local_obs + 1
                self.s_lb, self.s_ub = 15, 63
                self.cycle_length, self.yellow_time, self.allred_time = 90, 3, 3
                self.allred_phases, self.step_length, self.current_step = True, 90, 0
                self._base_states = {a: COMMANDED for a in cfg.agents}
            def set_splits(self, splits): self.phase_splits.update(splits)
            def _split_ub(self): return 63
            def reset(self, seed=None, route_file=None):
                fake.simulation.now = self.current_step = 0
                return {}
            def step(self, actions):
                for a, action in actions.items():
                    self.phase_splits[a] = max(15, min(63, self.phase_splits[a] + {0:-5, 1:0, 2:5}[action]))
                for _ in range(self.step_length): fake.simulationStep()
                self.current_step = fake.simulation.now
                return {}, {}, {a: False for a in cfg.agents}
            def close(self): pass

        class FakeTaker:
            def clear_cache(self): pass
            def take(self, env, current_splits=None):
                return {a: healthy_snapshot(cfg, a, split=current_splits[a], sim_time=fake.simulation.now)
                        for a in cfg.agents}

        def make_controller(inner, config, conflict_matrix, **kwargs):
            return SupervisedController(inner, config, conflict_matrix,
                                        snapshot_taker=FakeTaker(), **kwargs)

        plan = types.ModuleType('tod_plan')
        plan.ORIGINAL_RATE_VPH = 1000
        plan.load_route_mix = lambda path: {}
        plan.generate_route_file = lambda path, mix, seed, peak: Path(path).write_text('<routes/>')
        common = types.ModuleType('tod_eval_common')
        common.DEFAULT_SCALE = .37
        common.make_config = lambda: {'agent_ids': list(cfg.agents),
                                     'lane_map': {a:[x.lane_id for x in cfg.lanes_of(a)] for a in cfg.agents}}
        env_module = types.ModuleType('sumo_env_marl_ps'); env_module.SumoMARLEnvPS = FakeEnv
        modules = {'traci': fake, 'tod_plan': plan, 'tod_eval_common': common,
                   'sumo_env_marl_ps': env_module}
        old_cwd = Path.cwd()
        with tempfile.TemporaryDirectory() as temp, mock.patch.dict(sys.modules, modules):
            project = Path(temp)
            inputs = project / '2int_files'; inputs.mkdir()
            (inputs / 'routes.xml').write_text('<routes/>')
            (inputs / '2int.sumocfg').write_text(
                '<sumoConfiguration><input><net-file value="net.xml"/>'
                '<route-files value="routes.xml"/></input>'
                '<output><statistic-output value="stats.xml"/></output></sumoConfiguration>')
            args = types.SimpleNamespace(controller='demo', model=None, output_dir='run',
                enable_handback_trial=True, enable_slow_trial=False, fault_demo=True,
                fault_agent='tl1', fault_at_cycle=2, max_cycles=13, seed=42, peak_scale=.37)
            try:
                os.chdir(project)
                sys.modules.pop('run_handback_trial_GPT', None)
                runner = importlib.import_module('run_handback_trial_GPT')
                # Line-ending differences are accepted; actual source changes are not.
                tracked = project / 'tracked.py'
                tracked.write_bytes(b'a = 1\r\n')
                manifest = {'files': {'tracked.py': hashlib.sha256(b'a = 1\n').hexdigest()}}
                (project / 'source_fingerprints_GPT.json').write_text(json.dumps(manifest))
                with mock.patch.object(runner, '__file__', str(project/'runner.py')):
                    self.assertEqual(runner.verify_compatible_sources(project), manifest['files'])
                    tracked.write_text('a = 2\n')
                    with self.assertRaises(RuntimeError): runner.verify_compatible_sources(project)
                # SUMO configuration that writes outside the isolated run is rejected.
                original_cfg = (inputs/'2int.sumocfg').read_text()
                (inputs/'2int.sumocfg').write_text(original_cfg.replace('stats.xml', '../../../outside.xml'))
                unsafe = project/'unsafe'; unsafe.mkdir()
                with self.assertRaises(ValueError): runner.isolate_scenario(project, unsafe)
                self.assertFalse((project/'outside.xml').exists())
                (inputs/'2int.sumocfg').write_text(original_cfg)
                with mock.patch.object(runner, 'parse_args', return_value=args), \
                     mock.patch.object(runner, 'verify_compatible_sources', return_value={}), \
                     mock.patch.object(runner, 'load_config', return_value=cfg), \
                     mock.patch.object(runner, 'load_fixture', return_value=matrix), \
                     mock.patch.object(runner, 'SupervisedController', side_effect=make_controller):
                    runner.main()
                summary = json.loads((project/'run'/'summary_GPT.json').read_text())
                self.assertTrue(summary['fault_demo_complete'], summary)
                events = [json.loads(x) for x in (project/'run'/'handback_events_GPT.jsonl').read_text().splitlines()]
                names = [e['event'] for e in events]
                self.assertLess(names.index('synthetic_fault_confirmed'), names.index('handback_completed'))
                self.assertEqual(names.count('synthetic_fault_confirmed'), 2)
                self.assertEqual(next(e for e in events if e['event']=='handback_completed')['program']['green_a'], 36)
                rows = [json.loads(x) for x in (project/'run'/'handback_observations_GPT.jsonl').read_text().splitlines()]
                self.assertTrue(all(row['junctions']['tl2']['mode_after'] == 'ASTA' for row in rows))
                self.assertFalse((project/'2int_files'/'stats.xml').exists())

                # Simulate an observation failure precisely at the release boundary.
                # End and start boundary probes succeed; the fresh policy read then fails.
                target_lane = cfg.lanes_of('tl1')[0].lane_id
                calls = {'at_release': 0, 'failed': False}
                def transient_queue(lane):
                    if fake.simulation.now == 720 and lane == target_lane:
                        calls['at_release'] += 1
                        if calls['at_release'] == 3 and not calls['failed']:
                            calls['failed'] = True
                            raise RuntimeError('injected observation failure')
                    return 0
                fake.lane.getLastStepHaltingNumber = transient_queue
                args.output_dir, args.max_cycles = 'run_observation_failure', 23
                with mock.patch.object(runner, 'parse_args', return_value=args), \
                     mock.patch.object(runner, 'verify_compatible_sources', return_value={}), \
                     mock.patch.object(runner, 'load_config', return_value=cfg), \
                     mock.patch.object(runner, 'load_fixture', return_value=matrix), \
                     mock.patch.object(runner, 'SupervisedController', side_effect=make_controller):
                    runner.main()
                self.assertTrue(calls['failed'])
                retry_summary = json.loads((project/'run_observation_failure'/'summary_GPT.json').read_text())
                self.assertTrue(retry_summary['fault_demo_complete'], retry_summary)
                retry_events = [json.loads(x) for x in (project/'run_observation_failure'/'handback_events_GPT.jsonl').read_text().splitlines()]
                veto = next(e for e in retry_events if e['event'] == 'handback_vetoed')
                self.assertEqual(veto['sim_time'], 720)
                self.assertEqual(veto['reason'], 'INVALID_PROPOSAL')
                completed = next(e for e in retry_events if e['event'] == 'handback_completed')
                self.assertGreater(completed['sim_time'], 810)

                # A SARL policy that always selects tl2 cannot manufacture tl1
                # recovery samples from tl1's adapter no-ops.
                fake.lane.getLastStepHaltingNumber = lambda lane: 0
                class AlwaysOther:
                    def predict(self, obs, deterministic=True): return 3, None
                args.controller, args.output_dir, args.max_cycles = 'sarl-dqn', 'run_sarl_other', 15
                sarl_policy = runner.StrictPolicy('sarl', AlwaysOther(), fake)
                with mock.patch.object(runner, 'parse_args', return_value=args), \
                     mock.patch.object(runner, 'verify_compatible_sources', return_value={}), \
                     mock.patch.object(runner, 'load_config', return_value=cfg), \
                     mock.patch.object(runner, 'load_fixture', return_value=matrix), \
                     mock.patch.object(runner, 'build_policy', return_value=(sarl_policy, None)), \
                     mock.patch.object(runner, 'SupervisedController', side_effect=make_controller):
                    runner.main()
                sarl_summary = json.loads((project/'run_sarl_other'/'summary_GPT.json').read_text())
                self.assertTrue(sarl_summary['first_fault_confirmed'])
                self.assertFalse(sarl_summary['handback_completed'])
                self.assertFalse(sarl_summary['second_fault_confirmed'])
                sarl_rows = [json.loads(x) for x in (project/'run_sarl_other'/'handback_observations_GPT.jsonl').read_text().splitlines()]
                self.assertEqual(sarl_rows[-1]['recovery']['tl1']['selected_in_recent_cycles'], 0)
                self.assertEqual(sarl_rows[-1]['junctions']['tl1']['mode_after'], 'CONTINGENCY')
            finally:
                os.chdir(old_cwd)
                sys.modules.pop('run_handback_trial_GPT', None)


if __name__ == '__main__':
    unittest.main()
