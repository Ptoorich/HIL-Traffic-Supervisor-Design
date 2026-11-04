"""Integration of the opt-in loop with the real Fast Gate and a fake SUMO."""
from collections import Counter
from dataclasses import replace
import importlib
import json
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock
import numpy  # Keep NumPy loaded while mocked modules are installed and restored.

from supervisor.config import load_config
from supervisor.conflicts import ConflictMatrix, JunctionConflicts
from supervisor.supervised_integrated_GPT import IntegratedSupervisedController
from supervisor.tests.helpers import COMMANDED, healthy_snapshot


class FakeSimulation:
    def __init__(self, pending): self.now, self.pending = 0, pending
    def getTime(self): return self.now
    def getPendingVehicles(self): return range(self.pending(self.now))
    def getDepartedNumber(self): return 0
    def getArrivedNumber(self): return 0
    def getStartingTeleportNumber(self): return 0


class FakeEnvironment:
    def __init__(self, cfg, traci):
        self.cfg, self.traci = cfg, traci
        traci.env = self
        self.agent_ids = list(cfg.agents)
        self.lane_map = {a: [x.lane_id for x in cfg.lanes_of(a)] for a in cfg.agents}
        self.tl_map = {a: f'tl_{a}' for a in cfg.agents}
        self.phase_splits = {a: 39.0 for a in cfg.agents}
        self.agent_index = {'tl1':0.0, 'tl2':0.5}
        self.max_local_obs = max(len(x) + 1 for x in self.lane_map.values())
        self.obs_size = self.max_local_obs + 1
        self.s_lb, self.s_ub = 15, 63
        self.cycle_length, self.yellow_time, self.allred_time = 90, 3, 3
        self.allred_phases, self.step_length, self.current_step = True, 90, 0
        self.stop_at = None
        self._base_states = {a: COMMANDED for a in cfg.agents}
    def set_splits(self, values): self.phase_splits.update(values)
    def _split_ub(self): return 63
    def reset(self, seed=None, route_file=None):
        self.traci.simulation.now = self.current_step = 0
        return {}
    def step(self, actions):
        for a, act in actions.items():
            self.phase_splits[a] = max(15, min(63, self.phase_splits[a] + {0:-5, 1:0, 2:5}[act]))
        for _ in range(self.step_length):
            if self.stop_at is not None and self.traci.simulation.now >= self.stop_at:
                break
            self.traci.simulationStep()
        self.current_step = self.traci.simulation.now
        done = self.stop_at is not None and self.traci.simulation.now >= self.stop_at
        return {}, {}, {a:done for a in self.cfg.agents}
    def close(self): pass


def make_traci(pending):
    traci = types.ModuleType('traci')
    traci.simulation = FakeSimulation(pending)
    traci.lane = types.SimpleNamespace(getLastStepHaltingNumber=lambda lane:0)
    def step(): traci.simulation.now += 1
    traci.simulationStep = step
    class Lights:
        def getProgram(self, tl_id):
            agent = next(a for a in traci.env.agent_ids if traci.env.tl_map[a] == tl_id)
            return f'tod6_{traci.env.phase_splits[agent]:g}'
        def getAllProgramLogics(self, tl_id):
            agent = next(a for a in traci.env.agent_ids if traci.env.tl_map[a] == tl_id)
            a = traci.env.phase_splits[agent]
            durations = [a, 3, 3, 78-a, 3, 3]
            states = ['GGrr', 'yyrr', 'rrrr', 'rrGG', 'yyrr', 'rrrr']
            # COMMANDED's final phase is rryy.
            states[-2] = 'rryy'
            phases = [types.SimpleNamespace(duration=d, state=s) for d, s in zip(durations, states)]
            return [types.SimpleNamespace(programID=self.getProgram(tl_id), phases=phases)]
    traci.trafficlight = Lights()
    return traci


class FakeTaker:
    def __init__(self, cfg, traci, faults): self.cfg, self.traci, self.faults = cfg, traci, faults
    def clear_cache(self): pass
    def take(self, env, current_splits=None):
        snapshots = {a:healthy_snapshot(self.cfg, a, split=current_splits[a], sim_time=self.traci.simulation.now)
                     for a in self.cfg.agents}
        for agent in self.faults.get(self.traci.simulation.now, ()):
            snap = snapshots[agent]
            snapshots[agent] = replace(snap, lanes=snap.lanes[1:])
        return snapshots


class Model:
    def __init__(self, action): self.action = action
    def predict(self, obs, deterministic=True): return self.action, None


class IntegratedLoopTest(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config()
        foes = frozenset({(0, 2), (0, 3), (1, 2), (1, 3)})
        self.matrix = ConflictMatrix({a:JunctionConflicts(f'tl_{a}', 4, foes, COMMANDED)
                                      for a in self.cfg.agents})
        self.temp = tempfile.TemporaryDirectory()

    def tearDown(self): self.temp.cleanup()

    def run_case(self, label, *, slow, handback, kind='marl', action=0,
                 pending=lambda t:0, faults=None, max_cycles=20, stop_at=None):
        fake = make_traci(pending)
        env = FakeEnvironment(self.cfg, fake)
        env.stop_at = stop_at
        stub_plan = types.ModuleType('tod_plan')
        stub_env = types.ModuleType('sumo_env_marl_ps')
        stub_env.SumoMARLEnvPS = FakeEnvironment
        modules = {'traci':fake, 'tod_plan':stub_plan, 'sumo_env_marl_ps':stub_env}
        with mock.patch.dict(sys.modules, modules):
            sys.modules.pop('supervisor.integrated_runner_GPT', None)
            runner = importlib.import_module('supervisor.integrated_runner_GPT')
            policy = runner.StrictPolicy(kind, Model(action), fake)
            taker = FakeTaker(self.cfg, fake, faults or {})
            def factory(policy, cfg, matrix, **kwargs):
                return IntegratedSupervisedController(policy, cfg, matrix, snapshot_taker=taker, **kwargs)
            out = Path(self.temp.name)/label; out.mkdir()
            summary = runner.run_episode(env, policy, self.cfg, self.matrix, 42, out/'route.xml', out,
                        slow_enabled=slow, handback_enabled=handback, max_cycles=max_cycles,
                        traci_mod=fake, controller_factory=factory)
            def read(name): return [json.loads(x) for x in (out/name).read_text().splitlines()]
            result = summary, read('events_GPT.jsonl'), read('observations_GPT.jsonl'), read('decisions_GPT.jsonl')
            sys.modules.pop('supervisor.integrated_runner_GPT', None)
            return result

    def test_fast_fault_handback_and_renewed_fault(self):
        summary, events, rows, _ = self.run_case('fast', slow=False, handback=True,
                                                  faults={180:('tl1',),810:('tl1',)})
        self.assertEqual(summary['fast_contingencies'], {'tl1':2, 'tl2':0})
        self.assertEqual(summary['handbacks_completed'], {'tl1':2, 'tl2':0})
        self.assertEqual([e['sim_time'] for e in events if e['event']=='handback_completed'], [810,1440])
        self.assertTrue(all(row['junctions']['tl2']['mode_after']=='ASTA' for row in rows))
        self.assertTrue(all(row['slow_monitor_evaluation']['status']=='disabled' for row in rows))
        first_release = next(row for row in rows if row['start_time']==720)
        self.assertEqual(first_release['junctions']['tl1']['mode_before'],'CONTINGENCY')
        self.assertTrue(first_release['junctions']['tl1']['handback_accepted'])
        self.assertEqual(first_release['behavior']['tl1']['cycles_available'],1)

    def test_live_veto_at_release_keeps_fixed_programme(self):
        summary, events, rows, decisions = self.run_case('veto', slow=False, handback=True,
                                                          faults={180:('tl1',),720:('tl1',)}, max_cycles=22)
        veto = next(e for e in events if e['event']=='handback_vetoed')
        self.assertEqual(veto['sim_time'], 720)
        row = next(x for x in rows if x['start_time']==720)
        self.assertEqual(row['junctions']['tl1']['mode_after'], 'CONTINGENCY')
        self.assertEqual(row['active_programs_after_first_step']['tl1']['green_a'], 31)
        decision = next(x for x in decisions if x['sim_time']==720)['decisions']['tl1']
        self.assertTrue(decision['handback_requested']); self.assertFalse(decision['handback_accepted'])
        self.assertGreater(next(e['sim_time'] for e in events if e['event']=='handback_completed'), 810)

    def test_slow_monitor_latches_and_stays_latched_without_handback(self):
        summary, events, rows, _ = self.run_case('slow', slow=True, handback=False,
                                                pending=lambda t:t//3, max_cycles=16)
        self.assertEqual(summary['slow_contingencies'], {'tl1':1,'tl2':1})
        self.assertEqual(summary['handbacks_completed'], {'tl1':0,'tl2':0})
        self.assertEqual(summary['final_modes'], {'tl1':'CONTINGENCY','tl2':'CONTINGENCY'})
        self.assertEqual({e['sim_time'] for e in events if e['event']=='slow_contingency_set'}, {900})
        self.assertTrue(all(r['recovery'] is None for r in rows))

    def test_both_features_switch_then_hand_back_after_backlog_recovery(self):
        def demand(t):
            if t<900: return t//3
            if t<1800: return 300-(t-900)//5
            return max(0,120-(t-1800))
        summary, events, _, _ = self.run_case('both', slow=True, handback=True,
                                             pending=demand, max_cycles=28)
        self.assertEqual(summary['slow_contingencies'], {'tl1':1,'tl2':1})
        self.assertEqual(summary['handbacks_completed'], {'tl1':1,'tl2':1})
        for event in events:
            if event['event']=='transition_started':
                self.assertGreaterEqual(event['recovery']['low_pending_cycles'], 3)

    def test_sarl_unselected_noops_do_not_create_recovery_samples(self):
        summary, events, rows, _ = self.run_case('sarl', slow=False, handback=True,
                            kind='sarl', action=5, faults={180:('tl1',)}, max_cycles=20)
        self.assertEqual(summary['handbacks_completed']['tl1'], 0)
        self.assertEqual(summary['final_modes']['tl1'], 'CONTINGENCY')
        self.assertEqual(rows[-1]['recovery']['tl1']['selected_in_recent_cycles'], 0)
        self.assertTrue(all(r['sarl_selected_agent']=='tl2' for r in rows))

    def test_partial_release_cycle_is_not_reported_as_a_completed_handback(self):
        summary, events, rows, _ = self.run_case('partial', slow=False, handback=True,
                                                faults={180:('tl1',)}, stop_at=735)
        self.assertEqual(summary['handbacks_completed']['tl1'], 0)
        self.assertEqual(summary['handbacks_unverified_partial']['tl1'], 1)
        self.assertTrue(any(e['event']=='handback_unverified_partial' for e in events))
        self.assertEqual((rows[-1]['start_time'],rows[-1]['end_time'],rows[-1]['complete_cycle']),
                         (720,735,False))

    def test_unavailable_pending_data_cancels_transition_and_is_not_zero_filled(self):
        def pending(t):
            if t == 720: raise RuntimeError('pending unavailable')
            return 0
        summary, events, rows, _ = self.run_case('missing_pending', slow=False, handback=True,
                            pending=pending, faults={180:('tl1',)}, max_cycles=16)
        transition = next(e for e in events if e['event']=='transition_cancelled')
        self.assertEqual(transition['sim_time'],720)
        self.assertIsNone(next(r for r in rows if r['end_time']==720)['boundary_end']['pending_insertion'])
        self.assertGreater(next(e['sim_time'] for e in events if e['event']=='handback_completed'),810)
        self.assertEqual(summary['handbacks_completed']['tl1'],1)

    def test_seed_outputs_are_isolated_and_reset_independently(self):
        fake = make_traci(lambda t:0)
        plan = types.ModuleType('tod_plan')
        plan.generate_route_file = lambda path,mix,seed,peak:Path(path).write_text(f'<routes seed="{seed}"/>')
        envmod = types.ModuleType('sumo_env_marl_ps'); envmod.SumoMARLEnvPS = FakeEnvironment
        modules = {'traci':fake, 'tod_plan':plan, 'sumo_env_marl_ps':envmod}
        project = Path(self.temp.name)/'project'; inputs = project/'2int_files'; inputs.mkdir(parents=True)
        (inputs/'2int.sumocfg').write_text('<sumoConfiguration><input><net-file value="net.xml"/>'
            '<route-files value="routes.xml"/></input><output><statistic-output value="stats.xml"/>'
            '</output></sumoConfiguration>')
        (inputs/'stats.xml').write_text('original output')
        model_path = project/'model.zip'; model_path.write_bytes(b'model')
        model = Model(0)
        model.observation_space = types.SimpleNamespace(shape=(9,))
        model.action_space = types.SimpleNamespace(n=3)
        config = {'agent_ids':list(self.cfg.agents),
                  'lane_map':{a:[x.lane_id for x in self.cfg.lanes_of(a)] for a in self.cfg.agents}}
        original_cwd = Path.cwd()
        with mock.patch.dict(sys.modules, modules):
            sys.modules.pop('supervisor.integrated_runner_GPT', None)
            runner = importlib.import_module('supervisor.integrated_runner_GPT')
            def make_controller(policy,cfg,matrix,**kwargs):
                return IntegratedSupervisedController(policy,cfg,matrix,
                    snapshot_taker=FakeTaker(cfg,fake,{}),**kwargs)
            try:
                os.chdir(project)
                with mock.patch.object(runner,'SumoMARLEnvPS',side_effect=lambda c:FakeEnvironment(self.cfg,fake)), \
                     mock.patch.object(runner,'IntegratedSupervisedController',side_effect=make_controller):
                    kwargs = dict(config=config,cfg=self.cfg,matrix=self.matrix,model=model,kind='marl',
                        model_path=model_path,name='test',seeds=[42,123],mix={},peak=370,
                        run_dir='run',slow_enabled=True,handback_enabled=True,max_cycles=2)
                    runner.run_integrated(**kwargs)
                    with self.assertRaises(FileExistsError): runner.run_integrated(**kwargs)
                for seed in (42,123):
                    folder=project/'run'/f'seed_{seed}'
                    summary=json.loads((folder/'summary_GPT.json').read_text())
                    self.assertEqual(summary['seed'],seed)
                    self.assertEqual(summary['cycles_observed'],2)
                    self.assertEqual(summary['slow_contingencies'],{'tl1':0,'tl2':0})
                    self.assertEqual(summary['handbacks_completed'],{'tl1':0,'tl2':0})
                self.assertEqual((inputs/'stats.xml').read_text(),'original output')
                unsafe_out=project/'unsafe'; unsafe_out.mkdir()
                (inputs/'2int.sumocfg').write_text('<sumoConfiguration><output>'
                    '<statistic-output value="../../../outside.xml"/></output></sumoConfiguration>')
                with self.assertRaises(ValueError): runner.isolate_scenario(project,unsafe_out)
                self.assertFalse((project/'outside.xml').exists())
            finally:
                os.chdir(original_cwd)
                sys.modules.pop('supervisor.integrated_runner_GPT', None)


if __name__ == '__main__':
    unittest.main()
