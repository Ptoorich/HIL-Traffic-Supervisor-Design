"""Tests for the opt-in handback trial. Run: python -m unittest -v test_handback_trial_GPT"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import tempfile
import types
import unittest
from unittest import mock

import numpy as np

from handback_trial_logic_GPT import Recovery, valid_pending
from handback_trial_adapters_GPT import (InjectedSnapshotTaker, StrictPolicy,
    read_active_programs, strict_observations)


def cycle(state, *, good=True, pending=0, selected=True, complete=True):
    state.finish_fixed_cycle(complete=complete, gate_ok=good, measurements_ok=good,
                             pending_end=pending, selected=selected)


class RecoveryTests(unittest.TestCase):
    def test_fast_latch_requires_five_complete_cycles_and_three_clean(self):
        r = Recovery(); r.enter('fast_gate', 180)
        cycle(r, good=False)
        for _ in range(3): cycle(r)
        self.assertFalse(r.ready())
        cycle(r)
        self.assertTrue(r.ready())

    def test_missing_invalid_and_partial_data_never_count(self):
        r = Recovery(); r.enter('fast_gate', 0)
        for _ in range(5): cycle(r)
        self.assertTrue(r.ready())
        cycle(r, pending=None)
        self.assertFalse(r.ready())
        self.assertEqual(r.clean, 0)
        dwell = r.dwell
        cycle(r, complete=False)
        self.assertEqual(r.dwell, dwell)
        self.assertFalse(valid_pending(float('nan')))
        self.assertFalse(valid_pending(-1))
        self.assertFalse(valid_pending(True))
        self.assertTrue(valid_pending(0))

    def test_slow_origin_requires_three_consecutive_values_strictly_below_100(self):
        r = Recovery(); r.enter('slow_monitor', 0)
        for _ in range(5): cycle(r, pending=100)
        self.assertFalse(r.ready())
        cycle(r, pending=99); cycle(r, pending=90)
        self.assertFalse(r.ready())
        cycle(r, pending=80)
        self.assertTrue(r.ready())
        cycle(r, pending=100)
        self.assertFalse(r.ready())

    def test_sarl_requires_real_selections_in_latest_ten_cycles(self):
        r = Recovery(is_sarl=True); r.enter('fast_gate', 0)
        for _ in range(10): cycle(r, selected=False)
        self.assertFalse(r.ready())
        for _ in range(2): cycle(r, selected=True)
        self.assertFalse(r.ready())
        cycle(r, selected=True)
        self.assertTrue(r.ready())
        for _ in range(8): cycle(r, selected=False)
        self.assertFalse(r.ready())
        cycle(r, good=False, selected=True)
        self.assertEqual(r.describe()['selected_in_recent_cycles'], 0)

    def test_transition_and_retrigger_restart(self):
        r = Recovery(); r.enter('slow_monitor', 0)
        for _ in range(5): cycle(r, pending=50)
        self.assertTrue(r.ready())
        r.finish_transition(complete=True, gate_ok=True, measurements_ok=True,
                            pending_end=50, selected=True)
        self.assertTrue(r.transition_completed)
        r.enter('fast_gate', 540, restart=True)
        self.assertFalse(r.transition_completed)
        self.assertEqual(r.dwell, 0)
        self.assertEqual(r.causes, {'slow_monitor', 'fast_gate'})
        self.assertFalse(r.ready())

    def test_invalid_during_transition_revokes_release(self):
        r = Recovery(); r.enter('fast_gate', 0)
        for _ in range(5): cycle(r)
        r.finish_transition(complete=True, gate_ok=False, measurements_ok=True,
                            pending_end=0, selected=True)
        self.assertFalse(r.transition_completed)
        self.assertFalse(r.ready())


class FakeEnvironment:
    def __init__(self):
        self.agent_ids = ['tl1', 'tl2']
        self.lane_map = {'tl1': ['a', 'b'], 'tl2': ['c', 'd', 'e']}
        self.phase_splits = {'tl1': 31.0, 'tl2': 59.0}
        self.agent_index = {'tl1': 0.0, 'tl2': 0.5}
        self.s_lb, self.s_ub, self.max_local_obs, self.obs_size = 15, 63, 4, 5
        self.allred_phases, self.allred_time, self.yellow_time, self.cycle_length = True, 3, 3, 90
        self.tl_map = {'tl1': 'T1', 'tl2': 'T2'}
        self._base_states = {a: ('GGrr', 'yyrr', 'rrGG', 'rryy') for a in self.agent_ids}


class FakeLane:
    def __init__(self): self.fail = None
    def getLastStepHaltingNumber(self, lane):
        if lane == self.fail: raise RuntimeError('sensor failed')
        return {'a': 0, 'b': 10, 'c': 20, 'd': 30, 'e': 50}[lane]


class FakeClock:
    def __init__(self): self.time = 90
    def getTime(self): return self.time


class FakeModel:
    def __init__(self, action): self.action, self.received = action, []
    def predict(self, obs, deterministic=True):
        self.received.append(np.array(obs))
        return self.action, None


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.env = FakeEnvironment()
        self.traci = types.SimpleNamespace(lane=FakeLane(), simulation=FakeClock())

    def test_observations_match_normalisation_padding_and_actual_fixed_split(self):
        obs, errors = strict_observations(self.env, self.traci, 90)
        self.assertFalse(errors)
        np.testing.assert_allclose(obs['tl1'], [0, .2, 16 / 48, 0, 0])
        np.testing.assert_allclose(obs['tl2'], [.4, .6, 1, 44 / 48, .5])

    def test_missing_reading_is_not_replaced_with_a_zero_observation(self):
        self.traci.lane.fail = 'a'
        obs, errors = strict_observations(self.env, self.traci, 90)
        self.assertIn('tl1', errors); self.assertNotIn('tl1', obs)
        p = StrictPolicy('marl', FakeModel(2), self.traci)
        self.assertEqual(p.act(self.env, {}, 90), {'tl1': -1, 'tl2': 2})
        s = StrictPolicy('sarl', FakeModel(0), self.traci)
        self.assertEqual(s.act(self.env, {}, 90), {'tl1': -1, 'tl2': -1})
        self.assertIsNone(s.last_selected_agent)

    def test_stale_time_invalidates_all_observations(self):
        obs, errors = strict_observations(self.env, self.traci, 91)
        self.assertEqual(obs, {})
        self.assertEqual(set(errors), {'tl1', 'tl2'})

    def test_all_six_sarl_actions_and_noop_mapping(self):
        for action in range(6):
            p = StrictPolicy('sarl', FakeModel(action), self.traci)
            result = p.act(self.env, {}, 90)
            target = self.env.agent_ids[action // 3]
            self.assertEqual(p.last_joint_action, action)
            self.assertEqual(p.last_selected_agent, target)
            self.assertEqual(result[target], action % 3)
            self.assertEqual(result[self.env.agent_ids[1 - action // 3]], 1)
            self.assertEqual(len(p.model.received[0]), 7)

    def test_synthetic_fault_removes_only_one_lane_once(self):
        @dataclass(frozen=True)
        class Reading: lane_id: str
        @dataclass(frozen=True)
        class Snapshot: lanes: tuple; sim_time: float
        class Taker:
            def take(self, env, current_splits=None):
                return {'tl1': Snapshot((Reading('a'), Reading('b')), 90)}
            def clear_cache(self): pass
        injector = InjectedSnapshotTaker(Taker()); injector.arm('tl1', 'a')
        result = injector.take(self.env)
        self.assertEqual([r.lane_id for r in result['tl1'].lanes], ['b'])
        self.assertEqual(injector.last_injected['lane'], 'a')
        self.assertEqual(len(injector.take(self.env)['tl1'].lanes), 2)

    def test_active_program_mismatch_is_detected(self):
        env = self.env
        class Lights:
            def __init__(self): self.bad = False
            def getProgram(self, tl_id): return 'p'
            def getAllProgramLogics(self, tl_id):
                agent = 'tl1' if tl_id == 'T1' else 'tl2'
                a = env.phase_splits[agent]; b = 78 - a
                durations = [a, 3, 3, b, 3, 3]
                if self.bad: durations[0] += 1
                states = ['GGrr', 'yyrr', 'rrrr', 'rrGG', 'rryy', 'rrrr']
                return [types.SimpleNamespace(programID='p', phases=[types.SimpleNamespace(duration=d, state=s)
                                                                    for d, s in zip(durations, states)])]
        self.traci.trafficlight = Lights()
        result = read_active_programs(env, self.traci)
        self.assertEqual((result['tl1']['green_a'], result['tl1']['green_b']), (31, 47))
        self.traci.trafficlight.bad = True
        with self.assertRaises(ValueError): read_active_programs(env, self.traci)


if __name__ == '__main__':
    unittest.main()
