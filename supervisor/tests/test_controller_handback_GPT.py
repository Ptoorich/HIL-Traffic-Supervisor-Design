"""Regression checks for explicit boundary latch and gated release."""
import json
from pathlib import Path
import tempfile
import unittest

from supervisor.config import load_config
from supervisor.conflicts import ConflictMatrix, JunctionConflicts
from supervisor.datatypes import GateResult, Overall
from supervisor.supervised_integrated_GPT import IntegratedSupervisedController
from supervisor.supervised_controller_GPT import SupervisedController as LegacyController
from supervisor.tests.helpers import COMMANDED, healthy_snapshot


class Env:
    def __init__(self, cfg):
        self.agent_ids = list(cfg.agents)
        self.tl_map = {a: f'tl_{a}' for a in cfg.agents}
        self.cycle_length, self.yellow_time, self.allred_time = 90, 3, 3
        self.lane_map = {a: [x.lane_id for x in cfg.lanes_of(a)] for a in cfg.agents}
        self.phase_splits = {a: 39.0 for a in cfg.agents}
    def set_splits(self, values): self.phase_splits.update(values)


class Taker:
    def __init__(self, cfg): self.cfg, self.fail = cfg, False
    def clear_cache(self): pass
    def take(self, env, current_splits=None):
        if self.fail: raise RuntimeError('snapshot failure')
        return {a: healthy_snapshot(self.cfg, a, split=current_splits[a]) for a in self.cfg.agents}


class Gate:
    def __init__(self, cfg): self.cfg, self.veto, self.missing = cfg, set(), set()
    def evaluate(self, snapshots, proposals, **kwargs):
        return {a: GateResult(a, (), Overall.VETO if a in self.veto else Overall.APPROVE,
                             (), (), 'FREE', 0, 0)
                for a in self.cfg.agents if a not in self.missing}


class Inner:
    name = 'test'
    def __init__(self): self.actions = {'tl1': 2, 'tl2': 1}
    def act(self, env, obs, t): return self.actions


class ControllerHandbackTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.cfg = load_config()
        self.env, self.inner, self.taker, self.gate = Env(self.cfg), Inner(), Taker(self.cfg), Gate(self.cfg)
        foes = frozenset({(0, 2), (0, 3), (1, 2), (1, 3)})
        self.matrix = ConflictMatrix({a: JunctionConflicts(f'tl_{a}', 4, foes, COMMANDED)
                                      for a in self.cfg.agents})
        self.path = Path(self.tmp.name)/'decisions.jsonl'
        self.ctrl = IntegratedSupervisedController(self.inner, self.cfg, self.matrix, self.path,
                                                   snapshot_taker=self.taker, gate=self.gate)

    def tearDown(self):
        self.ctrl.close(); self.tmp.cleanup()

    def test_default_path_keeps_latch_until_episode_reset(self):
        self.gate.veto.add('tl1')
        self.assertEqual(self.ctrl.act(self.env, {}, 0), {'tl1': 1, 'tl2': 1})
        self.gate.veto.clear()
        self.assertEqual(self.ctrl.act(self.env, {}, 90), {'tl1': 1, 'tl2': 1})
        self.assertEqual(self.ctrl.modes['tl1'], 'CONTINGENCY')
        self.assertNotIn('handback_requested', self.ctrl.last_entry['decisions']['tl1'])
        self.ctrl.reset_episode('new')
        self.assertIsNone(self.ctrl.last_entry)
        self.assertEqual(self.ctrl.modes, {'tl1':'ASTA', 'tl2':'ASTA'})

    def test_external_latch_is_local_logged_and_cannot_release_immediately(self):
        self.assertTrue(self.ctrl.enter_contingency('tl1', reason='slow_monitor'))
        self.assertFalse(self.ctrl.enter_contingency('tl1', reason='different'))
        with self.assertRaises(ValueError):
            self.ctrl.act(self.env, {}, 0, release_candidates={'tl1'})
        self.assertEqual(self.ctrl.act(self.env, {}, 0), {'tl1':1, 'tl2':1})
        self.assertEqual(self.ctrl.last_entry['decisions']['tl1']['external_contingency_reason'], 'slow_monitor')
        self.assertEqual(self.ctrl.modes['tl2'], 'ASTA')
        self.assertEqual(self.ctrl.act(self.env, {}, 90, release_candidates={'tl1'}), {'tl1':2, 'tl2':1})
        d = self.ctrl.last_entry['decisions']['tl1']
        self.assertTrue(d['handback_requested']); self.assertTrue(d['handback_accepted'])
        self.assertFalse(d['entered_contingency']); self.assertEqual(d['mode_after'], 'ASTA')
        self.assertEqual(json.loads(self.path.read_text().splitlines()[-1]), self.ctrl.last_entry)

    def test_live_gate_veto_or_missing_result_blocks_release(self):
        self.ctrl.enter_contingency('tl1', reason='slow_monitor')
        self.ctrl.act(self.env, {}, 0)
        self.gate.veto.add('tl1')
        self.assertEqual(self.ctrl.act(self.env, {}, 90, release_candidates={'tl1'})['tl1'], 1)
        d = self.ctrl.last_entry['decisions']['tl1']
        self.assertFalse(d['handback_accepted']); self.assertEqual(d['mode_after'], 'CONTINGENCY')
        self.gate.veto.clear(); self.gate.missing.add('tl1')
        self.assertEqual(self.ctrl.act(self.env, {}, 180, release_candidates={'tl1'})['tl1'], 1)
        self.assertEqual(self.ctrl.modes['tl1'], 'CONTINGENCY')

    def test_invalid_proposal_and_snapshot_failure_block_release(self):
        self.ctrl.enter_contingency('tl1', reason='slow_monitor')
        self.ctrl.act(self.env, {}, 0)
        self.inner.actions['tl1'] = -1
        self.ctrl.act(self.env, {}, 90, release_candidates={'tl1'})
        self.assertEqual(self.ctrl.modes['tl1'], 'CONTINGENCY')
        self.inner.actions['tl1'] = 2; self.taker.fail = True
        self.ctrl.act(self.env, {}, 180, release_candidates={'tl1'})
        self.assertEqual(self.ctrl.modes['tl1'], 'CONTINGENCY')

    def test_simultaneous_release_is_resolved_per_junction(self):
        self.ctrl.enter_contingency('tl1', reason='slow_monitor')
        self.ctrl.enter_contingency('tl2', reason='slow_monitor')
        self.ctrl.act(self.env, {}, 0)
        self.gate.veto.add('tl1')
        self.ctrl.act(self.env, {}, 90, release_candidates={'tl1','tl2'})
        self.assertEqual(self.ctrl.modes, {'tl1':'CONTINGENCY','tl2':'ASTA'})
        self.assertFalse(self.ctrl.last_entry['decisions']['tl1']['handback_accepted'])
        self.assertTrue(self.ctrl.last_entry['decisions']['tl2']['handback_accepted'])

    def test_integrated_wrapper_matches_legacy_when_no_release_is_requested(self):
        legacy_env, modern_env = Env(self.cfg), Env(self.cfg)
        legacy_path, modern_path = Path(self.tmp.name)/'legacy.jsonl', Path(self.tmp.name)/'modern.jsonl'
        with LegacyController(self.inner,self.cfg,self.matrix,legacy_path,snapshot_taker=self.taker,
                              gate=self.gate,clock=lambda:0,run_id='parity') as legacy, \
             IntegratedSupervisedController(self.inner,self.cfg,self.matrix,modern_path,snapshot_taker=self.taker,
                                           gate=self.gate,clock=lambda:0,run_id='parity') as modern:
            for t, veto in [(0,set()),(90,{'tl1'}),(180,set())]:
                self.gate.veto = veto
                self.assertEqual(legacy.act(legacy_env,{},t),modern.act(modern_env,{},t))
                self.assertEqual(legacy.modes,modern.modes)
                self.assertEqual(legacy_env.phase_splits,modern_env.phase_splits)
            self.assertEqual([json.loads(x) for x in legacy_path.read_text().splitlines()],
                             [json.loads(x) for x in modern_path.read_text().splitlines()])


if __name__ == '__main__':
    unittest.main()
