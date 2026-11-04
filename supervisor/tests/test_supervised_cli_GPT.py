"""Legacy and opt-in CLI routing without starting SUMO."""
import contextlib
import importlib
import io
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock


class CliTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.old_cwd = Path.cwd()
        os.chdir(self.tmp.name)
        self.calls = []
        self.plan = types.ModuleType('tod_plan')
        self.plan.ORIGINAL_RATE_VPH = 1000
        self.plan.load_route_mix = lambda path: {'source':path}
        self.plan.generate_route_file = lambda *a: None
        self.plan.build_schedule = lambda *a: {}
        self.common = types.ModuleType('tod_eval_common')
        self.common.DEFAULT_SCALE = .37
        self.common.STANDARD_SEEDS = [42,123]
        self.common.make_config = lambda:{'lane_map':{}, 'cycle_length':90, 'yellow_time':3,
                                         'allred_time':3, 's_lb':15, 's_ub':63}
        self.common.default_gen_dir = lambda scale:'routes'
        self.common.FixedController = lambda:object()
        self.common.StaticController = lambda schedule:object()
        self.common.MarlPolicy = self.common.SarlPolicy = lambda *a,**k:object()
        def legacy(env, seed, route, ctrl):
            self.calls.append(('legacy',seed)); return [],[]
        self.common.run_episode = legacy
        envmod = types.ModuleType('sumo_env_marl_ps')
        envmod.SumoMARLEnvPS = lambda config:types.SimpleNamespace(close=lambda:None)
        integrated = types.ModuleType('supervisor.integrated_runner_GPT')
        integrated.run_integrated = lambda **kwargs:self.calls.append(('integrated',kwargs))
        sb3 = types.ModuleType('stable_baselines3')
        owner = self
        class Algo:
            @staticmethod
            def load(path, **kwargs):
                owner.calls.append(('load',path,kwargs)); return object()
        sb3.PPO = sb3.DQN = Algo
        self.modules = {'tod_plan':self.plan, 'tod_eval_common':self.common,
                        'sumo_env_marl_ps':envmod, 'supervisor.integrated_runner_GPT':integrated,
                        'stable_baselines3':sb3}
        self.patcher = mock.patch.dict(sys.modules, self.modules)
        self.patcher.start()
        sys.modules.pop('run_supervised_GPT',None)
        self.runner = importlib.import_module('run_supervised_GPT')
        self.runner.load_config = lambda p:types.SimpleNamespace(conflicts=types.SimpleNamespace(fixture_path='fixture'))
        self.runner.load_fixture = lambda p:object()
        self.runner.validate_against_env = lambda *a:None
        class Ctrl:
            def __init__(self,*a,**k): self.modes={'tl1':'ASTA','tl2':'ASTA'}
            def __enter__(self): return self
            def __exit__(self,*a): pass
            def reset_episode(self,episode_id): pass
        self.runner.SupervisedController = Ctrl

    def tearDown(self):
        sys.modules.pop('run_supervised_GPT',None)
        self.patcher.stop()
        os.chdir(self.old_cwd)
        self.tmp.cleanup()

    def invoke(self,args):
        with mock.patch.object(sys, 'argv', ['run_supervised_GPT.py']+args), contextlib.redirect_stdout(io.StringIO()):
            self.runner.main()

    def test_legacy_fixed_path_remains_separate(self):
        self.invoke(['--controller','fixed','--seeds','1'])
        self.assertEqual(self.calls,[('legacy',42)])

    def test_each_opt_in_combination_routes_to_integrated_loop(self):
        base=['--controller','rl','--kind','marl','--algo','ppo','--model','x.zip',
              '--name','policy','--seeds','1','--run-dir','new_output']
        for extra,slow,handback in [(['--enable-slow-monitor'],True,False),
                                    (['--enable-auto-handback'],False,True),
                                    (['--enable-slow-monitor','--enable-auto-handback'],True,True)]:
            with self.subTest(extra=extra):
                self.calls.clear(); self.invoke(base+extra)
                self.assertEqual(self.calls[0],('load','x.zip',{'device':'cpu'}))
                routed=self.calls[1][1]
                self.assertEqual((routed['slow_enabled'],routed['handback_enabled']),(slow,handback))
                self.assertEqual(routed['seeds'],[42])
                self.assertFalse(any(x[0]=='legacy' for x in self.calls))

    def test_opt_in_requires_isolated_directory_and_rl(self):
        base=['--controller','rl','--kind','marl','--algo','ppo','--model','x.zip','--name','p']
        for args in [base+['--enable-slow-monitor'],
                     ['--controller','fixed','--enable-auto-handback','--run-dir','out'],
                     base+['--enable-slow-monitor','--run-dir','out','--max-cycles','0']]:
            with self.subTest(args=args),contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as raised: self.invoke(args)
                self.assertEqual(raised.exception.code,2)
        self.assertEqual(self.calls,[])


if __name__ == '__main__':
    unittest.main()
