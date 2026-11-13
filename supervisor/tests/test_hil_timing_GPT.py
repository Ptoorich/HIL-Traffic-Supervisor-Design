"""Timing endpoints, failures, and control parity using the actual host stack.

The board/clock used here are test doubles, not physical latency evidence.
"""
import importlib
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

from supervisor.config import load_config
from supervisor.conflicts import ConflictMatrix, JunctionConflicts
from supervisor.hil_display_GPT import PhaseDisplay
from supervisor.hil_serial_GPT import SerialDisplay, DisplayError, encode_frame, decode_frame, PHASE_MASKS
from supervisor.hil_timing_GPT import BoundaryTiming, TimedPolicy
from supervisor.sp28_monitor_GPT import SP28ReadbackFailure
from supervisor.supervised_integrated_GPT import IntegratedSupervisedController
from supervisor.tests.helpers import COMMANDED
from supervisor.tests.test_integrated_runner_GPT import FakeEnvironment, FakeTaker, make_traci


class Clock:
    def __init__(self): self.now = 100.0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class BoardDouble:
    def __init__(self, clock, *, drop_first=False, wrong_sequence=None):
        self.clock, self.drop_first, self.wrong_sequence = clock, drop_first, wrong_sequence
        self.buffer = bytearray(encode_frame(1, 'R', 1))
        self.due, self.writes, self.attempts = clock(), [], {}

    def write(self, data):
        self.writes.append(data)
        f = decode_frame(data)
        kind, session = f[1:3]
        if kind == 'F': return len(data)
        sequence = 0 if kind == 'H' else int(f[3])
        mask = PHASE_MASKS[int(f[5])] | PHASE_MASKS[int(f[6])] << 6 if kind == 'S' else 2340
        self.attempts[sequence] = self.attempts.get(sequence, 0) + 1
        if self.drop_first and kind == 'S' and self.attempts[sequence] == 1:
            return len(data)
        if sequence == self.wrong_sequence: mask ^= 1
        self.buffer.extend(encode_frame(1, 'A', session, sequence, mask, kind))
        self.due = self.clock() + .010
        return len(data)

    def read(self, size):
        if self.buffer and self.clock() >= self.due:
            value = bytes(self.buffer[:size]); del self.buffer[:size]
            return value
        self.clock.sleep(.001)
        return b''

    def close(self): pass


def read_rows(path):
    return [json.loads(s) for s in Path(path).read_text().splitlines() if s.strip()]


class TimingTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
    def tearDown(self): self.temp.cleanup()

    def test_receipt_precedes_ack_log_and_retry_preserves_the_same_frame(self):
        clock = Clock(); board = BoardDouble(clock, drop_first=True)
        class SlowLogSerial(SerialDisplay):
            def record(self, event, **fields):
                clock.sleep(.006)
                super().record(event, **fields)
        with SlowLogSerial('TEST', log_path=self.root/'display.jsonl', serial_instance=board, clock=clock) as transport:
            receipt = transport.state(0, [0, 3], [0, 0])
            self.assertEqual(receipt['attempts'], 2)
            self.assertGreaterEqual(receipt['rtt_ms'], 250)
            self.assertLess(receipt['rtt_ms'], 500)
            self.assertEqual(board.writes[-1], board.writes[-2])
            row = read_rows(self.root/'display.jsonl')[-1]
            self.assertEqual(row['ack_received_at'], receipt['ack_received_at'])
            self.assertAlmostEqual(row['host_monotonic'] - row['ack_received_at'], .006)
            self.assertEqual(receipt['sequence'], 2)

    def test_continuous_endpoints_include_policy_and_keep_sleep_explicit(self):
        clock = Clock()
        with BoundaryTiming(self.root, clock=clock, budget_ms=3000) as timing:
            timing.begin(0, 0)
            clock.sleep(.003)
            policy = types.SimpleNamespace(act=lambda: (clock.sleep(.005), {'tl1': 1})[1])
            self.assertEqual(TimedPolicy(policy, timing).act(), {'tl1': 1})
            clock.sleep(.007); timing.mark('decision_finished_at'); timing.mark('env_step_called_at')
            def sample(sim, sampled, sent, acked, sequence):
                return {'sim_time':sim, 'sampled_at':sampled,
                        'ack':{'kind':'S', 'session':7, 'sequence':sequence,
                               'sent_at':sent, 'ack_received_at':acked}}
            timing.boundary_ack(sample(0, 100.018, 100.020, 100.030, 2))
            clock.now = 101.032
            timing.before_first_step(types.SimpleNamespace(step_sleep_requested_ms=1000, step_sleep_actual_ms=1000))
            timing.first_step_ack(sample(1, 101.036, 101.038, 101.048, 3))
        row = read_rows(self.root/'timing_GPT.jsonl')[0]
        m = row['metrics_ms']
        for key, expected in [('policy_call_ms',5), ('post_policy_to_boundary_ack_ms',22),
                              ('policy_to_boundary_ack_ms',27), ('policy_to_first_step_ack_ms',1045),
                              ('policy_to_first_step_ack_excluding_pacing_ms',45)]:
            self.assertAlmostEqual(m[key], expected)
        self.assertEqual(row['status'], 'complete')
        self.assertFalse(any(row['budget_exceeded'].values()))

    def test_wrong_boundary_receipt_is_invalid_not_a_small_successful_latency(self):
        clock = Clock()
        with BoundaryTiming(self.root, clock=clock, budget_ms=3000) as timing:
            timing.begin(0, 0)
            timing.boundary_ack({'sim_time':0, 'ack':{'kind':'S','session':1,'sequence':2}})
            timing.first_step_ack({'sim_time':1, 'ack':{'kind':'S','session':1,'sequence':5}})
        result = json.loads((self.root/'timing_summary_GPT.json').read_text())
        self.assertEqual(result['measurement_status'], 'INCOMPLETE')
        self.assertEqual(result['budget_status'], 'INCOMPLETE')
        self.assertEqual(result['boundaries_completed'], 0)
        self.assertEqual(read_rows(self.root/'timing_GPT.jsonl')[0]['status'], 'invalid')

    def test_interruption_cannot_be_summarised_as_success(self):
        with self.assertRaises(DisplayError):
            with BoundaryTiming(self.root, clock=Clock(), budget_ms=3000) as timing:
                timing.begin(0, 0)
                raise DisplayError('ACK failed')
        result = json.loads((self.root/'timing_summary_GPT.json').read_text())
        self.assertEqual(result['measurement_status'], 'INCOMPLETE')
        self.assertEqual(result['budget_status'], 'INCOMPLETE')
        self.assertEqual(result['metrics_ms'], {})

    def run_loop(self, label, *, display=True, wrong_sequence=None, stop_at=None, prediction_s=.002):
        cfg = load_config()
        foes = frozenset({(0,2), (0,3), (1,2), (1,3)})
        matrix = ConflictMatrix({a:JunctionConflicts(f'tl_{a}',4,foes,COMMANDED) for a in cfg.agents})
        fake = make_traci(lambda t:0)
        env = FakeEnvironment(cfg, fake); env.stop_at = stop_at
        fake.trafficlight.getPhase = lambda tl: 0
        fake.trafficlight.getRedYellowGreenState = lambda tl: 'GGrr'
        clock = Clock()
        class Model:
            def predict(self, obs, deterministic=True):
                clock.sleep(prediction_s)
                return 0, None
        plan = types.ModuleType('tod_plan')
        envmod = types.ModuleType('sumo_env_marl_ps'); envmod.SumoMARLEnvPS = FakeEnvironment
        out = self.root/label; out.mkdir()
        with mock.patch.dict(sys.modules, {'traci':fake,'tod_plan':plan,'sumo_env_marl_ps':envmod}):
            sys.modules.pop('supervisor.integrated_runner_GPT', None)
            runner = importlib.import_module('supervisor.integrated_runner_GPT')
            policy = runner.StrictPolicy('marl', Model(), fake)
            def factory(inner, cfg, matrix, **kwargs):
                return IntegratedSupervisedController(inner,cfg,matrix,
                    snapshot_taker=FakeTaker(cfg,fake,{90:('tl1',)}),**kwargs)
            kwargs = dict(slow_enabled=False,handback_enabled=True,max_cycles=10,
                          traci_mod=fake,controller_factory=factory)
            try:
                if display:
                    board = BoardDouble(clock, wrong_sequence=wrong_sequence)
                    with SerialDisplay('TEST',log_path=out/'display_GPT.jsonl',serial_instance=board,clock=clock) as transport:
                        phase = PhaseDisplay(transport, speed=1, sleep=clock.sleep)
                        summary = runner.run_episode(env,policy,cfg,matrix,42,out/'routes.xml',out,display=phase,**kwargs)
                else:
                    summary = runner.run_episode(env,policy,cfg,matrix,42,out/'routes.xml',out,**kwargs)
            finally:
                sys.modules.pop('supervisor.integrated_runner_GPT', None)
        return summary, out

    def test_real_host_loop_preserves_fallback_handback_and_all_samples(self):
        summary, out = self.run_loop('timed')
        baseline, original = self.run_loop('baseline', display=False)
        self.assertEqual({k:v for k,v in summary.items() if k!='hil_timing'}, baseline)
        self.assertEqual(read_rows(out/'observations_GPT.jsonl'), read_rows(original/'observations_GPT.jsonl'))
        self.assertEqual(read_rows(out/'events_GPT.jsonl'), read_rows(original/'events_GPT.jsonl'))
        self.assertFalse((original/'timing_GPT.jsonl').exists())
        self.assertEqual(summary['fast_contingencies'], {'tl1':1,'tl2':0})
        self.assertEqual(summary['handbacks_completed'], {'tl1':1,'tl2':0})
        rows = read_rows(out/'timing_GPT.jsonl')
        self.assertEqual(len(rows),10)
        self.assertTrue(all(r['status']=='complete' for r in rows))
        self.assertTrue(all(abs(r['metrics_ms']['policy_call_ms']-4) < 1e-6 for r in rows))
        self.assertTrue(all(r['metrics_ms']['policy_to_first_step_ack_ms'] > 1000 for r in rows))
        self.assertTrue(all(r['metrics_ms']['policy_to_first_step_ack_excluding_pacing_ms'] < 100 for r in rows))
        self.assertEqual([r['boundary_output']['ack']['sequence'] for r in rows], [2+91*c for c in range(10)])
        display = read_rows(out/'display_GPT.jsonl')
        self.assertEqual(sum(r['event']=='sample' for r in display),910)
        self.assertEqual(display[-1]['kind'],'X')
        acks = {r['sequence']:r for r in display if r['event']=='ack'}
        for r in rows:
            for field in ('boundary_output','first_step_output'):
                ack = r[field]['ack']
                self.assertEqual(ack['ack_received_at'],acks[ack['sequence']]['ack_received_at'])

    def test_failed_first_step_ack_leaves_incomplete_measurement_and_fault(self):
        # The integrated SP-28 monitor wraps a raw DisplayError into an
        # explicitly classified execution-verification FAIL.
        with self.assertRaises(SP28ReadbackFailure) as caught:
            self.run_loop('failed',wrong_sequence=3)
        self.assertEqual(caught.exception.requirement, 'SP-28')
        self.assertEqual(caught.exception.status, 'FAIL')
        self.assertEqual(caught.exception.source, 'arduino_gpio_or_communication')
        self.assertEqual(caught.exception.stage, 'after_simulation_step')
        self.assertIsInstance(caught.exception.__cause__, DisplayError)
        out = self.root/'failed'
        result = json.loads((out/'timing_summary_GPT.json').read_text())
        self.assertEqual(result['boundaries_completed'],0)
        self.assertEqual(result['budget_status'],'INCOMPLETE')
        self.assertTrue(any(r['event']=='fault' for r in read_rows(out/'display_GPT.jsonl')))
        execution_faults = [r for r in read_rows(out/'events_GPT.jsonl')
                            if r.get('event') == 'execution_readback_FAIL']
        self.assertEqual(len(execution_faults), 1)
        self.assertEqual(execution_faults[0]['status'], 'FAIL')
        self.assertEqual(execution_faults[0]['sim_time'], 1)
        self.assertEqual(execution_faults[0]['step_in_cycle'], 1)
        self.assertEqual(execution_faults[0]['source'], 'arduino_gpio_or_communication')
        self.assertEqual(execution_faults[0]['response'], 'ABORT_UNTRUSTED_EXECUTION')
        self.assertIs(execution_faults[0]['ftsc_commanded'], False)

    def test_zero_step_episode_has_no_false_completed_latency(self):
        _, out = self.run_loop('empty',stop_at=0)
        result = json.loads((out/'timing_summary_GPT.json').read_text())
        self.assertEqual(result['measurement_status'],'INCOMPLETE')
        self.assertEqual(result['boundaries_completed'],0)

    def test_budget_exceedance_is_reported_without_a_new_control_rule(self):
        summary, out = self.run_loop('slow_policy',prediction_s=1.6)
        result = json.loads((out/'timing_summary_GPT.json').read_text())
        self.assertEqual(result['budget_status'],'EXCEEDED')
        self.assertEqual(result['budget_exceedances']['policy_to_boundary_ack_ms'],10)
        self.assertEqual(summary['fast_contingencies'],{'tl1':1,'tl2':0})
        # This virtual board does not emulate watchdog expiry; no physical claim.


if __name__ == '__main__':
    unittest.main()
