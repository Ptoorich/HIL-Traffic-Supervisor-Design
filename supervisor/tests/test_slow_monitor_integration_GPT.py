"""Provisional Slow Monitor thresholds and unknown-data behaviour."""
import math
import types
import unittest

from supervisor.slow_monitor_GPT import (aggregate_window, behavior_summary,
                                         evaluate_experimental_rule)


class SlowMonitorTest(unittest.TestCase):
    def setUp(self):
        self.cfg = types.SimpleNamespace(agents=('tl1','tl2'))

    def row(self, pending=150, growth=30, *, bound=5, alternate=0, complete=True):
        return {'complete_cycle':complete,
                'traffic_window':{'available':True, 'pending_end':pending, 'pending_change':growth},
                'behavior':{a:{'outward_bound_attempt_suffix_cycles':bound,
                               'alternating_applied_split_suffix_cycles':alternate} for a in self.cfg.agents},
                'junctions':{a:{'mode_before':'ASTA','mode_after':'ASTA'} for a in self.cfg.agents}}

    def test_both_traffic_and_behaviour_are_required(self):
        self.assertTrue(evaluate_experimental_rule(self.row(),self.cfg)['candidates']['tl1']['candidate'])
        for row in (self.row(pending=149),self.row(growth=29),self.row(bound=4)):
            self.assertFalse(evaluate_experimental_rule(row,self.cfg)['candidates']['tl1']['candidate'])
        alt=self.row(bound=0,alternate=6)
        self.assertTrue(evaluate_experimental_rule(alt,self.cfg)['candidates']['tl1']['candidate'])
        alt['junctions']['tl2']['mode_before']='CONTINGENCY'
        self.assertFalse(evaluate_experimental_rule(alt,self.cfg)['candidates']['tl2']['candidate'])

    def test_missing_nonfinite_or_incomplete_data_never_match(self):
        for row in (self.row(pending=None),self.row(growth=math.nan),self.row(complete=False)):
            result=evaluate_experimental_rule(row,self.cfg)
            self.assertNotEqual(result['status'],'evaluated')
            self.assertFalse(result['candidates'])

    def test_window_requires_three_complete_contiguous_cycles(self):
        def boundary(t,p):
            return {'sim_time':t,'pending_insertion':p,
                    'worst_lane_fill':{a:{'lane':a,'fill':.2} for a in self.cfg.agents}}
        def row(t):
            return {'complete_cycle':True,'start_time':t,'end_time':t+90,
                    'boundary_start':boundary(t,t//9),'boundary_end':boundary(t+90,(t+90)//9),
                    'step_counts':{'inserted':1,'arrived':2,'teleports_started':0},
                    'junctions':{a:{'mode_after':'ASTA'} for a in self.cfg.agents}}
        rows=[row(0),row(90),row(180)]
        result=aggregate_window(rows,self.cfg)
        self.assertTrue(result['available']); self.assertEqual(result['pending_change'],30)
        rows[1]['complete_cycle']=False
        self.assertFalse(aggregate_window(rows,self.cfg)['available'])
        rows[1]['complete_cycle']=True; rows[1]['end_time']=181
        self.assertFalse(aggregate_window(rows,self.cfg)['available'])

    def test_unselected_noop_is_not_an_outward_bound_attempt(self):
        records=[]
        for _ in range(6):
            records.append({'complete_cycle':True,'junctions':{'tl1':{'mode_before':'ASTA',
                'mode_after':'ASTA','proposed_action':1,'split_before':15,'split_after':15}}})
        result=behavior_summary(records,'tl1',15,63)
        self.assertEqual(result['outward_bound_attempt_suffix_cycles'],0)
        records[-1]['junctions']['tl1']['mode_after']='CONTINGENCY'
        self.assertEqual(behavior_summary(records,'tl1',15,63)['cycles_available'],0)

    def test_accepted_handback_cycle_begins_new_behavior_history(self):
        row={'complete_cycle':True,'junctions':{'tl1':{'mode_before':'CONTINGENCY',
             'mode_after':'ASTA','handback_accepted':True,'proposed_action':0,
             'split_before':31,'split_after':26}}}
        self.assertEqual(behavior_summary([row],'tl1',15,63)['cycles_available'],1)
        row['junctions']['tl1']['handback_accepted']=False
        self.assertEqual(behavior_summary([row],'tl1',15,63)['cycles_available'],0)


if __name__ == '__main__':
    unittest.main()
