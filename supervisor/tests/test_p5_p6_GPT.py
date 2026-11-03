"""Regression tests: python -m unittest supervisor.tests.test_p5_p6_GPT.
Set ASTA_NET to the supplied actual net to run real-network tests.
"""
import dataclasses
import os
import tempfile
import unittest
from pathlib import Path
from supervisor.config import load_config
from supervisor.datatypes import Status
from supervisor.checks import traffic_classifier as tc
from supervisor.conflicts import parse_net, save_fixture, load_fixture, ConflictMatrixError
from supervisor.tests.helpers import healthy_snapshot, with_lane, jammed, proposal

class PhysicalClassifierTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config()
        self.s = healthy_snapshot(self.cfg, 'tl1')
    def check(self, s):
        return tc.check(s, proposal(), self.cfg)
    def test_free_boundary_and_light(self):
        s = with_lane(self.s, '596638461#0_0', halting=5, count=5, mean_speed=2)
        self.assertAlmostEqual(tc.mean_fill(s), .1)
        self.assertEqual(self.check(s).reason, 'FREE')
        s = with_lane(s, '596638461#0_0', halting=6, count=6, mean_speed=2)
        self.assertEqual(self.check(s).reason, 'LIGHT')
    def test_one_two_three_jams(self):
        s = self.s
        for lane, reason in zip(self.cfg.active_lanes_of('tl1')[:3], ['HEAVY', 'HEAVY', 'GRIDLOCK']):
            s = jammed(s, lane.lane_id)
            self.assertEqual(self.check(s).reason, reason)
    def test_excluded_and_speed_boundary(self):
        s = jammed(healthy_snapshot(self.cfg, 'tl2'), '669676574#0_0')
        self.assertEqual(tc.check(s, proposal('tl2'), self.cfg).reason, 'FREE')
        s = jammed(self.s, '596638461#0_0')
        s = with_lane(s, '596638461#0_0', mean_speed=0.5)
        self.assertNotEqual(self.check(s).reason, 'HEAVY')
    def test_invalid(self):
        s = with_lane(self.s, '596638461#0_0', occupancy=float('nan'))
        self.assertEqual(self.check(s).status, Status.FAIL)

class NetworkTests(unittest.TestCase):
    def test_real_network(self):
        path = os.environ.get('ASTA_NET')
        if not path: self.skipTest('set ASTA_NET to actual network')
        m = parse_net(path, {'tl1':'30406197','tl2':'cluster_30406198_7161921427_7161921428'})
        self.assertEqual((m.junctions['tl1'].n_links,m.junctions['tl2'].n_links),(8,27))
        j = m.junctions['tl2']
        self.assertEqual(len(j.foes),121)
        self.assertEqual(j.asymmetric_pairs, frozenset({(8,22),(8,23)}))
        self.assertTrue({(8,22),(8,23)} <= j.foes)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'fixture.json'; save_fixture(m,p); self.assertEqual(load_fixture(p),m)
    def test_bad_matrix(self):
        for bits in ('01', 'x0', '0'):
            xml=f'<net><tlLogic id="J"><phase state="rr"/></tlLogic><junction id="J"><request index="0" foes="{bits}"/><request index="1" foes="00"/></junction></net>'
            with tempfile.TemporaryDirectory() as d:
                p=Path(d)/'net.xml'; p.write_text(xml)
                with self.assertRaises(ConflictMatrixError): parse_net(p, {'a':'J'})

if __name__ == '__main__': unittest.main()

class CalibrationRegressionTests(unittest.TestCase):
    def test_real_snapshots(self):
        import json
        from supervisor.datatypes import LaneReading, JunctionSnapshot, Proposal
        samples = json.loads(Path(__file__).with_name('calibration_samples_GPT.json').read_text())
        self.assertEqual(len(samples), 6)
        cfg = load_config()
        for sample in samples:
            rows = sample['rows']
            lanes = tuple(LaneReading(r['lane'], r['halting'], r['count'], r['occupancy'], r['mean_speed'], r['length'], r['max_speed'], r['capacity'], r['excluded']) for r in rows)
            r = rows[0]
            snap = JunctionSnapshot(r['agent'], '', r['t'], lanes, '', (), r['split'])
            res = tc.check(snap, Proposal(snap.agent, snap.current_split, 0, snap.current_split, 'calib'), cfg)
            self.assertEqual(res.reason, sample['expected'])
