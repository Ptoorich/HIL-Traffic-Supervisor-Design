import json
import pytest
from supervisor.fast_gate import FastGate
from supervisor.supervised_controller_GPT import SupervisedController
from .helpers import healthy_snapshot, jammed

class Env:
    def __init__(self, cfg):
        self.agent_ids = list(cfg.agents)
        self.tl_map = {a: f"tl_{a}" for a in cfg.agents}
        self.cycle_length, self.yellow_time, self.allred_time = 90, 3, 3
        self.lane_map = {a: [l.lane_id for l in cfg.lanes_of(a)] for a in cfg.agents}
        self.phase_splits = {a: 39.0 for a in cfg.agents}
    def set_splits(self, splits): self.phase_splits.update(splits)

class Taker:
    def __init__(self, cfg): self.cfg, self.jams, self.fail = cfg, {}, False
    def clear_cache(self): pass
    def take(self, env, current_splits):
        if self.fail: raise RuntimeError('sensor failure')
        out = {}
        for a in self.cfg.agents:
            s = healthy_snapshot(self.cfg, a, split=current_splits[a])
            for lane in self.jams.get(a, []): s = jammed(s, lane)
            out[a] = s
        return out

class Inner:
    name='test'
    def __init__(self, actions=None, change=None):
        self.actions = actions if actions is not None else {'tl1':2, 'tl2':1}
        self.change = change
    def act(self, env, obs, t):
        if self.change: env.set_splits(self.change)
        return self.actions

@pytest.fixture
def setup(cfg, matrix, tmp_path):
    env, taker = Env(cfg), Taker(cfg)
    gate = FastGate(cfg, matrix)
    def build(inner):
        return SupervisedController(inner, cfg, matrix, tmp_path/'events.jsonl', snapshot_taker=taker, gate=gate)
    yield env, taker, build, tmp_path/'events.jsonl'
    gate.close()

def log(path): return [json.loads(s) for s in path.read_text().splitlines()]

def test_approved_actions_and_noop_agent(setup):
    env,taker,build,path=setup
    with build(Inner({'tl1':2})) as ctrl:
        assert ctrl.act(env, {}, 0) == {'tl1':2, 'tl2':1}
        assert env.phase_splits == {'tl1':39, 'tl2':39}
    d=log(path)[0]['decisions']
    assert d['tl1']['proposal']['requested_split'] == 44
    assert d['tl2']['gate']['overall'] == 'APPROVE'

def test_single_junction_veto_latches_and_other_continues(setup,cfg):
    env,taker,build,path=setup
    taker.jams['tl1']=[x.lane_id for x in cfg.active_lanes_of('tl1')[:3]]
    with build(Inner({'tl1':2,'tl2':2})) as ctrl:
        assert ctrl.act(env,{},0) == {'tl1':1,'tl2':2}
        assert env.phase_splits == {'tl1':31,'tl2':39}
        taker.jams.clear()
        assert ctrl.act(env,{},90) == {'tl1':1,'tl2':2}
        assert ctrl.modes == {'tl1':'CONTINGENCY','tl2':'ASTA'}
        assert env.phase_splits['tl1'] == 31
        ctrl.reset_episode('next')
        assert ctrl.modes['tl1'] == 'ASTA'
    records=log(path)
    assert records[0]['decisions']['tl1']['entered_contingency']
    assert not records[1]['decisions']['tl1']['entered_contingency']

def test_static_side_effect_checked_before_accepting(setup,cfg):
    env,taker,build,path=setup
    taker.jams['tl1']=[x.lane_id for x in cfg.active_lanes_of('tl1')[:3]]
    with build(Inner({'tl1':1,'tl2':1}, {'tl1':55,'tl2':50})) as ctrl:
        assert ctrl.act(env,{},0) == {'tl1':1,'tl2':1}
        assert env.phase_splits == {'tl1':31,'tl2':50}
    d=log(path)[0]['decisions']
    assert d['tl1']['proposal']['current_split'] == 39
    assert d['tl1']['proposal']['requested_split'] == 55

def test_unclipped_requested_split_warn(setup):
    env,taker,build,path=setup
    env.phase_splits['tl1']=15
    with build(Inner({'tl1':0})) as ctrl:
        assert ctrl.act(env,{},0)['tl1'] == 0
    d=log(path)[0]['decisions']['tl1']
    assert d['proposal']['requested_split']==10
    assert d['gate']['overall']=='APPROVE_WARN'

def test_snapshot_failure_latches_both(setup):
    env,taker,build,path=setup
    taker.fail=True
    with build(Inner()) as ctrl:
        assert ctrl.act(env,{},0) == {'tl1':1,'tl2':1}
        assert env.phase_splits == {'tl1':31,'tl2':59}
        assert all(v=='CONTINGENCY' for v in ctrl.modes.values())
    assert 'SNAPSHOT_ERROR' in log(path)[0]['decisions']['tl1']['reason']

def test_invalid_action_vetoes_only_affected_junction(setup):
    env,taker,build,path=setup
    with build(Inner({'tl1':9,'tl2':2})) as ctrl:
        assert ctrl.act(env,{},0)=={'tl1':1,'tl2':2}
        assert ctrl.modes=={'tl1':'CONTINGENCY','tl2':'ASTA'}


def test_fixture_mapping_must_match_env(setup):
    env,taker,build,path=setup
    env.tl_map['tl1'] = 'wrong'
    with build(Inner()) as ctrl, pytest.raises(ValueError, match='fixture'):
        ctrl.act(env, {}, 0)
