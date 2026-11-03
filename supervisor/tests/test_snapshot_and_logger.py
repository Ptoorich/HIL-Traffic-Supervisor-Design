import dataclasses
import json
from types import SimpleNamespace

from supervisor.calibration_logger import LoggingController
from supervisor.config import SnapshotCfg
from supervisor.snapshot import SnapshotTaker


def fake_traci(cfg, occ_raw=0.4):
    calls = []
    def rec(name, val):
        def f(*a):
            calls.append(name)
            return val(*a) if callable(val) else val
        return f
    return SimpleNamespace(
        calls=calls,
        simulation=SimpleNamespace(getTime=rec("time", 900.0)),
        lane=SimpleNamespace(
            getLength=rec("len", 80.0), getMaxSpeed=rec("vmax", 13.89),
            getLastStepHaltingNumber=rec("halt", 2), getLastStepVehicleNumber=rec("cnt", 3),
            getLastStepOccupancy=rec("occ", occ_raw), getLastStepMeanSpeed=rec("v", 4.0)),
        trafficlight=SimpleNamespace(getRedYellowGreenState=rec("tls", "rrrr")))


def fake_env(cfg):
    return SimpleNamespace(
        phase_splits={a: 39.0 for a in cfg.agents},
        tl_map={"tl1": "T1", "tl2": "T2"},
        _base_states={a: ("GGrr", "yyrr", "rrGG", "rryy") for a in cfg.agents})


def test_snapshot_contents(cfg):
    snaps = SnapshotTaker(cfg, fake_traci(cfg)).take(fake_env(cfg))
    s = snaps["tl1"]
    assert len(s.lanes) == 5 and s.tl_id == "T1" and s.sim_time == 900.0
    r = s.lanes[0]
    assert (r.halting, r.count, r.occupancy, r.mean_speed, r.length) == (2, 3, 0.4, 4.0, 80.0)
    assert s.commanded_phase_strings == ("GGrr", "yyrr", "rrGG", "rryy")
    assert len(snaps["tl2"].lanes) == 7            # excluded lanes are still read (validator range check)
    assert {l.lane_id for l in snaps["tl2"].lanes if l.excluded} == {"669676574#0_0", "-669676574#8_0"}


def test_percent_unit_is_normalised_to_fraction(cfg):
    c = dataclasses.replace(cfg, snapshot=SnapshotCfg("percent"))
    s = SnapshotTaker(c, fake_traci(c, occ_raw=40.0)).take(fake_env(c))["tl1"]
    assert abs(s.lanes[0].occupancy - 0.4) < 1e-12


def test_static_lane_data_is_read_once(cfg):
    T = fake_traci(cfg)
    taker = SnapshotTaker(cfg, T)
    taker.take(fake_env(cfg)); taker.take(fake_env(cfg))
    assert T.calls.count("len") == 12 and T.calls.count("halt") == 24


def test_explicit_current_splits_override_env(cfg):
    env = fake_env(cfg)
    env.phase_splits = {"tl1": 99.0, "tl2": 99.0}      # contaminated by the inner controller
    s = SnapshotTaker(cfg, fake_traci(cfg)).take(env, current_splits={"tl1": 39.0, "tl2": 31.0})
    assert (s["tl1"].current_split, s["tl2"].current_split) == (39.0, 31.0)


class Inner:
    name = "static"
    def act(self, env, obs, t):
        env.phase_splits = {a: 31.0 for a in env.phase_splits}   # like StaticController.set_splits
        return {a: 1 for a in env.phase_splits}


def test_logger_writes_rows_reads_before_inner_acts_and_passes_actions_through(cfg, tmp_path):
    out = tmp_path / "logs" / "cal.jsonl"
    ctrl = LoggingController(Inner(), cfg, out, period_of=lambda t: "AM_peak", traci_mod=fake_traci(cfg))
    ctrl.seed = 42
    env = fake_env(cfg)
    actions = ctrl.act(env, {}, 900)
    ctrl.act(env, {}, 990)
    ctrl.close()
    assert actions == {"tl1": 1, "tl2": 1} and ctrl.name == "static"
    rows = [json.loads(l) for l in out.read_text().splitlines()]
    assert len(rows) == 2 * 12
    first = rows[0]
    assert (first["seed"], first["t"], first["period"], first["agent"]) == (42, 900, "AM_peak", "tl1")
    assert first["split"] == 39.0                 # the split BEFORE the inner controller changed it
    assert rows[12]["split"] == 31.0              # next cycle sees the new split
    assert {"halting", "count", "occupancy", "mean_speed", "length", "capacity", "excluded"} <= set(first)
