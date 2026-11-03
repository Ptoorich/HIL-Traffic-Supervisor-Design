"""
Runs the REAL SnapshotTaker and Fast Gate against live SUMO on a generated toy
junction. Skipped automatically when SUMO is not installed.

Verifies, on real TraCI: the API calls and units the snapshot relies on, that
the net's own phase strings pass the Conflict Monitor, and that the Gridlock
rule fires on a genuinely jammed lane (occupancy ~0.64, not 0.9).
"""
import re
import shutil
import subprocess
from types import SimpleNamespace

import pytest

from supervisor.config import DEFAULT_CONFIG_PATH, load_config
from supervisor.conflicts import ConflictMatrix, parse_net
from supervisor.datatypes import Overall
from supervisor.fast_gate import FastGate
from supervisor.snapshot import SnapshotTaker

from .helpers import proposal

pytestmark = pytest.mark.skipif(
    not (shutil.which("sumo") and shutil.which("netgenerate")), reason="SUMO not installed")

TOY_LANES = {"tl1": [("A1B1_0", 11), ("B0B1_0", 11)], "tl2": [("B2B1_0", 11), ("C1B1_0", 11)]}


@pytest.fixture(scope="module")
def toy(tmp_path_factory):
    d = tmp_path_factory.mktemp("toy")
    subprocess.run(["netgenerate", "--grid", "--grid.number", "3", "--grid.length", "100",
                    "--tls.set", "B1", "--default.lanenumber", "1", "-o", str(d / "toy.net.xml")],
                   check=True, capture_output=True)
    (d / "r.rou.xml").write_text(
        '<routes><route id="r" edges="A1B1 B1C1"/>'
        '<flow id="f" route="r" begin="0" end="200" number="60"/></routes>')
    (d / "t.sumocfg").write_text('<configuration><input><net-file value="toy.net.xml"/>'
                                 '<route-files value="r.rou.xml"/></input></configuration>')
    text = DEFAULT_CONFIG_PATH.read_text()
    text = text[:text.index("# Lane capacities")]
    for agent, lanes in TOY_LANES.items():
        for lane, cap in lanes:
            text += f'[lanes."{lane}"]\nagent = "{agent}"\ncapacity = {cap}\nexcluded = false\n'
    (d / "toy.toml").write_text(text)
    return d


def test_real_traci_snapshot_gate_and_gridlock(toy):
    import traci
    cfg = load_config(toy / "toy.toml")
    one = parse_net(toy / "toy.net.xml", {"tl1": "B1"})       # both toy agents sit on junction B1
    matrix = ConflictMatrix({"tl1": one.junctions["tl1"], "tl2": one.junctions["tl1"]})
    traci.start(["sumo", "-c", str(toy / "t.sumocfg"), "--no-step-log", "true", "--no-warnings", "true"])
    try:
        tl = "B1"
        phases = traci.trafficlight.getAllProgramLogics(tl)[0].phases
        env = SimpleNamespace(phase_splits={"tl1": 39.0, "tl2": 39.0}, tl_map={"tl1": tl, "tl2": tl},
                              _base_states={a: tuple(p.state for p in phases) for a in cfg.agents})
        taker = SnapshotTaker(cfg)
        with FastGate(cfg, matrix) as gate:
            props = {a: proposal(a) for a in cfg.agents}

            # 1. empty network, real signal program
            res = gate.evaluate(taker.take(env), props)
            assert all(r.overall != Overall.VETO for r in res.values()), \
                [(r.agent, r.triggering, [c.details for c in r.checks if c.status.value == "FAIL"]) for r in res.values()]
            s0 = taker.take(env)["tl1"]
            assert s0.lanes[0].mean_speed == pytest.approx(s0.lanes[0].max_speed)   # empty lane = speed limit
            assert len(s0.live_state) == matrix.junctions["tl1"].n_links

            # 2. force all red, let the approach jam, read real sensors
            traci.trafficlight.setRedYellowGreenState(tl, "r" * len(s0.live_state))
            for _ in range(300):
                traci.simulationStep()
            snap = taker.take(env)
            lane = next(r for r in snap["tl1"].lanes if r.lane_id == "A1B1_0")
            assert 0.55 < lane.occupancy <= 0.70               # fraction, vehicle length only
            assert lane.halting == lane.count == 11 and lane.mean_speed == 0.0
            res = gate.evaluate(snap, props)
            assert res["tl1"].overall == Overall.VETO and res["tl1"].triggering == ("traffic_classifier",)
            assert res["tl1"].classifier_label == "GRIDLOCK"
            assert res["tl2"].overall != Overall.VETO            # tl2's lanes are empty
    finally:
        traci.close()
