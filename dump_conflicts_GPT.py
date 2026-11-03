"""
Write the conflict matrix fixture the Conflict Monitor loads at startup.
Run once from the agents folder (no SUMO process needed, it reads the net file):

    py dump_conflicts_GPT.py
    py dump_conflicts_GPT.py --net 2int_files/osm.net.xml.gz     # if auto-detect fails

Prints each junction's link count, number of foe pairs, and the net's own phase
strings checked against the rule (G+G fail, g+g warn). Asymmetric relationships are recorded and treated conservatively.
"""
import argparse
import os
import xml.etree.ElementTree as ET

import tod_eval_common as ec
from supervisor.checks.conflict_monitor import check_state
from supervisor.config import load_config
from supervisor.conflicts import parse_net, save_fixture

parser = argparse.ArgumentParser()
parser.add_argument("--net", default=None, help="net file (default: from the sumocfg)")
parser.add_argument("--config", default=None)
args = parser.parse_args()

config = ec.make_config()
sup = load_config(args.config)
net = args.net
if net is None:
    sumocfg = config["sumocfg"]
    nf = ET.parse(sumocfg).getroot().find(".//net-file")
    if nf is None:
        raise SystemExit(f"no <net-file> in {sumocfg}; pass --net")
    net = os.path.join(os.path.dirname(sumocfg), nf.get("value"))
print(f"net file: {net}")

matrix = parse_net(net, config["tl_map"])
save_fixture(matrix, sup.conflicts.fixture_path, source=net)
for agent, j in matrix.junctions.items():
    print(f"\n{agent} ({j.tl_id}): {j.n_links} links, {len(j.foes)} foe pairs")
    if j.asymmetric_pairs:
        print(f"  asymmetric directed foes: {sorted(j.asymmetric_pairs)}")
    for k, s in enumerate(j.net_phase_states):
        hard, soft = check_state(s, j.foes, j.n_links, f"phase{k}")
        print(f"  phase {k}: {s}  G+G conflicts={len(hard)}  g+g={len(soft)}")
print(f"\nwritten to {sup.conflicts.fixture_path}")
