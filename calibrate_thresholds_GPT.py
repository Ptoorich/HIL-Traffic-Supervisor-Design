"""Report physical traffic classifications and sensor diagnostics from JSONL logs.

Usage: py calibrate_thresholds_GPT.py --healthy static.jsonl --stressed fixed.jsonl
Warm-up and cool-down are excluded; no thresholds are inferred or written.
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path


from supervisor.checks import traffic_classifier
from supervisor.config import load_config
from supervisor.datatypes import JunctionSnapshot, LaneReading, Proposal, Status

parser = argparse.ArgumentParser()
parser.add_argument("--healthy", nargs="+", required=True)
parser.add_argument("--stressed", nargs="*", default=[])
parser.add_argument("--config", default=None)
args = parser.parse_args()
cfg = load_config(args.config)
EXCLUDE_PERIODS = {"warmup", "cooldown", "n/a"}


def load(paths):
    rows = []
    for p in paths:
        with open(p, encoding="utf-8") as f:
            rows += [json.loads(line) for line in f if line.strip()]
    return [r for r in rows if r["period"] not in EXCLUDE_PERIODS]


def group_snapshots(rows):
    """{(run, seed, t, agent): JunctionSnapshot} rebuilt from the log rows."""
    g = defaultdict(list)
    for r in rows:
        g[(r["run"], r["seed"], r["t"], r["agent"])].append(r)
    snaps = {}
    for (run, seed, t, agent), rs in g.items():
        lanes = tuple(LaneReading(r["lane"], r["halting"], r["count"],
                                  r["occupancy"], r["mean_speed"], r["length"],
                                  r["max_speed"], r["capacity"], r["excluded"])
                      for r in rs)
        snaps[(run, seed, t, agent)] = JunctionSnapshot(
            agent, "", float(t), lanes, "", (), rs[0]["split"])
    return snaps


healthy = load(args.healthy)
if not healthy:
    raise SystemExit("no healthy samples after excluding warm-up/cool-down")
hs = group_snapshots(healthy)

print("=== Physical classifier settings ===")
print(f"FREE mean fill <= {cfg.classifier.free_max:.3f} with no jammed lanes; "
      "HEAVY for jammed lanes below gridlock threshold; GRIDLOCK vetoes.")

print("\n=== Data checks behind the config assumptions (all supplied runs) ===")
allrows = healthy + load(args.stressed)
by_lane = defaultdict(list)
for r in allrows:
    by_lane[r["lane"]].append(r)
print(f"{'lane':22s} {'cap':>4s} {'maxcnt':>6s} {'maxhalt':>7s} {'maxocc':>7s} "
      f"{'jamocc':>7s} {'max v/limit':>11s}")
for lane, rs in by_lane.items():
    cap, length = rs[0]["capacity"], rs[0]["length"]
    jam = min(1.0, cap * cfg.classifier.vehicle_length_m / length)
    ratio = max((r["mean_speed"] / r["max_speed"] for r in rs if r["count"] > 0),
                default=float("nan"))
    print(f"{lane:22s} {cap:4d} {max(r['count'] for r in rs):6d} "
          f"{max(r['halting'] for r in rs):7d} "
          f"{max(r['occupancy'] for r in rs):7.3f} {jam:7.3f} {ratio:11.2f}")

print("\n=== How often the rules fire per run (all periods after warm-up) ===")
for label, rows in [("healthy", healthy), ("stressed", load(args.stressed))]:
    if not rows:
        continue
    snaps = group_snapshots(rows)
    stats = defaultdict(lambda: defaultdict(int))
    for (run, seed, t, agent), s in snaps.items():
        res = traffic_classifier.check(s, Proposal(agent, s.current_split, 0,
                                                   s.current_split, "calib"), cfg)
        stats[(run, agent)]["n"] += 1
        stats[(run, agent)][res.reason] += 1
    for (run, agent), st in sorted(stats.items()):
        n = st.pop("n")
        parts = ", ".join(f"{k} {100 * v / n:.1f}%" for k, v in sorted(st.items()))
        print(f"{label:9s} {run:16s} {agent:5s} n={n:5d}: {parts}")
