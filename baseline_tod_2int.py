"""
Time-of-day fixed-time baseline — 2 intersection network.

Runs ONE continuous compressed day per seed:
    warm-up (15 min) | AM peak | midday | PM peak | evening | night | cool-down
with time-varying demand (generated per seed) and two controllers on the
SAME demand:
    fixed : 39/39 split all day (the old baseline, now on varying demand)
    static: Webster split derived from demand. NOTE: the route mix is the
            same in every period, so this is ONE static split all day, NOT
            time-of-day varying (plans only differ if demand mix differs)
Both use a fixed 90 s cycle and 3 s yellow + 3 s all-red per phase.

Metrics are reported per period (warm-up/cool-down excluded) so RL agents
can be compared on exactly the same breakdown.

Usage (from the agents folder, like the old script):
    py baseline_tod_2int.py                  # 10 seeds, peak scale 0.37
    py baseline_tod_2int.py --peak-scale 0.55
    py baseline_tod_2int.py --seeds 2        # quick smoke test
"""
import argparse
import os

import numpy as np

import tod_plan as tp
import tod_eval_common as ec
from sumo_env_marl_ps import SumoMARLEnvPS

parser = argparse.ArgumentParser()
parser.add_argument("--peak-scale", type=float, default=ec.DEFAULT_SCALE,
                    help="peak demand as a fraction of the original "
                         "7200 veh/h route file")
parser.add_argument("--seeds", type=int, default=10,
                    help="how many of the 10 standard seeds to run")
parser.add_argument("--tidal", action="store_true",
                    help="use the ILLUSTRATIVE directional bias in "
                         "tod_plan.TIDAL_EXAMPLE (off = same mix all day)")
parser.add_argument("--routes-src", default="2int_files/routes.xml",
                    help="original per-vehicle route file (for the route mix)")
parser.add_argument("--gen-dir", default=None,
                    help="default: 2int_files/generated_tod_s<scale>")
parser.add_argument("--out", default="results/baseline_tod_2int_results.json")
args = parser.parse_args()

SEEDS = ec.STANDARD_SEEDS[:args.seeds]
GEN_DIR = args.gen_dir or ec.default_gen_dir(args.peak_scale)
config = ec.make_config()

TIDAL = tp.TIDAL_EXAMPLE if args.tidal else None
PEAK_RATE = args.peak_scale * tp.ORIGINAL_RATE_VPH
mix       = tp.load_route_mix(args.routes_src)
schedule  = tp.build_schedule(mix, config["lane_map"], PEAK_RATE,
                              config["cycle_length"], config["yellow_time"],
                              config["allred_time"], config["s_lb"],
                              config["s_ub"], tidal=TIDAL)

# ── Show the derived plan and its SARTSM checks ────────────────────────
inter = config["yellow_time"] + config["allred_time"]
print(f"Peak demand: {PEAK_RATE:.0f} veh/h "
      f"(scale {args.peak_scale} x {tp.ORIGINAL_RATE_VPH:.0f}), "
      f"cycle {config['cycle_length']} s, intergreen {inter} s/phase\n")
print(f"{'period':8s} {'veh/h':>6s}  junction  phaseA  phaseB   yA    yB    "
      f"X(90s)  C_webster")
for p in schedule:
    for agent, d in p["diag"].items():
        gB = config["cycle_length"] - d["split"] - 2 * inter
        cw = f"{d['C_webster']:.0f}s" if d["C_webster"] else "n/a"
        print(f"{p['name']:8s} {p['rate_vph']:6.0f}  {agent:8s}  "
              f"{d['split']:4d}s  {gB:4d}s   {d['yA']:.2f}  {d['yB']:.2f}  "
              f"{d['X']:6.2f}  {cw:>8s}")
    for w in p["warnings"]:
        print(f"   SARTSM warning: {w}")
sat = [(p["name"], a) for p in schedule for a, d in p["diag"].items()
       if d["X"] > 0.95]
if sat:
    print("\nWARNING: junction at/over capacity for a 90 s cycle (X > 0.95) "
          f"in: {sat}. Consider a lower --peak-scale.")
if len({tuple(sorted(p["splits"].items())) for p in schedule}) == 1:
    print("\nNOTE: every period got the SAME split, so this baseline is a single "
          "optimised STATIC split, not time-of-day varying. The route mix does not change by period; use --tidal "
          "(illustrative) or supply a real directional pattern.")
print()

os.makedirs("results", exist_ok=True)
os.makedirs(GEN_DIR, exist_ok=True)
env = SumoMARLEnvPS(config)

controllers = [ec.FixedController(), ec.StaticController(schedule)]
NAMES = [c.name for c in controllers]

results = {
    "kind": "baseline",
    "config": {k: v for k, v in config.items()},
    "peak_scale": args.peak_scale,
    "tidal": TIDAL,
    "demand_index_ASSUMED": dict(tp.PERIODS),
    "schedule": [{"name": p["name"], "start": p["start"], "end": p["end"],
                  "rate_vph": p["rate_vph"], "splits": p["splits"],
                  "X": {a: d["X"] for a, d in p["diag"].items()}}
                 for p in schedule],
    "seeds": SEEDS,
    "route_md5": {},
}
for n in NAMES:
    results[n] = []

for seed in SEEDS:
    route_file = os.path.join(GEN_DIR, f"routes_seed{seed}.xml")
    deps = tp.generate_route_file(route_file, mix, seed, PEAK_RATE,
                                  tidal=TIDAL)
    results["route_md5"][str(seed)] = ec.file_md5(route_file)
    for ctrl in controllers:
        cycles, trips = ec.run_episode(env, seed, route_file, ctrl)
        res = ec.summarise(cycles, trips, deps, config["agent_ids"])
        results[ctrl.name].append(res)
        print(ec.seed_line(seed, ctrl.name, res["overall"]))

env.close()
ec.print_summary(results, NAMES)
ec.save_json(args.out, results)