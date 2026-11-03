"""
Evaluate ONE trained 2-intersection RL model on the same time-varying demand,
env settings, seeds and metric code as baseline_tod_2int.py.

Run from the agents folder. Examples (paths as in your old eval scripts):
    py eval_rl_2int.py --kind marl --algo ppo --model models/marl_ppo_ps_2int_260k --name marl_ppo_ps
    py eval_rl_2int.py --kind marl --algo dqn --model models/marl_dqn_ps_2int_260k --name marl_dqn_ps
    py eval_rl_2int.py --kind sarl --algo ppo --model sarl_ppo_2int_260k --name sarl_ppo
    py eval_rl_2int.py --kind sarl --algo dqn --model sarl_dqn_2int_260k --name sarl_dqn
Add --seeds 1 for a quick check. Actions are deterministic, as in the old
eval scripts (use --stochastic to sample instead).
"""
import argparse
import os

import tod_plan as tp
import tod_eval_common as ec
from sumo_env_marl_ps import SumoMARLEnvPS

parser = argparse.ArgumentParser()
parser.add_argument("--kind", choices=["marl", "sarl"], required=True)
parser.add_argument("--algo", choices=["ppo", "dqn"], required=True)
parser.add_argument("--model", required=True,
                    help="path to the SB3 .zip (with or without .zip)")
parser.add_argument("--name", required=True, help="label used in the output")
parser.add_argument("--peak-scale", type=float, default=ec.DEFAULT_SCALE)
parser.add_argument("--seeds", type=int, default=10)
parser.add_argument("--stochastic", action="store_true")
parser.add_argument("--routes-src", default="2int_files/routes.xml")
parser.add_argument("--gen-dir", default=None)
parser.add_argument("--out", default=None)
args = parser.parse_args()

from stable_baselines3 import DQN, PPO
Algo = PPO if args.algo == "ppo" else DQN
model = Algo.load(args.model)

SEEDS = ec.STANDARD_SEEDS[:args.seeds]
GEN_DIR = args.gen_dir or ec.default_gen_dir(args.peak_scale)
OUT = args.out or f"results/{args.name}_tod_eval.json"
config = ec.make_config()
PEAK_RATE = args.peak_scale * tp.ORIGINAL_RATE_VPH
mix = tp.load_route_mix(args.routes_src)

det = not args.stochastic
Policy = ec.MarlPolicy if args.kind == "marl" else ec.SarlPolicy
controller = Policy(model, args.name, deterministic=det)

# the model's input size must match what we feed it
expected = (model.observation_space.shape[0])
env = SumoMARLEnvPS(config)
fed = env.obs_size if args.kind == "marl" else \
    sum(len(v) for v in config["lane_map"].values()) + len(config["agent_ids"])
if expected != fed:
    raise SystemExit(f"Model expects {expected} observation values but this "
                     f"{args.kind} setup provides {fed}; wrong --kind/--model?")
print(f"Evaluating {args.name} ({args.kind} {args.algo}) from {args.model}; "
      f"deterministic={det}; peak scale {args.peak_scale}; seeds {SEEDS}\n")

os.makedirs(GEN_DIR, exist_ok=True)
results = {
    "kind": "rl", "name": args.name, "model": args.model,
    "algo": args.algo, "agent_type": args.kind, "deterministic": det,
    "config": dict(config), "peak_scale": args.peak_scale,
    "seeds": SEEDS, "route_md5": {}, args.name: [],
}
for seed in SEEDS:
    route_file = os.path.join(GEN_DIR, f"routes_seed{seed}.xml")
    deps = tp.generate_route_file(route_file, mix, seed, PEAK_RATE)
    results["route_md5"][str(seed)] = ec.file_md5(route_file)
    cycles, trips = ec.run_episode(env, seed, route_file, controller)
    res = ec.summarise(cycles, trips, deps, config["agent_ids"])
    results[args.name].append(res)
    print(ec.seed_line(seed, args.name, res["overall"]))

env.close()
ec.print_summary(results, [args.name])
ec.save_json(OUT, results)