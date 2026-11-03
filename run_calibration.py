"""
Run controllers on the standard time-of-day demand and log per-lane sensor
data at every cycle boundary, for Traffic Classifier calibration.

Run from the agents folder (full 19,800 s episodes, so allow time):
    py run_calibration.py --controller static               # healthy, 10 seeds
    py run_calibration.py --controller fixed                # 39/39, stressed
    py run_calibration.py --controller rl --kind marl --algo ppo \
        --model models/marl_ppo_ps_2int_260k --name marl_ppo_ps
    py run_calibration.py --controller static --seeds 1     # quick check

Output: supervisor_logs/calibration_<name>.jsonl (appended; delete old files
to start fresh). Then:  py calibrate_thresholds.py --healthy ... --stressed ...
"""
import argparse
import os

import tod_plan as tp
import tod_eval_common as ec
from sumo_env_marl_ps import SumoMARLEnvPS
from supervisor.calibration_logger import LoggingController
from supervisor.config import load_config, validate_against_env

parser = argparse.ArgumentParser()
parser.add_argument("--controller", choices=["static", "fixed", "rl"], required=True)
parser.add_argument("--kind", choices=["marl", "sarl"], help="rl only")
parser.add_argument("--algo", choices=["ppo", "dqn"], help="rl only")
parser.add_argument("--model", help="rl only: SB3 .zip path")
parser.add_argument("--name", help="label for the log (default: controller name)")
parser.add_argument("--peak-scale", type=float, default=ec.DEFAULT_SCALE)
parser.add_argument("--seeds", type=int, default=10)
parser.add_argument("--routes-src", default="2int_files/routes.xml")
parser.add_argument("--gen-dir", default=None)
parser.add_argument("--config", default=None, help="supervisor_config.toml path")
parser.add_argument("--out", default=None)
args = parser.parse_args()

SEEDS = ec.STANDARD_SEEDS[:args.seeds]
GEN_DIR = args.gen_dir or ec.default_gen_dir(args.peak_scale)
config = ec.make_config()
sup_cfg = load_config(args.config)
PEAK_RATE = args.peak_scale * tp.ORIGINAL_RATE_VPH
mix = tp.load_route_mix(args.routes_src)

if args.controller == "static":
    schedule = tp.build_schedule(mix, config["lane_map"], PEAK_RATE,
                                 config["cycle_length"], config["yellow_time"],
                                 config["allred_time"], config["s_lb"],
                                 config["s_ub"])
    inner = ec.StaticController(schedule)
elif args.controller == "fixed":
    inner = ec.FixedController()
else:
    if not (args.kind and args.algo and args.model and args.name):
        parser.error("--controller rl needs --kind --algo --model --name")
    from stable_baselines3 import DQN, PPO
    model = (PPO if args.algo == "ppo" else DQN).load(args.model)
    Policy = ec.MarlPolicy if args.kind == "marl" else ec.SarlPolicy
    inner = Policy(model, args.name, deterministic=True)

name = args.name or inner.name
out = args.out or f"supervisor_logs/calibration_{name}.jsonl"
os.makedirs(GEN_DIR, exist_ok=True)

env = SumoMARLEnvPS(config)
validate_against_env(sup_cfg, env)          # config must describe this env
ctrl = LoggingController(inner, sup_cfg, out, run_id=name)
ctrl.name = name

for seed in SEEDS:
    route_file = os.path.join(GEN_DIR, f"routes_seed{seed}.xml")
    tp.generate_route_file(route_file, mix, seed, PEAK_RATE)
    ctrl.seed = seed
    cycles, trips = ec.run_episode(env, seed, route_file, ctrl)
    print(f"seed {seed}: {len(cycles)} cycles logged -> {out}")

ctrl.close()
env.close()
