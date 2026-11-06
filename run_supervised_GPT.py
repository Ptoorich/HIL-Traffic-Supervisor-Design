"""Run a supervised evaluation using the existing SUMO evaluation project.

Examples (run from agents folder):
  py run_supervised_GPT.py --controller static --seeds 1
  py run_supervised_GPT.py --controller fixed --seeds 1
  py run_supervised_GPT.py --controller rl --kind marl --algo ppo --model models/model.zip --name marl_ppo --seeds 1
This is a full simulation episode per seed, not a short 1-cycle smoke test.
"""
import argparse
import math
import os

import tod_plan as tp
import tod_eval_common as ec
from sumo_env_marl_ps import SumoMARLEnvPS
from supervisor.config import load_config, validate_against_env
from supervisor.conflicts import load_fixture
from supervisor.supervised_controller_GPT import SupervisedController


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--controller', choices=['static','fixed','rl'], required=True)
    p.add_argument('--kind', choices=['marl','sarl'])
    p.add_argument('--algo', choices=['ppo','dqn'])
    p.add_argument('--model')
    p.add_argument('--name')
    p.add_argument('--seeds', type=int, default=1)
    p.add_argument('--peak-scale', type=float, default=ec.DEFAULT_SCALE)
    p.add_argument('--routes-src', default='2int_files/routes.xml')
    p.add_argument('--gen-dir')
    p.add_argument('--config')
    p.add_argument('--out', default='supervisor_logs/decisions_GPT.jsonl')
    p.add_argument('--enable-slow-monitor', action='store_true',
                   help='enable the provisional per-junction Slow Monitor for RL')
    p.add_argument('--enable-auto-handback', action='store_true',
                   help='enable the provisional gated handback for RL')
    p.add_argument('--run-dir', help='new, isolated output directory required with either opt-in feature')
    p.add_argument('--max-cycles', type=int, help='optional short opt-in smoke run per seed')
    p.add_argument('--hil-port', help='opt-in UNO display serial port, e.g. COM4; RL only')
    p.add_argument('--hil-speed', type=float, choices=[1.0, 3.0],
                   help='HIL playback speed (default 1); requires --hil-port')
    p.add_argument('--gui', action='store_true', help='show the live SUMO GUI')
    args = p.parse_args()
    if not 1 <= args.seeds <= len(ec.STANDARD_SEEDS): p.error('seeds outside available range')
    if args.controller == 'rl' and not all((args.kind,args.algo,args.model,args.name)):
        p.error('RL requires --kind, --algo, --model and --name')
    enhanced = args.enable_slow_monitor or args.enable_auto_handback or args.hil_port is not None
    if args.gui and not enhanced:
        p.error('--gui requires --hil-port or an opt-in monitoring feature')
    if args.hil_speed is not None and args.hil_port is None:
        p.error('--hil-speed requires --hil-port')
    if args.hil_port is not None and not args.hil_port.strip():
        p.error('--hil-port cannot be empty')
    if enhanced:
        if args.controller != 'rl': p.error('opt-in monitoring, handback and HIL currently support RL only')
        if not args.run_dir: p.error('--run-dir is required with either opt-in feature')
        if args.gen_dir is not None: p.error('--gen-dir is not used by isolated opt-in runs')
        if args.out != 'supervisor_logs/decisions_GPT.jsonl':
            p.error('--out is for the legacy path; opt-in decisions are written inside --run-dir')
        if args.max_cycles is not None and args.max_cycles < 1: p.error('--max-cycles must be positive')
        if not math.isfinite(args.peak_scale) or args.peak_scale <= 0: p.error('--peak-scale must be positive')
    elif args.run_dir or args.max_cycles is not None:
        p.error('--run-dir and --max-cycles require an opt-in feature')
    config, sup = ec.make_config(), load_config(args.config)
    if args.gui:
        config['use_gui'] = True
    matrix = load_fixture(sup.conflicts.fixture_path)
    mix = tp.load_route_mix(args.routes_src)
    peak = args.peak_scale * tp.ORIGINAL_RATE_VPH
    if args.controller == 'static':
        schedule = tp.build_schedule(mix, config['lane_map'], peak, config['cycle_length'],
                                     config['yellow_time'], config['allred_time'], config['s_lb'], config['s_ub'])
        inner = ec.StaticController(schedule)
    elif args.controller == 'fixed': inner = ec.FixedController()
    else:
        from stable_baselines3 import PPO, DQN
        algorithm = PPO if args.algo == 'ppo' else DQN
        model = algorithm.load(args.model, device='cpu') if enhanced else algorithm.load(args.model)
        if not enhanced:
            policy = ec.MarlPolicy if args.kind == 'marl' else ec.SarlPolicy
            inner = policy(model, args.name, deterministic=True)
    if enhanced:
        from supervisor.integrated_runner_GPT import run_integrated
        run_integrated(config=config, cfg=sup, matrix=matrix, model=model, kind=args.kind,
                       model_path=args.model, name=args.name, seeds=ec.STANDARD_SEEDS[:args.seeds],
                       mix=mix, peak=peak, run_dir=args.run_dir,
                       slow_enabled=args.enable_slow_monitor,
                       handback_enabled=args.enable_auto_handback, max_cycles=args.max_cycles,
                       hil_port=args.hil_port, hil_speed=args.hil_speed or 1.0)
        return
    gen_dir = args.gen_dir or ec.default_gen_dir(args.peak_scale)
    os.makedirs(gen_dir, exist_ok=True)
    env = SumoMARLEnvPS(config)
    try:
        validate_against_env(sup, env)
        with SupervisedController(inner, sup, matrix, log_path=args.out, run_id=args.name) as ctrl:
            for seed in ec.STANDARD_SEEDS[:args.seeds]:
                route_file = os.path.join(gen_dir, f'routes_seed{seed}.xml')
                tp.generate_route_file(route_file, mix, seed, peak)
                # run_episode resets the environment; clear the latch just before it.
                ctrl.reset_episode(episode_id=seed)
                cycles, trips = ec.run_episode(env, seed, route_file, ctrl)
                print(f'seed {seed}: {len(cycles)} cycles, {len(trips)} trips, final modes {ctrl.modes}; log {args.out}')
    finally:
        env.close()

if __name__ == '__main__': main()
