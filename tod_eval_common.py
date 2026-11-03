"""
Shared evaluation code for the 2-intersection time-varying-demand benchmark.

Used by baseline_tod_2int.py AND eval_rl_2int.py so every controller
(fixed, static, SARL PPO/DQN, MARL PPO/DQN) is run and scored by the same
code on the same generated demand.

Controllers share one interface:  act(env, obs_dict, t) -> {agent: action}
    action index: 0 = -delta_s, 1 = no change, 2 = +delta_s
"""
import hashlib
import json
import os

import numpy as np
import traci

import tod_plan as tp
from sumo_env_marl_ps import SumoMARLEnvPS

STANDARD_SEEDS = [42, 123, 256, 512, 999, 1337, 2024, 3141, 7777, 9999]
DEFAULT_SCALE = 0.37


def make_config():
    return {
        "sumocfg"  : "2int_files/2int.sumocfg",
        "agent_ids": ["tl1", "tl2"],
        "tl_map"   : {
            "tl1": "30406197",
            "tl2": "cluster_30406198_7161921427_7161921428",
        },
        "lane_map" : {
            "tl1": [
                "596638461#0_0",
                "4761588#1_0", "4761588#1_1", "4761588#1_2", "4761588#1_3"
            ],
            "tl2": [
                "-621271001_0", "669676574#0_0", "1190301745#0_0",
                "-669676574#8_0", "622835372_0", "622835372_1",
                "622835372_2"
            ],
        },
        "cycle_length" : 90,
        "yellow_time"  : 3,
        "allred_time"  : 3,
        "allred_phases": True,          # real all-red phases (env patch)
        "allred_program_id": "tod6",    # 4->6 phases needs a new program id
        "track_step_metrics": True,     # count arrivals/teleports every second
        "init_split"   : 39,
        "s_lb"         : 15,
        "s_ub"         : 63,
        "delta_s"      : 5,
        "q_lc"         : 5,
        "q_hc"         : 15,
        "w_l"          : 1.0,
        "w_cp"         : 3.0,
        "sim_steps"    : tp.TOTAL_TIME,
        "step_length"  : 90,
        "use_gui"      : False,
    }


def default_gen_dir(scale):
    # scale is part of the folder name so runs at different scales can
    # never overwrite each other's route files
    return f"2int_files/generated_tod_s{scale:g}"


def file_md5(path):
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


# ── Controllers ────────────────────────────────────────────────────────
class FixedController:
    """Splits stay at init_split (39/39) all run."""
    name = "fixed"

    def act(self, env, obs, t):
        return {a: 1 for a in env.agent_ids}


class StaticController:
    """Webster-derived split per period (same in every period when the route
    mix does not change by period, i.e. NOT time-of-day varying)."""
    name = "static"

    def __init__(self, schedule):
        self.schedule = schedule

    def act(self, env, obs, t):
        env.set_splits(tp.plan_for_time(self.schedule, t)["splits"])
        return {a: 1 for a in env.agent_ids}


class MarlPolicy:
    """One shared SB3 model applied to every agent (parameter sharing).
    Same call as the old eval scripts: predict(obs[agent], deterministic)."""

    def __init__(self, model, name, deterministic=True):
        self.model, self.name, self.det = model, name, deterministic

    def act(self, env, obs, t):
        return {a: int(self.model.predict(obs[a],
                                          deterministic=self.det)[0])
                for a in env.agent_ids}


class SarlPolicy:
    """
    SARL model trained on SumoTSCEnv: obs = [12 lane queues (tl1 lanes then
    tl2 lanes), tl1 split, tl2 split]; action = 3*junction + {0,1,2}
    (-delta, 0, +delta). Here the SAME patched SumoMARLEnvPS dynamics are
    driven through an adapter: the selected junction gets its delta and the
    other junction a no-op (equivalent to SumoTSCEnv, which only touches the
    selected junction). Observation values use the same normalisation
    (queue/50, (split-s_lb)/(s_ub-s_lb)).
    """

    def __init__(self, model, name, deterministic=True):
        self.model, self.name, self.det = model, name, deterministic

    def build_obs(self, env, obs):
        qs, splits = [], []
        for a in env.agent_ids:
            n = len(env.lane_map[a])
            qs.extend(obs[a][:n])
            splits.append(obs[a][n])
        return np.array(qs + splits, dtype=np.float32)

    def act(self, env, obs, t):
        action = int(self.model.predict(self.build_obs(env, obs),
                                        deterministic=self.det)[0])
        tl_idx, adj = action // 3, action % 3
        acts = {a: 1 for a in env.agent_ids}
        acts[env.agent_ids[tl_idx]] = adj
        return acts


# ── One episode ────────────────────────────────────────────────────────
def run_episode(env, seed, route_file, controller):
    obs = env.reset(seed=seed, route_file=route_file)
    first = env.agent_ids[0]
    cycles, terminated = [], False
    while not terminated:
        t0 = env.current_step
        actions = controller.act(env, obs, t0)
        try:
            obs, rewards, terminateds = env.step(actions)
        except Exception as e:
            print(f"\n!! SUMO died in cycle {t0 // 90} (sim t={t0}s, "
                  f"controller={controller.name}) with splits "
                  f"{env.phase_splits} -> {type(e).__name__}")
            raise
        terminated = terminateds[first]
        q = None
        if not terminated:      # traci is closed once the episode ends
            try:
                q = float(np.mean([traci.lane.getLastStepHaltingNumber(l)
                                   for l in env.all_lanes]))
            except Exception:
                pass
        cycles.append({"t": t0,
                       "reward": rewards[first],
                       "queue": q,
                       "teleports": env.step_teleports,
                       # split in force during the cycle just simulated
                       "splits": {a: float(env.phase_splits[a])
                                  for a in env.agent_ids}})
    return cycles, list(env.tripinfo_records)


# ── Metrics ────────────────────────────────────────────────────────────
def sched(x):
    """Scheduled departure (when the vehicle wanted to enter). Tripinfo's
    'depart' is the ACTUAL insertion time, which is much later when the
    network is backed up."""
    return x["depart"] - x["departDelay"]


def mean_of(tr, key):
    return float(np.mean([x[key] for x in tr])) if tr else 0.0


def censored_total(trips, deps, s, e):
    """
    Mean total delay (timeLoss + wait-to-enter) over ALL vehicles scheduled in
    [s, e), including ones that never finished. A vehicle that had not
    finished by the end of the run contributes (end - scheduled departure),
    which is a LOWER bound on its delay, so this is still optimistic for a
    congested plan, but no longer drops the worst vehicles.
    Vehicle ids in the generated file are indices into deps.
    """
    done = {x["id"]: x["timeLoss"] + x["departDelay"] for x in trips}
    vals, n_unf = [], 0
    for i in np.where((deps >= s) & (deps < e))[0]:
        v = done.get(str(i))
        if v is None:
            v = tp.TOTAL_TIME - deps[i]
            n_unf += 1
        vals.append(v)
    return (float(np.mean(vals)) if vals else 0.0), n_unf


def _window(cycles, trips, deps, s, e, agent_ids):
    cyc = [c for c in cycles if s <= c["t"] < e]
    tr = [x for x in trips if s <= sched(x) < e]
    generated = int(np.sum((deps >= s) & (deps < e)))
    cens, _ = censored_total(trips, deps, s, e)
    qs = [c["queue"] for c in cyc if c["queue"] is not None]
    sp = {a: [c["splits"][a] for c in cyc] for a in agent_ids}
    return {
        "generated": generated,
        "completed": len(tr),
        "unfinished": generated - len(tr),
        "mean_travel_time": mean_of(tr, "duration"),
        "mean_delay": mean_of(tr, "timeLoss"),
        "mean_depart_delay": mean_of(tr, "departDelay"),
        "mean_total_delay": mean_of(tr, "timeLoss")
                            + mean_of(tr, "departDelay"),
        "censored_total_delay": cens,
        "avg_queue": float(np.mean(qs)) if qs else 0.0,
        "total_reward": float(sum(c["reward"] for c in cyc)),
        "teleports": int(sum(c["teleports"] for c in cyc)),
        # what the controller actually did (phase-A green, seconds)
        "mean_split": {a: float(np.mean(v)) if v else 0.0
                       for a, v in sp.items()},
    }


def summarise(cycles, trips, deps, agent_ids):
    """Per-period metrics (vehicles binned by SCHEDULED departure)."""
    out = {p["name"]: _window(cycles, trips, deps, p["start"], p["end"],
                              agent_ids)
           for p in tp.period_table()}
    s0 = tp.WARMUP
    e0 = tp.WARMUP + len(tp.PERIODS) * tp.PERIOD_LEN
    out["overall"] = _window(cycles, trips, deps, s0, e0, agent_ids)
    return out


# ── Reporting ──────────────────────────────────────────────────────────
HEADER = ("delay = SUMO timeLoss; wait = delay before entering the network; "
          "CENSORED-total = timeLoss + wait over ALL vehicles, unfinished "
          "counted as end-of-run minus scheduled departure (a lower bound)")


def seed_line(seed, name, o):
    return (f"seed {seed:4d} {name:9s}: completed {o['completed']:5d}/"
            f"{o['generated']:5d}  tt={o['mean_travel_time']:6.1f}s  "
            f"delay={o['mean_delay']:6.1f}s  "
            f"wait-to-enter={o['mean_depart_delay']:7.1f}s  "
            f"censored-total={o['censored_total_delay']:7.1f}s  "
            f"queue={o['avg_queue']:5.2f}  teleports={o['teleports']}  "
            f"mean split={ {k: round(v, 1) for k, v in o['mean_split'].items()} }")


def print_summary(results, names):
    """results[name] = list of per-seed summarise() dicts."""
    print(f"\n=== Mean over seeds (± std). {HEADER} ===")
    periods = [p["name"] for p in tp.period_table()] + ["overall"]
    for per in periods:
        for name in names:
            def col(k):
                v = [r[per][k] for r in results[name]]
                return np.mean(v), np.std(v)
            d, w, t = (col("mean_delay"), col("mean_depart_delay"),
                       col("censored_total_delay"))
            tt, q, u, g = (col("mean_travel_time"), col("avg_queue"),
                           col("unfinished"), col("generated"))
            tp_ = col("teleports")
            print(f"{per:8s} {name:9s} delay {d[0]:6.1f}±{d[1]:4.1f}  "
                  f"wait {w[0]:7.1f}±{w[1]:5.1f}  "
                  f"CENSORED-total {t[0]:7.1f}±{t[1]:5.1f}  "
                  f"tt {tt[0]:6.1f}±{tt[1]:4.1f}  "
                  f"queue {q[0]:5.2f}±{q[1]:4.2f}  "
                  f"unfinished {u[0]:6.1f}±{u[1]:5.1f} of {g[0]:.0f}  "
                  f"teleports {tp_[0]:.1f}")


def save_json(path, payload):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\nResults saved to {path}")