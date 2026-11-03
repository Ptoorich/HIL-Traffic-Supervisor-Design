"""
Calibration logger: a controller wrapper that records per-lane sensor data at
every cycle boundary, using the SAME SnapshotTaker as the Fast Gate.

    ctrl = LoggingController(inner_controller, cfg, "supervisor_logs/static.jsonl")
    ctrl.seed = 42                       # set before each episode
    ec.run_episode(env, 42, route_file, ctrl)

The reading is taken BEFORE the inner controller acts, at the cycle boundary
(end of the all-red of phase B), where phase A queues peak (handoff 5).
It has the same interface as every other controller (act(env, obs, t) and a
`name`), so it drops into ec.run_episode unchanged and never alters actions.

One JSON object per line, one line per (cycle, junction, lane).
"""
from __future__ import annotations

import json
from pathlib import Path

from .config import SupervisorConfig
from .snapshot import SnapshotTaker


def default_period_of(t: float) -> str:
    """Period label from tod_plan (agents folder). 'n/a' if it is not importable."""
    try:
        import tod_plan as tp
    except ImportError:
        return "n/a"
    if t < tp.WARMUP:
        return "warmup"
    if t >= tp.WARMUP + len(tp.PERIODS) * tp.PERIOD_LEN:
        return "cooldown"
    return tp.period_name_at(t)


class LoggingController:
    def __init__(self, inner, cfg: SupervisorConfig, out_path: str | Path,
                 period_of=default_period_of, traci_mod=None,
                 run_id: str | None = None):
        self.inner = inner
        self.name = getattr(inner, "name", type(inner).__name__)
        self.seed = None
        self.run_id = run_id or self.name
        self._period_of = period_of
        self._taker = SnapshotTaker(cfg, traci_mod)
        out = Path(out_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        self._f = open(out, "a", encoding="utf-8")

    def act(self, env, obs, t):
        snaps = self._taker.take(env)                 # before the inner controller acts
        period = self._period_of(t)
        for agent, snap in snaps.items():
            for r in snap.lanes:
                self._f.write(json.dumps({
                    "run": self.run_id, "controller": self.name,
                    "seed": self.seed, "t": t, "period": period,
                    "agent": agent, "lane": r.lane_id,
                    "halting": r.halting, "count": r.count,
                    "occupancy": r.occupancy, "mean_speed": r.mean_speed,
                    "length": r.length, "max_speed": r.max_speed,
                    "capacity": r.capacity, "excluded": r.excluded,
                    "split": snap.current_split,
                }) + "\n")
        self._f.flush()
        return self.inner.act(env, obs, t)

    def close(self) -> None:
        self._f.close()
