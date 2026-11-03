"""
Supervisor configuration: loaded from TOML into frozen dataclasses.

Rules (decision D8):
  * every key is required; a missing OR unknown key is a startup error
  * values are range-checked at load, so a bad config never reaches the gate
  * `validate_against_env` asserts the config agrees with the running env
"""
from __future__ import annotations

import tomllib
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

DEFAULT_CONFIG_PATH = Path(__file__).with_name("supervisor_config_GPT.toml")


class ConfigError(ValueError):
    """Raised at startup for any problem with the configuration."""


# ── Section dataclasses ────────────────────────────────────────────────
@dataclass(frozen=True)
class TimingCfg:
    total_budget_s: float
    fast_gate_stage_s: float
    resolver_stage_s: float
    cycle_s: int
    yellow_s: int
    allred_s: int
    hard_min_green_s: float
    latency_margin_s: float
    pref_min_green_s: float


@dataclass(frozen=True)
class SnapshotCfg:
    occupancy_unit: str


@dataclass(frozen=True)
class ValidatorCfg:
    speed_warn_factor: float
    speed_fail_factor: float
    count_slack_vehicles: int
    all_halting_max_speed_mps: float
    high_speed_mps: float
    high_speed_max_occupancy: float


@dataclass(frozen=True)
class ClassifierCfg:
    vehicle_length_m: float
    gridlock_speed_mps: float
    gridlock_occupancy_frac_of_jam: float
    gridlock_fill_frac: float
    gridlock_lane_fraction: float
    free_max: float


@dataclass(frozen=True)
class ConflictsCfg:
    fixture_path: str


@dataclass(frozen=True)
class LoggingCfg:
    jsonl_path: str


@dataclass(frozen=True)
class LaneCfg:
    lane_id: str
    agent: str
    capacity: int
    excluded: bool


@dataclass(frozen=True)
class SupervisorConfig:
    timing: TimingCfg
    snapshot: SnapshotCfg
    validator: ValidatorCfg
    classifier: ClassifierCfg
    contingency: dict          # agent -> phase A green (s)
    conflicts: ConflictsCfg
    logging: LoggingCfg
    lanes: tuple               # tuple[LaneCfg, ...] in file order

    # ── derived helpers ────────────────────────────────────────────────
    @property
    def intergreen_s(self) -> int:
        return self.timing.yellow_s + self.timing.allred_s

    @property
    def lost_time_s(self) -> int:
        return 2 * self.intergreen_s

    @property
    def min_green_reject_s(self) -> float:
        return self.timing.hard_min_green_s + self.timing.latency_margin_s

    @property
    def agents(self) -> tuple:
        seen = []
        for lane in self.lanes:
            if lane.agent not in seen:
                seen.append(lane.agent)
        return tuple(seen)

    def lanes_of(self, agent: str) -> tuple:
        return tuple(l for l in self.lanes if l.agent == agent)

    def active_lanes_of(self, agent: str) -> tuple:
        """Lanes used by the Gridlock test and the classifier (not excluded)."""
        return tuple(l for l in self.lanes_of(agent) if not l.excluded)

    def lane(self, lane_id: str) -> LaneCfg:
        for l in self.lanes:
            if l.lane_id == lane_id:
                return l
        raise KeyError(lane_id)


# ── Loading and validation ─────────────────────────────────────────────
def _build(cls, table: Any, where: str):
    """Instantiate a section dataclass; exact key set, correct types."""
    if not isinstance(table, dict):
        raise ConfigError(f"[{where}] must be a table")
    want = {f.name: f.type for f in fields(cls)}
    missing = sorted(set(want) - set(table))
    extra = sorted(set(table) - set(want))
    if missing:
        raise ConfigError(f"[{where}] missing keys: {missing}")
    if extra:
        raise ConfigError(f"[{where}] unknown keys: {extra}")
    kwargs = {}
    for name, typ in want.items():
        v = table[name]
        typ = typ if isinstance(typ, str) else typ.__name__
        if typ == "float":
            ok = isinstance(v, (int, float)) and not isinstance(v, bool)
            v = float(v) if ok else v
        elif typ == "int":
            ok = isinstance(v, int) and not isinstance(v, bool)
        elif typ == "bool":
            ok = isinstance(v, bool)
        elif typ == "str":
            ok = isinstance(v, str)
        else:
            ok = True
        if not ok:
            raise ConfigError(f"[{where}] {name} must be {typ}, got {v!r}")
        kwargs[name] = v
    return cls(**kwargs)


def _need(cond: bool, msg: str) -> None:
    if not cond:
        raise ConfigError(msg)


def _validate(cfg: SupervisorConfig) -> None:
    t = cfg.timing
    _need(t.total_budget_s > 0, "timing.total_budget_s must be > 0")
    _need(t.fast_gate_stage_s > 0 and t.resolver_stage_s > 0,
          "timing stage deadlines must be > 0")
    _need(t.fast_gate_stage_s + t.resolver_stage_s < t.total_budget_s,
          "timing: fast_gate_stage_s + resolver_stage_s must leave time for "
          "the LED write inside total_budget_s")
    _need(t.cycle_s > 0 and t.yellow_s > 0 and t.allred_s >= 0,
          "timing: cycle/yellow/allred out of range")
    _need(0 < t.hard_min_green_s <= cfg.min_green_reject_s <= t.pref_min_green_s,
          "timing: need hard_min <= hard_min+margin <= preferred minimum")
    _need(t.latency_margin_s >= 0, "timing.latency_margin_s must be >= 0")

    _need(cfg.snapshot.occupancy_unit in ("fraction", "percent"),
          'snapshot.occupancy_unit must be "fraction" or "percent"')

    v = cfg.validator
    _need(1 <= v.speed_warn_factor < v.speed_fail_factor,
          "validator: need 1 <= speed_warn_factor < speed_fail_factor")
    _need(v.count_slack_vehicles >= 0, "validator.count_slack_vehicles must be >= 0")
    _need(v.all_halting_max_speed_mps > 0, "validator.all_halting_max_speed_mps must be > 0")
    _need(v.high_speed_mps > 0, "validator.high_speed_mps must be > 0")
    _need(0 < v.high_speed_max_occupancy <= 1,
          "validator.high_speed_max_occupancy must be in (0, 1]")

    c = cfg.classifier
    _need(c.vehicle_length_m > 0, "classifier.vehicle_length_m must be > 0")
    _need(c.gridlock_speed_mps > 0, "classifier.gridlock_speed_mps must be > 0")
    for name in ("gridlock_occupancy_frac_of_jam", "gridlock_fill_frac",
                 "gridlock_lane_fraction"):
        _need(0 < getattr(c, name) <= 1, f"classifier.{name} must be in (0, 1]")

    _need(0 <= c.free_max <= 1, "classifier.free_max must be in [0, 1]")

    _need(cfg.lanes, "no [lanes.*] tables defined")
    ids = [l.lane_id for l in cfg.lanes]
    _need(len(ids) == len(set(ids)), "duplicate lane ids in [lanes]")
    for l in cfg.lanes:
        _need(l.capacity > 0, f"lane {l.lane_id}: capacity must be > 0")

    for agent in cfg.agents:
        _need(agent in cfg.contingency, f"[contingency] missing {agent}")
        _need(cfg.active_lanes_of(agent),
              f"{agent}: every lane is excluded, Gridlock test has no lanes")
        split = cfg.contingency[agent]
        b = t.cycle_s - split - cfg.lost_time_s
        _need(min(split, b) >= cfg.min_green_reject_s,
              f"contingency {agent}: greens A={split}, B={b} violate the "
              f"{cfg.min_green_reject_s}s minimum")
    extra = set(cfg.contingency)
    _need(extra <= set(cfg.agents),
          f"classifier/contingency name unknown agents: {sorted(extra - set(cfg.agents))}")


def load_config(path: str | Path | None = None) -> SupervisorConfig:
    p = Path(path) if path else DEFAULT_CONFIG_PATH
    try:
        with open(p, "rb") as f:
            raw = tomllib.load(f)
    except FileNotFoundError:
        raise ConfigError(f"config file not found: {p}") from None
    except tomllib.TOMLDecodeError as e:
        raise ConfigError(f"config file {p} is not valid TOML: {e}") from None

    sections = {"timing", "snapshot", "validator", "classifier", "contingency",
                "conflicts", "logging", "lanes"}
    missing = sorted(sections - set(raw))
    extra = sorted(set(raw) - sections)
    if missing:
        raise ConfigError(f"missing sections: {missing}")
    if extra:
        raise ConfigError(f"unknown sections: {extra}")

    classifier = _build(ClassifierCfg, raw["classifier"], "classifier")

    contingency = raw["contingency"]
    for a, s in contingency.items():
        if not isinstance(s, int) or isinstance(s, bool):
            raise ConfigError(f"[contingency] {a} must be an integer, got {s!r}")

    lanes = tuple(
        _build(LaneCfg, {"lane_id": lid, **tbl}, f"lanes.{lid}")
        for lid, tbl in raw["lanes"].items()
    )

    cfg = SupervisorConfig(
        timing=_build(TimingCfg, raw["timing"], "timing"),
        snapshot=_build(SnapshotCfg, raw["snapshot"], "snapshot"),
        validator=_build(ValidatorCfg, raw["validator"], "validator"),
        classifier=classifier,
        contingency=dict(contingency),
        conflicts=_build(ConflictsCfg, raw["conflicts"], "conflicts"),
        logging=_build(LoggingCfg, raw["logging"], "logging"),
        lanes=lanes,
    )
    _validate(cfg)
    return cfg


def validate_against_env(cfg: SupervisorConfig, env) -> None:
    """Startup assertion: the config must describe the env it supervises."""
    problems = []
    for name, mine, theirs in (
        ("cycle", cfg.timing.cycle_s, getattr(env, "cycle_length", None)),
        ("yellow", cfg.timing.yellow_s, getattr(env, "yellow_time", None)),
        ("all-red", cfg.timing.allred_s, getattr(env, "allred_time", None)),
    ):
        if mine != theirs:
            problems.append(f"{name}: config {mine} != env {theirs}")
    if tuple(env.agent_ids) != cfg.agents:
        problems.append(f"agents: config {cfg.agents} != env {tuple(env.agent_ids)}")
    for agent in env.agent_ids:
        env_lanes = set(env.lane_map[agent])
        cfg_lanes = {l.lane_id for l in cfg.lanes_of(agent)}
        if env_lanes != cfg_lanes:
            problems.append(
                f"{agent} lanes differ: only in env {sorted(env_lanes - cfg_lanes)}, "
                f"only in config {sorted(cfg_lanes - env_lanes)}")
    if problems:
        raise ConfigError("config does not match the env: " + "; ".join(problems))
