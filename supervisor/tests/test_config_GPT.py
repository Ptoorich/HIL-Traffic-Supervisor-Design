import dataclasses
import pathlib
import tempfile
from types import SimpleNamespace

import pytest

from supervisor.config import (ConfigError, DEFAULT_CONFIG_PATH, load_config,
                               validate_against_env)

SRC = DEFAULT_CONFIG_PATH.read_text()


def load_text(text):
    p = pathlib.Path(tempfile.mkdtemp()) / "c.toml"
    p.write_text(text)
    return load_config(p)


def test_default_config_loads(cfg):
    assert cfg.agents == ("tl1", "tl2")
    assert cfg.min_green_reject_s == 8.0
    assert cfg.lost_time_s == 12
    assert len(cfg.active_lanes_of("tl1")) == 5
    assert len(cfg.active_lanes_of("tl2")) == 5          # 7 lanes, 2 excluded (L9)
    assert cfg.contingency == {"tl1": 31, "tl2": 59}


@pytest.mark.parametrize("old,new,msg", [
    ("total_budget_s        = 3.0\n", "", "missing keys"),
    ("[contingency]", "[contingency]\ntl3 = 20", "unknown agents"),
    ("tl2 = 59", "tl2 = 75", "contingency tl2"),
    ('occupancy_unit        = "fraction"', 'occupancy_unit = "ratio"', "occupancy_unit"),
    ("fast_gate_stage_s     = 2.0", "fast_gate_stage_s = 3.0", "LED write"),
    ("speed_warn_factor         = 1.2", "speed_warn_factor = 2.0", "speed_warn_factor"),
])
def test_bad_configs_rejected(old, new, msg):
    assert old in SRC
    with pytest.raises(ConfigError, match=msg):
        load_text(SRC.replace(old, new))


def test_free_threshold_validation():
    with pytest.raises(ConfigError, match="free_max"):
        load_text(SRC.replace("free_max                     = 0.10", "free_max = 1.2"))
    with pytest.raises(ConfigError, match="unknown keys"):
        load_text(SRC.replace("[contingency]", "[classifier.tl1]\ncalibrated = true\n[contingency]"))


def test_config_is_frozen(cfg):
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.timing.total_budget_s = 99


def _env(**kw):
    base = dict(cycle_length=90, yellow_time=3, allred_time=3,
                agent_ids=["tl1", "tl2"], lane_map={})
    base.update(kw)
    return SimpleNamespace(**base)


def test_validate_against_env(cfg):
    lane_map = {a: [l.lane_id for l in cfg.lanes_of(a)] for a in cfg.agents}
    validate_against_env(cfg, _env(lane_map=lane_map))
    with pytest.raises(ConfigError, match="cycle"):
        validate_against_env(cfg, _env(lane_map=lane_map, cycle_length=100))
    lane_map["tl1"] = lane_map["tl1"][:-1]
    with pytest.raises(ConfigError, match="lanes differ"):
        validate_against_env(cfg, _env(lane_map=lane_map))
