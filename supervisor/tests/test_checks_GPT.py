import dataclasses
import math

import pytest

from supervisor.checks import conflict_monitor, input_validator, safe_gate, traffic_classifier
from supervisor.datatypes import Status

from .helpers import healthy_snapshot, jammed, proposal, with_lane

L1 = "596638461#0_0"
EXCL = "669676574#0_0"


# ── Input Validator ────────────────────────────────────────────────────
def codes(res):
    return {v["code"] for v in res.details["violations"]}


def test_validator_healthy_passes(cfg):
    for a in cfg.agents:
        assert input_validator.check(healthy_snapshot(cfg, a), proposal(a), cfg).status == Status.PASS


@pytest.mark.parametrize("fields,code", [
    (dict(occupancy=1.2), "OCCUPANCY_RANGE"),
    (dict(occupancy=50.0), "OCCUPANCY_RANGE"),              # a percent value read as a fraction
    (dict(occupancy=-0.1), "OCCUPANCY_RANGE"),
    (dict(halting=5, count=3), "HALTING_GT_COUNT"),
    (dict(halting=-1), "NEGATIVE_COUNT"),
    (dict(count=13, halting=0), "COUNT_GT_CAPACITY"),       # capacity 10, slack 1
    (dict(mean_speed=-1.0), "NEGATIVE_SPEED"),
    (dict(mean_speed=25.0), "SPEED_OVER_LIMIT"),            # > 1.5 x 13.89
    (dict(mean_speed=float("nan")), "NON_FINITE"),
    (dict(occupancy=float("inf")), "NON_FINITE"),
    (dict(count=5, halting=0, occupancy=0.97, mean_speed=8.0), "SPEED_OCCUPANCY_IMPOSSIBLE"),
    (dict(count=4, halting=4, mean_speed=9.0, occupancy=0.3), "HALTING_SPEED_MISMATCH"),
])
def test_validator_fails(cfg, fields, code):
    res = input_validator.check(with_lane(healthy_snapshot(cfg, "tl1"), L1, **fields),
                                proposal(), cfg)
    assert res.status == Status.FAIL
    assert code in codes(res)


def test_validator_soft_limits_warn_not_fail(cfg):
    s = with_lane(healthy_snapshot(cfg, "tl1"), L1, count=11, halting=0)   # capacity + 1
    assert input_validator.check(s, proposal(), cfg).status == Status.WARN
    s = with_lane(healthy_snapshot(cfg, "tl1"), L1, count=2, halting=0,
                  mean_speed=13.89 * 1.3, occupancy=0.1)
    assert input_validator.check(s, proposal(), cfg).status == Status.WARN


def test_validator_empty_lane_speed_is_not_a_cross_check_error(cfg):
    # empty lane: SUMO reports the speed limit; halting == count == 0 must not trip HALTING_SPEED_MISMATCH
    assert input_validator.check(healthy_snapshot(cfg, "tl1"), proposal(), cfg).status == Status.PASS


def test_validator_missing_and_unexpected_lanes(cfg):
    s = healthy_snapshot(cfg, "tl1")
    short = dataclasses.replace(s, lanes=s.lanes[1:])
    assert "LANE_MISSING" in codes(input_validator.check(short, proposal(), cfg))
    extra = dataclasses.replace(s, lanes=s.lanes + (dataclasses.replace(s.lanes[0], lane_id="ghost_0"),))
    assert "LANE_UNEXPECTED" in codes(input_validator.check(extra, proposal(), cfg))


def test_validator_checks_excluded_lanes_against_theoretical_capacity(cfg):
    s = healthy_snapshot(cfg, "tl2")
    ok = with_lane(s, EXCL, count=150, halting=0)               # < 194, fine
    assert input_validator.check(ok, proposal("tl2"), cfg).status == Status.PASS
    bad = with_lane(s, EXCL, count=250, halting=0)
    assert "COUNT_GT_CAPACITY" in codes(input_validator.check(bad, proposal("tl2"), cfg))


# ── Traffic Classifier ─────────────────────────────────────────────────
def test_gridlock_needs_half_of_active_lanes(cfg):
    s = healthy_snapshot(cfg, "tl1")
    lanes = [l.lane_id for l in cfg.active_lanes_of("tl1")]
    for l in lanes[:2]:
        s = jammed(s, l)
    assert traffic_classifier.check(s, proposal(), cfg).status != Status.FAIL   # 2 of 5
    s = jammed(s, lanes[2])
    res = traffic_classifier.check(s, proposal(), cfg)                           # 3 of 5
    assert (res.status, res.reason, res.details["label"]) == (Status.FAIL, "GRIDLOCK", "GRIDLOCK")


def test_excluded_lanes_never_count_toward_gridlock(cfg):
    s = healthy_snapshot(cfg, "tl2")
    s = jammed(jammed(s, EXCL), "-669676574#8_0")
    s = jammed(jammed(s, "-621271001_0"), "1190301745#0_0")      # only 2 of 5 active lanes
    assert traffic_classifier.check(s, proposal("tl2"), cfg).status != Status.FAIL
    s = jammed(s, "622835372_0")                                  # 3 of 5
    assert traffic_classifier.check(s, proposal("tl2"), cfg).reason == "GRIDLOCK"


def test_gridlock_rule_is_reachable_with_real_sumo_occupancy(cfg):
    """SUMO occupancy counts vehicle length only: a jammed lane reads ~0.67, never 0.9."""
    s = healthy_snapshot(cfg, "tl1")
    r = next(x for x in s.lanes if x.lane_id == L1)
    assert r.capacity * 5.0 / r.length < 0.7
    for l in cfg.active_lanes_of("tl1")[:3]:
        s = jammed(s, l.lane_id)
    assert traffic_classifier.check(s, proposal(), cfg).status == Status.FAIL


def test_gridlock_requires_all_three_signals(cfg):
    base = jammed(healthy_snapshot(cfg, "tl1"), L1)
    r = next(x for x in base.lanes if x.lane_id == L1)
    cc = cfg.classifier
    assert traffic_classifier.lane_is_gridlocked(r, cc)
    assert not traffic_classifier.lane_is_gridlocked(dataclasses.replace(r, mean_speed=2.0), cc)
    assert not traffic_classifier.lane_is_gridlocked(dataclasses.replace(r, occupancy=r.occupancy * 0.5), cc)
    assert not traffic_classifier.lane_is_gridlocked(dataclasses.replace(r, halting=r.capacity // 2), cc)


def test_labels_free_light_heavy(cfg):
    s = healthy_snapshot(cfg, "tl1")
    assert traffic_classifier.check(s, proposal(), cfg).reason == "FREE"
    light = with_lane(s, L1, halting=6, count=6, mean_speed=2.0)
    assert traffic_classifier.check(light, proposal(), cfg).reason == "LIGHT"
    s = jammed(s, L1)
    assert traffic_classifier.check(s, proposal(), cfg).reason == "HEAVY"
    s = jammed(s, "4761588#1_0")
    assert traffic_classifier.check(s, proposal(), cfg).reason == "HEAVY"


@pytest.mark.parametrize("current,delta,status,worst", [
    (39, 0, Status.PASS, None),
    (63, 0, Status.PASS, None),                 # A=63, B=15
    (15, -5, Status.WARN, "A"),                 # unclipped request 10 s: A=10 < 11
    (15, -10, Status.FAIL, "A"),                # 5 s
    (63, 5, Status.WARN, "B"),                  # A=68, B=10
    (63, 10, Status.FAIL, "B"),                 # A=73, B=5
    (19, -11, Status.FAIL, "A"),                # A=8 is the boundary: 8 is not < 8, but < 11
])
def test_safe_gate_boundaries(cfg, current, delta, status, worst):
    if (current, delta) == (19, -11):
        status = Status.WARN                    # exactly 8.0 s is accepted (warn band)
    res = safe_gate.check(healthy_snapshot(cfg, "tl1"), proposal(current=current, delta=delta), cfg)
    assert res.status == status
    if worst:
        assert res.details["worst_phase"] == worst


def test_safe_gate_exact_thresholds(cfg):
    snap = healthy_snapshot(cfg, "tl1")
    check = lambda a: safe_gate.check(snap, proposal(current=a), cfg).status
    assert check(7.99) == Status.FAIL
    assert check(8.0) == Status.WARN
    assert check(10.99) == Status.WARN
    assert check(11.0) == Status.PASS
    assert check(90 - 12 - 8.0) == Status.WARN          # phase B at 8.0
    assert check(90 - 12 - 7.99) == Status.FAIL
    assert check(90 - 12 - 11.0) == Status.PASS


def test_safe_gate_nan_is_a_fail(cfg):
    res = safe_gate.check(healthy_snapshot(cfg, "tl1"), proposal(current=float("nan")), cfg)
    assert (res.status, res.reason) == (Status.FAIL, "NON_FINITE")


def test_safe_gate_uses_unclipped_request(cfg):
    p = proposal(current=15, delta=-5)
    assert p.requested_split == 10 and p.current_split == 15
    assert safe_gate.check(healthy_snapshot(cfg, "tl1"), p, cfg).details["green_a"] == 10


# ── Conflict Monitor ───────────────────────────────────────────────────
def cm(cfg, matrix, **kw):
    return conflict_monitor.check(healthy_snapshot(cfg, "tl1", **kw), proposal(), cfg, matrix)


def test_conflict_monitor_normal_program_passes(cfg, matrix):
    assert cm(cfg, matrix).status == Status.PASS


def test_conflict_monitor_gg_fails(cfg, matrix):
    res = cm(cfg, matrix, live_state="GrGr")                     # links 0 and 2 are foes
    assert (res.status, res.reason) == (Status.FAIL, "CONFLICT")


def test_conflict_monitor_G_with_g_is_allowed_gg_warns(cfg, matrix):
    assert cm(cfg, matrix, live_state="Grgr").status == Status.PASS     # G(0) + g(2)
    res = cm(cfg, matrix, live_state="grgr")
    assert (res.status, res.reason) == (Status.WARN, "PERMISSIVE_CONFLICT")


def test_conflict_monitor_checks_commanded_strings(cfg, matrix):
    res = cm(cfg, matrix, commanded_phase_strings=("GGGG", "yyrr", "rrGG", "rryy"))
    assert res.status == Status.FAIL


def test_conflict_monitor_length_mismatch_fails(cfg, matrix):
    assert cm(cfg, matrix, live_state="rrr").details["violations"][0]["code"] == "STATE_LENGTH"


def test_conflict_monitor_missing_matrix_fails(cfg, matrix):
    from supervisor.conflicts import ConflictMatrix
    res = conflict_monitor.check(healthy_snapshot(cfg, "tl1"), proposal(), cfg, ConflictMatrix({}))
    assert res.reason == "NO_MATRIX"
