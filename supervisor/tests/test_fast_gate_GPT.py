import time

import pytest

from supervisor.checks import NamedCheck
from supervisor.datatypes import CheckResult, Overall, Status
from supervisor.fast_gate import FastGate

from .helpers import healthy_snapshot, jammed, proposal, with_lane


def inputs(cfg, **kw):
    snaps = {a: healthy_snapshot(cfg, a) for a in cfg.agents}
    props = {a: proposal(a) for a in cfg.agents}
    return snaps, props


@pytest.fixture
def gate(cfg, matrix):
    g = FastGate(cfg, matrix)
    yield g
    g.close()


def test_healthy_state_is_approved(cfg, gate):
    res = gate.evaluate(*inputs(cfg), snapshot_ms=1.5)
    assert set(res) == {"tl1", "tl2"}
    for r in res.values():
        assert r.overall == Overall.APPROVE and r.triggering == () and r.warnings == ()
        assert len(r.checks) == 4 and r.snapshot_ms == 1.5
        assert r.classifier_label == "FREE"
        assert all(c.elapsed_ms >= 0 for c in r.checks)
        assert r.decision_ms < 500                      # microsecond-scale checks


def test_gridlock_vetoes_only_that_junction(cfg, gate):
    snaps, props = inputs(cfg)
    s = snaps["tl1"]
    for l in cfg.active_lanes_of("tl1")[:3]:
        s = jammed(s, l.lane_id)
    snaps["tl1"] = s
    res = gate.evaluate(snaps, props)
    assert res["tl1"].overall == Overall.VETO and res["tl1"].triggering == ("traffic_classifier",)
    assert res["tl1"].classifier_label == "GRIDLOCK"
    assert res["tl2"].overall == Overall.APPROVE


def test_warn_without_veto(cfg, gate):
    snaps, props = inputs(cfg)
    props["tl1"] = proposal("tl1", current=15, delta=-5)       # request 10 s: warn band
    r = gate.evaluate(snaps, props)["tl1"]
    assert r.overall == Overall.APPROVE_WARN and r.warnings == ("safe_gate",)


def test_unsafe_proposal_vetoes(cfg, gate):
    snaps, props = inputs(cfg)
    props["tl2"] = proposal("tl2", current=15, delta=-10)      # 5 s green
    assert gate.evaluate(snaps, props)["tl2"].triggering == ("safe_gate",)


def test_invalid_sensor_data_vetoes(cfg, gate):
    snaps, props = inputs(cfg)
    snaps["tl1"] = with_lane(snaps["tl1"], "596638461#0_0", occupancy=1.7)
    r = gate.evaluate(snaps, props)["tl1"]
    assert r.overall == Overall.VETO and "input_validator" in r.triggering


# ── fail-safe behaviour ────────────────────────────────────────────────
def custom_gate(cfg, matrix, fn, name="safe_gate"):
    from supervisor.checks import build_checks
    others = [c for c in build_checks(cfg, matrix) if c.name != name]
    return FastGate(cfg, matrix, checks=tuple(others) + (NamedCheck(name, fn),))


def test_check_timeout_is_a_fail_and_others_still_report(cfg, matrix):
    def slow(snap, prop):
        time.sleep(0.6)
        return CheckResult("safe_gate", Status.PASS, "OK")
    with custom_gate(cfg, matrix, slow) as g:
        t = time.perf_counter()
        res = g.evaluate(*inputs(cfg), deadline_s=0.1)
        waited = time.perf_counter() - t
        assert waited < 0.4                                     # did not wait for the slow check
        r = res["tl1"]
        sg = next(c for c in r.checks if c.check == "safe_gate")
        assert (sg.status, sg.reason) == (Status.FAIL, "TIMEOUT")
        assert r.overall == Overall.VETO and r.triggering == ("safe_gate",)
        assert next(c for c in r.checks if c.check == "input_validator").status == Status.PASS
        assert g.stuck_count == 2                               # one hung thread per junction
        time.sleep(0.7)
        assert g.stuck_count == 0


def test_check_exception_is_a_fail(cfg, matrix):
    def boom(snap, prop):
        raise RuntimeError("kaput")
    with custom_gate(cfg, matrix, boom) as g:
        r = g.evaluate(*inputs(cfg))["tl1"]
    sg = next(c for c in r.checks if c.check == "safe_gate")
    assert (sg.status, sg.reason) == (Status.FAIL, "ERROR") and "kaput" in sg.details["error"]
    assert r.overall == Overall.VETO


@pytest.mark.parametrize("bad", [lambda s, p: None, lambda s, p: "PASS",
                                 lambda s, p: CheckResult("other_name", Status.PASS, "OK")])
def test_malformed_check_result_is_a_fail(cfg, matrix, bad):
    with custom_gate(cfg, matrix, bad) as g:
        r = g.evaluate(*inputs(cfg))["tl1"]
    sg = next(c for c in r.checks if c.check == "safe_gate")
    assert (sg.status, sg.reason) == (Status.FAIL, "MALFORMED")


def test_missing_input_vetoes_that_junction_only(cfg, gate):
    snaps, props = inputs(cfg)
    del snaps["tl2"]
    res = gate.evaluate(snaps, props)
    assert res["tl2"].overall == Overall.VETO
    assert all(c.reason == "MISSING_INPUT" for c in res["tl2"].checks)
    assert res["tl1"].overall == Overall.APPROVE


def test_exhausted_deadline_fails_every_unfinished_check(cfg, matrix):
    def slow(name):
        def f(snap, prop):
            time.sleep(0.3)
            return CheckResult(name, Status.PASS, "OK")
        return f
    names = ("input_validator", "traffic_classifier", "safe_gate", "conflict_monitor")
    with FastGate(cfg, matrix, checks=tuple(NamedCheck(n, slow(n)) for n in names)) as g:
        res = g.evaluate(*inputs(cfg), deadline_s=0.0)
    assert all(r.overall == Overall.VETO for r in res.values())
    assert all(c.reason == "TIMEOUT" for r in res.values() for c in r.checks)


def test_evaluate_never_raises_after_close(cfg, matrix):
    g = FastGate(cfg, matrix)
    g.close()
    res = g.evaluate(*inputs(cfg))
    assert all(r.overall == Overall.VETO for r in res.values())
    assert all(c.reason == "ERROR" for r in res.values() for c in r.checks)


def test_result_is_loggable(cfg, gate):
    import json
    r = gate.evaluate(*inputs(cfg))["tl1"]
    d = json.loads(json.dumps(r.to_log_dict()))
    assert d["overall"] == "APPROVE" and len(d["checks"]) == 4
