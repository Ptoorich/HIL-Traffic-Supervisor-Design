"""SP-28 readback fault-injection and explicit FAIL-event regression tests.

The detection within one simulation step, recorded FAIL and subsequent safe
interruption are under test. A detected fault does NOT imply verified FTSC
handover; aggregate agreement and physical optics require separate evidence.

No SUMO installation, physical Arduino, or controller model is required.
Run from repository root:
    python -m pytest -q supervisor/tests/test_sp28_readback_GPT.py
"""
from __future__ import annotations

import importlib
import json
import sys
import types
from unittest import mock

import pytest

from supervisor.config import load_config
from supervisor.conflicts import ConflictMatrix, JunctionConflicts
from supervisor.hil_display_GPT import PhaseDisplay
from supervisor.hil_serial_GPT import (
    DisplayError, SerialDisplay, decode_frame,
)
from supervisor.observation_adapter_GPT import read_active_programs
from supervisor.supervised_integrated_GPT import IntegratedSupervisedController
from supervisor.tests.helpers import COMMANDED
from supervisor.tests.test_hil_timing_GPT import BoardDouble, Clock
from supervisor.tests.test_integrated_runner_GPT import (
    FakeEnvironment, FakeTaker, Model, make_traci,
)


def _environment():
    cfg = load_config()
    fake = make_traci(lambda t: 0)
    env = FakeEnvironment(cfg, fake)
    return cfg, fake, env


def _matrix(cfg):
    foes = frozenset({(0, 2), (0, 3), (1, 2), (1, 3)})
    return ConflictMatrix({
        a: JunctionConflicts(f"tl_{a}", 4, foes, COMMANDED)
        for a in cfg.agents
    })


def _inject_duration_error(fake, env, monkeypatch, *, from_step=0):
    original = fake.trafficlight.getAllProgramLogics

    def altered(tl_id):
        programs = original(tl_id)
        if (tl_id == env.tl_map["tl1"]
                and fake.simulation.now >= from_step):
            programs[0].phases[0].duration += 2.0
        return programs

    monkeypatch.setattr(fake.trafficlight, "getAllProgramLogics", altered)


def test_sp28_matching_program_is_read_back():
    cfg, fake, env = _environment()
    result = read_active_programs(env, fake)
    assert set(result) == set(cfg.agents)
    for agent in cfg.agents:
        assert result[agent]["green_a"] == env.phase_splits[agent]
        assert result[agent]["states_match"] is True


def test_sp28_wrong_sumo_green_duration_is_detected(monkeypatch):
    _, fake, env = _environment()
    _inject_duration_error(fake, env, monkeypatch)
    with pytest.raises(ValueError, match="active durations"):
        read_active_programs(env, fake)


def test_sp28_wrong_sumo_phase_state_is_detected(monkeypatch):
    _, fake, env = _environment()
    original = fake.trafficlight.getAllProgramLogics

    def altered(tl_id):
        programs = original(tl_id)
        if tl_id == env.tl_map["tl1"]:
            programs[0].phases[0].state = "rrrr"
        return programs

    monkeypatch.setattr(fake.trafficlight, "getAllProgramLogics", altered)
    with pytest.raises(ValueError, match="active signal phase states"):
        read_active_programs(env, fake)


def test_sp28_live_phase_indication_mismatch_blocks_hil_state(monkeypatch):
    _, fake, env = _environment()
    monkeypatch.setattr(
        fake.trafficlight, "getPhase", lambda tl_id: 0, raising=False
    )
    monkeypatch.setattr(
        fake.trafficlight, "getRedYellowGreenState",
        lambda tl_id: "rrrr" if tl_id == env.tl_map["tl1"] else "GGrr",
        raising=False,
    )

    class TransportDouble:
        def __init__(self):
            self.clock = Clock()
            self.calls = []

        def state(self, *args):
            self.calls.append(args)
            raise AssertionError("A mismatched SUMO state must not be sent")

        def record(self, *args, **kwargs):
            self.calls.append(("record", args, kwargs))

    transport = TransportDouble()
    display = PhaseDisplay(
        transport, speed=1.0, sleep=transport.clock.sleep
    )
    with pytest.raises(DisplayError, match="live state does not match"):
        display.sample(env, fake, {"tl1": "ASTA", "tl2": "ASTA"})
    assert transport.calls == []


def test_sp28_wrong_arduino_gpio_ack_faults_display_session(tmp_path):
    clock = Clock()
    # HELLO uses sequence 0, PAUSE uses 1; first STATE uses sequence 2.
    board = BoardDouble(clock, wrong_sequence=2)
    log_path = tmp_path / "sp28_display_GPT.jsonl"

    with SerialDisplay(
        "VIRTUAL", log_path=log_path, serial_instance=board,
        clock=clock, sleep=clock.sleep,
    ) as transport:
        with pytest.raises(DisplayError, match="ACK output mask/type differs"):
            transport.state(0, [0, 3], [0, 0])
        assert transport.failed is True

    assert decode_frame(board.writes[-1])[1] == "F"
    rows = [json.loads(line) for line in log_path.read_text().splitlines()]
    assert any(row["event"] == "fault" for row in rows)


def test_sp28_missing_arduino_ack_faults_display_session(tmp_path):
    clock = Clock()

    class NoStateAck(BoardDouble):
        def write(self, data):
            if decode_frame(data)[1] == "S":
                self.writes.append(data)
                return len(data)  # Suppress STATE ACK on both attempts.
            return super().write(data)

    board = NoStateAck(clock)
    with SerialDisplay(
        "VIRTUAL", log_path=tmp_path / "sp28_no_ack_GPT.jsonl",
        serial_instance=board, clock=clock, sleep=clock.sleep,
    ) as transport:
        with pytest.raises(DisplayError, match="ACK deadline exceeded"):
            transport.state(0, [0, 3], [0, 0])
        assert transport.failed is True
    assert decode_frame(board.writes[-1])[1] == "F"


def test_sp28_first_step_programme_mismatch_records_fail_then_aborts(
    tmp_path, monkeypatch,
):
    """The failure is an SP-28 execution FAIL, not a pre-command gate VETO."""
    cfg, fake, env = _environment()
    _inject_duration_error(fake, env, monkeypatch, from_step=1)
    stub_plan = types.ModuleType("tod_plan")
    stub_env = types.ModuleType("sumo_env_marl_ps")
    stub_env.SumoMARLEnvPS = FakeEnvironment

    modules = {
        "traci": fake,
        "tod_plan": stub_plan,
        "sumo_env_marl_ps": stub_env,
    }
    with mock.patch.dict(sys.modules, modules):
        sys.modules.pop("supervisor.integrated_runner_GPT", None)
        try:
            runner = importlib.import_module(
                "supervisor.integrated_runner_GPT"
            )
            policy = runner.StrictPolicy("marl", Model(1), fake)

            def factory(inner, cfg, matrix, **kwargs):
                return IntegratedSupervisedController(
                    inner, cfg, matrix,
                    snapshot_taker=FakeTaker(cfg, fake, {}), **kwargs
                )

            with pytest.raises(ValueError, match="active durations"):
                runner.run_episode(
                    env, policy, cfg, _matrix(cfg), 42,
                    tmp_path / "route.xml", tmp_path,
                    slow_enabled=False,
                    handback_enabled=False,
                    max_cycles=1,
                    traci_mod=fake,
                    controller_factory=factory,
                )
        finally:
            sys.modules.pop("supervisor.integrated_runner_GPT", None)

    assert fake.simulation.now == 1
    decision_file = tmp_path / "decisions_GPT.jsonl"
    decisions = [
        json.loads(row) for row in decision_file.read_text().splitlines()
    ]
    assert len(decisions) == 1
    first = decisions[0]["decisions"]["tl1"]
    assert first["gate"]["overall"] in ("APPROVE", "APPROVE_WARN")
    assert first["mode_after"] == "ASTA"
    # The execution fault occurs AFTER the supervisor's cycle-boundary
    # approval. A distinct SP-28 FAIL event must therefore be recorded.
    event_file = tmp_path / "events_GPT.jsonl"
    events = [json.loads(row) for row in event_file.read_text().splitlines()]
    failures = [x for x in events if x.get("event") == "execution_readback_FAIL"]
    assert len(failures) == 1
    failure = failures[0]
    assert failure["requirement"] == "SP-28"
    assert failure["status"] == "FAIL"
    assert failure["sim_time"] == 1
    assert failure["step_in_cycle"] == 1
    assert failure["cycle"] == 0
    assert failure["agent"] == "tl1"
    assert failure["source"] == "sumo_program_readback"
    assert failure["response"] == "ABORT_UNTRUSTED_EXECUTION"
    assert failure["ftsc_commanded"] is False
    # No unsafe FTSC handover is asserted just because a readback failed.
