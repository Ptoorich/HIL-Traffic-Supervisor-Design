"""The final-HIL auditor must not manufacture physical or requirements compliance."""
from __future__ import annotations

import json

from audit_final_hil_GPT import audit_run


def _write(path, rows):
    path.write_text("".join(json.dumps(row) + "\n" for row in rows),
                    encoding="utf-8")


def _phase(t, durations):
    total = 0
    for i, d in enumerate(durations):
        total += d
        if t < total:
            return i
    raise AssertionError("invalid")


def _complete_fixture(tmp_path):
    (tmp_path / "metadata_GPT.json").write_text(json.dumps({
        "test": "one_tl1_missing_lane", "seed": 42, "cycles": 10,
        "slow_monitor_enabled": False, "auto_handback_enabled": True,
        "sumo_gui": True,
    }), encoding="utf-8")
    (tmp_path / "summary_GPT.json").write_text(json.dumps({
        "complete_cycles": 10,
        "fast_contingencies": {"tl1": 1, "tl2": 0},
        "slow_contingencies": {"tl1": 0, "tl2": 0},
        "handbacks_completed": {"tl1": 1, "tl2": 0},
        "final_modes": {"tl1": "ASTA", "tl2": "ASTA"},
    }), encoding="utf-8")
    (tmp_path / "fault_test_result_GPT.json").write_text(json.dumps({
        "status": "PASS", "checks": {"normal_stop_ack": True,
                                    "one_verified_handback": True}
    }), encoding="utf-8")
    (tmp_path / "timing_summary_GPT.json").write_text(json.dumps({
        "measurement_status": "COMPLETE", "boundaries_completed": 10,
        "budget_status": "WITHIN_OBSERVED_BUDGET",
        "budget_exceedances": {"policy_to_boundary_ack_ms": 0},
        "metrics_ms": {"policy_to_boundary_ack_ms": {"n": 10, "max": 42}},
    }), encoding="utf-8")
    obs = []
    for c in range(10):
        obs.append({
            "cycle": c, "start_time": c*90, "end_time": (c+1)*90,
            "junctions": {"tl1": {"split_after": 31},
                          "tl2": {"split_after": 59}},
            "active_programs_after_first_step": {
                "tl1": {"green_a": 31, "states_match": True},
                "tl2": {"green_a": 59, "states_match": True},
            },
        })
    _write(tmp_path / "observations_GPT.jsonl", obs)
    _write(tmp_path / "decisions_GPT.jsonl", [])
    _write(tmp_path / "events_GPT.jsonl", [])
    _write(tmp_path / "timing_GPT.jsonl", [])
    out = []
    seq = 0
    for c in range(10):
        for n in range(91):
            t = c*90+n
            phases = [_phase(n % 90, [31,3,3,47,3,3]),
                      _phase(n % 90, [59,3,3,19,3,3])]
            out.append({
                "event": "sample", "sim_time": t, "phases": phases,
                "programs": {
                    "tl1": {"durations": [31,3,3,47,3,3], "states_match": True},
                    "tl2": {"durations": [59,3,3,19,3,3], "states_match": True},
                },
            })
            seq += 1
            out.append({"event": "ack", "kind": "S", "sequence": seq})
    out.append({"event": "ack", "kind": "X"})
    _write(tmp_path / "display_GPT.jsonl", out)


def test_final_hil_ideal_software_logs_pass_but_physical_remains_unverified(tmp_path):
    _complete_fixture(tmp_path)
    result = audit_run(tmp_path)
    assert result["automated_subset_passed"]
    assert result["checks"]["phase_sequence"]["status"] == "PASS"
    assert result["checks"]["boundary_timing"]["status"] == "PASS"
    assert result["checks"]["physical_leds_and_lcd"]["status"] == "UNVERIFIED"
    assert result["checks"]["SP-28_99_9_percent"]["status"] == "UNVERIFIED"
    assert not result["overall_final_hil_verified"]


def test_final_hil_missing_hil_log_is_not_pass(tmp_path):
    _complete_fixture(tmp_path)
    (tmp_path / "display_GPT.jsonl").unlink()
    result = audit_run(tmp_path)
    assert not result["automated_subset_passed"]
    assert result["checks"]["phase_sequence"]["status"] == "UNVERIFIED"


def test_final_hil_illegal_phase_sequence_is_detected(tmp_path):
    _complete_fixture(tmp_path)
    path = tmp_path / "display_GPT.jsonl"
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    sample = next(row for row in rows if row.get("event") == "sample" and row.get("sim_time") == 32)
    sample["phases"][0] = 0  # Should be TL1 yellow (phase 1).
    _write(path, rows)
    result = audit_run(tmp_path)
    assert result["checks"]["phase_sequence"]["status"] == "FAIL"
    assert not result["automated_subset_passed"]
