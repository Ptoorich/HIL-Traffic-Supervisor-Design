"""Deterministic SP-28 audit tests. No Arduino/SUMO required."""
from __future__ import annotations

import json

import pytest

from audit_sp28_GPT import audit_run
from supervisor.hil_serial_GPT import PHASE_MASKS


def _write(path, records):
    path.write_text("".join(json.dumps(row) + "\n" for row in records),
                    encoding="utf-8")


def _record(folder, *, corrupt_mask=False, omit_sample=False,
            duplicated_boundary=False):
    # Two junctions, each one four-step cycle. Sample 0..4.
    observations = [{
        "start_time": 0, "end_time": 4,
        "junctions": {
            "tl1": {"split_after": 31},
            "tl2": {"split_after": 59},
        },
    }]
    _write(folder / "observations_GPT.jsonl", observations)
    rows, acks = [], []
    for sim_time in range(5):
        if omit_sample and sim_time == 2:
            continue
        repetitions = 2 if duplicated_boundary and sim_time == 4 else 1
        for j in range(repetitions):
            seq = 10 + sim_time * 2 + j
            phases = [0, 3]
            mask = PHASE_MASKS[phases[0]] | (PHASE_MASKS[phases[1]] << 6)
            if corrupt_mask and sim_time == 2:
                mask ^= 1
            rows.append({
                "event": "sample", "sim_time": sim_time, "session": 7,
                "ack_sequence": seq, "ack_received_at": float(sim_time) + 0.1,
                "phases": phases,
                "programs": {
                    "tl1": {"green_a": 31, "states_match": True},
                    "tl2": {"green_a": 59, "states_match": True},
                },
            })
            acks.append({
                "event": "ack", "kind": "S", "session": 7,
                "sequence": seq, "mask": mask,
                "ack_received_at": float(sim_time) + 0.1,
            })
    _write(folder / "display_GPT.jsonl", [*rows, *acks])
    _write(folder / "events_GPT.jsonl", [])


def test_sp28_audit_unmodified_agreement_is_100(tmp_path):
    _record(tmp_path)
    result = audit_run(tmp_path)
    assert result["total"] == {"assessed": 8, "agreed": 8, "failed": 0,
                                "agreement_pct": 100.0}
    assert result["threshold_met_for_assessed_steps"]
    assert result["coverage_complete"]


def test_sp28_audit_wrong_gpio_mask_counts_as_failure(tmp_path):
    _record(tmp_path, corrupt_mask=True)
    result = audit_run(tmp_path)
    assert result["total"]["assessed"] == 8
    # A 12-bit display mask is shared; no affected agent can be reliably
    # assigned to a single junction, so both are conservatively flagged.
    assert result["total"]["failed"] == 2
    assert not result["threshold_met_for_assessed_steps"]


def test_sp28_audit_missing_sample_counts_as_failure(tmp_path):
    _record(tmp_path, omit_sample=True)
    result = audit_run(tmp_path)
    assert result["total"]["failed"] == 2
    assert result["per_agent"]["tl1"]["failures"][0]["sim_time"] == 2


def test_sp28_audit_allows_boundary_double_sample(tmp_path):
    _record(tmp_path, duplicated_boundary=True)
    result = audit_run(tmp_path)
    assert result["total"]["agreement_pct"] == 100.0


def test_sp28_audit_unaccounted_interrupted_cycle_blocks_acceptance(tmp_path):
    _record(tmp_path)
    events = [{
        "event": "execution_readback_FAIL",
        "requirement": "SP-28", "status": "FAIL",
        "sim_time": 5, "step_in_cycle": 1,
    }]
    _write(tmp_path / "events_GPT.jsonl", events)
    result = audit_run(tmp_path)
    assert result["coverage_complete"] is False
    assert len(result["unaccounted_execution_faults"]) == 1

def _to_legacy_log(folder):
    """Recreate the archived format: STATE ACK then sample, no sample sequence."""
    log = folder / "display_GPT.jsonl"
    records = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
    receipts = {row["sequence"]: dict(row) for row in records
                if row.get("event") == "ack"}
    samples = [dict(row) for row in records if row.get("event") == "sample"]
    old = []
    for index, sample in enumerate(samples):
        seq = sample.pop("ack_sequence")
        sample.pop("ack_received_at")
        receipt = receipts[seq]
        receipt.pop("ack_received_at")
        # All legacy clocks are monotonic; each sample occurs after the ACK.
        begin = 100.0 + index
        sample["sampled_at"] = begin
        receipt["sent_at"] = begin + 0.1
        receipt["host_monotonic"] = begin + 0.2
        sample["host_monotonic"] = begin + 0.3
        old.extend([receipt, sample])
    _write(log, old)
    return old


def test_sp28_audit_archived_legacy_ack_log_with_ordering_and_timestamps(tmp_path):
    _record(tmp_path)
    _to_legacy_log(tmp_path)
    result = audit_run(tmp_path)
    assert result["total"]["agreement_pct"] == 100.0
    assert result["simulation_steps_assessed"] == 4
    assert result["ack_linkage"]["legacy_inferred_samples"] == 4
    assert result["ack_linkage"]["explicit_sequence_samples"] == 0


def test_sp28_audit_legacy_ack_with_bad_temporal_order_is_rejected(tmp_path):
    _record(tmp_path)
    records = _to_legacy_log(tmp_path)
    # The event stream is intact, but the ACK was recorded before sampling
    # began: legacy timestamp inference must refuse it.
    records[0]["host_monotonic"] = records[1]["sampled_at"] - 0.01
    _write(tmp_path / "display_GPT.jsonl", records)
    result = audit_run(tmp_path)
    assert result["total"]["failed"] == 2
    assert "LEGACY_ACK_ORDER_OR_TIME_INVALID" in result["per_agent"]["tl1"]["failures"][0]["codes"]


def test_sp28_audit_legacy_multiple_ack_before_sample_is_ambiguous(tmp_path):
    _record(tmp_path)
    records = _to_legacy_log(tmp_path)
    # Two STATE ACKs before a single sample: no guessing which is right.
    records.insert(1, dict(records[0]))
    _write(tmp_path / "display_GPT.jsonl", records)
    result = audit_run(tmp_path)
    assert result["total"]["failed"] == 2
    assert result["ack_linkage"]["legacy_unmatched_or_ambiguous_samples"] == 1

def test_sp28_audit_boundary_uses_previous_cycle_post_step_readback(tmp_path):
    _record(tmp_path, duplicated_boundary=True)
    path = tmp_path / "display_GPT.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    boundary_samples = [row for row in rows if row.get("event") == "sample"
                        and row.get("sim_time") == 4]
    assert len(boundary_samples) == 2
    # The second sample reflects the *new* next-cycle programme and must not
    # be compared against the final executed step of the previous cycle.
    boundary_samples[1]["programs"]["tl1"]["green_a"] = 42
    _write(path, rows)
    result = audit_run(tmp_path)
    assert result["simulation_steps_assessed"] == 4
    assert result["total"]["agreement_pct"] == 100.0
