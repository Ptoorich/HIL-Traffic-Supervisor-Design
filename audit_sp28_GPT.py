"""SP-28 HIL readback agreement audit. Does not infer physical optical output.

Example:
python audit_sp28_GPT.py --run-dir supervisor_logs/hil_fault_handback_01_GPT/seed_42

Requires display_GPT.jsonl and observations_GPT.jsonl from one completed run.
If execution was interrupted, supply events_GPT.jsonl; the audit counts up to
the failure step, not unexecuted future steps.
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

from supervisor.hil_serial_GPT import PHASE_MASKS


def read_jsonl(path):
    if not path.is_file():
        return []
    records = []
    with path.open(encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, 1):
            if line.strip():
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{path.name}:{line_no}: invalid JSON") from exc
    return records


def _step(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if not math.isfinite(value) or not math.isclose(value, round(value), abs_tol=1e-6):
        return None
    return int(round(value))


def audit_run(folder):
    folder = Path(folder)
    displays = read_jsonl(folder / "display_GPT.jsonl")
    observations = read_jsonl(folder / "observations_GPT.jsonl")
    events = read_jsonl(folder / "events_GPT.jsonl")
    if not displays:
        raise ValueError("No display_GPT.jsonl rows. GPIO agreement is not assessable.")
    if not observations:
        raise ValueError("No completed observation cycles; cannot reconstruct expected command timeline.")
    samples = defaultdict(list)
    acks = defaultdict(list)
    for row in displays:
        if row.get("event") == "sample":
            timestamp = _step(row.get("sim_time"))
            if timestamp is not None:
                samples[timestamp].append(row)
        elif row.get("event") == "ack" and row.get("kind") == "S":
            seq = row.get("sequence")
            if type(seq) is int:
                acks[(row.get("session"), seq)].append(row)
    expected = {}
    for observation in observations:
        start = _step(observation.get("start_time"))
        end = _step(observation.get("end_time"))
        junctions = observation.get("junctions", {})
        if start is None or end is None or end < start:
            raise ValueError("Invalid observation cycle boundary")
        for t in range(start, end):
            if t in expected:
                raise ValueError(f"Overlapping observation coverage at step {t}")
            expected[t] = {a: x.get("split_after") for a, x in junctions.items()}
    if not expected:
        raise ValueError("No recorded simulation steps found.")
    # The final boundary is a real display sample and must be accounted for,
    # though it may represent the next phase reset if a following cycle exists.
    max_t = max(expected) + 1
    final = observations[-1]["junctions"]
    expected[max_t] = {a: x.get("split_after") for a, x in final.items()}

    faults = [x for x in events if x.get("event") == "execution_readback_FAIL"]
    # An interrupted cycle may not produce an observations row. In such cases
    # we cannot reconstruct its approved split; flag coverage as incomplete.
    unaccounted_faults = [x for x in faults if _step(x.get("sim_time")) not in expected]
    agents = sorted({a for row in expected.values() for a in row})
    outcomes = {a: {"assessed": 0, "agreed": 0, "failed": 0,
                    "failures": []} for a in agents}
    for sim_t, commanded in sorted(expected.items()):
        records = samples.get(sim_t, [])
        row = records[0] if len(records) == 1 else None
        for index, agent in enumerate(("tl1", "tl2")):
            if agent not in commanded:
                continue
            target = outcomes[agent]
            target["assessed"] += 1
            failures = []
            if row is None:
                failures.append("MISSING_OR_DUPLICATED_SAMPLE")
            else:
                programs = row.get("programs") or {}
                p = programs.get(agent) or {}
                split = commanded[agent]
                if (not isinstance(split, (int, float))
                    or not isinstance(p.get("green_a"), (int, float))
                    or not math.isclose(float(split), float(p["green_a"]),
                                        rel_tol=0, abs_tol=1e-6)):
                    failures.append("PROGRAMME_SPLIT_MISMATCH")
                if p.get("states_match") is not True:
                    failures.append("STATES_NOT_VERIFIED")
                phases = row.get("phases")
                if not isinstance(phases, list) or len(phases) != 2 or any(
                    type(x) is not int or x not in range(6) for x in phases
                ):
                    failures.append("INVALID_PHASE_CODES")
                seq = row.get("ack_sequence")
                session = row.get("session")
                receipts = acks.get((session, seq), [])
                if len(receipts) != 1:
                    failures.append("MISSING_OR_DUPLICATED_STATE_ACK")
                elif isinstance(phases, list) and len(phases) == 2 and all(
                    type(x) is int and x in range(6) for x in phases
                ):
                    expected_mask = PHASE_MASKS[phases[0]] | (PHASE_MASKS[phases[1]] << 6)
                    if receipts[0].get("mask") != expected_mask:
                        failures.append("GPIO_ACK_MASK_MISMATCH")
                    if (not isinstance(row.get("ack_received_at"), (int, float))
                            or receipts[0].get("ack_received_at") != row["ack_received_at"]):
                        failures.append("ACK_TIMESTAMP_MISMATCH")
            if failures:
                target["failed"] += 1
                if len(target["failures"]) < 25:
                    target["failures"].append({"sim_time": sim_t, "codes": failures})
            else:
                target["agreed"] += 1
    total = {"assessed": sum(x["assessed"] for x in outcomes.values()),
             "agreed": sum(x["agreed"] for x in outcomes.values()),
             "failed": sum(x["failed"] for x in outcomes.values())}
    for agent, row in outcomes.items():
        row["agreement_pct"] = round(100 * row["agreed"] / row["assessed"], 6) if row["assessed"] else None
    total["agreement_pct"] = round(100 * total["agreed"] / total["assessed"], 6) if total["assessed"] else None
    return {
        "requirement": "SP-28",
        "scope": "SUMO validated programme, HIL protocol and GPIO readback; not optical output",
        "run_dir": str(folder),
        "per_agent": outcomes,
        "total": total,
        "threshold_pct": 99.9,
        "threshold_met_for_assessed_steps": (total["agreement_pct"] is not None
                                              and total["agreement_pct"] >= 99.9),
        "coverage_complete": not unaccounted_faults,
        "unaccounted_execution_faults": unaccounted_faults,
        "caution": ("A threshold pass is only for steps reconstructed from completed observation "
                    "cycles. No physical LED illumination or general reliability is established. "
                    "Unaccounted interrupted-cycle faults prevent a full-run acceptance claim."),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = audit_run(args.run_dir)
    output = args.output or args.run_dir / "sp28_agreement_GPT.json"
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n",
                      encoding="utf-8")
    print(json.dumps({"output": str(output), "total": report["total"],
                      "coverage_complete": report["coverage_complete"]}, indent=2))
    return 0 if report["threshold_met_for_assessed_steps"] and report["coverage_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
