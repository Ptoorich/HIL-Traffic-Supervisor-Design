"""Final HIL system-level acceptance auditor for the two-junction laboratory design.

Read-only: does NOT run SUMO, modify firmware, or infer LED/LCD optical state.
Reads output of run_hil_fault_test_GPT.py, which runs with Slow Monitor OFF.

From the repo root:
  python audit_final_hil_GPT.py --run-dir supervisor_logs/final_hil_acceptance_GPT/seed_42
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path


def _load_json(path):
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None


def _load_rows(path):
    if not path.is_file():
        return None
    try:
        return [json.loads(s) for s in path.read_text(encoding="utf-8").splitlines()
                if s.strip()]
    except (ValueError, UnicodeError) as exc:
        raise ValueError(f"{path}: invalid JSONL: {exc}") from exc


def _result(state, reason, **extra):
    return {"status": state, "reason": reason, **extra}


def _finite_number(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _phase_for_second(t_in_cycle, durations):
    elapsed = 0.0
    for i, duration in enumerate(durations):
        elapsed += duration
        if t_in_cycle < elapsed - 1e-6:
            return i
    return None


def audit_run(folder):
    folder = Path(folder)
    data = {name: _load_json(folder / f"{name}_GPT.json") for name in (
        "metadata", "summary", "fault_test_result", "timing_summary")}
    data.update({name: _load_rows(folder / f"{name}_GPT.jsonl") for name in (
        "display", "observations", "decisions", "events", "timing")})
    checks = {}
    meta = data["metadata"] or {}
    summary = data["summary"] or {}
    fault = data["fault_test_result"] or {}
    checks["software_integration"] = (
        _result("PASS", "Existing twelve-point integrated fault runner passed all checks",
                checks=fault.get("checks"))
        if fault.get("status") == "PASS" and fault.get("checks")
        and all(v is True for v in fault["checks"].values())
        else _result("FAIL" if fault else "UNVERIFIED",
                     "Existing fault-runner result missing or one or more checks failed",
                     checks=fault.get("checks")))

    config_ok = (meta.get("test") == "one_tl1_missing_lane" and
                 meta.get("seed") == 42 and meta.get("cycles") == 10 and
                 meta.get("slow_monitor_enabled") is False and
                 meta.get("auto_handback_enabled") is True and
                 meta.get("sumo_gui") is True)
    checks["test_configuration"] = (
        _result("PASS", "One-shot tl1 lane fault, 10 cycles, GUI+HIL, recovery enabled, Slow Monitor OFF")
        if config_ok else _result("UNVERIFIED", "Metadata does not confirm required final acceptance conditions"))
    checks["control_outcome"] = (
        _result("PASS", "Ten complete cycles, expected tl1 fallback and handback, tl2 independent",
                complete_cycles=summary.get("complete_cycles"))
        if (summary.get("complete_cycles") == 10
            and summary.get("fast_contingencies") == {"tl1": 1, "tl2": 0}
            and summary.get("slow_contingencies") == {"tl1": 0, "tl2": 0}
            and summary.get("handbacks_completed") == {"tl1": 1, "tl2": 0}
            and summary.get("final_modes") == {"tl1": "ASTA", "tl2": "ASTA"})
        else _result("UNVERIFIED" if not summary else "FAIL",
                     "Summary missing or contingency/handback outcome differs from expected"))

    observations = data["observations"]
    if observations is None:
        checks["step_one_programme"] = _result("UNVERIFIED", "No observation log")
    else:
        evaluated = 0
        errors = []
        for r in observations:
            after = r.get("active_programs_after_first_step") or {}
            junctions = r.get("junctions") or {}
            for agent in ("tl1", "tl2"):
                expected_split = (junctions.get(agent) or {}).get("split_after")
                actual = (after.get(agent) or {}).get("green_a")
                evaluated += 1
                if (not _finite_number(expected_split)
                    or not _finite_number(actual)
                    or not math.isclose(expected_split, actual, abs_tol=1e-6, rel_tol=0)
                    or (after.get(agent) or {}).get("states_match") is not True):
                    errors.append({"cycle": r.get("cycle"), "agent": agent,
                                   "expected": expected_split, "readback": actual})
        checks["step_one_programme"] = _result(
            "PASS" if evaluated and not errors else "FAIL",
            "Accepted split compared with first-step SUMO programme readback",
            junction_cycles=evaluated, mismatches=errors[:15])

    display = data["display"]
    if display is None:
        checks["phase_sequence"] = _result("UNVERIFIED", "Missing display log")
        checks["gpio_ack_records"] = _result("UNVERIFIED", "Missing display log")
        checks["serial_session"] = _result("UNVERIFIED", "Missing display log")
    else:
        samples = defaultdict(list)
        states = [r for r in display if r.get("event") == "sample"]
        state_acks = [r for r in display if r.get("event") == "ack" and r.get("kind") == "S"]
        faults = [r for r in display if r.get("event") == "fault"]
        for r in states:
            t = r.get("sim_time")
            if _finite_number(t) and abs(t - round(t)) < 1e-6:
                samples[int(round(t))].append(r)
        # Cycle 0..9, exactly 900 elapsed seconds. At each cycle boundary the
        # LAST sample is the new boundary phase-zero command; the FIRST at the
        # prior end belongs to the preceding completed cycle.
        failed_steps = []
        evaluated_steps = 0
        if not states:
            failed_steps.append({"error": "NO_SAMPLES"})
        for t in range(900):
            batch = samples.get(t) or []
            row = batch[-1] if 1 <= len(batch) <= 2 else None
            if row is None:
                failed_steps.append({"time": t, "error": "MISSING_OR_AMBIGUOUS_SAMPLE"})
                continue
            evaluated_steps += 1
            for i, agent in enumerate(("tl1", "tl2")):
                program = (row.get("programs") or {}).get(agent) or {}
                durations = program.get("durations")
                phases = row.get("phases")
                if (not isinstance(durations, list) or len(durations) != 6
                    or any(not _finite_number(v) or v <= 0 for v in durations)
                    or not math.isclose(sum(durations), 90.0, abs_tol=1e-6)
                    or any(not math.isclose(durations[j], 3, abs_tol=1e-6)
                           for j in (1, 2, 4, 5))
                    or program.get("states_match") is not True
                    or not isinstance(phases, list) or len(phases) != 2):
                    failed_steps.append({"time": t, "agent": agent,
                                         "error": "INVALID_PROGRAMME_OR_SAMPLE"})
                    continue
                expected_phase = _phase_for_second(t % 90, durations)
                if phases[i] != expected_phase:
                    failed_steps.append({"time": t, "agent": agent,
                                         "expected_phase": expected_phase,
                                         "actual_phase": phases[i]})
        checks["phase_sequence"] = _result(
            "PASS" if evaluated_steps == 900 and not failed_steps else "FAIL",
            "Step-indexed 90-second six-phase sequence and 3-second intergreens",
            seconds_evaluated=evaluated_steps, first_mismatches=failed_steps[:20])
        checks["gpio_ack_records"] = _result(
            "PASS" if len(state_acks) == len(states) and not faults else "FAIL",
            "STATE ACK count compared with sampled output events; NOT optical evidence",
            samples=len(states), state_acks=len(state_acks), serial_faults=len(faults))
        stop_ok = (bool(display) and display[-1].get("event") == "ack"
                   and display[-1].get("kind") == "X")
        checks["serial_session"] = _result(
            "PASS" if stop_ok and not faults else "FAIL",
            "Protocol stopped with STOP acknowledgement and no logged serial faults",
            stop_ack=stop_ok)

    timing = data["timing_summary"]
    if timing is None:
        checks["boundary_timing"] = _result("UNVERIFIED", "No timing summary")
    else:
        timings = timing.get("metrics_ms") or {}
        boundary = timings.get("policy_to_boundary_ack_ms") or {}
        failures = timing.get("budget_exceedances") or {}
        complete = (timing.get("measurement_status") == "COMPLETE"
                    and timing.get("boundaries_completed") == 10
                    and boundary.get("n") == 10)
        observed_under = (complete and _finite_number(boundary.get("max"))
                          and boundary["max"] <= 3000 and all(v == 0 for v in failures.values()))
        checks["boundary_timing"] = _result(
            "PASS" if observed_under else ("FAIL" if complete else "UNVERIFIED"),
            "Measured host monotonic policy-start to boundary GPIO ACK; not optical latency, "
            "not separately a check/resolve/logging-overhead demonstration",
            measured_boundaries=timing.get("boundaries_completed"),
            max_policy_to_boundary_ack_ms=boundary.get("max"),
            budget_status=timing.get("budget_status"),
            budget_exceedances=failures)
    # Never infer human observation or URS acceptance from machine logs.
    checks["physical_leds_and_lcd"] = _result(
        "UNVERIFIED", "Requires direct observed colour/phase + LCD TL1/TL2 mode records")
    checks["physical_watchdog_and_power"] = _result(
        "UNVERIFIED", "Requires powered communication-loss fault test and visual record; "
        "USB power loss is NOT fail-safe")
    checks["SP-28_99_9_percent"] = _result(
        "UNVERIFIED", "User-directed: do not treat previous parser artefact or passing tests as measured compliance")
    checks["SP-35_logging_overhead"] = _result(
        "UNVERIFIED", "No matched logging-on/logging-off end-to-end timing evidence")
    acceptance = all(checks[x]["status"] == "PASS" for x in (
        "test_configuration", "software_integration", "control_outcome",
        "step_one_programme", "phase_sequence", "gpio_ack_records",
        "serial_session", "boundary_timing"))
    return {
        "report": "Final HIL system-level laboratory verification",
        "run_dir": str(folder),
        "automated_subset_passed": acceptance,
        "overall_final_hil_verified": False,  # requires physical sign-off, never fabricated
        "checks": checks,
        "limitation": "No optical sensor or independent physical latency clock. "
                      "Some URS-0 targets remain unmet or unverified.",
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True,
                        help="The seed_42 folder with log JSONL and summary JSON files")
    args = parser.parse_args(argv)
    report = audit_run(args.run_dir)
    output = args.run_dir / "final_hil_acceptance_GPT.json"
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n",
                      encoding="utf-8")
    print(f"Saved: {output}")
    for name, result in report["checks"].items():
        print(f"{result['status']:10s} {name}: {result['reason']}")
    return 0 if report["automated_subset_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
