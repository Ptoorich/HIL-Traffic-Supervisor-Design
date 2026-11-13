# SP-28 — execution/readback verification record (investigation; not acceptance)

Date: 2026-11-13
Scope: frozen two-intersection laboratory supervisor, URS-1 SP-28.

## Applicable criterion

- Assessed simulation-step command/readback agreement >= 99.9% (provisional target).
- Detected mismatch flagged within one simulation step and treated as FAIL.
- Distinguish accepted command, SUMO programme/phase, Arduino GPIO acknowledgment, and **directly observed** LED illumination.

## Source review findings

1. `supervisor/observation_adapter_GPT.py::read_active_programs` validates six phases, durations and state strings against environment-held split; mismatch raises ValueError.
2. `supervisor/hil_display_GPT.py::PhaseDisplay.sample` verifies live phase index and actual SUMO state against the validated programme, then transmits phase/mode codes. Mismatch raises DisplayError before STATE is sent. This path runs only if a HIL display is configured.
3. `supervisor/hil_serial_GPT.py::SerialDisplay._exchange` verifies session, sequence, expected GPIO mask and command kind. Incorrect matching-sequence ACK causes DisplayError and an attempted FAULT frame. Missing ACK after retry causes timeout/fault, with an up-to-500-ms acknowledgment deadline. ACK proves firmware/GPIO-level reported output, not optical LED illumination.
4. `supervisor/integrated_runner_GPT.py::EvidenceProbe.step` reads the programme after the first simulation step. An error propagates and interrupts the simulation. It does not create a junction-specific `CheckResult(Status.FAIL)` or automatically latch affected junction to FTSC. Under no-HIL runs, live indication states are not checked on every step; only a first-step programme readback is performed per cycle.
5. `supervisor/supervised_integrated_GPT.py` retains a separate Fast Gate check path at 90-second boundaries. Its regular decision log can have APPROVE/ASTA even when a subsequent execution readback fails.

## Targeted tests authored, not yet executed

`supervisor/tests/test_sp28_readback_GPT.py` on branch `sp28-verification-20261113_GPT` contains seven regression tests:

- correct SUMO programme readback;
- wrong green duration;
- wrong phase-state string;
- wrong live signal indication;
- wrong Arduino GPIO ACK mask;
- missing Arduino ACK;
- injected mismatch at simulation step 1 in a fake integrated run, expected to interrupt before a resolver-level FAIL/contingency.

These are code-level deterministic tests with mocks, not independent SUMO/physical hardware measurements. They intentionally characterise current behaviour and should **not** be reinterpreted as an acceptance test for SP-28.

## Execution instructions

From the repository root on the SP-28 branch, in the project's activated Python environment:

```powershell
python -m pytest -q supervisor/tests/test_sp28_readback_GPT.py
```

Then execute the broader existing regression suite:

```powershell
python -m pytest -q supervisor/tests
```

Retain the actual console output and any resulting fault logs. Tests may reveal discrepancies and must not be labelled PASS before execution.

## Remaining acceptance evidence

- Instrument and audit all assessed steps and calculate numerator/denominator of successful accepted-command versus SUMO programme/phase readbacks; define how missing readback is counted.
- Verify response to injected faults at multiple arbitrary simulation steps and demonstrate <=1-step detection, including when no HIL display is attached.
- Decide/design how detected execution mismatch should be converted into FAIL and junction-specific FTSC; an exception that simply stops the laboratory run does not satisfy this portion.
- Independently verify GPIO response; photograph/video directly observed LEDs for optical claims. Never infer physical illumination from GPIO ACK.
- Retain SP-28 as **partial / not accepted** until the full specified response and agreement target are demonstrated.

No operational controller or firmware files were modified in this investigation.

## Implementation update — 13 November 2026 (pending execution)

New files:
- `supervisor/sp28_monitor_GPT.py`: structured readback failure with requirement, FAIL, source, junction (if determinable), event timing, and safe-abort response.
- `audit_sp28_GPT.py`: programme/GPIO agreement audit using one run folder's `observations_GPT.jsonl`, `display_GPT.jsonl`, and optional `events_GPT.jsonl`.
- `supervisor/tests/test_sp28_audit_GPT.py`: five deterministic audit tests.

Changed:
- `supervisor/integrated_runner_GPT.py`: every executed simulation step now validates active SUMO programme even without HIL; if HIL is enabled, its existing every-step live-phase/GPIO-ACK verification continues. An SP-28 failure is logged to `events_GPT.jsonl` before the runner stops.
- `supervisor/tests/test_sp28_readback_GPT.py`: the injected first-step mismatch test now requires a structured FAIL event at simulation time 1; the normal Fast Gate may still have approved the prior command (this is a distinct downstream execution fault).

### Execute tests

```powershell
git fetch origin
git switch sp28-verification-20261113_GPT
python -m pytest -q supervisor/tests/test_sp28_readback_GPT.py supervisor/tests/test_sp28_audit_GPT.py
python -m pytest -q supervisor/tests
```

Record exact pass/fail outputs and Python/SUMO versions. This branch was prepared remotely; **tests were not executed in the assistant's environment**.

### Audit a completed HIL run

```powershell
python audit_sp28_GPT.py --run-dir "PATH\TO\ONE\RUN_FOLDER"
```

The target folder needs both `display_GPT.jsonl` and `observations_GPT.jsonl` from the same run. If logs are packaged as a ZIP, extract them first. The script writes `sp28_agreement_GPT.json` with counts for each junction and a combined percentage.

The audit counts a junction-step as success only if: a recorded, uniquely matched STATE ACK exists with the correct combined GPIO mask; the HIL sample includes validated signal-state/programme evidence; and the programme phase A split matches the approved split recorded in cycle observations. Missing, duplicated or inconsistent records count as failures. Cycle boundaries may have two samples (end of previous cycle, and next cycle phase-zero reset); the latter is compared to the next approved programme. Optical LEDs are excluded from the numerical agreement count.

**Limitations:** The percentage describes only the reconstructed assessed steps. Interrupted runs lacking a completed observation cycle cannot support full-run compliance. It is not statistical evidence of overall reliability or independent optical verification. The *mere fact* that the tests pass must not be reported as 99.9% compliance.

### Physical LED observation log (fill only what was actually seen)

| Item | Observation | Evidence file / time |
|---|---|---|
| Junction tl1 Phase A green, Phase B red | To record | |
| tl1 A yellow, B red | To record | |
| tl1 all red | To record | |
| tl1 A red, B green | To record | |
| tl1 A red, B yellow | To record | |
| Junction tl2: repeat corresponding six states | To record | |
| Fault, STOPPED and watchdog while Arduino remains powered | To record | |

Note any defective LEDs/wiring, unexpected colours, and whether a state was directly observed or inferred from GPIO readback. Visual records establish only those observed states, not all-step automatic optical verification.

### Reporting status

SP-28 remains **not yet fully accepted** until tests and controlled-run audit are completed. The FAIL record represents a post-execution verification fault and stops the experimental run. A FAIL event is not, by itself, a successful safe FTSC handover.

## Archived HIL log audit compatibility (13 November 2026)

The first audit of the pre-existing 10-cycle HIL record returned 0/1802 agreement, **but this was a parser artefact, not observed optical mismatch**. The older `sample` records omit `ack_sequence` and `ack_received_at`; earlier auditor versions therefore classified every otherwise available STATE ACK as missing.

The revised auditor:
- pairs a legacy `sample` to a *single* preceding STATE `ack` in the same USB session;
- requires monotonically consistent `sampled_at <= ack.sent_at <= ack.host_monotonic <= sample.host_monotonic`; multiple unmatched ACKs or invalid timestamps are failures, never assumed matches;
- uses explicit `(session, ack_sequence)` linkage when available in newer logs;
- assesses actual executed simulation steps `start_time+1 .. end_time`, excluding the initial time-zero boundary; at a cycle end with two recorded samples, uses the **first** (post-step, old programme), not the subsequent phase-zero sample for the next cycle.

For the archived 900-second ten-cycle run, the corrected step-based denominator should be **900 simulation steps × 2 junctions = 1,800 junction-step checks**, assuming all ten cycles were executed and logged. The pass percentage itself must be determined by rerunning the script on the original ZIP extraction; it must not be inferred from test-suite success or the prior 0% output.

Legacy ACK linkage is weaker than explicit sampled sequence IDs. Present any passing archived percentage as retrospective evidence for the assessed recorded steps, not proof of all-step optical agreement or independent field reliability. The original invalid audit record should be superseded, not cited as an operational failure.
