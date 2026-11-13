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
