# Final System-Level HIL Verification Protocol — GPT
Date: 13 November 2026
Scope: Two-intersection SUMO/TraCI, MARL PPO, integrated supervisor, Arduino Uno, four RGB signal heads (12 LEDs), one 16×2 LCD at 0x27. Preserve SP-28 numerical result as UNVERIFIED.

## Acceptance approach
Use a repeat of the existing 10-cycle HIL one-shot TL1 input fault and handback demonstration, with the **final physical single-LCD firmware**, GUI, serial display, and hardware observed. Do NOT conduct any additional Slow Monitor performance experiments. The test runner itself sets slow_enabled=False and handback_enabled=True. No numerical specification is accepted solely from source review, unit-test passes, or a GPIO acknowledgement.

## 1. Preflight
Run these in Windows CMD, from the repository root, with the .venv active:

    git status --short
    git pull origin sp28-verification-20261113_GPT
    python -m pytest -q supervisor/tests
    python bench_arduino_GPT.py --list-ports

Do not overwrite local uncommitted work. Confirm the final one-LCD Arduino sketch is flashed. IMPORTANT: the archived Arduino sketch in the connected GitHub repo still contains LED+serial+watchdog but not the later single-LCD firmware. Use the user's actual local final firmware, not the older repo sketch. Record its filename, SHA-256, board/AVR package, library versions and compile/upload outcome:

    certutil -hashfile "PATH_TO_FINAL_LCD_SKETCH.ino" SHA256

Confirm all 12 LEDs are functional and mapped correctly. The earlier TL1-A yellow lamp repair must be physically checked. USB provides power; the Arduino cannot light LEDs on power loss. Check one LCD at I2C 0x27, TL1 top row, TL2 bottom row; LCD connects A4 SDA, A5 SCL, 5V, GND. Close the Arduino Serial Monitor before opening COM5 in Python. Confirm port, actual resistors and breadboard connections. Take one as-built photo.

## 2. Final integrated demonstration (NO Slow Monitor experiment)
From CMD, use a NEW output directory. Substitute the correct port if not COM5:

    python run_hil_fault_test_GPT.py --port COM5 --speed 1 --gui --run-dir supervisor_logs\final_hil_acceptance_01_GPT

The existing runner injects a single missing-lane supervisor snapshot at t=90 simulated seconds for TL1 only. It does not alter SUMO's actual lane network or the RL inputs. It runs 10 x 90 s cycles and tests independent TL1 fallback, recovery transition and handback, plus TL2 uninterrupted ASTA.

Expected checkpoints (mark observed YES/NO and photograph/video with simulation timestamps):
- Initial: TL1=ASTA and TL2=ASTA on separate LCD rows, GUI and four signal heads agree visually.
- At simulated t=90 boundary: TL1 enters FTSC 31/47, TL2 remains ASTA; LCD accurately distinguishes both.
- Under FTSC: TL1 displays each of its six phase states (green A, yellow A, all red, green B, yellow B, all red) without contradictory green heads.
- Qualified transition: TL1 LCD indicates FTSC transition for one extra FTSC cycle, with no interrupted intergreen.
- Gated release: TL1 returns to ASTA, independent TL2 remains ASTA.
- End: LCD displays STOPPED; all four physical RED lamps steadily ON, yellows and greens OFF. USB remains powered.
- Record at least one complete signal sequence at each junction, including 3 s yellow and 3 s all-red per phase using GUI/video. Human visual agreement is qualitative, not all-step optical verification.

The built-in runner writes fault_test_result_GPT.json containing checks for injection, FTSC, transition/handback, 10 complete cycles, SUMO state readback, all six phases, per-second samples, and STOP ACK. It cannot prove actual physical light emission.

## 3. Automated logs and timing audit
Run after the integrated experiment finishes:

    python audit_final_hil_GPT.py --run-dir "supervisor_logs\final_hil_acceptance_01_GPT\seed_42"

The script creates final_hil_acceptance_GPT.json and prints checks for the supported *recorded* behaviour: configuration, first-step programme readback, complete six-phase cycle and 3-second intergreens, one STATE ACK per sample, STOP and measured host boundary timing. Its physical optical result, SP-28 result and SP-35 result deliberately remain UNVERIFIED.

Retain the raw display_GPT.jsonl, timing_GPT.jsonl, timing_summary_GPT.json, decisions_GPT.jsonl, events_GPT.jsonl, observations_GPT.jsonl, fault_test_result_GPT.json, summary_GPT.json, metadata_GPT.json and final_hil_acceptance_GPT.json.

Timing: policy_to_boundary_ack_ms is host-monotonic from policy call start to validated GPIO ACK after phase-zero update. This includes model call and host integration but NOT physical optical light onset; first-step timing includes deliberate pacing. Distinguish this from individual check <=1 s, resolver <=0.1 s and the wider proposal-to-decision <=3 s in original SP-21. SP-35 needs a matched logging-on/off overhead comparison and cannot be claimed from this single run.

## 4. Powered communication loss, STOP and display states
After the integrated test, with the same final sketch, run:

    python bench_arduino_GPT.py --port COM5 --watchdog-test --log supervisor_logs\final_watchdog_GPT.jsonl

Leave USB connected during the test. Confirm LCD FAULT, all four red lamps flashing, and no stale ASTA or FTSC text after ~3 s of silence. Note how response time is observed; an exact optical <=3 s claim requires proper timing measurement. The firmware's powered watchdog does NOT protect against USB power failure. After the latched fault, reset the board before another test.

## 5. URS compliance register (pre-test; update only from actual evidence)
Keywords: SUPPORTED = source/tests support implemented mechanism, not full numerical claim; PARTIAL = incomplete acceptance coverage; UNVERIFIED = no acceptance basis; DEVIATION = known departure from unchanged URS-0.

| ID | Preliminary status | Evidence / acceptance limitation |
|---|---|---|
| C-01, C-02, C-03 | SUPPORTED | Two intersections, SUMO and TraCI integration |
| SP-01 | PARTIAL | Vehicle two-phase green/yellow/all-red; pedestrian clearance not verified |
| SP-02 | PARTIAL | Hard-constraint injection tests, but 100% rejection and zero false-veto not comprehensively tabulated |
| SP-03 | DEVIATION | Implemented fixed 90 s cycle and A/B order, not free legal cycle/offset choices |
| SP-04 | UNVERIFIED | No demonstrated >=95% held-out degenerate-policy sensitivity; no new Slow Monitor experiment |
| SP-05 | PARTIAL | Slow Monitor next-boundary scheduling; detection sensitivity across targets unverified |
| SP-06 | UNVERIFIED | No 24 h false-alert rate for healthy operation |
| SP-07 | PARTIAL | Bound and alternating behaviour definitions; starvation catalogue remains incomplete |
| SP-08 | PARTIAL | Detector-level Fast Gate inputs; original metric-only requirement needs exact trace check |
| SP-09 | UNVERIFIED | No >=90% independent classifier reference label agreement |
| SP-10 (URS-1) | UNVERIFIED | Implemented 90 s boundary; no 100% all-onset gridlock detection + 24 h false alarms |
| SP-11 | PARTIAL | Conflict fixture and code checks; exhaustive 100% with zero false-alarm record outstanding |
| SP-12 | SUPPORTED | Conflict fixture derived from SUMO movement relationships |
| SP-13 | DEVIATION | Range/plausibility implemented, independent frozen-value detection unimplemented |
| SP-14 | DEVIATION | Observed occupancy fraction and speed/capacity tolerances differ from literal URS-0 ranges |
| SP-15 | SUPPORTED | One-shot missing-lane fault, veto and FTSC at boundary |
| SP-16 | DEVIATION | Original FTSC rolling performance thresholds not used; implemented backlog + policy Slow Monitor |
| SP-17 | UNVERIFIED | Immediate injected teleport alert not demonstrated |
| SP-18 | UNVERIFIED | No 24 h low-false-trigger verification |
| SP-19 | DEVIATION | URS-0 all PASS requirement; implemented APPROVE_WARN can approve |
| SP-20 | SUPPORTED | Error/timeout/missing result treated as fail-closed |
| SP-21 | PARTIAL | Scope-labelled timing samples; entire original 3 s/check <=1 s criterion not yet proven |
| SP-22 | PARTIAL | Decision-boundary FTSC issuance and timing recorded; universal deadline not established |
| SP-23 | PARTIAL; RETEST | Final phase trace and optical observations at switch boundaries |
| SP-24 | SUPPORTED | Webster fallback programme readback, phase and conflict checks |
| SP-25 | SUPPORTED | FTSC preset present at start-up independent of ASTA |
| SP-26 | SUPPORTED | Five-cycle dwell, three clean cycles, transition and fresh gate; targeted handback |
| SP-27 | PARTIAL; RETEST | Accepted split and first simulation step programme match for each completed cycle |
| SP-28 (URS-1) | UNVERIFIED | User decision; numerical >=99.9% agreement not established; GPIO != optics |
| SP-29 (URS-1) | PARTIAL; RETEST | Permitted programmes and observed group signals; no independent all-step optical proof |
| SP-30 | PARTIAL; RETEST | 6-phase sequences, 3 s intergreens in logged simulation and observed indications |
| SP-31 | SUPPORTED | Four representative logical A/B heads (12 LEDs) |
| SP-32 | PARTIAL; RETEST | Per-step phase records, not continuous physical optical recording |
| SP-33 | PARTIAL; RETEST | Powered 3 s watchdog and FAULT/red flash; no power-loss fail-safe |
| SP-34 | PARTIAL; RETEST | Structured per-cycle decision/error/readback logs; audit completeness and timing fields |
| SP-35 (URS-1) | UNVERIFIED | Matched logging enabled/disabled overhead not quantified |
| SP-36 | UNVERIFIED | Decision replay from logs not demonstrated |
| UR-05 | SUPPORTED WITH SCOPE | SARL/MARL PPO/DQN same supervisor interface; earlier runs not field certification |

## 6. Final archival / sign-off
Record run date, engineer/observer, COM port, Arduino sketch filename and SHA-256, IDE/board package, LCD library and I2C address, actual wiring/resistors, camera/photo IDs, SUMO and Python versions, model/route hashes, detailed observations at t=90 / transition / handback / STOPPED / FAULT, timing summary, exceptions, and any mismatch. Keep the raw run directory unchanged; do not conflate GPIO acknowledgement with physical LED performance. Final Design Specification should present a concise PASS/PARTIAL/UNVERIFIED/DEVIATION summary with evidence IDs; do not silently rewrite URS-0 or force SP-28 to PASS.
