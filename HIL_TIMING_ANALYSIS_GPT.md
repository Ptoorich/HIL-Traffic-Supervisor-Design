# HIL validation: fault recovery and continuous timing

Updated: 6 November 2026 (Africa/Johannesburg). The initial fault-run analysis was completed on 5 November; the follow-up timing summary was supplied on 6 November. Neither evidence source independently establishes its original execution date.

**Current outcome: the original physical fault/handback run passed its recorded functional and GPIO checks. The subsequent instrumented run reports three complete continuous timing measurements, all within the configured 3,000 ms budget for the recorded host-to-GPIO-ACK scopes. The broadest observed maximum is 1,087.457 ms, including deliberate pacing.** This is an observed result for three boundaries, not a proven worst-case deadline or optical response measurement. The assistant did not perform a physical COM5 run.

## Follow-up physical timing result — 6 November 2026

Evidence: the user pasted the complete JSON printed from `supervisor_logs\hil_timing_01_GPT\seed_42\timing_summary_GPT.json` after installing the continuous-timing update. The prescribed command used the existing MARL PPO model, GUI, COM5, speed 1 and three cycles with Slow Monitor and automatic handback enabled. These run settings come from the prescribed command and context; the new run's metadata and raw JSONL files have not been independently supplied/audited.

The pasted summary reports `measurement_status=COMPLETE`, 3 started and 3 completed boundaries, zero incomplete/invalid boundaries, no interruption, `budget_status=WITHIN_OBSERVED_BUDGET`, and zero exceedances in all four compared scopes. It reports zero uncaught policy-call exceptions. That last count does not establish the absence of observation errors or errors caught within `StrictPolicy`; those have separate controller logs.

| Measurement | n | Average (ms) | Maximum (ms) |
|---|---:|---:|---:|
| Policy call, including observation validation and predictions | 3 | 20.188 | 49.501 |
| Policy completion to boundary output ACK | 3 | 23.175 | 25.776 |
| Policy start to boundary output ACK | 3 | 43.364 | 71.318 |
| Boundary processing start to boundary output ACK | 3 | 46.452 | 73.411 |
| Policy completion to first-step output ACK | 3 | 1,039.518 | 1,042.710 |
| Policy start to first-step output ACK, including pacing | 3 | 1,059.706 | 1,085.365 |
| Boundary processing start to first-step output ACK, including pacing | 3 | 1,062.795 | 1,087.457 |
| Policy start to first-step output ACK, excluding measured pacing | 3 | 59.433 | 85.311 |
| Pacing requested | 3 | 999.898 | 999.917 |
| Pacing actually spent in sleep | 3 | 1,000.273 | 1,000.418 |
| Decision return to boundary output ACK | 3 | 13.365 | 14.576 |
| First simulation step and readback to output ACK | 3 | 15.725 | 17.435 |

The broadest recorded maximum occupies 36.249% of the 3,000 ms allowance, leaving **1,912.543 ms of observed margin**. The approximately one-second pacing interval is intentional at speed 1; it is included in the first-step totals and excluded only in the explicitly derived field. The immediate boundary-output ACK is already received before that sleep and first simulation step. Thus the 1.087-second value should not be described as a one-second delay before any traffic output is acknowledged.

The earlier measurement gap is now addressed for the defined decision-response path: the new timers continuously include boundary preparation (in the broadest scope), the policy call, gate/resolver work, logging before the endpoint, programme application, readback and the correlated validated ACK. The first-step endpoint additionally includes pacing and one actual SUMO step. Model loading, startup/reset, the rest of the 90-second cycle, work/logging after the endpoint, GUI rendering and optical LED response are outside these intervals.

There are only three boundary observations. The reported high percentiles are interpolations within this small sample and should not be presented as established tail-latency estimates. The summary does not identify the cycle or cause of the 49.501 ms maximum policy call, so no warm-up attribution or performance change is inferred. No repeat of the already successful fault/handback demonstration is required by this result.

The following sections retain the independent raw-log audit of the original ten-cycle fault run. Its totals, counts and hashes refer to that original archive, not the follow-up three-cycle run.

## Original fault run: evidence

Source: `hil_fault_handback_evidence_01_GPT.zip`, containing eight files from `supervisor_logs\hil_fault_handback_01_GPT\seed_42` on the user's Windows machine. Configuration: MARL PPO, seed 42, COM5, SUMO GUI, speed 1, automatic handback enabled, Slow Monitor disabled, 10 complete 90-second cycles. The test omits one lane from one TL1 supervisor snapshot at t=90; it does not remove the actual SUMO lane or change the policy's observations.

The result file reports all 12 checks passing. This analysis additionally parses every log record and performs 27 consistency checks listed below. The archive contains 10 decision records, 10 cycle observations, 5 control events, 910 display samples, 913 ACKs and one READY record. Model and runner identities are recorded in metadata; their binary/source files are absent, so their hashes cannot be independently recomputed here. The included route file's SHA-256 was recomputed and matches metadata.

## Original fault run: measured timing

| Measurement | n | Mean (ms) | Median (ms) | P95 (ms) | P99 (ms) | Maximum (ms) |
|---|---:|---:|---:|---:|---:|---:|
| Snapshot acquisition | 10 | 4.163 | 4.015 | 5.597 | 5.603 | 5.605 |
| Fast Gate evaluation alone | 10 | 0.944 | 0.816 | 1.676 | 1.760 | 1.781 |
| Snapshot + Fast Gate stage | 10 | 5.107 | 4.699 | 7.271 | 7.348 | 7.367 |
| Resolver | 10 | 0.094 | 0.082 | 0.160 | 0.186 | 0.192 |
| Supervisor subtotal before logging | 10 | 5.315 | 4.916 | 7.580 | 7.701 | 7.731 |
| STATE serial round trip | 910 | 10.603 | 10.578 | 11.221 | 11.437 | 11.914 |
| Observation-to-ACK | 910 | 14.043 | 14.125 | 15.818 | 16.332 | 17.453 |
| Logged pacing lag | 910 | 0.444 | 0.440 | 0.732 | 0.988 | 1.470 |

P95 and P99 use linear interpolation at index `(n - 1) × percentile` in the sorted observations. Every available record is included, with no warm-up or outlier removal. The stage timings have only 10 samples; their high percentiles are descriptive, not stable tail estimates. Display and serial measurements have 910 samples. Per-junction copies of gate timing are not counted twice. The combined Fast Gate stage is computed per decision as `snapshot_ms + gate_ms`, then summarised.

The logged supervisor scope is `post_prediction_through_action_preparation; excludes log, env.step and LED`. Consequently, the 7.731 ms maximum is a subtotal: it excludes policy inference, logging, SUMO stepping and serial/LED output. The observation-to-ACK field is a separate measurement of the recorded readback-to-acknowledgement path; deliberate pacing is excluded according to the project handover. It is not a direct optical measurement, a GUI-rendering measurement, or a timestamped fault-injection-to-lamp measurement.

## Targets and supported conclusions

Targets below come from the project handover; the deployed configuration file itself is not included in this archive. The available earlier supervisor source also charges snapshot time to the Fast Gate stage. The run's own budget flags are all false.

| Requirement or target | Evidence | Assessment |
|---|---|---|
| Fast Gate stage, including snapshot: 2,000 ms | Maximum 7.367 ms across 10 decisions; no stage timeout flag | Met in this run |
| Resolver: 100 ms | Maximum 0.192 ms across 10 decisions; no resolver budget flag | Met in this run |
| ACK deadline: 500 ms | Maximum 11.914 ms across all 913 ACKs | Met in this run |
| Retry point: 250 ms | All 913 ACKs recorded `attempts=1` | No retry was needed; retry handling was not exercised |
| Provisional whole-supervisor budget: 3,000 ms | Recorded subtotal maximum 7.731 ms; unmeasured sections remain | Not measured in this original archive; see the follow-up continuous measurements above |
| Communication freshness watchdog: 3,000 ms | Largest host send-to-send gap 1034.046 ms; uninterrupted ACK stream | Supports communication continuity; does not test expiry or establish device-side arrival gaps |
| Physical LED illumination or optical latency | GPIO masks and readback flags are logged; no optical measurement supplied | Not established by this archive |

There are no logged communication-fault or timeout events. This does not establish a worst-case execution bound or a future failure probability. The Slow Monitor, SARL and other model/demand conditions were not exercised by this run.

## Serial and output audit

ACK sequences are continuous from 0 to 912 in one session: one handshake (H), one initialization pause (P), 910 state commands (S) and one normal stop (X). Every state sample immediately follows a matching same-session state ACK. All 910 masks agree with the phase-to-pin mapping for D2-D13. The final stop acknowledges mask 2340, the four-red GPIO state.

All six phase codes occur at both junctions. The sample stream contains every integer simulation time from 0 through 900 plus the nine additional boundary samples at 90, 180, ..., 810. These represent the cycle-end readback and the subsequent boundary action at the same simulation time, not duplicate sequence numbers or retries. Every recorded programme `states_match` flag is true. This audit checks recorded evidence; it does not independently observe SUMO or the board.

## Fault and recovery sequence

| Simulation time | Evidence-backed result |
|---:|---|
| 0 s | Both junctions in ASTA |
| 90 s | One TL1 input-validator failure for missing lane `596638461#0_0`; TL1 enters FTSC and TL2 remains ASTA |
| 91 s | Contingency programme is additionally recorded in control events with A/B = 31/47 s |
| 540 s | TL1 begins transition after five complete contingency cycles and four clean cycles; FTSC still controls |
| 630 s | Transition finishes; a fresh approved gate decision accepts TL1 back to ASTA |
| 720 s | A full returned ASTA cycle verifies handback |
| 900 s | Run completes; normal-stop ACK reports all red |

The raw display mode changes agree with the result JSON. During both FTSC and its transition, every TL1 sample records durations `[31, 3, 3, 47, 3, 3]`. TL2 remains ASTA throughout. There is exactly one TL1 contingency entry and one completed TL1 handback, no partial unverified handback and no Slow Monitor intervention.

## Wall-clock pacing

The first and last `sampled_at` timestamps span **915.282053 s** for 900 simulated seconds: approximately **15 min 15.28 s**, with **15.282053 s (1.698%)** extra elapsed time relative to an exact 1× duration. The measured effective rate is **0.983303 simulated seconds per wall-clock second**. READY-to-final-stop spans 916.219028 s; this excludes any process startup before READY.

The `lag_ms` field peaks at 1.470 ms, while cumulative wall-clock excess reaches 15.28 s. These are different quantities. The subsequently supplied HIL source confirms that `lag_ms` measures lateness against the local pre-step pacing target (`last_wall + 1/speed`), with `last_wall` refreshed after each displayed sample. It does not measure total drift from the initial wall-clock timestamp. The elapsed excess does not mean the LEDs were 15 seconds behind SUMO: the recorded observation-to-ACK maximum remains 17.453 ms. The archive does not isolate the causes of accumulated pacing overhead. No pacing correction is justified solely by this result unless exact wall-clock speed is a project requirement.

## Current project checkpoint

- The physical ten-cycle fault/handback test is complete, with recorded software and GPIO checks passing. TL1 alone falls back and returns after qualified recovery. This supersedes the earlier handoff's statement that physical execution was unreported.
- Continuous timing instrumentation is implemented and its first physical collection has been reported: 3/3 complete boundaries and zero observed 3,000 ms exceedances. Retain the exact endpoint scopes and small-sample limitation.
- The update was based on the user's current source ZIP; the produced package is `hil_end_to_end_timing_update_GPT.zip`. Its four production files cover the transport receipt timestamp, display sample correlation/pacing measurement, integrated loop hooks and new timing collector. The existing control rules and Arduino protocol remain the basis of the test.
- Keep LCD wiring, address scans and LCD firmware work paused, as requested.
- Next, preserve the new run's top-level evidence files and updated source in the user's Git repository. Inspect current Windows `git status --short` before choosing the files to stage; preserve unrelated edits and avoid adding copied simulation work trees or backup directories.
- Visual LED illumination and the powered-watchdog observation remain separate evidence. This summary supplies no new optical measurement or communication-loss test.

Use the continuous timing fields when discussing the decision-response budget. Do not sum maxima from separate fields/runs or describe the largest observed value as a guaranteed worst case.

## Per-boundary timing

| Boundary (sim s) | Snapshot + gate (ms) | Resolver (ms) | Supervisor subtotal (ms) |
|---:|---:|---:|---:|
| 0 | 7.367 | 0.093 | 7.731 |
| 90 | 4.239 | 0.065 | 4.429 |
| 180 | 7.153 | 0.192 | 7.396 |
| 270 | 4.437 | 0.074 | 4.563 |
| 360 | 3.714 | 0.086 | 3.848 |
| 450 | 4.336 | 0.067 | 4.459 |
| 540 | 5.379 | 0.080 | 5.562 |
| 630 | 5.111 | 0.080 | 5.333 |
| 720 | 4.961 | 0.120 | 5.193 |
| 810 | 4.371 | 0.085 | 4.638 |

## Independent consistency checks

| Independent check | Outcome |
|---|---|
| reported all 12 checks pass | PASS |
| single session | PASS |
| expected event types and counts | PASS |
| ack kind counts | PASS |
| continuous ack sequence | PASS |
| all acknowledged first attempt | PASS |
| all acks under 500ms | PASS |
| host timestamps monotonic | PASS |
| all 910 samples match immediately preceding state ack mask | PASS |
| both junctions visit all six phases | PASS |
| all program state match flags true | PASS |
| normal stop is final ack with all red mask | PASS |
| all seconds and nine repeated boundaries | PASS |
| mode timeline matches raw samples and report | PASS |
| tl2 stays in asta | PASS |
| ftsc and transition use 31 47 program | PASS |
| one matching missing lane failure | PASS |
| ten complete contiguous cycles | PASS |
| ten decision boundaries | PASS |
| no logged budget flags | PASS |
| gate and resolver below documented targets | PASS |
| transition qualified | PASS |
| release accepted by fresh gate | PASS |
| one completed returned cycle | PASS |
| slow monitor disabled throughout | PASS |
| summary counts agree | PASS |
| route bytes match metadata hash | PASS |

## Provenance

The following hashes identify the analysed bytes. The route hash matches the run metadata. Analysis read the ZIP directly without modifying it.

| File | SHA-256 |
|---|---|
| Uploaded ZIP | `c620e9a849d396b3e70de257c5f9f2429b1e5339dda580f90c3d3cba261b2031` |
| `decisions_GPT.jsonl` | `6aeeae78857f188b7c445cabdffb26d86ee7d852f50ffb6b77c9e9d577decff4` |
| `display_GPT.jsonl` | `aa05a72ab308e8c3bda08aabcc4b0d4ae3546f5e7021d4f94a4fadad84b93b52` |
| `events_GPT.jsonl` | `3a05fc34aea7414de5cb3905a637ad486a9c6a3bef51a59f8878def7a631311d` |
| `fault_test_result_GPT.json` | `99c92d72cbbb7f5aef39fb56be6774afedfd0a70985263fbc2a0c87b96c7c04e` |
| `metadata_GPT.json` | `bbc866563c82c223ea70d9c7e59422af049082fa416f938398807ffc8c8072de` |
| `observations_GPT.jsonl` | `fe951b4b654f2aeee206464ab1fcd721f46a944c845f6eb6cdbdcfbc1643de99` |
| `routes_seed42_GPT.xml` | `86fe8dc18d40a084d0873638e951032d34a4fa525c24aab460ef4444d192eb15` |
| `summary_GPT.json` | `05f209539ca092984bb4ae9d4572ff7eed71d987ee395e9cb2e03c260b5721e9` |

Reported model SHA-256: `a45080e45965a7fc99146c473631407cf6d3d1c5ee624bdc623679350ffcfe20`.

Reported runner SHA-256: `dc1d547fe4241600ad5f1ba77a7ba3148755a8d9b37f1459b1818eceb89dd84c`.
