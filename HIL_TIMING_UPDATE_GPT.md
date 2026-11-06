# HIL end-to-end timing update

Prepared 6 November 2026 (Africa/Johannesburg) from the user's uploaded `hil_timing_source_01_GPT.zip`.

The update measures each HIL decision boundary continuously through two matching output acknowledgements. It uses the serial transport's monotonic clock, records policy-call time, and reports the actual pacing sleep separately. It adds observations; it does not introduce a new controller switching rule.

## Install on the user's Windows project

1. Stop the Python/SUMO run and close Arduino Serial Monitor.
2. Extract this package into a temporary folder. Copy `install_hil_timing_GPT.py` into `C:\Users\sihle\Desktop\HIL-Traffic-Supervisor-Design`.
3. From that project root with `.venv` active, run:

```cmd
python install_hil_timing_GPT.py
```

The installer is self-contained. It checks all four production file versions and syntax before changing any code. It accepts CRLF or LF source, refuses an unexpected local version, and saves the replaced files under `supervisor_logs\hil_timing_backups_GPT\<timestamp>`. If a replacement fails, it attempts to restore already-written files. Running the installer again is harmless when the update is already present.

`files/` contains the complete source for review. `changes_GPT.patch` contains the production diff, and `manifest_GPT.json` records before/after normalized SHA-256 identities. The installer installs four production files; the two new tests in `files/supervisor/tests/` are supplied separately for optional copying to the same paths in the project. The original source ZIP and earlier evidence archives have not been changed.

No Arduino firmware upload or new wiring is needed. Keep the working phase-display sketch on COM5. LCD work remains paused.

## Collect the new timing evidence

Use a fresh run directory. This is a three-cycle timing run, approximately 4.5 minutes plus overhead. It is separate from the already completed ten-cycle fault-and-handback test.

```cmd
python run_supervised_GPT.py --controller rl --kind marl --algo ppo --model models\marl_ppo_ps_2int_260k.zip --name hil_timing --seeds 1 --enable-slow-monitor --enable-auto-handback --hil-port COM5 --hil-speed 1 --gui --max-cycles 3 --run-dir supervisor_logs\hil_timing_01_GPT
```

Keep GUI playback running with delay zero. Then inspect:

```cmd
type supervisor_logs\hil_timing_01_GPT\seed_42\timing_summary_GPT.json
```

The usual decisions, observations, display and summary logs are retained. Two new files are created beside them:

- `timing_GPT.jsonl`: one record for each attempted decision boundary, including exact timestamps and correlated ACK session/sequence IDs.
- `timing_summary_GPT.json`: sample counts, average/median/P95/P99/minimum/maximum for each interval, completeness and budget comparison.

Both the normal HIL runner and the dedicated fault runner use the instrumented shared loop automatically. Non-HIL execution does not create these files. The dedicated runner itself is unchanged.

## Endpoints and scope

| Field | Meaning |
|---|---|
| `policy_call_ms` | Full `StrictPolicy.act` call, including live observation validation and model prediction(s); not model-only inference |
| `post_policy_to_boundary_ack_ms` | Policy return/exception through gate, resolver, decision logging, recovery processing, SUMO action/programme application, readback and validated output ACK |
| `policy_to_boundary_ack_ms` | Same boundary endpoint, beginning before the policy call |
| `boundary_to_boundary_ack_ms` | Same endpoint, also including boundary evidence collection and recovery preparation before the policy call |
| `policy_to_first_step_ack_ms` | Policy call start through the next output ACK after one SUMO step, including deliberate pacing |
| `boundary_to_first_step_ack_ms` | Broadest recorded interval, from boundary processing start through that first-step ACK |
| `pacing_requested_ms` / `pacing_actual_ms` | Requested delay and measured time spent in the first `sleep` call |
| `policy_to_first_step_ack_excluding_pacing_ms` | Derived value: measured inclusive total minus measured pacing sleep; other processing remains included |

The boundary ACK confirms readback immediately after `env.step` applies the selected programmes, before `simulationStep` advances time. The first-step ACK follows the first actual simulation step. The two consecutive ACKs have the same session and distinct sequence numbers; their simulation times must be the decision time and decision time plus one second.

`ack_received_at` is captured after a CRC-valid response passes session, sequence, kind and mask checks, and before ACK-log writing. State samples now include `ack_sequence` and `ack_received_at`. The existing `observation_to_ack_ms` field retains its prior scope through the returning transport call, including its log write; it should not be confused with the new precise receipt timestamp.

The broader timers include work and log writes that occur before their chosen ACK endpoint. Writing a record after its endpoint is necessarily excluded. Model loading, process startup, environment reset, the other 89 seconds of a cycle, later cycle-end logging, normal STOP, GUI rendering and optical LED response are outside these decision-response intervals.

## Interpreting status

`measurement_status=COMPLETE` means all started boundary measurements have valid matching endpoints. A missing ACK, interrupted run, zero-step boundary, mismatched sequence/session or out-of-order marker cannot be counted as a completed timing observation. Incomplete/invalid records remain in the JSONL and are excluded from the statistics. An exception during the timed run makes the summary incomplete even if earlier boundaries finished successfully.

`budget_ms` comes from the actual loaded `cfg.timing.total_budget_s` (currently 3,000 ms). Four named endpoint scopes are compared separately. `budget_status=WITHIN_OBSERVED_BUDGET` describes the recorded complete sample; `EXCEEDED` counts observed exceedances; `INCOMPLETE` prevents a partial sample from being reported as a complete success. These are logging results, not new contingency triggers or changes to the hardware watchdog.

P95/P99 use linear interpolation at `(n-1)*p`. A three-cycle run has only three boundary observations. Treat its maximum as an observed maximum, not a worst-case bound or a statistical reliability guarantee. GPIO ACK timestamps do not establish actual LED illumination or optical latency.

## Changes and validation

Production changes are confined to:

- `supervisor/hil_serial_GPT.py`: return a validated ACK receipt timestamp before logging.
- `supervisor/hil_display_GPT.py`: share the transport clock by default, retain the latest sample receipt and measure the existing pacing sleep.
- `supervisor/integrated_runner_GPT.py`: attach boundary/policy timers and capture both output endpoints.
- `supervisor/hil_timing_GPT.py`: new telemetry collector and summary writer.

Traffic phase mapping, serial frames/checksums, ACK deadline/retry policy, watchdog commands, pre-step pacing target, gate checks, contingency splits, recovery rules and trained models are preserved. The old source fingerprints all matched their recorded values; no expected fingerprint was edited.

Validation in this environment used Python 3.12 and SUMO 1.28; the user's Windows environment reported Python 3.14 and SUMO 1.27.1. The uploaded existing test suite plus nine new timing/integration tests yielded 129 passed, one environment-dependent skip and six passing subtests. The skipped actual-network test then passed separately with `ASTA_NET` set, giving 130 distinct passing tests in total.

The new tests cover ACK timestamps preceding synchronous log writes, identical retries, exact continuous intervals, explicit sleep accounting, invalid receipt rejection, interruption and failed ACK completeness, zero-step termination, diagnostic budget exceedance and control parity. A ten-cycle loop with the actual host serial code and simulated hardware retained the same observations and events as the untimed path.

A live SUMO two-junction run also passed all 12 existing fault/handback checks and produced 10 complete boundary timing records with matching ACKs. That test used a deterministic no-change policy, virtual serial responses and a virtual clock. It validates software integration, not PPO inference duration, physical USB latency, AVR execution or optical response. No new physical COM5 run has been performed by the assistant.

## Retained checkpoint

The user's earlier physical fault/handback run remains complete: TL1 fallback at t=90, FTSC transition at 540, ASTA release at 630 and verified handback at 720; TL2 stayed ASTA. Its 913 ACKs all succeeded on the first attempt. Its measured supervisor subtotal, Fast Gate, resolver and serial timings were analysed in `HIL_TIMING_ANALYSIS_GPT.md`. That archive predates these new continuous timers and cannot supply their missing measurements retrospectively.

The next user result needed is the new `timing_summary_GPT.json`, preferably accompanied by the corresponding `timing_GPT.jsonl` and `display_GPT.jsonl`. Do not repeat the old functional test merely because an earlier handover omitted the user's successful result.
