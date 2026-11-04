# Experimental handback trial

This package adds a separate, opt-in SUMO experiment for returning an affected
junction from fixed-time contingency (FTSC) to its policy. It does not change
`run_supervised_GPT.py` or the normal wrapper. These thresholds are provisional
and the trial is not a hardware-safety validation.

## Before running

1. Extract all files from the ZIP into the root of
   `HIL-Traffic-Supervisor-Design`, next to `run_supervised_GPT.py`. These are
   new `GPT`-named files; check before overwriting if a file with the same
   name already exists.
2. Use the same Python/SUMO environment and scenario used for the earlier
   supervisor trial. The integration test also needs the existing
   `supervisor/tests/helpers.py` file.
3. Open **Command Prompt** in the project root and activate `.venv`.

The runner checks fingerprints of the relevant controller, environment,
configuration and fixture files against the source snapshot used to create this
package (Windows/Unix line endings are normalised). If it reports a mismatch,
stop and share the listed current files for review. Do not edit the fingerprint
file merely to make the check pass.

The runner copies `2int_files` into a new run directory and runs SUMO from that
copy. It checks SUMO configuration and additional-file references for file
paths escaping the copy; such paths stop the run. The copy may take disk space.
The output directory must not exist already, even after a failed attempt.

## 1. Run the automated tests

```cmd
cd /d "C:\Users\sihle\Desktop\HIL-Traffic-Supervisor-Design"
.venv\Scripts\activate
python -m unittest -v test_handback_trial_GPT test_handback_integration_GPT
```

The integration test uses a simulated TraCI environment with the existing real
Fast Gate and wrapper. It covers a normal fault/recovery/handback/renewed-fault
sequence, an observation failure at release, and a SARL policy that never
selects the affected junction. It does **not** run SUMO.

## 2. Run the controlled SUMO fault demonstration

```cmd
python run_handback_trial_GPT.py --controller demo --fault-demo --enable-handback-trial --output-dir supervisor_logs\handback_fault_demo_seed42_GPT --max-cycles 16
```

`demo` is a deterministic scripted controller, not a trained RL model. The
first synthetic fault removes one `tl1` lane reading from the Fast Gate's
frozen snapshot at cycle 2 (simulation time 180 s with 90 s cycles). The
Input Validator must confirm `LANE_MISSING` and `tl1` must enter contingency.
After sufficient recovery and a full transition cycle, a fresh proposal is
checked and applied. When the new programme and a complete controlled cycle
are observed, the runner injects a second missing-reading fault and confirms a
new contingency cycle.

The signal programme check reads the active SUMO programme ID, phase durations
and states at the start of each cycle and after the first simulated step. It
checks these against the commanded split and the six-phase network structure.
This is a programme readback, not a measurement of a physical signal or proof
that every instant of every phase executed correctly.

If conditions are not met within 16 cycles, `fault_demo_complete` may be false.
For example, a continuing gate failure or missing measurements can delay
handback. A failed or incomplete run must not be described as a pass. Use a
new output directory to rerun. Check console errors and `run_error_GPT.txt`
when present.

## Results to share

Please send the console output plus these files from the new output folder:

- `summary_GPT.json`
- `handback_events_GPT.jsonl`
- `handback_observations_GPT.jsonl`
- `decisions_GPT.jsonl`
- `metadata_GPT.json`

If the run stops early, include `run_error_GPT.txt` if it exists. You do not
need to send the copied `sumo_work` folder. The event log contains the other
junction's mode and gate result at each injected fault; its behaviour still
depends on its own checks.

## Agreed experimental rules

- Recovery is per junction. A complete cycle with an actual fixed-time program
  counts toward a minimum five-cycle contingency stay. Three most recent full
  cycles must have valid observations and measurements, approved Fast Gate
  results (PASS or approved WARN), and programme readback. Any missing data,
  incomplete cycle or gate failure clears the clean evidence.
- For SARL, at least three *actual* selections of that junction in the last ten
  healthy complete contingency cycles are required. An adapter no-op for the
  unselected junction is not counted. This may leave a junction latched if the
  policy does not select it.
- If the Slow Monitor caused the latch, pending insertion backlog must also be
  under 100 at the end of three consecutive healthy complete cycles. This does
  not apply to a Fast Gate-only latch, but backlog data must still be available.
  If another gate fault occurs while already latched, the original slow reason
  and its threshold are retained.
- Once ready, the junction stays on FTSC for one transition cycle. At the next
  boundary, a fresh policy proposal must pass the existing Fast Gate. A veto
  or invalid policy observation keeps the junction in contingency. A new
  fault after handback causes a new five-cycle hold. The rolling network
  traffic history continues; the per-junction RL behaviour history starts
  afresh after the fixed-time/transition cycles.
- Fresh policy observations are reconstructed from TraCI with the existing
  normalization and actual split. Observation errors never become dummy
  all-zero policy inputs. For SARL, a missing observation for either junction
  invalidates its joint observation.

## Later: run the provisional Slow Monitor with a trained policy

After reviewing the controlled fault demonstration, this optional example
enables the previous provisional Slow Monitor rule with MARL DQN:

```cmd
python run_handback_trial_GPT.py --controller marl-dqn --model models\marl_dqn_ps_2int_260k.zip --enable-handback-trial --enable-slow-trial --output-dir supervisor_logs\handback_slow_marl_dqn_seed42_GPT
```

The existing slow trigger remains pending >=150 and growth >=30 over 270 s,
combined with the per-junction bound or alternating-split behaviour. The
post-handback conditions and trigger are research settings, not validated
limits. No automatic network-wide switching or mid-episode `reset_episode()`
is added. A shadow proposal based on the actual fixed-time split does not prove
that the policy will perform well when it regains control.
