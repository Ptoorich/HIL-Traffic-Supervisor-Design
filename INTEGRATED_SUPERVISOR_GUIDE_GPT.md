# Opt-in Slow Monitor and handback

The regular `run_supervised_GPT.py` runner now has separate, optional Slow
Monitor and automatic handback switches for the existing RL policies. With
neither switch, it continues to use the original controller and evaluation
loop. The original `supervisor/supervised_controller_GPT.py` is unchanged, so
the earlier stand-alone trial and its source-fingerprint check remain usable.

These features and their thresholds are experimental. SUMO results do not
validate an end-to-end HIL deadline or a physical traffic signal system.

## Switches

| Switch | Effect |
| --- | --- |
| `--enable-slow-monitor` | Enable the provisional per-junction traffic-and-behaviour trigger. |
| `--enable-auto-handback` | Enable the provisional recovery and Fast Gate-checked handback. |
| `--run-dir PATH` | Required with either switch. Must be a new directory outside `2int_files`. |
| `--max-cycles N` | Optional short test per seed; omit for a full episode. |

Either switch can be used independently. With Slow Monitor only, contingency
remains latched to the episode boundary. With handback only, a Fast Gate latch
can recover but no Slow Monitor intervention is enabled. Both switches use
validated fresh policy observations and active SUMO programme readback.

The opt-in path currently supports `--controller rl` with `--kind marl` or
`--kind sarl`. Static and fixed controllers continue to work on the legacy
path. `--gen-dir` and a custom `--out` apply to the legacy path; opt-in runs
use the isolated `--run-dir` instead and reject those options rather than
silently ignoring them.

## Check the installation

In Windows Command Prompt, from the project root:

```cmd
.venv\Scripts\activate
set ASTA_NET=2int_files\osm.net.xml.gz
python -m pytest supervisor\tests -q
```

This includes the existing supervisor tests and the new integration tests.
The latter use a simulated TraCI environment with the real Fast Gate; run a
SUMO simulation as well before drawing conclusions about the integrated path.

## Run a full-episode integration check

The following uses the previously tested MARL DQN model and the first standard
seed, 42. The output directory must not already exist:

```cmd
python run_supervised_GPT.py --controller rl --kind marl --algo dqn --model models\marl_dqn_ps_2int_260k.zip --name integrated_marl_dqn --seeds 1 --enable-slow-monitor --enable-auto-handback --run-dir supervisor_logs\integrated_marl_dqn_seed42_GPT
```

Use `--max-cycles 20` only for a short smoke run. The first of the standard
seeds is 42; increasing `--seeds` creates an independent folder per seed.

Opt-in runs copy `2int_files` into each seed's `sumo_work` folder and run SUMO
there. Explicit SUMO configuration and additional-file `file` references that
escape that copy are rejected. This copy can require disk space. The route
generated for each seed, model and route SHA-256 values are recorded. The
legacy path keeps its pre-existing output behaviour.

## Outputs and evidence

Each seed has a folder such as `integrated_marl_dqn_seed42_GPT\seed_42`:

- `metadata_GPT.json`: controller, seed, hashes, switches and provisional values.
- `decisions_GPT.jsonl`: Fast Gate decisions; handback requests and their results.
- `events_GPT.jsonl`: Slow Monitor, gate, transition and handback events.
- `observations_GPT.jsonl`: boundary snapshots, traffic windows, actual programme
  readback, policy actions and recovery evidence.
- `summary_GPT.json`: cycle counts and intervention/handback counts **per junction**.
- `run_error_GPT.txt`: traceback if an opt-in seed fails after its folder is created.

A contingency-set event is recorded before `env.step()`. The separate
`contingency_program_observed` event records the actual active SUMO programme
after the first simulated step. A completed handback requires a complete cycle
and programme readback; a partial final cycle is recorded as unverified.

To share the JSON logs for the example run:

```cmd
powershell -NoProfile -Command "Compress-Archive -Path 'supervisor_logs\integrated_marl_dqn_seed42_GPT\seed_42\*.json*' -DestinationPath 'supervisor_logs\integrated_marl_dqn_results_GPT.zip'"
```

Send the ZIP and console output. If the run fails, also send
`run_error_GPT.txt` when it exists. The copied `sumo_work` folder is not needed
for the initial review.

## Current experimental decisions

- Slow trigger: a three-cycle window with pending insertion backlog >=150 and
  growth >=30, combined with either five outward bound requests or six
  alternating applied splits at that junction. Unknown traffic values do not
  become zero and do not match the combined trigger.
- Handback: at least five complete FTSC cycles and three clean cycles; for SARL,
  at least three actual selections of the affected junction in the most recent
  ten clean complete cycles. An adapter no-op does not count as a selection.
- If the Slow Monitor caused the latch, backlog must be below 100 at three
  consecutive healthy cycle ends. This threshold does not apply to a Fast
  Gate-only latch, but measurements must be available.
- A further FTSC transition cycle is held. At the next boundary the integrated
  controller asks the Fast Gate to check the fresh policy proposal before any
  release. A veto, invalid observation, missing reading, or timeout keeps the
  junction in contingency. A new fault after a release starts a new hold.
- The rolling network traffic window remains continuous. RL behaviour history
  is limited to the current contiguous RL-controlled cycles after handback.
  Switching and recovery remain per junction; the deferred network-wide
  switching question is unchanged.

The policy sees the actual FTSC split while in shadow. A passing shadow
proposal does not prove good future traffic performance. The SARL case where an
unselected junction can remain latched is retained as an initial design
observation, without adding a fallback.

The experimental loop temporarily wraps `traci.simulationStep` only during an
environment step to collect per-second counts, and restores it afterward. This
and the observation/programme-readback overhead have not been validated against
an end-to-end hardware deadline.
