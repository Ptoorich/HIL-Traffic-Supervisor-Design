# Initial supervised controller — guide and verification

The package includes the P5/P6 changes and adds `supervisor/supervised_controller_GPT.py`, `run_supervised_GPT.py`, and supervisor wrapper tests. It supports the existing `controller.act(env, obs, t)` / `env.step(actions)` interface.

## Behavior

Each cycle the wrapper saves the prior splits, calls the underlying controller, records any change it made through `env.set_splits`, restores the prior splits, builds both proposals, collects a single snapshot and runs the Fast Gate. A FAIL, missing or invalid result, invalid action, sensor failure, or observed Fast Gate stage overrun causes the affected junction (both on shared snapshot failure) to latch in contingency. The command uses Webster 31/59 seconds and no-op actions for latched junctions. The other junction continues independently. A controller exception latches both. The wrapper catches a missing agent action as a no-op; an unknown agent key is an error. The default no-op action is 1.

The latch is deliberately held for the remainder of the episode. `reset_episode(episode_id)` clears latches explicitly at the start of a new episode; never call it mid-episode as a handback mechanism. Automatic handback, shadow mode, Slow Monitor, and Arduino HIL remain unimplemented. The wrapper continues to call the inner controller while latched, but discards actions for latched junctions; this is not an approved shadow-handback evaluation.

A static controller's `set_splits` side effects become the proposed base split. Since the environment itself clips `set_splits`, the wrapper can only observe that already-clipped base value. RL action deltas are checked before environment clipping. Startup validates lane mappings, signal timing and traffic-light IDs against the fixture. Preserve the provided fixture and configuration.

`supervisor_logs/decisions_GPT.jsonl` is appended to by the runner. Each cycle records both decisions, check results, modes, proposals, actions, and timing. A log write error stops execution before `env.step`. Do not assume a record proves `env.step` completed. The log uses the configured 2-second Fast Gate deadline; since the snapshot runs on the main thread, a blocking TraCI call cannot be interrupted and timeout is detected after it returns. The resolver and three-second flags are observed timings, not hard deadlines: logging and the future LED stage are excluded, and resolver overruns are recorded rather than triggering a new veto. Full end-to-end budget enforcement awaits the agreed LED/timing architecture.

## Install and local checks

Back up your current `supervisor` folder. Replace the whole folder with the one in the ZIP; copy the top-level scripts alongside your existing `tod_eval_common.py`, `tod_plan.py`, `sumo_env_marl_ps.py` and `2int_files`. This avoids collecting old and renamed test files together. In Windows Command Prompt with the existing virtual environment active:

```
set ASTA_NET=2int_files\osm.net.xml.gz
py -m pytest supervisor\tests -q
py run_supervised_GPT.py --controller static --seeds 1
```

The final command runs one complete 19,800-second simulated episode, not one cycle. It generates a route file in the configured generated-routes directory, and appends to the decision log. It needs SUMO, TraCI, route files, and the existing project. To start with a separate log, pass `--out supervisor_logs/my_run_GPT.jsonl`. `--controller fixed` and RL options are documented by `py run_supervised_GPT.py --help`.

If the local tests or run fail, provide the full error, not only the last line. Review decisions and mode transitions in the JSONL log; the output alone is not a benchmark comparison. Live tests with this wrapper and full 2int episodes were not run in the development workspace.

## Verification

In the development workspace: 87 passed, 1 skipped (SUMO not installed), including the real supplied network, fixture round-trip, prior Fast Gate tests, and new tests for per-junction veto/latching, reset, no-op junctions, unclipped requested split, static-controller mutations, sensor failure, invalid action and fixture mapping. The user previously ran the P5/P6 suite with 81 passed locally; that result predates this wrapper and does not validate it.
