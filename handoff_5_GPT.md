# ASTA Traffic Supervisor — Handoff 5 (GPT)

**Date:** 3 November 2026  
**Project:** Adaptive Split-Time Algorithm (ASTA) traffic signal supervisor and eventual hardware-in-the-loop (HIL) demonstration  
**Purpose:** Provide the next workspace with project context, approved decisions, implementation status, local verification and next steps.

This handoff supersedes handoff 4 where it reports a newer implementation or result. Earlier design decisions remain in force unless amended below. Distinguish confirmed observations, approved decisions and open questions. The GitHub repository is the user's new working copy: inspect its current contents before editing; a previous handoff is not authoritative about later commits.

## 1. Working agreement

- Discuss proposed designs and obtain Siphetfo's agreement before implementing new architecture. Explain the reasoning and trade-offs, especially safety and handback decisions.
- Deliver code as files rather than pasting large implementations into the conversation. Maintain a running decision log in the handoff rather than a separate decision-log file.
- Use `GPT` in new AI-generated deliverable filenames and updated implementation filenames. Some unchanged-name compatibility shims are required so existing Python imports continue to work; retain them unless a change is explicitly agreed and tested.
- The user works in VS Code on Windows, typically Command Prompt (not PowerShell), with Python 3.14 and SUMO 1.27.1. Provide executable commands for that environment.
- Previous assistant cannot see or alter the user's local Windows files without an appropriate connection. Do not claim a GitHub change was made without evidence. The user reported that their files showed up on GitHub after pushing.

## 2. Project objective and scope

The supervisor sits between existing RL traffic signal controllers and the two-intersection SUMO environment. It checks each proposed signal split and intervenes when a safety or health check fails, switching the affected junction to a fixed-time signal-control (FTSC) contingency. Eventually Arduino-controlled LEDs will show commanded phases.

Current scope is the 2-intersection network (`tl1`, `tl2`), each with phases A/B; SARL and MARL controllers are in scope. Four- and six-intersection networks, multi-phase junctions and Arduino integration are deferred. Supervisory changes must not be represented as validated on real signal hardware.

## 3. Current repository and local working copy

- GitHub: https://github.com/Ptoorich/HIL-Traffic-Supervisor-Design
- User's Windows working folder: `C:\Users\sihle\Desktop\HIL-Traffic-Supervisor-Design`.
- Previous project, retained as backup: `C:\Users\sihle\Desktop\WITS\SM8\MECN4035A Investigational Project\RL_TSC_Comparison\agents`.
- User created `.venv` in the new repository with Python 3.14.0; installed project requirements plus pytest. `sumo.exe` and `netgenerate.exe` were on PATH. `.venv`, caches and `supervisor_logs/` were placed in `.gitignore`.
- The user reported a successful push and confirmed files were visible on GitHub. No exact commit SHA or final post-push status was provided; inspect the repository before relying on its current contents.
- User's prior local inventory included `supervisor/`, `sumo_env_marl_ps.py`, `tod_eval_common.py`, `tod_plan.py`, `eval_rl_2int.py`, `baseline_tod_2int.py`, `run_calibration.py`, `dump_conflicts_GPT.py`, `calibrate_thresholds_GPT.py`, `probe_jam_occupancy.py`, `run_supervised_GPT.py`, `requirements.txt`, and `SUPERVISED_CONTROLLER_GUIDE_GPT.md`. Confirm actual tracked files rather than assuming all are present.
- Four 2-intersection models were copied from old `agents/models/` into `models/`. Six SARL variants from old `agents/` were copied into `models/agents_original/` to preserve provenance. **Which SARL variants correspond to past experiments is not yet verified**; do not delete variants or assume matching names/sizes mean identical files.
- Required `2int_files/2int.sumocfg` references `osm.net.xml.gz`, `routes.xml`, `osm.poly.xml.gz`, `output.add.xml`, `Jorissen_model_two_intersection.dat.add.xml` and, for the GUI, `osm.view.xml`. Confirm additional XMLs have no further missing references. Simulation generates outputs and routes; inspect and avoid committing transient outputs by accident.
- Project packages delivered earlier: `supervisor_package_GPT.zip` (P5/P6) and `supervisor_controller_package_GPT.zip` (includes wrapper). The repo may have evolved beyond either package.

## 4. Fixed operating parameters and contingency

| Parameter | Value |
|---|---|
| Cycle | 90 s |
| Phases | Two phases A and B per junction |
| Yellow / all-red | 3 s each per phase; 12 s total lost time |
| RL A-green bounds | 15–63 s |
| RL action index | 0 = −5 s; 1 = no change; 2 = +5 s on A |
| B-green | 90 − A − 12 s |
| Safe Gate | FAIL below 8.0 s; WARN from 8.0 to below 11.0 s; PASS at least 11.0 s, for either phase |
| FTSC contingency, tl1 | A = 31 s; B = 47 s |
| FTSC contingency, tl2 | A = 59 s; B = 19 s |
| Vehicle length / minimum gap | 5.0 m / 2.5 m |
| Provisional whole-supervisor budget | 3 s; arbitrary and subject to testing |
| Fast Gate stage / resolver target | 2.0 s / 0.1 s |
| Slow Monitor proposed window | 270 s = three cycles |

Do not use 39/39 as contingency. The prior experiments found Webster static performs better. A switch should occur at a cycle boundary. The Fast Gate evaluates an RL action's requested split before environment clipping (e.g. 15 − 5 gives 10 s and WARN even if env clips to 15).

## 5. Decision log and current architecture

### Previously agreed decisions

- One frozen snapshot per decision, taken on the main thread; checks run in parallel on that snapshot. Fast Gate runs for both junctions every cycle, including a no-op junction. Check timeout, exception or invalid/missing result fails safe.
- Fast Gate results are PASS, WARN or FAIL; FAIL vetoes the junction, WARN permits the action. A junction's veto affects that junction, not automatically both.
- Contingency is the Webster static plan and remains latched pending agreed handback. **Initial implementation approved this session:** no automatic handback during an episode; an explicit `reset_episode()` clears the latch only at an episode boundary.
- Planned future handback: shadow proposals plus consecutive clean Fast Gate and degeneracy checks; performance comparison restarts after handback and a grace period. Details, especially shadow proposed split relative to the actual FTSC split, are not finalized.
- Slow Monitor remains network-level; its interaction with junction latches needs design. Window 270 s. A tentative threshold of three teleports/window is arbitrary and not finally justified; whether counting applies only when ASTA controls remains open. Fixed queue/throughput thresholds previously considered were inadequate because upstream insertion backlog can be severe while spatial queues appear modest.
- HIL remains optional and separate. LEDs display commanded phases. A previous design says an LED-stage budget overrun should be logged while retaining the intended state, not alter SUMO; integration order and the full timing boundary still require resolution.
- P1–P8 were approved; the revised P5/P6 definitions below supersede the earlier percentile classifier and strict symmetry rejection.

### Fast Gate checks

1. Input Validator checks sensor data, lane completeness and plausibility; speed WARN/FAIL thresholds are 1.2×/1.5× lane limit, count `capacity+1` is WARN and greater is FAIL. Occupancy is treated as a 0–1 fraction, confirmed locally by the user's SUMO 1.27.1 probe. Empty lanes can report lane speed limit, so speed must be interpreted with vehicle count.
2. Physical Traffic Classifier: per non-excluded lane, queue fill = halting / empirical capacity; jam occupancy = min(1, capacity × 5 m / lane length). A lane is jammed if queue fill ≥ 0.90, observed occupancy ≥ 0.90 × jam occupancy, and mean speed < 0.5 m/s. Both junctions have five active lanes. With zero jammed lanes: FREE if unweighted mean fill ≤ 0.10, otherwise LIGHT. One or two jammed lanes: HEAVY/WARN. Three or more: GRIDLOCK/FAIL. HEAVY is descriptive, not proof of malfunction. Excluded lanes `669676574#0_0` and `-669676574#8_0` remain validator-checked against theoretical capacity but are excluded from classification. The four-vehicle lane has coarse queue-fill resolution.
3. Safe Gate checks both requested phase durations; use unclipped RL requested splits.
4. Conflict Monitor reads a network-derived fixture. Protected G+G on foes = FAIL; G+g is allowed; g+g = WARN; invalid lengths and missing/invalid matrix fail safely. Foe bitstrings read right-to-left (rightmost is link 0). Asymmetric directed relationships are recorded and conservatively treated as an undirected conflict; malformed matrices and self-conflicts are rejected.

On the supplied actual network, tl1 has 8 links and 13 foe pairs; tl2 has 27 links and 121 foe pairs including asymmetric directed pairs (8,22) and (8,23). All listed network phases have zero protected G+G conflicts; tl2 phase 0 has 16 permissive g+g pairs. `supervisor/conflicts_2int_GPT.json` is the current fixture name. `py dump_conflicts_GPT.py` generated it successfully on the user's actual network.

## 6. SupervisedController — implemented initial version

Implemented in `supervisor/supervised_controller_GPT.py`, with `run_supervised_GPT.py` and wrapper tests `supervisor/tests/test_supervised_controller_GPT.py`. It fits the existing evaluation interface: `controller.act(env, obs, t)` followed by `env.step(actions)` in `tod_eval_common.py`.

Per cycle:
1. Save both existing splits before calling the inner controller. `StaticController.act()` calls `env.set_splits()` itself, so the resulting base split is treated as a proposal; the wrapper restores the previous split before the gate.
2. Build per-junction proposals and no-op for a missing agent action; invalid actions are rejected. For a static controller the env's `set_splits()` may already clip its proposed base split, so the original unclipped absolute request is not observable. RL action deltas are assessed before subsequent environment clipping.
3. Take one main-thread snapshot, run checks under the configured stage budget, and use strict per-junction veto decisions. Exceptions from the inner controller affect both; a shared snapshot failure affects both. Each failing junction latches in contingency; set its Webster split and return a no-op action. The other junction may continue.
4. Log a JSONL entry before `env.step()`, including proposals, gate results, per-junction modes and timings. A log write error stops the call before `env.step()`. A log entry alone does not prove the simulation step completed.

The wrapper continues calling the inner controller while a junction is latched, but discards its action for that junction. **This is not a finalized shadow-handback mechanism.** The supplied runner explicitly resets the latch per new episode; direct users of the wrapper must call `reset_episode()` at an episode boundary. Never use it as an unapproved mid-episode handback.

The wrapper checks config/env lane mappings and fixture traffic-light IDs at startup. The main-thread TraCI snapshot cannot be preempted if it blocks; overrun can only be detected after it returns. Resolver and 3 s flags currently report observed timings; no enforced whole-process deadline including logging and LED exists. The logged `supervisor_ms_before_log` starts after policy prediction and excludes the log write, `env.step()` and any LED stage. Treat those as explicit limitations, not as proof that the entire final HIL budget is met.

## 7. Verification completed

### P5/P6 local verification

The user ran `py dump_conflicts_GPT.py` on `2int_files\osm.net.xml.gz` and got the exact counts and asymmetric pairs described above, plus no protected G+G conflicts. A first attempt ran an old parser and failed; replacing the `supervisor` folder corrected it. The user then ran `py -m pytest supervisor\tests -q` and received **81 passed**. An unrelated full-project pytest collection failed because `test_baseline_hil.py` imports a missing `extract_lanes`; it is not evidence of a supervisor test failure and its status remains separate.

### New repository and wrapper verification

The user created `.venv`, installed the copied requirements and pytest, and ran:

```cmd
set ASTA_NET=2int_files\osm.net.xml.gz
python -m pytest supervisor\tests -q
```

The user's pasted pytest output was **88 passed in 4.55 s** with no skips shown. This includes the live SUMO test, which uses a generated toy junction, and the new wrapper tests. The actual-network regression runs when `ASTA_NET` is set in the active shell; the user was instructed to set it, but the pasted final output did not separately show that command. The development workspace earlier got 87 passed, 1 skipped because SUMO was not available.

The user then ran one full *requested* static-controller evaluation with `python run_supervised_GPT.py --controller static --seeds 1`. Console output: seed 42, 212 cycles, 9,031 trips, final modes `{'tl1': 'ASTA', 'tl2': 'ASTA'}`; SUMO stopped at **19,001 s**, 9,031 inserted, zero running/waiting, average time loss 30.82 s and departure delay 47.64 s. The mode label `ASTA` in this run means the wrapped **static** controller remained in control; it does not prove an RL model ran.

The attached user-provided `decisions_GPT.jsonl` contained 212 cycle records for `supervised_static`, seed/episode 42, times 0 through 18,990 s. There were no vetoes or contingency entries. tl1: 211 APPROVE and 1 APPROVE_WARN; classifier labels 174 FREE, 37 LIGHT, 1 HEAVY. tl2: 212 APPROVE_WARN; classifier 53 FREE, 43 LIGHT, 116 HEAVY; Conflict Monitor `PERMISSIVE_CONFLICT` WARN on all 212 cycles. This matches the known g+g situation. Input Validator and Safe Gate passed on both junctions throughout. No stage timeout or observed timing budget flags. Maximum `supervisor_ms_before_log` was 94.712 ms, with the scope limitations described above. No real intervention occurred in this episode.

## 8. Open issue: episode ended at 19,001 s

The configured timeline is 900 s warm-up + five 3,600 s periods + 900 s cool-down = **19,800 s**. The episode reported SUMO ended at 19,001 s. The env code includes an early termination branch when `traci.simulation.getMinExpectedNumber() == 0`, which sets `current_step` to `sim_steps`; this is a **plausible hypothesis**, not proven from the decision log. The SUMO console indicated zero running/waiting at termination. Investigate the exit condition, generated route departure times, demand periods and whether an early empty network may occur while more departures are still expected. Determine whether the cool-down was merely cut short after all vehicles exited or whether a period/vehicle was missed. Compare baseline and supervised runs under identical generated routes and seeds before deciding whether behavior should change. Discuss implications and get design agreement before modifying simulation termination.

## 9. Other open work and recommended next steps

1. Inspect the current GitHub working tree and user environment. The repo's actual current state wins over this handoff. Resolve the 19,001 s early-end question and the separate `test_baseline_hil.py` error only if relevant to the user's chosen scope.
2. Test the supervised wrapper with the user's trained two-intersection RL models, first confirming which model file is intended and keeping a distinct log per run. Verify SARL/MARL action conventions and compare on matching seeds, routes and metrics; do not label the static run as RL validation.
3. Design an explicit fault-injection/integration experiment that demonstrates one-junction veto, Webster split application and persistent latching while the other junction continues. Existing unit tests cover these behaviors; the real 2int static run did not trigger them. Preserve the original network and avoid presenting synthetic faults as naturally observed behavior.
4. Discuss and design automatic handback (shadow reference split, consecutive clean counts, degenerate actions, cooldown/hysteresis and per-junction/network interactions) before implementation.
5. Revisit Slow Monitor metrics: pending insertion backlog, worst-lane queue/fill and throughput relative to demand; frozen sensors must distinguish legitimate inactivity. Decide teleports and meaningful comparison baselines.
6. Resolve end-to-end timing boundary, LED update order relative to the potentially expensive `env.step()`, serial protocol and timeouts before HIL work. User's hardware concept: Arduino, breadboard, four signal heads, 12 red/yellow/green LEDs and about six spares; mapping and fail-safe display behavior not yet designed.
7. Keep user informed of assumptions, distinguish PASS/WARN/VETO, and keep all design decisions in the next handoff.

## 10. Useful local commands

From the new Windows repository in Command Prompt:

```cmd
.venv\Scripts\activate
set ASTA_NET=2int_files\osm.net.xml.gz
python -m pytest supervisor\tests -q
python dump_conflicts_GPT.py
python run_supervised_GPT.py --controller static --seeds 1
```

The runner's default log `supervisor_logs\decisions_GPT.jsonl` is append-only; use `--out` with a unique GPT-labelled path to distinguish new runs. A fresh static run can take a full simulation episode; no timing or performance claims transfer automatically to other controllers or machines. Do not run destructive Git commands or overwrite user files without checking status. The `.venv` and logs are not expected to be committed. Respect the user's decision on whether logs should be archived.

**End of handoff 5 — GPT.**
