# ASTA Traffic Supervisor — Handoff 6 (GPT)

**Date:** 4 November 2026 (Africa/Johannesburg)  
**Project:** Adaptive Split-Time Algorithm (ASTA) traffic signal supervisor and eventual hardware-in-the-loop (HIL) demonstration  
**Purpose:** Resume the project in a new conversation with the latest design decisions, experiment evidence, implementation status and outstanding work.

This document carries forward Handoff 5 and updates it with the work since then. Where a statement below is newer, it supersedes Handoff 5. **A handoff is not a substitute for inspecting the current repository or user's Windows working copy before making changes.** Distinguish decisions, actual implementation, experiment results and open questions. Values from a single simulation seed or synthetic fault are not proof of real-world safety or general performance.

## 1. User's priorities and working agreement

- Siphetfo's primary goal is to get a working supervisor running, then iterate the design. Avoid getting stuck on exhaustive performance matching when the user has explicitly deferred it.
- Discuss significant new architecture, safety, contingency and handback decisions with the user and obtain agreement before implementing them. The Slow Monitor per-junction trial was explicitly approved; network-wide switching was not.
- Explain trade-offs clearly; distinguish PASS, WARN, FAIL/VETO and latched modes. Provide code in files rather than pasting long implementations. Use `GPT` in new AI-generated deliverable and updated implementation filenames; preserve compatibility shims unless otherwise agreed and tested.
- Keep the design decision log **inside the handoff**, not a separate decision-log file.
- User works in VS Code on Windows, generally Command Prompt, in `C:\Users\sihle\Desktop\HIL-Traffic-Supervisor-Design`, with `.venv`, Python 3.14.0 and SUMO 1.27.1. Give runnable CMD commands.
- In this conversation the assistant could not directly access the user's Windows terminal or the GitHub repository; work was done by inspecting an uploaded source snapshot, delivering scripts, and having the user run them. Do not imply current direct access in a new environment without verifying it.
- Do not describe an observation-only monitor or an experimental runner as integrated production functionality. No physical signal hardware safety has been validated.

## 2. Files and how to start a new chat

**Minimum:** Attach **`handoff_6_GPT.md`** to the new chat and say you want to continue the traffic supervisor project. This file is designed to carry the context and decision log.

**For code/design work, if the new assistant cannot access the current repository:** also attach:

1. **`source_review_GPT.zip`** — Python source, supervisor configuration/fixture and `2int.sumocfg` snapshot provided in this chat. This is a snapshot associated with the user's reported commit `8c3fd3e34ffb0661b7ccdf50d62ef301b10c39f4` at the beginning of this work; it is **not proof of current repository contents** and does not include the new experiment runners, model files or SUMO network. Library ID `libfile_c8c8ef5ea67c819192d92cf7043c1dfd`.
2. **`run_slow_trial_GPT.py`** — current opt-in trial runner. Library ID `libfile_393528dbf8f481919b7fdcef8614e137`.
3. **`slow_trial_results_GPT.zip`** — the two intervention-trial metadata files, observation logs, event logs and Fast Gate decision logs. Library ID `libfile_74447582cdd88191bf6b9acc68801f57`.

**Optional, when needed:** `run_slow_observer_sarl_GPT.py` (observation-only runner supporting static/MARL/SARL; Library ID `libfile_38def8d68b048191969c50e81e5a5fa1`), `run_slow_observer_GPT.py` (earlier static/MARL observer, `libfile_24f83c599be48191812545e8e12b3f48`), `fault_experiment_GPT.py` (Library ID `libfile_eb9b6593207881918a23cf11ad017248`), and `handoff_5_GPT.md` (`libfile_d4e460b7b7888191a73462f5b9a14cdb`) for fuller prior context.

The user's Windows project has the SUMO network, route inputs and model archives. A source-only ZIP cannot run the real simulations alone. If a new workspace must execute SUMO itself rather than guiding the user in VS Code, it will need the applicable model, referenced SUMO network and configuration dependencies and SUMO/Python environment; do not ask for all of these merely to discuss a design.

A code snapshot may be refreshed from the user's actual workspace when implementation starts; avoid overwriting local files or generated results. The user can attach files directly or use available connected tooling, if any.

## 3. Project goal, scope and repository

The supervisor sits between existing RL traffic signal controllers and the two-intersection SUMO environment. It validates signal-split proposals and can switch an affected junction to fixed-time signal-control (FTSC) contingency. The eventual HIL demonstration uses Arduino-controlled LEDs to display commanded phases; serial protocol, timing boundary, hardware mapping and fail-safe display behavior are not yet finalized.

Scope: the two-intersection network (`tl1`, `tl2`), two phases A/B per junction, SARL and MARL. Four/six intersections, multi-phase expansion and Arduino integration are deferred. Repository reported by user: `https://github.com/Ptoorich/HIL-Traffic-Supervisor-Design`. Last explicitly reported `git rev-parse HEAD` was `8c3fd3e34ffb0661b7ccdf50d62ef301b10c39f4`. This is not a promise of current HEAD. At that time untracked entries included `2int_files/edgeData.xml`, `2int_files/generated_tod_s0.37/`, `2int_files/stats.xml`, `handoff_5_GPT.md`; subsequent scripts and ZIPs may also be untracked. Ask for fresh status if needed. `.venv`, caches and `supervisor_logs/` were ignored earlier. Do not commit generated SUMO outputs by accident; simulations can overwrite standard outputs including `2int_files\stats.xml`.

## 4. Fixed control and safety parameters

| Parameter | Agreed value |
|---|---|
| Cycle | 90 s |
| Signal phases | A and B per junction |
| Intergreens | Each phase: 3 s yellow + 3 s all-red; total lost time 12 s |
| RL A-green bounds | 15–63 s |
| RL action index | 0 = −5 s; 1 = no change; 2 = +5 s on A |
| B-green | 90 − A − 12 s |
| Safe Gate | FAIL below 8.0 s; WARN 8.0 to <11.0 s; PASS ≥11.0 s, for either phase |
| FTSC contingency | `tl1`: A=31 s, B=47 s; `tl2`: A=59 s, B=19 s |
| Main Fast Gate stage target | 2.0 s; resolver target 0.1 s |
| Provisional total supervisor budget | 3 s, not finalized or validated |
| Slow Monitor traffic window | 270 s = three cycles |

Do **not** use 39/39 as the contingency. Gate evaluates the RL-requested split **before** environment clipping; e.g. at A=15, action −5 requests A=10 and produces a WARN while the environment clips back to 15. A switch takes place at a cycle boundary.

## 5. Design decision log

### 5.1 Existing Fast Gate and wrapper — approved and implemented

- One main-thread TraCI snapshot per decision, then checks run in parallel on that frozen data. Fast Gate runs for both junctions each cycle, including a no-op action. Timeouts, exceptions, malformed/missing gate results fail safe.
- Four checks: Input Validator, physical Traffic Classifier, Safe Gate and network-derived Conflict Monitor. PASS/WARN/FAIL: WARN permits an action; FAIL vetoes the affected junction. A local veto does not automatically switch the other junction.
- Input validation: occupancy normalized to fraction 0–1 (confirmed with user's SUMO 1.27.1 probe); speed WARN/FAIL at 1.2×/1.5× speed limit; count capacity+1 WARN, greater FAIL; interpret empty-lane speed together with count. Both junctions have five non-excluded classification lanes. `669676574#0_0` and `-669676574#8_0` remain validator-checked against theoretical capacity but excluded from the physical classifier.
- Classifier: per active lane queue fill = halting/empirical capacity; jam occupancy = min(1, capacity × 5 m / lane length). A lane is jammed if fill ≥0.90, occupancy ≥0.90 × jam occupancy, and mean speed <0.5 m/s. No jammed lanes: FREE if unweighted mean fill ≤0.10, else LIGHT; one or two jammed lanes HEAVY/WARN; three or more GRIDLOCK/FAIL. HEAVY is descriptive and not proof of malfunction.
- Conflict Monitor: network-derived fixture `supervisor/conflicts_2int_GPT.json`; protected G+G on foes FAIL, G+g allowed, g+g WARN; malformed/missing matrices, invalid lengths and self-conflicts fail safely. Asymmetric directed foes conservatively treated as undirected; bitstrings read right-to-left. Actual network: tl1 8 links/13 foe pairs; tl2 27 links/121 foe pairs including asymmetric (8,22), (8,23); no protected G+G in listed phases; tl2 phase 0 has 16 permissive g+g pairs. Previous 81 supervisor tests passed, then 88 passed in new repository; a generated toy-network SUMO test was included.
- Wrapper `supervisor/supervised_controller_GPT.py` and runner `run_supervised_GPT.py`: latches contingency per junction on veto. No automatic handback during an episode; `reset_episode()` clears latches at an episode boundary. The inner policy is still called while latched and actions for latched junctions are discarded; **this is not yet an approved shadow-handback mechanism**. Snapshot failures can affect both junctions; static controller's side effects are restored before evaluation. Fast Gate JSONL is written before `env.step()`: a decision log is not by itself proof SUMO completed/applied a cycle.
- `supervisor_ms_before_log` excludes policy prediction, logging, `env.step()` and LEDs. The main-thread TraCI snapshot cannot be preempted; observed timing flags are not a verified end-to-end HIL deadline. HIL LED-stage overrun previously proposed to be logged while retaining intended state; full ordering and timing design still open.

### 5.2 Early end — user explicitly deferred further investigation

Configured 19,800 s = 900 warm-up + five 3,600 s periods + 900 cool-down. Static/PPO examples stopped around 19,001 s. Environment checks `getMinExpectedNumber() == 0` inside its per-second loop, a likely early-stop reason. Seed-42 generated route inspected in this chat contained 9,031 vehicles, last scheduled departure 18,976.37 s and none at or after 19,001; six departures occurred after 18,900 because the demand ramp is centered on that boundary. SUMO previously reported 9,031 inserted, 0 running/waiting, 9,031 trip records. **User asked to assume traffic cleared during cool-down, not change termination, and revisit only if later early endings warrant it.** Don't claim the stop cause was instrumented or exact historical file identity independently established.

### 5.3 Slow Monitor — initial design agreed; thresholds experimental

The user explicitly decided policy behavior must be accompanied by poor traffic-performance evidence before triggering contingency. Repetition, pinning or toggling alone is not proof of a faulty RL policy. Traffic + behavior makes a suspicious condition; it does not prove causation.

- Traffic evidence: monitor 270 s rolling windows of pending insertion backlog and its change; insertions, arrivals, worst monitored lane fill at cycle boundaries, teleports and modes provide context. Backlog waiting outside the network is critical; spatial queues alone can conceal it.
- Behavior evidence per junction: persistent outward requests at min/max, and exact alternation between two applied splits (plus logged actions). Use proposed action/request and actual environment split after stepping; SARL selects one junction per joint action and the other gets an adapter no-op, so do not mistake that no-op for an independent RL decision. Current observer behavior-history length is 10 cycles, a descriptive configurable cap, **not a validated threshold**.
- User agreed an **opt-in per-junction trial** using provisional conditions: backlog ≥150 AND growth ≥30 over the prior 270 s, AND per junction either ≥5 consecutive outward requests at a bound OR ≥6 consecutive alternating applied splits. A matching junction switches to Webster FTSC at next cycle boundary and stays latched to episode end. These numbers were chosen after seeing seed-42 data: not validated, not approved as universal final thresholds.
- Pattern without adverse evidence is an observation/warning, not a trigger. Missing traffic measurements must not become zero; no combined trial match on unavailable measurements. Fast Gate's own fail-safe logic is unchanged. The experimental rule does not yet account for a large backlog clearing slowly/decreasing; user deferred further design/threshold study in favor of moving on.
- Per-junction scope is the approved initial design; user explicitly asked to **defer the network-wide-versus-per-junction question**. Do not silently switch both on a one-junction trigger.
- Observation-only first version and opt-in experimental implementation exist in separate runners. **Slow Monitor is not integrated into normal `run_supervised_GPT.py`** and no final operational integration or handback is complete.

### 5.4 Model selection

User chose the two 260k SARL models below as the correct models for this project and explicitly said no need to test the other SARL variants. Do not delete other variants or claim historical provenance has been independently proven. MARL uses per-agent shared-model observation shape `(9,)`, action `Discrete(3)`; SARL uses aggregate `(14,)`, joint `Discrete(6)`, with action = `3*junction_index + adjustment`.

| Model | Reported/observed SHA-256 |
|---|---|
| `models\marl_ppo_ps_2int_260k.zip` | `a45080e45965a7fc99146c473631407cf6d3d1c5ee624bdc623679350ffcfe20` |
| `models\marl_dqn_ps_2int_260k.zip` | `c99792f0b5fa201fa292cdbf8104921d6482f391aedc1f5e3b0a23c76fd57298` |
| `models\sarl_dqn_2int_260k.zip` | `08d109c3acf6165c45ac258be2ae100c22523e94f1ce738edeb206184bef0e2d` |
| `models\agents_original\sarl_ppo_2int_260k.zip` | `f526e55a7762be45aaeb1a4b86ee5b963b8f6d0424e3958af2c2839ebf753d0f` |

## 6. Evidence from this conversation

### 6.1 Fault-injection integration demonstration

`fault_experiment_GPT.py` ran short clean/injected real-SUMO trials. At t=180 s one synthetic `tl1` lane reading `596638461#0_0` was removed from a frozen snapshot for one decision. Input Validator returned `LANE_MISSING`/FAIL, the gate vetoed tl1 and the environment's active SUMO program reported tl1 A/B=31/47. The reading returned; tl1 stayed latched for subsequent observed cycles. tl2 stayed independently controlled and changed between 39/39 and 44/34 as requested. Clean control kept tl1 at 39/39. The two supplied trace logs did not include the separate reset check, so don't claim the trace itself proves reset. Synthetic fault demonstrates integration behavior, not a naturally occurring detector fault or hardware safety.

### 6.2 RL observations — seed 42, same generated route

The model route SHA-256 recorded in these observation/trial metadata files was `86fe8dc18d40a084d0873638e951032d34a4fa525c24aab460ef4444d192eb15`. All comparisons here are one seed and assumed same relevant runtime settings. Largest pending values are **cycle-boundary samples**, not continuous maxima.

| Controller | Boundary-sampled maximum pending | Key behavior and outcome |
|---|---:|---|
| Webster static | 130 | All 9,031 inserted/arrived; static action is a no-op after setting splits, not RL. |
| MARL PPO | 90 | `tl1` mostly alternated 15/20 s; `tl2` repeatedly requested +5 at 63 s max; all 9,031 inserted/arrived. |
| MARL DQN | 2,657 | Both junctions requested −5 on all 220 decisions, then remained at min 15 s; 6,644 inserted, 6,630 completed, 14 running, 2,387 waiting at t=19,800. 1 collision-related teleport; severe congestion did not trigger Fast Gate contingency. User explicitly accepted this as an initial design outcome and asked not to change the Fast Gate on that basis. |
| SARL DQN 260k | 786 | Joint action 0 selected `tl1` on all 215 decisions, requesting −5 and pinning at 15; `tl2` got no-op and remained 39/39; 9,031 completed, mean departure delay 924.48 s, 1 collision-related teleport. |
| SARL PPO 260k | 91 | Joint action 5 selected `tl2` on all 212 decisions, repeatedly requesting +5 and pinning at 63; `tl1` remained 39/39; 9,031 completed, mean departure delay 43.15 s. |

The MARL PPO run had no vetoes; tl2 Safe Gate WARNs stemmed from unclipped requests and the known permissive-conflict WARN was present. DQN severe congestion without contingency is an intentional observation of the current initial design, not grounds to retroactively claim failure of its specified Fast Gate.

Observer per-cycle totals matched SUMO insertion/arrival totals for checked runs. The `pending=None` printed for static/PPO/SARL partial final cycles means observer did not collect a full-cycle end boundary; it does not mean SUMO reported unknown pending. Later SARL observer excludes incomplete cycles from behavior history; use caution with older observer's partial-cycle behavior summaries. Windows sample lane fill at boundaries, not continuously. Backlog trend alone can miss a very large backlog that is shrinking slowly. Existing environment reports average trip delay for completed vehicles; do not directly compare such averages between runs with different completion populations.

### 6.3 Approved Slow Monitor switching trial — actual SUMO

`run_slow_trial_GPT.py` was run in both DQN cases, explicitly with `--enable-slow-trial`. Saved zip contains logs/metadata. There were no Fast Gate vetoes in these trial episodes. Normal decisions log describes Fast Gate processing; slow latch is set externally at a boundary, so look in `slow_trial_events_GPT.jsonl` and `slow_trial_applied` for the slow reason. Script modifies the wrapper instance's `modes` in a trial hook before `ctrl.act`; it is not a finalized public latch API. Next cycle log reports the environment's post-step split. In contrast to the earlier fault experiment, these trial traces did not independently read active SUMO program phase durations.

| Case | Slow event | Relevant findings |
|---|---|---|
| SARL DQN | `tl1` at t=1,350; pending 154, growth +35/270 s, sustained outward bound | `tl1` logged A=31 s after switch and stayed latched; `tl2` remained ASTA at A=39. Peak boundary backlog 772 vs 786 observation-only; all 9,031 completed in both; departure delay 930.83 s trial vs 924.48 s observation-only. No clear improvement from the per-junction switch. |
| MARL DQN | `tl1` and `tl2` at t=900; pending 200, growth +67/270 s, five outward bound attempts | Logged A splits 31 and 59 respectively for remainder; 9,031 completed and zero waiting in trial; peak boundary backlog 276. Observation-only completed 6,630 and had 2,387 waiting, max boundary backlog 2,657. Completed-trip average departure delay was 147.42 s vs 2,114.43 s respectively, but populations differ, so not a direct paired average comparison. |

The user chose to defer the design question of switching both junctions for a network-level concern and to move on. Results from one route/seed do not validate a universal trigger or establish causality.

## 7. Current scripts, tests and caveats

- `run_slow_observer_GPT.py`: observation only, static/MARL; records pending, arrival/insertion and teleport per-second counts, and lane fill at cycle boundaries. `run_slow_observer_sarl_GPT.py` adds SARL, preserves original joint action and selected junction, and excludes partial final cycles from behavior histories. Outputs go to a new refused-if-existing directory, plus standard SUMO outputs may be overwritten.
- `run_slow_trial_GPT.py`: based on SARL observer, supports static/MARL/SARL; trial activates **only** with `--enable-slow-trial`. Rule hard-codes experimental numbers above; logs metadata, decisions, observations and `slow_trial_events_GPT.jsonl`. It wraps `traci.simulationStep` temporarily while the existing environment steps to collect counts, restores it, and uses explicit `ctrl.modes` trial hook to set latches. Runtime observation overhead and behavior on all failure cases need review before production integration. No automatic handback.
- Trial rule replayed against five saved logs: MARL DQN first eligible t=900, SARL DQN tl1 t=1,350; static and both PPO logs no eligible match. Additional tests were run in the development workspace with mock TraCI/fixtures: per-junction latch, missing/incomplete input guards, reset, all six SARL action mappings and mocked end-to-end trial loop. Real-SUMO trial evidence is as above. These tests are not necessarily installed as repository tests and do not validate HIL.
- `fault_experiment_GPT.py`: synthetic frozen-snapshot missing-lane fault, copies inputs into an isolated run folder. See section 6.1.
- Normal wrapper logging failure stops before step; its latch is reset only at episode boundary. Slow experimental trial event is separately logged because setting `ctrl.modes` directly occurs before `ctrl.act` and wrapper's `entered_contingency` field relates to the Fast Gate path.
- If direct `env.step()` closes TraCI when an episode finishes, preserve partial-cycle data, mark unavailable values correctly and do not accidentally count an incomplete cycle as a full behavior cycle. User previously did not want unrelated `test_baseline_hil.py` missing `extract_lanes` issue investigated now.

## 8. Next topic: automatic handback design

Automatic handback was identified as the next design area before the user requested a new chat. **No automatic handback has been approved or implemented.** Discuss concrete alternatives and get agreement before new architecture. At minimum cover:

1. What a shadow RL proposal means while FTSC controls the junction. An important unresolved issue is whether the proposal is formed against the actual FTSC split (and how SARL's joint action/no-op is represented); do not assume it is settled. Existing discarded inner actions are not a complete shadow implementation.
2. Which checks and consecutive clean observations justify leaving contingency, including handling a continuing policy pattern, unhealthy sensors, unavailable measurements and check failures. A threshold such as a number of cycles must be presented as a proposal, not an agreed value.
3. When handback takes effect (cycle boundary), confirmation/actual commanded split, hysteresis/cooldown/grace period, preserving junction independence, and preventing oscillation or immediate re-entry. Avoid using `reset_episode()` as mid-episode handback.
4. How Slow Monitor network metrics and any grace/reset of performance comparisons interact with per-junction latches; network-wide switching question is deferred at user's request.
5. How to verify in controlled simulation with trigger, continued contingency, shadow recovery, handback and renewed fault; no claim of real hardware validation.

Normal production integration of Slow Monitor, validated thresholds across seeds/demand, slow measurement unavailability, behavior patterns with slow-clearing backlog, broader controller validation, and HIL timing/serial/LED design remain later work. The user values forward progress: do not restart the deferred exact baseline equivalence investigation without a reason.

## 9. Useful Windows CMD commands

From `C:\Users\sihle\Desktop\HIL-Traffic-Supervisor-Design`:

```cmd
.venv\Scripts\activate
set ASTA_NET=2int_files\osm.net.xml.gz
python -m pytest supervisor\tests -q
```

Example opt-in trial (new output directory required; this is the already-run SARL experiment and must be renamed for a rerun):

```cmd
python run_slow_trial_GPT.py --controller sarl-dqn --model models\sarl_dqn_2int_260k.zip --output-dir supervisor_logs\trial_sarl_dqn_260k_seed42_GPT --enable-slow-trial
```

Avoid reusing append-only Fast Gate decision-log paths. User decides whether to preserve/archive simulation outputs. Check current files and Git status before edits or commits; do not run destructive Git operations.

**End of handoff 6 — GPT.**
