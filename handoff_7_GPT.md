# ASTA Traffic Supervisor — Handover for Arduino Design

**Checkpoint:** 4 November 2026  
**Project:** Two-intersection SUMO traffic supervisor and Arduino LED demonstration  
**Next topic:** Design the Arduino communication and LED phase display with Siphetfo before implementing it.

This handover carries forward `handoff_6_GPT.md` and records the subsequent handback design, integration, GitHub merge and four-model simulations. Where an earlier handoff or guide says that automatic handback or the Slow Monitor has not been integrated, that statement is now outdated: both exist in the regular runner as **experimental opt-in features**. Inspect the current repository and local working tree before editing; this handover is a checkpoint, not a substitute for current files.

## 1. Start the next chat

Attach this `handover_GPT.md` and say: **“Continue the traffic supervisor project. Let’s design the Arduino communication and LED phase display.”** The handover alone is sufficient to begin the design discussion.

- Repository: [Ptoorich/HIL-Traffic-Supervisor-Design](https://github.com/Ptoorich/HIL-Traffic-Supervisor-Design). The repository is private, so do not assume a new chat has access; check the connection.
- If the new chat cannot access the repository and needs code, attach a **fresh snapshot of the current repository**, or the particular current files requested. At minimum for interface design/implementation these will likely include `run_supervised_GPT.py`, `sumo_env_marl_ps.py`, `tod_eval_common.py`, `supervisor/integrated_runner_GPT.py`, `supervisor/supervised_integrated_GPT.py`, `supervisor/slow_monitor_GPT.py`, `supervisor/observation_adapter_GPT.py`, the current configuration and tests. A complete source snapshot is preferable for implementation.
- `source_review_GPT.zip` and `handoff_6_GPT.md` are older snapshots. They are useful historical context but do not contain the latest integrated state. Do not replace current code with them.
- The result ZIPs listed in section 8 are needed only if the new chat must reanalyse those logs. The trained models and the complete SUMO scenario are not needed merely to discuss the hardware design; they are needed to run the simulation.

## 2. Working agreement and project objective

The supervisor sits between existing reinforcement-learning (RL) traffic controllers and a two-intersection SUMO model. It checks proposed signal splits, switches an affected junction to fixed-time signal control (FTSC) when the current rules require it, and can return control to RL when the experimental recovery conditions are met. The intended later demonstration uses an Arduino and LEDs to display the simulated signal phases.

- Focus on `tl1` and `tl2`, two logical phases A/B per junction, and the four selected SARL/MARL PPO/DQN models. Four- and six-intersection expansion and multi-phase expansion remain outside the current scope.
- Siphetfo prefers a working initial supervisor and incremental improvements over exhaustive tuning before moving on. Explain design choices and trade-offs, especially new architecture, safety, contingency, recovery and hardware behaviour, and obtain agreement before implementing substantive new design decisions.
- Keep the decision log **inside the handover**, not in a separate decision-log file. Use `GPT` in new AI-generated deliverable and updated implementation filenames; preserve required compatibility shims and test existing imports.
- User environment: Windows, usually Command Prompt in VS Code; project folder `C:\Users\sihle\Desktop\HIL-Traffic-Supervisor-Design`, with `.venv`, Python 3.14.0 and SUMO 1.27.1 as previously reported. Provide runnable CMD commands. The assistant does not have direct access to the Windows terminal unless a suitable connection is explicitly available.
- Do not claim that the current simulation, thresholds or future LED demonstration validate a physical traffic-signal system or an end-to-end hardware deadline.

## 3. Repository and current software state

- [Pull request #1](https://github.com/Ptoorich/HIL-Traffic-Supervisor-Design/pull/1) added the opt-in integration and was merged into `main`. The verified merge commit on 4 November was `116c6c96684c636fb7918003bfd27e7143ce8e01`; the tested feature-branch head was `c46ece21df3143e5a36c4838c36b1aefbe13a424`. The repository may receive new commits after this checkpoint: verify the current head.
- Siphetfo intended to push the locally saved simulation-result archives after this handover. **That push was not confirmed at the time this document was written.** Do not assume the archives are already on GitHub; inspect the current repository and working tree.
- `run_supervised_GPT.py` uses the original controller/evaluation loop if neither optional switch is supplied. With `--enable-slow-monitor` and/or `--enable-auto-handback`, it uses the separate integrated path. Both switches default to **off** and may be enabled independently; either requires a new isolated `--run-dir` outside `2int_files`.
- The opt-in path currently supports `--controller rl` with `--kind marl` or `--kind sarl`. Static and fixed controllers continue on the legacy path. The original `supervisor/supervised_controller_GPT.py` remains unchanged for compatibility.
- Current integrated components are `supervisor/integrated_runner_GPT.py`, `supervisor/supervised_integrated_GPT.py`, `supervisor/slow_monitor_GPT.py`, `supervisor/observation_adapter_GPT.py` and `supervisor/recovery_GPT.py`. See `INTEGRATED_SUPERVISOR_GUIDE_GPT.md` for current switches, rules and output files. Older trial guides describe their own separate runners; they are not authoritative about the current integrated path.
- The repository ignores `supervisor_logs/`, `.venv` and caches. Integrated runs copy the SUMO scenario into an isolated per-seed `sumo_work` folder. Avoid committing the copied work trees, temporary route files or overwritten SUMO outputs. `supervisor_logs/` can contain explicitly tracked result ZIPs if the user later force-adds those individual files.

## 4. Fixed simulation parameters

| Item | Current agreed value |
|---|---|
| Cycle | 90 simulated seconds |
| Junction phases | A and B for each of `tl1` and `tl2` |
| Intergreens | 3 s yellow and 3 s all-red after each green; 12 s total per cycle |
| RL action | `0 = -5 s`, `1 = no change`, `2 = +5 s` to A green |
| A green bounds | 15–63 s; B green = `90 - A - 12` |
| Fixed-time contingency | `tl1` A/B = **31/47 s**; `tl2` A/B = **59/19 s** |
| Safe Gate | FAIL below 8 s; WARN from 8 s to below 11 s; PASS at least 11 s, for either requested green |
| Slow Monitor traffic window | 270 s, i.e. three complete 90 s cycles |
| Fast Gate stage target | 2.0 s; resolver target 0.1 s |
| Whole-supervisor budget | 3 s is provisional and not validated end to end |

Do not mistake 39/39 for the fixed-time contingency. The Fast Gate checks the RL **requested** split before environment clipping: for example, a request from A=63 to A=68 gives B=10 and a warning; the environment subsequently clips the applied A split to 63, leaving B=15.

## 5. Design decision log

### 5.1 Fast Gate and contingency

- One frozen main-thread TraCI snapshot is taken per decision; checks run in parallel on that snapshot. Both junctions are checked, including adapter no-ops. The four checks are Input Validator, physical Traffic Classifier, Safe Gate and network-derived Conflict Monitor.
- PASS/WARN/FAIL are distinct: an approved WARN permits the action; FAIL, timeout, exception, malformed data or a missing gate result vetoes the affected junction and latches it in FTSC. One junction’s failure does not automatically switch the other. A shared snapshot or controller failure can affect both. Switches take effect at cycle boundaries.
- Input validation treats occupancy as a 0–1 fraction, confirmed by a local SUMO 1.27.1 probe. Speed WARN/FAIL is at 1.2/1.5 times the speed limit; vehicle count at capacity+1 is WARN, higher is FAIL. Interpret empty-lane speed with the vehicle count.
- Physical classifier: queue fill = halting vehicles / empirical capacity; jam occupancy = `min(1, capacity * 5 m / lane length)`. A lane is jammed if fill ≥0.90, occupancy ≥0.90 of jam occupancy and mean speed <0.5 m/s. With no jammed lanes, FREE means unweighted mean fill ≤0.10, otherwise LIGHT; one or two jammed lanes gives HEAVY/WARN, and three or more gives GRIDLOCK/FAIL. HEAVY is descriptive, not proof of a faulty policy. Both junctions have five non-excluded classification lanes. `669676574#0_0` and `-669676574#8_0` are still validator-checked but excluded from physical classification.
- Conflict fixture: `supervisor/conflicts_2int_GPT.json`. Protected G+G on foes fails; G+g is allowed; g+g gives a WARN. Malformed/missing matrices and self-conflicts fail safely; asymmetric directed foes are treated conservatively as undirected. The actual network fixture has 8 links/13 foe pairs for `tl1` and 27 links/121 pairs for `tl2`. `tl2` includes known permissive g+g warnings; a warning is not the same as a collision.
- Approved P1–P8 changes: lane-specific jam occupancy; occupancy as a fraction; speed-limit factors; a one-vehicle count tolerance; physical classification; conservative asymmetric conflicts; `datatypes.py` instead of `types.py`; and gate evaluation of the unclipped requested split. Preserve compatibility imports.
- A synthetic missing-lane fault at `tl1` was observed to cause `LANE_MISSING`, gate veto and the actual SUMO FTSC programme, while `tl2` continued independently. A synthetic fault is not proof of behaviour under every real detector failure.

### 5.2 Slow Monitor

- A policy pattern alone is not proof of failure. The provisional, opt-in per-junction trigger requires pending insertion backlog ≥150 **and** growth ≥30 over the prior 270 s, plus either ≥5 consecutive outward requests at the relevant bound or ≥6 consecutive alternating applied splits for that junction. The trigger is evaluated from cycle data and is applied at the next cycle boundary.
- Backlog counts vehicles waiting to be inserted, which spatial queues can miss. Insertions, arrivals, boundary-sampled lane fill, teleports and modes provide context. Unknown values do not become zero and do not satisfy the combined trigger. Lane-fill and pending maxima reported from boundary observations are samples, not continuous maxima.
- The 10-cycle behaviour-history length is a descriptive configurable cap; the trigger values are experimental, chosen after seed-42 observation. Slow-clearing but decreasing backlog remains a limitation. Whether a network-wide condition should switch both junctions was **explicitly deferred**; preserve per-junction switching in the current design.

### 5.3 Automatic handback

- Handback is experimental, independent per junction and opt-in. During FTSC the policy is evaluated in shadow with a fresh observation representing the actual FTSC split; a passing shadow proposal does not prove that later traffic performance will be good.
- Require at least **five complete fixed-time cycles** and **three consecutive clean cycles**, with valid measurements and acceptable gate/programme evidence. For SARL, also require at least **three actual selections of the affected junction** in the recent window of up to ten clean complete cycles. A no-op generated for an unselected junction is not an actual selection.
- If the Slow Monitor caused the latch, pending backlog must additionally be below 100 at three consecutive healthy cycle ends and remain valid at release. A Fast Gate-only latch does not use that backlog threshold, although measurements must be available. Missing/invalid data and failed checks clear recovery evidence; incomplete cycles are not counted as verified recovery.
- Once ready, hold one further FTSC transition cycle. At the next boundary, check a **fresh** policy proposal through the existing Fast Gate. A veto or invalid evidence retains contingency. A new fault after release starts a new hold. Rolling network traffic history continues; per-junction RL behaviour history restarts after fixed-time/transition cycles.
- An initial SARL PPO fault at unselected `tl1` remained latched throughout its 20-cycle test; an injected fault at selected `tl2` did hand back. Siphetfo chose to document the unselected-junction outcome as an **initial design observation**, without adding a SARL fallback. Do not introduce one without a new agreed decision.

### 5.4 Timing, hardware and deferred items

- `supervisor_ms_before_log` measures a limited software scope: it excludes policy prediction, logging, `env.step()` and LEDs. The main-thread TraCI snapshot cannot be preempted, and resolver/overall flags are observations within their stated scope. Neither the 2 s stage target nor the provisional 3 s overall budget is an end-to-end HIL guarantee.
- A prior discussion proposed logging an LED-stage overrun while retaining the intended state; serial-loss behaviour, LED update order, timing boundaries and the complete failure strategy have **not** been agreed or implemented. Do not treat the earlier proposal as a complete fail-safe design.
- The episode may end before the configured 19,800 s when the network has cleared; an environment check of `getMinExpectedNumber()` is a plausible mechanism, but the exact historical termination cause was not instrumented. Siphetfo explicitly deferred further early-end investigation unless later evidence warrants it. The unrelated `test_baseline_hil.py` import issue and exhaustive baseline equivalence investigation were also deferred.
- Threshold validation across seeds/demand, large slowly decreasing backlog, broader performance evaluation, and the per-junction versus network-wide decision remain future work. Do not silently reopen them as prerequisites to the Arduino design.

## 6. Simulation and automated-test evidence

The user ran `python -m pytest supervisor\tests -q` on Windows: **121 passed, 6 subtests passed**. The earlier standalone trial suite and local mock integration checks also passed. Simulation results below are all **seed 42**, same recorded route SHA-256, peak scale 0.37 / peak rate 2664 vehicles/hour, and both opt-in features enabled. All 9,031 vehicles were recorded as inserted and arrived in each integrated run; all final supervisory modes were `ASTA`.

| Model | Slow contingencies `tl1/tl2` | Verified handbacks `tl1/tl2` | Peak sampled pending | Mean departure delay | Teleports | End time |
|---|---:|---:|---:|---:|---:|---:|
| MARL PPO | 0 / 0 | 0 / 0 | 90 | 28.61 s | 0 | 19001 s |
| SARL PPO | 0 / 0 | 0 / 0 | 91 | 43.15 s | 0 | 19002 s |
| MARL DQN | 2 / 2 | 2 / 2 | 276 | 264.25 s | 0 | 19283 s |
| SARL DQN | 1 / 0 | 1 / 0 | 772 | 930.83 s | 1 | 19401 s |

All four had zero Fast Gate contingency entries in these integrated episodes. The SARL DQN teleport was reported as collision-related; its cause was not established. SUMO reported emergency stops/brakings respectively: MARL PPO 5/5; SARL PPO 8/8; MARL DQN 6/6; SARL DQN 15/16. Do not infer causation or general controller ranking from one seed or attribute that teleport to handback.

In the PPO runs there were no Slow Monitor triggers, handbacks or logged observation errors. Both had 212 observed cycles: 211 complete and a final partial cycle of 11 s for MARL PPO and 12 s for SARL PPO. The active SUMO programme after the first simulation step of each observed cycle matched the recorded split and expected states. No logged Fast Gate stage, resolver or pre-log total timing flags were raised; this does not measure the end-to-end HIL deadline.

- MARL PPO's integrated and earlier Slow Monitor trial used the same route/model hashes and matched across all 212 compared records for sampled traffic, actions, splits, gate outcomes and active-programme readbacks, excluding runtime timing fields.
- SARL PPO selected joint action 5 / `tl2` on all 212 decisions; `tl1` stayed at 39/39 and `tl2` reached 63/15. This matches the earlier observation summary. This is not a full per-cycle comparison with an unavailable earlier SARL PPO integrated trial.
- No fault was injected in the PPO integrated runs. Zero contingencies and handbacks therefore do **not** mean a handback failed; handback was not exercised in those runs. The prior synthetic-fault prototype tests showed MARL PPO handback at `tl1`, SARL DQN handback at `tl1`, and SARL PPO handback at `tl2`, followed by renewed-fault handling in the reported demonstrations. The unselected SARL PPO `tl1` case stayed latched as noted above. MARL DQN and SARL DQN exercised Slow Monitor-triggered handbacks in the integrated runner.
- `tl2` commonly produces `APPROVE_WARN` because of the configured permissive-conflict and/or requested-green warnings. Do not equate WARN with a veto or assert that no warnings occurred.

## 7. Model and route identities

| Selected model file in the Windows project | SHA-256 |
|---|---|
| `models\marl_ppo_ps_2int_260k.zip` | `a45080e45965a7fc99146c473631407cf6d3d1c5ee624bdc623679350ffcfe20` |
| `models\marl_dqn_ps_2int_260k.zip` | `c99792f0b5fa201fa292cdbf8104921d6482f391aedc1f5e3b0a23c76fd57298` |
| `models\sarl_dqn_2int_260k.zip` | `08d109c3acf6165c45ac258be2ae100c22523e94f1ce738edeb206184bef0e2d` |
| `models\agents_original\sarl_ppo_2int_260k.zip` | `f526e55a7762be45aaeb1a4b86ee5b963b8f6d0424e3958af2c2839ebf753d0f` |

The user selected the two 260k SARL models; do not substitute similarly named variants or delete the others. MARL uses per-agent observation shape `(9,)` and `Discrete(3)` action; SARL uses aggregate shape `(14,)` and joint `Discrete(6)` action `3 * junction_index + adjustment`. The recorded route SHA-256 in the integrated runs is `86fe8dc18d40a084d0873638e951032d34a4fa525c24aab460ef4444d192eb15`.

## 8. Result files and interpretation

The four latest result archives created on the user’s computer are:

- `supervisor_logs\integrated_marl_ppo_results_GPT.zip`
- `supervisor_logs\integrated_sarl_ppo_results_GPT.zip`
- `supervisor_logs\integrated_marl_dqn_results_GPT.zip`
- `supervisor_logs\integrated_sarl_dqn_results_GPT.zip`

The earlier `handback*_results_GPT.zip` files cover the scripted demonstration, synthetic-fault and slow-trial cases. The current run archives contain `metadata_GPT.json`, `summary_GPT.json`, `decisions_GPT.jsonl`, `observations_GPT.jsonl` and `events_GPT.jsonl`. A pre-step contingency decision is not by itself proof of execution: use the programme readback and complete-cycle/handback events. Programme readback is evidence from SUMO, not an electrical measurement of a signal head.

The user intended to push handback and integrated result ZIPs to the repository. Since `supervisor_logs/` is ignored, explicit `git add -f` of the intended archives is required if retaining that location. Do not force-add the directory or `sumo_work` folders. Check whether the push actually occurred before relying on GitHub copies.

## 9. Arduino / LED design: start here

The user is ready to design the hardware interface. The earlier concept was an Arduino on a breadboard with **four signal heads and 12 red/yellow/green LEDs**, plus approximately six spare LEDs. The exact board, wiring, resistors, power arrangement and physical mapping have not been confirmed in this handover. Ask only for hardware details that materially affect the proposed design; do not invent a pinout or assume every LED has been wired.

An interface design should discuss and obtain agreement on:

1. **Display meaning and mapping.** Decide what each of the four heads represents at `tl1`/`tl2`, how phase A/B and red/yellow/green map to LEDs, and whether the display represents logical phases or particular SUMO links. The SUMO network has multiple links and permissive as well as protected movements, so do not assume a simple green LED represents every actual movement without an agreed abstraction.
2. **Source of truth and update order.** Decide whether messages represent actual SUMO phase states, intended commands, or another agreed representation; define when changes are read and sent relative to supervisor decisions and simulation steps. Cover normal operation, FTSC entry, transition and handback.
3. **Time base.** A SUMO cycle is 90 *simulated* seconds. The existing full runs execute much faster than wall-clock time; choose how real-time pacing or an explicitly agreed demonstration speed makes LED phase changes observable, and how intergreens are represented. Avoid assuming that a complete episode must run in real time.
4. **Serial protocol and lifecycle.** Define board/port configuration, message format and framing, timing, acknowledgements if needed, startup/shutdown, malformed/stale messages, disconnect/reconnect and what the LEDs show when communications fail. These are design choices, not current implementation facts.
5. **Electrical design.** Confirm the Arduino model, LED current/resistors, available pins, common ground and power needs before preparing wiring or code. Keep the display as a laboratory demonstration and do not present it as approved field control.
6. **Timing and verification.** Define the actual end-to-end timing boundary and how to measure it; distinguish simulation time, wall time, software gate timing and serial/LED latency. Plan a short reproducible demonstration and tests for phase transitions, two-junction independence, fault-driven contingency, handback and communication loss, with expected LED states and logged evidence.

**Current code constraint:** `SumoMARLEnvPS.step()` applies actions and then performs up to 90 one-second `traci.simulationStep()` calls before returning. The integrated runner uses a scoped `EvidenceProbe`/`StepProbe` wrapper around `traci.simulationStep` to collect counters and the first-step active-programme readback. A phase display that only updates after `env.step()` would miss intermediate phase changes; review suitable integration points and compatibility with this probe before choosing an approach. No Arduino code, protocol, LED pin map or hardware-driven phase synchronisation is implemented in the integrated runner at this checkpoint.

The current six-phase SUMO programme is constructed from network states as A green, A yellow, all-red, B green, B yellow, all-red, with the agreed durations. Junction timings can differ because their splits differ. Preserve the existing controller, safety logic, working default path and opt-in behaviour unless the user agrees to a change and it is tested.

## 10. Useful Windows commands

```cmd
cd /d "C:\Users\sihle\Desktop\HIL-Traffic-Supervisor-Design"
.venv\Scripts\activate
git status --short --branch
set ASTA_NET=2int_files\osm.net.xml.gz
python -m pytest supervisor\tests -q
```

For a future run, see `INTEGRATED_SUPERVISOR_GUIDE_GPT.md`; both opt-in flags require a **new** `--run-dir`, and the first standard seed with `--seeds 1` is 42. Check the current branch and preserve local changes before any checkout or pull. Do not reset, delete output directories or overwrite the user's files without a clear reason and authorization.

**End of handover — GPT.**
