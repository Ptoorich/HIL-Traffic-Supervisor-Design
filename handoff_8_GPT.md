# HIL Traffic Supervisor — handoff_8_GPT.md

Checkpoint: 5 November 2026, Africa/Johannesburg.
Repository: https://github.com/Ptoorich/HIL-Traffic-Supervisor-Design
Current task: add two 1602 I2C LCDs to show ASTA/FTSC control mode, one LCD per intersection.

This is a consolidated project checkpoint. It supersedes earlier statements that Arduino communication is only a proposal. Treat historical documents as context and follow the user's current request.

## 1. Resume here

The twelve traffic LEDs and USB serial interface are implemented. The user completed a three-cycle physical HIL run and reported that the subsequent real-time SUMO GUI/LED demonstration seemed to run fine.

The user now wants a physical indication of which controller owns each junction. They offered two 1602 I2C LCDs after considering discrete LEDs and bare seven-segment displays. The latest direction is to use the LCDs. Both have PCF8574T adapters, GND/VCC/SDA/SCL connectors and A0/A1/A2 address solder pads.

**The immediate pending input is the separately scanned I2C address of each LCD. Neither result has been reported. LCD mode-display firmware has not been implemented.** The user was given a temporary scanner sketch, but its upload and physical execution are unconfirmed. Do not assume which sketch is currently on the board.

Suggested opening message for the next chat:

> Continue the traffic supervisor project using handoff_8_GPT.md. The Arduino traffic LEDs work, and we are adding two PCF8574T 1602 I2C LCDs to show ASTA/FTSC control mode. Resume from the separate LCD address scans and then integrate the displays without disrupting the existing serial watchdog or phase output.

## 2. Workspace, delivery and source status

- User's Windows project: `C:\Users\sihle\Desktop\HIL-Traffic-Supervisor-Design`, using `.venv` and usually Command Prompt in VS Code. Reported Python 3.14 and SUMO 1.27.1.
- Hardware port: **COM5**, enumerated as `USB-SERIAL CH340 (COM5)`. Earlier COM4 examples are obsolete for this board.
- The assistant has no direct access to that Windows machine or physical hardware. User-reported observations and cloud test evidence are separate.
- Download links repeatedly failed. The user requested pasted content. Supply complete code or applicable patches inline, with Windows instructions; do not rely solely on download links.
- The user manually installed the HIL implementation and GUI update, evidenced by their runs. The fault-test runner and LCD scanner were supplied inline; their physical runs have not been confirmed.
- Cloud repository: `/workspace/HIL-Traffic-Supervisor-Design`, branch `work`, HEAD `22310895d638b2d0c55f9bc4b01c77b9c11b96e2` at this checkpoint.
- **HIL, GUI, fault-runner and scanner changes remain local/uncommitted. No commit or push of these changes was performed. A fresh clone of main may lack them.** Inspect the current checkout and preserve local edits before any Git operation. If missing, obtain current files from the user's Windows copy or retained workspace rather than recreating them from an older sketch.
- The existing integrated-result ZIPs and earlier handback-result ZIPs are tracked in this checkout. Older notes saying their push was unconfirmed are superseded by this local Git inspection; no new remote fetch was performed for this handoff.
- Keep substantive decisions in this handoff. Use `_GPT` naming for new deliverables, preserving compatibility imports. The user prefers incremental implementation and practical demonstrations; continue within the scope already authorized.

## 3. Confirmed hardware and wiring

UNO R3-compatible starter-kit controller with CH340 USB serial; powered by USB. Four heads represent TL1-A, TL1-B, TL2-A and TL2-B. Outputs are active-high. Each traffic LED cathode has its own resistor to the common ground rail. Resistors were estimated by the user at 150–220 ohms; actual values/current were not measured remotely.

| Head | Green | Yellow | Red |
|---|---|---|---|
| TL1-A | D2 | D3 | D4 |
| TL1-B | D5 | D6 | D7 |
| TL2-A | D8 | D9 | D10 |
| TL2-B | D11 | D12 | D13 |

D0/D1 serve serial. A0–A5 were unused by the phase-display firmware. TL1-A yellow was found faulty during testing and replaced with a separately checked working spare. Ground is common; reconnect any split breadboard rails as needed.

LCD wiring, for each confirmed four-pin 1602 backpack:

| LCD pin | UNO connection |
|---|---|
| GND | Common ground |
| VCC | 5V |
| SDA | A4 |
| SCL | A5 |

Once distinct addresses are established, both LCDs share these four connections. The backpack's A0/A1/A2 address pads are separate from the UNO's similarly named analog pins.

Other available parts: two bare 12-leg four-digit displays marked `5461AS-1` and a `74HC595`. Their polarity/pinout was not verified. The earlier proposal to use four extra status LEDs on A0–A3 was not implemented. Seven-segment drivers are not part of the current LCD plan.

## 4. Exact LCD checkpoint and next steps

Scanner file: `arduino/lcd_address_scan_GPT/lcd_address_scan_GPT.ino`.

The supplied procedure was:

1. Stop Python/SUMO, disconnect USB, and connect only LCD 1 using the table above.
2. Upload the scanner as a separate Arduino Uno sketch on COM5. It uses the built-in Wire library, 100 kHz I2C and a 25 ms bus timeout; it scans once at startup and holds all four traffic reds on.
3. Open Serial Monitor at 115200 baud and record `Found I2C device at 0x__`.
4. Close the monitor, disconnect USB, replace LCD 1 with LCD 2, reconnect and reopen the monitor. Record the second address.
5. Report `LCD 1: 0x__` and `LCD 2: 0x__`.

PCF8574T (non-A) uses the 0x20–0x27 range. The common default is 0x27, but neither actual address is known. Two devices at the same address appear as one scan result. They cannot display independent status on the same bus until one address changes. If necessary, identify the appropriate backpack address-pad connection, change one address and rescan; do not assume a particular jumper result without verifying it.

**The scanner replaces the running HIL sketch and does not implement its protocol. Restore/update `asta_display_GPT.ino` before any Python HIL run.** A scanner upload and its successful compilation do not establish that an LCD is wired correctly or displays text.

After address identification:

- Configure distinct TL1/TL2 LCD addresses and confirm each screen can be addressed separately.
- Reuse the mode fields already carried by the serial STATE frame. Show ASTA or FTSC for the corresponding intersection. During the recovery transition, FTSC still controls; only show ASTA after the accepted release, not after the later completed-cycle verification.
- Show a clear waiting/paused/stopped/fault status instead of retaining an obsolete active-controller label. Exact LCD wording/layout is still a design choice; no countdown display was requested.
- Preserve the traffic LED mapping, protocol sequencing and watchdog. Bound I2C operations so a failed LCD/bus cannot indefinitely block serial processing, red flashing or the software watchdog. Choose/update the LCD driver accordingly; no LCD library has been selected or installed for the HIL firmware yet.
- Extend relevant firmware/readback tests, compile for the UNO, check LCD behaviour on startup/normal states/transition/stop/communication loss, then repeat a physical GUI run and the controlled fault demonstration.

## 5. Supervisor behaviour to preserve

| Item | Current value |
|---|---|
| Junctions | tl1 and tl2, phases A/B at each |
| Cycle | 90 simulated seconds |
| Intergreens | 3 s yellow and 3 s all-red after each green |
| RL actions | 0 = -5 s, 1 = no change, 2 = +5 s to A green |
| A bounds | 15–63 s; B = 90 - A - 12 |
| FTSC contingency | TL1 A/B = 31/47 s; TL2 A/B = 59/19 s |
| Safe Gate | FAIL below 8 s; WARN from 8 to below 11 s; otherwise PASS |

39/39 is not the contingency plan. The Fast Gate checks the requested split before environment clipping. Its checks are Input Validator, physical Traffic Classifier, Safe Gate and Conflict Monitor, using a frozen main-thread snapshot. PASS/WARN can approve; failures, missing evidence and timeouts veto the affected junction. WARN is not a contingency. Preserve independent switching per junction.

The Slow Monitor and automatic handback are experimental, separate opt-in features:

- Slow trigger: pending insertion backlog at least 150 and growth at least 30 over three complete cycles, plus at least five outward bound requests or six alternating applied splits at that junction. Missing values do not become zero. Trigger application is at the next boundary.
- Recovery: at least five complete FTSC cycles and three consecutive clean cycles. SARL additionally needs at least three actual selections of the affected junction in up to ten recent clean complete cycles.
- A Slow Monitor latch additionally needs pending below 100 at three healthy cycle ends and valid evidence at release. A Fast Gate-only latch does not use that backlog threshold.
- Hold one additional FTSC transition cycle, then ask the Fast Gate to approve a fresh policy proposal before release. Invalid evidence or veto retains contingency. Completed handback is logged only after an entire returned cycle with programme readback.
- SARL PPO's unselected TL1 remaining latched was an explicitly retained design observation. Network-wide switching, broader threshold tuning and earlier episode-end investigations were deferred. Do not add alternative recovery rules as part of LCD work.

Gate-stage target 2 s, resolver target 0.1 s and provisional whole-supervisor budget 3 s are scoped software targets, not proven physical deadlines. `supervisor_ms_before_log` excludes policy prediction, logging, SUMO stepping and LEDs. A software gate failure and an Arduino communication failure are different events.

## 6. Implemented phase display and serial contract

The board mirrors actual SUMO programme, phase and state readback. It does not schedule the traffic plan. Each LED head is a logical phase group; individual permissive SUMO turning signals need not match a single group LED.

| Phase code | A head | B head |
|---|---|---|
| 0 | Green | Red |
| 1 | Yellow | Red |
| 2 | Red | Red |
| 3 | Red | Green |
| 4 | Red | Yellow |
| 5 | Red | Red |

Mode fields are already transmitted: 0=ASTA, 1=FTSC, 2=FTSC transition. Firmware currently validates these fields but does not drive auxiliary status displays.

- Serial: 115200 baud, protocol version 1, bounded ASCII/LF frames, maximum 160 bytes. CRC-16/CCITT-FALSE: polynomial 0x1021, initial 0xFFFF, no reflection/xorout; checksum is four uppercase hex characters after `*`.
- STATE payload: `1,S,session,seq,sim_ms,p1,p2,m1,m2`. ACK payload: `1,A,session,seq,mask,kind`. Other commands are H handshake, P paused/all-red, X normal stop/all-red and F host fault. See the current guide/code for the complete wire contract.
- Boot READY includes a persisted counter. H includes the matching counter and a random nonzero host session. EEPROM bytes 0–7 hold signature/counter. Sequence increases without wrapping. A latched fault requires board reset; stale buffered commands cannot silently rearm it.
- ACK deadline is 500 ms total, including one identical retry at 250 ms. Duplicate commands do not reapply outputs or refresh the watchdog. Incomplete input frames expire after 200 ms.
- GPIO ACK checks all twelve output levels. Bits 0–11 correspond to D2–D13; all-red mask is 2340. ACK is not proof of LED illumination. A failed LED was found visually despite pin-level readback.
- Outputs are cleared before the new mask is applied. Communication freshness watchdog is 3000 ms. On fault, reds immediately turn on and then flash 500 ms on/500 ms off, approximately 1 Hz. Fault handling is latched. USB power loss makes the board dark.
- Startup flashes red; handshake/explicit initialization pause and normal stop use steady all-red. Normal FTSC fallback continues green/yellow/red cycles. The watchdog is implemented in software, not the independent AVR hardware watchdog.
- No background heartbeat keeps an old green alive when the simulation stalls. Serial/display errors terminate HIL and request the existing fault indication; they are not counted as traffic-control contingencies.

## 7. Integration, GUI and source files

`SumoMARLEnvPS.step()` internally advances up to 90 one-second steps. The existing scoped EvidenceProbe wraps those calls, sampling after action application before the first step and after every simulated second. Do not add a competing global TraCI wrapper or update the LEDs only once per cycle.

Pacing now occurs in `PhaseDisplay.prepare_step()` **before SUMO advances**. Readback is sent immediately after each step. The earlier implementation slept after advancement, which could leave the GUI ahead of the LEDs. Preserve this correction. Observation-to-ACK excludes deliberate pacing; serial RTT is logged separately.

`--hil-port` enables the isolated RL path; `--hil-speed` supports 1 or 3. Slow Monitor/handback default off and can be enabled independently. `--gui` uses the enhanced path, enables `use_gui`, and changes only the run's copied SUMO configuration to start=true, delay=0 and quit-on-end=true. Use a fresh run directory outside `2int_files`. Static/fixed controllers retain their legacy path.

Keep GUI playback running with delay zero during active HIL. Pausing it can stall TraCI and trigger the command watchdog. GUI closes at normal completion. At speed 1, 270 simulated seconds take about 4.5 minutes plus overhead.

| File | Purpose |
|---|---|
| `arduino/asta_display_GPT/asta_display_GPT.ino` | Working phase-display firmware; no LCD support yet |
| `arduino/lcd_address_scan_GPT/lcd_address_scan_GPT.ino` | Temporary separate LCD address scanner |
| `supervisor/hil_serial_GPT.py` | Serial framing, sessions, ACKs, lifecycle and logs |
| `supervisor/hil_display_GPT.py` | SUMO readback, modes and pacing |
| `supervisor/integrated_runner_GPT.py` | Existing supervisor loop plus optional GUI/HIL integration |
| `run_supervised_GPT.py` | Main CLI, including HIL and GUI flags |
| `bench_arduino_GPT.py` | Lamp sequence, port listing and powered watchdog check |
| `run_hil_fault_test_GPT.py` | Dedicated ten-cycle fault/handback demonstration |
| `requirements-hil_GPT.txt` | Optional `pyserial==3.5` dependency |
| `ARDUINO_HIL_GUIDE_GPT.md` | HIL protocol, bench, GUI and fault-runner guide |
| `INTEGRATED_SUPERVISOR_GUIDE_GPT.md` | Slow Monitor/handback behaviour and evidence |
| `supervisor/tests/test_hil_GPT.py` | Host/firmware tests, native board and PTY coverage |
| `supervisor/tests/uno_native_GPT/` | Harness compiling the exact Arduino firmware with mocked peripherals |
| `supervisor/tests/test_hil_sumo_GPT.py` | SUMO-to-firmware contingency/handback test |
| `supervisor/tests/test_hil_fault_runner_GPT.py` | Actual fault runner and evidence validation tests |

The CLI and integrated-runner tests were also extended. Legacy controller/environment sources covered by `source_fingerprints_GPT.json` remain unchanged; their hashes were rechecked at this handoff. Never alter expected fingerprints merely to bypass a compatibility failure.

If the new chat lacks this working tree, obtain at least the current firmware, HIL transport/display files, integrated runner, CLI, fault runner, scanner and relevant tests. The original user's four-character `GRRG` sketch is superseded and incompatible with the current protocol.

## 8. Physical results versus cloud evidence

Physical/user-reported:

- COM5 identified and used successfully; opening the missing COM4 failed earlier.
- Powered watchdog bench test reported firmware expiry. The console asked for visual confirmation; a separate explicit report that all four reds flashed was not received.
- TL1-A yellow was the only failed lamp in the user's checks. User replaced it with a tested spare.
- `hil_marl_ppo_repaired_01_GPT`: seed 42, three complete cycles, 270 simulated seconds, 93.49 s reported duration, real-time factor 2.8881; both junctions ASTA, zero contingencies/handbacks. TraCI termination matched the requested cycle limit.
- After applying the GUI/pacing patch and running at speed 1, user said it “seemed to run fine.” This is qualitative, not measured synchronization latency.
- No physical result from the new ten-cycle fault runner or either LCD address scan has been reported.

Cloud verification completed before this documentation update:

- Full regression: **163 passed, 9 subtests passed** in 10.50 s. This was not a newly repeated suite for the handoff edit.
- Exact Arduino firmware runs in a native harness checking every GPIO write, malformed/stale frames, retries, watchdog and timer wrap. Real pyserial was also exercised through a POSIX pseudo-terminal.
- Actual PPO model + SUMO + compiled native firmware completed ten cycles with 910 samples and all six phases at both junctions. The same runner passed using real SUMO GUI under a virtual X display. Simulated serial/GPIO and a virtual clock do not measure physical USB or optical latency.
- Phase firmware AVR cross-build: 6,930 bytes flash and 656 bytes static RAM at the earlier checkpoint.
- LCD scanner AVR cross-build: **4,368 bytes flash and 404 bytes static RAM**, Arduino AVR core 1.8.7. No physical LCD result is implied.

Cloud references, if this environment survives: `/workspace/hil-venv` (Python 3.12, SUMO 1.28), `/workspace/avr-tools`, `/workspace/hil-runs/fault-runner-regression_GPT.log`, `/workspace/hil-runs/fault_gui_GPT_7u1w6tlt/run/seed_42/fault_test_result_GPT.json`, `/workspace/hil-runs/lcd-scan-build_GPT.log`. These paths are not on the user's PC and are not guaranteed in another chat.

## 9. Controlled fault demonstration

`run_hil_fault_test_GPT.py` uses MARL PPO, seed 42 and ten 90-second cycles. It removes lane `596638461#0_0` from exactly one TL1 supervisor snapshot at t=90. Subsequent snapshots are normal; SUMO's actual lanes and RL observations are unchanged. Slow Monitor is disabled to isolate this cause; existing Fast Gate and automatic handback remain active.

Observed cloud timeline:

| Simulation time | Outcome |
|---|---|
| 0 | Both ASTA |
| 90 | TL1 enters FTSC; TL2 stays ASTA |
| 540 | TL1 starts its FTSC transition cycle |
| 630 | TL1 accepted back to ASTA |
| 720 | First complete returned ASTA cycle verifies handback |
| 900 | Test ends normally with steady all-red |

Those times assume healthy recovery evidence; the runner does not force release. PASS requires the matching missing-lane rejection, one TL1 contingency and completed handback, no TL2/Slow Monitor intervention, the expected mode sequence, all 910 timestamps, correct TL1 fixed programme, qualified recovery and normal STOP ACK. Missing or incorrect evidence fails the test. Result file: `seed_42/fault_test_result_GPT.json`, beside the usual metadata, decisions, events, observations, summary and display logs. Runtime failures write `run_error_GPT.txt`.

## 10. Useful Windows commands

Run from the project root with the venv active. Close Serial Monitor before Python accesses COM5. First ensure the board has the HIL sketch, not the temporary scanner. Choose unused output names; logs/run directories intentionally refuse overwrite, including after some failed attempts.

```cmd
cd /d "C:\Users\sihle\Desktop\HIL-Traffic-Supervisor-Design"
.venv\Scripts\activate
python bench_arduino_GPT.py --list-ports
```

Normal GUI/LED repeat; this example uses a new suffix:

```cmd
python run_supervised_GPT.py --controller rl --kind marl --algo ppo --model models\marl_ppo_ps_2int_260k.zip --name hil_marl_ppo_gui --seeds 1 --enable-slow-monitor --enable-auto-handback --hil-port COM5 --hil-speed 1 --gui --max-cycles 3 --run-dir supervisor_logs\hil_marl_ppo_gui_02_GPT
```

Controlled fault/handback test, about 15 minutes plus overhead:

```cmd
python run_hil_fault_test_GPT.py --port COM5 --gui --speed 1 --run-dir supervisor_logs\hil_fault_handback_01_GPT
```

The dedicated runner uses `--speed`; the normal runner uses `--hil-speed`. Both accept 1 or 3. The dedicated runner automatically enables handback and injects the fault.

Powered watchdog test, using a fresh log:

```cmd
python bench_arduino_GPT.py --port COM5 --watchdog-test --log supervisor_logs\bench_watchdog_03_GPT.jsonl
```

Keep USB connected during that watchdog test. Reset/start a new session afterward. If a run path already exists, choose a fresh suffix instead of deleting earlier evidence.

## 11. Earlier simulation evidence and retained limits

Four earlier full integrated runs used seed 42, peak scale 0.37 / 2664 vehicles/hour, identical recorded demand, both optional features enabled, and all ended in ASTA. All 9,031 vehicles were recorded inserted and arrived. These were simulation results, not physical HIL runs.

| Model | Slow contingencies TL1/TL2 | Verified handbacks TL1/TL2 | Peak sampled pending | Mean depart delay | Teleports | End time |
|---|---|---|---:|---:|---:|---:|
| MARL PPO | 0/0 | 0/0 | 90 | 28.61 s | 0 | 19001 s |
| SARL PPO | 0/0 | 0/0 | 91 | 43.15 s | 0 | 19002 s |
| MARL DQN | 2/2 | 2/2 | 276 | 264.25 s | 0 | 19283 s |
| SARL DQN | 1/0 | 1/0 | 772 | 930.83 s | 1 | 19401 s |

All had zero Fast Gate contingency entries in those full episodes. No injected fault occurred in the PPO full runs, so zero handbacks meant none were exercised. SARL DQN's collision-related teleport cause was not established; do not attribute it to handback or infer broad rankings from one seed. TL2 permissive-conflict/requested-green warnings are expected in some conditions. The unselected SARL PPO TL1 synthetic-fault case remained latched; a selected TL2 case recovered.

Tracked evidence includes `supervisor_logs/integrated_*_results_GPT.zip` and the individual `handback_*_results_GPT.zip` files. Generated `supervisor_logs/` output is otherwise ignored. Preserve the selected archives and avoid adding copied `sumo_work` trees.

Selected model SHA-256 identities, rechecked in this checkout:

| File | SHA-256 |
|---|---|
| `models/marl_ppo_ps_2int_260k.zip` | `a45080e45965a7fc99146c473631407cf6d3d1c5ee624bdc623679350ffcfe20` |
| `models/marl_dqn_ps_2int_260k.zip` | `c99792f0b5fa201fa292cdbf8104921d6482f391aedc1f5e3b0a23c76fd57298` |
| `models/sarl_dqn_2int_260k.zip` | `08d109c3acf6165c45ac258be2ae100c22523e94f1ce738edeb206184bef0e2d` |
| `models/agents_original/sarl_ppo_2int_260k.zip` | `f526e55a7762be45aaeb1a4b86ee5b963b8f6d0424e3958af2c2839ebf753d0f` |

Recorded full-run route SHA-256: `86fe8dc18d40a084d0873638e951032d34a4fa525c24aab460ef4444d192eb15`. MARL observation/action spaces are (9,)/Discrete(3); SARL uses (14,)/Discrete(6), with action `3 * junction_index + adjustment`. Preserve the selected 260k SARL models rather than substituting similarly named variants.

End of handoff. The next concrete input remains the two separate LCD address results.