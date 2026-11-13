# One LCD for TL1 and TL2

Prepared 6 November 2026. Both PCF8574 LCD adapters were scanned individually
and reported 0x27. The user confirmed that the single-LCD text test displays
both rows. This package is the next step: connect the two rows to the existing
HIL mode fields. The second LCD stays disconnected; no soldering is required.

## Files and installation

- `arduino/asta_display_GPT/asta_display_GPT.ino`: complete updated firmware.
- `bench_lcd_modes_GPT.py`: short synthetic LCD/serial test, without SUMO.
- `lcd_single_update_GPT.patch`: firmware changes against GitHub source blob
  `9ef15a507f83c5958aea71b9ce426427aa9e3030`.
- `tests/`: native C++ firmware regression harness with Arduino/LCD API mocks.
- `validation_GPT.json`: exact validation scope and file hashes.

Extract the archive in the project directory. Its top-level folder is
`hil_lcd_single_update_GPT`. Stop any Python HIL run and close Serial Monitor.
Back up the existing firmware outside its sketch directory, then copy the
two runtime files. These commands are for Windows Command Prompt, from
`C:\Users\sihle\Desktop\HIL-Traffic-Supervisor-Design`:

```bat
mkdir hil_lcd_backup_01_GPT
copy arduino\asta_display_GPT\asta_display_GPT.ino hil_lcd_backup_01_GPT\asta_display_GPT.ino.bak
copy hil_lcd_single_update_GPT\arduino\asta_display_GPT\asta_display_GPT.ino arduino\asta_display_GPT\asta_display_GPT.ino
copy hil_lcd_single_update_GPT\bench_lcd_modes_GPT.py bench_lcd_modes_GPT.py
```

Use a new backup directory if `hil_lcd_backup_01_GPT` already exists. Check
that the backup copy succeeded before replacing the firmware. Keep old copies
as `.bak` files outside the sketch folder; extra `.ino` files in one sketch
folder are compiled together. Do not copy `tests/native` into an Arduino
sketch folder: those headers are desktop test doubles, not Arduino libraries.

The `hd44780` library by Bill Perry used for the successful standalone text
test is also used here. Open the project file
`arduino/asta_display_GPT/asta_display_GPT.ino` in Arduino IDE, select Arduino
Uno and COM5, then Verify and Upload. Keep one LCD connected at 0x27:

| LCD | Uno |
|---|---|
| GND | GND |
| VCC | 5V |
| SDA | A4 |
| SCL | A5 |

Disconnect USB before changing wires. The 12 LED connections remain as before.
After uploading, both LCD rows should say WAITING. The original startup
flashing-red behavior remains until the serial handshake. Close Serial Monitor
before running Python.

## Short physical checks

Run the new bench from the project root with the existing virtual environment:

```bat
python bench_lcd_modes_GPT.py --port COM5 --log supervisor_logs\lcd_modes_01_GPT.jsonl
```

Allow about 35 seconds plus connection overhead. All four traffic heads stay
steady red during this synthetic test. Each stage lasts approximately four
seconds, with a fresh STATE every second so the active watchdog remains live.
The terminal prints the expected LCD text. Check this sequence:

| Stage | First row | Second row |
|---|---|---|
| Startup pause | TL1: PAUSED | TL2: PAUSED |
| Both adaptive | TL1: ASTA | TL2: ASTA |
| TL1 fallback | TL1: FTSC | TL2: ASTA |
| TL1 transition | TL1: FTSC TRANS | TL2: ASTA |
| TL1 released | TL1: ASTA | TL2: ASTA |
| TL2 fallback | TL1: ASTA | TL2: FTSC |
| TL2 transition | TL1: ASTA | TL2: FTSC TRANS |
| Both adaptive again | TL1: ASTA | TL2: ASTA |
| Pause | TL1: PAUSED | TL2: PAUSED |
| Normal stop | TL1: STOPPED | TL2: STOPPED |

These are deliberately generated test modes, not decisions made by SUMO or the
supervisor. The script checks serial/GPIO acknowledgements, logs expected LCD
rows, and explicitly leaves visual verification false. Text visibility is a
manual check. Use a new log filename for another run; existing logs are kept.

After the mode bench, check the unchanged 3-second freshness watchdog with
the existing bench script:

```bat
python bench_arduino_GPT.py --port COM5 --watchdog-test --log supervisor_logs\lcd_watchdog_01_GPT.jsonl
```

This deliberately stops sending commands after an active STATE. Expect both
rows to change from FTSC to FAULT and all four red LEDs to flash. The fault is
latched until reset. The original bench verifies the firmware WATCHDOG event;
visually check the LEDs and LCD. A new Python session resets the Uno using DTR.

Once these short checks pass, use the existing supervised/SUMO command. No host
controller, simulator, policy, threshold, or protocol changes are required.
The previously passed ten-cycle fault/handback experiment remains valid for
its recorded firmware. Re-running it with this firmware would add new LCD
integration evidence, rather than repair a failed old test. Its existing
software verifier does not verify optical LCD text. To add that new evidence:

```bat
python run_hil_fault_test_GPT.py --port COM5 --gui --speed 1 --run-dir supervisor_logs\hil_fault_lcd_01_GPT
```

In that scenario expect TL1 FTSC at simulation t=90, FTSC TRANS at t=540,
and ASTA on release at t=630; TL2 stays ASTA. Handback completion is verified
later after a complete returned cycle. LCD ownership changes on the accepted
mode-0 frame, not on that later verification event. Confirm this against the
actual new run's terminal output; the LCD mirrors received modes.

## Design decisions

1. One 16x2 LCD at 0x27, first row TL1 and second row TL2.
2. Existing STATE fields m1 and m2 drive the labels: 0 = ASTA, 1 = FTSC,
   2 = FTSC TRANS. During transition, FTSC remains the controlling program.
3. WAITING, READY, PAUSED, STOPPED and FAULT replace owner labels when applicable.
   Invalid/stale/duplicate commands do not change displayed modes.
4. GPIO application/readback and its ACK happen before any LCD I2C work for
   that command. LCD text is queued; identical modes do not repaint every second.
5. A repaint hides the text, writes padded rows, and shows the completed pair.
   Each main-loop pass performs at most one LCD API operation. New labels
   replace pending labels; partial words or a pair of rows from different
   accepted states are not shown as a completed screen. A brief blank during
   a mode change is expected.
6. Serial input and protocol timers get priority. The firmware sets a 3000-us
   Wire timeout with bus reset on timeout, avoids starting LCD operations in
   the last 50 ms before watchdog expiry, and checks timers before and after
   the LCD service call. No `WIRE_HAS_TIMEOUT` macro test is used.
7. LCD initialization happens before protocol READY. An absent LCD, failed
   write, or timeout disables LCD servicing until reset. It does not alter
   traffic control or create a new supervisor fallback rule. There are no
   unframed LCD debug prints on the protocol port.

The LCD is an auxiliary display. If its bus or power fails, it can go blank or
retain stale text; this firmware cannot guarantee clearing an unresponsive
screen. GPIO ACK confirms the GPIO mask, not LCD pixels or optical latency.
Repair/reset the display before relying on its labels. No hard physical timing
bound is claimed from the desktop tests.

## Validation and limits

The actual updated `.ino` was compiled with desktop g++ against API mocks,
with warnings treated as errors, AddressSanitizer and UndefinedBehaviorSanitizer.
Leak detection was disabled because the container lacks the /proc access that
LeakSanitizer requires. All 16 scenario processes passed: startup; every pair
of mode codes; all 36 phase pairs and masks; duplicate and invalid commands;
watchdog expiry and millisecond wraparound; host and GPIO faults; absent LCD;
LCD initialization timeout; write error and timeout; newer-mode/fault arrival
during repaint; serial priority; and the watchdog guard interval.

The real library source was reviewed to confirm constructor/method signatures,
status handling and per-character I2C writes. These native tests do not compile
the actual AVR core or LCD library and do not model electrical I2C behavior.
Arduino IDE Verify/Upload and the physical checks above are still required.
The three-sample timing evidence from the earlier firmware is not a timing
measurement of this new LCD firmware.

To reproduce the native harness on a machine with Python and g++:

```text
python tests/run_native_GPT.py
```

To restore the earlier firmware, copy the backed-up `.ino.bak` over
`arduino/asta_display_GPT/asta_display_GPT.ino`, then upload that sketch again.
No host runtime files need rollback: the new bench is standalone and the
existing transport/controller files were not modified.
