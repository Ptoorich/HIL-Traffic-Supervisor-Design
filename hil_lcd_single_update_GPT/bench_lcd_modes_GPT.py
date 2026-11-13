"""Short, synthetic LCD/serial bench; all twelve traffic outputs stay all-red.

Uses the existing HIL transport, without SUMO or policy inference. Mode labels
are deliberately supplied by this test; they are not supervisor decisions.
"""
import argparse
from pathlib import Path
import time

from supervisor.hil_serial_GPT import SerialDisplay


STAGES = (
    ("Both ASTA", (0, 0), ("TL1: ASTA", "TL2: ASTA")),
    ("TL1 fallback", (1, 0), ("TL1: FTSC", "TL2: ASTA")),
    ("TL1 transition (FTSC still owns control)", (2, 0),
     ("TL1: FTSC TRANS", "TL2: ASTA")),
    ("TL1 released to ASTA", (0, 0), ("TL1: ASTA", "TL2: ASTA")),
    ("TL2 fallback", (0, 1), ("TL1: ASTA", "TL2: FTSC")),
    ("TL2 transition (FTSC still owns control)", (0, 2),
     ("TL1: ASTA", "TL2: FTSC TRANS")),
    ("Both ASTA again", (0, 0), ("TL1: ASTA", "TL2: ASTA")),
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--log", type=Path,
                        default=Path("supervisor_logs/lcd_modes_01_GPT.jsonl"))
    args = parser.parse_args(argv)
    if args.log.exists():
        parser.error("Log already exists; choose a new --log path.")
    args.log.parent.mkdir(parents=True, exist_ok=True)
    print("Synthetic mode test: all four red LEDs must stay ON.", flush=True)
    sim_ms = 0
    with SerialDisplay(args.port, log_path=args.log) as display:
        display.record("lcd_expected", source="synthetic_bench", labels=["PAUSED", "PAUSED"],
                       visual_verified=False)
        print("Startup: both rows PAUSED (3 seconds).", flush=True)
        time.sleep(3)
        for name, modes, rows in STAGES:
            print(f"{name}: {rows[0]} | {rows[1]}", flush=True)
            display.record("lcd_expected", source="synthetic_bench", labels=list(rows),
                           modes=list(modes), sim_ms=sim_ms, visual_verified=False)
            for _ in range(4):
                display.state(sim_ms, (2, 2), modes)
                sim_ms += 1000
                time.sleep(1)
        display.pause()
        display.record("lcd_expected", source="synthetic_bench", labels=["PAUSED", "PAUSED"],
                       visual_verified=False)
        print("Pause: both rows PAUSED (3 seconds).", flush=True)
        time.sleep(3)
        display.stop()
        display.record("lcd_expected", source="synthetic_bench", labels=["STOPPED", "STOPPED"],
                       visual_verified=False)
        print("Stop: both rows STOPPED; all four reds stay ON.", flush=True)
        time.sleep(1)
    print("Serial/GPIO exchanges succeeded. LCD text still needs your visual check.", flush=True)
    print(f"Log: {args.log.resolve()}")


if __name__ == "__main__":
    main()
