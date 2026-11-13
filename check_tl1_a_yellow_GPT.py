from datetime import datetime
from pathlib import Path
import time

from supervisor.hil_serial_GPT import SerialDisplay


def main():
    folder = Path("supervisor_logs")
    folder.mkdir(exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    log = folder / f"tl1_a_yellow_{stamp}_GPT.jsonl"

    with SerialDisplay("COM5", log_path=log) as display:
        # TL1: A yellow, B red. TL2: both heads red.
        display.state(0, [1, 2], [0, 0])
        print("TL1 A YELLOW (pin D3) should now be ON.")
        print("The other three heads should show RED.")

        # Keep sending fresh commands during the ten-second hold.
        for tick in range(1, 21):
            time.sleep(0.5)
            display.state(tick * 500, [1, 2], [0, 0])

    print("Finished. All four heads should now show steady RED.")


if __name__ == "__main__":
    main()