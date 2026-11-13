"""Compile and run the real .ino against host API mocks. Requires g++ (C++17).

This is firmware logic validation, not an AVR build or a hardware timing test.
"""
from pathlib import Path
import os
import subprocess
import tempfile

SCENARIOS = (
    "startup", "modes", "phases", "duplicate", "invalid", "watchdog", "wrap",
    "host_fault", "gpio_fault", "lcd_absent", "lcd_init_timeout", "lcd_error",
    "lcd_timeout", "supersede", "serial_priority", "watchdog_guard",
)


def main():
    native = Path(__file__).resolve().parent / "native"
    with tempfile.TemporaryDirectory(prefix="lcd_firmware_") as temp:
        exe = Path(temp) / "firmware_test"
        subprocess.run(["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
                        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                        "-g", "-I", str(native), str(native / "test_firmware.cpp"),
                        "-o", str(exe)], check=True)
        for scenario in SCENARIOS:
            # LeakSanitizer requires /proc thread access unavailable in some
            # containers. Address and undefined-behavior checks remain enabled.
            subprocess.run([str(exe), scenario], check=True,
                           env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
    print(f"{len(SCENARIOS)} native firmware scenarios passed (API mocks).")


if __name__ == "__main__":
    main()
