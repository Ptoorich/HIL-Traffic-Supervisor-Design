"""Bench exercise for the UNO display, without SUMO. No physical LED feedback is assumed."""
import argparse
from pathlib import Path
import time

from supervisor.hil_serial_GPT import SerialDisplay


def phase_at(second, a):
    t = second % 90
    elapsed = 0
    for phase, duration in enumerate((a, 3, 3, 78-a, 3, 3)):
        elapsed += duration
        if t < elapsed:
            return phase
    raise ValueError('Invalid cycle time')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list-ports', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--speed', type=int, choices=(1, 3), default=1)
    parser.add_argument('--cycles', type=int, default=2)
    parser.add_argument('--watchdog-test', action='store_true')
    parser.add_argument('--log', type=Path, default=Path('supervisor_logs/arduino_bench_GPT.jsonl'))
    args = parser.parse_args()
    if args.list_ports:
        from serial.tools.list_ports import comports
        for port in comports():
            print(f'{port.device}: {port.description}')
        return
    if not args.port:
        parser.error('--port is required unless listing ports')
    if args.cycles < 1:
        parser.error('--cycles must be positive')
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with SerialDisplay(args.port, log_path=args.log) as display:
        if args.watchdog_test:
            display.state(0, [0, 3], [1, 1])
            print('Stopping commands; leave USB connected. Expect flashing red after about 3 seconds.')
            display.verify_watchdog()
            print('Firmware reported watchdog expiry. Confirm all four red LEDs flash; reset for next run.')
            return
        print('Synthetic bench sequence: TL1 31/47 s, TL2 59/19 s; yellow/all-red 3 s each.')
        for second in range(args.cycles * 90):
            display.state(second*1000, [phase_at(second, 31), phase_at(second, 59)], [1, 1])
            time.sleep(1 / args.speed)
        print('Sequence acknowledged. Ending in steady all-red; visually verify the lamp mapping.')


if __name__ == '__main__':
    main()
