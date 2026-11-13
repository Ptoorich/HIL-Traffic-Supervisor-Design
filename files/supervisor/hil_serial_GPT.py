"""Bounded, acknowledged USB transport for the UNO logical-phase display."""
from __future__ import annotations

import binascii
import json
from pathlib import Path
import secrets
import time

ALL_RED = 2340
PHASE_MASKS = (33, 34, 36, 12, 20, 36)
MAX_FRAME = 160
UINT32_MAX = 2**32 - 1


class DisplayError(RuntimeError):
    pass


def encode_frame(*fields):
    payload = ','.join(str(x) for x in fields).encode('ascii')
    frame = payload + f'*{binascii.crc_hqx(payload, 0xFFFF):04X}\n'.encode('ascii')
    if len(frame) > MAX_FRAME:
        raise DisplayError('Serial frame exceeds 160 bytes')
    return frame


def decode_frame(frame):
    try:
        if len(frame) > MAX_FRAME or not frame.endswith(b'\n'):
            raise ValueError('missing LF or oversize frame')
        payload, crc = frame[:-1].split(b'*')
        if len(crc) != 4 or any(c not in b'0123456789ABCDEF' for c in crc):
            raise ValueError('invalid CRC spelling')
        if binascii.crc_hqx(payload, 0xFFFF) != int(crc, 16):
            raise ValueError('CRC mismatch')
        fields = payload.decode('ascii').split(',')
        if fields[0] != '1':
            raise ValueError('unsupported protocol version')
        return fields
    except (ValueError, UnicodeError) as exc:
        raise DisplayError(f'Invalid serial frame: {exc}') from exc


def uint32(value):
    if type(value) is not int or not 0 <= value <= UINT32_MAX:
        raise DisplayError(f'Expected uint32, got {value!r}')
    return value


class SerialDisplay:
    """One outstanding transaction; failure terminates this session, never reconnects."""
    def __init__(self, port, *, log_path, serial_instance=None, ack_timeout=0.5,
                 ready_timeout=8.0, clock=time.monotonic, sleep=time.sleep):
        if not 0 < ack_timeout <= 0.5 or ready_timeout <= 0:
            raise ValueError('Invalid serial deadline')
        self.port, self.log_path = port, Path(log_path)
        self.serial = serial_instance
        self.clock, self.sleep = clock, sleep
        self.ack_timeout, self.ready_timeout = ack_timeout, ready_timeout
        self.session = secrets.randbelow(UINT32_MAX) + 1
        self.sequence = 0
        self.log = None
        self.buffer = bytearray()
        self.failed = False
        self.started = False
        self.closed = False

    def record(self, event, **fields):
        if self.log is not None:
            self.log.write(json.dumps({'event': event, 'host_monotonic': self.clock(),
                'session': self.session, **fields}, allow_nan=False) + '\n')
            self.log.flush()

    def _line(self, deadline):
        while self.clock() < deadline:
            byte = self.serial.read(1)
            if not byte:
                continue
            self.buffer.extend(byte)
            if len(self.buffer) > MAX_FRAME:
                raise DisplayError('Oversize board response')
            if byte == b'\n':
                line = bytes(self.buffer)
                self.buffer.clear()
                return decode_frame(line)
        return None

    def __enter__(self):
        try:
            self.log = self.log_path.open('x', encoding='utf-8')
            if self.serial is None:
                import serial
                self.serial = serial.Serial(self.port, 115200, timeout=0.01, write_timeout=0.1)
                # Uno USB DTR reset. Drain prior traffic BEFORE releasing reset.
                self.serial.dtr = False
                self.sleep(0.05)
                self.serial.reset_input_buffer()
                self.serial.dtr = True
            ready = self._line(self.clock() + self.ready_timeout)
            if ready is None or len(ready) != 3 or ready[:2] != ['1', 'R']:
                raise DisplayError('No protocol-v1 READY; reset Uno and start a new run')
            boot = uint32(int(ready[2]))
            self.record('ready', boot=boot, port=self.port)
            self._exchange(encode_frame(1, 'H', self.session, boot), 'H', ALL_RED)
            self.started = True
            self.pause()  # SUMO startup may take seconds; active freshness begins with STATE.
            return self
        except BaseException:
            self._close()
            raise

    def _exchange(self, frame, kind, expected_mask):
        sent = self.clock()
        deadline = sent + self.ack_timeout
        for attempt in range(2):
            if self.clock() >= deadline:
                break
            if self.serial.write(frame) != len(frame):
                raise DisplayError('Incomplete serial write')
            until = sent + self.ack_timeout / 2 if attempt == 0 else deadline
            while self.clock() < until:
                fields = self._line(until)
                if fields is None:
                    break
                if fields[1:2] == ['R']:
                    raise DisplayError('Arduino reset during session')
                if len(fields) >= 3 and fields[2] != str(self.session):
                    continue
                if fields[1:2] in (['E'], ['N']):
                    raise DisplayError('Arduino rejected state: ' + ','.join(fields))
                if len(fields) != 6 or fields[:2] != ['1', 'A']:
                    raise DisplayError('Malformed ACK')
                if fields[3] != str(self.sequence):
                    continue
                if fields[4:] != [str(expected_mask), kind]:
                    raise DisplayError('ACK output mask/type differs from command')
                ack_received_at = self.clock()
                if ack_received_at >= deadline:
                    raise DisplayError('ACK arrived after deadline')
                receipt = {'session': self.session, 'sequence': self.sequence,
                           'kind': kind, 'mask': expected_mask, 'sent_at': sent,
                           'ack_received_at': ack_received_at,
                           'rtt_ms': (ack_received_at - sent) * 1000, 'attempts': attempt + 1}
                self.record('ack', **receipt)
                return receipt
        raise DisplayError('Arduino ACK deadline exceeded (500 ms maximum)')

    def _command(self, kind, values=(), mask=ALL_RED):
        if not self.started or self.failed or self.closed:
            raise DisplayError('Display session is not active')
        try:
            self.sequence = uint32(self.sequence + 1)
            return self._exchange(encode_frame(1, kind, self.session, self.sequence, *values), kind, mask)
        except BaseException as exc:
            self.fault(exc)
            raise

    def state(self, sim_ms, phases, modes):
        uint32(sim_ms)
        if (len(phases) != 2 or len(modes) != 2 or
            any(type(p) is not int or p not in range(6) for p in phases) or
            any(type(m) is not int or m not in range(3) for m in modes)):
            raise DisplayError('Expected two phase codes 0..5 and mode codes 0..2')
        mask = PHASE_MASKS[phases[0]] | PHASE_MASKS[phases[1]] << 6
        return self._command('S', (sim_ms, *phases, *modes), mask)

    def pause(self):
        self._command('P')

    def stop(self):
        self._command('X')
        self.closed = True

    def verify_watchdog(self):
        """Bench test: deliberately send nothing and require the firmware timeout event."""
        deadline = self.clock() + 4.0
        expected = ['1', 'E', str(self.session), str(self.sequence), 'WATCHDOG']
        while self.clock() < deadline:
            response = self._line(deadline)
            if response == expected:
                self.failed = True  # Preserve flashing red on exit; do not issue STOP.
                self.record('watchdog_verified', evidence='firmware report; visually check flashing LEDs')
                return
            if response and response[1:2] == ['A'] and response[2:3] == [str(self.session)]:
                continue  # A delayed duplicate ACK is not the watchdog outcome.
            raise DisplayError(f'Expected watchdog report, received {response!r}')
        raise DisplayError('Missing watchdog report')

    def fault(self, reason):
        if self.failed:
            return
        self.failed = True
        try:
            if self.serial is not None and self.started and self.sequence < UINT32_MAX:
                self.serial.write(encode_frame(1, 'F', self.session, self.sequence + 1))
        except Exception:
            pass  # The board's own freshness watchdog remains the fallback.
        try:
            self.record('fault', reason=str(reason))
        except Exception:
            pass

    def _close(self):
        try:
            if self.serial is not None:
                self.serial.close()
        finally:
            if self.log is not None:
                self.log.close()

    def __exit__(self, typ, value, tb):
        try:
            if typ is not None:
                self.fault(value)
            elif not self.failed and not self.closed:
                try:
                    self.stop()
                except BaseException as exc:
                    self.fault(exc)
                    raise
        finally:
            self._close()
