"""Main-thread SUMO readback and pacing; the Arduino never schedules phases."""
from __future__ import annotations

import math
import time

from .hil_serial_GPT import DisplayError
from .observation_adapter_GPT import read_active_programs


class PhaseDisplay:
    def __init__(self, transport, *, speed=1.0, clock=None, sleep=time.sleep):
        if speed not in (1.0, 3.0):
            raise ValueError('HIL speed must be 1 or 3')
        self.transport, self.speed = transport, speed
        # End-to-end timestamps must share the transport's monotonic clock.
        self.clock, self.sleep = transport.clock if clock is None else clock, sleep
        self.last_time = None
        self.last_wall = None
        self.step_lag_ms = 0.0
        self.step_sleep_requested_ms = self.step_sleep_actual_ms = 0.0
        self.last_sample = None

    def prepare_step(self):
        """Hold the current GUI/LED state together before advancing SUMO."""
        self.step_lag_ms = 0.0
        self.step_sleep_requested_ms = self.step_sleep_actual_ms = 0.0
        if self.last_wall is not None:
            target = self.last_wall + 1.0 / self.speed
            before_sleep = self.clock()
            wait = max(0.0, target - before_sleep)
            self.sleep(wait)
            after_sleep = self.clock()
            self.step_sleep_requested_ms = wait * 1000
            self.step_sleep_actual_ms = (after_sleep - before_sleep) * 1000
            self.step_lag_ms = max(0.0, after_sleep - target) * 1000

    def sample(self, env, traci_mod, modes, transitions=()):
        if tuple(env.agent_ids) != ('tl1', 'tl2'):
            raise DisplayError('HIL requires tl1, tl2 in that order')
        sampled_at = self.clock()
        sim = float(traci_mod.simulation.getTime())
        if not math.isfinite(sim) or sim < 0 or not math.isclose(sim * 1000, round(sim * 1000)):
            raise DisplayError('Invalid SUMO display timestamp')
        if self.last_time is not None and sim not in (self.last_time, self.last_time + 1):
            raise DisplayError('Display missed a simulated second')
        programs = read_active_programs(env, traci_mod)
        phases, mode_codes = [], []
        for agent in env.agent_ids:
            tl = env.tl_map[agent]
            phase = traci_mod.trafficlight.getPhase(tl)
            if type(phase) is not int or phase not in range(6):
                raise DisplayError(f'{agent}: invalid live phase')
            ga, ya, gb, yb = env._base_states[agent]
            states = (ga, ya, 'r' * len(ga), gb, yb, 'r' * len(ga))
            if traci_mod.trafficlight.getRedYellowGreenState(tl) != states[phase]:
                raise DisplayError(f'{agent}: live state does not match validated program')
            if modes[agent] not in ('ASTA', 'CONTINGENCY'):
                raise DisplayError('Unknown control mode')
            phases.append(phase)
            mode_codes.append(2 if agent in transitions else int(modes[agent] == 'CONTINGENCY'))
        if float(traci_mod.simulation.getTime()) != sim:
            raise DisplayError('SUMO time changed during readback')
        # Send readback immediately after the step; pacing occurs in prepare_step().
        receipt = self.transport.state(round(sim * 1000), phases, mode_codes)
        self.last_time, self.last_wall = sim, self.clock()
        self.last_sample = {'sim_time': sim, 'sampled_at': sampled_at,
                            'phases': phases, 'modes': mode_codes, 'ack': receipt}
        self.transport.record('sample', sim_time=sim, phases=phases, modes=mode_codes,
                              programs=programs, sampled_at=sampled_at,
                              ack_sequence=receipt['sequence'],
                              ack_received_at=receipt['ack_received_at'],
                              lag_ms=self.step_lag_ms,
                              observation_to_ack_ms=(self.clock() - sampled_at) * 1000)
        return self.last_sample
