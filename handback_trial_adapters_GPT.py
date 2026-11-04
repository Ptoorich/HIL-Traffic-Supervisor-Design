"""Strict policy observations, synthetic snapshot faults and SUMO readback.

These adapters are used only by the opt-in simulation runner.
"""
from __future__ import annotations

from dataclasses import replace
import math
import operator

import numpy as np


def _read_time(traci_mod, expected_time):
    value = float(traci_mod.simulation.getTime())
    if not math.isfinite(value) or not math.isclose(value, float(expected_time), abs_tol=1e-6):
        raise ValueError(f'Unexpected or stale simulation time {value!r}; expected {expected_time!r}')
    return value


def strict_observations(env, traci_mod, expected_time):
    """Rebuild the repository's policy observations without its zero-on-error fallback."""
    agents = list(env.agent_ids)
    errors, observations = {}, {}
    try:
        _read_time(traci_mod, expected_time)
        low, high = float(env.s_lb), float(env.s_ub)
        if not math.isfinite(low) or not math.isfinite(high) or high <= low:
            raise ValueError('Invalid split bounds')
    except Exception as exc:
        msg = f'{type(exc).__name__}: {exc}'
        return {}, {agent: msg for agent in agents}
    for agent in agents:
        try:
            queues = []
            for lane in env.lane_map[agent]:
                raw = traci_mod.lane.getLastStepHaltingNumber(lane)
                if isinstance(raw, (bool, np.bool_)):
                    raise ValueError(f'Boolean queue for {lane}')
                q = operator.index(raw)
                if q < 0:
                    raise ValueError(f'Negative queue for {lane}')
                queues.append(min(q / 50.0, 1.0))
            split = float(env.phase_splits[agent])
            identity = float(env.agent_index[agent])
            if not math.isfinite(split) or not low <= split <= high:
                raise ValueError(f'Invalid actual split {split!r}')
            if not math.isfinite(identity):
                raise ValueError('Invalid agent index')
            values = np.array(queues + [(split - low) / (high - low)], dtype=np.float32)
            if len(values) > env.max_local_obs:
                raise ValueError('Observation exceeds expected local size')
            if len(values) < env.max_local_obs:
                values = np.pad(values, (0, env.max_local_obs - len(values)))
            values = np.concatenate([values, np.array([identity], dtype=np.float32)])
            if values.shape != (env.obs_size,) or not np.isfinite(values).all():
                raise ValueError('Invalid observation shape or contents')
            observations[agent] = values
        except Exception as exc:
            errors[agent] = f'{type(exc).__name__}: {exc}'
    try:
        _read_time(traci_mod, expected_time)
    except Exception as exc:
        msg = f'{type(exc).__name__}: {exc}'
        return {}, {agent: msg for agent in agents}
    return observations, errors


class StrictPolicy:
    """Fresh validated observations; invalid data become invalid actions, not zero arrays."""
    def __init__(self, kind, model, traci_mod, demo_agent='tl1'):
        self.kind, self.model, self.traci = kind, model, traci_mod
        self.demo_agent = demo_agent
        self.name = f'handback_{kind}'
        self.last_observation_errors = {}
        self.last_selected_agent = None
        self.last_joint_action = None

    def act(self, env, ignored_obs, t):
        self.last_selected_agent = self.last_joint_action = None
        observations, errors = strict_observations(env, self.traci, t)
        self.last_observation_errors = dict(errors)
        agents = list(env.agent_ids)
        actions = {a: -1 for a in agents}
        if self.kind == 'sarl':
            if errors or len(observations) != len(agents):
                return actions
            try:
                queues, splits = [], []
                for agent in agents:
                    n = len(env.lane_map[agent])
                    queues.extend(observations[agent][:n])
                    splits.append(observations[agent][n])
                obs = np.array(queues + splits, dtype=np.float32)
                joint = int(self.model.predict(obs, deterministic=True)[0])
                if joint < 0 or joint >= 3 * len(agents):
                    raise ValueError(f'Invalid SARL action {joint}')
                selected = agents[joint // 3]
                actions = {a: 1 for a in agents}
                actions[selected] = joint % 3
                self.last_joint_action, self.last_selected_agent = joint, selected
            except Exception as exc:
                self.last_observation_errors = {a: f'POLICY_ERROR: {type(exc).__name__}: {exc}' for a in agents}
            return actions
        for agent in agents:
            if agent in errors:
                continue
            try:
                if self.kind == 'demo':
                    action = 2 if agent == self.demo_agent else 1
                else:
                    action = int(self.model.predict(observations[agent], deterministic=True)[0])
                if action not in (0, 1, 2):
                    raise ValueError(f'Invalid action {action}')
                actions[agent] = action
            except Exception as exc:
                self.last_observation_errors[agent] = f'POLICY_ERROR: {type(exc).__name__}: {exc}'
        return actions


class InjectedSnapshotTaker:
    """Remove exactly one lane from one otherwise real frozen Fast Gate snapshot."""
    def __init__(self, delegate):
        self.delegate = delegate
        self.pending = None
        self.last_injected = None

    def clear_cache(self):
        self.delegate.clear_cache()

    def arm(self, agent, lane):
        if self.pending is not None:
            raise RuntimeError('A fault is already armed')
        self.pending = (agent, lane)

    def take(self, env, current_splits=None):
        snapshots = self.delegate.take(env, current_splits=current_splits)
        self.last_injected = None
        if self.pending is None:
            return snapshots
        agent, lane = self.pending
        self.pending = None
        snap = snapshots[agent]
        retained = tuple(x for x in snap.lanes if x.lane_id != lane)
        if len(retained) != len(snap.lanes) - 1:
            raise ValueError(f'Injection lane {lane!r} was not present exactly once')
        result = dict(snapshots)
        result[agent] = replace(snap, lanes=retained)
        self.last_injected = {'agent': agent, 'lane': lane, 'sim_time': snap.sim_time}
        return result


def read_active_programs(env, traci_mod):
    """Read and verify active six-phase logic against the split held by the environment."""
    if not env.allred_phases or env.allred_time <= 0:
        raise ValueError('This trial requires the six-phase all-red environment')
    output = {}
    for agent in env.agent_ids:
        tl_id = env.tl_map[agent]
        active = str(traci_mod.trafficlight.getProgram(tl_id))
        matches = [x for x in traci_mod.trafficlight.getAllProgramLogics(tl_id)
                   if str(x.programID) == active]
        if len(matches) != 1:
            raise ValueError(f'{agent}: cannot uniquely find active program {active!r}')
        phases = list(matches[0].phases)
        if len(phases) != 6:
            raise ValueError(f'{agent}: expected 6 phases, found {len(phases)}')
        split = float(env.phase_splits[agent])
        yellow, red = float(env.yellow_time), float(env.allred_time)
        b_green = float(env.cycle_length) - split - 2 * (yellow + red)
        if not all(math.isfinite(x) for x in (split, yellow, red, b_green)):
            raise ValueError(f'{agent}: non-finite expected durations')
        expected_durations = [split, yellow, red, b_green, yellow, red]
        ga, ya, gb, yb = env._base_states[agent]
        expected_states = [ga, ya, 'r' * len(ga), gb, yb, 'r' * len(ga)]
        actual_durations = [float(p.duration) for p in phases]
        actual_states = [str(p.state) for p in phases]
        if not all(math.isclose(x, y, rel_tol=0, abs_tol=1e-6)
                   for x, y in zip(actual_durations, expected_durations)):
            raise ValueError(f'{agent}: active durations {actual_durations} != expected {expected_durations}')
        if actual_states != expected_states:
            raise ValueError(f'{agent}: active signal phase states do not match the network fixture')
        output[agent] = {'program_id': active, 'durations': actual_durations,
                         'green_a': actual_durations[0], 'green_b': actual_durations[3],
                         'states_match': True}
    return output
