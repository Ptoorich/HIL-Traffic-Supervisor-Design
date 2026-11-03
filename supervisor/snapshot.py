"""
One TraCI snapshot per decision, taken in the MAIN thread (L3).

TraCI is a single socket and not thread-safe, so every raw reading the Fast
Gate needs is collected here, frozen into immutable dataclasses, and only then
handed to worker threads. The calibration logger uses this SAME class, so
calibration data and runtime data come from identical code.

Conventions verified on SUMO 1.28 (toy net):
  * getLastStepOccupancy returns a FRACTION 0..1 (docs say percent). The unit
    is a config switch, normalised to a fraction here.
  * getLastStepMeanSpeed returns the lane's SPEED LIMIT for an empty lane.
  * Occupancy counts vehicle length only (5 m of a 7.5 m footprint), so a
    fully jammed lane reads about 0.64-0.67, never 1.0.
"""
from __future__ import annotations

from .config import SupervisorConfig
from .datatypes import JunctionSnapshot, LaneReading


class SnapshotTaker:
    def __init__(self, cfg: SupervisorConfig, traci_mod=None):
        if traci_mod is None:
            import traci as traci_mod  # imported lazily so tests need no SUMO
        self._traci = traci_mod
        self._cfg = cfg
        self._static: dict[str, tuple[float, float]] = {}  # lane -> (length, max_speed)

    def clear_cache(self) -> None:
        self._static.clear()

    def _lane_static(self, lane_id: str) -> tuple[float, float]:
        if lane_id not in self._static:
            lane = self._traci.lane
            self._static[lane_id] = (float(lane.getLength(lane_id)),
                                     float(lane.getMaxSpeed(lane_id)))
        return self._static[lane_id]

    def take(self, env, current_splits: dict | None = None) -> dict:
        """
        Returns {agent: JunctionSnapshot}. Raises on any TraCI failure; the
        caller treats that as "no trustworthy data" and fails safe.

        current_splits: the splits in force BEFORE the inner controller acted.
        StaticController.act() calls env.set_splits() itself, so reading
        env.phase_splits after act() would already contain the proposal.
        """
        cfg, T = self._cfg, self._traci
        splits = dict(env.phase_splits if current_splits is None else current_splits)
        sim_time = float(T.simulation.getTime())
        scale = 0.01 if cfg.snapshot.occupancy_unit == "percent" else 1.0
        base_states = getattr(env, "_base_states", {})

        out = {}
        for agent in cfg.agents:
            readings = []
            for lc in cfg.lanes_of(agent):
                length, vmax = self._lane_static(lc.lane_id)
                lane = T.lane
                readings.append(LaneReading(
                    lane_id=lc.lane_id,
                    halting=int(lane.getLastStepHaltingNumber(lc.lane_id)),
                    count=int(lane.getLastStepVehicleNumber(lc.lane_id)),
                    occupancy=float(lane.getLastStepOccupancy(lc.lane_id)) * scale,
                    mean_speed=float(lane.getLastStepMeanSpeed(lc.lane_id)),
                    length=length,
                    max_speed=vmax,
                    capacity=lc.capacity,
                    excluded=lc.excluded,
                ))
            tl_id = env.tl_map[agent]
            out[agent] = JunctionSnapshot(
                agent=agent,
                tl_id=tl_id,
                sim_time=sim_time,
                lanes=tuple(readings),
                live_state=str(T.trafficlight.getRedYellowGreenState(tl_id)),
                commanded_phase_strings=tuple(base_states.get(agent, ())),
                current_split=float(splits[agent]),
            )
        return out
