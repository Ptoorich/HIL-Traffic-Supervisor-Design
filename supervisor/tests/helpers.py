import dataclasses

from supervisor.datatypes import JunctionSnapshot, LaneReading, Proposal

SPEED_LIMIT = 13.89
COMMANDED = ("GGrr", "yyrr", "rrGG", "rryy")


def healthy_snapshot(cfg, agent, split=39.0, **overrides):
    """All lanes empty (SUMO reports the speed limit for an empty lane)."""
    lanes = tuple(LaneReading(
        lane_id=l.lane_id, halting=0, count=0, occupancy=0.0,
        mean_speed=SPEED_LIMIT, length=l.capacity * 7.5 + 2.0,
        max_speed=SPEED_LIMIT, capacity=l.capacity, excluded=l.excluded)
        for l in cfg.lanes_of(agent))
    snap = JunctionSnapshot(agent, f"tl_{agent}", 900.0, lanes, "rrrr",
                            COMMANDED, split)
    return dataclasses.replace(snap, **overrides) if overrides else snap


def with_lane(snap, lane_id, **fields):
    lanes = tuple(dataclasses.replace(r, **fields) if r.lane_id == lane_id else r
                  for r in snap.lanes)
    return dataclasses.replace(snap, lanes=lanes)


def jammed(snap, lane_id):
    """Fully jammed lane as SUMO reports it: all halted, speed 0, occupancy at jam."""
    r = next(x for x in snap.lanes if x.lane_id == lane_id)
    return with_lane(snap, lane_id, halting=r.capacity, count=r.capacity,
                     mean_speed=0.0, occupancy=r.capacity * 5.0 / r.length)


def proposal(agent="tl1", current=39.0, delta=0):
    return Proposal(agent, current, delta, current + delta, "asta")
