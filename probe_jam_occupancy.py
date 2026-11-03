"""
Probe the sensor conventions the supervisor config depends on, on YOUR SUMO
version (the supervisor was developed against 1.28; you run 1.27.1).

Same idea as network_capacity_test.py (all signals forced red for 1500 s so
the lanes pack), but headless and it also records occupancy and speed:

    py probe_jam_occupancy.py

Prints per lane: capacity, max vehicle count, max RAW occupancy as TraCI
returns it, the expected jammed occupancy, and the mean speed with a full lane.
It also shows what an EMPTY lane reports as mean speed.

What to check:
  * max raw occupancy <= 1.0  -> TraCI returns a fraction; keep
    snapshot.occupancy_unit = "fraction". If it reaches ~60-67 it is percent;
    set occupancy_unit = "percent".
  * max raw occupancy is far below 90% either way: vehicle length (5 m) over a
    7.5 m footprint caps occupancy near 0.67, which is why the Gridlock rule
    uses occupancy relative to the lane's jam occupancy.
  * empty-lane mean speed == speed limit (SUMO convention).
"""
import traci

import tod_eval_common as ec

config = ec.make_config()
lanes = [l for ls in config["lane_map"].values() for l in ls]
traci.start(["sumo", "-c", config["sumocfg"], "--no-step-log", "true",
             "--route-files", "2int_files/routes.xml"])
for tl in config["tl_map"].values():
    n = len(traci.trafficlight.getRedYellowGreenState(tl))
    traci.trafficlight.setRedYellowGreenState(tl, "r" * n)

empty_speed = {l: traci.lane.getLastStepMeanSpeed(l) for l in lanes}
stat = {l: dict(cnt=0, occ=0.0, v_at_max=None) for l in lanes}
for _ in range(1500):
    traci.simulationStep()
    for l in lanes:
        c = traci.lane.getLastStepVehicleNumber(l)
        o = traci.lane.getLastStepOccupancy(l)
        s = stat[l]
        s["occ"] = max(s["occ"], o)
        if c > s["cnt"]:
            s["cnt"] = c
            s["v_at_max"] = traci.lane.getLastStepMeanSpeed(l)

print(f"{'lane':22s} {'cap(L/7.5)':>10s} {'maxcnt':>6s} {'maxOccRaw':>9s} "
      f"{'jamOcc(5m)':>10s} {'v@full':>7s} {'limit':>6s} {'v empty@t0':>10s}")
for l in lanes:
    length = traci.lane.getLength(l)
    cap = int(length // 7.5)
    s = stat[l]
    jam = min(1.0, s["cnt"] * 5.0 / length)
    v = "-" if s["v_at_max"] is None else f"{s['v_at_max']:.2f}"
    print(f"{l:22s} {cap:10d} {s['cnt']:6d} {s['occ']:9.3f} {jam:10.3f} "
          f"{v:>7s} {traci.lane.getMaxSpeed(l):6.2f} {empty_speed[l]:10.2f}")
traci.close()
