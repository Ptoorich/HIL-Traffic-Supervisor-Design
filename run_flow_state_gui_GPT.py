#!/usr/bin/env python3
"""Section 3.4.4: reproducible FREE -> LIGHT -> HEAVY -> GRIDLOCK GUI demo.

Run from the HIL-Traffic-Supervisor-Design repository root on Windows:
    python run_flow_state_gui_GPT.py

Uses the *actual project network and original route templates*, but isolated
synthetic traffic demand and deliberate ALL-RED blocking for visual staging.
Neither the trained ASTA policies nor the integrated supervisor are run.

The predicates mirror supervisor/checks/traffic_classifier_GPT.py and
supervisor/supervisor_config_GPT.toml. No screenshot is described as a given
category unless the corresponding live measurement actually satisfies it.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import random
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from collections import Counter

# SUMO's Python tools are normally installed in the project's virtual environment.
try:
    import traci
except ImportError:
    sumo_home = os.environ.get("SUMO_HOME", "")
    if sumo_home:
        sys.path.insert(0, str(Path(sumo_home) / "tools"))
    import traci

NETWORK_TL = {
    "tl1": "30406197",
    "tl2": "cluster_30406198_7161921427_7161921428",
}
# Matches supervisor/supervisor_config_GPT.toml (November 2026).
ACTIVE_CAPACITIES = {
    "tl1": {
        "596638461#0_0": 10,
        "4761588#1_0": 13,
        "4761588#1_1": 13,
        "4761588#1_2": 13,
        "4761588#1_3": 13,
    },
    "tl2": {
        "-621271001_0": 10,
        "1190301745#0_0": 4,
        "622835372_0": 14,
        "622835372_1": 14,
        "622835372_2": 14,
    },
}
VEHICLE_LENGTH = 5.0
FREE_MAX = 0.10
GRIDLOCK_SPEED = 0.5
GRIDLOCK_OCCUPANCY_FRAC_OF_JAM = 0.90
GRIDLOCK_FILL = 0.90
GRIDLOCK_LANE_FRACTION = 0.50

# Valid start edges explicitly sampled from the existing project's routes.xml.
# Their combined traffic excites both junctions, with a bias to the five lanes
# used by the TL2 classifier.
ENTRY_WEIGHTS = {
    "622835372": 0.42,
    "-621271001": 0.18,
    "1190301745#0": 0.16,
    "596638461#0": 0.10,
    "4761588#1": 0.14,
}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--focus", choices=("tl1", "tl2"), default="tl2",
                   help="junction whose measured states drive the four pauses")
    p.add_argument("--root", type=Path, default=Path(__file__).resolve().parent,
                   help="repository root containing 2int_files")
    p.add_argument("--output-dir", type=Path, default=Path("supervisor_logs/flow_states_gui_2_GPT"),
                   help="new directory for screenshots and measurement log")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--speed", type=float, default=0.025,
                   help="wall-clock sleep in seconds after each SUMO simulation step")
    p.add_argument("--max-steps", type=int, default=1400)
    p.add_argument("--no-pause", action="store_true",
                   help="save screenshots and continue without asking for ENTER")
    return p.parse_args()


def prepare_route_templates(routes_file: Path):
    """Read existing valid network routes, grouped by source approach."""
    root = ET.parse(routes_file).getroot()
    counts: dict[str, Counter] = {edge: Counter() for edge in ENTRY_WEIGHTS}
    for vehicle in root.findall("vehicle"):
        route = vehicle.find("route")
        if route is None:
            continue
        edges = tuple(route.get("edges", "").split())
        if edges and edges[0] in counts:
            counts[edges[0]][edges] += 1
    missing = [edge for edge, group in counts.items() if not group]
    if missing:
        raise RuntimeError("No valid route templates start at: " + ", ".join(missing))
    return counts


def register_routes(counts):
    """TraCI routes are loaded from the user's existing routed vehicle trips."""
    choices = {}
    for entry, c in counts.items():
        opts = []
        for i, (edges, count) in enumerate(c.items()):
            rid = f"flow_demo_{entry.replace('#', 'x').replace('-', 'm')}_{i}"
            traci.route.add(rid, list(edges))
            opts.append((rid, count))
        choices[entry] = opts
    return choices


def classification(agent):
    """Match traffic_classifier_GPT.py using TraCI's 0..1 occupancy convention."""
    gridlocked = []
    fills = []
    details = []
    for lane, capacity in ACTIVE_CAPACITIES[agent].items():
        halting = traci.lane.getLastStepHaltingNumber(lane)
        count = traci.lane.getLastStepVehicleNumber(lane)
        occ = traci.lane.getLastStepOccupancy(lane)
        speed = traci.lane.getLastStepMeanSpeed(lane)
        length = traci.lane.getLength(lane)
        jam_occ = min(1.0, capacity * VEHICLE_LENGTH / length)
        fill = halting / capacity
        is_jam = (halting > 0 and speed < GRIDLOCK_SPEED
                  and occ >= GRIDLOCK_OCCUPANCY_FRAC_OF_JAM * jam_occ
                  and fill >= GRIDLOCK_FILL)
        if is_jam:
            gridlocked.append(lane)
        fills.append(fill)
        details.append((lane, count, halting, round(occ, 3), round(speed, 2),
                        round(fill, 3), is_jam))
    mean_fill = sum(fills) / len(fills)
    need = math.ceil(GRIDLOCK_LANE_FRACTION * len(fills) - 1e-9)
    if len(gridlocked) >= need:
        label = "GRIDLOCK"
    elif gridlocked:
        label = "HEAVY"
    else:
        label = "FREE" if mean_fill <= FREE_MAX else "LIGHT"
    return {"label": label, "mean_fill": mean_fill,
            "jammed": gridlocked, "need": need, "details": details}


def add_demand(rng, route_choices, accumulator, rate, next_id):
    """Deterministic average injection rate, varied by entry and route mix."""
    # Fractional vehicle arrivals are carried to subsequent seconds.
    for entry, share in ENTRY_WEIGHTS.items():
        accumulator[entry] += rate * share
        n = int(accumulator[entry])
        accumulator[entry] -= n
        route_ids = [rid for rid, count in route_choices[entry]]
        weights = [count for rid, count in route_choices[entry]]
        for _ in range(n):
            route = rng.choices(route_ids, weights=weights, k=1)[0]
            # Each is an independent trip, using the original network's routes.
            traci.vehicle.add(f"flow_demo_veh_{next_id:06d}", route,
                              depart="now", departLane="random", departSpeed="max")
            next_id += 1
    return next_id


def set_demo_red_hold():
    """Intentionally block both junctions; DEMONSTRATION ONLY, not safety control."""
    for tl in NETWORK_TL.values():
        state = traci.trafficlight.getRedYellowGreenState(tl)
        traci.trafficlight.setRedYellowGreenState(tl, "r" * len(state))


def gui_setup():
    try:
        ids = traci.gui.getIDList()
        if not ids:
            return None
        view = ids[0]
        # Both controlled junctions and most short monitored approaches fit.
        traci.gui.setBoundary(view, 735.0, 145.0, 1075.0, 365.0)
        try:
            traci.gui.setSchema(view, "real world")
        except Exception:
            pass
        return view
    except Exception as e:
        print("GUI viewport setup skipped:", e)
        return None


def capture_and_pause(stage, view, out, before, after, args):
    """Request a screenshot (executed on next SUMO step), then pause."""
    n = {"FREE": 1, "LIGHT": 2, "HEAVY": 3, "GRIDLOCK": 4}[stage]
    filename = out / f"{n:02d}_{stage.lower()}_{args.focus}_GPT.png"
    if view is not None:
        try:
            traci.gui.screenshot(view, str(filename), 1600, 1000)
        except Exception as e:
            print("Unable to schedule GUI screenshot:", e)
            filename = None
    else:
        filename = None
    # Screenshot is taken on the *next* simulationStep(), not immediately.
    traci.simulationStep()
    now = traci.simulation.getTime()
    state = classification(args.focus)
    print("\n" + "=" * 72)
    print(f"CAPTURE POINT: {stage} | focus={args.focus} | sim t={now:.0f}s")
    print(f"Current live classifier: {state['label']}; mean fill {state['mean_fill']:.3f}; "
          f"jammed {len(state['jammed'])}/{state['need']} required")
    print(f"Other junction: {classification('tl1' if args.focus == 'tl2' else 'tl2')['label']}")
    print(f"Vehicles on network: {traci.vehicle.getIDCount()}")
    for lane, cnt, hlt, occ, spd, fill, jam in state["details"]:
        print(f"  {lane:20s} count={cnt:2d} halted={hlt:2d} "
              f"occ={occ:.3f} speed={spd:5.2f} fill={fill:.3f} jam={jam}")
    if filename is not None:
        print("Screenshot requested:", filename)
    print("The SUMO GUI is paused while Python waits for keyboard input.")
    print("You can use Win+Shift+S for your own screenshot now.")
    if not args.no_pause:
        try:
            input("Press ENTER here in the VS Code terminal to continue... ")
        except EOFError:
            pass
    print("=" * 72 + "\n")
    return state, float(now), str(filename or "")


def main():
    args = parse_args()
    root = args.root.resolve()
    network = root / "2int_files" / "osm.net.xml.gz"
    if not network.exists():
        network = root / "2int_files" / "osm.net.xml"
    routes = root / "2int_files" / "routes.xml"
    if not network.exists() or not routes.exists():
        sys.exit("Missing network and/or routes.xml in 2int_files; run from project root.")
    binary = shutil.which("sumo-gui")
    if binary is None and os.environ.get("SUMO_HOME"):
        binary = str(Path(os.environ["SUMO_HOME"]) / "bin" / "sumo-gui.exe")
    if binary is None:
        sys.exit("sumo-gui not found. Install SUMO and add SUMO/bin to PATH.")
    if args.output_dir.exists():
        sys.exit(f"Output directory already exists (original files preserved): {args.output_dir}\n"
                 "Use --output-dir supervisor_logs/flow_states_gui_02_GPT")
    args.output_dir.mkdir(parents=True)

    templates = prepare_route_templates(routes)
    rng = random.Random(args.seed)
    print("Isolated traffic-state VISUALIZATION run: no trained policy or supervisor.")
    print("Provisional schedule: normal signal plans for FREE/LIGHT, then a synthetic")
    print("all-red hold with increasing demand for HEAVY/GRIDLOCK. Not a field-safe plan.")
    print("Captures use the live classifier rules for:", args.focus)
    print("Starting SUMO GUI...", binary)

    # No use of the repo's 2int.sumocfg: prevents changes to benchmark output files,
    # avoids auxiliary detectors and routes, uses the original network only.
    cmd = [binary, "--net-file", str(network),
           "--begin", "0", "--step-length", "1", "--no-step-log", "true",
           "--duration-log.disable", "true", "--time-to-teleport", "-1",
           "--collision.action", "warn", "--start"]

    traci.start(cmd)
    csv_path = args.output_dir / "flow_state_measurements_GPT.csv"
    try:
        choice = register_routes(templates)
        view = gui_setup()
        accumulator = {entry: 0.0 for entry in ENTRY_WEIGHTS}
        next_id = 0
        achieved = set()
        hold_red = False
        stage_mode = "FREE"
        screenshot_info = []
        with csv_path.open("w", newline="", encoding="utf-8") as handle:
            w = csv.writer(handle)
            w.writerow(["sim_time_s", "demo_stage", "focus", "focus_label",
                        "focus_mean_fill", "focus_jammed_lanes", "tl1_label",
                        "tl2_label", "network_vehicles", "forced_all_red"])
            while traci.simulation.getTime() < args.max_steps and len(achieved) < 4:
                t = traci.simulation.getTime()
                # A stage is advanced by a CONFIRMED screenshot trigger only.
                if stage_mode == "FREE":
                    rate = 0.25               # 900 vehicles/h, light background demand
                elif stage_mode == "LIGHT":
                    rate = 1.5                # 5400 vehicles/h transient inflow
                    if t >= 150 and not hold_red:
                        print("LIGHT assist: temporarily holding all-red so a queue becomes visible.")
                        set_demo_red_hold()
                        hold_red = True
                else:
                    rate = 4.5                # 16,200 vehicles/h short synthetic stress
                    if not hold_red:
                        print("Blocking signal movements for HEAVY / GRIDLOCK imagery.")
                        set_demo_red_hold()
                        hold_red = True
                next_id = add_demand(rng, choice, accumulator, rate, next_id)
                traci.simulationStep()
                t = traci.simulation.getTime()
                states = {a: classification(a) for a in NETWORK_TL}
                focus = states[args.focus]
                jammed = len(focus["jammed"])
                w.writerow([int(t), stage_mode, args.focus, focus["label"],
                            round(focus["mean_fill"], 4), jammed,
                            states["tl1"]["label"], states["tl2"]["label"],
                            traci.vehicle.getIDCount(), hold_red])
                if int(t) % 20 == 0:
                    print(f"t={int(t):4d}s | stage={stage_mode:8s} | "
                          f"TL1={states['tl1']['label']:8s} | "
                          f"TL2={states['tl2']['label']:8s} | "
                          f"focus fill={focus['mean_fill']:.2f}, jam={jammed}, "
                          f"vehicles={traci.vehicle.getIDCount()}")
                if stage_mode == "FREE":
                    matched = focus["label"] == "FREE" and t >= 15 and traci.vehicle.getIDCount() >= 1
                elif stage_mode == "LIGHT":
                    matched = focus["label"] == "LIGHT" and t >= 65
                else:
                    matched = focus["label"] == stage_mode
                if matched and stage_mode not in achieved:
                    after, screenshot_time, filename = capture_and_pause(
                        stage_mode, view, args.output_dir, focus, states, args)
                    if after["label"] == stage_mode:
                        achieved.add(stage_mode)
                        screenshot_info.append((stage_mode, screenshot_time, filename))
                        if stage_mode == "FREE":
                            stage_mode = "LIGHT"
                        elif stage_mode == "LIGHT":
                            stage_mode = "HEAVY"
                        elif stage_mode == "HEAVY":
                            stage_mode = "GRIDLOCK"
                        else:
                            break
                    else:
                        print("State changed during screenshot step; retrying stage.")
                if args.speed:
                    time.sleep(args.speed)
            print("\nConfirmed screenshot states:", sorted(achieved))
            if len(achieved) != 4:
                print("WARNING: Not all four measured states were obtained. Increase --max-steps")
                print("or select the other --focus. Do NOT relabel an unmatched screenshot.")
            for lab, when, filename in screenshot_info:
                print(f"  {lab:8s} at t={when:.0f}s: {filename}")
            print("Measurement log:", csv_path)
    finally:
        # Explicit exit from Python closes this demonstration's SUMO process.
        try:
            traci.close(False)
        except Exception:
            pass


if __name__ == "__main__":
    main()
