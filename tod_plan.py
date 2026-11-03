"""
Time-of-day signal plans + time-varying demand for the 2-intersection network.

Everything here is plain Python (no SUMO needed), so it can be tested alone.

Design (agreed with Siphetfo):
  * Fixed 90 s cycle for ALL plans; only the green split changes by period.
  * Intergreen = 3 s yellow + 3 s all-red per phase (lost time 12 s/cycle).
  * Two phases per junction (A and B), no protected turns, pedestrians ignored.
  * Compressed day: warm-up, then AM peak, midday, PM peak, evening, night
    (1 h each, run continuously), then a zero-demand cool-down.
  * Splits derived from each period's demand with Webster's method.

ASSUMPTIONS (labelled so they can be stated in the report):
  * Demand index per period relative to the peak (PERIODS below).
  * Saturation flow 1800 veh/h/lane (SAT_FLOW). Permissive 'g' movements at
    tl2 phase A mean the true value is lower, so capacity is overestimated.
  * Webster lost time L = total intergreen per cycle.
"""
import numpy as np
import xml.etree.ElementTree as ET
from collections import Counter

# ── Timeline (simulated seconds) ───────────────────────────────────────
WARMUP     = 900      # 10 cycles at AM-peak demand/plan, excluded from metrics
PERIOD_LEN = 3600     # 40 cycles per period, so changeovers sit on cycle edges
COOLDOWN   = 900      # zero demand so vehicles can finish and be counted
RAMP       = 300      # width of the linear demand ramp centred on a boundary

# (name, demand index relative to peak)  -- ASSUMED, not measured
PERIODS = [
    ("AM_peak", 1.00),
    ("midday",  0.55),
    ("PM_peak", 1.00),
    ("evening", 0.45),
    ("night",   0.15),
]
TOTAL_TIME = WARMUP + len(PERIODS) * PERIOD_LEN + COOLDOWN   # 19 800 s

# Original routes.xml carries 7200 veh in 3600 s
ORIGINAL_RATE_VPH = 7200.0

SAT_FLOW = 1800.0     # veh/h/lane, ASSUMED

# SARTSM vol 3 ch 6 (as quoted by the user)
MIN_GREEN_HARD = 7.0      # main signal phase, absolute minimum
MIN_GREEN_PREF = 11.0     # preferred

# Which approach edges each junction phase serves (from getControlledLinks)
PHASE_MAP = {
    "tl1": (["596638461#0"],
            ["4761588#1"]),
    "tl2": (["-621271001", "669676574#0", "1190301745#0"],
            ["-669676574#8", "622835372"]),
}


# ── Timeline helpers ───────────────────────────────────────────────────
def period_table():
    rows = []
    for i, (name, idx) in enumerate(PERIODS):
        s = WARMUP + i * PERIOD_LEN
        rows.append({"name": name, "start": s, "end": s + PERIOD_LEN,
                     "index": idx})
    return rows


def _segments():
    """(start_time, demand_index) segments including warm-up and cool-down."""
    segs = [(0.0, PERIODS[0][1])]                      # warm-up at AM level
    for r in period_table():
        segs.append((float(r["start"]), r["index"]))
    segs.append((float(WARMUP + len(PERIODS) * PERIOD_LEN), 0.0))  # cool-down
    return segs


def demand_index(t):
    """Piecewise-constant demand index with linear ramps at boundaries."""
    segs = _segments()
    val = segs[0][1]
    for b, v in segs:
        if t >= b:
            val = v
    for i in range(1, len(segs)):
        b, v = segs[i]
        prev = segs[i - 1][1]
        if abs(t - b) < RAMP / 2.0:
            frac = (t - (b - RAMP / 2.0)) / RAMP
            return prev + (v - prev) * frac
    return val


# ── Directional (tidal) mix by period ──────────────────────────────────
# Multipliers on routes by FIRST edge (approach where the vehicle enters).
# With NO bias the route mix is identical in every period, so Webster gives
# the same split in every period and the "time-of-day" plan collapses to one
# static split. TIDAL_EXAMPLE is ILLUSTRATIVE ONLY (directions invented to
# exercise the code) - replace with the real inbound/outbound pattern.
TIDAL_EXAMPLE = {
    "AM_peak": {"-621271001": 1.3, "1190301745#0": 1.3, "622835372": 0.7},
    "PM_peak": {"-621271001": 0.8, "1190301745#0": 0.8, "622835372": 1.5},
}


def mix_for_period(mix, name, tidal=None):
    """Reweight the route mix for a period (weights re-normalised to 1, so
    total volume is unchanged and only the directional split moves)."""
    bias = (tidal or {}).get(name, {})
    w = np.array([wt * bias.get(edges.split()[0], 1.0) for edges, wt in mix])
    w = w / w.sum()
    return [(edges, float(x)) for (edges, _), x in zip(mix, w)]


def period_name_at(t):
    """Warm-up counts as the first period, cool-down as the last."""
    rows = period_table()
    if t < rows[0]["start"]:
        return rows[0]["name"]
    for r in rows:
        if r["start"] <= t < r["end"]:
            return r["name"]
    return rows[-1]["name"]


# ── Demand generation ──────────────────────────────────────────────────
def load_route_mix(routes_xml):
    """Return [(edges_string, weight)] from the original per-vehicle file."""
    root = ET.parse(routes_xml).getroot()
    cnt = Counter()
    for v in root.iter("vehicle"):
        cnt[v.find("route").get("edges")] += 1
    total = float(sum(cnt.values()))
    return [(e, c / total) for e, c in sorted(cnt.items())]


def generate_route_file(path, mix, seed, peak_rate_vph, tidal=None):
    """
    Non-homogeneous Poisson departures (thinning) following demand_index(t),
    routes drawn from the route mix of the period each departure falls in
    (identical mix every period unless `tidal` is given).
    Returns departure times.
    """
    rng = np.random.default_rng(seed)
    lam_max = peak_rate_vph / 3600.0
    deps, t = [], 0.0
    while True:
        t += rng.exponential(1.0 / lam_max)
        if t >= TOTAL_TIME:
            break
        if rng.random() < demand_index(t):
            deps.append(t)
    deps = np.round(np.array(deps), 2)   # match the 0.01 s written below

    picks = np.zeros(len(deps), dtype=int)
    names = np.array([period_name_at(d) for d in deps])
    for r in period_table():
        sel = np.where(names == r["name"])[0]
        w = np.array([x for _, x in mix_for_period(mix, r["name"], tidal)])
        picks[sel] = rng.choice(len(mix), size=len(sel), p=w)

    with open(path, "w") as f:
        f.write('<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
                'xsi:noNamespaceSchemaLocation='
                '"http://sumo.dlr.de/xsd/routes_file.xsd">\n')
        for i, (d, k) in enumerate(zip(deps, picks)):
            f.write(f'    <vehicle id="{i}" depart="{d:.2f}">'
                    f'<route edges="{mix[k][0]}"/></vehicle>\n')
        f.write('</routes>\n')
    return deps


# ── Webster splits ─────────────────────────────────────────────────────
def lanes_per_edge(lane_map):
    cnt = Counter()
    for lanes in lane_map.values():
        for lane in lanes:
            cnt[lane.rsplit("_", 1)[0]] += 1
    return cnt


def approach_flows(mix, rate_vph):
    """Expected veh/h on every edge that appears in any route."""
    flows = Counter()
    for edges, w in mix:
        for e in set(edges.split()):
            flows[e] += w * rate_vph
    return flows


def webster_split(flows, lanes, cycle, inter, s_lb, s_ub, sat=SAT_FLOW):
    """
    Split for phase A of one junction at a FIXED cycle.
    Returns (split_A, diagnostics). Phase B green is cycle - A - 2*inter.
    """
    lost = 2 * inter
    eff = cycle - lost
    out = {}
    for agent, (phA, phB) in PHASE_MAP.items():
        yA = max(flows.get(e, 0.0) / (lanes[e] * sat) for e in phA)
        yB = max(flows.get(e, 0.0) / (lanes[e] * sat) for e in phB)
        Y = yA + yB
        gA = eff / 2.0 if Y <= 0 else eff * yA / Y
        gA = int(round(min(max(gA, s_lb), s_ub)))
        out[agent] = {
            "split": gA,
            "yA": yA, "yB": yB, "Y": Y,
            # degree of saturation of the whole junction at this fixed cycle
            "X": Y * cycle / eff,
            # Webster optimum, shown for information only (we keep 90 s)
            "C_webster": (1.5 * lost + 5.0) / (1.0 - Y) if Y < 0.95 else None,
        }
    return out


# ── SARTSM validator ───────────────────────────────────────────────────
def validate_plan(splits, cycle, yellow, allred):
    """Hard checks; raises ValueError if a plan breaks SARTSM minimums."""
    inter = yellow + allred
    errors, warns = [], []
    for agent, s in splits.items():
        gA, gB = s, cycle - s - 2 * inter
        if abs(gA + gB + 2 * inter - cycle) > 1e-9:
            errors.append(f"{agent}: phases do not sum to the cycle")
        for name, g in (("A", gA), ("B", gB)):
            if g < MIN_GREEN_HARD:
                errors.append(f"{agent} phase {name} green {g}s < "
                              f"{MIN_GREEN_HARD}s hard minimum")
            elif g < MIN_GREEN_PREF:
                warns.append(f"{agent} phase {name} green {g}s < "
                             f"{MIN_GREEN_PREF}s preferred")
    if errors:
        raise ValueError("; ".join(errors))
    return warns


# ── Schedule ───────────────────────────────────────────────────────────
def build_schedule(mix, lane_map, peak_rate_vph, cycle, yellow, allred,
                   s_lb, s_ub, tidal=None):
    lanes = lanes_per_edge(lane_map)
    sched = []
    for r in period_table():
        rate = peak_rate_vph * r["index"]
        pmix = mix_for_period(mix, r["name"], tidal)
        diag = webster_split(approach_flows(pmix, rate), lanes, cycle,
                             yellow + allred, s_lb, s_ub)
        splits = {a: d["split"] for a, d in diag.items()}
        warns = validate_plan(splits, cycle, yellow, allred)
        sched.append({**r, "rate_vph": rate, "splits": splits,
                      "diag": diag, "warnings": warns})
    return sched


def plan_for_time(schedule, t):
    """Active plan at sim time t (warm-up uses first plan, cool-down last)."""
    if t < schedule[0]["start"]:
        return schedule[0]
    for p in schedule:
        if p["start"] <= t < p["end"]:
            return p
    return schedule[-1]