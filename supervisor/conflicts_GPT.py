"""
Conflict matrix (which signal links are foes) read from the SUMO net file.

net.xml stores, per tl junction, one <request index=".." foes=".."/> per link.
`foes` is a bit string in REVERSED order: the right-most character is link 0
(this is also how sumolib's Node.areFoes reads it). The request index equals
the traffic-light linkIndex, so a foe pair (i, j) indexes the characters of a
signal state string directly.

Directed asymmetries are recorded and conservatively treated as foes in both
directions. Self-conflicts and malformed matrices are rejected.

dump_conflicts_GPT.py (in the agents folder) writes this once to a JSON fixture
which the supervisor loads at startup and the unit tests reuse.
"""
from __future__ import annotations

import gzip
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path


class ConflictMatrixError(ValueError):
    pass


@dataclass(frozen=True)
class JunctionConflicts:
    tl_id: str
    n_links: int
    foes: frozenset                      # frozenset[tuple[int, int]], i < j
    net_phase_states: tuple = field(default=())   # first program of the net, in order
    asymmetric_pairs: frozenset = field(default_factory=frozenset)  # directed (i, j)


@dataclass(frozen=True)
class ConflictMatrix:
    junctions: dict                      # agent -> JunctionConflicts


def _open(path: Path):
    return gzip.open(path, "rb") if str(path).endswith(".gz") else open(path, "rb")


def _foes_from_requests(tl_id: str, requests: dict[int, str]) -> tuple[int, frozenset]:
    n = len(requests)
    if sorted(requests) != list(range(n)):
        raise ConflictMatrixError(f"{tl_id}: request indices are not 0..{n - 1}")
    pairs = set()
    for i, bits in requests.items():
        if not isinstance(bits, str):
            raise ConflictMatrixError(f"{tl_id}: missing foes bits for request {i}")
        if len(bits) != n:
            raise ConflictMatrixError(
                f"{tl_id}: foes string for request {i} has length {len(bits)}, expected {n}")
        if not isinstance(bits, str) or any(ch not in "01" for ch in bits):
            raise ConflictMatrixError(f"{tl_id}: invalid foes bits for request {i}")
        for pos, ch in enumerate(bits):
            if ch == "1":
                j = n - 1 - pos                     # reversed: right-most = link 0
                if j == i:
                    raise ConflictMatrixError(
                        f"{tl_id}: link {i} is its own foe; bit order assumption is wrong")
                pairs.add((i, j))
    asymmetric = frozenset((i, j) for i, j in pairs if (j, i) not in pairs)
    return n, frozenset((min(i, j), max(i, j)) for i, j in pairs), asymmetric


def parse_net(net_path: str | Path, tl_ids: dict[str, str]) -> ConflictMatrix:
    """tl_ids: {agent: traffic-light id}. The junction id equals the tl id."""
    path = Path(net_path)
    wanted = {tl: agent for agent, tl in tl_ids.items()}
    if len(wanted) != len(tl_ids):
        raise ConflictMatrixError(f"two agents share one traffic light id: {tl_ids}")
    requests: dict[str, dict[int, str]] = {tl: {} for tl in wanted}
    phases: dict[str, list[str]] = {}
    links: dict[str, set[int]] = {tl: set() for tl in wanted}
    with _open(path) as f:
        for _, el in ET.iterparse(f, events=("end",)):
            if el.tag == "junction" and el.get("id") in wanted:
                for rq in el.findall("request"):
                    try:
                        idx = int(rq.get("index"))
                    except (TypeError, ValueError):
                        raise ConflictMatrixError(f"{el.get('id')}: invalid request index") from None
                    if idx in requests[el.get("id")]:
                        raise ConflictMatrixError(f"{el.get('id')}: duplicate request index {idx}")
                    requests[el.get("id")][idx] = rq.get("foes")
            elif el.tag == "tlLogic" and el.get("id") in wanted:
                phases.setdefault(el.get("id"), []).extend(
                    p.get("state") for p in el.findall("phase"))
            elif el.tag == "connection" and el.get("tl") in wanted:
                try:
                    links[el.get("tl")].add(int(el.get("linkIndex")))
                except (TypeError, ValueError):
                    raise ConflictMatrixError(f"{el.get('tl')}: invalid connection linkIndex") from None
            if el.tag in ("junction", "tlLogic", "edge", "connection"):
                el.clear()
    out = {}
    for tl, agent in wanted.items():
        if not requests[tl]:
            raise ConflictMatrixError(f"junction {tl} ({agent}) not found in {path}")
        n, foes, asymmetric = _foes_from_requests(tl, requests[tl])
        if links[tl] and links[tl] != set(range(n)):
            raise ConflictMatrixError(f"{tl}: connection link indices disagree with request matrix")
        states = tuple(phases.get(tl, ()))
        if not states:
            raise ConflictMatrixError(f"{tl}: missing signal phases")
        for state in states:
            if not isinstance(state, str) or len(state) != n:
                raise ConflictMatrixError(f"{tl}: signal state length differs from {n} links")
            if any(state[i] == state[j] == "G" for i, j in foes):
                raise ConflictMatrixError(f"{tl}: conflicting protected greens in network phase")
        out[agent] = JunctionConflicts(tl_id=tl, n_links=n, foes=foes,
                                       asymmetric_pairs=asymmetric, net_phase_states=states)
    return ConflictMatrix(out)


def save_fixture(matrix: ConflictMatrix, path: str | Path, source: str = "") -> None:
    data = {"version": 1, "source_net": source, "junctions": {
        a: {"tl_id": j.tl_id, "n_links": j.n_links,
            "foes": sorted([list(p) for p in j.foes]),
            "asymmetric_pairs": sorted([list(p) for p in j.asymmetric_pairs]),
            "net_phase_states": list(j.net_phase_states)}
        for a, j in matrix.junctions.items()}}
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(data, indent=1))


def load_fixture(path: str | Path) -> ConflictMatrix:
    try:
        data = json.loads(Path(path).read_text())
    except FileNotFoundError:
        raise ConflictMatrixError(
            f"conflict fixture {path} not found; run dump_conflicts_GPT.py once") from None
    except (json.JSONDecodeError, OSError) as e:
        raise ConflictMatrixError(f"invalid conflict fixture {path}: {e}") from e
    if not isinstance(data, dict):
        raise ConflictMatrixError(f"invalid conflict fixture {path}")
    if data.get("version") != 1:
        raise ConflictMatrixError(f"unsupported fixture version in {path}")
    try:
        out = {}
        used = set()
        for agent, j in data["junctions"].items():
            n = j["n_links"]
            if not isinstance(n, int) or isinstance(n, bool) or n <= 0 or j["tl_id"] in used:
                raise ValueError("invalid link count or duplicate traffic light")
            used.add(j["tl_id"])
            def validate_pair(pair):
                if (not isinstance(pair, list) or len(pair) != 2 or
                    any(not isinstance(x, int) or isinstance(x, bool) or not 0 <= x < n for x in pair) or pair[0] == pair[1]):
                    raise ValueError("invalid foe pair")
                return tuple(pair)
            foes = frozenset(tuple(sorted(validate_pair(p))) for p in j["foes"])
            asym = frozenset(validate_pair(p) for p in j.get("asymmetric_pairs", []))
            if any(tuple(sorted(p)) not in foes for p in asym):
                raise ValueError("asymmetry missing from foes")
            states = tuple(j.get("net_phase_states", ()))
            for state in states:
                if not isinstance(state, str) or len(state) != n or any(state[i] == state[k] == "G" for i,k in foes):
                    raise ValueError("invalid or conflicting phase")
            out[agent] = JunctionConflicts(j["tl_id"], n, foes, states, asym)
        if not out:
            raise ValueError("empty junctions")
        return ConflictMatrix(out)
    except (KeyError, TypeError, ValueError) as e:
        raise ConflictMatrixError(f"invalid conflict fixture {path}: {e}") from e
