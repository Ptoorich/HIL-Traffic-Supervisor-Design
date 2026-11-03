"""
Conflict Monitor: no two conflicting movements may be green together.

For every foe pair (i, j) in the net's conflict matrix, in each phase string
checked (the strings the env will command, plus the live read-back):

  G + G   -> FAIL  (two priority greens on foes: always a real conflict)
  G + g   -> allowed (the 'g' movement yields, e.g. permissive turns at tl2)
  g + g   -> WARN  (mutual yielding, legal but worth seeing)

Other characters (r, y, u, o ...) are ignored. A string whose length does not
match the matrix is a FAIL (wrong program or wrong net).

With the current env a split change cannot alter the phase strings, so this
check cannot fail because of a proposal. It guards against misconfigured
programs and future multi-phase junctions (D4: backstop).

NOTE: the live read-back is taken at a cycle boundary, where the program has
just finished its all-red. The commanded strings carry the real substance.
"""
from __future__ import annotations

from ..config import SupervisorConfig
from ..conflicts import ConflictMatrix
from ..datatypes import CheckResult, JunctionSnapshot, Proposal, Status

NAME = "conflict_monitor"
_MAX_LISTED = 10


def check_state(state: str, foes, n_links: int, label: str):
    """Return (hard, soft) lists of violation dicts for one phase string."""
    hard, soft = [], []
    if len(state) != n_links:
        hard.append({"string": label, "code": "STATE_LENGTH",
                     "len": len(state), "expected": n_links})
        return hard, soft
    for i, j in sorted(foes):
        a, b = state[i], state[j]
        if a == "G" and b == "G":
            hard.append({"string": label, "code": "CONFLICT_GG", "links": [i, j]})
        elif a == "g" and b == "g":
            soft.append({"string": label, "code": "CONFLICT_gg", "links": [i, j]})
    return hard, soft


def check(snap: JunctionSnapshot, proposal: Proposal, cfg: SupervisorConfig,
          matrix: ConflictMatrix) -> CheckResult:
    jc = matrix.junctions.get(snap.agent)
    if jc is None:
        return CheckResult(NAME, Status.FAIL, "NO_MATRIX", {"agent": snap.agent})

    strings = [("live", snap.live_state)]
    strings += [(f"commanded[{k}]", s) for k, s in enumerate(snap.commanded_phase_strings)]
    hard, soft = [], []
    for label, s in strings:
        h, w = check_state(s, jc.foes, jc.n_links, label)
        hard += h
        soft += w

    if hard:
        status, reason = Status.FAIL, "CONFLICT"
    elif soft:
        status, reason = Status.WARN, "PERMISSIVE_CONFLICT"
    else:
        status, reason = Status.PASS, "OK"
    return CheckResult(NAME, status, reason, {
        "violations": hard[:_MAX_LISTED], "n_violations": len(hard),
        "soft": soft[:_MAX_LISTED], "n_soft": len(soft),
        "strings_checked": len(strings)})
