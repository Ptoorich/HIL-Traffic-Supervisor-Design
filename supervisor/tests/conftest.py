import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # agents folder

from supervisor.config import load_config
from supervisor.conflicts import ConflictMatrix, JunctionConflicts

from .helpers import COMMANDED


@pytest.fixture(scope="session")
def cfg():
    return load_config()


@pytest.fixture(scope="session")
def matrix():
    foes = frozenset({(0, 2), (0, 3), (1, 2), (1, 3)})
    return ConflictMatrix({a: JunctionConflicts(f"tl_{a}", 4, foes, COMMANDED)
                           for a in ("tl1", "tl2")})


