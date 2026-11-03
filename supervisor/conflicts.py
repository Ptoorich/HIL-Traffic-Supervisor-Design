"""Compatibility alias for the GPT-labelled implementation."""
import sys
from . import conflicts_GPT as _implementation
sys.modules[__name__] = _implementation
