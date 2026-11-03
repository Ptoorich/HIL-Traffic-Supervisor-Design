"""Compatibility alias for the GPT-labelled implementation."""
import sys
from . import traffic_classifier_GPT as _implementation
sys.modules[__name__] = _implementation
