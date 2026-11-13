"""SP-28 post-command execution verification (simulation laboratory scope).

A readback failure is a structured FAIL and a controlled test interruption.
It is NOT a Fast Gate decision and does NOT claim that switching to FTSC
is safe when SUMO signal execution or the display interface is untrusted.
"""
from __future__ import annotations

from .hil_serial_GPT import DisplayError
from .observation_adapter_GPT import read_active_programs


class SP28ReadbackFailure(ValueError):
    """Explicit SP-28 FAIL; ValueError compatibility preserves older callers."""

    def __init__(self, *, source, agent, reason, stage):
        self.requirement = "SP-28"
        self.status = "FAIL"
        self.source = source
        self.agent = agent
        self.reason = reason
        self.stage = stage
        super().__init__(reason)

    def record(self, *, sim_time, step_in_cycle, cycle):
        return {
            "requirement": self.requirement,
            "status": self.status,
            "source": self.source,
            "agent": self.agent,
            "reason": self.reason,
            "stage": self.stage,
            "sim_time": sim_time,
            "cycle": cycle,
            "step_in_cycle": step_in_cycle,
            "response": "ABORT_UNTRUSTED_EXECUTION",
            "ftsc_commanded": False,
            "note": "Post-command verification FAIL, not a pre-command Fast Gate VETO.",
        }


def _agent_from_message(reason, agents):
    for agent in agents:
        if reason.startswith(agent + ":"):
            return agent
    return None  # No justified attribution; potentially affects both junctions.


def verify_program(env, traci_mod, *, stage):
    """Validate accepted split versus active SUMO programme, per requested sample."""
    try:
        return read_active_programs(env, traci_mod)
    except Exception as exc:
        reason = str(exc)
        raise SP28ReadbackFailure(
            source="sumo_program_readback",
            agent=_agent_from_message(reason, env.agent_ids),
            reason=f"{type(exc).__name__}: {reason}",
            stage=stage,
        ) from exc


def verify_display(display, env, traci_mod, modes, transitions, *, stage):
    """Verify live SUMO indication and Arduino protocol/GPIO ACK as one readback."""
    try:
        return display.sample(env, traci_mod, modes, transitions)
    except Exception as exc:
        reason = str(exc)
        if isinstance(exc, SP28ReadbackFailure):
            raise
        if isinstance(exc, DisplayError):
            source = ("arduino_gpio_or_communication"
                      if getattr(display.transport, "failed", False)
                      else "sumo_live_phase_or_display")
        elif isinstance(exc, ValueError):
            source = "sumo_program_readback"
        else:
            source = "execution_readback_unavailable"
        raise SP28ReadbackFailure(
            source=source,
            agent=_agent_from_message(reason, env.agent_ids),
            reason=f"{type(exc).__name__}: {reason}",
            stage=stage,
        ) from exc
