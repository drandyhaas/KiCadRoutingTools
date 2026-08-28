"""Validated defaults and process-local session policy for Track Gloss."""

from __future__ import annotations

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path


CONFIG_PATH = Path(__file__).with_name("internal_config.json")


@dataclass(frozen=True)
class GlossPolicy:
    minimum_saved_length_mm: float


@dataclass(frozen=True)
class ConvergencePolicy:
    interactive_group_max_passes: int
    cli_max_passes: int


@dataclass(frozen=True)
class SafetyPolicy:
    kicad_drc_for_single_track: bool


@dataclass(frozen=True)
class TimingPolicy:
    interactive_total_time_budget_seconds: float
    interactive_planning_time_budget_seconds: float
    interactive_cancellation_grace_seconds: float
    cli_total_time_budget_seconds: float | None


@dataclass(frozen=True)
class InternalConfig:
    schema_version: int
    gloss: GlossPolicy
    convergence: ConvergencePolicy
    timing: TimingPolicy
    safety: SafetyPolicy


def _positive_integer(value, field):
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError("{} must be an integer of at least one".format(field))
    return value


def _positive_number(value, field, *, allow_zero=False):
    minimum_ok = value >= 0.0 if allow_zero else value > 0.0
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or not minimum_ok):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError("{} must be a finite {} number".format(
            field, qualifier))
    return float(value)


def load_internal_config(path=CONFIG_PATH):
    """Load and strictly validate the packaged internal policy document."""
    source = Path(path)
    try:
        document = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(
            "cannot load Track Gloss internal configuration: {}".format(error))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("unsupported Track Gloss configuration schema")
    try:
        gloss = document["gloss"]
        convergence = document["convergence"]
        timing = document["timing"]
        safety = document["safety"]
        minimum = gloss["minimum_saved_length_mm"]
        single_track_drc = safety["kicad_drc_for_single_track"]
    except (KeyError, TypeError) as error:
        raise ValueError(
            "missing Track Gloss internal configuration field: {}".format(error))
    if (isinstance(minimum, bool) or not isinstance(minimum, (int, float)) or
            not math.isfinite(minimum) or minimum < 0.0):
        raise ValueError(
            "gloss.minimum_saved_length_mm must be a finite non-negative number")
    if not isinstance(single_track_drc, bool):
        raise ValueError(
            "safety.kicad_drc_for_single_track must be a boolean")
    total_budget = _positive_number(
        timing.get("interactive_total_time_budget_seconds"),
        "timing.interactive_total_time_budget_seconds")
    planning_budget = _positive_number(
        timing.get("interactive_planning_time_budget_seconds"),
        "timing.interactive_planning_time_budget_seconds")
    if planning_budget > total_budget:
        raise ValueError(
            "interactive planning time budget cannot exceed total time budget")
    cli_budget = timing.get("cli_total_time_budget_seconds")
    if cli_budget is not None:
        cli_budget = _positive_number(
            cli_budget, "timing.cli_total_time_budget_seconds")
    return InternalConfig(
        schema_version=1,
        gloss=GlossPolicy(float(minimum)),
        convergence=ConvergencePolicy(
            _positive_integer(
                convergence.get("interactive_group_max_passes"),
                "convergence.interactive_group_max_passes"),
            _positive_integer(
                convergence.get("cli_max_passes"),
                "convergence.cli_max_passes")),
        timing=TimingPolicy(
            total_budget,
            planning_budget,
            _positive_number(
                timing.get("interactive_cancellation_grace_seconds"),
                "timing.interactive_cancellation_grace_seconds",
                allow_zero=True),
            cli_budget),
        safety=SafetyPolicy(single_track_drc))


CONFIG = load_internal_config()
_SESSION_CONFIG = CONFIG


def get_session_config():
    """Return the policy currently used by both interactive ActionPlugins."""
    return _SESSION_CONFIG


def update_session_config(*, minimum_saved_length_mm,
                          interactive_group_max_passes,
                          interactive_total_time_budget_seconds,
                          interactive_planning_time_budget_seconds,
                          interactive_cancellation_grace_seconds,
                          kicad_drc_for_single_track):
    """Validate and install process-local values; never writes the JSON file."""
    global _SESSION_CONFIG
    minimum = _positive_number(
        minimum_saved_length_mm, "minimum saved length", allow_zero=True)
    group_passes = _positive_integer(
        interactive_group_max_passes, "interactive group maximum passes")
    total_budget = _positive_number(
        interactive_total_time_budget_seconds, "interactive total time budget")
    planning_budget = _positive_number(
        interactive_planning_time_budget_seconds,
        "interactive planning time budget")
    cancellation_grace = _positive_number(
        interactive_cancellation_grace_seconds,
        "interactive cancellation grace", allow_zero=True)
    if planning_budget > total_budget:
        raise ValueError(
            "interactive planning time budget cannot exceed total time budget")
    if not isinstance(kicad_drc_for_single_track, bool):
        raise ValueError("single-track KiCad DRC must be a boolean")
    current = _SESSION_CONFIG
    _SESSION_CONFIG = replace(
        current,
        gloss=replace(
            current.gloss, minimum_saved_length_mm=minimum),
        convergence=replace(
            current.convergence,
            interactive_group_max_passes=group_passes),
        timing=replace(
            current.timing,
            interactive_total_time_budget_seconds=total_budget,
            interactive_planning_time_budget_seconds=planning_budget,
            interactive_cancellation_grace_seconds=cancellation_grace),
        safety=replace(
            current.safety,
            kicad_drc_for_single_track=kicad_drc_for_single_track))
    return _SESSION_CONFIG


def reset_session_config():
    """Restore packaged defaults, primarily for tests and plugin reloads."""
    global _SESSION_CONFIG
    _SESSION_CONFIG = CONFIG
    return _SESSION_CONFIG
