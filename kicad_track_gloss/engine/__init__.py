"""KiCad-independent Track Gloss optimization engine."""

from .planner import generate_candidate_plans, smooth_selected_chains
from .terminals import find_track_terminal_targets, find_track_terminal_vertices

__all__ = (
    "find_track_terminal_targets",
    "find_track_terminal_vertices",
    "generate_candidate_plans",
    "smooth_selected_chains",
)
