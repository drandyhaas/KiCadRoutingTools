"""KiCad-independent Track Gloss optimization engine."""

from .planner import (generate_candidate_plans, generate_converged_plan,
                      smooth_selected_chains)
from .statistics import summarize_plan
from .terminals import (find_pad_terminal_targets, find_track_terminal_targets,
                        find_track_terminal_vertices)

__all__ = (
    "find_pad_terminal_targets",
    "find_track_terminal_targets",
    "find_track_terminal_vertices",
    "generate_candidate_plans",
    "generate_converged_plan",
    "smooth_selected_chains",
    "summarize_plan",
)
