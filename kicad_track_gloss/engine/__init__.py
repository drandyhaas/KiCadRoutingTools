"""KiCad-independent Track Gloss optimization engine."""

from .planner import (generate_candidate_plans, generate_converged_plan,
                      smooth_selected_chains)
from .statistics import summarize_plan
from .terminals import (find_pad_terminal_targets, find_track_terminal_targets,
                        find_track_terminal_vertices)
from .workflow import generate_conservative_candidate, plan_identity

__all__ = (
    "find_pad_terminal_targets",
    "find_track_terminal_targets",
    "find_track_terminal_vertices",
    "generate_candidate_plans",
    "generate_conservative_candidate",
    "generate_converged_plan",
    "plan_identity",
    "smooth_selected_chains",
    "summarize_plan",
)
