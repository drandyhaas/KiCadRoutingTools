"""KiCad-independent Track Gloss optimization engine."""

from .planner import (PlanningCancelled, generate_candidate_plans,
                      generate_converged_plan, smooth_selected_chains)
from .statistics import summarize_plan
from .terminals import (find_pad_terminal_targets, find_track_terminal_targets,
                        find_track_terminal_vertices)
from .workflow import (generate_conservative_candidate, plan_identity,
                       plan_net_gain, plan_net_ids, subset_plan_by_nets)

__all__ = (
    "find_pad_terminal_targets",
    "find_track_terminal_targets",
    "find_track_terminal_vertices",
    "generate_candidate_plans",
    "generate_conservative_candidate",
    "generate_converged_plan",
    "PlanningCancelled",
    "plan_identity",
    "plan_net_gain",
    "plan_net_ids",
    "smooth_selected_chains",
    "summarize_plan",
    "subset_plan_by_nets",
)
