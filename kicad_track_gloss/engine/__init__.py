"""KiCad-independent Track Gloss optimization engine."""

from .planner import (PlanningCancelled, generate_candidate_plans,
                      generate_converged_plan, smooth_selected_chains)
from .statistics import summarize_plan
from .terminals import (find_pad_terminal_targets, find_track_terminal_targets,
                        find_track_terminal_vertices)
from .workflow import (combine_plans, compose_compatible_connection_plans,
                       generate_connection_candidates,
                       generate_conservative_candidate, plan_identity,
                       plan_net_ids, rank_candidate_plans)

__all__ = (
    "find_pad_terminal_targets",
    "find_track_terminal_targets",
    "find_track_terminal_vertices",
    "combine_plans",
    "compose_compatible_connection_plans",
    "generate_candidate_plans",
    "generate_connection_candidates",
    "generate_conservative_candidate",
    "generate_converged_plan",
    "PlanningCancelled",
    "plan_identity",
    "plan_net_ids",
    "rank_candidate_plans",
    "smooth_selected_chains",
    "summarize_plan",
)
