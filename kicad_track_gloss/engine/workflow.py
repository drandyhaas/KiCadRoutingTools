"""Shared planning policies used by the interactive plugin and the CLI.

The front ends deliberately keep different time and convergence budgets, but
they must build equivalent candidates from equivalent scopes. Keeping the
candidate ladder here prevents those policies from drifting apart.
"""

from __future__ import annotations

from .planner import generate_converged_plan


def plan_identity(plan):
    """Return a stable, front-end-neutral identity for a composed plan."""
    additions = tuple(sorted((
        addition.start, addition.end, round(addition.width, 6),
        addition.layer, addition.net_id) for addition in plan.additions))
    return tuple(sorted(plan.remove_keys)), additions


def generate_conservative_candidate(
        model, eligible_keys, *, min_gain, clearance, deadline=None,
        cancellation_grace_seconds=1.0, collect_statistics=False,
        pass_observer=None, cancel_check=None):
    """Build the common one-pass fallback used after a native DRC rejection."""
    return generate_converged_plan(
        model, eligible_keys, max_passes=1, return_partial_on_limit=True,
        group_max_passes=1, max_refinement_passes=0,
        _allow_junction_scopes=False, min_gain=min_gain,
        allow_equal_length_simpler=True, clearance=clearance,
        # Large conservative retries are still independent per net/layer.
        # Reuse the same bounded group workers as the primary planner; small
        # interactive selections stay in-process below the planner threshold.
        collect_statistics=collect_statistics, parallel=True,
        pass_observer=pass_observer, cancel_check=cancel_check,
        deadline=deadline,
        cancellation_grace_seconds=cancellation_grace_seconds)


__all__ = ("generate_conservative_candidate", "plan_identity")
