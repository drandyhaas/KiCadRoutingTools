"""Shared planning policies used by the interactive plugin and the CLI.

The front ends deliberately keep different time and convergence budgets, but
they must build equivalent candidates from equivalent scopes. Keeping the
candidate ladder here prevents those policies from drifting apart.
"""

from __future__ import annotations

from .geometry import length
from .model import GlossResult, segment_key
from .planner import generate_converged_plan
from .validation import validate_result


def plan_identity(plan):
    """Return a stable, front-end-neutral identity for a composed plan."""
    additions = tuple(sorted((
        addition.start, addition.end, round(addition.width, 6),
        addition.layer, addition.net_id) for addition in plan.additions))
    return tuple(sorted(plan.remove_keys)), additions


def plan_net_ids(model, plan):
    """Return every net modified by a composed plan."""
    removed = set(plan.remove_keys)
    net_ids = {segment.net_id for segment in model.segments
               if segment_key(segment) in removed}
    net_ids.update(addition.net_id for addition in plan.additions)
    return tuple(sorted(net_ids))


def plan_net_gain(model, plan, net_id):
    """Return the exact signed copper gain contributed by one net."""
    removed = set(plan.remove_keys)
    removed_mm = sum(
        length((segment.start_x, segment.start_y),
               (segment.end_x, segment.end_y))
        for segment in model.segments
        if segment.net_id == net_id and segment_key(segment) in removed)
    added_mm = sum(length(addition.start, addition.end)
                   for addition in plan.additions
                   if addition.net_id == net_id)
    return removed_mm - added_mm


def subset_plan_by_nets(model, eligible_keys, plan, net_ids):
    """Build and fully validate the requested net subset of a composed plan."""
    net_ids = set(net_ids)
    segment_by_key = {segment_key(segment): segment
                      for segment in model.segments}
    remove_keys = [key for key in plan.remove_keys
                   if key in segment_by_key and
                   segment_by_key[key].net_id in net_ids]
    additions = [addition for addition in plan.additions
                 if addition.net_id in net_ids]
    transformations = [item for item in plan.transformations
                       if item.net_id in net_ids]
    removed_mm = sum(length(
        (segment_by_key[key].start_x, segment_by_key[key].start_y),
        (segment_by_key[key].end_x, segment_by_key[key].end_y))
        for key in remove_keys)
    added_mm = sum(length(item.start, item.end) for item in additions)

    def non_octolinear(segment):
        dx = abs(segment.end_x - segment.start_x)
        dy = abs(segment.end_y - segment.start_y)
        return dx > 1e-6 and dy > 1e-6 and abs(dx - dy) > 1e-6

    result = GlossResult(
        remove_keys=remove_keys,
        additions=additions,
        saved_mm=max(0.0, removed_mm - added_mm),
        chains_considered=plan.chains_considered,
        chains_changed=len(transformations),
        warnings=list(plan.warnings),
        transformations=transformations,
        search_counts=dict(plan.search_counts),
        blocking_nets=dict(plan.blocking_nets),
        angle_corrections=sum(
            non_octolinear(segment_by_key[key]) for key in remove_keys),
        convergence_passes=plan.convergence_passes,
        fixed_point=plan.fixed_point)
    validate_result(model, set(eligible_keys), result,
                    check_connectivity=True)
    return result


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


__all__ = (
    "generate_conservative_candidate", "plan_identity", "plan_net_gain",
    "plan_net_ids", "subset_plan_by_nets")
