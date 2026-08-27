"""Shared planning policies used by the interactive plugin and the CLI.

The front ends deliberately keep different time and convergence budgets, but
they must build equivalent candidates from equivalent scopes. Keeping the
candidate ladder here prevents those policies from drifting apart.
"""

from __future__ import annotations

import time

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


def rank_candidate_plans(plans):
    """Deduplicate and rank plans by the shared monotone quality objective."""
    unique = {}
    for plan in plans:
        unique.setdefault(plan_identity(plan), plan)
    return tuple(sorted(unique.values(), key=lambda plan: (
        -plan.angle_corrections, -round(plan.saved_mm, 9),
        -(len(plan.remove_keys) - len(plan.additions)),
        len(plan.additions), plan_identity(plan))))


def plan_net_ids(model, plan):
    """Return every net modified by a composed plan."""
    removed = set(plan.remove_keys)
    net_ids = {segment.net_id for segment in model.segments
               if segment_key(segment) in removed}
    net_ids.update(addition.net_id for addition in plan.additions)
    return tuple(sorted(net_ids))


def combine_plans(model, eligible_keys, plans):
    """Compose disjoint connection-local plans and run the complete gate.

    Local plans are generated from the same original board.  Their source
    scopes must therefore be disjoint; additions may still interact, so the
    ordinary clearance and connectivity validation remains authoritative on
    every composition.
    """
    plans = tuple(plans)
    segment_by_key = {segment_key(segment): segment
                      for segment in model.segments}
    result = GlossResult()
    for plan in plans:
        result.remove_keys.extend(plan.remove_keys)
        result.additions.extend(plan.additions)
        result.chains_considered += plan.chains_considered
        result.chains_changed += plan.chains_changed
        result.warnings.extend(plan.warnings)
        result.transformations.extend(plan.transformations)
        result.angle_corrections += plan.angle_corrections
        result.convergence_passes = max(
            result.convergence_passes, plan.convergence_passes)
        for key, value in plan.search_counts.items():
            result.search_counts[key] = result.search_counts.get(key, 0) + value
        for key, value in plan.blocking_nets.items():
            result.blocking_nets[key] = result.blocking_nets.get(key, 0) + value
    result.fixed_point = bool(plans) and all(plan.fixed_point for plan in plans)
    removed = set(result.remove_keys)
    removed_mm = sum(length(
        (segment_by_key[key].start_x, segment_by_key[key].start_y),
        (segment_by_key[key].end_x, segment_by_key[key].end_y))
        for key in removed if key in segment_by_key)
    added_mm = sum(length(item.start, item.end) for item in result.additions)
    result.saved_mm = max(0.0, removed_mm - added_mm)
    result.warnings = sorted(set(result.warnings))
    validate_result(model, set(eligible_keys), result,
                    check_connectivity=True)
    return result


def generate_connection_candidates(
        model, eligible_keys, connection_scopes, source_plan, *, min_gain,
        allow_equal_length_simpler, clearance, max_passes, group_max_passes,
        collect_statistics, planning_deadline, cancellation_grace_seconds):
    """Rebuild exact one-selection candidates for a larger selected scope.

    Scopes already modified by the global plan are visited first, then longer
    untouched connections. Every returned plan has independently passed the
    normal engine validation and addresses the original model directly.
    """
    eligible = set(eligible_keys)
    removed = set(source_plan.remove_keys)
    segment_by_key = {segment_key(segment): segment
                      for segment in model.segments}

    def copper(keys):
        return sum(length(
            (segment.start_x, segment.start_y),
            (segment.end_x, segment.end_y))
            for key in keys for segment in (segment_by_key.get(key),)
            if segment is not None)

    scopes = sorted(
        {frozenset(scope) & eligible for scope in connection_scopes
         if frozenset(scope) & eligible},
        key=lambda scope: (
            not bool(scope & removed), -round(copper(scope & removed), 9),
            -round(copper(scope), 9), tuple(sorted(scope))))
    plans = []
    seen = set()
    deadline_reached = False
    rejected = []
    for scope in scopes:
        if (planning_deadline is not None and
                time.monotonic() >= planning_deadline):
            deadline_reached = True
            break
        try:
            plan = generate_converged_plan(
                model, scope, max_passes=max_passes,
                return_partial_on_limit=True,
                batch_group_convergence=False,
                group_max_passes=group_max_passes,
                min_gain=min_gain,
                allow_equal_length_simpler=allow_equal_length_simpler,
                clearance=clearance,
                collect_statistics=collect_statistics,
                parallel=False, deadline=planning_deadline,
                cancellation_grace_seconds=cancellation_grace_seconds)
        except ValueError as error:
            rejected.append(str(error))
            continue
        except RuntimeError:
            if (planning_deadline is not None and
                    time.monotonic() >= planning_deadline):
                deadline_reached = True
                break
            raise
        if not plan.changed:
            continue
        identity = plan_identity(plan)
        if identity not in seen:
            seen.add(identity)
            plans.append(plan)
    plans.sort(key=lambda plan: (
        -plan.angle_corrections, -round(plan.saved_mm, 9),
        -(len(plan.remove_keys) - len(plan.additions)),
        len(plan.additions), plan_identity(plan)))
    return plans, rejected, deadline_reached


def compose_compatible_connection_plans(model, eligible_keys, plans):
    """Preserve the best compatible local connections with batch isolation."""
    selected = []
    rejected = []

    def extend(batch):
        if not batch:
            return
        try:
            combine_plans(model, eligible_keys, selected + list(batch))
        except ValueError as error:
            rejected.append(str(error))
            if len(batch) == 1:
                return
            midpoint = len(batch) // 2
            extend(batch[:midpoint])
            extend(batch[midpoint:])
            return
        selected.extend(batch)

    extend(tuple(plans))
    if not selected:
        return None, (), rejected
    return (combine_plans(model, eligible_keys, selected),
            tuple(selected), rejected)


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
    "combine_plans", "compose_compatible_connection_plans",
    "generate_connection_candidates",
    "generate_conservative_candidate", "plan_identity", "plan_net_ids",
    "rank_candidate_plans")
