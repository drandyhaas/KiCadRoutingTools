"""Anytime native-DRC recovery for independently planned connections."""

from __future__ import annotations

import time
from dataclasses import dataclass

from ..engine.workflow import (combine_plans, plan_identity,
                               rank_candidate_plans)


@dataclass
class NativeCandidateDecision:
    """Best native-approved plan retained throughout an anytime search."""

    plan: object = None
    native: object = None
    primary_native: object = None
    fallback_used: bool = False
    salvage_used: bool = False
    salvage_attempts: int = 0
    salvage_deadline: bool = False
    connections_retained: int = 0
    connections_planned: int = 0
    initial_portfolio: bool = False
    followup_validations: int = 0
    initial_validation_ms: float = 0.0
    followup_validation_ms: float = 0.0
    salvage_ms: float = 0.0


def _is_better_plan(candidate, incumbent):
    """Return whether candidate outranks an existing approved incumbent."""
    if candidate is None:
        return False
    if incumbent is None:
        return True
    if plan_identity(candidate) == plan_identity(incumbent):
        return False
    ranked = rank_candidate_plans((candidate, incumbent))
    return plan_identity(ranked[0]) == plan_identity(candidate)


def maximize_safe_native_candidates(
        adapter, board, model, eligible_keys, planning_candidates, *,
        conservative_plan, connection_plans, force_native, skip_native,
        operation_deadline, wait_callback):
    """Validate diverse candidates while never losing an approved incumbent.

    Pure geometric rank cannot predict native DRC validity.  The first wave
    therefore pairs the best geometric candidate with the conservative plan,
    when distinct, instead of discarding that safety anchor behind a fixed
    top-N cutoff.  Remaining higher-quality alternatives and connection-local
    salvage may improve the incumbent, but a timeout can only return the best
    plan already approved by KiCad.
    """
    ranked = list(rank_candidate_plans(planning_candidates))
    decision = NativeCandidateDecision()
    if not ranked:
        return decision

    primary = ranked[0]
    initial = [primary]
    conservative_identity = (
        plan_identity(conservative_plan)
        if conservative_plan is not None and conservative_plan.changed else
        None)
    if (conservative_identity is not None and
            conservative_identity != plan_identity(primary)):
        initial.append(conservative_plan)
    elif len(ranked) > 1:
        initial.append(ranked[1])
    initial_identities = {plan_identity(plan) for plan in initial}

    remaining = (None if operation_deadline is None else
                 max(0.0, operation_deadline - time.monotonic()))
    stage_started = time.monotonic()
    if len(initial) > 1:
        results = adapter.validate_plan_ladder(
            board, initial, force_native=force_native,
            skip_native=skip_native, timeout_seconds=remaining,
            wait_callback=wait_callback)
        decision.initial_portfolio = True
    else:
        results = [adapter.validate_plan(
            board, initial[0], force_native=force_native,
            skip_native=skip_native, timeout_seconds=remaining,
            wait_callback=wait_callback)]
    decision.initial_validation_ms = (
        time.monotonic() - stage_started) * 1000.0
    decision.primary_native = results[0]
    last_native = results[0]
    for plan, native in zip(initial, results):
        if native.validation_mode == "not_needed":
            continue
        last_native = native
        if native.allowed and _is_better_plan(plan, decision.plan):
            decision.plan = plan
            decision.native = native

    # Infrastructure errors and timeouts are terminal.  A previously approved
    # incumbent remains usable; without one, preserve the error for fail-closed
    # reporting by the caller.
    terminal_error = next((result for result in results
                           if result.error or
                           result.validation_mode == "native_timeout"), None)
    if terminal_error is not None:
        if decision.native is None:
            decision.native = terminal_error
        decision.fallback_used = (
            decision.plan is not None and
            plan_identity(decision.plan) != plan_identity(primary))
        return decision

    # Only candidates capable of improving the incumbent need another native
    # validation.  This is normally the connection composition that ranked
    # between the aggressive and conservative plans.
    if decision.plan is not None:
        for candidate in ranked:
            if plan_identity(candidate) in initial_identities:
                continue
            if not _is_better_plan(candidate, decision.plan):
                continue
            if (operation_deadline is not None and
                    time.monotonic() >= operation_deadline):
                break
            remaining = (None if operation_deadline is None else
                         max(0.0, operation_deadline - time.monotonic()))
            stage_started = time.monotonic()
            native = adapter.validate_plan(
                board, candidate, force_native=force_native,
                skip_native=skip_native, timeout_seconds=remaining,
                wait_callback=wait_callback)
            decision.followup_validation_ms += (
                time.monotonic() - stage_started) * 1000.0
            decision.followup_validations += 1
            last_native = native
            if native.allowed and _is_better_plan(candidate, decision.plan):
                decision.plan = candidate
                decision.native = native
            if native.error or native.validation_mode == "native_timeout":
                if decision.native is None:
                    decision.native = native
                break

    # A rejected full local composition is only an upper bound.  Its safe
    # subsets may still beat the incumbent, so use the remaining budget while
    # retaining the approved plan as a floor.
    connection_upper = next((
        candidate for candidate in ranked
        if connection_plans and
        set(plan_identity(candidate)[0]) == set().union(*(
            set(plan.remove_keys) for plan in connection_plans))), None)
    should_salvage = bool(connection_plans) and (
        decision.plan is None or connection_upper is None or
        _is_better_plan(connection_upper, decision.plan))
    if (should_salvage and
            (operation_deadline is None or
             time.monotonic() < operation_deadline)):
        stage_started = time.monotonic()
        (partial_plan, partial_native, decision.salvage_attempts,
         decision.salvage_deadline, retained, total) = \
            maximize_safe_native_connections(
                adapter, board, model, eligible_keys, connection_plans,
                force_native=force_native, skip_native=skip_native,
                operation_deadline=operation_deadline,
                wait_callback=wait_callback)
        decision.salvage_ms = (
            time.monotonic() - stage_started) * 1000.0
        if (partial_plan is not None and partial_native is not None and
                partial_native.allowed and
                _is_better_plan(partial_plan, decision.plan)):
            decision.plan = partial_plan
            decision.native = partial_native
            decision.salvage_used = True
            decision.connections_retained = retained
            decision.connections_planned = total

    if decision.native is None:
        decision.native = last_native
    decision.fallback_used = (
        decision.plan is not None and not decision.salvage_used and
        plan_identity(decision.plan) != plan_identity(primary))
    return decision


def maximize_safe_native_connections(
        adapter, board, model, eligible_keys, connection_plans, *,
        force_native, skip_native, operation_deadline, wait_callback):
    """Return the best connection composition validated before the deadline.

    Before a safe base exists, two local plans are probed in one portfolio
    wave. Afterwards the primary candidate contains every remaining plan and
    the fallback adds only the next plan. A broad success ends immediately;
    a rejection can still extend the already safe base in the same DRC wave.
    """
    pending = list(connection_plans)
    accepted = []
    best_plan = None
    best_native = None
    attempts = 0
    deadline_reached = False
    chunks = []

    def compose(plans):
        try:
            return combine_plans(model, eligible_keys, plans)
        except ValueError:
            return None

    while pending:
        if (operation_deadline is not None and
                time.monotonic() >= operation_deadline):
            deadline_reached = True
            break
        candidates = []
        meanings = []
        if accepted:
            batch = compose(accepted + pending)
            if batch is not None:
                candidates.append(batch)
                meanings.append(("batch", tuple(pending)))
            if not chunks:
                midpoint = max(1, len(pending) // 2)
                chunks = [tuple(pending[:midpoint])]
                if midpoint < len(pending):
                    chunks.append(tuple(pending[midpoint:]))
            chunk = chunks.pop(0)
            incremental = compose(accepted + list(chunk))
            if (incremental is not None and
                    all(plan_identity(incremental) != plan_identity(item)
                        for item in candidates)):
                candidates.append(incremental)
                meanings.append(("chunk", chunk))
        else:
            for plan in pending[:2]:
                candidate = compose([plan])
                if candidate is not None:
                    candidates.append(candidate)
                    meanings.append(("one", plan))
        if not candidates:
            pending.pop(0)
            continue
        remaining = (None if operation_deadline is None else
                     operation_deadline - time.monotonic())
        if remaining is not None and remaining <= 0.0:
            deadline_reached = True
            break
        if len(candidates) == 2:
            native_results = adapter.validate_plan_ladder(
                board, candidates, force_native=force_native,
                skip_native=skip_native, timeout_seconds=remaining,
                wait_callback=wait_callback)
        else:
            native_results = [adapter.validate_plan(
                board, candidates[0], force_native=force_native,
                skip_native=skip_native, timeout_seconds=remaining,
                wait_callback=wait_callback)]
        attempts += sum(result.validation_mode != "not_needed"
                        for result in native_results)
        accepted_index = next(
            (index for index, result in enumerate(native_results)
             if result.allowed), None)
        if accepted_index is not None:
            kind, payload = meanings[accepted_index]
            best_plan = candidates[accepted_index]
            best_native = native_results[accepted_index]
            if kind == "batch":
                accepted.extend(payload)
                pending.clear()
                break
            additions = (payload if kind == "chunk" else (payload,))
            accepted.extend(additions)
            for plan in additions:
                if plan in pending:
                    pending.remove(plan)
            chunks = []
            continue

        tested_units = [(kind, payload) for kind, payload in meanings
                        if kind in ("one", "chunk")]
        if not tested_units and len(pending) == 1:
            pending.pop()
        for kind, payload in tested_units:
            plans = (payload if kind == "chunk" else (payload,))
            if kind == "chunk" and len(plans) > 1:
                midpoint = len(plans) // 2
                chunks[0:0] = [plans[:midpoint], plans[midpoint:]]
                continue
            for plan in plans:
                if plan in pending:
                    pending.remove(plan)
        errors = [result for result in native_results if result.error]
        if errors:
            deadline_reached = any(
                result.validation_mode == "native_timeout"
                for result in errors)
            break

    if (operation_deadline is not None and
            time.monotonic() >= operation_deadline):
        deadline_reached = True
    return (best_plan, best_native, attempts, deadline_reached,
            len(accepted), len(connection_plans))


__all__ = (
    "NativeCandidateDecision", "maximize_safe_native_candidates",
    "maximize_safe_native_connections")
