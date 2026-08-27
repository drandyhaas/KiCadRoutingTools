"""Anytime native-DRC recovery for independently planned connections."""

from __future__ import annotations

import time

from ..engine.workflow import combine_plans, plan_identity


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


__all__ = ("maximize_safe_native_connections",)
