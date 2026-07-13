"""Process-scoped ledger of the smallest copper clearance any routing step
actually used this run.

Why this exists
---------------
Some routing steps escalate BELOW the nominal ``--clearance`` to fit tight
geometry -- most visibly the fine-pitch plane tap (``plane_pad_tap.py``), which
drops to ~0.15 mm to thread between 0.5 mm-pitch IC pads. If the board is then
graded (``check_drc``) or its KiCad DRC floor (``.kicad_pro``) is set at the
nominal clearance, that legitimately-tight copper reads as a violation -- the
phantom-violation trap that motivated this module.

The fix is to grade at the *minimum clearance actually used in any step*. Each
``main()`` parses its own ``PCBData`` separate from the engine's internal copy,
so a board-attribute accumulator can't bridge engine -> main. This module is
that bridge: deep sub-nominal sites call :func:`record`, and each entrypoint's
``main()`` reads :func:`get_min` once to feed ``fix_project_for_output`` (the
``.kicad_pro`` floor, which the next step and ``check_drc`` read back) and the
``JSON_SUMMARY``.

Scope / lifetime
----------------
Process-scoped and single-threaded: one CLI invocation = one run. Long-lived
callers that route repeatedly in one process (the KiCad GUI plugin) must call
:func:`reset` at the start of each operation so an earlier board's tight
clearance does not leak into a later one. Recording is monotonic (keeps the
minimum), so order does not matter within a run.
"""
from __future__ import annotations

from contextlib import contextmanager
from typing import Optional

_min_clearance: Optional[float] = None


def record(clearance: Optional[float]) -> None:
    """Note that a routing step used ``clearance`` mm of copper clearance.

    Keeps the running minimum. Ignores ``None`` and non-positive values."""
    global _min_clearance
    if clearance is None or clearance <= 0:
        return
    if _min_clearance is None or clearance < _min_clearance:
        _min_clearance = clearance


def get_min() -> Optional[float]:
    """The smallest clearance recorded this run, or ``None`` if nothing was
    recorded (in which case callers should fall back to their nominal clearance)."""
    return _min_clearance


def effective(nominal: float) -> float:
    """``min(nominal, recorded_min)`` -- the clearance the board should be graded
    at: never looser than nominal, lowered to any tighter clearance a step used."""
    m = _min_clearance
    return nominal if m is None else min(nominal, m)


def reset() -> None:
    """Clear the ledger. Call at the start of each independent routing operation
    in a long-lived process (e.g. the GUI plugin)."""
    global _min_clearance
    _min_clearance = None


@contextmanager
def fresh_run():
    """Scope one independent routing operation in a long-lived process (#382 E10).

    Resets the ledger on ENTRY so an earlier board's tight clearance can't leak
    into this operation, then leaves the newly-recorded minimum intact on exit
    (it is NOT reset when the block ends) -- callers read :func:`get_min` /
    :func:`effective` after the engine call, so the value must survive the block.

    Replaces the scattered manual ``clearance_ledger.reset()`` calls in the GUI
    actions (planes / diff-pair / route tabs) with one self-documenting guard::

        with clearance_ledger.fresh_run():
            ... run the engine ...
        floor = clearance_ledger.get_min()
    """
    reset()
    yield
