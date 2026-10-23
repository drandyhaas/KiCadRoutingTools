#!/usr/bin/env python3
"""What a converge ledger row's `blocking` IS: one rule, shared (#1088).

`converge._score_key` (the ranking), `converge record` (the refusal),
`check_complete` and `run_watch` all decide what a `blocking` value means with
the two functions here. They live on the router side so the router can import
them without importing `py_placer` (`_placer_path`'s one-way rule -- the router
side does not import placement engines); converge re-exports them.
"""
from __future__ import annotations

import json
import math
import sys


def blocking_defect(b):
    """None when `b` is a count a verdict can rank (or null/absent); else WHY
    it is neither (#1071, #1075).

    A verdict ranks every lap on `blocking` and asks `blocking == 0` for a
    finished board, so the value must be a non-negative number. Anything else
    either breaks the ranking outright (a per-term dict: two different dicts
    compare with `<` and raise) or ranks wrong without a word (`false == 0`
    reads as a finished board, `"10" < "9"`, NaN never compares below
    anything so its half reads as plateaued).
    """
    if b is None:
        return None
    if isinstance(b, bool):
        return (f'the boolean {json.dumps(b)}, not a count (true would rank '
                f'as 1 and false as a finished board)')
    if not isinstance(b, (int, float)):
        kind = {dict: 'a JSON object', list: 'a JSON array',
                str: 'a string'}.get(type(b), type(b).__name__)
        try:
            text = json.dumps(b, sort_keys=True)     # the JSON it arrived as
        except (TypeError, ValueError):
            text = repr(b)
        text = text if len(text) <= 60 else text[:57] + '...'
        hint = {dict: ' -- a per-term breakdown belongs in `blocking_by`',
                str: ' -- strings compare letter by letter',
                }.get(type(b), '')
        return f'{kind} ({text}), not a number{hint}'
    # FLOATS only: an int is always finite, and `math.isfinite` converts its
    # argument to float -- a 400-digit JSON integer raised OverflowError here.
    if isinstance(b, float) and not math.isfinite(b):
        return f'{b!r}, which no board measures'
    if b < 0:
        return f'negative ({b!r}); a count of blockers cannot be below zero'
    # Past the float range: nothing measures that many blockers, and any
    # reader that takes `float(b)` raises OverflowError on such a row.
    # (int > float compares exactly, without converting.)
    if b > sys.float_info.max:
        return (f'an integer of {len(str(b))} digits, beyond any float, which '
                f'no board measures')
    return None


def blocking_value(b):
    """`b` as a rankable count, or None when it is null OR not a count.

    ONE rule for `converge._score_key` (the ranking), `converge record`
    (the refusal) and `check_complete` -- defined here, imported by all of
    them (#1088).
    """
    return None if b is None or blocking_defect(b) else b
