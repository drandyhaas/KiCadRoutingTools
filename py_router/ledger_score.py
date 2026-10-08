#!/usr/bin/env python3
"""What a converge ledger row SAYS about its board, for the film (#1081).

The stage3d benchmark band draws one curve with two regimes: above a DONE
line, `blocking` falling toward it; below it, the board getting BETTER. This
module answers the three questions that needs, per row, without importing
`py_placer` (`_placer_path`'s one-way rule -- the router side does not import
placement engines):

  * `row_done(row)` -- is this row's board a fully working PCB? The DONE line
    is drawn at the first row that is.
  * `quality_key(score)` -- the ranking below the line: `(vias, copper_mm,
    segments)`, LEXICOGRAPHIC, the tie-break of `converge._score_key` read
    the same way, so a record the film draws is a record the run counts. It is
    never a weighted sum: a weighted sum lets a router buy off a disconnected
    net with a lower via count, and below the line it lets a board "improve"
    by trading one term for another the run's own ranking does not accept.
  * `deciding_term(prev, new)` -- WHICH term made a record a record, for its
    label (`copper -3.2 mm`): a lap that ties on vias and wins on copper is
    still progress, and a curve plotted on vias alone would draw it flat.

`tests/test_1081_benchmark_band.py` holds `quality_key` to
`converge._score_key`'s quality half on a shuffled ledger, so the two cannot
drift apart.
"""
from __future__ import annotations

import json
import math
import re
import sys
from typing import Optional, Tuple

#: The quality tuple, in `converge._score_key`'s order.
TERMS = ('vias', 'copper_mm', 'segments')
#: How a deciding term reads on a record label.
TERM_LABEL = {'vias': 'vias', 'copper_mm': 'copper', 'segments': 'segments'}
TERM_UNIT = {'vias': '', 'copper_mm': ' mm', 'segments': ''}

#: A verifier's verdict line, `converge._LENS_RE`'s grammar.
_LENS_RE = re.compile(r'^VERDICT=(PASS|FAIL):lens=([A-Za-z0-9_-]+)')

#: Kinds whose row grades a ROUTED board. A placement lap's `blocking` counts
#: what a routed result would still block on, but it is not a routed board:
#: the film never calls a placement lap a working PCB.
DONE_KINDS = ('completion', 'routing')
#: Kinds that are LAPS at all (placement and routing), `converge._HALF`'s.
LAP_KINDS = ('placement', 'completion', 'routing')


def is_lap(row) -> bool:
    """`converge._is_lap` for either half: a placement or routing row that
    is not a `--final` record of a verdict and not an `--exhausted`
    declaration -- neither changes a board, so neither is a turn of the loop
    the band should draw."""
    if not isinstance(row, dict):
        return False
    if str(row.get('kind') or '').lower() not in LAP_KINDS:
        return False
    if row.get('final'):
        return False
    return not isinstance(row.get('exhausted'), dict)


def _term(v):
    """One quality term EXACTLY as `_score_key` reads it: an int as it is
    (however large -- `float()` would overflow it), a finite float, else
    +inf (unmeasured ranks last, never first). A bool is not a count."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return math.inf
    if isinstance(v, int) or math.isfinite(v):
        return v
    return math.inf


def quality_key(score) -> Tuple[float, float, float]:
    """`(vias, copper_mm, segments)` of a score document; +inf per missing
    or non-numeric term. A non-dict score or quality is all +inf."""
    q = score.get('quality') if isinstance(score, dict) else None
    if not isinstance(q, dict):
        q = {}
    return tuple(_term(q.get(k)) for k in TERMS)


def is_inf(v) -> bool:
    """True for +/-inf. Safe on an int of any size (`math.isinf(10**400)`
    raises OverflowError)."""
    return isinstance(v, float) and math.isinf(v)


def plottable(v) -> bool:
    """True when `v` can be drawn: a number whose float is finite."""
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return False
    try:
        return math.isfinite(float(v))
    except OverflowError:
        return False


def deciding_term(prev, new) -> Optional[Tuple[str, float]]:
    """`(term, new - prev)` for the FIRST term the two keys differ on --
    the one the lexicographic ranking decided on -- or None when equal."""
    if prev is None or new is None:
        return None
    for name, a, b in zip(TERMS, prev, new):
        if a != b:
            if not (plottable(a) and plottable(b)):
                # unmeasured, or too large to subtract: the term decided it,
                # but only its DIRECTION can be printed honestly
                return (name, -math.inf if b < a else math.inf)
            return (name, float(b) - float(a))
    return None


def term_label(term) -> str:
    """`('copper_mm', -3.2)` -> `'copper -3.2 mm'`."""
    if not term:
        return ''
    name, d = term
    if is_inf(d):
        return '%s %s' % (TERM_LABEL.get(name, name),
                          'lower' if d < 0 else 'higher')
    num = ('%d' % d) if float(d).is_integer() else ('%.1f' % d)
    if d > 0:
        num = '+' + num
    return '%s %s%s' % (TERM_LABEL.get(name, name), num,
                        TERM_UNIT.get(name, ''))


def lens_verdicts(row):
    """`{lens: 'PASS'|'FAIL'}` from a row's `lenses` (case-folded); a line
    that is not a verdict is ignored. FAIL wins over PASS for one lens."""
    out = {}
    lenses = row.get('lenses') if isinstance(row, dict) else None
    if not isinstance(lenses, (list, tuple)):
        return out
    for raw in lenses:
        m = _LENS_RE.match(str(raw or '').strip())
        if not m:
            continue
        name = m.group(2).lower()
        if out.get(name) != 'FAIL':
            out[name] = m.group(1)
    return out


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
    # Past the float range: nothing measures that many blockers, and the film
    # plots `float(b)`, which raised OverflowError on a row `record` had
    # accepted. (int > float compares exactly, without converting.)
    if b > sys.float_info.max:
        return (f'an integer of {len(str(b))} digits, beyond any float, which '
                f'no board measures')
    return None


def blocking_value(b):
    """`b` as a rankable count, or None when it is null OR not a count.

    ONE rule for `converge._score_key` (the ranking), `converge record`
    (the refusal), `check_complete` and the film's axes
    (`movie_attempts`, `movie_benchmark`) -- defined here, imported by all
    of them (#1088).
    """
    return None if b is None or blocking_defect(b) else b


def _blocking(score):
    """`score.blocking` as a count, or None."""
    return blocking_value(score.get('blocking')
                          if isinstance(score, dict) else None)


def row_done(row) -> Optional[bool]:
    """True when this row's board is a FULLY WORKING PCB, False when the
    row says it is not, None when the row cannot say (not a routed kind,
    or no countable `blocking`).

    Working means: `blocking == 0`, nothing `unknown` (a component RAN and
    could not answer), no lens FAILed, and a score that is about THIS board
    (`score_stale` names a score that is not: #963).

    An `ungraded` LIST does not stop it, exactly as it does not stop
    `converge verdict`'s DONE: a board with no spec file has nothing to
    grade those components against (run 24 and run 19 shipped
    DONE-EXHAUSTED with floorplan and impedance named unexamined). It is
    never silent either: `unexamined(row)` names them and the band's chip
    counts them. An `ungraded` that is NOT a list names no component, and
    converge says to re-score before trusting anything -- so that, like a
    non-list `unknown`, is not working.

    An ABSENT lens is not a failure -- ordinary laps carry none; only a
    `--final` row is guaranteed to, which `done_evidence` reports apart. It
    is still a MEASUREMENT: the band says "measured" until a verifier has
    said "verified".
    """
    if not isinstance(row, dict):
        return None
    if str(row.get('kind') or '').lower() not in DONE_KINDS:
        return None
    score = row.get('score')
    b = _blocking(score)
    if b is None:
        return None
    if b != 0:
        return False
    unknown = score.get('unknown')
    if unknown not in (None, [], (), '', {}):
        return False                # any value: RAN and could not answer
    ung = score.get('ungraded')
    if ung not in (None, [], (), '', {}) and not isinstance(ung,
                                                            (list, tuple)):
        return False                # names no component (#1076)
    stale = row.get('score_stale')
    if isinstance(stale, dict) and stale.get('binding') in ('other',
                                                            'unbound'):
        return False
    if 'FAIL' in lens_verdicts(row).values():
        return False
    return True


def unexamined(row) -> list:
    """The components a row's score names as `ungraded`: a working board
    carries them as UNEXAMINED, never as passed."""
    sc = row.get('score') if isinstance(row, dict) else None
    ung = sc.get('ungraded') if isinstance(sc, dict) else None
    return (sorted(str(x) for x in ung)
            if isinstance(ung, (list, tuple)) else [])


def done_evidence(row) -> str:
    """`'verified'` when a DONE row is a `--final` row whose every lens PASSed
    (connectivity, drc and spec at least -- `record --final`'s own set), else
    `'measured'`: the numbers say done, no verifier has said so."""
    v = lens_verdicts(row)
    need = ('connectivity', 'drc', 'spec')
    if (isinstance(row, dict) and row.get('final')
            and all(v.get(k) == 'PASS' for k in need)
            and 'FAIL' not in v.values()):
        return 'verified'
    return 'measured'


if __name__ == '__main__':                                     # pragma: no cover
    for line in open(sys.argv[1], encoding='utf-8'):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        if isinstance(r, dict):
            print(r.get('iteration'), r.get('kind'), row_done(r),
                  quality_key(r.get('score')))
