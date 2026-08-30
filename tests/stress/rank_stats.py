#!/usr/bin/env python3
"""Rank statistics for the #703 predictor study, with the pooling trap designed out.

Why this file exists
--------------------
Every correlation number quoted in this repo's skills and drivers is measured
against *distance-to-the-correct-placement* or against *the gap a human left*,
never against routed ``blocking`` -- which CLAUDE.md's "What a placement run is
FOR" names as the only right headline. #703 is the measurement. This module is
its arithmetic, and nothing else: no I/O, no repo imports, no board parsing, so
it loads in milliseconds and every branch is reachable from a unit test.

Pure python, no scipy, deliberately
-----------------------------------
KiCad's bundled python has no scipy (``py_router/startup_checks.py``), and the
whole placement toolchain treats it as optional at runtime. A statistics layer
that only works under the system interpreter would be unavailable exactly where
a GUI-side consumer would want it. The functions here are twenty lines each.

THE POOLING TRAP, AND WHY IT IS DESIGNED OUT RATHER THAN DOCUMENTED
------------------------------------------------------------------
Pooling one placement per board across boards measures *board size*, not
placement quality. This is not a hypothetical: on the recorded corpus,
rho(crossings, vias) is **+0.714 pooled across six boards** and **-0.400 within
the one recorded slate of one board** -- opposite signs, same two quantities.

So no function here will compute a correlation over rows from more than one
board. ``per_board`` is the way in; ``board_rho`` and ``classify_board`` call
``_one_board`` and REFUSE a pile spanning several ``board_key`` values; and
``sign_test`` takes a *mapping* keyed by board and refuses a sequence. A caller
who wants a pooled number has to strip the board keys first, which is a thing
you cannot do by accident.

The first version of this file only *claimed* that, and an adversarial review
falsified it in one line: ``board_rho`` never read ``board_key``, so handing it
the six recorded runs returned ``rho=+0.339 [LOO +0.053..+0.632, K=6]`` -- the
exact pooled number this docstring calls the trap. The introspection check that
was supposed to catch it whitelisted ``board_rho`` and ``classify_board`` by
name, so it could not fail. A guard that is documented and not executed is the
same shape of defect as an r-value quoted without its dependent variable, which
makes it a poor thing to leave in this particular file.

RHO DOES NOT REACH A READER WITHOUT ITS SCOPE
---------------------------------------------
The failure #703 is about is a number getting quoted without its scope: nine
sites in this repo cite ``r = +0.780`` and none of them says the dependent
variable was distance-to-truth. So every rendering built for a HUMAN carries the
leave-one-out span and K in one atomic token:

    rho=+0.339 [LOO +0.053..+0.632, K=6]

``fmt_rho`` is that renderer, ``BoardRho.__repr__`` delegates to it, and
``BoardRho.as_dict()['display']`` carries it into any document generated from
the rows. Copy-pasting the token carries the instability with it: the LOO span
on the recorded corpus is +0.053..+0.632 around a headline of +0.339, so
dropping any single board moves it by 12x, and that instability IS the finding.

What this does NOT claim, because an earlier draft did and it was false:
``fmt`` is a general number formatter and will happily print a bare float, and
``as_dict()['rho']`` is deliberately a bare number so a machine can read it.
The guarantee is about the human-facing renderings, not about arithmetic being
unable to produce a float. ``fmt_rho`` called without a span says
``[LOO not computed]`` rather than falling silent about it.

p-VALUES: TWO-SIDED, AND WHY THE RESEARCH NOTE'S 0.11 IS NOT REPRODUCED
-----------------------------------------------------------------------
``wk/research-placement/10-routability-predictors.md`` quotes "6/6 -> p = 0.031;
5/6 -> p = 0.11". Those are two different tests: 0.031 is the TWO-sided sign
test at 6-of-6, and 0.11 is the ONE-sided value at 5-right-1-wrong (the
two-sided value there is 0.219). This module reports both, labelled, and takes
the two-sided one as the headline -- a predictor's direction is not known in
advance, and an exploratory study over a dozen predictors that quotes one-sided
p-values is choosing its tail after seeing the data.
"""
from __future__ import annotations

import math
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

NAN = float('nan')

#: Rows with fewer than this many samples cannot produce a rank correlation at
#: all. Three is the arithmetic floor (two points are always perfectly
#: monotone); it is NOT a claim that three is enough to believe.
MIN_N = 3

#: A leave-one-out span needs each subset to still clear MIN_N.
MIN_N_FOR_LOO = MIN_N + 1

#: Boards required before `sign_test` will report `passes_sign_rule`. Same
#: value and same reason as `test_placement_ab.MIN_TRIAL_BOARDS`: a term on
#: trial is judged on at least three DISTINCT boards. See `sign_test` for the
#: shuffle-control measurement that forced it.
MIN_SIGN_BOARDS = 3


class StatsRefusal(ValueError):
    """A named refusal, raised where a bare TypeError would read as a crash.

    The specific case this exists for: a predictor column containing ``None``.
    Python 3 raises ``TypeError: '<' not supported between instances of
    'NoneType' and 'int'`` from deep inside ``sorted``, which reads as a bug in
    the ranking rather than as "this row has no measurement for this key".
    """


# ---------------------------------------------------------------------------
# ranks and rho
# ---------------------------------------------------------------------------

def rank(vals: Sequence[float]) -> List[float]:
    """Average ranks, ties sharing the mean of the positions they occupy.

    Ties are detected BY POSITION in the sorted order, never by a tolerance.
    ``sorted()`` on a non-transitive key is undefined behaviour, and an
    "almost equal" tie rule is exactly such a key: with a tolerance t, a and b
    can tie, b and c can tie, and a and c not -- so which values share a rank
    depends on the order the comparisons happened to run in.
    """
    for v in vals:
        if v is None:
            raise StatsRefusal(
                'rank() got a None in the column. A missing measurement is not '
                'a value: give the row an explicit null and exclude it, or '
                'record why it is absent -- do not rank it.')
        if isinstance(v, float) and v != v:
            raise StatsRefusal(
                'rank() got a NaN in the column. NaN has no position in a sort '
                'order, so ranking it silently invents one.')
    order = sorted(range(len(vals)), key=lambda i: vals[i])
    out = [0.0] * len(vals)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and vals[order[j + 1]] == vals[order[i]]:
            j += 1
        avg = (i + j) / 2.0 + 1
        for k in range(i, j + 1):
            out[order[k]] = avg
        i = j + 1
    return out


def spearman(a: Sequence[float], b: Sequence[float]) -> float:
    """Tie-corrected Spearman rho, or NaN when the question is not answerable.

    NaN and NOT 0.0 when either side is constant or n < MIN_N. 0.0 reads as
    "measured, and there is no relationship"; the truth is "this cannot be
    measured here". The repo has been bitten by the general form of this before
    -- ``test_placement_ab.py`` refuses ``or 0.0`` on a missing metric because a
    key the optimizer stopped reporting would be recorded as a real measurement
    of zero.
    """
    if len(a) != len(b) or len(a) < MIN_N:
        return NAN
    ra, rb = rank(a), rank(b)
    n = len(a)
    ma, mb = sum(ra) / n, sum(rb) / n
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    da = math.sqrt(sum((x - ma) ** 2 for x in ra))
    db = math.sqrt(sum((y - mb) ** 2 for y in rb))
    return num / (da * db) if da and db else NAN


def constant_side(a: Sequence[float], b: Sequence[float]) -> Optional[str]:
    """Which side has no dynamic range -- 'predictor', 'dependent', 'both', None.

    A constant PREDICTOR and a constant DEPENDENT are different facts and the
    study reports them in different buckets. A predictor that never varies on a
    board is a finding about the predictor (``check_pockets`` produces exactly
    this: an empty ranked list on three of five real boards). A dependent that
    never varies is saturation -- every variant routed clean, so the board has
    no headroom to rank. ``spearman`` returns NaN for both and cannot say which.
    """
    ca = len(set(a)) <= 1
    cb = len(set(b)) <= 1
    if ca and cb:
        return 'both'
    if ca:
        return 'predictor'
    if cb:
        return 'dependent'
    return None


def tie_count(vals: Sequence[float]) -> int:
    """How many samples sit in a tie group. 15 ties in 20 is a rank-degenerate
    rho that the headline number does not show on its own."""
    seen: Dict[float, int] = {}
    for v in vals:
        seen[v] = seen.get(v, 0) + 1
    return sum(c for c in seen.values() if c > 1)


def leave_one_out(a: Sequence[float], b: Sequence[float]) -> List[float]:
    """rho with each sample dropped in turn. Empty when n is too small to try."""
    if len(a) != len(b) or len(a) < MIN_N_FOR_LOO:
        return []
    out = []
    for drop in range(len(a)):
        sa = [v for i, v in enumerate(a) if i != drop]
        sb = [v for i, v in enumerate(b) if i != drop]
        out.append(spearman(sa, sb))
    return out


def loo_span(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float]:
    """(min, max) of the leave-one-out rhos, or (NaN, NaN).

    A span that CROSSES ZERO means one sample decides the sign of the finding.
    On the recorded corpus rho(crossings, blocking) = +0.339 spans
    +0.053..+0.632 at n=6: it does not cross zero, but dropping one board moves
    it by 12x, which is why n=6 pooled is not a measurement.
    """
    vals = [v for v in leave_one_out(a, b) if v == v]
    if not vals:
        return (NAN, NAN)
    return (min(vals), max(vals))


# ---------------------------------------------------------------------------
# Kendall tau -- added for #789, which asks a different question from rho
# ---------------------------------------------------------------------------
#
# rho asks "does this predictor track the outcome"; #789 asks "does
# `portfolio.rank_key`'s ORDER agree with the routed order over the same
# candidates", which is a statement about PAIRS -- for how many pairs does the
# key put the better-routing candidate first. tau is the coefficient with that
# reading; rho has no pair-level interpretation.
#
# tau-B is the coefficient, and the tie correction is load-bearing rather than
# pedantic. The routed side of #789 is `board_score`'s `blocking`, and on the
# recorded corpus a slate's ten candidates routed to `blocking` 0 on
# splitflap_driver and to {0 x9, 2} on sonde_u. With most of one side tied,
# tau-a's denominator counts pairs no measurement can order and drags the
# coefficient toward zero by construction. `tau_a` is returned beside tau-b
# anyway, because the GAP between them is how much tie the board carries --
# the same role `tie_count` plays beside rho.

#: Two points are always perfectly ordered, so tau needs the same floor as rho.
MIN_N_FOR_TAU = MIN_N


def tau_counts(a: Sequence[float], b: Sequence[float]) -> Dict[str, int]:
    """Concordant/discordant pairs and the tie structure both sides carry.

    Computed on ``rank()``'s output, not on the raw values, for two reasons.
    It reuses ``rank``'s refusal for a None or a NaN in the column -- one
    validation rule, not two that can drift apart -- and it is exactly
    equivalent: average ranks are a non-decreasing map that sends equal values
    to equal ranks, so every pair keeps its sign and every tie group keeps its
    membership. ``_self_test`` pins that equivalence rather than asserting it
    here.
    """
    ra, rb = rank(a), rank(b)
    n = len(ra)
    c = d = ta = tb = tboth = 0
    for i in range(n):
        for j in range(i + 1, n):
            da = ra[i] - ra[j]
            db = rb[i] - rb[j]
            if da == 0 and db == 0:
                tboth += 1
            elif da == 0:
                ta += 1
            elif db == 0:
                tb += 1
            elif (da > 0) == (db > 0):
                c += 1
            else:
                d += 1
    return {'concordant': c, 'discordant': d, 'ties_a': ta + tboth,
            'ties_b': tb + tboth, 'ties_both': tboth, 'n0': n * (n - 1) // 2}


def kendall_tau(a: Sequence[float], b: Sequence[float]) -> float:
    """Kendall tau-b, or NaN when the question is not answerable.

    NaN and NOT 0.0, for the reason ``spearman`` gives: 0.0 reads as "measured,
    and the orders are unrelated", where the truth is "this cannot be measured
    here". A constant side and a degenerate denominator both land here.
    """
    if len(a) != len(b) or len(a) < MIN_N_FOR_TAU:
        return NAN
    k = tau_counts(a, b)
    da = k['n0'] - k['ties_a']
    db = k['n0'] - k['ties_b']
    if da <= 0 or db <= 0:
        return NAN
    return (k['concordant'] - k['discordant']) / math.sqrt(da * db)


def tau_a(a: Sequence[float], b: Sequence[float]) -> float:
    """Kendall tau-a: the same numerator over the UNCORRECTED pair count.

    Reported beside tau-b, never instead of it. tau_a == tau_b exactly when
    neither side carries a tie, so the gap between them is a reading of how
    tied the board is.
    """
    if len(a) != len(b) or len(a) < MIN_N_FOR_TAU:
        return NAN
    k = tau_counts(a, b)
    return (k['concordant'] - k['discordant']) / k['n0'] if k['n0'] else NAN


def tau_leave_one_out(a: Sequence[float], b: Sequence[float]) -> List[float]:
    """tau-b with each sample dropped in turn. Empty when n is too small."""
    if len(a) != len(b) or len(a) < MIN_N_FOR_LOO:
        return []
    out = []
    for drop in range(len(a)):
        sa = [v for i, v in enumerate(a) if i != drop]
        sb = [v for i, v in enumerate(b) if i != drop]
        out.append(kendall_tau(sa, sb))
    return out


def tau_loo_span(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float]:
    """(min, max) of the leave-one-out tau-b values, or (NaN, NaN).

    Same reading as ``loo_span``: a span that CROSSES ZERO means one candidate
    decides the sign of the finding.
    """
    vals = [v for v in tau_leave_one_out(a, b) if v == v]
    if not vals:
        return (NAN, NAN)
    return (min(vals), max(vals))


# ---------------------------------------------------------------------------
# formatting -- there is no way to print a naked rho
# ---------------------------------------------------------------------------

def fmt(x: Optional[float], w: int = 7) -> str:
    """A NaN renders as ``n/a``, never as ``+0.000``."""
    if x is None:
        return f'{"none":>{w}}'
    return f'{x:+{w}.3f}' if x == x else f'{"n/a":>{w}}'


def fmt_rho(rho: float, lo: float = NAN, hi: float = NAN,
            k: Optional[int] = None, reason: Optional[str] = None) -> str:
    """The ONLY sanctioned rendering of a rho: one token carrying its scope.

    Deliberately not composable with a bare ``f'{rho:+.3f}'`` anywhere in the
    study, so a number cannot reach a document without the span and K that say
    how much to believe it.
    """
    if rho != rho:
        return f'rho=n/a [{reason or "not measurable"}' + (
            f', K={k}]' if k is not None else ']')
    span = (f'LOO {fmt(lo, 0)}..{fmt(hi, 0)}' if lo == lo and hi == hi
            else 'LOO not computed')
    kk = f', K={k}' if k is not None else ''
    return f'rho={rho:+.3f} [{span}{kk}]'


def fmt_tau(tau: float, lo: float = NAN, hi: float = NAN,
            k: Optional[int] = None, ties_a: int = 0, ties_b: int = 0,
            reason: Optional[str] = None) -> str:
    """The ONLY sanctioned rendering of a tau. Same contract as ``fmt_rho``.

    It carries the tie counts as well as the span, because tau-b's whole
    difference from tau-a is the tie correction: a reader who cannot see how
    tied the board was cannot tell a real +0.3 from a coefficient rescued by
    its denominator.

    The ``LOO`` token is deliberate -- it is already a recognised scope in
    ``tests/test_703_predictor_claims.py``, so a tau rendered through this
    function arrives in a document carrying its own scope rather than needing a
    prose clause beside it.
    """
    if tau != tau:
        return f'tau=n/a [{reason or "not measurable"}' + (
            f', K={k}]' if k is not None else ']')
    span = (f'LOO {fmt(lo, 0)}..{fmt(hi, 0)}' if lo == lo and hi == hi
            else 'LOO not computed')
    kk = f', K={k}' if k is not None else ''
    return (f'tau={tau:+.3f} [{span}{kk}, tied_a={ties_a}, tied_b={ties_b}]')


# ---------------------------------------------------------------------------
# the board-scoped API -- the only way to a rho
# ---------------------------------------------------------------------------

class BoardRho:
    """One board's answer, carrying everything needed to not over-read it."""

    __slots__ = ('rho', 'n', 'reason', 'loo_lo', 'loo_hi',
                 'ties_predictor', 'ties_dependent')

    def __init__(self, rho, n, reason=None, loo_lo=NAN, loo_hi=NAN,
                 ties_predictor=0, ties_dependent=0):
        self.rho = rho
        self.n = n
        self.reason = reason
        self.loo_lo = loo_lo
        self.loo_hi = loo_hi
        self.ties_predictor = ties_predictor
        self.ties_dependent = ties_dependent

    def __repr__(self):
        return fmt_rho(self.rho, self.loo_lo, self.loo_hi, self.n, self.reason)

    def as_dict(self) -> Dict:
        return {'rho': None if self.rho != self.rho else round(self.rho, 6),
                'n': self.n, 'reason': self.reason,
                'loo_lo': None if self.loo_lo != self.loo_lo
                else round(self.loo_lo, 6),
                'loo_hi': None if self.loo_hi != self.loo_hi
                else round(self.loo_hi, 6),
                'ties_predictor': self.ties_predictor,
                'ties_dependent': self.ties_dependent,
                'display': fmt_rho(self.rho, self.loo_lo, self.loo_hi,
                                   self.n, self.reason)}


def per_board(rows: Sequence[Mapping]) -> Dict[str, List[Mapping]]:
    """Group rows by ``board_key``. THE only entry point to a correlation.

    Rows without a ``board_key`` are refused rather than bucketed under None:
    a row whose board is unknown would silently join every other unknown row
    into a phantom board, which is pooling wearing a disguise.
    """
    out: Dict[str, List[Mapping]] = {}
    for i, r in enumerate(rows):
        key = r.get('board_key')
        if not key:
            raise StatsRefusal(
                f'row {i} ({r.get("row_id", "unnamed")}) has no board_key. '
                f'Every correlation here is computed WITHIN a board; a row that '
                f'cannot name its board cannot be ranked against anything.')
        out.setdefault(key, []).append(r)
    return out


def _one_board(rows: Sequence[Mapping], fn: str) -> Optional[str]:
    """Refuse a row pile spanning more than one board. Returns the board key.

    This is the anti-pooling guard, EXECUTED rather than documented. Handing
    the six recorded runs to ``board_rho`` used to return the pooled +0.339
    that this module's own docstring names as the trap, because nothing read
    ``board_key``. Rows with no ``board_key`` at all are allowed through, so a
    caller can still rank two bare columns for a unit test -- what is refused is
    the specific mistake of ranking several boards' rows as if they were one
    board's variants.
    """
    keys = {r.get('board_key') for r in rows if r.get('board_key')}
    if len(keys) > 1:
        # str() and sort BY str: a board_key is typed str, but a caller with
        # int or mixed keys would otherwise get `TypeError: sequence item 0:
        # expected str instance` out of the join -- the precise failure shape
        # StatsRefusal exists to replace. A guard that crashes instead of
        # refusing is the defect this module keeps finding in other people's
        # code. (Found by an adversarial review of this very fix.)
        raise StatsRefusal(
            f'{fn}() was given rows from {len(keys)} boards '
            f'({", ".join(str(k) for k in sorted(keys, key=str)[:6])}). '
            f'Every correlation here is '
            f'computed WITHIN one board: pooling across boards measures board '
            f'size, and on this repo\'s own corpus it flips the sign of '
            f'rho(crossings, vias) from -0.400 to +0.714. Use per_board() '
            f'first.')
    return next(iter(keys)) if keys else None


def _column(rows: Sequence[Mapping], group: str, key: str) -> List:
    return [(r.get(group) or {}).get(key) for r in rows]


def board_rho(rows: Sequence[Mapping], predictor: str, dependent: str,
              *, predictor_group: str = 'predictors',
              dependent_group: str = 'truth') -> BoardRho:
    """rho for ONE board's rows. Rows missing either value are dropped, named.

    Dropping is honest here and only here: a predictor that raised on one
    variant is recorded null (never 0), and a null cannot be ranked. The
    resulting n is reported, so a board whose effective K collapsed says so.
    """
    _one_board(rows, 'board_rho')
    pv, dv = [], []
    dropped = 0
    dropped_nan = 0
    for p, d in zip(_column(rows, predictor_group, predictor),
                    _column(rows, dependent_group, dependent)):
        if p is None or d is None:
            dropped += 1
            continue
        if isinstance(p, float) and p != p:
            dropped_nan += 1
            continue
        if isinstance(d, float) and d != d:
            dropped_nan += 1
            continue
        pv.append(p)
        dv.append(d)
    n = len(pv)
    _bits = ([f'{dropped} row(s) dropped for a null value'] if dropped else [])
    # A NaN value and an absent one are different facts: a null is "this
    # predictor was never computed here", a NaN is "it was computed and came
    # back undefined". Reporting both as "null" misnames the cause in the one
    # string a reader has to go on.
    _bits += ([f'{dropped_nan} row(s) dropped for a NaN value']
              if dropped_nan else [])
    note = (', ' + '; '.join(_bits)) if _bits else ''
    if n < MIN_N:
        return BoardRho(NAN, n, f'n={n} < {MIN_N}{note}')
    side = constant_side(pv, dv)
    if side == 'both':
        return BoardRho(NAN, n, f'predictor AND truth constant{note}')
    if side == 'predictor':
        return BoardRho(NAN, n, f'predictor constant (no dynamic range){note}')
    if side == 'dependent':
        return BoardRho(NAN, n, f'truth constant (saturated){note}')
    lo, hi = loo_span(pv, dv)
    return BoardRho(spearman(pv, dv), n,
                    (note[2:] if note else None), lo, hi,
                    tie_count(pv), tie_count(dv))


class BoardTau:
    """One board's tau, carrying everything needed to not over-read it.

    Mirrors ``BoardRho`` deliberately, including ``as_dict()['display']``, so a
    consumer that already knows how to report a rho reports a tau the same way.
    ``tau_a`` rides along because the gap to ``tau`` is the tie correction's
    size, which the headline number hides.
    """

    __slots__ = ('tau', 'tau_a', 'n', 'reason', 'loo_lo', 'loo_hi',
                 'ties_a', 'ties_b', 'concordant', 'discordant')

    def __init__(self, tau, n, reason=None, loo_lo=NAN, loo_hi=NAN,
                 ties_a=0, ties_b=0, tau_a_=NAN, concordant=0, discordant=0):
        self.tau = tau
        self.tau_a = tau_a_
        self.n = n
        self.reason = reason
        self.loo_lo = loo_lo
        self.loo_hi = loo_hi
        self.ties_a = ties_a
        self.ties_b = ties_b
        self.concordant = concordant
        self.discordant = discordant

    def __repr__(self):
        return fmt_tau(self.tau, self.loo_lo, self.loo_hi, self.n,
                       self.ties_a, self.ties_b, self.reason)

    def as_dict(self) -> Dict:
        r = lambda v: None if v != v else round(v, 6)  # noqa: E731
        return {'tau': r(self.tau), 'tau_a': r(self.tau_a), 'n': self.n,
                'reason': self.reason, 'loo_lo': r(self.loo_lo),
                'loo_hi': r(self.loo_hi), 'ties_a': self.ties_a,
                'ties_b': self.ties_b, 'concordant': self.concordant,
                'discordant': self.discordant,
                'display': fmt_tau(self.tau, self.loo_lo, self.loo_hi, self.n,
                                   self.ties_a, self.ties_b, self.reason)}


def board_tau(rows: Sequence[Mapping], predictor: str, dependent: str,
              *, predictor_group: str = 'predictors',
              dependent_group: str = 'truth') -> BoardTau:
    """tau-b for ONE board's rows. Same contract and same refusals as
    ``board_rho``, including the anti-pooling guard.

    ``_one_board`` is CALLED here, not merely referred to. That guard exists
    because ``board_rho`` once claimed it and did not execute it, so handing it
    six recorded runs returned the pooled +0.339 this module's docstring names
    as the trap. A second correlation function that skipped it would reopen the
    same hole on its first day.
    """
    _one_board(rows, 'board_tau')
    pv, dv = [], []
    dropped = dropped_nan = 0
    for p, d in zip(_column(rows, predictor_group, predictor),
                    _column(rows, dependent_group, dependent)):
        if p is None or d is None:
            dropped += 1
            continue
        if (isinstance(p, float) and p != p) or (isinstance(d, float)
                                                 and d != d):
            dropped_nan += 1
            continue
        pv.append(p)
        dv.append(d)
    n = len(pv)
    _bits = ([f'{dropped} row(s) dropped for a null value'] if dropped else [])
    _bits += ([f'{dropped_nan} row(s) dropped for a NaN value']
              if dropped_nan else [])
    note = (', ' + '; '.join(_bits)) if _bits else ''
    if n < MIN_N_FOR_TAU:
        return BoardTau(NAN, n, f'n={n} < {MIN_N_FOR_TAU}{note}')
    side = constant_side(pv, dv)
    if side == 'both':
        return BoardTau(NAN, n, f'predictor AND truth constant{note}')
    if side == 'predictor':
        return BoardTau(NAN, n, f'predictor constant (no dynamic range){note}')
    if side == 'dependent':
        return BoardTau(NAN, n, f'truth constant (saturated){note}')
    k = tau_counts(pv, dv)
    lo, hi = tau_loo_span(pv, dv)
    return BoardTau(kendall_tau(pv, dv), n, (note[2:] if note else None),
                    lo, hi, k['ties_a'], k['ties_b'], tau_a(pv, dv),
                    k['concordant'], k['discordant'])


def classify_board(rows: Sequence[Mapping], dependent: str = 'headline',
                   *, dependent_group: str = 'truth') -> str:
    """'measurable' | 'saturated' | 'starved' | 'thin' -- never a silent drop.

    A saturated board (every variant routed clean) has no headroom and can rank
    nothing; a starved board (every variant equally broken) is the same problem
    at the other end. Both are REPORTED with their constant value and excluded
    from the sign test's denominator, following the neutral-board rule
    ``test_placement_ab.py`` already encodes: a term with no effect on 3 of 4
    boards must not read as a clean sweep.
    """
    _one_board(rows, 'classify_board')
    vals = [v for v in _column(rows, dependent_group, dependent)
            if v is not None and not (isinstance(v, float) and v != v)]
    if len(vals) < MIN_N:
        return 'thin'
    uniq = set(vals)
    if len(uniq) > 1:
        return 'measurable'
    return 'saturated' if uniq == {0} else 'starved'


# ---------------------------------------------------------------------------
# aggregation across boards -- a mapping in, never a list
# ---------------------------------------------------------------------------

def _binom_tail(n: int, k: int) -> float:
    """P(X >= k) for X ~ Binomial(n, 1/2)."""
    return sum(math.comb(n, i) for i in range(k, n + 1)) / (2.0 ** n)


def sign_test(rhos_by_board: Mapping[str, BoardRho]) -> Dict:
    """Aggregate per-board rhos by SIGN, never by averaging them.

    Takes a MAPPING and not a sequence, on purpose: the argument type is the
    anti-pooling guard. A caller holding a flat list of rows cannot reach this
    function without first deciding, explicitly, which board each row belongs
    to.

    The rule is ``test_placement_ab.gate()``'s, transposed from marks to signs:
    a predictor points the right way on >= N-1 boards and the wrong way on
    none. ``coin_flip_null`` prints how often a direction-free predictor passes
    that rule by chance -- at N=3 it is 1 in 8, which the caller should print
    beside any 3-board claim.
    """
    if not isinstance(rhos_by_board, Mapping):
        raise StatsRefusal(
            'sign_test takes a MAPPING of board_key -> BoardRho. A flat '
            'sequence of rhos has lost which board each came from, which is '
            'the one thing that makes an aggregate here meaningful.')
    defined, undefined = {}, {}
    for b, br in rhos_by_board.items():
        (undefined if br.rho != br.rho else defined)[b] = br
    pos = sorted(b for b, br in defined.items() if br.rho > 0)
    neg = sorted(b for b, br in defined.items() if br.rho < 0)
    zero = sorted(b for b, br in defined.items() if br.rho == 0)
    n_dir = len(pos) + len(neg)
    k = max(len(pos), len(neg))
    if n_dir:
        one_sided = _binom_tail(n_dir, k)
        two_sided = min(1.0, 2.0 * one_sided)
    else:
        one_sided = two_sided = NAN
    n_boards = len(defined)
    consistent = max(len(pos), len(neg))
    # >= N-1 in one direction AND zero in the other, over the boards that
    # produced a defined rho. This is `test_placement_ab.gate()`'s rule
    # transposed from marks to signs, deliberately including its treatment of
    # a NEUTRAL board: there, `improve >= N-1, regress == 0` lets one board be
    # neutral out of three. Here a rho of exactly 0.0 is that board -- it
    # counts in N, it is not evidence for either direction, and it does not by
    # itself sink the rule.
    #
    # An earlier draft of this comment said a zero board counts AGAINST the
    # rule, which the code never did; `tests/test_703_rank_stats.py` caught the
    # disagreement. The comment was the wrong half: being stricter than the
    # house rule is a change to the acceptance bar, and this repo's own
    # doctrine is that a bar moves on a measurement, not on a docstring.
    # `--shuffle-control` is what measures whether this bar is honest.
    #
    # AND >= MIN_SIGN_BOARDS, which a shuffle control forced. With
    # `n_boards >= 1` the rule is `consistent >= max(1, 0) == 1`, so a
    # predictor defined on ONE board passes whatever that board says.
    # Measured on the #703 study: permuting the truth WITHIN each board 200
    # times, `cross_side_stacks` -- constant on three of four boards, so
    # defined on one -- passed this rule in 100% of the shuffles. Every
    # predictor defined on all four passed in 13-17%. A rule that a
    # signal-free predictor clears every single time is not a rule.
    #
    # Three is `test_placement_ab.MIN_TRIAL_BOARDS`, for the same reason it is
    # three there: a term on trial is judged on at least three DISTINCT boards.
    passes = (n_boards >= MIN_SIGN_BOARDS
              and consistent >= max(1, n_boards - 1)
              and min(len(pos), len(neg)) == 0)
    med = NAN
    vals = sorted(br.rho for br in defined.values())
    if vals:
        m = len(vals) // 2
        med = vals[m] if len(vals) % 2 else (vals[m - 1] + vals[m]) / 2.0
    return {
        'boards_attempted': len(rhos_by_board),
        'boards_defined': n_boards,
        'positive': pos, 'negative': neg, 'zero': zero,
        'undefined': {b: br.reason for b, br in sorted(undefined.items())},
        'direction': ('positive' if len(pos) > len(neg)
                      else 'negative' if len(neg) > len(pos) else None),
        'consistent': consistent,
        'median_rho': None if med != med else round(med, 6),
        'p_two_sided': None if two_sided != two_sided else round(two_sided, 4),
        'p_one_sided': None if one_sided != one_sided else round(one_sided, 4),
        'p_denominator': n_dir,
        'passes_sign_rule': passes,
        'below_min_boards': n_boards < MIN_SIGN_BOARDS,
        'coin_flip_null': (None if not n_boards
                           else f'1 in {2 ** n_boards}'),
    }


def format_sign_test(name: str, st: Mapping) -> List[str]:
    """The sign test as lines, with the denominator printed beside every p.

    A p-value labelled with the PLANNED board count when saturated boards
    reduced the actual denominator is #694 in miniature -- a verdict resting on
    a criterion nobody printed. So the denominator is never implicit here.
    """
    lines = [
        f'{name}: {len(st["positive"])} pos / {len(st["negative"])} neg / '
        f'{len(st["zero"])} zero over {st["boards_defined"]} board(s) with a '
        f'defined rho ({st["boards_attempted"]} attempted)']
    if st['p_two_sided'] is not None:
        lines.append(
            f'    p(two-sided) = {st["p_two_sided"]}  '
            f'[one-sided {st["p_one_sided"]}]  over n = {st["p_denominator"]} '
            f'board(s) that pointed a direction -- NOT over the planned count')
    if st.get('below_min_boards'):
        lines.append(
            f'    NO VERDICT: {st["boards_defined"]} board(s) produced a '
            f'defined rho and the rule needs {MIN_SIGN_BOARDS}. A predictor '
            f'defined on one board passes a >= N-1 rule whatever that board '
            f'says -- measured at 100% under a within-board shuffle.')
    lines.append(f'    median rho {fmt(st["median_rho"])}   '
                 f'sign rule (>= N-1 one way, 0 the other): '
                 f'{"PASSES" if st["passes_sign_rule"] else "fails"}   '
                 f'a direction-free predictor passes it '
                 f'{st["coin_flip_null"]} runs')
    for b, why in st['undefined'].items():
        lines.append(f'    {b}: no rho -- {why}')
    return lines


# ---------------------------------------------------------------------------
# self-test -- runs at the top of every invocation of every tool that imports
# this module, because the live tables cannot reach any of these branches
# ---------------------------------------------------------------------------

def _close(a: float, b: float, tol: float = 1e-9) -> bool:
    return abs(a - b) <= tol


#: The six recorded runs, joined by hand in August 2026 and published in the
#: #703 issue body. Kept HERE, as literals, because the runs they came from
#: live under the gitignored ``wk/`` tree and are one ``git clean`` from gone.
#: They are a REPRODUCTION PIN for the arithmetic, never evidence about
#: placement: n=6 pooled across boards, and case 6 below asserts the very
#: instability that makes them uninterpretable.
LEGACY_POOLED = [
    # board      crossings   hpwl    halo    blocking  vias
    ('castor',       453,  3069.0,   739.6,        0,   545),
    ('neo6502',      172,  1129.7,   321.3,       79,   427),
    ('piantor',       84,  1990.0,   986.9,        0,   162),
    ('urchin',       191,  3231.4, 12441.9,        3,   175),
    ('tigard',       670,  1988.3,  1152.1,        3,   377),
    ('esp_prog',      36,   221.9,    85.6,        0,    28),
]

_SELF_TESTED = [False]


def _self_test(force: bool = False) -> int:
    """Every branch of the arithmetic, in milliseconds. Raises on failure.

    Runs once per process (``force=True`` re-runs it). It is called at the top
    of every tool that imports this module, following the doctrine
    ``tests/test_placement_ab.py`` already encodes: the live table cannot reach
    any of this, so a test that only exercises the live path proves nothing
    about the branches that decide what "not measurable" means.
    """
    if _SELF_TESTED[0] and not force:
        return 0
    n = 0

    def ok(cond, case, msg):
        nonlocal n
        n += 1
        if not cond:
            raise AssertionError(f'rank_stats self-test case {case}: {msg}')

    # 1. ties share the mean of the positions they occupy
    ok(rank([3, 1, 1, 2]) == [4.0, 1.5, 1.5, 3.0], 1,
       f'ties not averaged: {rank([3, 1, 1, 2])}')
    ok(rank([5, 5, 5]) == [2.0, 2.0, 2.0], 1, 'all-tied should all be rank 2')

    # 2. near-but-unequal values must NOT merge -- ranking has no tolerance
    r = rank([1.0, 1.0 + 1e-12, 2.0])
    ok(r == [1.0, 2.0, 3.0], 2, f'a 1e-12 gap was treated as a tie: {r}')

    # 3. perfect monotone, reversal, symmetry
    a, b = [1, 2, 3, 4], [10, 20, 30, 40]
    ok(_close(spearman(a, b), 1.0), 3, 'monotone pair is not +1')
    ok(_close(spearman(a, b[::-1]), -1.0), 3, 'reversed pair is not -1')
    ok(_close(spearman(a, b), spearman(b, a)), 3, 'rho is not symmetric')

    # 4. NaN, never 0.0, and asserted with x != x rather than == 0
    for case, x, y in (('constant a', [1, 1, 1], [1, 2, 3]),
                       ('constant b', [1, 2, 3], [7, 7, 7]),
                       ('n<3', [1, 2], [3, 4])):
        v = spearman(x, y)
        ok(v != v, 4, f'{case} returned {v!r}, must be NaN (0.0 would read as '
                      f'"measured, no relationship")')

    # 5. a hand-computed tie-corrected value, ties on BOTH sides.
    #    x = [1,2,2,3] -> ranks [1, 2.5, 2.5, 4], mean 2.5, dev [-1.5,0,0,1.5],
    #        SS = 4.5
    #    y = [1,1,2,3] -> ranks [1.5, 1.5, 3, 4], mean 2.5, dev [-1,-1,.5,1.5],
    #        SS = 1 + 1 + 0.25 + 2.25 = 4.5
    #    cov = 1.5 + 0 + 0 + 2.25 = 3.75  ->  rho = 3.75 / 4.5 = 0.8333...
    #    (The first version of this case asserted 0.5145 from a mis-added SS of
    #    4.25 on the y side. The kernel was right and the literal was wrong,
    #    which is the whole reason a hand-computed case is worth having.)
    v = spearman([1, 2, 2, 3], [1, 1, 2, 3])
    ok(_close(v, 3.75 / 4.5, 1e-12), 5, f'tie-corrected rho is {v!r}')
    # ... and it must NOT equal the no-tie-correction shortcut
    # 1 - 6*sum(d^2)/(n(n^2-1)), which on this pair gives 1 - 6*1.5/60 = 0.85
    ok(not _close(v, 0.85, 1e-9), 5,
       'rho matches the uncorrected 1-6*sum(d^2) formula -- ties are not '
       'being corrected for')

    # 6. REPRODUCTION PIN: the published pooled numbers, and their instability
    cross = [r[1] for r in LEGACY_POOLED]
    hpwl = [r[2] for r in LEGACY_POOLED]
    halo = [r[3] for r in LEGACY_POOLED]
    blk = [r[4] for r in LEGACY_POOLED]
    vias = [r[5] for r in LEGACY_POOLED]
    for label, col, want in (('crossings', cross, 0.339),
                             ('hpwl', hpwl, -0.062),
                             ('halo', halo, 0.247)):
        got = spearman(col, blk)
        ok(_close(round(got, 3), want, 1e-9), 6,
           f'rho({label}, blocking) is {got:+.6f}, published {want:+.3f}')
    got = spearman(cross, vias)
    ok(_close(round(got, 3), 0.714, 1e-9), 6,
       f'rho(crossings, vias) is {got:+.6f}, published +0.714')
    lo, hi = loo_span(cross, blk)
    ok(_close(round(lo, 3), 0.053, 1e-9) and _close(round(hi, 3), 0.632, 1e-9),
       6, f'LOO span is {lo:+.3f}..{hi:+.3f}, published +0.053..+0.632')
    # and the whole point: the span is 12x the headline
    ok(hi / max(lo, 1e-9) > 10, 6,
       'the published LOO instability no longer reproduces')

    # 7. classification, and a saturated board keeps a reason
    def _rows(vals, key='b'):
        return [{'board_key': key, 'truth': {'headline': v},
                 'predictors': {'x': i}} for i, v in enumerate(vals)]
    ok(classify_board(_rows([0, 0, 0, 0])) == 'saturated', 7, 'all-zero')
    ok(classify_board(_rows([5, 5, 5])) == 'starved', 7, 'all-equal-nonzero')
    ok(classify_board(_rows([0, 1, 2])) == 'measurable', 7, 'mixed')
    ok(classify_board(_rows([0, 1])) == 'thin', 7, 'n<3')
    br = board_rho(_rows([0, 0, 0, 0]), 'x', 'headline')
    ok(br.rho != br.rho and 'saturated' in (br.reason or ''), 7,
       f'a saturated board must carry a REASON, got {br.reason!r}')
    br = board_rho([{'board_key': 'b', 'truth': {'headline': i},
                     'predictors': {'x': 4}} for i in range(4)],
                   'x', 'headline')
    ok(br.rho != br.rho and 'predictor constant' in (br.reason or ''), 7,
       f'a constant PREDICTOR must be named as such, got {br.reason!r}')

    # 8. the sign test's p-values and its denominator
    def _br(v):
        return BoardRho(v, 20)
    st = sign_test({f'b{i}': _br(0.5) for i in range(6)})
    ok(st['p_two_sided'] == 0.0312 or _close(st['p_two_sided'], 0.0313, 1e-3),
       8, f'6/6 two-sided is {st["p_two_sided"]}, expected ~0.031')
    ok(st['passes_sign_rule'], 8, '6-of-6 must pass the sign rule')
    st = sign_test({**{f'b{i}': _br(0.5) for i in range(5)}, 'b5': _br(-0.4)})
    ok(_close(st['p_one_sided'], 0.1094, 1e-3), 8,
       f'5/6 one-sided is {st["p_one_sided"]}, the note\'s 0.11')
    ok(_close(st['p_two_sided'], 0.2188, 1e-3), 8,
       f'5/6 two-sided is {st["p_two_sided"]}, expected ~0.219')
    ok(not st['passes_sign_rule'], 8,
       'one wrong-direction board must fail the rule, whatever the p-value')
    st = sign_test({f'b{i}': _br(0.5) for i in range(3)})
    ok(_close(st['p_two_sided'], 0.25, 1e-9), 8, '3/3 two-sided is 0.25')
    ok(st['coin_flip_null'] == '1 in 8', 8, 'N=3 null is 1 in 8')
    st = sign_test({f'b{i}': _br(0.5) for i in range(2)})
    ok(_close(st['p_two_sided'], 0.5, 1e-9), 8, 'N=2 minimum p is 0.50')
    # a NaN board is counted in "attempted", excluded from the denominator
    st = sign_test({'a': _br(0.5), 'b': _br(0.5), 'c': _br(0.5),
                    'd': BoardRho(NAN, 20, 'truth constant (saturated)')})
    ok(st['boards_attempted'] == 4 and st['boards_defined'] == 3, 8,
       f'saturated board must not join the denominator: {st}')
    ok(st['p_denominator'] == 3, 8, 'p denominator counts only defined rhos')
    ok('d' in st['undefined'] and 'saturated' in st['undefined']['d'], 8,
       'an undefined board must be NAMED with its reason, not dropped')

    # 9. formatting: NaN is n/a, and a rho cannot be printed without its scope
    ok(fmt(NAN).strip() == 'n/a', 9, f'fmt(NaN) is {fmt(NAN)!r}')
    ok(fmt(0.0).strip() == '+0.000', 9, 'a real zero must still print')
    s = fmt_rho(0.339, 0.053, 0.632, 6)
    ok('LOO' in s and 'K=6' in s and s.startswith('rho=+0.339'), 9,
       f'fmt_rho lost the scope: {s!r}')
    s = fmt_rho(NAN, reason='truth constant (saturated)')
    ok('n/a' in s and 'saturated' in s, 9, f'fmt_rho(NaN) is {s!r}')

    # 10. a None in a column is a NAMED refusal, not a TypeError from sorted()
    try:
        rank([1, None, 3])
    except StatsRefusal as e:
        ok('None' in str(e), 10, 'the refusal must say what was wrong')
    except TypeError:
        ok(False, 10, 'a None column raised TypeError, not StatsRefusal')
    else:
        ok(False, 10, 'a None column was ranked silently')
    try:
        rank([1.0, NAN, 3.0])
    except StatsRefusal:
        pass
    else:
        ok(False, 10, 'a NaN column was ranked silently')

    # 11. the anti-pooling guard, EXECUTED rather than intended. An adversarial
    #     review falsified the earlier claim in one line, so the case that
    #     catches it is the pooled fixture itself: board_rho over the six
    #     recorded runs used to return exactly +0.339.
    multi = [{'board_key': b, 'predictors': {'x': c},
              'truth': {'headline': k}}
             for b, c, _h, _hl, k, _v in LEGACY_POOLED]
    for name, fn in (('board_rho', lambda: board_rho(multi, 'x', 'headline')),
                     ('classify_board', lambda: classify_board(multi))):
        try:
            fn()
        except StatsRefusal as e:
            ok('6 boards' in str(e), 11,
               f'{name} must say how many boards it was given: {e}')
        else:
            ok(False, 11,
               f'{name} accepted rows from 6 boards -- it would return the '
               f'pooled +0.339 this module calls the trap')
    # ... and the same rows, one board at a time, are still accepted
    one = [r for r in multi if r['board_key'] == 'castor']
    ok(board_rho(one, 'x', 'headline').n == 1, 11,
       'rows from a single board are still accepted')
    # 11. the anti-pooling guard, tested rather than intended
    try:
        sign_test([_br(0.5), _br(0.5), _br(0.5)])       # type: ignore[arg-type]
    except StatsRefusal as e:
        ok('MAPPING' in str(e), 11, 'the refusal must say why a list is wrong')
    else:
        ok(False, 11, 'sign_test accepted a flat list -- pooling is reachable')
    try:
        per_board([{'row_id': 'x', 'truth': {}}])
    except StatsRefusal as e:
        ok('board_key' in str(e), 11, 'per_board must name the missing key')
    else:
        ok(False, 11, 'a row with no board_key was bucketed silently')
    groups = per_board([{'board_key': 'a'}, {'board_key': 'b'},
                        {'board_key': 'a'}])
    ok(sorted(groups) == ['a', 'b'] and len(groups['a']) == 2, 11,
       f'per_board grouped wrongly: { {k: len(v) for k, v in groups.items()} }')

    # 12. board_rho drops a null row, says so, and reports the surviving n
    rows = _rows([0, 1, 2, 3])
    rows[0]['predictors']['x'] = None
    br = board_rho(rows, 'x', 'headline')
    ok(br.n == 3 and 'dropped' in (br.reason or ''), 12,
       f'a null predictor must be dropped and NAMED: n={br.n} {br.reason!r}')

    # 12b. the MIN_SIGN_BOARDS floor, which a shuffle control forced
    for nb in (1, 2):
        st = sign_test({f'b{i}': BoardRho(0.9, 20) for i in range(nb)})
        ok(not st['passes_sign_rule'], 12,
           f'{nb} board(s) all agreeing must NOT pass: a >= N-1 rule is '
           f'vacuous at N<{MIN_SIGN_BOARDS} (measured: 100% under shuffle)')
        ok(st['below_min_boards'], 12,
           f'and the result SAYS it is below the floor, at N={nb}')
        ok(any('NO VERDICT' in ln for ln in format_sign_test('x', st)), 12,
           'and the printed form leads with NO VERDICT')
    st = sign_test({f'b{i}': BoardRho(0.9, 20) for i in range(3)})
    ok(st['passes_sign_rule'] and not st['below_min_boards'], 12,
       'three agreeing boards is the floor, and it passes')
    # A board with an UNDEFINED rho does not count toward the floor.
    st = sign_test({'a': BoardRho(0.9, 20), 'b': BoardRho(0.9, 20),
                    'c': BoardRho(NAN, 20, 'predictor constant')})
    ok(not st['passes_sign_rule'] and st['below_min_boards'], 12,
       'a NaN board does not help reach the floor -- 2 defined of 3 attempted')

    # 13. the renderings the review found uncovered
    ok(fmt(None).strip() == 'none', 13,
       f'fmt(None) must be distinguishable from n/a and from a number: '
       f'{fmt(None)!r}')
    ok('LOO not computed' in fmt_rho(0.5), 13,
       f'fmt_rho with no span must SAY there is no span: {fmt_rho(0.5)!r}')
    ok(repr(BoardRho(0.5, 4, None, 0.4, 0.6)).startswith('rho=+0.500 ['), 13,
       'BoardRho.__repr__ delegates to fmt_rho')
    st0 = sign_test({})
    lines = format_sign_test('empty', st0)
    ok(all('p(two-sided)' not in ln for ln in lines), 13,
       'an empty mapping prints no p-value line at all')
    ok(' zero ' in lines[0] and 'pos' in lines[0], 13,
       f'the zero count is labelled, not glued to a digit: {lines[0]!r}')
    st3 = sign_test({'a': BoardRho(0.0, 4), 'b': BoardRho(0.0, 4),
                     'c': BoardRho(0.0, 4)})
    ok('3 zero' in format_sign_test('z', st3)[0], 13,
       f'three zero boards must not render as "30": '
       f'{format_sign_test("z", st3)[0]!r}')

    # 14. board_rho's remaining arms
    br = board_rho(_rows([0, 1]), 'x', 'headline')
    ok(br.rho != br.rho and br.reason == 'n=2 < 3', 14,
       f'n below MIN_N is named with the number: {br.reason!r}')
    br = board_rho([{'board_key': 'b', 'predictors': {'x': 1},
                     'truth': {'headline': 1}} for _ in range(4)],
                   'x', 'headline')
    ok('predictor AND truth constant' in (br.reason or ''), 14,
       f'both-constant is its own reason: {br.reason!r}')
    rows_nan = _rows([0, 1, 2, 3])
    rows_nan[0]['predictors']['x'] = NAN
    br = board_rho(rows_nan, 'x', 'headline')
    ok('NaN value' in (br.reason or '') and 'null' not in (br.reason or ''),
       14, f'a NaN value is not reported as a null one: {br.reason!r}')

    # 15. Kendall tau-b (#789). Perfect agreement, reversal, symmetry.
    a, b = [1, 2, 3, 4], [10, 20, 30, 40]
    ok(_close(kendall_tau(a, b), 1.0), 15, 'monotone pair is not tau +1')
    ok(_close(kendall_tau(a, b[::-1]), -1.0), 15, 'reversed pair is not tau -1')
    ok(_close(kendall_tau(a, b), kendall_tau(b, a)), 15, 'tau is not symmetric')
    ok(_close(tau_a(a, b), 1.0), 15, 'tau-a on a tie-free pair is not +1')

    # 16. THE FIXTURE THAT SEPARATES THREE COEFFICIENTS. Case 5's pair, ties on
    #     both sides, hand-computed:
    #       ranks a = [1, 2.5, 2.5, 4]   ranks b = [1.5, 1.5, 3, 4]
    #       pairs: (0,1) tied in b; (1,2) tied in a; the other four concordant
    #       C=4, D=0, n0=6, ties_a=1, ties_b=1
    #       tau-b = 4/sqrt(5*5) = 0.800   tau-a = 4/6 = 0.6667   rho = 0.8333
    #     Three different numbers from one fixture, so "tau-a returned where
    #     tau-b was meant" and "tau was implemented by calling spearman" are
    #     both caught here rather than in a document.
    x5, y5 = [1, 2, 2, 3], [1, 1, 2, 3]
    k5 = tau_counts(x5, y5)
    ok(k5['concordant'] == 4 and k5['discordant'] == 0 and k5['n0'] == 6
       and k5['ties_a'] == 1 and k5['ties_b'] == 1, 16, f'tau_counts: {k5}')
    ok(_close(kendall_tau(x5, y5), 0.8, 1e-12), 16,
       f'tau-b is {kendall_tau(x5, y5)!r}, hand-computed 0.800')
    ok(_close(tau_a(x5, y5), 4 / 6, 1e-12), 16,
       f'tau-a is {tau_a(x5, y5)!r}, hand-computed 0.6667')
    ok(not _close(kendall_tau(x5, y5), spearman(x5, y5), 1e-6), 16,
       'tau equals rho on the tie fixture -- one is computing the other')
    ok(not _close(kendall_tau(x5, y5), tau_a(x5, y5), 1e-6), 16,
       'tau-b equals tau-a on a pair with ties -- the correction is inert')

    # 17. the equivalence tau_counts relies on: average ranks are a monotone
    #     map that preserves ties, so tau on raw values == tau on ranks. This
    #     is asserted, not asserted-in-a-docstring.
    ok(_close(kendall_tau(x5, y5), kendall_tau(rank(x5), rank(y5)), 1e-12), 17,
       'tau on values disagrees with tau on ranks')

    # 18. NaN, never 0.0 -- the same rule rho follows, for the same reason.
    for case, x, y in (('constant a', [1, 1, 1], [1, 2, 3]),
                       ('constant b', [1, 2, 3], [7, 7, 7]),
                       ('n<3', [1, 2], [2, 1]),
                       ('length mismatch', [1, 2, 3], [1, 2])):
        v = kendall_tau(x, y)
        ok(v != v, 18, f'tau({case}) returned {v!r}, must be NaN')
    lo18, hi18 = tau_loo_span([1, 2], [2, 1])
    ok(lo18 != lo18 and hi18 != hi18, 18,
       f'tau_loo_span below the floor must be (NaN, NaN), got {lo18},{hi18}')

    # 19. board_tau CALLS the anti-pooling guard, and renders like board_rho
    try:
        board_tau([{'board_key': 'a', 'predictors': {'x': 1},
                    'truth': {'headline': 1}},
                   {'board_key': 'b', 'predictors': {'x': 2},
                    'truth': {'headline': 2}}], 'x', 'headline')
        ok(False, 19, 'board_tau pooled two boards without refusing')
    except StatsRefusal as exc:
        ok('2 boards' in str(exc), 19, f'refusal does not name the count: {exc}')
    bt = board_tau(_rows([0, 1, 2, 3]), 'x', 'headline')
    ok(bt.n == 4 and bt.tau == bt.tau, 19, f'board_tau on a live board: {bt!r}')
    ok('tau=' in bt.as_dict()['display'] and 'tied_a=' in repr(bt), 19,
       f'BoardTau does not render through fmt_tau: {bt!r}')
    bt = board_tau([{'board_key': 'b', 'predictors': {'x': 1},
                     'truth': {'headline': i}} for i in range(4)],
                   'x', 'headline')
    ok(bt.tau != bt.tau and 'predictor constant' in (bt.reason or ''), 19,
       f'a constant predictor must be NaN with a reason: {bt.reason!r}')

    # 20. fmt_tau is the only rendering, and an n/a carries its reason
    s = fmt_tau(NAN, reason='truth constant (saturated)', k=11)
    ok('tau=n/a' in s and 'saturated' in s and 'K=11' in s, 20, s)
    s = fmt_tau(0.5, k=4)
    ok('LOO not computed' in s and 'tau=+0.500' in s, 20, s)
    s = fmt_tau(0.5, 0.1, 0.9, 4, 2, 3)
    ok('LOO' in s and 'tied_a=2' in s and 'tied_b=3' in s, 20, s)

    _SELF_TESTED[0] = True
    return n


if __name__ == '__main__':
    import sys
    cases = _self_test(force=True)
    print(f'rank_stats self-test: {cases} assertions PASSED')
    sys.exit(0)
