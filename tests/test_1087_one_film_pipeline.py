#!/usr/bin/env python3
"""make_movie and make_film share ONE band/panel pipeline (#1087).

`make_film.build_film` used to re-implement `make_movie`'s post-passes --
attempts discovery, the placement panels, the band reservation and the
composition -- so every film-level feature had to be threaded twice. Both
now go through `film_passes.plan` / `compose`. Pinned on the CALLS, not the text: each front end is run once
with `film_passes` spied, and each must plan, compose and hand the frames it
got back onward.
"""
import contextlib
import io
import os
import sys
import tempfile

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import importlib.util                                           # noqa: E402
if importlib.util.find_spec('PIL') is None:
    print('SKIP: needs Pillow')
    sys.exit(77)

import film_passes as FP                                        # noqa: E402

_FAIL = []


def _check(ok, msg):
    print('  %s %s' % ('ok  ' if ok else 'FAIL', msg))
    if not ok:
        _FAIL.append(msg)


def _spied(run):
    calls = []
    orig = (FP.plan, FP.compose)

    def _plan(*a, **k):
        calls.append(('plan', k.get('who')))
        return orig[0](*a, **k)

    def _compose(*a, **k):
        calls.append(('compose', k.get('who')))
        return orig[1](*a, **k)
    FP.plan, FP.compose = _plan, _compose
    try:
        with contextlib.redirect_stderr(io.StringIO()), \
                contextlib.redirect_stdout(io.StringIO()):
            run()
    finally:
        FP.plan, FP.compose = orig
    return calls


def test_both_front_ends_run_the_one_pipeline():
    from fixture_boards import ensure_many
    boards = ensure_many('fanout_output1.kicad_pcb',
                         'fanout_output2.kicad_pcb')
    d = tempfile.mkdtemp(prefix='t1087_')
    import make_movie as MM
    import make_film as MF
    mm = _spied(lambda: MM.make_movie(boards, out=os.path.join(d, 'a.gif'),
                                      size=240, quiet=True, board3d='2d'))
    mf = _spied(lambda: MF.build_film(MF.parse_positional(boards, []),
                                      size=240, fps=6.0, camera='off',
                                      quiet=True,
                                      placement={'board3d': '2d'}))
    _check(('plan', 'make_movie') in mm and ('compose', '') in mm,
           'make_movie plans and composes through film_passes (%s)' % mm)
    _check(('plan', 'make_film') in mf and ('compose', 'make_film') in mf,
           'make_film plans and composes through film_passes (%s)' % mf)
    import inspect
    for mod, name in ((MM, 'make_movie'), (MF, 'make_film')):
        src = inspect.getsource(mod)
        # the pipeline's own entry points are film_passes'; a front end that
        # calls one of them directly has grown a second copy again
        dup = [f for f in ('movie_attempts.attach(', 'movie_benchmark.attach(',
                           'movie_placement.split_band(')
               if f in src]
        _check(not dup, '%s calls no band/panel composer itself (%s)'
               % (name, dup))


def test_stage3d_without_a_ledger_keeps_the_placement_panels():
    """A placement chain made from boards alone has no ledger, so no
    benchmark band; stage3d then keeps the #1042 placement panels rather
    than losing every placement number (the final review)."""
    import film_chain_1081 as FC
    with FC.Chain() as c:
        steps = [('s%d' % i, b, None) for i, b in enumerate(c.boards[:3])]
        with contextlib.redirect_stderr(io.StringIO()):
            b = FP.plan(steps, c.boards[2], quiet=True)
        _check(b.btrack is None and b.ptrack is not None
               and b.band, 'stage3d, no ledger: the placement panels are '
               'measured and reserved (%s, band %r)' % (b.pwhy, b.band))


def test_plan_takes_no_layout_and_no_aspect():
    """There is one layout, so the band plan cannot depend on one: an
    extreme aspect (outside 0.50..3.00) is still a stage3d frame, a
    board-only one, and its band is decided by plan_frame's height floor,
    not by a second plan here (the legacy half-fallback this replaced)."""
    import inspect
    params = inspect.signature(FP.plan).parameters
    _check('layout' not in params and 'aspect' not in params,
           'film_passes.plan takes neither (%s)' % list(params))


TESTS = (test_both_front_ends_run_the_one_pipeline,
         test_stage3d_without_a_ledger_keeps_the_placement_panels,
         test_plan_takes_no_layout_and_no_aspect)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for msg in _FAIL:
            print('  - %s' % msg)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
