#!/usr/bin/env python3
"""The iso panel is drawn only when there are bodies to show (#946 item 3,
#1016).

`plan_iso_shots`'s own docstring says it plainly: "copper sits under soldermask
and the 3D view SHOWS NO ROUTING PROGRESS AT ALL; what it shows is the parts
moving and the board turning." At `height_frac 0.62` that is 620 px of a 1620 px
film -- **38% of every frame** -- carrying no routing information by design.

That is a fair trade on a populated board, where the iso view is the only thing
in the film that shows the board as an OBJECT. It is not a trade at all when
there are no bodies: kit-dev-coldfire-xilinx_5213 resolves 1/160 models,
orangecrab_ext_pll 5/148, splitflap_driver 3/58, watchy 7/75.

The decision data already existed and was used only as a CAPTION. This gate
reuses it, and these tests pin four things about how:

  * the SAME list object comes back, untouched, which is `compose_two_panel`'s
    stated degradation contract -- "not crash, not silently drop a panel, and
    not change the frame size";
  * the OFF state cannot read like success, the rule its four siblings follow;
  * the gate runs BEFORE resolving kicad-cli, because "should this panel be
    drawn" is cheaper and more fundamental than "can it be" -- and putting it
    after meant a machine without kicad-cli reported `did_not_run` for a board
    that would have been gated anyway, a true statement that hides the useful
    one;
  * and there is an opt-out, because on a populated board the panel is the
    point.

Needs no kicad-cli and renders nothing: the gate is reached before either.
"""
import os
import sys

RUN_ALL_FAST_OK = True

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import kicad_iso_render as kir   # noqa: E402
import movie_panels as MP        # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def _frames(n=3):
    return [Image.new('RGB', (200, 120), (10, 10, 10)) for _ in range(n)]


def _marks(board, n):
    return [('step1', board, 0, n)]


def _run(models, **opt_kw):
    """compose_two_panel with `resolve_models` forced to `models`."""
    saved = kir.resolve_models
    kir.resolve_models = lambda *_a, **_k: models
    try:
        fr = _frames()
        before = list(fr)
        out, rep = MP.compose_two_panel(
            fr, _marks(BOARD, len(fr)), BOARD, MP.IsoOpts(**opt_kw))
        return out, rep, before
    finally:
        kir.resolve_models = saved


def test_a_mostly_bare_board_gets_no_panel():
    _mark = len(_FAIL)
    for models, why in ((dict(total=58, found=3), '3/58, the splitflap case'),
                        (dict(total=160, found=1), '1/160, kit-dev-coldfire'),
                        (dict(total=0, found=0), 'no models referenced'),
                        (None, 'resolve_models answered nothing')):
        out, rep, before = _run(models)
        if rep.get('state') != 'mostly_bare':
            fail('%s: state is %r, expected mostly_bare'
                 % (why, rep.get('state')))
            continue
        if out is not before and list(out) != before:
            fail('%s: frames were not returned untouched' % why)
            continue
        if any(a is not b for a, b in zip(out, before)):
            fail('%s: a frame object was replaced' % why)
            continue
        if len({f.size for f in out}) != 1 or out[0].size != (200, 120):
            fail('%s: the frame size moved' % why)
            continue
        print('    gated: %-34s -> %s' % (why, rep.get('detail')))
    if len(_FAIL) == _mark:
        print('  PASS: a bare board keeps its frames, untouched, and says why')


def test_a_populated_board_is_not_gated():
    """THE CONTROL. A gate that refuses everything is not a gate.

    A populated board must get PAST the models check. It then stops at the next
    real obstacle -- kicad-cli, usually absent here -- and the point is that
    the reason is no longer `mostly_bare`.
    """
    _mark = len(_FAIL)
    out, rep, _b = _run(dict(total=75, found=62))
    if rep.get('state') == 'mostly_bare':
        fail('a board resolving 62/75 models was gated as mostly bare -- the '
             'gate refuses everything, so gating proves nothing')
    else:
        print('    62/75 models -> past the gate, state %r' % rep.get('state'))
    if len(_FAIL) == _mark:
        print('  PASS: the gate lets a populated board through')


def test_the_threshold_is_the_one_that_already_existed():
    _mark = len(_FAIL)
    f = getattr(kir, 'MOSTLY_BARE_FRACTION', None)
    if f is None:
        fail('kicad_iso_render.MOSTLY_BARE_FRACTION is gone; this gate was '
             'built on it precisely so there would not be a second threshold')
        return
    # just above and just below, on a total that makes the boundary exact
    total = 100
    out_lo, rep_lo, _ = _run(dict(total=total, found=int(total * f) - 1))
    out_hi, rep_hi, _ = _run(dict(total=total, found=int(total * f) + 1))
    if rep_lo.get('state') != 'mostly_bare':
        fail('%d/%d (below %.2f) was not gated' % (int(total * f) - 1, total, f))
    if rep_hi.get('state') == 'mostly_bare':
        fail('%d/%d (above %.2f) WAS gated' % (int(total * f) + 1, total, f))
    if len(_FAIL) == _mark:
        print('  PASS: the boundary is MOSTLY_BARE_FRACTION = %.2f, not a '
              'second copy of it' % f)


def test_there_is_an_opt_out_and_it_is_off_by_default():
    _mark = len(_FAIL)
    if not MP.IsoOpts().require_models:
        fail('require_models is not on by default -- a bare board would still '
             'spend 38%% of every frame on a rotating rectangle')
    out, rep, _b = _run(dict(total=58, found=3), require_models=False)
    if rep.get('state') == 'mostly_bare':
        fail('require_models=False still gated the panel')
    else:
        print('    --iso-allow-bare -> past the gate, state %r'
              % rep.get('state'))
    if len(_FAIL) == _mark:
        print('  PASS: on by default, with an opt-out for when the board as an '
              'object IS the point')


def test_the_off_state_cannot_read_as_success():
    _mark = len(_FAIL)
    line = MP.iso_status_line({'state': 'mostly_bare',
                               'detail': '3 of 58 3D models resolve (< 25%)'})
    if 'OFF' not in line:
        fail('the status line does not say OFF: %r' % line)
    if 'ON' in line.replace('OFF', ''):
        fail('the status line contains "ON": %r' % line)
    if '3 of 58' not in line:
        fail('the status line does not carry the reason: %r' % line)
    if len(_FAIL) == _mark:
        print('  PASS: %s' % line)


def test_the_gate_runs_before_resolving_kicad_cli():
    """Source order, asserted. 'Should this be drawn' needs no binary."""
    _mark = len(_FAIL)
    src = open(MP.__file__, encoding='utf-8').read()
    try:
        gate = src.index('#1016')
        probe = src.index('resolve_cli(opts.cli)')
    except ValueError as exc:
        fail('BROKEN: cannot find both anchors (%s)' % exc)
        return
    if gate > probe:
        fail('the models gate runs AFTER resolve_cli, so a machine without '
             'kicad-cli reports did_not_run for a board that would have been '
             'gated anyway')
    if len(_FAIL) == _mark:
        print('  PASS: the gate is reached without kicad-cli')


TESTS = (
    test_a_mostly_bare_board_gets_no_panel,
    test_a_populated_board_is_not_gated,
    test_the_threshold_is_the_one_that_already_existed,
    test_there_is_an_opt_out_and_it_is_off_by_default,
    test_the_off_state_cannot_read_as_success,
    test_the_gate_runs_before_resolving_kicad_cli,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for m in _FAIL:
            print('  - %s' % m)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
