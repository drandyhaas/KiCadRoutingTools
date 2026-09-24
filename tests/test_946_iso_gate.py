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
  * the gate runs WITHOUT kicad-cli (env-only, and says so), because "should
    this panel be drawn" is more fundamental than "can it be" -- a machine
    without kicad-cli reporting `did_not_run` for a board that would have been
    gated anyway is a true statement that hides the useful one -- and WITH
    kicad-cli it counts models with the render's own `model_dirs(cli, board)`
    (#1035: without them Windows read 0/224 while the render found 213/224);
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


def _run_with_cli(cli, models):
    """compose_two_panel with kicad-cli resolved to `cli` (None = absent) and
    `resolve_models` recording the dirs it was handed."""
    seen = []
    saved_models, saved_cli = kir.resolve_models, kir.resolve_cli
    kir.resolve_models = lambda board, dirs=None: (seen.append(dirs), models)[1]
    kir.resolve_cli = lambda _explicit=None: (
        (cli, '') if cli else (None, 'kicad-cli not found (test)'))
    try:
        fr = _frames()
        _out, rep = MP.compose_two_panel(
            fr, _marks(BOARD, len(fr)), BOARD, MP.IsoOpts())
        return rep, seen
    finally:
        kir.resolve_models, kir.resolve_cli = saved_models, saved_cli


def test_the_gate_runs_without_kicad_cli_env_only_and_says_so():
    """Behaviour, not source order (#1035). With no kicad-cli the gate still
    runs -- 'should this be drawn' needs no binary -- on environment-only
    resolution, and its detail SAYS env-only, because that count can
    undercount an install tree it could not locate."""
    _mark = len(_FAIL)
    rep, seen = _run_with_cli(None, dict(total=58, found=3))
    if rep.get('state') != 'mostly_bare':
        fail('with no kicad-cli the gate did not run: state %r (a machine '
             'without kicad-cli must still hear mostly_bare, not did_not_run)'
             % rep.get('state'))
    elif 'env-only' not in (rep.get('detail') or ''):
        fail('the no-cli gate did not say it was env-only: %r'
             % rep.get('detail'))
    if not seen:
        fail('resolve_models was never called by the gate')
    elif seen[0] != kir.model_dirs(None, BOARD):
        fail('the no-cli gate resolved with %r, expected the env-only '
             'model_dirs(None, board) %r'
             % (seen[0], kir.model_dirs(None, BOARD)))
    # and a board that passes the gate with no cli then reports did_not_run
    rep2, _ = _run_with_cli(None, dict(total=75, found=62))
    if rep2.get('state') != 'did_not_run':
        fail('a populated board with no kicad-cli reported %r, expected '
             'did_not_run AFTER the gate' % rep2.get('state'))
    if len(_FAIL) == _mark:
        print('  PASS: no kicad-cli -> gate ran env-only (%s); a populated '
              'board then reports did_not_run' % rep.get('detail'))


def test_the_gate_counts_with_the_install_model_dirs():
    """#1035: the gate must see the same dirs the RENDER uses,
    `kir.model_dirs(cli, board)`. It used to pass none, so on Windows (no
    KICAD10_3DMODEL_DIR env var) it read 0/224 while the render found 213/224.
    A fake cli path is enough: the claim is about WHICH dirs are passed.
    """
    _mark = len(_FAIL)
    cli, _why = kir.resolve_cli(None)
    probe_cli = cli or os.path.join(ROOT, 'no_such_dir', 'bin', 'kicad-cli')
    rep, seen = _run_with_cli(probe_cli, dict(total=224, found=213))
    want = kir.model_dirs(probe_cli, BOARD)
    if not seen:
        fail('resolve_models was never called by the gate')
    elif seen[0] != want:
        fail('the gate resolved with %r, not model_dirs(cli, board) %r'
             % (seen[0], want))
    if rep.get('state') == 'mostly_bare':
        fail('213/224 was gated as mostly bare')
    if 'env-only' in (rep.get('detail') or ''):
        fail('a resolved cli still reported env-only')
    # With a REAL kicad-cli, the dirs map the versioned variables to the
    # install tree -- exactly what the old gate was missing.
    if cli and kir.kicad_share_dirs(cli):
        tree = kir.kicad_share_dirs(cli)[0]
        if not any(v == tree for k, v in want.items() if k != 'KIPRJMOD'):
            fail('model_dirs(%s) maps no variable to the install tree %s'
                 % (cli, tree))
        else:
            print('    real kicad-cli: the gate searches %s' % tree)
    if len(_FAIL) == _mark:
        print('  PASS: the gate resolves with model_dirs(cli, board), the '
              'render\'s own dirs')


TESTS = (
    test_a_mostly_bare_board_gets_no_panel,
    test_a_populated_board_is_not_gated,
    test_the_threshold_is_the_one_that_already_existed,
    test_there_is_an_opt_out_and_it_is_off_by_default,
    test_the_off_state_cannot_read_as_success,
    test_the_gate_runs_without_kicad_cli_env_only_and_says_so,
    test_the_gate_counts_with_the_install_model_dirs,
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
