#!/usr/bin/env python3
"""PR C's pre-push code review, each finding pinned (#1036, #946).

  1. make_film closes BOTH FrameSpools on any exception (an injected
     append that raises 'disk full', during the board frames and during
     assembly) -- it left two krt_frames_* directories behind; and it takes
     --max-frames.
  4. An invalid $KICAD_RENDER_THEME warns ONCE per film, not per frame; a
     caller's IsoOpts is not mutated.
  5a. A synthesised placement step that also lays copper plays that copper
      through the normal reveal (its trace), not a silent snap.
  5b. Stage keys boards by resolved absolute path: two chain boards with one
      basename are not one round.
  5c. Duplicate-reference `~N` keys: pairing by uuid; an ordinal whose block
      count changed is not read as a move.
  6.  A part overhanging the outline is placed; only a part entirely off it
      counts as unplaced.
  7.  `leading_copper_free` ignores `(arc` inside a zone's `(pts ...)`.
  GIF: a strided GIF holds EXACTLY `GIF_MAX_FRAMES`.

Needs Pillow; small in-repo boards, no kicad-cli.
"""
import contextlib
import glob
import io
import json
import os
import shutil
import sys
import tempfile

RUN_ALL_TIMEOUT = 900

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image, ImageSequence
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import animate_route as A          # noqa: E402
import frame_spool                 # noqa: E402

KF = os.path.join(ROOT, 'kicad_files')
BOARD = os.path.join(KF, 'routed_output.kicad_pcb')

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def _spool_dirs():
    return set(glob.glob(os.path.join(tempfile.gettempdir(),
                                      'krt_frames_*')))


def test_make_film_closes_its_spools_on_failure():
    _mark = len(_FAIL)
    import make_film
    tmp = tempfile.mkdtemp(prefix='t1036cr_')
    argv = [BOARD, BOARD, '-o', os.path.join(tmp, 'f.gif'), '--size', '200',
            '--quiet', '--camera', 'off', '--no-attempts']
    orig = frame_spool.FrameSpool.append
    calls = [0]

    def _count(self, img):
        calls[0] += 1
        return orig(self, img)
    frame_spool.FrameSpool.append = _count
    try:
        make_film.main(argv)
    finally:
        frame_spool.FrameSpool.append = orig
    total = calls[0]
    for at in (2, total - 1):
        before = _spool_dirs()
        calls[0] = 0

        def _boom(self, img, at=at):
            calls[0] += 1
            if calls[0] == at:
                raise OSError('disk full (simulated)')
            return orig(self, img)
        frame_spool.FrameSpool.append = _boom
        raised = False
        try:
            make_film.main(argv)
        except OSError:
            raised = True
        finally:
            frame_spool.FrameSpool.append = orig
        left = _spool_dirs() - before
        if not raised:
            fail('BROKEN: the injected failure at append %d never fired' % at)
        if left:
            fail('append %d of %d failed and %d spool dir(s) were left: %r'
                 % (at, total, len(left), sorted(left)))
    # and it takes the budget flag
    try:
        rc = make_film.main(argv + ['--max-frames', '5'])
    except SystemExit as e:
        rc = e.code
    if rc != 0:
        fail('make_film --max-frames 5 exited %r' % rc)
    shutil.rmtree(tmp, ignore_errors=True)
    if len(_FAIL) == _mark:
        print('  PASS: failures at append 2 and %d of %d leave no spool dir; '
              '--max-frames accepted' % (total - 1, total))


def test_an_invalid_theme_warns_once_and_iso_opts_are_not_mutated():
    _mark = len(_FAIL)
    import env_knobs
    import make_movie
    import movie_panels
    old = os.environ.get('KICAD_RENDER_THEME')
    os.environ['KICAD_RENDER_THEME'] = 'chartreuse'
    env_knobs.refresh()
    err = io.StringIO()
    opts = movie_panels.IsoOpts(max_renders=0)
    tmp = tempfile.mkdtemp(prefix='t1036th_')
    try:
        with contextlib.redirect_stderr(err):
            make_movie.make_movie([BOARD], out=os.path.join(tmp, 'm.gif'),
                                  size=200, quiet=True, attempts=False,
                                  layout='split', aspect='16:9',
                                  panels='xray+iso', iso_opts=opts,
                                  # a run clock, so the per-FRAME clock band
                                  # (the path that warned once per frame)
                                  # is drawn
                                  timing=os.path.join(_TESTS, 'fixtures',
                                                      'cmd_timing',
                                                      'synthetic_run.jsonl'))
    finally:
        if old is None:
            os.environ.pop('KICAD_RENDER_THEME', None)
        else:
            os.environ['KICAD_RENDER_THEME'] = old
        env_knobs.refresh()
        shutil.rmtree(tmp, ignore_errors=True)
    n = err.getvalue().count("'chartreuse' is not one of")
    if n != 1:
        fail('an invalid $KICAD_RENDER_THEME warned %d times' % n)
    if opts.theme is not None:
        fail("the caller's IsoOpts was mutated: theme %r" % (opts.theme,))
    if len(_FAIL) == _mark:
        print('  PASS: one warning for the film; the IsoOpts passed in is '
              'unchanged')


def test_a_placement_step_that_lays_copper_plays_it():
    _mark = len(_FAIL)
    import movie_camera as MC
    from kicad_parser import parse_kicad_pcb
    from test_film_composition import _variant
    d = tempfile.mkdtemp(prefix='t1036pc_')
    try:
        a = os.path.join(d, 'a.kicad_pcb')
        b = os.path.join(d, 'b.kicad_pcb')
        shutil.copy(BOARD, a)
        _variant(BOARD, b, dx=3.0, dy=0.0, n=3)
        pcb = parse_kicad_pcb(b)
        layers = list(pcb.board_info.copper_layers)
        s0 = A._board_rows(pcb, layers)[0][0]
        tr = os.path.join(d, 'b_trace.json')
        with open(tr, 'w') as f:
            json.dump({'layers': layers, 'events': [
                {'event': 'route', 'net_name': 'VIA_STEP',
                 'add_s': [[s0[0] + .5, s0[1] + .5, s0[2] + .5, s0[3] + .5,
                            s0[4], s0[5]]]}]}, f)
        labels = []
        orig = A.Movie._note_chrome

        def _spy(self, label):
            labels.append(label)
            return orig(self, label)
        A.Movie._note_chrome = _spy
        try:
            st = MC.Stage(MC.synth_rounds([a, b]), '', tween=3)
            A.build_boards([('a', a, None), ('b', b, tr)], b, 200, 1, None,
                           2, 6, stage=st)
        finally:
            A.Movie._note_chrome = orig
        glide = [x for x in labels if 'moving' in x]
        played = [x for x in labels if 'VIA_STEP' in x]
        if not glide:
            fail('BROKEN: no glide frames: %r' % labels[:10])
        if not played:
            fail("the moving step's own trace never played -- snapped")
        elif labels.index(played[0]) < labels.index(glide[-1]):
            fail('the copper played before the glide finished')
        # 5b: a same-named board in ANOTHER directory is not that round
        other = os.path.join(d, 'x')
        os.makedirs(other)
        shutil.copy(b, os.path.join(other, 'b.kicad_pcb'))
        st2 = MC.Stage(MC.synth_rounds([a, b]), '', tween=3)
        st2.attach(object(), type('R', (), {
            'bounds': (0, 0, 1, 1), 'set_view': lambda s, v=None: None})(),
            layers)
        if not st2.handles(b):
            fail('the round is not found by its own board')
        if st2.handles(os.path.join(other, 'b.kicad_pcb')):
            fail('a same-named board in another directory matched the round')
    finally:
        shutil.rmtree(d, ignore_errors=True)
    if len(_FAIL) == _mark:
        print("  PASS: the glide plays, then the step's own trace; rounds are "
              'keyed by absolute path')


class _Fp(object):
    def __init__(self, x, y, uuid='', rot=0.0):
        self.x, self.y, self.rotation, self.uuid = x, y, rot, uuid
        self.pads = []


class _Pcb(object):
    def __init__(self, fps):
        self.footprints = fps


def test_duplicate_references_pair_by_uuid():
    _mark = len(_FAIL)
    import kicad_parser
    import movie_camera as MC
    boards = {
        # the two TP4 blocks swap FILE ORDER: by key both "move" 10 mm, by
        # uuid nothing moved
        'p': _Pcb({'TP4': _Fp(0, 0, 'u1'), 'TP4~2': _Fp(10, 0, 'u2')}),
        'q': _Pcb({'TP4': _Fp(10, 0, 'u2'), 'TP4~2': _Fp(0, 0, 'u1')}),
        # no uuids, and the block count changed: the ordinal is not a move
        'r': _Pcb({'J1': _Fp(0, 0), 'J1~2': _Fp(5, 0), 'J1~3': _Fp(9, 0)}),
        's': _Pcb({'J1': _Fp(0, 0), 'J1~2': _Fp(9, 0)}),
    }
    orig = kicad_parser.parse_kicad_pcb
    kicad_parser.parse_kicad_pcb = lambda path, *a, **k: boards[path]
    try:
        r1 = MC.synth_rounds(['p', 'q'], min_mm=0.5)
        r2 = MC.synth_rounds(['r', 's'], min_mm=0.5)
    finally:
        kicad_parser.parse_kicad_pcb = orig
    if any(rd['moved'] for rd in r1):
        fail('a file-order swap of two TP4 blocks read as moves: %r' % r1)
    if any(rd['moved'] for rd in r2):
        fail('J1~2 read as moved after a J1 block was removed: %r' % r2)
    if len(_FAIL) == _mark:
        print('  PASS: uuid pairing, and a changed block count is no move')


def test_an_overhanging_part_is_placed():
    _mark = len(_FAIL)

    class _Pad(object):
        def __init__(self, x, y):
            self.global_x, self.global_y, self.size_x, self.size_y = \
                x, y, 1.0, 1.0

    class _F(object):
        def __init__(self, x, y, pads):
            self.x, self.y, self.pads = x, y, pads

    class _BI(object):
        board_bounds = (0.0, 0.0, 10.0, 10.0)
        copper_layers = ['F.Cu', 'B.Cu']

    pcb = type('P', (), {})()
    pcb.board_info = _BI()
    pcb.footprints = {
        'J1': _F(10.8, 5.0, [_Pad(9.6, 5.0), _Pad(11.5, 5.0)]),   # overhangs
        'R1': _F(5.0, 5.0, [_Pad(5.0, 5.0)]),
        'C9': _F(30.0, 30.0, [_Pad(30.0, 30.0)]),                  # in a pile
    }
    m = A.Movie.__new__(A.Movie)
    m.want_panel, m.unplaced, m.inventory = True, False, {}
    m.refresh_placement(pcb, None)
    placed = sum(a for a, _b in m.inventory.values())
    if placed != 2:
        fail('placed %d of 3 -- the overhanging J1 must count, the pile C9 '
             'must not: %r' % (placed, m.inventory))
    if len(_FAIL) == _mark:
        print('  PASS: 2 of 3 placed: the overhanging connector is placed, '
              'the part off the board is not')


def test_leading_copper_free_ignores_arcs_inside_pts():
    _mark = len(_FAIL)
    import make_movie
    d = tempfile.mkdtemp(prefix='t1036lc_')
    try:
        zone = os.path.join(d, 'zone.kicad_pcb')
        with open(zone, 'w') as f:
            f.write('(kicad_pcb\n\t(zone (net 1)\n\t\t(polygon\n\t\t\t(pts '
                    '(xy 0 0) (arc (start 1 1) (mid 2 2) (end 3 3))))\n\t)\n'
                    '\t(gr_poly\n\t\t(pts\n\t\t\t(arc (start 0 0) (mid 1 1) '
                    '(end 2 2))\n\t\t)\n\t)\n)\n')
        steps = [('z', zone, None), ('r', BOARD, None)]
        n = make_movie.leading_copper_free(steps)
    finally:
        shutil.rmtree(d, ignore_errors=True)
    if n != 1:
        fail('a zone/gr_poly arc was read as copper: %d leading copper-free'
             % n)
    if make_movie.leading_copper_free([('r', BOARD, None)]) != 0:
        fail('a routed board was read as copper-free')
    if len(_FAIL) == _mark:
        print('  PASS: pts arcs ignored; the routed board still has copper')


def test_a_strided_gif_holds_exactly_the_cap():
    _mark = len(_FAIL)
    tmp = tempfile.mkdtemp(prefix='t1036gf_')
    n = A.GIF_MAX_FRAMES * 2 + 7
    with frame_spool.FrameSpool() as sp:
        for i in range(n):
            sp.append(Image.new('RGB', (32, 24), (i % 256, (i * 7) % 256,
                                                  (i * 13) % 256)))
        g = os.path.join(tmp, 's.gif')
        A.save_movie(sp, g, fps=6, end_hold=1.0)
    with Image.open(g) as im:
        got = sum(1 for _ in ImageSequence.Iterator(im))
    shutil.rmtree(tmp, ignore_errors=True)
    if got != A.GIF_MAX_FRAMES:
        fail('a %d-frame film made a %d-frame GIF; the cap is %d'
             % (n, got, A.GIF_MAX_FRAMES))
    if len(_FAIL) == _mark:
        print('  PASS: %d frames -> exactly %d in the GIF' % (n, got))


TESTS = (
    test_make_film_closes_its_spools_on_failure,
    test_an_invalid_theme_warns_once_and_iso_opts_are_not_mutated,
    test_a_placement_step_that_lays_copper_plays_it,
    test_duplicate_references_pair_by_uuid,
    test_an_overhanging_part_is_placed,
    test_leading_copper_free_ignores_arcs_inside_pts,
    test_a_strided_gif_holds_exactly_the_cap,
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
