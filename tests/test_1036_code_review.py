#!/usr/bin/env python3
"""PR C's pre-push code review, each finding pinned (#1036, #946).

  1. make_film closes BOTH FrameSpools on any exception (an injected
     append that raises 'disk full', during the board frames and during
     assembly) -- it left two krt_frames_* directories behind; and it takes
     --max-frames.
  1b. make_film resolves its frame budget exactly as make_movie does, through
      the one `make_movie.resolve_max_frames`: None -> $KICAD_MOVIE_MAX_FRAMES
      -> 2400, and an explicit value (0 = none) wins. It used to hand None to
      build_boards, which reads it as "no budget".
  4. An invalid $KICAD_RENDER_THEME warns ONCE per film, not per frame; a
     caller's IsoOpts is not mutated.
  5a. A synthesised placement step that also lays copper plays that copper
      through the normal reveal (its trace), not a silent snap.
  5b. Stage keys boards by resolved absolute path: two chain boards with one
      basename are not one round.
  5c. Duplicate-reference `~N` keys: pairing by uuid; an ordinal whose block
      count changed is not read as a move. A uuid two blocks SHARE is no
      identity: those pair by reference, so cap_chain against itself moves
      nothing.
  6.  A part overhanging the outline is placed; only a part entirely off it
      counts as unplaced.
  7.  `leading_copper_free` ignores `(arc` inside a zone's `(pts ...)`.
  GIF: a strided GIF holds EXACTLY `GIF_MAX_FRAMES`.
  Lazy overlays: a frame the attempts band or the run clock fails to draw
      drops THAT overlay for the rest of the film, said once, and the film
      is still written, one size throughout; a frame that cannot be PRODUCED
      is re-raised as itself, never reported as "mp4 encode failed".

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


def _spool_dirs(where):
    return set(glob.glob(os.path.join(where, 'krt_frames_*')))


def test_make_film_closes_its_spools_on_failure():
    """Counted in a PRIVATE temp dir: the spools are made by
    `tempfile.mkdtemp`, so pointing `tempfile.tempdir` at a directory this
    test owns means another film rendering on the machine at the same time
    cannot leave a `krt_frames_*` directory the count would blame on this
    one."""
    _mark = len(_FAIL)
    import make_film
    tmp = tempfile.mkdtemp(prefix='t1036cr_')
    private = os.path.join(tmp, 'spools')
    os.makedirs(private)
    saved_tempdir = tempfile.tempdir
    tempfile.tempdir = private
    try:
        _closes_its_spools(make_film, tmp, private)
    finally:
        tempfile.tempdir = saved_tempdir
        shutil.rmtree(tmp, ignore_errors=True)


def _closes_its_spools(make_film, tmp, private):
    _mark = len(_FAIL)
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
        before = _spool_dirs(private)
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
        left = _spool_dirs(private) - before
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
    if len(_FAIL) == _mark:
        print('  PASS: failures at append 2 and %d of %d leave no spool dir; '
              '--max-frames accepted' % (total - 1, total))


def test_make_film_resolves_the_frame_budget_like_make_movie():
    """Both front ends hand build_boards the SAME budget for the same ask."""
    _mark = len(_FAIL)
    import env_knobs
    import make_film
    import make_movie
    seen = []

    def _spy(*a, **kw):
        seen.append(kw.get('max_frames'))
        return []
    orig = A.build_boards
    old_env = os.environ.pop('KICAD_MOVIE_MAX_FRAMES', None)
    tmp = tempfile.mkdtemp(prefix='t1036mf_')
    A.build_boards = _spy
    got = {}
    try:
        for env, flag, want in ((None, None, make_movie.DEFAULT_MAX_FRAMES),
                                ('7', None, 7), ('7', 5, 5), ('7', 0, 0),
                                ('0', None, 0)):
            if env is None:
                os.environ.pop('KICAD_MOVIE_MAX_FRAMES', None)
            else:
                os.environ['KICAD_MOVIE_MAX_FRAMES'] = env
            env_knobs.refresh()
            argv = [BOARD, BOARD, '-o', os.path.join(tmp, 'f.gif'),
                    '--quiet', '--camera', 'off', '--no-attempts',
                    '--no-placement-panel']
            if flag is not None:
                argv += ['--max-frames', str(flag)]
            del seen[:]
            with contextlib.redirect_stderr(io.StringIO()):
                make_film.main(argv)
                make_movie.make_movie([BOARD, BOARD],
                                      out=os.path.join(tmp, 'm.gif'),
                                      quiet=True, camera='off',
                                      attempts=False, placement_panel=False,
                                      max_frames=flag)
            got[(env, flag)] = list(seen)
            if seen != [want, want]:
                fail('env %r, --max-frames %r: build_boards got %r from '
                     '(make_film, make_movie), want %r for both'
                     % (env, flag, seen, want))
    finally:
        A.build_boards = orig
        if old_env is None:
            os.environ.pop('KICAD_MOVIE_MAX_FRAMES', None)
        else:
            os.environ['KICAD_MOVIE_MAX_FRAMES'] = old_env
        env_knobs.refresh()
        shutil.rmtree(tmp, ignore_errors=True)
    if len(_FAIL) == _mark:
        print('  PASS: make_film and make_movie resolve one budget: %r' % got)


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


def test_a_shared_uuid_is_not_an_identity():
    """Two blocks sharing one uuid (a text-editor copy: cap_chain's C1/C2 and
    J1/J2 do) pair by REFERENCE, as before #1036 -- pairing on the shared uuid
    matched C1 to C2 and read a board compared with itself as moving parts,
    which turned the camera on for a film in which nothing moved."""
    _mark = len(_FAIL)
    import kicad_parser
    import movie_camera as MC
    cap = os.path.join(KF, 'cap_chain.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = kicad_parser.parse_kicad_pcb(cap)
        _u = [f.uuid for f in pcb.footprints.values() if f.uuid]
        if len(set(_u)) == len(_u):
            fail('BROKEN: cap_chain no longer carries a shared footprint '
                 'uuid, so this test pins nothing')
        rounds = MC.synth_rounds([cap, cap])
    if rounds:
        fail('cap_chain against itself reports moves: %r'
             % [m for rd in rounds for m in rd['moved']])
    # ...and a REAL move of a part whose uuid is shared is still seen, by ref
    boards = {
        'a': _Pcb({'C1': _Fp(0, 0, 'dup'), 'C2': _Fp(5, 0, 'dup')}),
        'b': _Pcb({'C1': _Fp(0, 0, 'dup'), 'C2': _Fp(5, 9, 'dup')}),
    }
    orig = kicad_parser.parse_kicad_pcb
    kicad_parser.parse_kicad_pcb = lambda path, *a, **k: boards[path]
    try:
        r = MC.synth_rounds(['a', 'b'], min_mm=0.5)
    finally:
        kicad_parser.parse_kicad_pcb = orig
    got = sorted(m['reference'] for rd in r for m in rd['moved'])
    if got != ['C2']:
        fail('a shared-uuid pair where only C2 moved read as %r' % got)
    if len(_FAIL) == _mark:
        print('  PASS: a shared uuid pairs by reference: cap_chain vs itself '
              'moves nothing, and a real move is still C2 alone')


def _two_attempts_track():
    import movie_attempts as MA
    return MA.Track(tuple(MA.Attempt(i, 'r%d' % i, 'round',
                                     (i - 1) if i else None, True, False,
                                     float(10 - i), False, 'b')
                          for i in range(4)), 'failures', 'loop', 'x')


def _gif_frames(path):
    with Image.open(path) as im:
        return [fr.size for fr in ImageSequence.Iterator(im)]


def test_a_failing_overlay_costs_the_overlay_not_the_film():
    """The post-passes run LAZILY, while the encoder streams. One frame an
    overlay failed to draw used to raise out of the encoder -- past the
    `try` that registered the overlay -- and lose the whole film."""
    _mark = len(_FAIL)
    import cmd_timing
    import make_movie
    import movie_attempts as MA
    clock = os.path.join(_TESTS, 'fixtures', 'cmd_timing',
                         'synthetic_run.jsonl')
    tmp = tempfile.mkdtemp(prefix='t1036ov_')

    def _film(name, **kw):
        err = io.StringIO()
        out = os.path.join(tmp, name + '.gif')
        got = None
        with contextlib.redirect_stderr(err):
            # 400 px: at 200 the band declines (over its share of the frame)
            try:
                got = make_movie.make_movie([BOARD], out=out, size=400,
                                            quiet=True, camera='off',
                                            placement_panel=False, **kw)
            except Exception as exc:                            # noqa: BLE001
                # the regression itself: the overlay's error escaped
                err.write(' RAISED %s: %s' % (type(exc).__name__, exc))
        return got, err.getvalue()

    try:
        # the attempts band: the probe draw passes, frame 2's draw raises
        ok_path, _e = _film('band_ok', attempts=_two_attempts_track())
        orig, calls = MA.draw_track, [0]

        def _boom(*a, **k):
            calls[0] += 1
            if calls[0] >= 3:
                raise RuntimeError('band draw failed (injected)')
            return orig(*a, **k)
        MA.draw_track = _boom
        try:
            got, err = _film('band', attempts=_two_attempts_track())
        finally:
            MA.draw_track = orig
        # the run clock: frame 2's band raises
        c_ok, _e2 = _film('clock_ok', attempts=False, timing=clock)
        c_orig, c_calls = cmd_timing.add_clock_band, [0]

        def _cboom(*a, **k):
            c_calls[0] += 1
            if c_calls[0] >= 3:
                raise RuntimeError('clock draw failed (injected)')
            return c_orig(*a, **k)
        cmd_timing.add_clock_band = _cboom
        try:
            c_got, c_err = _film('clock', attempts=False, timing=clock)
        finally:
            cmd_timing.add_clock_band = c_orig
        for what, path, e, ctl in (('attempts band', got, err, ok_path),
                                   ('run clock', c_got, c_err, c_ok)):
            if not (path and os.path.isfile(path)):
                fail('%s: a failed overlay frame lost the film: %r'
                     % (what, e[-300:]))
                continue
            n = e.count('%s DROPPED' % what)
            if n != 1:
                fail('%s: said %d times, want once: %r' % (what, n, e[-300:]))
            if 'mp4 encode failed' in e or 'MIXED FRAME SIZES' in e:
                fail('%s: the drop was misreported: %r' % (what, e[-300:]))
            sizes = set(_gif_frames(path))
            want = set(_gif_frames(ctl))
            if len(sizes) != 1 or sizes != want:
                fail('%s: frames %r, the undamaged film is %r'
                     % (what, sorted(sizes), sorted(want)))
        if c_calls[0] < 3:
            fail('BROKEN: the run clock was never drawn, so its arm pins '
                 'nothing')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # A frame that cannot be PRODUCED is the film's defect: re-raised as
    # itself, not "mp4 encode failed" and a GIF retry. An ENCODER error still
    # falls back. A stand-in imageio, so this runs without imageio-ffmpeg.
    import types
    state = {'appended': 0, 'closed': 0, 'raise_on_append': False}

    class _W(object):
        def append_data(self, _a):
            if state['raise_on_append']:
                raise IOError('encoder broke (injected)')
            state['appended'] += 1

        def close(self):
            state['closed'] += 1
    fake = types.ModuleType('imageio')
    fake_v2 = types.ModuleType('imageio.v2')
    fake_v2.get_writer = lambda *a, **k: _W()
    fake.v2 = fake_v2
    saved = {k: sys.modules.get(k) for k in ('imageio', 'imageio.v2')}
    sys.modules['imageio'], sys.modules['imageio.v2'] = fake, fake_v2
    tmp2 = tempfile.mkdtemp(prefix='t1036mp_')
    try:
        def _frames():
            yield Image.new('RGB', (8, 8))
            raise ValueError('frame 1 could not be composed (injected)')
        err = io.StringIO()
        raised = None
        with contextlib.redirect_stderr(err):
            try:
                A._write_mp4(_frames(), os.path.join(tmp2, 'p.mp4'), 6)
            except ValueError as exc:
                raised = exc
        if raised is None:
            fail('a frame that could not be produced was swallowed')
        if 'mp4 encode failed' in err.getvalue():
            fail('a frame-production error was reported as an encoder '
                 'failure: %r' % err.getvalue())
        if not state['closed']:
            fail('the writer was left open after the frame error')
        state['raise_on_append'] = True
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            ok = A._write_mp4(iter([Image.new('RGB', (8, 8))]),
                              os.path.join(tmp2, 'q.mp4'), 6)
        if ok is not False or 'mp4 encode failed' not in err.getvalue():
            fail('an encoder failure did not fall back: %r, %r'
                 % (ok, err.getvalue()))
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v
        shutil.rmtree(tmp2, ignore_errors=True)
    if len(_FAIL) == _mark:
        print('  PASS: a failed band or clock frame drops that overlay once '
              'and the film is written one size; a frame error is re-raised, '
              'an encoder error falls back')


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
    test_make_film_resolves_the_frame_budget_like_make_movie,
    test_an_invalid_theme_warns_once_and_iso_opts_are_not_mutated,
    test_a_placement_step_that_lays_copper_plays_it,
    test_duplicate_references_pair_by_uuid,
    test_a_shared_uuid_is_not_an_identity,
    test_a_failing_overlay_costs_the_overlay_not_the_film,
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
