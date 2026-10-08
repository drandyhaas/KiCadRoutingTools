#!/usr/bin/env python3
"""The `stage3d` layout (#1081): the board gets at least 70% x 70%.

`stage3d` is the film's only layout: its board box holds the 3D board, with the layer
column to its right and one benchmark band along the bottom. Its promise is a
FLOOR, not a ratio: the box is at least `STAGE3D_BOARD_W_FRAC` of the frame's
width and `STAGE3D_BOARD_H_FRAC` of its height, whatever else asks for room.
What this file pins:

  * **the floor holds** across every named ratio, three sizes and five board
    shapes, with and without a band -- on a LANDSCAPE frame both floors, on a
    portrait one the height (the board then takes the full width);
  * **a band that would breach the floor is shrunk, then declined -- never the
    board**, and either is SAID in `frame_status_line`, because a missing band
    that nothing explains reads as a missing feature;
  * **portrait turns the column into a row** under the board, and drops it
    (said) when it would be too short to read;
  * **an extreme declared aspect stays a stage3d frame at the ratio asked
    for**, board-only: no layer column, the board box the full width, the
    band only if it fits -- and said;
  * **the retired flags are gone**: neither CLI offers `--layout` or
    `--panels`, and a script still passing one is refused (exit 2) by
    argparse, naming the flag, rather than ignored;
  * **a retired layout name given as the ASPECT** (`--aspect stacked`,
    `$KICAD_MOVIE_ASPECT=sidebar`, `build_boards(aspect='inset')`) is said
    once as retired and films at the default 16:9 on every path, where it
    used to raise and lose the movie.
"""
import os
import subprocess
import sys

RUN_ALL_FAST_OK = True

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import frame_layout as FL                                       # noqa: E402

_FAIL = []

SHAPES = {'wide 1.85': (0, 0, 185, 100), '4:3': (0, 0, 133, 100),
          'square': (0, 0, 100, 100), 'tall 1:1.6': (0, 0, 100, 160),
          'very wide 6.5': (0, 0, 650, 100)}


def _check(ok, msg):
    if not ok:
        print('  FAIL ' + msg)
        _FAIL.append(msg)
    return ok


def _plan(bb, ratio, size, track, foot=24):
    return FL.plan_frame(bb, ratio=FL.parse_ratio(ratio),
                         size=size, foot_px=foot,
                         track_px=track, quiet=True)


def test_the_board_keeps_seventy_by_seventy():
    mark = len(_FAIL)
    n = 0
    for ratio in [r for r in FL.RATIOS if r != 'board'] + [None]:
        for size in (500, 1000, 1400):
            for sn, bb in SHAPES.items():
                for track in (0, 120, 400):
                    g = _plan(bb, ratio, size, track)
                    n += 1
                    if any('outside' in x for x in g.notes):
                        continue            # an extreme ratio; tested below
                    W, H = g.frame.w, g.frame.h
                    land = W >= FL.ISO_SIDE_ASPECT * H
                    tag = '%s/%s/%s/band %d' % (ratio, size, sn, track)
                    small = any('too small' in n for n in g.notes)
                    _check(g.board.h >= FL.STAGE3D_BOARD_H_FRAC * H or small,
                           '%s: board %d of %d px high, and nothing says '
                           'why' % (tag, g.board.h, H))
                    if land:
                        _check(g.board.w >= FL.STAGE3D_BOARD_W_FRAC * W
                               or small, '%s: board %d of %d px wide'
                               % (tag, g.board.w, W))
                        _check(g.panel is not None and g.panel.x == g.board.w,
                               '%s: the layer column sits right of the board'
                               % tag)
                    else:
                        _check(g.board.w == W, '%s: a portrait board takes '
                               'the full width' % tag)
                    _check(g.board.x == 0 and g.board.y == g.rail.h,
                           '%s: the board is top-left' % tag)
                    if g.track is not None:
                        _check(g.track.w == W and g.track.y >= g.board.y
                               + g.board.h, '%s: the band is full width '
                               'under the board' % tag)
    if len(_FAIL) == mark:
        print('  PASS: 70 x 70 holds on %d plans' % n)


def test_a_band_is_shrunk_then_declined_and_either_is_said():
    mark = len(_FAIL)
    g = _plan(SHAPES['wide 1.85'], '16:9', 1400, 400)
    _check(g.track is not None and g.track.h < 400
           and g.board.h >= FL.STAGE3D_BOARD_H_FRAC * g.frame.h,
           '1400 16:9, band 400: shrunk to %s, board %d of %d'
           % (g.track and g.track.h, g.board.h, g.frame.h))
    _check('band 400 ->' in FL.frame_status_line(g),
           'the shrink is said: %r' % FL.frame_status_line(g))
    g = _plan(SHAPES['wide 1.85'], '16:9', 500, 120)
    _check(g.track is None, '500 16:9: the band is declined (%s)' % (g.track,))
    _check('no benchmark band' in FL.frame_status_line(g),
           'the decline is said: %r' % FL.frame_status_line(g))
    g = _plan(SHAPES['wide 1.85'], '16:9', 1400, 120)
    _check(g.track is not None and g.track.h == 120 and not any(
        'band' in n for n in g.notes),
           'a band that fits is untouched and unremarked (%s, %s)'
           % (g.track, g.notes))
    if len(_FAIL) == mark:
        print('  PASS: shrink, decline, and both said')


def test_a_floor_it_cannot_keep_is_said():
    """The phase-3 verifier: at size 100 the board was 14% of the frame,
    and with a 250 px clock band 47%, with no note and no error."""
    mark = len(_FAIL)
    for size, foot in ((100, 0), (200, 0), (1000, 250)):
        g = _plan(SHAPES['wide 1.85'], '16:9', size, 0, foot=foot)
        if g.board.h < FL.STAGE3D_BOARD_H_FRAC * g.frame.h:
            _check(any('too small' in n for n in g.notes),
                   'size %d, foot %d: board %dx%d of %dx%d is said (%s)'
                   % (size, foot, g.board.w, g.board.h, g.frame.w,
                      g.frame.h, g.notes))
    g = _plan(SHAPES['wide 1.85'], '16:9', 1000, 0)
    _check(not any('too small' in n for n in g.notes),
           'a frame that keeps the floor says nothing about it')
    if len(_FAIL) == mark:
        print('  PASS: a broken floor is always said')


def test_portrait_makes_the_column_a_row_or_says_why_not():
    mark = len(_FAIL)
    g = _plan(SHAPES['square'], '9:16', 1400, 0)
    _check(g.panel is not None and g.panel.y == g.board.y + g.board.h
           and g.panel.w == g.frame.w,
           '9:16: the layer row sits under the board (%s)' % (g.panel,))
    _check('row under the board' in FL.frame_status_line(g),
           'and it is said: %r' % FL.frame_status_line(g))
    g = _plan(SHAPES['square'], '9:16', 500, 300)
    _check(g.panel is None and 'no layer row' in FL.frame_status_line(g),
           '9:16 500 with a tall band: the row is dropped and said (%s, %r)'
           % (g.panel, FL.frame_status_line(g)))
    if len(_FAIL) == mark:
        print('  PASS: portrait row, or a stated drop')


def test_an_extreme_aspect_is_a_board_only_stage3d_frame_and_says_so():
    """No legacy frame to fall back to any more. The declared ratio is
    KEPT, the board box takes the whole width under the rail, there is no
    layer column, and a band only when the board keeps its height floor."""
    mark = len(_FAIL)
    for ratio in ('4:1', '1:3'):
        want = FL.parse_ratio(ratio)
        for size in (500, 1400):
            for track in (0, 120):
                g = _plan(SHAPES['wide 1.85'], ratio, size, track)
                line = FL.frame_status_line(g)
                tag = '%s size %d band %d' % (ratio, size, track)
                _check(g.layout == 'stage3d' and 'outside' in line
                       and 'no layer column' in line,
                       '%s: stage3d, board-only, said (%s: %r)'
                       % (tag, g.layout, line))
                _check(abs(g.frame.w / float(g.frame.h) - want) < 0.05,
                       '%s: the declared ratio is kept (%dx%d)'
                       % (tag, g.frame.w, g.frame.h))
                _check(g.panel is None and g.board.w == g.frame.w
                       and g.board.y == g.rail.h and g.rail.h > 0,
                       '%s: the board box is the full width under the rail, '
                       'no column (%s, panel %s)' % (tag, g.board, g.panel))
                if g.track is not None:
                    _check(g.board.h >= FL.STAGE3D_BOARD_H_FRAC * g.frame.h,
                           '%s: a band only when the board keeps its floor'
                           % tag)
    if len(_FAIL) == mark:
        print('  PASS: extreme aspects are board-only stage3d frames, said')


def test_the_retired_flags_are_gone_from_both_clis():
    """stage3d is the only film layout, so neither CLI offers `--layout` or
    `--panels` any more, and a script still passing one is REFUSED by
    argparse (exit 2, naming the flag) rather than silently ignored."""
    mark = len(_FAIL)
    for script in (os.path.join(ROOT, 'py_router', 'make_movie.py'),
                   os.path.join(ROOT, 'py_tools', 'make_film.py')):
        name = os.path.basename(script)
        r = subprocess.run([sys.executable, script, '--help'],
                           capture_output=True, text=True, timeout=120,
                           cwd=ROOT)
        _check(r.returncode == 0, '%s --help exited %d: %s'
               % (name, r.returncode, r.stderr[-300:]))
        for flag in ('--layout', '--panels'):
            _check(flag not in r.stdout, '%s --help still offers %s'
                   % (name, flag))
        _check('--aspect' in r.stdout, '%s --help lost --aspect' % name)
        for flag, value in (('--layout', 'sidebar'), ('--panels', 'xray')):
            r = subprocess.run([sys.executable, script, 'x.kicad_pcb', flag,
                                value], capture_output=True, text=True,
                               timeout=120, cwd=ROOT)
            _check(r.returncode == 2
                   and 'unrecognized arguments: %s' % flag in r.stderr,
                   '%s %s %s: exit %d, %r' % (name, flag, value,
                                              r.returncode, r.stderr[-200:]))
    if len(_FAIL) == mark:
        print('  PASS: --layout and --panels are gone, and refused')


_BOARD = os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb')
_MOVIE_KNOBS = ('KICAD_MOVIE_LAYOUT', 'KICAD_MOVIE_PANELS',
                'KICAD_MOVIE_ASPECT', 'KICAD_MOVIE_BOARD3D')

_PARSE_PROBE = r'''
import sys
sys.path[:0] = [%r]
import frame_layout as FL
for name in FL.RETIRED_LAYOUTS + ('STACKED',):
    print('RATIO', name, FL.parse_ratio(name), FL.parse_ratio(name))
try:
    FL.parse_ratio('banana')
    print('GARBAGE accepted')
except ValueError:
    print('GARBAGE raises')
'''

_API_PROBE = r'''
import sys
sys.path[:0] = [%r, %r]
import animate_route as A
g = []
A.build_boards([('s', %r, None)], %r, 320, 1, None, 2, 6, aspect='inset',
               board3d='2d', geom_out=g)
print('FRAME', g[0].frame.w, g[0].frame.h)
'''


def _run(argv, env_extra=None):
    env = {k: v for k, v in os.environ.items() if k not in _MOVIE_KNOBS}
    env['KICAD_MOVIE_BOARD3D'] = '2d'
    env.update(env_extra or {})
    return subprocess.run([sys.executable, '-X', 'utf8'] + argv,
                          capture_output=True, text=True, timeout=300,
                          cwd=ROOT, env=env)


def _said(stderr, name):
    return [ln for ln in stderr.splitlines()
            if "aspect '%s' is retired" % name in ln]


def _gif_size(path):
    from PIL import Image
    if not os.path.isfile(path):
        return None
    with Image.open(path) as im:
        return im.size


def test_a_retired_layout_name_as_the_aspect_is_said_not_fatal():
    """`--aspect stacked` was the stacked layout's ratio, and a layout name
    can still arrive where the aspect goes -- on the CLI, in
    `$KICAD_MOVIE_ASPECT`, or through the API. Each is SAID once as retired
    and the frame is the default stage3d 16:9; it must not take the movie
    down. A value that is no ratio and no retired name still raises."""
    mark = len(_FAIL)
    import tempfile
    r = _run(['-c', _PARSE_PROBE % os.path.join(ROOT, 'py_router')])
    _check(r.returncode == 0, 'the parse probe ran: %s' % r.stderr[-300:])
    for name in FL.RETIRED_LAYOUTS:
        _check('RATIO %s None None' % name in r.stdout,
               'parse_ratio(%r) declares nothing (%r)' % (name, r.stdout))
        _check(len(_said(r.stderr, name)) == 1,
               '%r is said once over two calls (%r)'
               % (name, _said(r.stderr, name)))
    _check('RATIO STACKED None None' in r.stdout,
           'a retired name is matched case-blind')
    _check('GARBAGE raises' in r.stdout,
           'a value that is no ratio and no retired name still raises')
    d = tempfile.mkdtemp(prefix='t1081_aspect_')
    arms = []
    out = os.path.join(d, 'cli.gif')
    arms.append(('make_movie --aspect stacked', 'stacked', out,
                 _run([os.path.join(ROOT, 'py_router', 'make_movie.py'),
                       _BOARD, '--aspect', 'stacked', '--size', '320',
                       '-o', out])))
    out = os.path.join(d, 'env.gif')
    arms.append(('make_movie $KICAD_MOVIE_ASPECT=split', 'split', out,
                 _run([os.path.join(ROOT, 'py_router', 'make_movie.py'),
                       _BOARD, '--size', '320', '-o', out],
                      {'KICAD_MOVIE_ASPECT': 'split'})))
    out = os.path.join(d, 'film.gif')
    arms.append(('make_film --aspect sidebar', 'sidebar', out,
                 _run([os.path.join(ROOT, 'py_tools', 'make_film.py'),
                       _BOARD, '--aspect', 'sidebar', '--size', '320',
                       '--no-cards', '-o', out])))
    run_dir = os.path.join(d, 'run')
    os.makedirs(run_dir)
    import shutil
    shutil.copy(_BOARD, run_dir)
    out = os.path.join(d, 'run.gif')
    arms.append(('animate_route --run-dir $KICAD_MOVIE_ASPECT=legacy',
                 'legacy', out,
                 _run([os.path.join(ROOT, 'py_router', 'animate_route.py'),
                       '--run-dir', run_dir, '--size', '320', '-o', out],
                      {'KICAD_MOVIE_ASPECT': 'legacy'})))
    for tag, name, out, r in arms:
        _check(r.returncode == 0, '%s: exit %d (%s)'
               % (tag, r.returncode, r.stderr[-300:]))
        _check(_gif_size(out) == (320, 180),
               '%s: the film is the default 16:9 frame (%s)'
               % (tag, _gif_size(out)))
        _check(len(_said(r.stderr, name)) == 1,
               '%s: the retired name is said once (%r)'
               % (tag, _said(r.stderr, name)))
    r = _run(['-c', _API_PROBE % (ROOT, os.path.join(ROOT, 'py_router'),
                                  _BOARD, _BOARD)])
    _check(r.returncode == 0 and 'FRAME 320 180' in r.stdout
           and len(_said(r.stderr, 'inset')) == 1,
           "build_boards(aspect='inset'): the default 16:9 frame, said "
           "(exit %d, %r, %r)" % (r.returncode, r.stdout[-200:],
                                  r.stderr[-300:]))
    shutil.rmtree(d, ignore_errors=True)
    if len(_FAIL) == mark:
        print('  PASS: a retired layout name as the aspect is said once and '
              'films at 16:9, on the CLI, env and API paths')


TESTS = (
    test_the_board_keeps_seventy_by_seventy,
    test_a_band_is_shrunk_then_declined_and_either_is_said,
    test_a_floor_it_cannot_keep_is_said,
    test_portrait_makes_the_column_a_row_or_says_why_not,
    test_an_extreme_aspect_is_a_board_only_stage3d_frame_and_says_so,
    test_the_retired_flags_are_gone_from_both_clis,
    test_a_retired_layout_name_as_the_aspect_is_said_not_fatal,
)


def main():
    for fn in TESTS:
        print('%s:' % fn.__name__)
        fn()
    if _FAIL:
        print('')
        print('%d FAILURE(S)' % len(_FAIL))
        for msg in _FAIL[:40]:
            print('  - %s' % msg)
        return 1
    print('')
    print('all %d checks passed' % len(TESTS))
    return 0


if __name__ == '__main__':
    sys.exit(main())
