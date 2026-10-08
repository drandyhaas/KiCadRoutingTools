#!/usr/bin/env python3
"""The frame is planned, decided once (#946, #1018).

`BoardRenderer.__init__` derived W/H from the board's bounding box and the
(retired) iso panel derived its panel from `height_frac`, in two places, with
the size invariant enforced by COMMENT rather than by code -- and Pillow does
not raise on a mismatch, it silently resizes every later frame to the first.
The frame is the stage3d frame now (the only film layout).

Three claims pinned here, and the third is the one nothing else can make.

  * **Both dimensions are even**, across the whole cross product. Only the
    HEIGHT was ever forced; `_write_mp4` crops `a.shape[1] & ~1` too, so a
    taller-than-wide board has been losing a pixel column in every mp4 this
    repo has written.
  * **A declared ratio is the size asked for**, the band reserved inside.
  * **A GIF is actually encoded and read back.** Checking the in-memory list
    cannot see the bug this whole invariant exists for: Pillow writes a valid
    file and resizes silently, so the only place the defect is visible is in
    the FILE. The mis-sized control proves the check can see it.
"""
import os
import sys
import tempfile

# stage3d is the only film layout, so an unnamed layout is a stage3d
# frame. These tests grade the 2D board, not the Node/Chromium 3D
# render: set before env_knobs is read.
os.environ.setdefault('KICAD_MOVIE_BOARD3D', '2d')

RUN_ALL_TIMEOUT = 600

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import animate_route as A          # noqa: E402
import frame_layout as FL          # noqa: E402
from kicad_parser import parse_kicad_pcb   # noqa: E402
from route_render import BoardRenderer     # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')

#: Real corpus proportions, as the layout study used them.
SHAPES = {'wide 1.85': (0, 0, 185, 100), '4:3': (0, 0, 130, 100),
          'square': (0, 0, 100, 100), 'tall 1:1.6': (0, 0, 62, 100),
          'very tall': (0, 0, 40, 100)}

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def test_every_plan_is_even_on_both_axes():
    _mark = len(_FAIL)
    n = odd = 0
    ratios = dict(FL.RATIOS, **{'4:1': 4.0, '1:3': 1 / 3.0})
    for lk in ('stage3d',):
        for rk in ratios:
            for sn, bb in SHAPES.items():
                for foot, track in ((0, 0), (37, 0), (0, 61), (37, 61)):
                    n += 1
                    g = FL.plan_frame(bb, ratio=ratios[rk],
                                      size=901, foot_px=foot,
                                      track_px=track)
                    if g.frame.w % 2 or g.frame.h % 2:
                        odd += 1
                        fail('%s/%s/%s foot=%d track=%d -> %dx%d is odd'
                             % (lk, rk, sn, foot, track, g.frame.w, g.frame.h))
    if not odd:
        print('    %d plans across %d ratios x %d shapes x 4 '
              'chrome cases' % (n, len(ratios), len(SHAPES)))
    if len(_FAIL) == _mark:
        print('  PASS: both dimensions even everywhere')


def test_every_named_box_is_inside_the_frame():
    _mark = len(_FAIL)
    for lk in ('16:9', '9:16', '1:1', '4:1'):
        for sn, bb in SHAPES.items():
            g = FL.plan_frame(bb, ratio=FL.parse_ratio(lk), size=800,
                              foot_px=30, track_px=40)
            for name in ('board', 'rail', 'foot', 'panel', 'track'):
                b = getattr(g, name)
                if b is None or b.w <= 0 or b.h <= 0:
                    continue
                if not g.frame.contains(b):
                    fail('%s/%s: %s %s escapes the frame %s'
                         % (lk, sn, name, tuple(b), tuple(g.frame)))
            if g.panel is not None and g.board.overlaps(g.panel):
                fail('%s/%s: the layer column overlaps the board' % (lk, sn))
    if len(_FAIL) == _mark:
        print('  PASS: every box inside the frame; the column off the board')


def test_a_real_gif_encodes_at_one_size_per_ratio():
    """The in-memory list cannot see the defect. The FILE can."""
    _mark = len(_FAIL)
    d = tempfile.mkdtemp()
    for lk in ('16:9', '9:16', '1:1', '4:1'):
        out = os.path.join(d, '%s.gif' % lk.replace(':', 'x'))
        import make_movie
        got = make_movie.make_movie([BOARD], out=out, size=220, quiet=True,
                                    aspect=lk)
        if not got or not os.path.exists(got):
            fail('%s: no film written' % lk)
            continue
        im = Image.open(got)
        sizes = set()
        try:
            while True:
                sizes.add(im.size)
                im.seek(im.tell() + 1)
        except EOFError:
            pass
        if len(sizes) != 1:
            fail('%s: the ENCODED film has %d sizes: %s'
                 % (lk, len(sizes), sizes))
        else:
            print('    %-8s encoded %d frame(s) at %s'
                  % (lk, im.n_frames, sizes.pop()))
    if len(_FAIL) == _mark:
        print('  PASS: one size per film, read back from the file')


def test_the_guard_reports_and_pads_rather_than_squashing():
    """THE CONTROL. Without it, "one size" is true of any film."""
    _mark = len(_FAIL)
    # The odd frame is SMALLER than the first, and that is the whole design of
    # this control: a LARGER one is cropped by the centering paste and fills
    # the canvas, which looks exactly like a squash. Smaller, a pad leaves the
    # corners at the pad colour and a squash does not -- so the two outcomes
    # are distinguishable by pixel, which is what the mutation
    # `mixed-frame-sizes-are-squashed-silently` proved this check could not do
    # when it only compared SIZES. Pillow's silent resize makes every outcome
    # the same size; only the content tells them apart.
    good = [Image.new('RGB', (64, 40), (10, 10, 10)) for _ in range(2)]
    odd = Image.new('RGB', (30, 18), (200, 10, 10))
    out = os.path.join(tempfile.mkdtemp(), 'mixed.gif')
    ok = A.save_movie(good + [odd], out, 6, 0.0)
    if not ok or not os.path.exists(out):
        fail('a mixed-size film was not written at all -- the guard raised '
             'instead of padding, which takes a movie down for a cosmetic '
             'reason')
        return
    im = Image.open(out)
    sizes = set()
    try:
        while True:
            sizes.add(im.size)
            im.seek(im.tell() + 1)
    except EOFError:
        pass
    if sizes != {(64, 40)}:
        fail('the padded film is %s, expected {(64, 40)}' % sizes)
    # PADDED, not squashed: the corners of the odd frame must be the pad
    # colour and its centre must be the frame's own content.
    # the LAST frame, found by walking: the GIF encoder collapses runs of
    # identical frames, so the odd one is not reliably at index 2.
    im.seek(0)
    last = 0
    try:
        while True:
            im.seek(im.tell() + 1)
            last = im.tell()
    except EOFError:
        pass
    im.seek(last)
    px = im.convert('RGB')
    corner = px.getpixel((1, 1))
    middle = px.getpixel((32, 20))
    if corner == middle:
        fail('the odd frame was SQUASHED to fill the canvas (corner %s == '
             'centre %s) -- padding is what keeps a mis-sized frame honest'
             % (corner, middle))
    elif middle[0] < 100:
        fail('the odd frame did not survive the pad: centre is %s' % (middle,))
    else:
        print('    padded: corner %s, centre %s' % (corner, middle))
    # and the guard must actually DETECT it, not merely survive
    try:
        FL.assert_frames_uniform([(64, 40), (64, 40), (64, 52)])
        fail('assert_frames_uniform accepted a mixed set -- it cannot see the '
             'defect, so a pass from it proves nothing')
    except FL.FrameSizeError as exc:
        if '64x52' not in str(exc) or '64x40' not in str(exc):
            fail('the refusal does not name both sizes: %s' % exc)
        else:
            print('    refusal: %s' % str(exc)[:96])
    if len(_FAIL) == _mark:
        print('  PASS: detected, reported, padded -- not squashed, not lost')


def test_set_canvas_keys_the_margin_to_the_box():
    """UPDATED DELIBERATELY (#946 review). This pinned that `set_canvas` left
    the size-keyed margin alone, so a layout box inherited a margin sized for
    the whole frame: the r2 stills' 16:9 board filled ~600x370 of a 980x594
    box, and a 124 px box at size 400 kept 12 px of it each side. The default
    canvas keeps the pinned arithmetic (`test_431_render_seams`); a canvas the
    LAYOUT chose keys its margin to the box's short side."""
    _mark = len(_FAIL)
    import route_render as RRm
    pcb = parse_kicad_pcb(BOARD)
    r = BoardRenderer(pcb, size=600, supersample=2)
    if abs(r._margin_px - 0.03 * 600 * 2) > 1e-9:
        fail('the DEFAULT canvas margin moved: %r' % r._margin_px)
    r.set_canvas(321, 222)
    if r.frame(segments=[], vias=[]).size != (321, 222):
        fail('set_canvas did not take: %s' % (r.frame(segments=[], vias=[]).size,))
    want = RRm.CANVAS_MARGIN_FRAC * 222 * 2
    if abs(r._margin_px - want) > 1e-9:
        fail('set_canvas margin %r, expected %r (the box short side)'
             % (r._margin_px, want))
    if len(_FAIL) == _mark:
        print('  PASS: default canvas keeps its margin; a layout canvas keys '
              'it to the box')


def _declared(size, ratio):
    if ratio >= 1.0:
        w, h = size, max(1, int(round(size / ratio)))
    else:
        w, h = max(1, int(round(size * ratio))), size
    return FL.even(w), FL.even(h)


def test_a_declared_ratio_is_the_size_asked_for_with_the_band_inside():
    """#946/C4, as PLAN data over the whole cross product. With a declared
    ratio the frame is EXACTLY that ratio's size whatever chrome is reserved:
    the attempts band (`track_px`) and the clock (`foot_px`) come out of the
    board's share instead of growing the frame -- they used to be ADDED, so a
    16:9 film with a band was 1600x1036. The band sits inside the frame and
    off the board.
    """
    _mark = len(_FAIL)
    n = 0
    for lk in ('stage3d',):
        for rk, ratio in FL.RATIOS.items():
            if not ratio:
                continue
            for sn, bb in SHAPES.items():
                n += 1
                g = FL.plan_frame(bb, ratio=ratio, size=900,
                                  track_px=120, foot_px=0)
                want = _declared(900, ratio)
                if (g.frame.w, g.frame.h) != want:
                    fail('%s/%s/%s: frame %dx%d, declared %dx%d'
                         % (lk, rk, sn, g.frame.w, g.frame.h,
                            want[0], want[1]))
                    continue
                t = g.track
                if t is None or not g.frame.contains(t):
                    fail('%s/%s/%s: band %r is not inside the frame'
                         % (lk, rk, sn, t))
                elif t.overlaps(g.board):
                    fail('%s/%s/%s: band %r overlaps the board %r'
                         % (lk, rk, sn, tuple(t), tuple(g.board)))
    if len(_FAIL) == _mark:
        print('  PASS: %d declared plans hold their size, band inside, off '
              'the board' % n)


def test_the_encoded_film_is_the_declared_size_in_both_themes():
    """The same claim read back from real FILES: ratio x theme, each with a
    band, encoded and measured. The render path is where the band used to
    grow the frame (attached after the fact), so a plan that is right and a
    film that is not is the failure this exists to see. The band is the
    stage3d frame's benchmark band, from a converge ledger."""
    _mark = len(_FAIL)
    import json
    import make_movie
    d = tempfile.mkdtemp()
    led = os.path.join(d, 'ledger.jsonl')
    with open(led, 'w', encoding='utf-8') as f:
        for i in range(8):
            f.write(json.dumps({'iteration': i, 'kind': 'completion',
                                'accepted': i % 3 != 1, 't': 1e9 + 60 * i,
                                'score': {'blocking': 20 - i}}) + chr(10))
    n = 0
    for lk in ('stage3d',):
        for rk in ('16:9', '9:16', '1:1'):
            for th in ('dark', 'light'):
                n += 1
                out = os.path.join(d, '%s_%s_%s.gif'
                                   % (lk, rk.replace(':', 'x'), th))
                import contextlib
                import io
                err = io.StringIO()
                # 1000 px, so the 16:9 frame (1000x562) can carry the
                # 64 px band under the board's 70% floor: a film where the
                # band DECLINED would pass the size check without testing it.
                with contextlib.redirect_stderr(err):
                    got = make_movie.make_movie(
                        [BOARD], out=out, size=1000, quiet=True,
                        aspect=rk, theme=th, attempts_ledger=led)
                if not got:
                    fail('%s/%s/%s: no film' % (lk, rk, th))
                    continue
                if 'benchmark band: converge' not in err.getvalue():
                    fail('%s/%s/%s: the band was not drawn into a reserved '
                         'box, so this film does not test the claim: %s'
                         % (lk, rk, th, err.getvalue()[-200:]))
                with Image.open(got) as im:
                    sz = im.size
                    corner = im.convert('RGB').getpixel((sz[0] - 1, 0))
                want = _declared(1000, FL.parse_ratio(rk))
                if sz != want:
                    fail('%s/%s/%s: encoded %s, declared %s'
                         % (lk, rk, th, sz, want))
                # the theme reached the frame: its top-right corner is the
                # theme's ground or chrome, never the OTHER theme's ground
                other = ('light' if th == 'dark' else 'dark')
                if corner == RT_theme(other).rgb('ground'):
                    fail('%s/%s/%s: the frame corner is the %s ground'
                         % (lk, rk, th, other))
    import shutil
    shutil.rmtree(d, ignore_errors=True)
    if len(_FAIL) == _mark:
        print('  PASS: %d films (3 ratios x 2 themes) encoded at '
              'the declared size, band inside' % n)


def RT_theme(name):
    import render_theme
    return render_theme.theme(name)


TESTS = (
    test_a_declared_ratio_is_the_size_asked_for_with_the_band_inside,
    test_the_encoded_film_is_the_declared_size_in_both_themes,
    test_every_plan_is_even_on_both_axes,
    test_every_named_box_is_inside_the_frame,
    test_a_real_gif_encodes_at_one_size_per_ratio,
    test_the_guard_reports_and_pads_rather_than_squashing,
    test_set_canvas_keys_the_margin_to_the_box,
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
