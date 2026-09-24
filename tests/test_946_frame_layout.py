#!/usr/bin/env python3
"""The frame is a NAMED layout, decided once (#946, #1018).

`BoardRenderer.__init__` derived W/H from the board's bounding box and
`movie_panels.panel_geometry` derived the panel from `height_frac`, in two
places, with the size invariant enforced by COMMENT rather than by code -- and
Pillow does not raise on a mismatch, it silently resizes every later frame to
the first.

Four claims pinned here, and the fourth is the one nothing else can make.

  * **Both dimensions are even**, across the whole cross product. Only the
    HEIGHT was ever forced; `_write_mp4` crops `a.shape[1] & ~1` too, so a
    taller-than-wide board has been losing a pixel column in every mp4 this
    repo has written.
  * **A-vs-B is inferred, C-vs-D is declared.** Measured at equal pixel budget
    the two quality metrics never agree, so `'auto'` picks between the two
    layouts that genuinely swap by board shape and NOTHING picks between the
    two that are stances.
  * **`'legacy'` is bit-for-bit today's frame**, which is what lets this land
    without changing any existing artifact.
  * **A GIF is actually encoded and read back.** Checking the in-memory list
    cannot see the bug this whole invariant exists for: Pillow writes a valid
    file and resizes silently, so the only place the defect is visible is in
    the FILE. The mis-sized control proves the check can see it.
"""
import os
import sys
import tempfile

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
    for lk in FL.LAYOUTS:
        for rk in FL.RATIOS:
            for sn, bb in SHAPES.items():
                for foot, track in ((0, 0), (37, 0), (0, 61), (37, 61)):
                    n += 1
                    g = FL.plan_frame(bb, layout=lk, ratio=FL.RATIOS[rk],
                                      size=901, panel=True, foot_px=foot,
                                      track_px=track)
                    if g.frame.w % 2 or g.frame.h % 2:
                        odd += 1
                        fail('%s/%s/%s foot=%d track=%d -> %dx%d is odd'
                             % (lk, rk, sn, foot, track, g.frame.w, g.frame.h))
    if not odd:
        print('    %d plans across %d layouts x %d ratios x %d shapes x 4 '
              'chrome cases' % (n, len(FL.LAYOUTS), len(FL.RATIOS),
                                len(SHAPES)))
    if len(_FAIL) == _mark:
        print('  PASS: both dimensions even everywhere')


def test_every_named_box_is_inside_the_frame():
    _mark = len(_FAIL)
    for lk in FL.LAYOUTS:
        for sn, bb in SHAPES.items():
            g = FL.plan_frame(bb, layout=lk, size=800, panel=True,
                              foot_px=30, track_px=40)
            for name in ('board', 'rail', 'foot', 'panel', 'track'):
                b = getattr(g, name)
                if b is None or b.w <= 0 or b.h <= 0:
                    continue
                if not g.frame.contains(b):
                    fail('%s/%s: %s %s escapes the frame %s'
                         % (lk, sn, name, tuple(b), tuple(g.frame)))
            if g.panel is not None and not g.overlays_board:
                if g.board.overlaps(g.panel):
                    fail('%s/%s: panel overlaps the board but this layout does '
                         'not overlay' % (lk, sn))
    if len(_FAIL) == _mark:
        print('  PASS: every box inside the frame; only inset overlays')


def test_a_vs_b_is_inferred_and_c_vs_d_is_never():
    _mark = len(_FAIL)
    # 'very tall' is 0.40, which is OUTSIDE the band where a chrome box is
    # affordable, so `auto` hands it `legacy`. That expectation MOVED with the
    # change that moves it, and the reason is arithmetic rather than taste: at
    # 0.40 the board fills 41% of `stacked`'s 0.98:1 box and 100% of legacy's,
    # because legacy's board box IS the board. Chrome you cannot afford is not
    # a feature -- see `resolve_layout` for the 6.5:1 table this came from.
    want = {'wide 1.85': 'sidebar', '4:3': 'sidebar', 'square': 'stacked',
            'tall 1:1.6': 'stacked', 'very tall': 'legacy'}
    for sn, bb in SHAPES.items():
        g = FL.plan_frame(bb, layout='auto', size=800, panel=True)
        if g.layout != want[sn]:
            fail('auto on %s chose %s, expected %s' % (sn, g.layout, want[sn]))
        elif 'aspect' not in g.chosen_by:
            fail('auto on %s did not say WHY: %r' % (sn, g.chosen_by))
        else:
            print('    %-11s -> %-8s  %s' % (sn, g.layout, g.chosen_by))
    # and nothing infers inset or split
    inferred = {FL.plan_frame(bb, layout='auto', size=800, panel=True).layout
                for bb in SHAPES.values()}
    for stance in ('inset', 'split'):
        if stance in inferred:
            fail('auto chose %r -- C and D are stances about what the viewer '
                 'is there to read, and must never be inferred' % stance)
    # THE EXTREME BAND. Every chrome layout has a FIXED board-box aspect and
    # only `legacy` inherits the board's, so a board far outside the corpus
    # range fills very little of whichever box it is given -- and the adaptive
    # cut, tuned on 0.5..2.5, cheerfully picked the SECOND WORST option for a
    # 6.5:1 board. Measured in a real placement film: the board held 4.6-4.9%
    # of the frame during the beats where parts were moving.
    for a, want_k in ((0.30, 'legacy'), (0.49, 'legacy'), (0.60, 'stacked'),
                      (2.90, 'sidebar'), (3.10, 'legacy'), (6.50, 'legacy')):
        k, why = FL.resolve_layout('auto', (0, 0, 100.0, 100.0 / a))
        if k != want_k:
            fail('auto on aspect %.2f chose %s, expected %s' % (a, k, want_k))
        elif want_k == 'legacy' and 'outside' not in why:
            fail('auto fell back to legacy without saying why: %r' % why)
    # and the band must actually BITE -- a band nothing falls outside of is
    # not a band, it is a comment.
    outside = [a for a in (0.30, 0.49, 3.10, 6.50)
               if FL.resolve_layout('auto', (0, 0, 100.0, 100.0 / a))[0]
               == 'legacy']
    if len(outside) != 4:
        fail('only %d of 4 extreme aspects fell back' % len(outside))
    else:
        print('    extreme band %.2f..%.2f -> legacy, with the reason stated'
              % (FL.EXTREME_ASPECT_LO, FL.EXTREME_ASPECT_HI))
    if len(_FAIL) == _mark:
        print('  PASS: A/B inferred with a stated reason; C/D never')


def test_legacy_reproduces_todays_frame():
    _mark = len(_FAIL)
    pcb = parse_kicad_pcb(BOARD)
    r = BoardRenderer(pcb, size=500, supersample=1)
    native = r.frame(segments=[], vias=[]).size
    g = FL.plan_frame(pcb.board_info.board_bounds, layout='legacy', size=500,
                      panel=False, legacy_size=(r.W, r.H))
    # Up to the EVEN forcing, which is the point: the renderer's native size
    # can be odd (routed_output at size 500 is 500x309) and `_write_mp4` crops
    # `& ~1` on BOTH axes, so that row was always being thrown away. Legacy is
    # today's frame with the silent crop made explicit.
    want = (FL.even(native[0]), FL.even(native[1]))
    if (g.board.w, g.board.h) != want:
        fail("legacy's board box %s is not the renderer's own %s evened to %s"
             % ((g.board.w, g.board.h), native, want))
    else:
        print('    renderer %s -> legacy %s  (even forcing; _write_mp4 was '
              'cropping that pixel away silently)' % (native, want))
    # an explicit ratio must WIN over the legacy shortcut
    g2 = FL.plan_frame(pcb.board_info.board_bounds, layout='legacy', size=500,
                       ratio=16 / 9.0, legacy_size=(r.W, r.H))
    if abs(g2.aspect - 16 / 9.0) > 0.02:
        fail('an explicit ratio did not win over legacy_size: %.3f'
             % g2.aspect)
    if len(_FAIL) == _mark:
        print('  PASS: legacy is today\'s frame; an explicit ratio overrides')


def test_a_real_gif_encodes_at_one_size_per_layout():
    """The in-memory list cannot see the defect. The FILE can."""
    _mark = len(_FAIL)
    d = tempfile.mkdtemp()
    for lk in ('legacy', 'auto', 'sidebar', 'inset', 'split'):
        out = os.path.join(d, '%s.gif' % lk)
        import make_movie
        got = make_movie.make_movie([BOARD], out=out, size=220, quiet=True,
                                    layout=lk)
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


def test_set_canvas_leaves_the_margin_rule_alone():
    _mark = len(_FAIL)
    pcb = parse_kicad_pcb(BOARD)
    r = BoardRenderer(pcb, size=600, supersample=2)
    before = r._margin_px
    r.set_canvas(321, 222)
    if r.frame(segments=[], vias=[]).size != (321, 222):
        fail('set_canvas did not take: %s' % (r.frame(segments=[], vias=[]).size,))
    if r._margin_px != before:
        fail('set_canvas moved _margin_px %r -> %r; '
             'tests/test_431_render_seams.py:42-53 pins that arithmetic to 1e-12'
             % (before, r._margin_px))
    if len(_FAIL) == _mark:
        print('  PASS: canvas moves, margin rule does not')


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
    off the board, and with `iso=True` the split boxes sit inside the panel.
    """
    _mark = len(_FAIL)
    n = 0
    for lk in FL.LAYOUTS:
        for rk, ratio in FL.RATIOS.items():
            if not ratio:
                continue
            for sn, bb in SHAPES.items():
                for iso in (False, True):
                    n += 1
                    g = FL.plan_frame(bb, layout=lk, ratio=ratio, size=900,
                                      panel=True, track_px=120, foot_px=0,
                                      iso=iso)
                    want = _declared(900, ratio)
                    if (g.frame.w, g.frame.h) != want:
                        fail('%s/%s/%s iso=%s: frame %dx%d, declared %dx%d'
                             % (lk, rk, sn, iso, g.frame.w, g.frame.h,
                                want[0], want[1]))
                        continue
                    t = g.track
                    if t is None or not g.frame.contains(t):
                        fail('%s/%s/%s: band %r is not inside the frame'
                             % (lk, rk, sn, t))
                    elif t.overlaps(g.board):
                        fail('%s/%s/%s: band %r overlaps the board %r'
                             % (lk, rk, sn, tuple(t), tuple(g.board)))
                    if iso and g.panel_split:
                        for b in g.panel_split:
                            if not g.panel.contains(b):
                                fail('%s/%s/%s: split box %r outside the '
                                     'panel' % (lk, rk, sn, tuple(b)))
                    if iso and g.layout in ('stacked', 'sidebar', 'split') \
                            and not g.panel_split:
                        fail('%s/%s/%s: iso asked, but no split box for the '
                             '3D view' % (lk, rk, sn))
    if len(_FAIL) == _mark:
        print('  PASS: %d declared plans hold their size, band inside, off '
              'the board' % n)


def test_the_encoded_film_is_the_declared_size_in_both_themes():
    """The same claim read back from real FILES: layout x ratio x theme, each
    with an attempts band, encoded and measured. The render path is where
    the band used to grow the frame (`movie_attempts.attach` after the
    fact), so a plan that is right and a film that is not is the failure
    this exists to see."""
    _mark = len(_FAIL)
    import make_movie
    import movie_attempts as MA
    rows = tuple(MA.Attempt(i, 'lap %d' % i, 'completion',
                            i - 1 if i else None, i % 3 != 1, False,
                            float(20 - i), False, None) for i in range(8))
    track = MA.Track(rows, 'blocking (lower better)', 'converge', 'fixture')
    d = tempfile.mkdtemp()
    n = 0
    for lk in ('stacked', 'sidebar', 'inset', 'split', 'legacy'):
        for rk in ('16:9', '9:16', '1:1'):
            for th in ('dark', 'light'):
                n += 1
                out = os.path.join(d, '%s_%s_%s.gif'
                                   % (lk, rk.replace(':', 'x'), th))
                import contextlib
                import io
                err = io.StringIO()
                # 400 px, so even the 16:9 frame (400x224) can carry the
                # 64 px band under its 34% ceiling: a film where the band
                # DECLINED would pass the size check without testing it.
                with contextlib.redirect_stderr(err):
                    got = make_movie.make_movie(
                        [BOARD], out=out, size=400, quiet=True, layout=lk,
                        aspect=rk, theme=th, attempts=track)
                if not got:
                    fail('%s/%s/%s: no film' % (lk, rk, th))
                    continue
                if 'layout-reserved band' not in err.getvalue():
                    fail('%s/%s/%s: the band was not drawn into a reserved '
                         'box, so this film does not test the claim: %s'
                         % (lk, rk, th, err.getvalue()[-200:]))
                with Image.open(got) as im:
                    sz = im.size
                    corner = im.convert('RGB').getpixel((sz[0] - 1, 0))
                want = _declared(400, FL.parse_ratio(rk))
                if sz != want:
                    fail('%s/%s/%s: encoded %s, declared %s'
                         % (lk, rk, th, sz, want))
                # the theme reached the frame: its top-right corner is the
                # theme's ground or chrome, never the OTHER theme's ground
                other = ('light' if th == 'dark' else 'dark')
                if corner == RT_theme(other).rgb('ground'):
                    fail('%s/%s/%s: the frame corner is the %s ground'
                         % (lk, rk, th, other))
    if len(_FAIL) == _mark:
        print('  PASS: %d films (5 layouts x 3 ratios x 2 themes) encoded at '
              'the declared size, band inside' % n)


def RT_theme(name):
    import render_theme
    return render_theme.theme(name)


TESTS = (
    test_a_declared_ratio_is_the_size_asked_for_with_the_band_inside,
    test_the_encoded_film_is_the_declared_size_in_both_themes,
    test_every_plan_is_even_on_both_axes,
    test_every_named_box_is_inside_the_frame,
    test_a_vs_b_is_inferred_and_c_vs_d_is_never,
    test_legacy_reproduces_todays_frame,
    test_a_real_gif_encodes_at_one_size_per_layout,
    test_the_guard_reports_and_pads_rather_than_squashing,
    test_set_canvas_leaves_the_margin_rule_alone,
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
