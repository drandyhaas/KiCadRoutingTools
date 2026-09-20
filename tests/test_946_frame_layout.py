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
    want = {'wide 1.85': 'sidebar', '4:3': 'sidebar', 'square': 'stacked',
            'tall 1:1.6': 'stacked', 'very tall': 'stacked'}
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
    good = [Image.new('RGB', (64, 40), (10, 10, 10)) for _ in range(2)]
    odd = Image.new('RGB', (64, 52), (200, 10, 10))
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


TESTS = (
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
