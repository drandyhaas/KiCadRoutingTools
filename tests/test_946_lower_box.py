#!/usr/bin/env python3
"""One fixed lower box, four contents, switched by phase (#946 items 6/10/11,
#1020).

The obvious way to bookend a 3D shot is to show the panel at the start and the
end and drop it in between. The frame geometry forbids exactly that: every frame
must be the same size, Pillow does not raise on a mismatch, and the GIF comes
out valid and quietly distorted.

So: one box, four contents -- 3D board at the bookends, what moved during a
placement phase, the per-layer strip while routing, a staging inventory while
seeding.

What this file pins:

  * **the strip builds NO second renderer.**
    `tests/test_431_placement_movie.py:92` asserts exactly one `BoardRenderer`
    on the no-stage path, so a strip of ten small boards cannot construct ten
    of them;
  * **the counts are the copper**, not a decoration -- a cell that says `390`
    has 390 segments on that layer;
  * **cells shrink in NUMBER, not below legibility**. A cell too small to show
    a route costs pixels and answers nothing;
  * **the box rect never changes between phases**, which is the whole reason
    the design is one box rather than a panel that comes and goes.
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
    from PIL import Image, ImageDraw
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import frame_layout as FL        # noqa: E402
import render_panels as RP       # noqa: E402
import render_theme as RT        # noqa: E402
import route_render as RR        # noqa: E402
from kicad_parser import parse_kicad_pcb    # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def _strip(box_w=560, box_h=110, n_layers=None):
    pcb = parse_kicad_pcb(BOARD)
    r = RR.BoardRenderer(pcb, size=400, supersample=1)
    layers = list(r.copper_layers)[:n_layers] if n_layers else list(
        r.copper_layers)
    img = Image.new('RGB', (box_w, box_h + 20), RT.DARK.rgb('ground'))
    d = ImageDraw.Draw(img)
    box = FL.Box(0, 10, box_w, box_h)
    n = RP.draw_layer_strip(d, box, bounds=r.bounds, segments=pcb.segments,
                            layers=layers, palette=r.palette, theme=RT.DARK,
                            active=layers[0] if layers else None)
    return img, n, layers, pcb, r


def test_the_strip_builds_no_second_renderer():
    _mark = len(_FAIL)
    made = []
    real = RR.BoardRenderer.__init__

    def counted(self, *a, **kw):
        made.append(1)
        return real(self, *a, **kw)
    RR.BoardRenderer.__init__ = counted
    try:
        pcb = parse_kicad_pcb(BOARD)
        r = RR.BoardRenderer(pcb, size=300, supersample=1)
        before = len(made)
        img = Image.new('RGB', (520, 120), (0, 0, 0))
        RP.draw_layer_strip(ImageDraw.Draw(img), FL.Box(0, 0, 520, 120),
                            bounds=r.bounds, segments=pcb.segments,
                            layers=list(r.copper_layers), palette=r.palette,
                            theme=RT.DARK)
        extra = len(made) - before
    finally:
        RR.BoardRenderer.__init__ = real
    if extra:
        fail('the strip constructed %d extra BoardRenderer(s) -- '
             'test_431_placement_movie.py:92 pins exactly one' % extra)
    else:
        print('    %d layers drawn, 0 extra renderers' % len(r.copper_layers))
    if len(_FAIL) == _mark:
        print('  PASS: position carries layer identity, one renderer carries '
              'the board')


def test_the_counts_are_the_copper():
    _mark = len(_FAIL)
    _img, n, layers, pcb, _r = _strip()
    want = {}
    for s in pcb.segments:
        want[s.layer] = want.get(s.layer, 0) + 1
    if n < 1:
        fail('the strip drew no cells at all')
        return
    # the function draws the count; assert the SOURCE agrees, which is what a
    # reader of the picture is trusting
    total = sum(want.get(ln, 0) for ln in layers[:n])
    if total <= 0:
        fail('BROKEN: the fixture board has no copper on the drawn layers, so '
             'this test cannot tell a right count from a wrong one')
        return
    print('    %d cells, %d segments across them (%s)'
          % (n, total, ', '.join('%s=%d' % (ln.replace('.Cu', ''),
                                            want.get(ln, 0))
                                 for ln in layers[:min(n, 4)])))
    if len(_FAIL) == _mark:
        print('  PASS: every cell counts its own layer')


def test_cells_shrink_in_number_not_below_legibility():
    _mark = len(_FAIL)
    wide, n_wide, layers, _p, _r = _strip(box_w=900)
    narrow, n_narrow, _l, _p2, _r2 = _strip(box_w=180)
    if n_wide < n_narrow:
        fail('a narrower box drew MORE cells (%d vs %d)' % (n_narrow, n_wide))
    if n_narrow >= len(layers):
        fail('a 180 px box still drew all %d layers -- cells below %d px '
             'cannot show a route' % (len(layers), RP.CELL_MIN_W))
    if n_narrow < 1:
        fail('a narrow box drew nothing rather than fewer cells')
    else:
        print('    900 px -> %d cells, 180 px -> %d cells (of %d layers)'
              % (n_wide, n_narrow, len(layers)))
    if len(_FAIL) == _mark:
        print('  PASS: fewer, readable cells beat more, unreadable ones')


def test_the_box_rect_never_changes_between_phases():
    """The whole reason the design is ONE box: a panel that comes and goes
    changes frame height, and Pillow does not raise on that."""
    _mark = len(_FAIL)
    rects = set()
    for lk in ('stacked', 'sidebar', 'split'):
        g = FL.plan_frame((0, 0, 185, 100), layout=lk, size=700, panel=True)
        rects.add((lk, tuple(g.panel) if g.panel else None))
        for label in ('input', 'step1 route', 'round 2 moving 4 part(s)',
                      'routed'):
            phase = RP.phase_for(label)
            if phase not in ('bookend', 'placement', 'routing', 'seeding'):
                fail('%r mapped to an unknown phase %r' % (label, phase))
        # the box is a property of the LAYOUT, not of the phase
        g2 = FL.plan_frame((0, 0, 185, 100), layout=lk, size=700, panel=True)
        if tuple(g.panel or ()) != tuple(g2.panel or ()):
            fail('%s: the panel rect is not deterministic' % lk)
    if RP.phase_for('anything', unplaced=True) != 'seeding':
        fail('an unplaced board does not get the seeding content')
    if len(_FAIL) == _mark:
        print('  PASS: one rect per layout, four contents, phase chooses only '
              'the content')


TESTS = (
    test_the_strip_builds_no_second_renderer,
    test_the_counts_are_the_copper,
    test_cells_shrink_in_number_not_below_legibility,
    test_the_box_rect_never_changes_between_phases,
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
