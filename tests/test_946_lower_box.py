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
    has 390 segments on that layer. Asserted against what `draw_layer_strip`
    REPORTS HAVING DRAWN, not against a tally re-derived here. The phase
    verifier measured why: the first version of this check built `want` from
    `pcb.segments` and never read the drawing, so a mutant stamping `cnt + 7`
    on every cell and a mutant counting EVERY segment on the board in EVERY
    cell both survived while it printed "PASS: every cell counts its own
    layer";
  * **a count that would touch the layer name is dropped, not overprinted**.
    `CELL_MIN_W` bounds the cell width; nothing bounded the text, so at the
    widths this feature actually produces the count was stamped on top of the
    name (+23 px of overlap in the 180 px case below, +25 px at `CELL_MIN_W`
    exactly);
  * **all four contents are reachable and draw**, because three of them were
    a literal string and one of them -- 'seeding' -- could not occur in a real
    film at all: nothing in production passed `unplaced`;
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
    cells = RP.draw_layer_strip(d, box, bounds=r.bounds,
                                segments=pcb.segments, layers=layers,
                                palette=r.palette, theme=RT.DARK,
                                active=layers[0] if layers else None)
    return img, cells, layers, pcb, r


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
    """Asserted on what was DRAWN. The previous version re-derived the counts
    here and then compared them to nothing at all."""
    _mark = len(_FAIL)
    _img, cells, layers, pcb, _r = _strip(box_w=900)
    want = {}
    for sg in pcb.segments:
        want[sg.layer] = want.get(sg.layer, 0) + 1
    if not cells:
        fail('the strip drew no cells at all')
        return
    if sum(want.get(c.layer, 0) for c in cells) <= 0:
        fail('BROKEN TEST: the fixture board has no copper on the drawn '
             'layers, so this cannot tell a right count from a wrong one')
        return
    for c in cells:
        true = want.get(c.layer, 0)
        # the STRING that was stamped, not the tally behind it
        if c.count_text and c.count_text != str(true):
            fail('%s: the cell drew %r, the board has %d segment(s) on that '
                 'layer' % (c.layer, c.count_text, true))
        if c.lines != true:
            fail('%s: %d line(s) were stroked for %d segment(s) -- a cell is '
                 'counting or drawing copper that is not its own'
                 % (c.layer, c.lines, true))
    stamped = [c for c in cells if c.count_text]
    if len(stamped) != len(cells):
        fail('a 900 px box dropped %d count(s); every cell there has room'
             % (len(cells) - len(stamped)))
    # INDEPENDENT of the report: `lines` is still the drawer's own word, so
    # probe the pixels. A cell with copper must carry ink in its own palette
    # colour, INSIDE its own rect -- which also catches a cell that draws its
    # neighbour's copper or draws outside the box.
    px = _img.convert('RGB')
    for c in cells:
        cx, cy, cw, ch = c.box
        col = _r.palette.get(c.layer)
        if col is None:
            continue
        ink = 0
        for yy in range(max(0, cy), min(px.height, cy + ch)):
            for xx in range(max(0, cx), min(px.width, cx + cw)):
                if px.getpixel((xx, yy)) == col:
                    ink += 1
        if c.lines and not ink:
            fail('%s reported %d line(s) and left NO ink in its own cell'
                 % (c.layer, c.lines))
        if not c.lines and ink:
            fail('%s reported no lines and yet has %d px of its own colour'
                 % (c.layer, ink))
    if len(_FAIL) == _mark:
        print('    %d cells, %d segments across them (%s)'
              % (len(cells), sum(c.lines for c in cells),
                 ', '.join('%s=%s' % (c.name_text, c.count_text)
                           for c in cells[:4])))
        print('  PASS: every cell counts, and draws, its own layer')


def test_a_count_that_would_touch_the_name_is_dropped():
    """The name is the identity; the count is the extra. Overprinting destroys
    both, and `CELL_MIN_W` cannot prevent it -- it bounds the cell, not the
    text."""
    _mark = len(_FAIL)
    from route_render import load_font
    probe = ImageDraw.Draw(Image.new('RGB', (8, 8)))
    font = load_font(max(8, min(13, int(130 * 0.16))))
    for box_w in (180, 370, 240, 900):
        _img, cells, _l, _p, _r = _strip(box_w=box_w)
        if not cells:
            fail('%d px box drew nothing' % box_w)
            continue
        for c in cells:
            if not c.count_text:
                continue
            cw = c.box[2]
            need = (probe.textlength(c.name_text, font=font)
                    + probe.textlength(c.count_text, font=font)
                    + RP.LABEL_GAP_PX + 8)
            if need > cw:
                fail('%d px box, %s: name+count need %.0f px in a %d px cell'
                     % (box_w, c.layer, need, cw))
        print('    %3d px box -> %d cell(s), %d count(s) kept'
              % (box_w, len(cells), sum(1 for c in cells if c.count_text)))
    # and the drop must be REAL at the narrow end, or this is asserting that a
    # condition which never fires never fires
    _img, narrow, _l, _p, _r = _strip(box_w=180)
    if narrow and all(c.count_text for c in narrow):
        fail('BROKEN TEST: no count was dropped even at 180 px, where the '
             'measured overlap was +23 px -- the guard is not engaged')
    if len(_FAIL) == _mark:
        print('  PASS: the count goes before it lands on the name')


def test_cells_shrink_in_number_not_below_legibility():
    _mark = len(_FAIL)
    _wi, wide, layers, _p, _r = _strip(box_w=900)
    _na, narrow, _l, _p2, _r2 = _strip(box_w=180)
    n_wide, n_narrow = len(wide), len(narrow)
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
    # a box too small for even ONE legible cell draws NOTHING, rather than a
    # negative-width rectangle the never-fail wrapper would swallow
    for w in (0, 10, 24, 38):
        boxes, n = RP._cell_boxes(FL.Box(0, 0, w, 40), 10)
        if not isinstance(boxes, list) or not isinstance(n, int):
            fail('_cell_boxes(w=%d) returned %r -- it must ALWAYS be '
                 '(list, int); a bare [] raises ValueError in its own caller'
                 % (w, (boxes, n)))
            continue
        for _x, _y, cw, _ch in boxes:
            if cw < RP.CELL_FLOOR_W:
                fail('_cell_boxes(w=%d) produced a %d px cell' % (w, cw))
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




def test_all_four_contents_are_reachable_and_draw():
    """One content and three captions is not four contents.

    `draw_inventory` had NO caller anywhere in the repo, and `unplaced` was
    never passed from production -- so 'seeding' could not occur in a film at
    all, and 'bookend' and 'placement' drew a literal string. Each branch is
    asserted here to reach a drawer and put ink on the box.
    """
    _mark = len(_FAIL)
    pcb = parse_kicad_pcb(BOARD)
    r = RR.BoardRenderer(pcb, size=300, supersample=1)
    ground = RT.DARK.rgb('ground')
    box = FL.Box(0, 0, 420, 130)
    inv = RP.inventory_counts(pcb)
    if not inv:
        fail('BROKEN FIXTURE: the board yielded no part classes')
        return
    cases = {
        'bookend': lambda d: RP.draw_summary(
            d, box, theme=RT.DARK,
            lines=RP.board_summary(pcb, pcb.segments, pcb.vias)),
        'placement': lambda d: RP.draw_inventory(
            d, box, counts=inv, placed=len(pcb.footprints) - 2,
            total=len(pcb.footprints), theme=RT.DARK),
        'seeding': lambda d: RP.draw_inventory(
            d, box, counts=inv, placed=0, total=len(pcb.footprints),
            theme=RT.DARK),
        'routing': lambda d: RP.draw_layer_strip(
            d, box, bounds=r.bounds, segments=pcb.segments,
            layers=list(r.copper_layers), palette=r.palette, theme=RT.DARK),
    }
    for phase, draw in cases.items():
        img = Image.new('RGB', (420, 130), ground)
        got = draw(ImageDraw.Draw(img))
        cols = {c for _n, c in img.getcolors(1 << 20)}
        if len(cols) < 3:
            fail('%s drew %d colour(s) -- it swallowed an exception'
                 % (phase, len(cols)))
        if not got:
            fail('%s reported drawing nothing' % phase)
        else:
            print('    %-10s %d colours, %d row(s) reported'
                  % (phase, len(cols), len(got)))
    # and the SEEDING branch must be reachable from a label, which is the half
    # that was missing: nothing in production passed `unplaced`.
    if RP.phase_for('round 2 moving 4 part(s)', unplaced=True) != 'seeding':
        fail('unplaced does not win over the label')
    src = open(os.path.join(ROOT, 'py_router', 'animate_route.py'),
               encoding='utf-8').read()
    if 'unplaced=bool(c.get(' not in src.replace('\n', '').replace(' ', ''):
        if 'unplaced' not in src:
            fail('nothing in animate_route passes unplaced, so the seeding '
                 'content cannot occur in a film')
    if 'inventory_counts' not in src:
        fail('nothing in animate_route builds the inventory, so the box has '
             'no data to draw it from')
    if len(_FAIL) == _mark:
        print('  PASS: four contents, four drawers, all reachable')


TESTS = (
    test_the_strip_builds_no_second_renderer,
    test_the_counts_are_the_copper,
    test_a_count_that_would_touch_the_name_is_dropped,
    test_cells_shrink_in_number_not_below_legibility,
    test_the_box_rect_never_changes_between_phases,
    test_all_four_contents_are_reachable_and_draw,
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
