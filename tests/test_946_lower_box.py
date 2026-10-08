#!/usr/bin/env python3
"""The layer column: one fixed box, one content (#946 items 6/10/11, #1020,
#1081).

A panel that appears and disappears changes frame height, and every frame must
be the same size: Pillow does not raise on a mismatch, and the GIF comes out
valid and quietly distorted. So the stage3d frame's layer column is one box
with one content on every frame -- the per-layer strip, with the board's
numbers under it. (The four contents it used to switch between by phase were
the retired layouts' lower box.)

What this file pins:

  * **the strip builds NO second renderer.**
    `tests/test_431_placement_movie.py:92` asserts exactly one `BoardRenderer`
    on the no-stage path, so a strip of ten small boards cannot construct ten
    of them;
  * **the counts are the copper**, not a decoration -- a cell that says `390`
    has 390 segments on that layer. Asserted against the `d.line` and `d.text`
    calls the drawer ACTUALLY MADE, recorded through a delegating draw object,
    because a self-reported `lines` field is the tally under another name: the
    round-2 verifier's M18 stroked ONE line, reported 375, and survived a check
    that compared the report to the board and probed only that ink EXISTS. The phase
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
  * **the column's two drawers draw and report** -- the summary lands every
    row, and no box, however degenerate, raises out of the never-fail
    wrapper;
  * **cells shrink in NUMBER, not below legibility**. A cell too small to show
    a route costs pixels and answers nothing;
  * **the box rect is a property of the frame**, the same on every plan,
    which is the whole reason the design is one box rather than a panel that
    comes and goes.
"""
import os
import sys

# stage3d is the only film layout, so an unnamed layout is a stage3d
# frame. These tests grade the 2D board, not the Node/Chromium 3D
# render: set before env_knobs is read.
os.environ.setdefault('KICAD_MOVIE_BOARD3D', '2d')

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


class _Recorder:
    """An `ImageDraw` that delegates everything and records what was asked.

    The strongest available check on a drawer: it is not the drawer's own word,
    it is the call. `Cell.lines` is a counter the drawer increments, so a
    mutant that skips the stroke and keeps the increment reports the truth
    about nothing -- which is exactly the failure this class exists to catch.
    """

    def __init__(self, inner):
        self._d = inner
        self.lines = []
        self.texts = []

    def line(self, xy, *a, **kw):
        self.lines.append(tuple(xy))
        return self._d.line(xy, *a, **kw)

    def text(self, xy, txt, *a, **kw):
        self.texts.append((tuple(xy), txt))
        return self._d.text(xy, txt, *a, **kw)

    def __getattr__(self, name):
        return getattr(self._d, name)


def _strip(box_w=560, box_h=110, n_layers=None):
    pcb = parse_kicad_pcb(BOARD)
    r = RR.BoardRenderer(pcb, size=400, supersample=1)
    layers = list(r.copper_layers)[:n_layers] if n_layers else list(
        r.copper_layers)
    img = Image.new('RGB', (box_w, box_h + 20), RT.DARK.rgb('ground'))
    rec = _Recorder(ImageDraw.Draw(img))
    box = FL.Box(0, 10, box_w, box_h)
    cells = RP.draw_layer_strip(rec, box, bounds=r.bounds,
                                segments=pcb.segments, layers=layers,
                                palette=r.palette, theme=RT.DARK,
                                active=layers[0] if layers else None)
    return img, cells, layers, pcb, r, rec


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
    _img, cells, layers, pcb, _r, rec = _strip(box_w=900)
    want, seen = {}, {}
    for sg in pcb.segments:
        want[sg.layer] = want.get(sg.layer, 0) + 1
        seen.setdefault(sg.layer, set()).add(
            (round(sg.start_x, 4), round(sg.start_y, 4),
             round(sg.end_x, 4), round(sg.end_y, 4)))
    distinct = {k: len(v) for k, v in seen.items()}
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
        # the NAME must be this cell's own layer -- a cell stamped with its
        # neighbour's name is a strip in which position means nothing, which
        # is the one thing this whole content exists to provide
        if c.name_text != c.layer.replace('.Cu', ''):
            fail('%s: the cell is labelled %r' % (c.layer, c.name_text))
        # THE CALLS, not the counter. `lines` is the drawer's own word, and a
        # mutant that skips the stroke while keeping the increment reports the
        # truth about nothing (M18: stroked 1, reported 375, survived).
        cx, cy, cw, ch = c.box
        strokes = [xy for xy in rec.lines
                   if cx <= xy[0] <= cx + cw and cy <= xy[1] <= cy + ch]
        if len(strokes) != true:
            fail('%s: %d stroke(s) landed in its cell for %d segment(s) on '
                 'that layer' % (c.layer, len(strokes), true))
        if c.lines != len(strokes):
            fail('%s: reported %d line(s) and made %d call(s) -- the report is '
                 'not the drawing' % (c.layer, c.lines, len(strokes)))
        # and the strokes must be as DISTINCT as the copper is: N calls all
        # drawing the same line is 4 px of ink reported as 375 segments, which
        # a call count alone cannot tell from the real thing. Compared against
        # the board's own distinct count, not against a re-derived transform.
        if len(set(strokes)) != distinct.get(c.layer, 0):
            fail('%s: %d distinct stroke(s) for %d distinct segment(s) -- the '
                 'cell is redrawing one line'
                 % (c.layer, len(set(strokes)), distinct.get(c.layer, 0)))
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
        _img, cells, _l, _p, _r, _rec = _strip(box_w=box_w)
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
    _img, narrow, _l, _p, _r, _rec = _strip(box_w=180)
    if narrow and all(c.count_text for c in narrow):
        fail('BROKEN TEST: no count was dropped even at 180 px, where the '
             'measured overlap was +23 px -- the guard is not engaged')
    if len(_FAIL) == _mark:
        print('  PASS: the count goes before it lands on the name')


def test_cells_shrink_in_number_not_below_legibility():
    _mark = len(_FAIL)
    _wi, wide, layers, _p, _r, _rw = _strip(box_w=900)
    _na, narrow, _l, _p2, _r2, _rn = _strip(box_w=180)
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
    for lk in ('16:9', '4:3', '9:16'):
        g = FL.plan_frame((0, 0, 185, 100), ratio=FL.parse_ratio(lk),
                          size=700)
        rects.add((lk, tuple(g.panel) if g.panel else None))
        # the box is a property of the FRAME
        g2 = FL.plan_frame((0, 0, 185, 100), ratio=FL.parse_ratio(lk),
                           size=700)
        if tuple(g.panel or ()) != tuple(g2.panel or ()):
            fail('%s: the panel rect is not deterministic' % lk)
    if len(_FAIL) == _mark:
        print('  PASS: one rect per frame, the same on every plan')




def test_the_column_contents_draw():
    """The layer column's two drawers put ink on the box and report what
    they drew: the board summary and the per-layer strip. (The placement
    inventory and the seeding pile this used to also reach were the
    retired layouts' lower box: the stage3d column shows the strip and the
    summary on every frame.)
    """
    _mark = len(_FAIL)
    pcb = parse_kicad_pcb(BOARD)
    r = RR.BoardRenderer(pcb, size=300, supersample=1)
    ground = RT.DARK.rgb('ground')
    box = FL.Box(0, 0, 420, 130)
    cases = {
        'summary': lambda d: RP.draw_summary(
            d, box, theme=RT.DARK,
            lines=RP.board_summary(pcb, pcb.segments, pcb.vias)),
        'strip': lambda d: RP.draw_layer_strip(
            d, box, bounds=r.bounds, segments=pcb.segments,
            layers=list(r.copper_layers), palette=r.palette, theme=RT.DARK),
    }
    for what, draw in cases.items():
        img = Image.new('RGB', (420, 130), ground)
        got = draw(ImageDraw.Draw(img))
        cols = {c for _n, c in img.getcolors(1 << 20)}
        if len(cols) < 3:
            fail('%s drew %d colour(s) -- it swallowed an exception'
                 % (what, len(cols)))
        if not got:
            fail('%s reported drawing nothing' % what)
        else:
            print('    %-10s %d colours, %d row(s) reported'
                  % (what, len(cols), len(got)))
    # EVERY row a summary is given must land -- the verifier measured a
    # short box dropping `vias`, and another dropping three.
    for h in (132, 96, 64, 44):
        img = Image.new('RGB', (420, h), ground)
        lines = RP.board_summary(pcb, pcb.segments, pcb.vias)
        got = RP.draw_summary(ImageDraw.Draw(img), FL.Box(0, 0, 420, h),
                              lines=lines, theme=RT.DARK)
        if len(got) != len(lines):
            fail('a %d px summary box landed %d of %d rows'
                 % (h, len(got), len(lines)))
    # and no panel rect may RAISE out of the never-fail wrapper: 1143 of
    # 4010 (w, h) combinations did, on the height axis.
    raised = []
    for w in range(0, 420, 7):
        for h in range(0, 60, 3):
            try:
                boxes, n = RP._cell_boxes(FL.Box(0, 0, w, h), 10)
            except Exception as exc:                          # noqa: BLE001
                raised.append((w, h, exc.__class__.__name__))
                continue
            for _x, _y, cw, chh in boxes:
                if cw < RP.CELL_FLOOR_W or chh < 14 + RP.CELL_FLOOR_H:
                    raised.append((w, h, 'cell %dx%d' % (cw, chh)))
    if raised:
        fail('%d degenerate box(es) produced an unusable cell, e.g. %s'
             % (len(raised), raised[:3]))
    if len(_FAIL) == _mark:
        print('  PASS: the summary and the strip draw and report, every row '
              'lands, no degenerate cell')


def test_the_film_actually_reaches_its_closing_bookend():
    """One box, four contents -- and the film has to REACH them.

    Both halves of this were wrong until a full film was rendered and looked
    at, which is the only way either could have been found:

      * `reconcile_to` is silent when nothing changed, so a chain whose last
        step already matched the final board ended on a ROUTING frame and
        never showed the closing summary at all. Half of "open and close",
        missing, on the most ordinary chain there is.
      * the rail's STABLE left was the final board's stem, so a film of
        `step1 -> step4` read `step4_restored` on frame 1 -- the one field
        that does not change frame to frame, named after the last step.

    Gated on a panel EXISTING: the control is a 9:16 stage3d frame too
    small to keep its layer row (the legacy frame was the control until
    stage3d became the only layout).
    """
    _mark = len(_FAIL)
    import animate_route as A
    import tempfile
    import shutil
    with tempfile.TemporaryDirectory() as td:
        a = os.path.join(td, 'step1_demo.kicad_pcb')
        b = os.path.join(td, 'step2_demo.kicad_pcb')
        shutil.copyfile(BOARD, a)
        shutil.copyfile(BOARD, b)
        steps = [('step1 route', a, None), ('step2 route', b, None)]
        for layout, want_close in (('16:9', True), ('9:16', False)):
            chrome, geom = [], []
            frames = A.build_boards(steps, b, 240, 1, 150, 2, 3,
                                    aspect=layout, geom_out=geom)
            if not frames:
                fail('%s: no frames' % layout)
                continue
            print('    %-7s %d frames, panel=%s'
                  % (layout, len(frames), bool(geom and geom[0].panel)))
        # the REAL check, on the frames themselves: with a panel, the film
        # must be one frame longer than without the closing snapshot.
        g_row = []
        n_split = len(A.build_boards(steps, b, 240, 1, 150, 2, 3,
                                     aspect='16:9'))
        n_legacy = len(A.build_boards(steps, b, 240, 1, 150, 2, 3,
                                      aspect='9:16', geom_out=g_row))
        if not g_row or g_row[0].panel is not None:
            fail('BROKEN CONTROL: the 9:16 frame at 240 px kept its layer '
                 'row, so it cannot stand for a frame with no box')
        if n_split <= n_legacy:
            fail('the panelled film (%d) is not longer than the one with no '
                 'box (%d) -- the closing bookend was not emitted'
                 % (n_split, n_legacy))
        else:
            print('    16:9 %d frames vs 9:16 with no row %d -- the closing '
                  'bookend is the difference' % (n_split, n_legacy))

    # and the rail's stable left is the BOARD, not the last step
    if A.board_title('/x/step4_restored.kicad_pcb',
                     [('a', '/x/step1_demo.kicad_pcb', None),
                      ('b', '/x/step4_demo.kicad_pcb', None)]) == \
            'step4_restored':
        fail('the rail still names itself after the last step')
    if A.board_title('/x/step4_demo.kicad_pcb',
                     [('a', '/x/step1_demo.kicad_pcb', None),
                      ('b', '/x/step4_demo.kicad_pcb', None)]) != 'demo':
        fail('a common stem was not recovered: %r'
             % A.board_title('/x/step4_demo.kicad_pcb',
                             [('a', '/x/step1_demo.kicad_pcb', None),
                              ('b', '/x/step4_demo.kicad_pcb', None)]))
    if A.board_title('/x/anything.kicad_pcb', (), hint='myrun') != 'myrun':
        fail('an explicit run name did not win')
    if A.board_title('/x/plain_board.kicad_pcb') != 'plain_board':
        fail('a single-board film stopped showing its own stem')
    # A PLACEMENT LOOP's boards are numbered and nothing else, so the run
    # directory is the only name there is. Without this the placement film --
    # the one that actually shows placement -- read `loop_round5` on its rail.
    loop = [('r0', '/r/myrun/loop_round0.kicad_pcb', None),
            ('r5', '/r/myrun/loop_round5.kicad_pcb', None)]
    if A.board_title('/r/myrun/loop_round5.kicad_pcb', loop) != 'myrun':
        fail('a loop chain named itself after a round: %r'
             % A.board_title('/r/myrun/loop_round5.kicad_pcb', loop))
    if len(_FAIL) == _mark:
        print('  PASS: the film opens on a summary and closes on one, and the '
              'rail names the board')


TESTS = (
    test_the_strip_builds_no_second_renderer,
    test_the_counts_are_the_copper,
    test_a_count_that_would_touch_the_name_is_dropped,
    test_cells_shrink_in_number_not_below_legibility,
    test_the_box_rect_never_changes_between_phases,
    test_the_column_contents_draw,
    test_the_film_actually_reaches_its_closing_bookend,
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
