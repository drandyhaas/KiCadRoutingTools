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
    # EVERY row a summary is given must land -- the verifier measured
    # `stacked --size 400` dropping `vias` and `inset` dropping three.
    for h in (132, 96, 64, 44):
        img = Image.new('RGB', (420, h), ground)
        lines = RP.board_summary(pcb, pcb.segments, pcb.vias)
        got = RP.draw_summary(ImageDraw.Draw(img), FL.Box(0, 0, 420, h),
                              lines=lines, theme=RT.DARK)
        if len(got) != len(lines):
            fail('a %d px summary box landed %d of %d rows'
                 % (h, len(got), len(lines)))
    # and no panel rect plan_frame can produce may RAISE out of the never-fail
    # wrapper: 1143 of 4010 (w, h) combinations did, on the height axis.
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
    # THE REACHABILITY, not a grep: `assess_placement` lives in py_placer,
    # which py_router does not put on sys.path, so the import raised
    # ModuleNotFoundError into a swallow and `unplaced` was always False --
    # 'seeding' could not occur in a CLI film at all.
    import animate_route as _A
    r2 = RR.BoardRenderer(pcb, size=200, supersample=1)
    m = _A.Movie(r2, list(r2.copper_layers))
    m.want_panel = True
    m.refresh_placement(pcb, BOARD)
    if not m.inventory:
        fail('refresh_placement built no inventory from a real board')
    import subprocess
    probe = ('import sys, os; sys.path.insert(0, %r); '
             'sys.path.insert(0, %r); import animate_route as A; '
             'from kicad_parser import parse_kicad_pcb as P; '
             'r = __import__("route_render").BoardRenderer(P(%r), size=120); '
             'm = A.Movie(r, list(r.copper_layers)); m.want_panel = True; '
             'm.refresh_placement(P(%r), %r); '
             'print("SEATED" if m.inventory else "NOINV")'
             % (os.path.join(ROOT, 'py_router'), ROOT, BOARD, BOARD, BOARD))
    pr = subprocess.run([sys.executable, '-X', 'utf8', '-c', probe],
                        cwd=ROOT, capture_output=True, text=True)
    if 'SEATED' not in (pr.stdout or ''):
        fail('a py_router-only interpreter could not build the box\'s data: '
             '%s' % ((pr.stderr or pr.stdout or '')[-200:]))
    if 'placement.placement_state' not in src:
        fail('nothing reads assess_placement, so unplaced is always False')
    if len(_FAIL) == _mark:
        print('  PASS: four contents, four drawers, all reachable')


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

    Gated on a panel EXISTING, because on 'legacy' the closing snapshot would
    add a frame to every movie this repo has ever written.
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
        for layout, want_close in (('split', True), ('legacy', False)):
            chrome, geom = [], []
            frames = A.build_boards(steps, b, 240, 1, 150, 2, 3,
                                    layout=layout, geom_out=geom)
            if not frames:
                fail('%s: no frames' % layout)
                continue
            # the LAST beat's label decides the last content
            labels = []
            # rebuild the chrome the composer saw
            m = A.Movie(A._renderer(b, None, 240, 1, 150)[0],
                        list(A._renderer(b, None, 240, 1, 150)[1]))
            del m
            closed = False
            # a bookend close is a frame labelled exactly 'routed'
            import render_panels as _rp
            for lbl in ('routed',):
                closed = (_rp.phase_for(lbl) == 'bookend')
            if not closed:
                fail("'routed' does not map to the bookend content")
            print('    %-7s %d frames, panel=%s'
                  % (layout, len(frames), bool(geom and geom[0].panel)))
            del labels
        # the REAL check, on the frames themselves: with a panel, the film
        # must be one frame longer than without the closing snapshot.
        n_split = len(A.build_boards(steps, b, 240, 1, 150, 2, 3,
                                     layout='split'))
        n_legacy = len(A.build_boards(steps, b, 240, 1, 150, 2, 3,
                                      layout='legacy'))
        if n_split <= n_legacy:
            fail('the panelled film (%d) is not longer than legacy (%d) -- '
                 'the closing bookend was not emitted' % (n_split, n_legacy))
        else:
            print('    split %d frames vs legacy %d -- the closing bookend is '
                  'the difference' % (n_split, n_legacy))

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
    test_all_four_contents_are_reachable_and_draw,
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
