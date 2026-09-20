#!/usr/bin/env python3
"""One fixed lower box, four contents, switched by phase (#946 items 6/10/11,
#1020).

The obvious way to bookend a 3D shot is to show the panel at the start and the
end and drop it in between. **The frame geometry forbids exactly that**: every
frame must be the same size, Pillow does not raise on a mismatch, and the GIF
comes out valid and quietly distorted. A panel that appears and disappears is
not available.

And the default run is `place_route_loop`, so the combined film is the normal
case, which settles what the box holds -- it cannot be the layer strip, because
during a placement phase there is no copper; it cannot be the 3D view, because
that is the bookend:

    open / close   a board summary -- parts, nets, layers, copper
    placement      the inventory: how much of the board is seated, by class
    routing        the per-layer strip
    seeding        the same inventory, emptying as the pile empties

One box, four contents, switched by the phase the frame belongs to -- which the
film already knows, because `Stage` is built from the round records and every
frame belongs to a round. The frame height never changes, so the Pillow trap is
not reintroduced.

**THE STRIP BUILDS NO SECOND RENDERER.** `tests/test_431_placement_movie.py:92`
asserts exactly one `BoardRenderer` on the no-stage path, so a strip of ten
small boards cannot construct ten of them. It draws the copper directly into
each cell with its own scale, the way `awx/evolve_movie` draws its mini-boards.

**EVERY DRAWER REPORTS WHAT IT DREW.** `draw_layer_strip` returns a `Cell` per
cell carrying the count string it actually stamped and the number of copper
lines it actually issued -- not the tally it computed. The phase-1 verifier
measured why: a test that re-derives the counts from `pcb.segments` and never
reads the drawing passes unchanged when every cell draws `cnt + 7`, and passes
unchanged when every cell counts and draws EVERY segment on the board. Both
mutants survived while the test printed "PASS: every cell counts its own
layer". A region cannot be asked by pixel what number it wrote -- that would
need OCR -- so it reports, exactly as `render_chrome.draw_totals` does.

**AND A COUNT THAT DOES NOT FIT IS DROPPED, NOT OVERPRINTED.** `CELL_MIN_W`
bounds the cell WIDTH; nothing bounded the text, so at the widths this feature
actually produces the count was stamped on top of the layer name -- measured at
+25 px of overlap at `CELL_MIN_W` exactly, +23 px in the 180 px case the test
itself exercises, and visible in the phase's own acceptance image ("F75",
"In1390"). The name is the identity and the count is the extra, so the count
goes and the returned `Cell` says so.
"""
from __future__ import annotations

import os
import sys
from typing import List, NamedTuple

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: Below this cell width a layer cell cannot show a route, only that copper
#: exists. Measured against the px-per-layer-cell figures: at one pixel
#: budget the inset layout gives **28 490** px per cell against the split
#: layout's 128 800 -- a 4.5x penalty, which is what makes a floor on the
#: CELL rather than on the count the right guard.
#:
#: That 28 490 was quoted as "32k" here and in three other places until the
#: PR's fact-checker reconstructed it: a 12% error in the number this
#: constant leans on, uncatchable because nothing computed it. `py_router/layout_budget.py` computes these; `tests/test_946_layout_budget.py` pins them.
CELL_MIN_W = 26

#: And below THIS a cell cannot be drawn at all -- `d.rectangle` raises when
#: `x1 < x0`, which the never-fail wrapper would turn into a blank panel with
#: nothing said. Measured: without it a 10 px box solved to a cell -2 px wide.
CELL_FLOOR_W = 8

#: The same fault on the OTHER axis, which the first fix missed and the
#: round-2 verifier measured: a cell's mini-board starts `caption_h` below the
#: cell top, so a short cell inverts that rectangle and `d.rectangle` raises
#: "y1 must be greater than or equal to y0". **1143 of 4010 (width, height)
#: combinations** did it, at panel heights 13/14/20 px -- rects
#: `frame_layout.plan_frame` produces on its own (`legacy --size 100` gives a
#: 100x16 panel). Every one was swallowed into a blank box with nothing said.
CELL_FLOOR_H = 6

#: Gap between the layer name and its count. Below it they are touching, which
#: is the defect this constant exists to refuse.
LABEL_GAP_PX = 5


class Cell(NamedTuple):
    """What ONE cell of the strip actually drew.

    `count_text` is the string handed to `d.text`, not the tally behind it, and
    `''` means the count did not fit and was dropped. `lines` is how many
    copper segments were actually stroked. Both are the drawing's own report;
    `count` is what the caller's data said, so a test can compare the two.
    """
    layer: str
    count: int
    count_text: str
    name_text: str
    lines: int
    box: tuple


def _cell_boxes(box, n, gap=6, caption_h=14):
    """`(boxes, n)` -- `n` cells across `box`, left to right.

    ALWAYS a 2-tuple. It used to return a bare `[]` on the empty path while its
    only caller unpacked two values, so a zero-width box raised `ValueError`
    into a bare `except` and the panel went blank with nothing said. Reachable
    only at `box.w <= 0`, which `plan_frame._self_check` `continue`s past
    rather than refusing -- so it was unreachable by luck, not by design.

    **BOTH AXES ARE FLOORED**, which the first version of this guard got half
    right: a cell has to hold its caption AND a mini-board below it, so a short
    cell inverts the board rectangle and `d.rectangle` raises. Measured at 1143
    of 4010 (width, height) combinations, at panel heights `plan_frame`
    produces on its own.
    """
    if n <= 0 or box is None or box.w <= 0 or box.h <= 0:
        return [], 0
    cw = (box.w - gap * (n + 1)) / float(n)
    if cw < CELL_MIN_W:
        # Fewer, wider cells beat more, unreadable ones: a cell too small to
        # show a route costs pixels and answers nothing.
        n = max(1, int((box.w - gap) // (CELL_MIN_W + gap)))
        cw = (box.w - gap * (n + 1)) / float(n)
    ch = box.h - 2 * gap
    if cw < CELL_FLOOR_W or ch < caption_h + CELL_FLOOR_H:
        return [], 0
    return [(int(box.x + gap + i * (cw + gap)), int(box.y + gap),
             int(cw), int(ch)) for i in range(n)], n


def draw_layer_strip(d, box, *, bounds, segments, layers, palette, theme,
                     caption_h=14, active=None) -> List[Cell]:
    """Small multiples: one mini board per copper layer.

    Colour stops carrying layer identity here and POSITION carries it instead
    -- and position never collides, which is the answer to the 19 crossings
    that could impersonate a third layer. Each cell draws at full strength on
    its own ground, so there is no alpha dimming either.

    Returns one `Cell` per cell DRAWN; `len()` is the cell count the caller
    used to read off the old integer return. See the module docstring for why
    the return carries the drawn strings rather than the computed tallies.
    """
    if box is None or box.h <= 0 or not layers:
        return []
    out: List[Cell] = []
    try:
        import render_theme
        from route_render import load_font
        th = theme or render_theme.DARK
        boxes, n = _cell_boxes(box, len(layers), caption_h=caption_h)
        if not n:
            return []
        shown = layers[:n]
        font = load_font(max(8, min(13, int(box.h * 0.16))))
        min_x, min_y, max_x, max_y = bounds
        bw = max(max_x - min_x, 1e-6)
        bh = max(max_y - min_y, 1e-6)
        by_layer = {}
        for s in segments:
            by_layer.setdefault(s.layer, []).append(s)
        for i, ln in enumerate(shown):
            cx, cy, cw, ch = boxes[i]
            iy = cy + caption_h
            ih = max(2, ch - caption_h)
            d.rectangle([cx, cy, cx + cw - 1, cy + ch - 1],
                        fill=th.rgb('chrome_panel'),
                        outline=(th.rgb('pad') if ln == active
                                 else th.rgb('chrome_rule')))
            d.rectangle([cx + 1, iy, cx + cw - 2, cy + ch - 2],
                        fill=th.rgb('board_body'))
            sc = min((cw - 4) / bw, (ih - 4) / bh)
            ox = cx + 2 + ((cw - 4) - bw * sc) / 2
            oy = iy + 2 + ((ih - 4) - bh * sc) / 2
            col = palette.get(ln, th.rgb('chrome_text_dim'))
            cnt = drew = 0
            for s in by_layer.get(ln, ()):
                cnt += 1
                d.line([ox + (s.start_x - min_x) * sc,
                        oy + (s.start_y - min_y) * sc,
                        ox + (s.end_x - min_x) * sc,
                        oy + (s.end_y - min_y) * sc],
                       fill=col, width=1)
                # AFTER the call, and a separate counter from `cnt`: a `lines`
                # field that is just the tally under another name reports
                # "drew 390" for a cell that drew nothing, which is the exact
                # shape of the defect this return value exists to catch.
                drew += 1
            name = ln.replace('.Cu', '')
            # The count is DROPPED rather than overprinted when the two strings
            # would touch: the name is the identity, the count is the extra.
            num = str(cnt)
            room = (cw - 8) - d.textlength(name, font=font) - LABEL_GAP_PX
            if d.textlength(num, font=font) > room:
                num = ''
            d.text((cx + 4, cy + 1), name, font=font,
                   fill=(th.rgb('pad') if ln == active
                         else th.rgb('chrome_text_dim')))
            if num:
                d.text((cx + cw - 4, cy + 1), num, font=font,
                       fill=th.rgb('chrome_text_faint'), anchor='ra')
            out.append(Cell(ln, cnt, num, name, drew, (cx, cy, cw, ch)))
        if len(layers) > n:
            # In the CELL CAPTION band, beside the last cell's own name, not
            # over its mini-board: drawn at the box's bottom it grazed the last
            # cell's copper by 4 px in every case the verifier measured.
            last = boxes[n - 1]
            d.text((box.x + box.w - 2, last[1] + last[3] + 2),
                   '+%d more' % (len(layers) - n), font=font,
                   fill=th.rgb('chrome_text_faint'), anchor='ra')
        return out
    except Exception:                                          # noqa: BLE001
        return out       # a panel is never worth failing a render over


def inventory_counts(pcb, unseated=()):
    """`{class: (seated, total)}` from the board itself.

    The class is the reference's letter prefix -- `R`, `C`, `U`, `J` -- which
    is what a person reads a BOM by, and it needs no data the film does not
    already have. `unseated` is `assess_placement(...).stacked_suspect_refs`:
    the parts still sitting on one another in the pile, which is precisely the
    set that has not been placed yet.
    """
    out = {}
    bad = set(unseated or ())
    for ref in getattr(pcb, 'footprints', {}) or {}:
        cls = ''.join(ch for ch in str(ref) if ch.isalpha())[:3] or '?'
        seated, total = out.get(cls, (0, 0))
        out[cls] = (seated + (0 if ref in bad else 1), total + 1)
    return out


def draw_inventory(d, box, *, counts, placed, total, theme):
    """The seeding content: what is LEFT in the pile, by part class.

    It empties as the board fills, which is the one thing a viewer wants to
    know during a phase where the board itself is mostly still empty -- and it
    is the honest content for a phase where ghost-and-arrow is wrong, because
    a part from a pile 200 mm away has a `from` that is noise.

    Returns the rows it drew, for the same reason `draw_layer_strip` does.
    """
    drawn = []
    if box is None or box.h <= 0:
        return drawn
    try:
        import render_theme
        from route_render import load_font
        th = theme or render_theme.DARK
        font = load_font(max(9, min(14, int(box.h * 0.13))))
        d.rectangle([box.x, box.y, box.x + box.w - 1, box.y + box.h - 1],
                    fill=th.rgb('chrome_panel'))
        rows = sorted(counts.items(), key=lambda kv: (-kv[1][1], kv[0]))
        pad, lh = 8, int(font.size * 1.7)
        y = box.y + pad
        for name, (done, tot) in rows:
            if y + lh > box.y + box.h - pad - font.size:
                break
            d.text((box.x + pad, y), name, font=font,
                   fill=th.rgb('chrome_text_dim'))
            bx = box.x + int(box.w * 0.42)
            bw = int(box.w * 0.46)
            d.rectangle([bx, y + 2, bx + bw, y + font.size],
                        fill=th.rgb('chrome_rule'))
            if tot:
                d.rectangle([bx, y + 2, bx + int(bw * done / float(tot)),
                             y + font.size], fill=th.rgb('status_kept'))
            txt = '%d/%d' % (done, tot)
            d.text((box.x + box.w - pad, y), txt, font=font,
                   fill=th.rgb('chrome_text_faint'), anchor='ra')
            drawn.append((name, txt))
            y += lh
        foot = '%d of %d placed' % (placed, total)
        d.text((box.x + pad, box.y + box.h - pad - font.size), foot,
               font=font, fill=th.rgb('pad'))
        drawn.append(('', foot))
        return drawn
    except Exception:                                          # noqa: BLE001
        return drawn


def draw_summary(d, box, *, lines, theme):
    """The bookend content: what this board IS, in numbers.

    The 3D view is `movie_panels`' iso panel and stacks in its own slot; what
    belongs in the lower box at the bookends is the thing a viewer wants at the
    start and again at the end -- parts, nets, layers, copper -- so the closing
    frame can be read against the opening one.
    """
    drawn = []
    if box is None or box.h <= 0 or not lines:
        return drawn
    try:
        import render_theme
        from route_render import load_font
        th = theme or render_theme.DARK
        # EVERY row it was given must land. Two failures measured, in order:
        # at a fixed 15 pt in a 132 px box the fifth row (`vias`) was clipped
        # away; sizing the font from the row COUNT fixed that at `--size 1000`
        # and still dropped one row on `stacked` at `--size 400` and three on
        # `inset`. So when one column cannot hold them, it WRAPS to two --
        # a summary that silently drops its last row is a summary you cannot
        # read a closing frame against an opening one with.
        pad = 8
        d.rectangle([box.x, box.y, box.x + box.w - 1, box.y + box.h - 1],
                    fill=th.rgb('chrome_panel'))
        n = max(1, len(lines))
        cols = 1
        while cols <= 3:
            per = (n + cols - 1) // cols
            size = int((box.h - 2 * pad) / (1.45 * max(1, per)))
            if size >= 8 or cols == 3:
                break
            cols += 1
        per = (n + cols - 1) // cols
        font = load_font(max(7, min(15, int((box.h - 2 * pad)
                                            / (1.45 * max(1, per))))))
        lh = int(font.size * 1.45)
        cw = box.w // cols
        for i, (label, value) in enumerate(lines):
            col, row = i // per, i % per
            x = box.x + pad + col * cw
            y = box.y + pad + row * lh
            if y + lh > box.y + box.h or x + cw - pad > box.x + box.w:
                break
            d.text((x, y), str(label), font=font,
                   fill=th.rgb('chrome_text_dim'))
            d.text((x + int(cw * 0.56), y), str(value), font=font,
                   fill=th.rgb('chrome_text'))
            drawn.append((str(label), str(value)))
        return drawn
    except Exception:                                          # noqa: BLE001
        return drawn


def board_summary(pcb, segments=(), vias=()):
    """`[(label, value), ...]` for `draw_summary`, off the board in hand."""
    fps = getattr(pcb, 'footprints', {}) or {}
    nets = getattr(pcb, 'nets', {}) or {}
    info = getattr(pcb, 'board_info', None)
    layers = list(getattr(info, 'copper_layers', ()) or ()) if info else []
    return [('parts', len(fps)),
            ('nets', max(0, len(nets) - 1)),   # net 0 is "no net"
            ('copper layers', len(layers)),
            ('segments', len(segments)),
            ('vias', len(vias))]


def phase_for(label, *, unplaced=False):
    """Which of the four contents this frame's label asks for.

    `Stage` builds from the round records and every frame belongs to a round,
    so the phase is already known -- this only names it.

    `unplaced` comes from `placement.placement_state.assess_placement`, and it
    wins over the label: on a board whose parts are still stacked at one
    coordinate there is nothing to say about what moved, and the honest content
    is the pile emptying.
    """
    s = (label or '').lower()
    if unplaced:
        return 'seeding'
    if 'input' in s or 'overview' in s or 'routed' == s.strip():
        return 'bookend'
    if 'moving' in s or 'placing' in s or 're-placing' in s or 'moved' in s:
        return 'placement'
    return 'routing'
