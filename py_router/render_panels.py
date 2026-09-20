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

    open / close   the 3D board, turning
    placement      what moved -- ghost, arrow, courtyard
    routing        the per-layer strip
    seeding        a staging inventory, emptying by part class

One box, four contents, switched by the phase the frame belongs to -- which the
film already knows, because `Stage` is built from the round records and every
frame belongs to a round. The frame height never changes, so the Pillow trap is
not reintroduced.

**THE STRIP BUILDS NO SECOND RENDERER.** `tests/test_431_placement_movie.py:92`
asserts exactly one `BoardRenderer` on the no-stage path, so a strip of ten
small boards cannot construct ten of them. It draws the copper directly into
each cell with its own scale, the way `awx/evolve_movie` draws its mini-boards.
"""
from __future__ import annotations

import os
import sys
from typing import Optional, Sequence

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: Below this cell width a layer cell cannot show a route, only that copper
#: exists. Measured against the layout study's px-per-layer-cell figures: the
#: inset layout gives 32k px per cell against the split layout's 130k.
CELL_MIN_W = 26


def _cell_boxes(box, n, gap=6):
    """`n` cells across `box`, left to right."""
    if n <= 0 or box is None or box.w <= 0:
        return []
    cw = (box.w - gap * (n + 1)) / float(n)
    if cw < CELL_MIN_W:
        # Fewer, wider cells beat more, unreadable ones: a cell too small to
        # show a route costs pixels and answers nothing.
        n = max(1, int((box.w - gap) // (CELL_MIN_W + gap)))
        cw = (box.w - gap * (n + 1)) / float(n)
    return [(int(box.x + gap + i * (cw + gap)), int(box.y + gap),
             int(cw), int(box.h - 2 * gap)) for i in range(n)], n


def draw_layer_strip(d, box, *, bounds, segments, layers, palette, theme,
                     caption_h=14, active=None):
    """Small multiples: one mini board per copper layer.

    Colour stops carrying layer identity here and POSITION carries it instead
    -- and position never collides, which is the answer to the 19 crossings
    that could impersonate a third layer. Each cell draws at full strength on
    its own ground, so there is no alpha dimming either.
    """
    if box is None or box.h <= 0 or not layers:
        return 0
    try:
        import render_theme
        from route_render import load_font
        th = theme or render_theme.DARK
        boxes, n = _cell_boxes(box, len(layers))
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
            cnt = 0
            for s in by_layer.get(ln, ()):
                cnt += 1
                d.line([ox + (s.start_x - min_x) * sc,
                        oy + (s.start_y - min_y) * sc,
                        ox + (s.end_x - min_x) * sc,
                        oy + (s.end_y - min_y) * sc],
                       fill=col, width=1)
            d.text((cx + 4, cy + 1), ln.replace('.Cu', ''), font=font,
                   fill=(th.rgb('pad') if ln == active
                         else th.rgb('chrome_text_dim')))
            d.text((cx + cw - 4, cy + 1), str(cnt), font=font,
                   fill=th.rgb('chrome_text_faint'), anchor='ra')
        if len(layers) > n:
            d.text((box.x + box.w - 4, box.y + box.h - 14),
                   '+%d more' % (len(layers) - n), font=font,
                   fill=th.rgb('chrome_text_faint'), anchor='ra')
        return n
    except Exception:                                          # noqa: BLE001
        return 0        # a panel is never worth failing a render over


def draw_inventory(d, box, *, counts, placed, total, theme):
    """The seeding content: what is LEFT in the pile, by part class.

    It empties as the board fills, which is the one thing a viewer wants to
    know during a phase where the board itself is mostly still empty -- and it
    is the honest content for a phase where ghost-and-arrow is wrong, because
    a part from a pile 200 mm away has a `from` that is noise.
    """
    if box is None or box.h <= 0:
        return
    try:
        import render_theme
        from route_render import load_font
        th = theme or render_theme.DARK
        font = load_font(max(9, min(14, int(box.h * 0.13))))
        d.rectangle([box.x, box.y, box.x + box.w - 1, box.y + box.h - 1],
                    fill=th.rgb('chrome_panel'))
        rows = sorted(counts.items(), key=lambda kv: -kv[1][1])
        pad, lh = 8, int(font.size * 1.7)
        y = box.y + pad
        for name, (done, tot) in rows:
            if y + lh > box.y + box.h - pad:
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
            d.text((box.x + box.w - pad, y), '%d/%d' % (done, tot), font=font,
                   fill=th.rgb('chrome_text_faint'), anchor='ra')
            y += lh
        d.text((box.x + pad, box.y + box.h - pad - font.size),
               '%d of %d placed' % (placed, total), font=font,
               fill=th.rgb('pad'))
    except Exception:                                          # noqa: BLE001
        pass


def phase_for(label, *, unplaced=False):
    """Which of the four contents this frame's label asks for.

    `Stage` builds from the round records and every frame belongs to a round,
    so the phase is already known -- this only names it.
    """
    s = (label or '').lower()
    if unplaced:
        return 'seeding'
    if 'input' in s or 'overview' in s or 'routed' == s.strip():
        return 'bookend'
    if 'moving' in s or 'placing' in s or 're-placing' in s or 'moved' in s:
        return 'placement'
    return 'routing'
