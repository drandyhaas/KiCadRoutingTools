#!/usr/bin/env python3
"""Draw the declared floorplan (#946 item 5, #1017).

A floorplan intent is a fully geometric document -- `docs/floorplan-intent.md`:
`blocks[]` carry a named `zone` rect plus the refs that belong in it,
`keepouts[]` a `rect` or a `circle`, `edge_connectors[]` an edge plus an
`along_edge_band` given as two fractions. All drawable.

**And nothing drew it.** `render_placement --intent` reads that file for exactly
one thing -- `overlap_waivers` -- so the plan existed only as JSON and as
pass/fail counts out of `check_floorplan`. The film opened on parts arriving
with no indication of where they were supposed to go, and on an UNPLACED board
the declared zones are the only statement of intent there is: no prior
arrangement to compare against, so without them the opening frames are an empty
outline and nothing else.

**Drawn through `frame(overlays=...)`**, which draws at supersampled resolution
above copper and below the label -- so this costs NO FRAME GEOMETRY and is
available to the still renderer and the movie on the same terms.

**This module reads no intent file.** It takes an already-parsed `Intent` (or
anything with the same three attributes), because `placement.floorplan` owns
what an intent MEANS and a renderer that re-parsed one would be a second
opinion about it. It also imports no PIL at module scope, for the reason
`render_theme` does not: `render_placement` may be imported by graders that
draw nothing.
"""
from __future__ import annotations

import os
import sys
from typing import Optional, Sequence

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: How far a zone's fill is pushed toward transparent. A plan is CONTEXT while
#: parts move -- loud enough to be a statement, quiet enough that the copper
#: and the parts stay the subject.
FILL_ALPHA = 0.06


def _rect_pts(tf, rect):
    x0, y0, x1, y1 = rect
    a = tf.pt(x0, y0)
    b = tf.pt(x1, y1)
    return [min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])]


def _dashed_rect(d, box, rgb, w, on=7, off=5):
    x0, y0, x1, y1 = box
    for (ax, ay, bx, by) in ((x0, y0, x1, y0), (x1, y0, x1, y1),
                             (x1, y1, x0, y1), (x0, y1, x0, y0)):
        dx, dy = bx - ax, by - ay
        ln = max(abs(dx), abs(dy)) or 1
        ux, uy = dx / ln, dy / ln
        t = 0.0
        while t < ln:
            t2 = min(t + on, ln)
            d.line([ax + ux * t, ay + uy * t, ax + ux * t2, ay + uy * t2],
                   fill=rgb, width=w)
            t += on + off


def draw_plan(d, r, intent, *, verdict=None, alpha=1.0, theme=None,
              label=True):
    """Draw `intent`'s geometry onto `d`, aimed by renderer `r`.

    `verdict` maps a block name to True (held) / False (drifted). When given,
    blocks are recoloured by it -- which is what `violations_by_rule` already
    computes, so the close of a film can grade against the plan it opened on
    without a second measurement.

    `alpha` scales the stroke weight and is how the same call serves the
    opening (full) and the context pass while parts move (faint).

    Never raises: a plan overlay is not worth failing a render over, the rule
    `render_placement.draw_legend` established.
    """
    try:
        import render_theme
        from route_render import load_font
        th = theme or getattr(r, 'theme', None) or render_theme.DARK
        ss = max(1, int(getattr(r, 'ss', 1)))
        w = max(1, int(round(1.4 * ss * (0.5 + 0.5 * alpha))))
        font = load_font(max(9, int(11 * ss)))
        tf = r.tf
        zone_c = th.rgb('place_court_back')
        keep_c = th.rgb('defect_courtyard')
        edge_c = th.rgb('place_arrow')
        ink = th.rgb('chrome_strip_text')

        for z in (getattr(intent, 'blocks', None) or ()):
            rect = getattr(z, 'rect', None)
            if not rect:
                continue
            box = _rect_pts(tf, rect)
            col = zone_c
            if verdict is not None:
                held = verdict.get(getattr(z, 'name', None))
                col = (th.rgb('status_kept') if held
                       else th.rgb('defect_conflict'))
            _dashed_rect(d, box, col, w)
            if label:
                refs = ' '.join((getattr(z, 'refs', None) or ())[:4])
                txt = str(getattr(z, 'name', '') or '').upper()
                if refs:
                    txt += '  ' + refs
                d.text((box[0] + 6 * ss, box[1] + 4 * ss), txt, fill=col,
                       font=font)
                if verdict is not None:
                    held = verdict.get(getattr(z, 'name', None))
                    d.text((box[0] + 6 * ss, box[3] - 16 * ss),
                           'held' if held else 'drifted', fill=col, font=font)

        for k in (getattr(intent, 'keepouts', None) or ()):
            rect, circ = k.get('rect'), k.get('circle')
            if rect:
                box = _rect_pts(tf, rect)
            elif circ and len(circ) >= 3:
                cx, cy, rad = circ[0], circ[1], circ[2]
                p0 = tf.pt(cx - rad, cy - rad)
                p1 = tf.pt(cx + rad, cy + rad)
                box = [p0[0], p0[1], p1[0], p1[1]]
                d.ellipse(box, outline=keep_c, width=w)
            else:
                continue
            if rect:
                d.rectangle(box, outline=keep_c, width=w)
            # HATCHED, not just coloured: a keep-out is a prohibition, and a
            # prohibition that reads only in the colour channel is the class of
            # defect this whole issue is about.
            step = max(5, 7 * ss)
            x0, y0, x1, y1 = [int(v) for v in box]
            # CLIPPED to the box. The first version drew the full diagonals and
            # they ran out past the keep-out on both sides, which reads as a
            # bigger prohibition than the one declared -- the exact failure a
            # drawing of a rule must not have.
            for i in range(int(y0 - (x1 - x0)), int(y1), step):
                ax, ay, bx, by = x0, i + (x1 - x0), x1, i
                # clamp the segment to y in [y0, y1] along its 45-degree slope
                if ay > y1:
                    ax += (ay - y1); ay = y1
                if by < y0:
                    bx -= (y0 - by); by = y0
                if ax > bx or ay < y0 or by > y1:
                    continue
                d.line([max(x0, ax), min(y1, ay), min(x1, bx), max(y0, by)],
                       fill=keep_c, width=1)
            if label:
                d.text((x0 + 4 * ss, y0 - 14 * ss), 'KEEP OUT', fill=keep_c,
                       font=font)

        bounds = getattr(r, 'bounds', None)
        for e in (getattr(intent, 'edge_connectors', None) or ()):
            band = e.get('along_edge_band') or {}
            edge = (e.get('edge') or '').lower()
            if not bounds or 'from' not in band or 'to' not in band:
                continue
            min_x, min_y, max_x, max_y = bounds
            f, t = float(band['from']), float(band['to'])
            if edge in ('west', 'east'):
                x = min_x if edge == 'west' else max_x
                a = tf.pt(x, min_y + (max_y - min_y) * f)
                b = tf.pt(x, min_y + (max_y - min_y) * t)
            elif edge in ('north', 'south'):
                y = min_y if edge == 'north' else max_y
                a = tf.pt(min_x + (max_x - min_x) * f, y)
                b = tf.pt(min_x + (max_x - min_x) * t, y)
            else:
                continue
            d.line([a[0], a[1], b[0], b[1]], fill=edge_c, width=max(3, 4 * ss))
            if label:
                d.text((a[0] + 6 * ss, (a[1] + b[1]) / 2 - 6 * ss),
                       '%s - %s' % (e.get('ref', '?'), edge), fill=edge_c,
                       font=font)
        return True
    except Exception:                                          # noqa: BLE001
        return False   # a plan overlay is never worth failing a render over


def plan_overlay(intent, *, verdict=None, alpha=1.0, theme=None, label=True):
    """`draw_plan` as an `frame(overlays=...)` callable."""
    def _fn(d, r):
        draw_plan(d, r, intent, verdict=verdict, alpha=alpha, theme=theme,
                  label=label)
    return _fn


def plan_summary(intent) -> str:
    """One line describing what was declared -- printed when a plan is drawn,
    and ALSO when it is empty.

    An intent with no blocks must SAY it drew nothing rather than silently
    rendering a clean-looking empty overlay: a picture of nothing is
    indistinguishable from a picture of a plan that was met.
    """
    nb = len(getattr(intent, 'blocks', None) or ())
    nk = len(getattr(intent, 'keepouts', None) or ())
    ne = len(getattr(intent, 'edge_connectors', None) or ())
    if not (nb or nk or ne):
        return ('floorplan intent: NOTHING DECLARED -- no blocks, no '
                'keep-outs, no edge connectors, so the plan overlay drew '
                'nothing')
    return ('floorplan intent: %d block(s), %d keep-out(s), %d edge '
            'connector(s)' % (nb, nk, ne))
