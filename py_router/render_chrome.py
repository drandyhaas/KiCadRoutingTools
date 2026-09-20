#!/usr/bin/env python3
"""The chrome around a picture: the key, and the marks it draws (#946, #1014).

`render_theme` says WHAT COLOUR a role is and WHAT MARK it carries; this draws
them. It is the only one of the design-system modules that touches PIL, and it
imports it the way `route_render.py:48-56` does.

**THE RULE THIS MODULE EXISTS TO SHARE** comes from
`py_tools/render_placement.draw_legend`, which has had it right since #896 and
had it alone:

    Only the keys the panel can ACTUALLY SHOW are drawn -- a legend listing
    arrows on a panel with no --before is itself misinformation.

That rule is if anything more useful on the routing side, because a movie of a
clean run never rips anything and must not advertise a rip colour.

**AND A KEY NEVER FAILS A RENDER.** `draw_legend` wraps its whole body in
`except Exception: pass` with the comment "a legend is never worth failing a
render over", and that is kept here. A movie is an artifact; losing one to a
font metric is trading a cosmetic problem for a real one, which this repo
refuses elsewhere too (`movie_panels._finite`).

The mark vocabulary is `render_theme.MARKS`, unchanged from what `draw_legend`
already drew, so that function ports as a caller rather than as a rewrite --
and so the key and the draw site cannot disagree, because both read the mark
off the same role.
"""
from __future__ import annotations

import os
import sys
from typing import Optional, Sequence, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

#: One key row. `mark` is a `render_theme.MARKS` member.
#: (rgb, mark, text)
Row = Tuple[Tuple[int, int, int], str, str]


def draw_mark(d, box: Sequence[int], rgb, mark: str) -> None:
    """One swatch, in its own channel. Lifted from `draw_legend` unchanged --
    the shapes are the shapes the placement renderer has always drawn."""
    x0, y0, x1, y1 = box
    sw = x1 - x0
    if mark == 'solid':
        d.rectangle(box, fill=rgb)
    elif mark == 'ring':
        d.ellipse(box, outline=rgb, width=2)
    elif mark == 'dashed':
        for k in range(0, max(1, sw), 4):
            d.line([x0 + k, y0, x0 + k + 2, y0], fill=rgb)
            d.line([x0 + k, y1, x0 + k + 2, y1], fill=rgb)
    elif mark == 'hatch':
        d.rectangle(box, outline=rgb, width=1)
        for k in range(0, max(1, sw), 3):
            d.line([x0 + k, y1, x0 + sw, y0 + k], fill=rgb)
    elif mark == 'arrow':
        d.line([x0, y1, x1, y0], fill=rgb, width=2)
    else:  # 'line'
        d.line([x0, (y0 + y1) // 2, x1, (y0 + y1) // 2], fill=rgb, width=2)


def draw_key(d, rows: Sequence[Row], *, width: int, height: int,
             theme=None, corner: str = 'bl', pad_scale: int = 1) -> None:
    """Draw a key of `rows` into a canvas of `width` x `height`.

    `corner` is 'bl' / 'br' / 'tl' / 'tr'. `pad_scale` is the supersample
    factor when drawing onto a supersampled canvas -- `render_placement`
    passes `r.ss`, because an overlay draws at `ss` times the output size and
    using the OUTPUT height there once put the legend a fraction of the way up
    the image, on top of the board.

    Never raises: see the module docstring.
    """
    if not rows:
        return
    try:
        import render_theme
        from route_render import load_font
        th = theme or render_theme.DARK
        font = load_font(max(10, height // 78))
        pad = 6 * max(1, int(pad_scale))
        sw = max(10, height // 90)
        lh = sw + 5
        h = lh * len(rows) + 8
        w = max(int(d.textlength(t, font=font)) for _, _, t in rows) + sw + 20
        x0 = pad if corner in ('bl', 'tl') else width - w - pad
        y0 = height - h - pad if corner in ('bl', 'br') else pad
        d.rectangle([x0, y0, x0 + w, y0 + h], fill=th.rgb('chrome_band'))
        for i, (rgb, mark, text) in enumerate(rows):
            yy = y0 + 4 + i * lh
            draw_mark(d, [x0 + 6, yy, x0 + 6 + sw, yy + sw], rgb, mark)
            d.text((x0 + 6 + sw + 6, yy - 1), text,
                   fill=th.rgb('chrome_strip_text'), font=font)
    except Exception:                                          # noqa: BLE001
        pass          # a key is never worth failing a render over


def rows_for_roles(theme, roles: Sequence[Tuple[str, str]]) -> list:
    """`[(role, label), ...]` -> key rows, reading BOTH the colour and the
    mark off the theme.

    This is what makes the key and the draw site agree by construction: a role
    that is drawn dashed is keyed dashed, because neither side chooses.
    """
    return [(theme.rgb(role), theme.mark(role), label) for role, label in roles]


#: The routing movie's event vocabulary, in the order a viewer meets it.
EVENT_KEY = (('event_new', 'new'),
             ('event_restored', 'restored'),
             ('event_ripped', 'ripped'))


def event_rows(theme, *, seen: Optional[Sequence[str]] = None) -> list:
    """Key rows for the events a run ACTUALLY produced.

    `seen` is the set of event roles the movie emitted. `None` means "all of
    them", which is only right for a still that is documenting the vocabulary
    rather than reporting a run. A clean run rips nothing and must not
    advertise a rip colour -- that is #896's rule, and the reason this argument
    is not optional in the movie path.
    """
    wanted = [(r, lbl) for r, lbl in EVENT_KEY if seen is None or r in seen]
    return rows_for_roles(theme, wanted)


# ---------------------------------------------------------------------------
# the rail, the event line and the totals (#946 item 12, #1019)
# ---------------------------------------------------------------------------
#
# `route_render._label` stamped ONE top-left strip that was simultaneously
# title, step/progress indicator, event name and metrics readout. Its own
# docstring records what that cost:
#
#     "Measured on a 217-part board: the caption built 156 chars and ~117 fit,
#     so `hole-conflict 0.60mm` and `oob 7` -- a fab blocker and the off-board
#     count ... were absent from the picture while the strip looked complete
#     because it ended at a plausible-looking field."
#
# Wrapping fixed the clipping and treated the symptom. ONE STRIP DOING FOUR
# JOBS is why it overflowed, and wrapping then cost vertical space on every
# frame to serve the worst case. Three regions, each sized for its own
# content:
#
#   rail    STABLE   -- the board, the lap, the phase
#   event   PER-FRAME-- what just happened
#   totals  A BLOCK  -- the running numbers
#
# And the rail counts LAPS, not steps. A loop revisits the same step, so
# `step 2 - route` cannot say whether this is the first attempt or the fourth.


def _fit(d, text, font, width):
    """`text`, ellipsised to `width`. Returns '' when nothing fits."""
    if not text:
        return ''
    if d.textlength(text, font=font) <= width:
        return text
    ell = '...'
    lo, hi = 0, len(text)
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if d.textlength(text[:mid] + ell, font=font) <= width:
            lo = mid
        else:
            hi = mid - 1
    return (text[:lo] + ell) if lo else ''


def draw_rail(d, box, left, right, *, theme, pad_scale=1, progress=None,
              ticks=()):
    """The stable strip: `left` at the left, `right` right-aligned, and an
    optional progress bar with tick marks where each lap began."""
    if box is None or box.h <= 0:
        return
    try:
        import render_theme
        from route_render import load_font
        th = theme or render_theme.DARK
        ss = max(1, int(pad_scale))
        pad = 6 * ss
        font = load_font(max(10, int(box.h * 0.42)))
        # -1 on both: PIL's rectangle is INCLUSIVE of its far corner, so
        # `box.y + box.h` bleeds one row into whatever is below. A
        # region that draws outside itself is the same defect as a
        # strip that overflows, one pixel at a time.
        d.rectangle([box.x, box.y,
                     box.x + box.w - 1, box.y + box.h - 1],
                    fill=th.rgb('chrome_panel'))
        rw = int(d.textlength(right or '', font=font))
        d.text((box.x + pad, box.y + pad // 2),
               _fit(d, left, font, box.w - rw - 4 * pad), font=font,
               fill=th.rgb('chrome_text'))
        if right:
            d.text((box.x + box.w - pad, box.y + pad // 2), right, font=font,
                   fill=th.rgb('chrome_text_dim'), anchor='ra')
        if progress is not None:
            bar_y = box.y + box.h - max(2, ss * 2)
            d.rectangle([box.x + pad, bar_y, box.x + box.w - pad,
                         bar_y + max(1, ss)], fill=th.rgb('chrome_rule'))
            w = (box.w - 2 * pad) * max(0.0, min(1.0, float(progress)))
            d.rectangle([box.x + pad, bar_y, box.x + pad + w,
                         bar_y + max(1, ss)], fill=th.rgb('pad'))
            for t in ticks or ():
                tx = box.x + pad + (box.w - 2 * pad) * max(0.0, min(1.0, t))
                d.rectangle([tx - max(1, ss // 2), bar_y - 3 * ss,
                             tx + max(1, ss // 2), bar_y + 2 * ss],
                            fill=th.rgb('chrome_text_faint'))
    except Exception:                                          # noqa: BLE001
        pass


def draw_totals(d, box, text, *, theme, pad_scale=1):
    """The running numbers, right-aligned in the foot.

    Given its own region precisely so it stops competing with the event line
    for the same characters -- which is how `hole-conflict 0.60mm` and `oob 7`
    fell off the edge of a strip that still looked complete.

    **Returns the strings it actually drew.** A region cannot be asked by
    pixel whether a field survived -- that would need OCR -- so it reports.
    `tests/test_946_caption_split.py` asserts every field is represented in
    what comes back, which is the claim that matters: no field is dropped
    because another was long.
    """
    if box is None or box.h <= 0 or not text:
        return []
    try:
        import render_theme
        from route_render import load_font
        th = theme or render_theme.DARK
        ss = max(1, int(pad_scale))
        font = load_font(max(9, int(box.h * 0.34)))
        pad = 6 * ss
        parts = [p.strip() for p in str(text).split('|') if p.strip()]
        if not parts:
            return
        lh = int(font.size * 1.25)
        # STACK when the region can hold every field, otherwise put them on ONE
        # line. The failure this whole change exists to fix is a field being
        # dropped because another was long -- so running out of rows must not
        # reintroduce it by a different route. One ellipsised line still shows
        # that a field is there; a missing row shows nothing at all.
        if len(parts) * lh <= box.h - pad:
            y = box.y + pad // 2
            drawn = []
            for p in parts:
                t = _fit(d, p, font, box.w - 2 * pad)
                d.text((box.x + box.w - pad, y), t, font=font,
                       fill=th.rgb('chrome_text_dim'), anchor='ra')
                drawn.append(t)
                y += lh
            return drawn
        t = _fit(d, '  -  '.join(parts), font, box.w - 2 * pad)
        d.text((box.x + box.w - pad, box.y + pad // 2), t, font=font,
               fill=th.rgb('chrome_text_dim'), anchor='ra')
        return [t]
    except Exception:                                          # noqa: BLE001
        return []


def lap_text(label, lap=None, laps=None, phase=None):
    """`lap 3 of 5 - route`, or the step label when there is no loop.

    A loop revisits the same step, so a step number cannot say whether this is
    the first attempt or the fourth. When the chain is not a loop there are no
    laps to count and the label is the honest thing to show.
    """
    if lap is None:
        return label or ''
    out = 'lap %d' % lap
    if laps:
        out += ' of %d' % laps
    if phase:
        out += '  -  %s' % phase
    return out
