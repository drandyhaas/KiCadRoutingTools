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


#: THE TYPE SCALE (#946 review). Font heights as a share of the FRAME height,
#: one row per role, so a caption in any region of any layout is the same size
#: as every other caption in that film. The rail's own text is ~1.9% of the
#: frame (`draw_rail`: 0.42 of a 4.5% rail); a caption sits a step below it.
TYPE_SCALE = {'title': 0.019, 'caption': 0.0135, 'small': 0.011}
TYPE_MIN_PX = {'title': 11, 'caption': 10, 'small': 9}

#: The inner margin every panel's content keeps from its box, as a share of
#: the frame width, floored. Content drawn flush to a box edge reads as cut
#: off -- the 4:3 inventory's counts touched the frame's right edge.
GUTTER_FRAC = 0.008
GUTTER_MIN_PX = 6


def type_px(role, frame_h):
    """The font height for `role` in a frame `frame_h` pixels tall."""
    return max(TYPE_MIN_PX.get(role, 9),
               int(round(frame_h * TYPE_SCALE.get(role, 0.0135))))


def gutter_px(frame_w):
    """The design system's panel gutter for a frame `frame_w` pixels wide."""
    return max(GUTTER_MIN_PX, int(round(frame_w * GUTTER_FRAC)))


def fit_words(d, text, font, width):
    """`text` shortened to `width` at a WORD boundary, with an ellipsis.

    Never cuts mid-word: whole words are dropped from the end. Returns ''
    when not even the first word fits.
    """
    if not text:
        return ''
    if d.textlength(text, font=font) <= width:
        return text
    words = text.split(' ')
    ell = '…'
    for k in range(len(words) - 1, 0, -1):
        cand = ' '.join(words[:k]).rstrip(' |-,;:') + ell
        if d.textlength(cand, font=font) <= width:
            return cand
    return ''


def fit_parts(d, parts, font, width, sep='  |  '):
    """The longest caption that fits, dropping PARTS before cutting words.

    `parts` is `[(text, drop_rank[, short]), ...]` in display order; rank 0
    is never dropped, higher ranks go first. A rank-0 part may carry a SHORT
    form ("213/224" for "3D models 213/224"), tried before any word is cut,
    so the number a panel exists to show survives. Past that the result is
    `fit_words` -- never a string cut mid-word.
    """
    keep = [(p[0], p[1], p[2] if len(p) > 2 else None)
            for p in parts if p[0]]
    ranks = sorted({r for _t, r, _s in keep if r > 0}, reverse=True)
    for n_drop in range(len(ranks) + 1):
        gone = set(ranks[:n_drop])
        txt = sep.join(t for t, r, _s in keep if r not in gone)
        if txt and d.textlength(txt, font=font) <= width:
            return txt
    short = sep.join((s or t) for t, r, s in keep if r == 0)
    if short and d.textlength(short, font=font) <= width:
        return short
    return fit_words(d, short, font, width)


def draw_key_inline(d, box, rows, *, theme=None, font=None):
    """A key laid out on ONE line, right-aligned inside `box` (a
    `frame_layout.Box`). Returns True when it fit; a key that does not fit is
    not drawn, never overprinted."""
    if not rows or box is None or box.w <= 0 or box.h <= 0:
        return False
    try:
        import render_theme
        from route_render import load_font
        th = theme or render_theme.DARK
        font = font or load_font(max(9, int(box.h * 0.36)))
        sw = max(8, int(box.h * 0.36))
        items = [(rgb, mark, t, d.textlength(t, font=font))
                 for rgb, mark, t in rows]
        need = sum(sw + 5 + tw + 14 for _r, _m, _t, tw in items)
        if need > box.w:
            return False
        x = box.x + box.w - need
        cy = box.y + box.h // 2
        for rgb, mark, t, tw in items:
            # INTEGER corners: `draw_mark`'s dashed and hatched swatches
            # `range()` over the width, and a float width raised -- into the
            # except below, which dropped every key item after the first.
            xi = int(round(x))
            draw_mark(d, [xi, cy - sw // 2, xi + sw, cy + sw // 2], rgb, mark)
            d.text((x + sw + 5, cy), t, fill=th.rgb('chrome_text_dim'),
                   font=font, anchor='lm')
            x += sw + 5 + tw + 14
        return True
    except Exception:                                          # noqa: BLE001
        return False


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
              ticks=(), key_rows=None):
    """The stable strip: `left` at the left, `right` right-aligned, and an
    optional progress bar with tick marks where each lap began.

    `key_rows` (#946 review) is the event key, drawn on one line in the
    rail's free middle, left of `right`: the rail is chrome, so a key there
    can never cover the board -- which the corner overlay did."""
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
        if key_rows:
            lw = int(d.textlength(_fit(d, left, font, box.w - rw - 4 * pad),
                                  font=font))
            kx0 = box.x + pad + lw + 3 * pad
            kx1 = box.x + box.w - pad - rw - 3 * pad
            draw_key_inline(d, box._replace(x=kx0, w=max(0, kx1 - kx0),
                                            h=max(8, box.h - 4 * ss)),
                            key_rows, theme=th)
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
