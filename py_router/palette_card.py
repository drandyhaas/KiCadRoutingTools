#!/usr/bin/env python3
"""Draw a palette's measurements as a picture (#946).

`palette_audit` measures; this draws what it measured. Separate file for one
reason: **`palette_audit` must never import PIL, at any scope**, because
`py_tools/render_placement.py` imports the palette at module scope and has to
keep importing with ``sys.modules['PIL'] = None``
(``tests/test_943_optional_render_dependency.py:153-178``). Keeping the drawer
out of the measurer is what lets that stay true without a lazy-import dance
inside the module that matters.

**The point of this file is that the evidence is CODE, not a loose PNG.** #946
is a design-system change, so a claim about how a frame reads has to be shown
rather than asserted -- and an image committed once rots the moment the palette
moves. Regenerate it:

    python3 -X utf8 py_router/palette_card.py -o card.png

Each row draws its own swatch under a deuteranope simulation beside the true
one, because the argument this card exists to make cannot be made in the colour
channel it is arguing about.
"""
from __future__ import annotations

KRT_TOOL = {'scope': ['routing', 'placement'], 'kind': 'instrument'}

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import palette_audit as PA  # noqa: E402

# Card chrome, in the subject's own materials: these are `route_render`'s
# constants, so the card is drawn in the palette it is reporting on.
_BG = (14, 16, 18)
_PANEL = (21, 26, 23)
_RULE = (42, 50, 44)
_INK = (225, 225, 210)
_DIM = (154, 160, 147)
_FAINT = (107, 114, 105)
_GOLD = (192, 168, 96)      # _PAD -- the accent, chosen because it is neither
                            # red nor green, and this card must not make its
                            # argument in the channel it is arguing about.
_BAD = (255, 110, 110)
_OK = (120, 210, 170)


def _font(px):
    from route_render import load_font
    return load_font(px)


def _text(d, xy, s, px, fill, anchor=None):
    d.text(xy, s, font=_font(px), fill=fill, anchor=anchor)


def _swatch_row(d, x, y, w, h, rgb, label, sub, show_cvd=True):
    """One colour, twice: as authored and as a deuteranope sees it."""
    half = w // 2 if show_cvd else w
    d.rectangle([x, y, x + half - 1, y + h], fill=tuple(rgb))
    if show_cvd:
        d.rectangle([x + half, y, x + w, y + h], fill=PA.deuteranope(rgb))
        d.line([x + half, y, x + half, y + h], fill=_BG, width=1)
    lum = PA.relative_luminance(rgb)
    ink = (20, 20, 20) if lum > 0.35 else (240, 240, 240)
    _text(d, (x + 8, y + h // 2 - 8), label, 12, ink)
    if sub:
        _text(d, (x + w - 8, y + h // 2 - 7), sub, 11, ink, anchor='ra')


def draw_card(doc=None, width=1180, theme_name='dark'):
    """The whole measurement as one image, drawn in the arm it reports on."""
    from PIL import Image, ImageDraw
    import render_theme as RT

    th = RT.theme(theme_name)
    global _BG, _RULE, _INK, _DIM, _FAINT, _GOLD, _BAD, _OK
    _BG = th.rgb('ground')
    _RULE = th.rgb('chrome_rule')
    _INK = th.rgb('chrome_text') if theme_name == 'dark' else th.rgb('chrome_text')
    _DIM = th.rgb('chrome_text_dim')
    _FAINT = th.rgb('chrome_text_faint')
    _GOLD = th.rgb('pad')
    _BAD = th.rgb('event_ripped')
    _OK = th.rgb('status_kept')

    doc = PA.audit(theme_name) if doc is None else doc
    ev, st, rf = doc['events'], doc['structure'], doc['red_family']
    ly, cr = doc['layers'], doc['crossings']

    pal = PA.current_palette(theme_name)
    M, GAP = 34, 26
    colw = (width - M * 2 - GAP) // 2
    rowh, head = 34, 26

    # Draw onto a generous canvas and crop to what was ACTUALLY used. The two
    # columns hold different numbers of rows AND different numbers of heading
    # and note lines, so any arithmetic guess at the block height is a guess
    # that goes stale the moment a row is added -- the first version of this
    # card ran the left column straight into the layer strip below it.
    img = Image.new('RGB', (width, 2000), _BG)
    d = ImageDraw.Draw(img)

    # ---- masthead
    _text(d, (M, 30), 'Routing movie palette', 30, _INK)
    _text(d, (M, 68), 'the %s arm, measured off the source' % doc['palette'],
          14, _DIM)
    _text(d, (width - M, 34),
          'left half: as authored     right half: deuteranope (Vienot 1999)',
          12, _FAINT, anchor='ra')
    _text(d, (width - M, 54), 'py_router/palette_audit.py', 12, _GOLD,
          anchor='ra')
    d.line([M, 96, width - M, 96], fill=_RULE, width=1)

    y0 = 112
    lx, rx = M, M + colw + GAP

    # ---- left: the events, and the red family
    _text(d, (lx, y0), 'THE THREE EVENTS', 12, _GOLD)
    y = y0 + head
    for role in PA.EVENT_ROLES:
        _swatch_row(d, lx, y, colw, rowh, pal[role],
                    role.replace('event_', ''),
                    '%.2fx vs board' % ev['contrast_vs_board'][role])
        y += rowh + 4
    y += 10
    worst_pre = rf['min']
    _text(d, (lx, y), 'RIP vs RESTORE', 12, _GOLD)
    _text(d, (lx + 150, y),
          '%.0f apart normally,  %.0f under deuteranopia'
          % (ev['pairs']['event_restored|event_ripped']['normal'],
             ev['rip_restore_deuteranope']), 12,
          _BAD if ev['rip_restore_deuteranope'] < 120 else _OK)
    y += head + 4

    _text(d, (lx, y),
          'RED MEANS FOUR THINGS' if worst_pre < 40 else
          'FOUR MEANINGS, FOUR COLOURS', 12, _GOLD)
    y += head
    meanings = {
        'event_ripped': 'copper the router DESTROYED',
        'defect_conflict': 'a pad/hole conflict -- the BOARD is wrong',
        'defect_net_fail': 'a net that failed to route',
        'status_tried': 'an attempt that was not kept',
    }
    for role in PA.RED_FAMILY:
        _swatch_row(d, lx, y, colw, rowh, pal[role], meanings[role], '',
                    show_cvd=False)
        y += rowh + 4
    worst = rf['min']
    note = ('closest pair: %.1f apart  -- indistinguishable, and opposite in '
            'kind' % worst) if worst < 40 else (
        'closest pair: %.1f apart  -- #1012 took this from 2.8; red now means '
        'ONE thing' % worst)
    _text(d, (lx, y + 6), note, 12, _BAD if worst < 40 else _OK)
    y_left = y + 6 + 18

    # ---- right: structure tokens, on both grounds
    _text(d, (rx, y0), 'STRUCTURE, ON EACH GROUND', 12, _GOLD)
    _text(d, (rx, y0 + 16),
          'contrast against the board body it is drawn on', 11, _FAINT)
    y = y0 + head + 16
    other = RT.theme('light' if theme_name == 'dark' else 'dark')
    cols = ((('%s %s' % (th.name, th.rgb('board_body'))), th.rgb('board_body')),
            (('%s %s' % (other.name, other.rgb('board_body'))),
             other.rgb('board_body')))
    cw = (colw - 150) // 2
    _text(d, (rx + 150, y - 16), cols[0][0], 11, _DIM)
    _text(d, (rx + 150 + cw + 10, y - 16), cols[1][0], 11, _DIM)
    for role in ('edge', 'pad', 'via', 'pad_hole'):
        rgb = pal[role]
        _text(d, (rx, y + 9), role, 12, _INK)
        for k, (_lbl, body) in enumerate(cols):
            x = rx + 150 + k * (cw + 10)
            d.rectangle([x, y, x + cw, y + rowh], fill=body)
            d.rectangle([x + 10, y + 9, x + cw - 56, y + rowh - 9],
                        fill=tuple(rgb))
            c = PA.contrast_ratio(rgb, body)
            ink = (20, 20, 20) if PA.relative_luminance(body) > 0.35 \
                else (235, 235, 235)
            _text(d, (x + cw - 8, y + 10), '%.2fx' % c, 11,
                  _BAD if c < 3.0 else ink, anchor='ra')
        y += rowh + 4
    _text(d, (rx, y + 8),
          'this arm draws its OWN structure tokens; the right column shows '
          'them on the other ground', 12, _DIM)
    _text(d, (rx, y + 26),
          'edge %.2fx here -- #946 never measured these'
          % PA.contrast_ratio(pal['edge'], th.rgb('board_body')), 12, _DIM)
    y_right = y + 26 + 18

    # ---- the layer strip, as rendered
    ys = max(y_left, y_right) + 24
    d.line([M, ys, width - M, ys], fill=_RULE, width=1)
    ys += 22
    _text(d, (M, ys), 'THE TEN LAYERS, AS RENDERED AT ALPHA %d' % ly['alpha'],
          12, _GOLD)
    _text(d, (width - M, ys),
          'closest pair %.1f (%s)   %d crossings impersonate a third layer'
          % (ly['closest_pair'], ly['closest_pair_names'], cr['count']),
          12, _BAD, anchor='ra')
    ys += head
    n = len(pal['layers'])
    sw = (width - M * 2 - (n - 1) * 6) // n
    for i, c in enumerate(pal['layers']):
        x = M + i * (sw + 6)
        rendered = PA.composite(c, pal['board_body'], ly['alpha'])
        d.rectangle([x, ys, x + sw, ys + 46], fill=rendered)
        d.rectangle([x, ys + 46, x + sw, ys + 78],
                    fill=PA.deuteranope(rendered))
        _text(d, (x + 6, ys + 6), PA.LAYER_NAMES[i], 11, (235, 235, 235))
    ys += 86
    w = cr['worst']
    if w:
        _text(d, (M, ys),
              'worst: %s over %s renders %s and reads as %s %s -- %.1f apart'
              % (w['over'], w['under'], tuple(w['renders']), w['reads_as'],
                 tuple(ly['rendered'][w['reads_as']]), w['distance']),
              12, _BAD)
        ys += 20
    return img.crop((0, 0, width, ys + 22))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Draw the palette measurements as a picture (#946).')
    ap.add_argument('-o', '--output', default='palette_card.png')
    ap.add_argument('--width', type=int, default=1180)
    ap.add_argument('--theme', default='dark')
    a = ap.parse_args(argv)
    if PA.self_test(quiet=True):
        return 1
    img = draw_card(width=a.width, theme_name=a.theme)
    os.makedirs(os.path.dirname(os.path.abspath(a.output)) or '.',
                exist_ok=True)
    img.save(a.output)
    print('wrote %s  (%dx%d)' % (a.output, img.width, img.height))
    return 0


if __name__ == '__main__':
    sys.exit(main())
