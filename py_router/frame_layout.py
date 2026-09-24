#!/usr/bin/env python3
"""Where everything in a frame goes, computed ONCE per film (#946, #1018).

`render_theme` plans WHAT COLOUR; this plans WHERE. Pure integer arithmetic
over stdlib -- no PIL, no board parser, no Image -- so a whole layout can be
checked as data before a pixel exists, the same posture as
`movie_camera.plan_shots` and `movie_panels.plan_iso_shots`.

**THE INVARIANT.** Every frame handed to `animate_route.save_movie` must be the
same size, and the reason is worse than an exception: **NOTHING RAISES.**
Measured on this repo's Pillow, saving a GIF whose frames differ in size does
not raise -- Pillow writes a valid file in which every later frame has been
SILENTLY RESIZED to the first (`movie_panels.py:19-33`). `_write_mp4` does fail
loudly, and falls back to the GIF that absorbs it without a word.
`tests/test_431_placement_movie.py:207-212` calls
`len({f.size for f in frames}) == 1` "the single highest-value assertion here".

This module is how that invariant becomes structural rather than hoped-for:
`plan_frame()` is called ONCE, before the first frame, and its `FrameGeometry`
is the only source of any size thereafter.

**BOTH DIMENSIONS ARE FORCED EVEN.** `movie_panels.panel_geometry` forced only
the height, and its docstring cites only the height crop -- but
`_write_mp4` does `h, wd = a.shape[0] & ~1, a.shape[1] & ~1`, BOTH dimensions.
Frame width is `round(size * bw / bh)` for a taller-than-wide board and can land
odd, so a tall board's mp4 has been silently losing a pixel column in every
movie this repo has ever written.

**C VS D IS A STANCE, NOT AN INFERENCE.** Measured at equal total pixel budget,
the two quality metrics never agree: px/mm on the copper and px per layer cell
pick different winners on every board shape. C (Inset) wins copper everywhere,
by +16% to +44%; D (Split) wins panel everywhere, 128 800 against C's 28 490.
A and B genuinely swap by board shape, by 8.8-23.8%, which is the ONLY part worth
automating -- so `'auto'` picks between A and B from `board_bounds`, and nothing
picks between C and D. A measure that never changes its mind is not measuring
the decision.

**`'legacy'` IS THE DEFAULT** and reproduces today's frame exactly. Making
`'auto'` the default would change the shape of every existing artifact --  the
GUI recorder's, `place_route_loop`'s `placement.mp4`, `render_run`'s. The repo
has already ruled on this trade for `KICAD_MOVIE_CAMERA`: "'off' (default)
keeps every existing movie bit-for-bit".
"""
from __future__ import annotations

import math
import sys
from typing import Dict, NamedTuple, Optional, Sequence, Tuple

#: Chrome, as fractions of FRAME height. A rail at the top (board, lap, phase)
#: and a foot at the bottom (key, totals).
RAIL_FRAC = 0.045
FOOT_FRAC = 0.045

#: Floors, because a fraction of a small frame is smaller than the text the
#: region has to hold. Measured: at 236 px tall, 4.5% is 10 px.
RAIL_MIN_PX = 22
FOOT_MIN_PX = 26

#: Above this board aspect (w/h) the sidebar beats the stack. Measured: B wins
#: on wide and 4:3 boards, A on square and tall, by 8.8-23.8%. The cut is
#: where the measurement puts the crossover -- `py_router/layout_budget.py` computes these; `tests/test_946_layout_budget.py` pins them.
ADAPTIVE_ASPECT_CUT = 1.25

#: Outside this band `'auto'` gives up its chrome and returns `'legacy'`.
#: The bounds are where the board stops filling half of the better of the two
#: adaptive boxes: `sidebar`'s box is ~1.53:1 and `stacked`'s ~0.98:1, so a
#: board narrower than ~0.50 or wider than ~3.00 fills under half of whichever
#: it would be given. Derived from the boxes rather than chosen -- see
#: `resolve_layout` for the measurement, and `layout_budget.py` for the
#: instrument that produces the box figures.
EXTREME_ASPECT_LO = 0.50
EXTREME_ASPECT_HI = 3.00

STACKED_PANEL_FRAC = 0.28           # of frame HEIGHT           (A)
SIDEBAR_BOARD_FRAC = 0.73           # of frame WIDTH            (B)
INSET_PANEL_FRAC = (0.30, 0.26)     # of frame W, H             (C)
SPLIT_PANEL_FRAC = 0.32             # of frame HEIGHT           (D)
SPLIT_ISO_FRAC = 0.42               # of the split box's WIDTH  (D)


class Box(NamedTuple):
    x: int
    y: int
    w: int
    h: int

    def contains(self, other: 'Box') -> bool:
        return (other.x >= self.x and other.y >= self.y
                and other.x + other.w <= self.x + self.w
                and other.y + other.h <= self.y + self.h)

    def overlaps(self, other: 'Box') -> bool:
        return not (other.x >= self.x + self.w or self.x >= other.x + other.w
                    or other.y >= self.y + self.h
                    or self.y >= other.y + other.h)


class LayoutSpec(NamedTuple):
    key: str
    letter: str
    title: str
    #: None = take the BOARD's own aspect (C, and 'legacy')
    aspect: Optional[float]
    panel: Optional[str]        # 'below' | 'right' | 'inset' | 'split' | None
    overlays_board: bool


LAYOUTS: Dict[str, LayoutSpec] = {
    'legacy':  LayoutSpec('legacy', '-', 'as shipped', None, 'below', False),
    'stacked': LayoutSpec('stacked', 'A', 'Stacked', 1000.0 / 1620.0,
                          'below', False),
    'sidebar': LayoutSpec('sidebar', 'B', 'Sidebar', 16.0 / 9.0, 'right',
                          False),
    'inset':   LayoutSpec('inset', 'C', 'Inset', None, 'inset', True),
    'split':   LayoutSpec('split', 'D', 'Split panel', 16.0 / 10.0, 'split',
                          False),
    'auto':    LayoutSpec('auto', 'E', 'Adaptive A/B', None, None, False),
}

#: Named target aspects. `'board'` is today's behaviour and the default.
RATIOS: Dict[str, Optional[float]] = {
    'board': None,
    'stacked': 1000.0 / 1620.0,
    '16:9': 16.0 / 9.0,
    '16:10': 16.0 / 10.0,
    '4:3': 4.0 / 3.0,
    '1:1': 1.0,
    '9:16': 9.0 / 16.0,
}


class FrameSizeError(ValueError):
    """Frames handed to the encoder are not all the same size."""


class FrameGeometry(NamedTuple):
    layout: str                 # RESOLVED: 'auto' is already A or B here
    requested_layout: str
    chosen_by: str              # 'flag' | 'adaptive: board aspect 1.41 > 1.25'
    aspect: float
    frame: Box                  # w and h BOTH EVEN
    board: Box
    panel: Optional[Box]
    panel_split: Optional[Tuple[Box, Box]]   # layout D: (iso, layer strip)
    rail: Box
    foot: Box
    track: Optional[Box]        # the attempts band (#1021), when asked for
    overlays_board: bool


def even(n) -> int:
    """Nearest even integer at or below `n`, floored at 2."""
    return max(2, int(n) & ~1)


def resolve_layout_aspect(layout=None, aspect=None):
    """`(layout, aspect)` for a render: an explicit argument wins, and `None`
    falls back to `$KICAD_MOVIE_LAYOUT` / `$KICAD_MOVIE_ASPECT` (env_knobs),
    then to `'legacy'` / the board's own aspect.

    The ONE resolution, for make_movie and make_film's build_film alike. It
    lived inline in make_movie only, so a film -- the render that actually
    shows placement -- ignored both knobs while make_movie's `--help`
    advertised them."""
    if layout is None or aspect is None:
        try:
            import env_knobs as _ek
        except Exception:                                       # noqa: BLE001
            _ek = None
        if layout is None:
            layout = getattr(_ek, 'MOVIE_LAYOUT', 'legacy')
        if aspect is None:
            aspect = getattr(_ek, 'MOVIE_ASPECT', '') or None
    return layout, aspect


def parse_ratio(text) -> Optional[float]:
    """`'16:9'`, `'16/9'`, `1.78`, or a RATIOS key. `None` = the board's own."""
    if text is None or text == '':
        return None
    if isinstance(text, (int, float)):
        return float(text) or None
    key = str(text).strip().lower()
    if key in RATIOS:
        return RATIOS[key]
    for sep in (':', '/', 'x'):
        if sep in key:
            a, _, b = key.partition(sep)
            try:
                w, h = float(a), float(b)
            except ValueError:
                break
            if w > 0 and h > 0:
                return w / h
            break
    try:
        v = float(key)
    except ValueError:
        raise ValueError('frame_layout: %r is not a ratio. Give W:H, a number, '
                         'or one of %s' % (text, ', '.join(sorted(RATIOS))))
    if v <= 0:
        raise ValueError('frame_layout: ratio %r must be positive' % text)
    return v


def resolve_layout(name, board_bounds, *, quiet=False) -> Tuple[str, str]:
    """`(resolved_key, why)`. Only `'auto'` consults the board."""
    key = (name or 'legacy')
    if isinstance(key, str):
        key = key.strip().lower()
    if key not in LAYOUTS:
        raise ValueError('frame_layout: %r is not one of %s'
                         % (name, ', '.join(LAYOUTS)))
    if key != 'auto':
        return key, 'flag'
    if not board_bounds:
        return 'stacked', 'adaptive: no board bounds, so the stack'
    min_x, min_y, max_x, max_y = board_bounds
    bw = max(max_x - min_x, 1e-6)
    bh = max(max_y - min_y, 1e-6)
    a = bw / bh
    # AN EXTREME BOARD GETS NO CHROME AT ALL, and that is an inference from
    # `board_bounds` like the other one rather than a new kind of decision.
    #
    # Every chrome layout has a FIXED board-box aspect; only `legacy` inherits
    # the board's. So a board far outside the corpus range fills very little of
    # whichever box it is given, and the adaptive cut -- tuned on aspects
    # 0.5..2.5 -- happily picks the SECOND WORST option for it. Measured on a
    # 6.5:1 board (splitflap_driver) at size 560:
    #
    #     legacy   board box 6.51 aspect   board fills 100%
    #     split    2.95                                 45%
    #     inset    14.74                                44%
    #     sidebar  1.53                                 24%   <- what auto chose
    #     stacked  0.98                                 15%
    #
    # and in a real placement film that showed up as the board holding
    # 4.6-4.9% of the frame during the beats where parts were moving -- the
    # camera zoomed IN and the subject got SMALLER.
    #
    # Outside the band, the honest answer is the layout whose box IS the
    # board: no rail, no lower box, and all of the frame for the thing the
    # film is about. Chrome you cannot afford is not a feature.
    if a < EXTREME_ASPECT_LO or a > EXTREME_ASPECT_HI:
        return 'legacy', ('adaptive: board aspect %.2f is outside %.2f..%.2f, '
                          'where every chrome box wastes most of the frame'
                          % (a, EXTREME_ASPECT_LO, EXTREME_ASPECT_HI))
    if a > ADAPTIVE_ASPECT_CUT:
        return 'sidebar', ('adaptive: board aspect %.2f > %.2f'
                           % (a, ADAPTIVE_ASPECT_CUT))
    return 'stacked', ('adaptive: board aspect %.2f <= %.2f'
                       % (a, ADAPTIVE_ASPECT_CUT))


def plan_frame(board_bounds, *, layout='legacy', ratio=None, size=1000,
               panel=False, foot_px=0, track_px=0,
               rail_frac=RAIL_FRAC, foot_frac=FOOT_FRAC,
               legacy_size=None, quiet=False) -> FrameGeometry:
    """The whole frame, decided ONCE.

    `size` is the longest dimension, as everywhere else in this subsystem.
    `panel` asks for a lower/side box; `foot_px` is a MEASURED height (the run
    clock's band) passed IN, so this module never needs PIL to measure text.
    `legacy_size` is the `(W, H)` the board-aspect path already produced, so
    `'legacy'` can reproduce it exactly rather than recompute it.
    """
    key, why = resolve_layout(layout, board_bounds, quiet=quiet)
    spec = LAYOUTS[key]

    # An explicit ratio always wins: `legacy_size` is a shortcut for
    # reproducing today's frame EXACTLY, and asking for a ratio is asking
    # for something other than today's frame.
    if key == 'legacy' and legacy_size and not ratio:
        W, H = int(legacy_size[0]), int(legacy_size[1])
        aspect = (W / float(H)) if H else 1.0
    else:
        aspect = ratio if ratio else spec.aspect
        if aspect is None:
            if board_bounds:
                min_x, min_y, max_x, max_y = board_bounds
                aspect = (max(max_x - min_x, 1e-6)
                          / max(max_y - min_y, 1e-6))
            else:
                aspect = 1.0
        if aspect >= 1.0:
            W, H = size, max(1, int(round(size / aspect)))
        else:
            W, H = max(1, int(round(size * aspect))), size

    H += int(foot_px) + int(track_px)
    # BOTH dimensions, see the module docstring.
    W, H = even(W), even(H)
    aspect = W / float(H)

    # 'legacy' means TODAY'S FRAME, and today's frame has no rail and no foot
    # -- the caption is stamped over the board by `_label`, not into reserved
    # chrome. Reserving any here would shrink the board box and stop legacy
    # being the bit-for-bit reproduction it exists to be.
    if key == 'legacy':
        rail_frac = foot_frac = 0.0
    # FLOORED at a legible height. 4.5% of a 236 px frame is 10 px, which is
    # smaller than the text it has to hold -- a reserved region too small for
    # its content is the same defect as a strip too narrow for its fields,
    # only quieter.
    rail_h = max(RAIL_MIN_PX, even(H * rail_frac)) if rail_frac else 0
    foot_h = max(FOOT_MIN_PX, even(H * foot_frac)) if foot_frac else 0
    band_h = even(foot_px) if foot_px else 0
    track_h = even(track_px) if track_px else 0
    inner_y = rail_h
    inner_h = max(2, H - rail_h - foot_h - band_h - track_h)

    panel_box = None
    split = None
    if not panel or spec.panel is None:
        board = Box(0, inner_y, W, inner_h)
    elif spec.panel == 'below':
        ph = even(H * STACKED_PANEL_FRAC)
        board = Box(0, inner_y, W, max(2, inner_h - ph))
        panel_box = Box(0, inner_y + board.h, W, ph)
    elif spec.panel == 'right':
        bw = even(W * SIDEBAR_BOARD_FRAC)
        board = Box(0, inner_y, bw, inner_h)
        panel_box = Box(bw, inner_y, W - bw, inner_h)
    elif spec.panel == 'inset':
        board = Box(0, inner_y, W, inner_h)
        pw = even(W * INSET_PANEL_FRAC[0])
        ph = even(inner_h * INSET_PANEL_FRAC[1])
        panel_box = Box(W - pw - 6, inner_y + inner_h - ph - 6, pw, ph)
    elif spec.panel == 'split':
        ph = even(H * SPLIT_PANEL_FRAC)
        board = Box(0, inner_y, W, max(2, inner_h - ph))
        panel_box = Box(0, inner_y + board.h, W, ph)
        iw = even(W * SPLIT_ISO_FRAC)
        split = (Box(0, panel_box.y, iw, ph),
                 Box(iw, panel_box.y, W - iw, ph))
    else:
        board = Box(0, inner_y, W, inner_h)

    y = inner_y + inner_h
    track_box = Box(0, y, W, track_h) if track_h else None
    y += track_h
    foot_box = Box(0, y, W, foot_h + band_h)

    geom = FrameGeometry(
        layout=key, requested_layout=str(layout or 'legacy'), chosen_by=why,
        aspect=aspect, frame=Box(0, 0, W, H), board=board, panel=panel_box,
        panel_split=split, rail=Box(0, 0, W, rail_h), foot=foot_box,
        track=track_box, overlays_board=spec.overlays_board)
    _self_check(geom)
    return geom


def _self_check(g: FrameGeometry) -> None:
    """Every named box inside the frame, and (unless the layout says so) not
    on top of the board. Cheap, and it turns a layout arithmetic slip into a
    refusal here rather than a distorted film later."""
    for name in ('board', 'rail', 'foot', 'panel', 'track'):
        b = getattr(g, name)
        if b is None or b.w <= 0 or b.h <= 0:
            continue
        if not g.frame.contains(b):
            raise FrameSizeError('layout %r: %s %s is outside the frame %s'
                                 % (g.layout, name, tuple(b), tuple(g.frame)))
    if g.panel is not None and not g.overlays_board:
        if g.board.overlaps(g.panel):
            raise FrameSizeError('layout %r: the panel %s overlaps the board '
                                 '%s but this layout does not overlay'
                                 % (g.layout, tuple(g.panel), tuple(g.board)))
    if g.frame.w % 2 or g.frame.h % 2:
        raise FrameSizeError('layout %r: frame %dx%d is not even on both axes '
                             '-- _write_mp4 crops with & ~1 on BOTH'
                             % (g.layout, g.frame.w, g.frame.h))


def assert_frames_uniform(sizes: Sequence[Tuple[int, int]],
                          expect: Optional[Box] = None) -> None:
    """Raise `FrameSizeError` unless every size is the same (and `expect`'s).

    Takes SIZE TUPLES, not images, so this module stays PIL-free. Wired into
    `animate_route.save_movie`, the one choke point `make_movie`,
    `make_film.build_film`, `animate_fanout_clearance.render_gif` and
    `tests/stress/render_run.py` all pass through.
    """
    uniq = []
    for s in sizes:
        t = (int(s[0]), int(s[1]))
        if t not in uniq:
            uniq.append(t)
    if len(uniq) > 1:
        first = None
        for i, s in enumerate(sizes):
            if first is None:
                first = (int(s[0]), int(s[1]))
            elif (int(s[0]), int(s[1])) != first:
                raise FrameSizeError(
                    'frame %d is %dx%d but frame 0 is %dx%d -- Pillow will NOT '
                    'raise on this, it will silently resize every later frame '
                    'to the first' % (i, s[0], s[1], first[0], first[1]))
    if expect is not None and uniq and uniq[0] != (expect.w, expect.h):
        raise FrameSizeError('frames are %dx%d but the layout planned %dx%d'
                             % (uniq[0][0], uniq[0][1], expect.w, expect.h))


def frame_status_line(geom: FrameGeometry) -> str:
    """One line saying which layout ran and why. Modelled on
    `movie_panels.iso_status_line`: none of the states may read like silence."""
    spec = LAYOUTS.get(geom.layout)
    title = spec.title if spec else geom.layout
    letter = (' (%s)' % spec.letter) if spec and spec.letter != '-' else ''
    line = ('movie: layout %s%s  %dx%d at %.2f:1, board %dx%d'
            % (title, letter, geom.frame.w, geom.frame.h, geom.aspect,
               geom.board.w, geom.board.h))
    if geom.requested_layout != geom.layout:
        line += '  |  %s -> %s' % (geom.requested_layout, geom.chosen_by)
    return line
