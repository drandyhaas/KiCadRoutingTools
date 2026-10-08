#!/usr/bin/env python3
"""Where everything in a frame goes, computed ONCE per film (#946, #1018, #1081).

`render_theme` plans WHAT COLOUR; this plans WHERE. Pure integer arithmetic
over stdlib -- no PIL, no board parser, no Image -- so a whole frame can be
checked as data before a pixel exists, the same posture as
`movie_camera.plan_shots`.

**ONE LAYOUT: `stage3d`** (#1081, and the only one since the others were
retired). The board box -- the 3D board when this machine can render it, else
the 2D X-ray -- gets at least `STAGE3D_BOARD_W_FRAC` x `STAGE3D_BOARD_H_FRAC`
of the frame; the layer column sits beside it (a row under it on a portrait
frame), a rail runs along the top, a foot along the bottom, and one benchmark
band above the foot. The frame is 16:9 unless an aspect is declared. The
legacy board-aspect frame and the stacked / sidebar / inset / split / auto
layouts are gone, and so is the `layout` argument.

**THE INVARIANT.** Every frame handed to `animate_route.save_movie` must be the
same size, and the reason is worse than an exception: **NOTHING RAISES.**
Measured on this repo's Pillow, saving a GIF whose frames differ in size does
not raise -- Pillow writes a valid file in which every later frame has been
SILENTLY RESIZED to the first. `_write_mp4` does fail
loudly, and falls back to the GIF that absorbs it without a word.
`tests/test_431_placement_movie.py` calls
`len({f.size for f in frames}) == 1` "the single highest-value assertion here".

This module is how that invariant becomes structural rather than hoped-for:
`plan_frame()` is called ONCE, before the first frame, and its `FrameGeometry`
is the only source of any size thereafter.

**BOTH DIMENSIONS ARE FORCED EVEN.** `_write_mp4` does
`h, wd = a.shape[0] & ~1, a.shape[1] & ~1`, BOTH dimensions, so an odd frame
silently loses a row or a column in the encoder.
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

#: Outside this band of DECLARED frame aspects a 70 x 70 board box and a
#: layer column (or row) cannot both fit, so the frame is BOARD-ONLY: still
#: stage3d, at the ratio asked for, with its rail and foot, but the board
#: box takes the whole width, there is no layer column, the benchmark band
#: is drawn only if the board keeps its height floor, and
#: `FrameGeometry.notes` says so.
EXTREME_ASPECT_LO = 0.50
EXTREME_ASPECT_HI = 3.00

#: At or above this frame aspect the frame is LANDSCAPE: the layer column
#: sits beside the board. Below it, it becomes a row under the board.
ISO_SIDE_ASPECT = 1.25

#: The frame's aspect when none is declared.
STAGE3D_ASPECT = 16.0 / 9.0
#: The 3D board is the film, so its box gets AT LEAST this share of the
#: frame's width and of its height -- the rail, the foot, the clock and the
#: benchmark band all fit in the rest, and a band that would push the board
#: under the floor is shrunk, then declined, never the board.
#: `movie_placement.plan_band` sizes the placement panels' band under the
#: same height floor.
STAGE3D_BOARD_W_FRAC = 0.70
STAGE3D_BOARD_H_FRAC = 0.70
#: A benchmark band shorter than this cannot hold its curve and its labels;
#: below it the band is declined, and the status line says so.
STAGE3D_BAND_MIN_PX = 64
#: A PORTRAIT stage3d frame (aspect below `ISO_SIDE_ASPECT`) has no width
#: for a side column, so the layer column becomes a ROW under the board --
#: dropped, and said, when it would be shorter than this. Not 40: a 52 px
#: row at 9:16 drew six 8 px-wide cells with their names cut to "In" (the
#: phase-3 verifier's render) -- a row too short to read is not a row.
STAGE3D_ROW_MIN_PX = 90


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


#: Named target aspects. `'board'` declares nothing: the frame's own 16:9.
RATIOS: Dict[str, Optional[float]] = {
    'board': None,
    '16:9': 16.0 / 9.0,
    '16:10': 16.0 / 10.0,
    '4:3': 4.0 / 3.0,
    '1:1': 1.0,
    '9:16': 9.0 / 16.0,
}


#: The retired film layouts' names. One arriving where the ASPECT goes --
#: `--aspect stacked` was the stacked layout's own ratio, and a script or a
#: shell may still hand a layout name to `--aspect` / `$KICAD_MOVIE_ASPECT`
#: -- is said once as retired and declares nothing: the frame's own 16:9.
RETIRED_LAYOUTS = ('legacy', 'stacked', 'sidebar', 'inset', 'split', 'auto')


class FrameSizeError(ValueError):
    """Frames handed to the encoder are not all the same size."""


class FrameGeometry(NamedTuple):
    layout: str                 # always 'stage3d', the only layout
    aspect: float
    frame: Box                  # w and h BOTH EVEN
    board: Box
    panel: Optional[Box]        # the layer column (or row); None = dropped
    rail: Box
    foot: Box
    track: Optional[Box]        # the benchmark band, when asked for
    #: What the frame had to give up to keep its promise, in words for
    #: `frame_status_line` (a declined band, a dropped layer row, a frame
    #: too extreme to hold a column). Empty = nothing given up.
    notes: Tuple[str, ...] = ()


def even(n) -> int:
    """Nearest even integer at or below `n`, floored at 2."""
    return max(2, int(n) & ~1)


#: The film's layout, and its only one (#1081): `FrameGeometry.layout`.
DEFAULT_FILM_LAYOUT = 'stage3d'


_RETIRED_SAID = []


def warn_retired_knobs():
    """ONE stderr line, once per process, naming every retired movie knob
    still set in the environment (`env_knobs.MOVIE_RETIRED`). A retired
    knob selects nothing, and a value that is ignored without a word reads
    as a feature that broke. Returns the line, or '' when there is none."""
    try:
        import env_knobs as _ek
        retired = dict(getattr(_ek, 'MOVIE_RETIRED', {}) or {})
    except Exception:                                           # noqa: BLE001
        retired = {}
    if not retired or _RETIRED_SAID:
        return ''
    line = ('movie: %s %s retired -- stage3d is the only film layout, so '
            'ignored' % (', '.join('%s=%s' % kv for kv in sorted(
                retired.items())), 'is' if len(retired) == 1 else 'are'))
    _RETIRED_SAID.append(line)
    print(line, file=sys.stderr)
    return line


_RETIRED_ASPECT_SAID = set()


def warn_retired_aspect(name):
    """ONE stderr line per retired layout name given as an aspect, in
    `warn_retired_knobs`' words. Returns the line, or '' once said."""
    if name in _RETIRED_ASPECT_SAID:
        return ''
    line = ("movie: aspect '%s' is retired -- stage3d is the only film "
            "layout, so ignored (the frame's own 16:9)" % name)
    print(line, file=sys.stderr)
    _RETIRED_ASPECT_SAID.add(name)
    return line


def resolve_aspect(aspect=None):
    """The film's declared aspect: an explicit argument wins, and `None`
    falls back to `$KICAD_MOVIE_ASPECT` (env_knobs), then to None -- the
    frame's own 16:9.

    The ONE resolution, for make_movie and make_film's build_film alike. A
    retired knob still set in the environment is said here, once."""
    warn_retired_knobs()
    if aspect is None:
        try:
            import env_knobs as _ek
        except Exception:                                       # noqa: BLE001
            _ek = None
        aspect = getattr(_ek, 'MOVIE_ASPECT', '') or None
    return aspect


def parse_ratio(text) -> Optional[float]:
    """`'16:9'`, `'16/9'`, `1.78`, or a RATIOS key. `None` = nothing
    declared: the frame's own `STAGE3D_ASPECT`. A `RETIRED_LAYOUTS` name is
    said (`warn_retired_aspect`) and declares nothing; anything else that is
    not a ratio raises."""
    if text is None or text == '':
        return None
    if isinstance(text, (int, float)):
        return float(text) or None
    key = str(text).strip().lower()
    if key in RATIOS:
        return RATIOS[key]
    if key in RETIRED_LAYOUTS:
        warn_retired_aspect(key)
        return None
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


def plan_frame(board_bounds=None, *, ratio=None, size=1000, foot_px=0,
               track_px=0, rail_frac=RAIL_FRAC, foot_frac=FOOT_FRAC,
               quiet=False) -> FrameGeometry:
    """The whole stage3d frame, decided ONCE.

    `size` is the longest dimension, as everywhere else in this subsystem.
    `ratio` is the declared aspect (`parse_ratio`); None is 16:9. The frame
    no longer depends on the board: `board_bounds` is accepted for the
    callers that pass it and not read. `foot_px` is a MEASURED height (the
    run clock's band) passed IN, so this module never needs PIL to measure
    text; `track_px` asks for the benchmark band.

    **A DECLARED SIZE IS KEPT (#946/C4).** `foot_px` and `track_px` are
    reserved INSIDE the frame, out of the board's share -- they used to be
    ADDED to it, so `--aspect 16:9` with a band came out 1600x1036 rather
    than 1600x900.
    """
    notes = []
    fa = ratio if ratio else STAGE3D_ASPECT
    # A declared frame this far from square cannot hold a 70 x 70 board box
    # AND a column or row beside it. It stays a stage3d frame at the ratio
    # asked for -- rail, foot, and the band only if it fits -- but the board
    # box takes the whole width, there is no layer column, and it is said.
    extreme = not (EXTREME_ASPECT_LO <= fa <= EXTREME_ASPECT_HI)
    if extreme:
        notes.append('stage3d: frame aspect %.2f is outside %.2f..%.2f, '
                     'so no layer column -- the board box takes the '
                     'frame' % (fa, EXTREME_ASPECT_LO, EXTREME_ASPECT_HI))
    if fa >= 1.0:
        W, H = size, max(1, int(round(size / fa)))
    else:
        W, H = max(1, int(round(size * fa))), size
    # BOTH dimensions, see the module docstring.
    W, H = even(W), even(H)
    aspect = W / float(H)

    # FLOORED at a legible height. 4.5% of a 236 px frame is 10 px, which is
    # smaller than the text it has to hold -- a reserved region too small for
    # its content is the same defect as a strip too narrow for its fields,
    # only quieter.
    rail_h = max(RAIL_MIN_PX, even(H * rail_frac)) if rail_frac else 0
    foot_h = max(FOOT_MIN_PX, even(H * foot_frac)) if foot_frac else 0
    band_h = even(foot_px) if foot_px else 0
    track_h = even(track_px) if track_px else 0
    track_h, why_band = _stage3d_band(H, rail_h, foot_h, band_h, track_h)
    if why_band:
        notes.append(why_band)
    inner_y = rail_h
    inner_h = max(2, H - rail_h - foot_h - band_h - track_h)

    if extreme:
        board, panel_box = Box(0, inner_y, W, inner_h), None
    else:
        board, panel_box, why_col = _stage3d_boxes(W, H, inner_y, inner_h)
        if why_col:
            notes.append(why_col)
    # The floor is a PROMISE only a frame big enough can keep. A tiny frame,
    # or a clock band taller than the frame can spare, breaks it -- and says
    # so rather than shipping a smaller board quietly.
    if (board.h < STAGE3D_BOARD_H_FRAC * H
            or (W >= ISO_SIDE_ASPECT * H
                and board.w < STAGE3D_BOARD_W_FRAC * W)):
        notes.append('stage3d: the frame is too small for the %d%% x '
                     '%d%% board (%dx%d of %dx%d)'
                     % (round(100 * STAGE3D_BOARD_W_FRAC),
                        round(100 * STAGE3D_BOARD_H_FRAC),
                        board.w, board.h, W, H))

    y = inner_y + inner_h
    track_box = Box(0, y, W, track_h) if track_h else None
    y += track_h
    foot_box = Box(0, y, W, foot_h + band_h)

    geom = FrameGeometry(
        layout=DEFAULT_FILM_LAYOUT, aspect=aspect, frame=Box(0, 0, W, H),
        board=board, panel=panel_box, rail=Box(0, 0, W, rail_h),
        foot=foot_box, track=track_box, notes=tuple(notes))
    _self_check(geom)
    return geom


def _up_even(n) -> int:
    """Smallest even integer at or above `n`."""
    v = int(math.ceil(n))
    return v + (v % 2)


def _stage3d_band(H, rail_h, foot_h, band_h, track_h):
    """`(track_h, why)`: the benchmark band's height, capped so the board
    keeps `STAGE3D_BOARD_H_FRAC` of the frame; `why` names a shrink or a
    decline, None when the band fit as asked."""
    if not track_h:
        return 0, None
    need = _up_even(STAGE3D_BOARD_H_FRAC * H)
    cap = H - rail_h - foot_h - band_h - need
    if cap < STAGE3D_BAND_MIN_PX:
        return 0, ('stage3d: no benchmark band -- %d px is left once the '
                   'board keeps its %d px floor, and the band needs %d'
                   % (max(0, cap), need, STAGE3D_BAND_MIN_PX))
    if track_h > cap:
        return even(cap), ('stage3d: benchmark band %d -> %d px so the '
                           'board keeps %d%% of the height'
                           % (track_h, even(cap),
                              round(100 * STAGE3D_BOARD_H_FRAC)))
    return track_h, None


def _stage3d_boxes(W, H, inner_y, inner_h):
    """`(board, panel, why)` for stage3d: the board top-left, and the layer
    column beside it (landscape) or a row under it (portrait)."""
    if W >= ISO_SIDE_ASPECT * H:
        bw = min(W - 2, _up_even(STAGE3D_BOARD_W_FRAC * W))
        return (Box(0, inner_y, bw, inner_h),
                Box(bw, inner_y, W - bw, inner_h), None)
    need = min(inner_h, _up_even(STAGE3D_BOARD_H_FRAC * H))
    row = inner_h - need
    if row < STAGE3D_ROW_MIN_PX:
        return (Box(0, inner_y, W, inner_h), None,
                'stage3d: portrait frame -- no layer row (%d px left under '
                'the board, needs %d)' % (row, STAGE3D_ROW_MIN_PX))
    return (Box(0, inner_y, W, need), Box(0, inner_y + need, W, row),
            'stage3d: portrait frame -- the layer column is a row under '
            'the board')


def _self_check(g: FrameGeometry) -> None:
    """Every named box inside the frame, and the column off the board.
    Cheap, and it turns an arithmetic slip into a refusal here rather than a
    distorted film later."""
    for name in ('board', 'rail', 'foot', 'panel', 'track'):
        b = getattr(g, name)
        if b is None or b.w <= 0 or b.h <= 0:
            continue
        if not g.frame.contains(b):
            raise FrameSizeError('layout %r: %s %s is outside the frame %s'
                                 % (g.layout, name, tuple(b), tuple(g.frame)))
    if g.panel is not None and g.board.overlaps(g.panel):
        raise FrameSizeError('layout %r: the panel %s overlaps the board %s'
                             % (g.layout, tuple(g.panel), tuple(g.board)))
    if (g.track is not None
            and g.frame.w >= ISO_SIDE_ASPECT * g.frame.h
            and (g.board.w < STAGE3D_BOARD_W_FRAC * g.frame.w
                 or g.board.h < STAGE3D_BOARD_H_FRAC * g.frame.h)):
        raise FrameSizeError('layout stage3d: board %dx%d is under %d%% x '
                             '%d%% of the frame %dx%d'
                             % (g.board.w, g.board.h,
                                round(100 * STAGE3D_BOARD_W_FRAC),
                                round(100 * STAGE3D_BOARD_H_FRAC),
                                g.frame.w, g.frame.h))
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
    """One line saying what frame ran and what it gave up: none of the
    states may read like silence."""
    line = ('movie: layout 3D stage  %dx%d at %.2f:1, board %dx%d'
            % (geom.frame.w, geom.frame.h, geom.aspect,
               geom.board.w, geom.board.h))
    if geom.track is not None:
        line += ', benchmark band %dpx inside' % geom.track.h
    for note in geom.notes:
        line += '  |  ' + note
    return line
