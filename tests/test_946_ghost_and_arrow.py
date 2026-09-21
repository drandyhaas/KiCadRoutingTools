#!/usr/bin/env python3
"""Where a part came FROM (#946 item 6/10, #1020).

A placement tween glides parts from their source pose to their parsed one, and
watched frame by frame that reads as *the board assembling itself* rather than
as *these eleven parts moved, from there to here*. A viewer sees where a part
ARRIVED and never where it came from -- the one question a placement film
exists to answer.

`render_placement` has drawn a ghost and an arrow for a STILL since #896 and
could not share them: the still needs a `--before` board and the film has none,
because it re-points `renderer.pcb` at interpolated poses. The "before" exists
only as an offset inside `Stage._tween`, and that offset is what this uses.

What this file pins:

  * **it draws, and only during the glide.** A drawer wrapped in
    `except Exception: pass` returns None whether it drew or not, so the ink is
    counted -- and counted on the frames AROUND the tween too, because an
    annotation that never clears is worse than none;
  * **the arrow GROWS from the ghost to the part**, so at the end it spans the
    whole journey. Asserted on ink, over the tween, monotonically;
  * **the ghost fades IN.** The first version faded it out, making it faintest
    exactly when the arrow was longest and the origin was the only thing left
    to show;
  * **a part that barely moved gets nothing** -- below `MIN_TRAVEL_MM` the
    ghost sits on the part and the arrow is a dot;
  * **the ghost and the arrow share one frame of reference.** The arrow runs
    footprint-origin to footprint-origin and the ghost carries the pad bbox's
    offset from that origin, because mixing the two puts the arrow's tail
    outside its own ghost;
  * **it costs NO frame geometry**, which is the whole reason it goes through
    the `overlays=` seam.
"""
import os
import sys

RUN_ALL_TIMEOUT = 600

_TESTS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_TESTS)
for _p in (ROOT, _TESTS, os.path.join(ROOT, 'py_router'),
           os.path.join(ROOT, 'py_placer'), os.path.join(ROOT, 'py_tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from PIL import Image, ImageDraw
except ImportError as exc:
    print('SKIP: needs Pillow (%s)' % exc)
    sys.exit(77)

import place_motion as PM          # noqa: E402
import render_theme as RT          # noqa: E402
import route_render as RR          # noqa: E402
from kicad_parser import parse_kicad_pcb    # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')

_FAIL = []


def fail(msg):
    _FAIL.append(msg)
    print('  FAIL: %s' % msg)


def _r():
    return RR.BoardRenderer(parse_kicad_pcb(BOARD), size=420, supersample=1)


def _ink(img, rgb):
    return sum(n for n, c in img.convert('RGB').getcolors(1 << 20) if c == rgb)


def _render(items, t):
    r = _r()
    return r.frame(segments=[], vias=[],
                   overlays=[PM.ghost_overlay(items, RT.DARK, t=t)]), r


def _item(ref, fx, fy, nx, ny, hw=2.0, hh=1.0, off=(0.0, 0.0)):
    return (ref, (fx, fy), (nx, ny), hw, hh, off)


def test_it_draws_and_the_arrow_grows():
    _mark = len(_FAIL)
    r0 = _r()
    b = r0.bounds
    cx = (b[0] + b[2]) / 2.0
    cy = (b[1] + b[3]) / 2.0
    arrow = RT.DARK.rgb('place_arrow')
    ink = []
    for t in (0.0, 0.25, 0.5, 0.75, 1.0):
        # the part travels from cx-20 toward cx+20 as t rises
        nx = cx - 20.0 + 40.0 * t
        img, _r2 = _render([_item('U1', cx - 20.0, cy, nx, cy)], t)
        ink.append(_ink(img, arrow))
    if not ink[-1]:
        fail('the arrow never reached the canvas at all')
        return
    if any(b_ < a_ for a_, b_ in zip(ink, ink[1:])):
        fail('the arrow does not grow with the glide: %s' % ink)
    if ink[0] > ink[-1] * 0.25:
        fail('the arrow starts at %d of a final %d -- it is not growing from '
             'the ghost' % (ink[0], ink[-1]))
    else:
        print('    arrow ink over t=0..1: %s' % ink)
    if len(_FAIL) == _mark:
        print('  PASS: it draws, and it stretches from the origin')


def test_the_ghost_fades_IN():
    """Faded OUT, it is faintest exactly when the arrow is longest and the
    origin is the only thing left to show."""
    _mark = len(_FAIL)
    if PM.GHOST_FADE[0] >= PM.GHOST_FADE[1]:
        fail('GHOST_FADE %s fades OUT' % (PM.GHOST_FADE,))
    body = RT.DARK.rgb('board_body')
    ghost = RT.DARK.rgb('place_ghost')

    def _dist(t):
        f = PM._lerp(PM.GHOST_FADE[0], PM.GHOST_FADE[1], t)
        g = tuple(body[i] + (ghost[i] - body[i]) * f for i in range(3))
        return sum((g[i] - body[i]) ** 2 for i in range(3)) ** 0.5

    d0, d1 = _dist(0.0), _dist(1.0)
    if d1 <= d0:
        fail('the ghost is no more visible at the end (%.1f) than at the '
             'start (%.1f)' % (d1, d0))
    else:
        print('    ghost against the board: %.1f at t=0 -> %.1f at t=1'
              % (d0, d1))
    if len(_FAIL) == _mark:
        print('  PASS: strongest when it is the only record of the origin')


def test_a_part_that_barely_moved_gets_nothing():
    _mark = len(_FAIL)
    r0 = _r()
    b = r0.bounds
    cx, cy = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
    arrow = RT.DARK.rgb('place_arrow')
    small = PM.MIN_TRAVEL_MM * 0.5
    big = PM.MIN_TRAVEL_MM * 8.0
    a, _x = _render([_item('U1', cx, cy, cx + small, cy)], 1.0)
    c, _y = _render([_item('U1', cx, cy, cx + big, cy)], 1.0)
    if _ink(a, arrow):
        fail('a %.2f mm move (below MIN_TRAVEL_MM %.2f) still drew an arrow'
             % (small, PM.MIN_TRAVEL_MM))
    if not _ink(c, arrow):
        fail('BROKEN TEST: a %.2f mm move drew nothing either, so the '
             'threshold is not what suppressed the small one' % big)
    else:
        print('    %.2f mm -> no annotation, %.2f mm -> %d px of arrow'
              % (small, big, _ink(c, arrow)))
    if len(_FAIL) == _mark:
        print('  PASS: a ghost on top of its own part says nothing')


def test_the_ghost_and_the_arrow_share_one_frame_of_reference():
    """The arrow runs origin-to-origin; the ghost carries the pad bbox's
    offset from that origin. Mixing them puts the tail outside its own
    ghost -- which is what the first version did."""
    _mark = len(_FAIL)
    pcb = parse_kicad_pcb(BOARD)
    ref = sorted(pcb.footprints)[0]
    fp = pcb.footprints[ref]
    home = {ref: (fp.x, fp.y,
                  [(p, p.global_x, p.global_y) for p in fp.pads], [])}
    deltas = [(ref, fp, 12.0, 7.0)]
    items = PM.items_from_deltas(deltas, home)
    if len(items) != 1:
        fail('items_from_deltas returned %d rows' % len(items))
        return
    _rf, (fx, fy), (nx, ny), hw, hh, (ox, oy) = items[0]
    # the FROM pose is the home ORIGIN plus the delta
    if abs(fx - (fp.x + 12.0)) > 1e-6 or abs(fy - (fp.y + 7.0)) > 1e-6:
        fail('the from-pose is %s, expected %s'
             % ((fx, fy), (fp.x + 12.0, fp.y + 7.0)))
    if abs(nx - fp.x) > 1e-6 or abs(ny - fp.y) > 1e-6:
        fail('the now-pose is not the footprint origin')
    # and the ghost, at from + offset, must CONTAIN the arrow's tail
    if not (fx + ox - hw - 1e-6 <= fx <= fx + ox + hw + 1e-6
            and fy + oy - hh - 1e-6 <= fy <= fy + oy + hh + 1e-6):
        fail('the arrow tail (%.3f, %.3f) is outside its own ghost '
             '(offset %.3f, %.3f, half %.3f x %.3f)'
             % (fx, fy, ox, oy, hw, hh))
    else:
        print('    ghost half-extent %.2f x %.2f mm, pad offset %.2f, %.2f'
              % (hw, hh, ox, oy))
    if len(_FAIL) == _mark:
        print('  PASS: one frame of reference, tail inside its own ghost')


def test_it_costs_no_frame_geometry_and_never_raises():
    _mark = len(_FAIL)
    r = _r()
    plain = r.frame(segments=[], vias=[])
    b = r.bounds
    cx, cy = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
    drawn = r.frame(segments=[], vias=[], overlays=[PM.ghost_overlay(
        [_item('U1', cx - 20, cy, cx + 20, cy)], RT.DARK, t=1.0)])
    if plain.size != drawn.size:
        fail('the frame size moved: %s -> %s' % (plain.size, drawn.size))
    if plain.tobytes() == drawn.tobytes():
        fail('the overlay changed nothing at all')
    # and every shape of broken input must be survivable
    for bad in ([], None, [('U1', (0, 0), (1, 1))],
                [('U1', ('x', 'y'), (1, 1), 1, 1, (0, 0))]):
        try:
            r.frame(segments=[], vias=[],
                    overlays=[PM.ghost_overlay(bad or [], RT.DARK, t=0.5)])
        except Exception as exc:                              # noqa: BLE001
            fail('a ghost raised %s on %r -- a movie is an artifact'
                 % (exc.__class__.__name__, bad))
    if len(_FAIL) == _mark:
        print('  PASS: same size, real ink, never raises')


def test_the_stage_clears_it_when_the_glide_ends():
    """An annotation that never clears is worse than none."""
    _mark = len(_FAIL)
    src = open(os.path.join(ROOT, 'py_router', 'movie_camera.py'),
               encoding='utf-8').read()
    if 'place_motion' not in src:
        fail('the Stage does not use the ghost at all')
        return
    if src.count('self.movie.overlay = None') < 1:
        fail('the Stage never clears the overlay, so the ghost would persist '
             'onto every frame after the glide')
    import animate_route as A
    r = _r()
    m = A.Movie(r, list(r.copper_layers))
    if getattr(m, 'overlay', 'missing') is not None:
        fail('Movie.overlay does not default to None')
    m.snapshot('input')
    n0 = len(m.frames)
    b = r.bounds
    cx, cy = (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0
    m.overlay = PM.ghost_overlay([_item('U1', cx - 20, cy, cx + 20, cy)],
                                 RT.DARK, t=1.0)
    m.snapshot('moving')
    m.overlay = None
    m.snapshot('landed')
    arrow = RT.DARK.rgb('place_arrow')
    ink = [_ink(f, arrow) for f in m.frames[n0 - 1:]]
    if len(ink) != 3:
        fail('expected 3 frames, got %d' % len(ink))
    elif not (ink[0] == 0 and ink[1] > 0 and ink[2] == 0):
        fail('the ghost is not confined to the glide: %s' % ink)
    else:
        print('    arrow ink before/during/after: %s' % ink)
    if len(_FAIL) == _mark:
        print('  PASS: it appears for the glide and clears after it')


TESTS = (
    test_it_draws_and_the_arrow_grows,
    test_the_ghost_fades_IN,
    test_a_part_that_barely_moved_gets_nothing,
    test_the_ghost_and_the_arrow_share_one_frame_of_reference,
    test_it_costs_no_frame_geometry_and_never_raises,
    test_the_stage_clears_it_when_the_glide_ends,
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
