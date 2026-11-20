#!/usr/bin/env python3
"""The lines escapes leave an array along: one lattice a face, as fine as the fanout's track fits (awx/escape_plan.py:
line_step, lattice, escape_menu, escape_out).

  python3 tests/test_622_escape_lattice.py

An escape runs out of the array along a line parallel to its face's normal. The lines stood on the half-ball grid --
a ball's own line and the gap line between two balls -- whatever the track: at the fab ladder's narrowest sizes two
lanes fit between two balls, and the menu offered one. The lines are a lattice now: the ball pitch cut in the most
EVEN parts each at least a lane's pitch at the fanout's sizes (its track, its clearance and a routing grid step). A
ball's own line and the gap line are always on it, so every escape of the half-ball grid is still offered, and two
runs a lattice line apart never conflict.

1. the step: the ball pitch over the largest EVEN count of parts no narrower than a lane's pitch -- half a ball pitch
   at today's sizes, a quarter at the fab ladder's narrowest; never finer than the lanes fit, never coarser than half
   a pitch;
2. the lattice out to a reach: every multiple of the step up to it, the reach itself the last;
3. a dog-boned ball's menu at today's sizes: its lines the half-ball grid's, as before;
4. ...at the narrowest sizes: the same lines and those between them, every one on the quarter-pitch lattice -- more
   escapes, none lost;
5. a ball in its pad (no via of its own) likewise: its own line, and every lattice line out to the gap line;
6. no side-step ALONG THE EXIT LINE: an outer ring's dog-bone stands on the gap point half a pitch out, and the "row"
   it could side-step along is the exit line itself -- no row at all. On an array whose balls stand a few nanometres
   off their grid (a rotated footprint's rounding) that row came out a few nanometres short of the exit line, the
   side-steps were offered, and their last leg -- along the exit line, that far off its axis -- was no span of the
   move's: a run the array's rules never saw, laid across the tips of the escapes ending on that line. With the
   balls off their grid the menu is the one the exact array has.
"""
import contextlib
import io
import os
import sys
import tempfile
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
os.environ.pop('AWX_RULES', None)

import rules as _rules  # noqa: E402

TODAY = _rules.Rules(clearance=0.09, track=0.15, via_size=0.45, via_drill=0.3, fan_track=0.12, hole_to_hole=0.25,
                     source='test')
FLOOR = _rules.Rules(clearance=0.09, track=0.0762, via_size=0.25, via_drill=0.15, fan_track=0.0762,
                     hole_to_hole=0.25, source='test')
_rules.install(TODAY)
import escape_moves as em  # noqa: E402
import escape_plan as ep  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402

BAD, NCHK = [], [0]
N, PITCH = 5, 0.8


def check(what, ok, got=None):
    NCHK[0] += 1
    if not ok:
        BAD.append(what)
    print(('ok   ' if ok else 'FAIL ') + what + ('' if ok or got is None else f' -- got {got}'))


def board(jitter=0.0):
    nets = '\n'.join(f'  (net {i + 1} "N{i}")' for i in range(N * N))
    # (`jitter`: the top ball of the last column that far past its grid line -- the array's box, and its exit line,
    # with it)
    pads = '\n'.join(f'    (pad "{i}" smd circle (at {(i % N) * PITCH + (jitter if i == N - 1 else 0.0):.6f} '
                     f'{(i // N) * PITCH:.3f}) (size 0.4 0.4) '
                     f'(layers "F.Cu" "F.Mask") (net {i + 1} "N{i}"))' for i in range(N * N))
    jpads = '\n'.join(f'    (pad "{i}" smd rect (at {i * 1.0:.3f} 0) (size 0.5 0.5) (layers "F.Cu" "F.Mask") '
                      f'(net {i + 1} "N{i}"))' for i in range(1, N * N))
    # (the middle ball dog-boned: a via half a pitch down and right of it)
    c = (N // 2) * PITCH
    return f"""(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
{nets}
  (footprint "t:BGA" (layer "F.Cu") (at 10 10)
    (property "Reference" "U1" (at 0 -2) (layer "F.SilkS"))
{pads}
  )
  (footprint "t:J" (layer "F.Cu") (at 10 40)
    (property "Reference" "J1" (at 0 -2) (layer "F.SilkS"))
{jpads}
  )
  (segment (start {10 + c:.3f} {10 + c:.3f}) (end {10 + c + 0.4:.3f} {10 + c + 0.4:.3f}) (width 0.12) (layer "F.Cu") (net {N * (N // 2) + N // 2 + 1}))
  (via (at {10 + c + 0.4:.3f} {10 + c + 0.4:.3f}) (size 0.45) (drill 0.3) (layers "F.Cu" "B.Cu") (net {N * (N // 2) + N // 2 + 1}))
  (gr_rect (start 0 0) (end 60 60) (stroke (width 0.1) (type solid)) (fill none) (layer "Edge.Cuts"))
)
"""


def step(r, pitch=PITCH):
    _rules.install(r)
    return ep.line_step(pitch, types.SimpleNamespace(track=r.fan_track, clear=r.fan_clear))


# 1. the step
check('1. today\'s sizes: half a ball pitch', abs(step(TODAY) - PITCH / 2) < 1e-9, step(TODAY))
check('1. the ladder\'s narrowest: a quarter', abs(step(FLOOR) - PITCH / 4) < 1e-9, step(FLOOR))
for r in (TODAY, FLOOR):
    s = step(r)
    parts = PITCH / s
    check(f'1. ...an even count of parts ({parts:g}), each no narrower than a lane\'s pitch',
          abs(parts - round(parts)) < 1e-9 and round(parts) % 2 == 0 and s >= r.fan_track + r.fan_clear + r.grid - 1e-9)
check('1. a pitch too fine for two lanes: half of it all the same', abs(step(TODAY, 0.3) - 0.15) < 1e-9, step(TODAY, 0.3))
# 2. the lattice
check('2. the multiples of the step out to the reach', [round(o, 6) for o in ep.lattice(0.8, 0.2)] == [0.2, 0.4, 0.6, 0.8]
      and [round(o, 6) for o in ep.lattice(0.4, 0.4)] == [0.4])


def lines(r, pad_no, face='right'):
    _rules.install(r)
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'b.kicad_pcb')
        open(p, 'w').write(board())
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(p)
    pad = next(q for q in pcb.footprints['U1'].pads if q.pad_number == str(pad_no))
    copper = ep.Copper(pcb, r.fan_track, r.fan_clear)
    grid = em.grid_of(pcb.footprints['U1'])
    site = ep.own_via(pcb, pad, pad.net_id, grid)
    layers = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    moves = ep.escape_menu(pad, site, grid, [face], layers, copper) if site is not None else \
        ep.escape_out(pad, grid, [face], copper)
    # (a move to the right face leaves along a horizontal line: its exit's y)
    return sorted({round(m.exit_pt[1], 4) for m in moves}), (site, pad)


mid = N * (N // 2) + N // 2
l_today, (site, pad) = lines(TODAY, mid)
l_floor, _ = lines(FLOOR, mid)
check('3. a dog-boned ball, today\'s sizes: some escape', bool(l_today))
check('3. ...its lines on the half-ball grid', all(abs((y - site[1]) / (PITCH / 2) - round((y - site[1]) / (PITCH / 2))) < 1e-6
                                                for y in l_today), l_today)
check('4. the narrowest sizes: every line on the quarter-pitch lattice',
      all(abs((y - site[1]) / (PITCH / 4) - round((y - site[1]) / (PITCH / 4))) < 1e-6 for y in l_floor), l_floor)
check('4. ...none of today\'s lost, and lines between them gained', set(l_today) <= set(l_floor) and len(l_floor) > len(l_today),
      (l_today, l_floor))
def out_lines(r, pad_no=2):
    # (a ball of the top row, no via of its own: its escapes straight out by the face above it, each along a
    # vertical line -- its exit's x)
    _rules.install(r)
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'b.kicad_pcb')
        open(p, 'w').write(board())
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(p)
    pad = next(q for q in pcb.footprints['U1'].pads if q.pad_number == str(pad_no))
    moves = ep.escape_out(pad, em.grid_of(pcb.footprints['U1']), ['up'], ep.Copper(pcb, r.fan_track, r.fan_clear))
    return sorted({round(m.exit_pt[0], 4) for m in moves}), pad.global_x


xs_t, x0 = out_lines(TODAY)
xs_f, _x = out_lines(FLOOR)
check('5. a ball in its pad: its own line among its escapes', any(abs(x - x0) < 1e-6 for x in xs_f), xs_f)
check('5. ...every line a multiple of the step from its own, none past the gap line',
      all(abs((x - x0) / (PITCH / 4) - round((x - x0) / (PITCH / 4))) < 1e-6 for x in xs_f)
      and all(abs(x - x0) <= PITCH / 2 + 1e-6 for x in xs_f), xs_f)
check('5. ...none of today\'s lost, and lines between them gained', set(xs_t) <= set(xs_f) and len(xs_f) > len(xs_t),
      (xs_t, xs_f))



def outer_menu(jitter):
    """the moves of the last column's second ball from a dog-bone on the gap point half a pitch OUT of the array, by
    the face beside it: (each move's exit along the face, its legs' count), and the exit line's x"""
    _rules.install(TODAY)
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'b.kicad_pcb')
        open(p, 'w').write(board(jitter))
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(p)
    pad = next(q for q in pcb.footprints['U1'].pads if q.pad_number == str(2 * N - 1))
    grid = em.grid_of(pcb.footprints['U1'])
    site = (pad.global_x + PITCH / 2, pad.global_y + PITCH / 2)
    moves = ep.escape_menu(pad, site, grid, ['right'], ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'],
                           ep.Copper(pcb, TODAY.fan_track, TODAY.fan_clear))
    e = grid.bbox[2] + ep.PAST * PITCH
    along = [m for m in moves for a, b, _L in m.legs if abs(a[0] - e) < 1e-3 and abs(b[0] - e) < 1e-3
             and abs(a[1] - b[1]) > 1e-3]
    return sorted((round(m.exit_pt[1], 3), m.layer, len(m.legs)) for m in moves), along


exact, along0 = outer_menu(0.0)
off, along1 = outer_menu(8e-6)
check('6. an outer ring\'s dog-bone half a pitch out: some escape, none along the exit line', bool(exact) and not along0,
      along0[:2])
check('6. ...nor with the balls a few nanometres off their grid', not along1, along1[:2])
check('6. ...its menu the exact array\'s', off == exact, (len(off), len(exact)))

print(f'{NCHK[0] - len(BAD)}/{NCHK[0]} checks passed')
sys.exit(1 if BAD else 0)
