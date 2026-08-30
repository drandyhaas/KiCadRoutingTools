#!/usr/bin/env python3
"""An edge seat must land ON the board, or not be a seat.

`_seat_edge` is the one seat in the system that runs neither `pose_ok` (an edge
part overhangs by design, so full containment is the wrong predicate) nor a
clearance ladder. That left it with no containment check of any kind, and three
defects that compounded into a connector placed entirely off the far side of
the board and reported as seated, exit 0:

  A. `_seat_edge` took no exclude set, so an unplaced PILE at the board centre
     vetoed the honest edge poses and the slide loop walked the part along the
     edge until one was "conflict free".
  B. `_edge_correct` drives a SCALAR SUM (`rect_outside_amount` sums all four
     sides) with a SINGLE-AXIS walk, so an along-edge overshoot is a constant
     it can never cancel -- it subtracts it again every iteration and marches
     the part inland past the opposite edge. It ran a fixed `range(4)` with no
     convergence test, returning a diverged pose indistinguishably from a
     converged one.
  C. The along-edge fraction was clamped to a bare [0.05, 0.95], which knows
     nothing of the part's own width.

Measured before the fix, on a staged pile of orangecrab_ext_pll:
`_seat_edge(J2, south)` against the pile returned pose (170.068, 88.767) and
reported a seat -- on a board spanning y 91.52..114.38, i.e. entirely off
the NORTH edge, 26.17mm from where the same call lands on the placed board
((159.909, 112.33)). The fix makes the pile case return that same correct
answer.

The fixture is the tracked `kicad_files/orangecrab_ext_pll.kicad_pcb`, driven
through the seeder helpers IN MEMORY -- no CLI, so the routed/unplaced gates
and the board's existing copper are irrelevant to what is being measured here.
It is used because it genuinely reproduces: J2's courtyard is 41.16mm on a
50.80mm edge, so the part is 81% of its own edge and the fraction clamp is
load-bearing. `flat_hierarchy` (the acceptance fixture) does NOT reproduce any
of this -- its connectors are small and far from the pile -- which is exactly
why this file exists separately.
"""
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_tools'),
          os.path.join(REPO, 'py_placer')):
    if p not in sys.path:
        sys.path.insert(0, p)

BOARD = os.path.join(REPO, 'kicad_files', 'orangecrab_ext_pll.kicad_pcb')
passed = failed = 0


def check(name, ok, detail=""):
    global passed, failed
    passed += bool(ok)
    failed += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {name}{(' -- ' + detail) if detail else ''}")


if not os.path.isfile(BOARD):
    print(f"FAIL fixture missing: {BOARD} (it is tracked; a skip here would "
          f"hide the whole file)")
    print("\n0 passed, 1 failed")
    sys.exit(1)

import pose_score
from kicad_parser import parse_kicad_pcb
from placement import seeder

pcb = parse_kicad_pcb(BOARD)


def fresh_state():
    return pose_score.make_state(pcb, BOARD, clearance=0.25,
                                 board_edge_clearance=0.55, grid_step=0.1)


def pile(state, keep=('J2',)):
    """Stack every movable part at the board centre, as staging does."""
    x0, y0, x1, y1 = state.board
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    piled = []
    for ref, p in state.parts.items():
        if ref in keep or getattr(p, 'locked', False):
            continue
        state.apply_move(ref, cx, cy, 0.0)
        piled.append(ref)
    return set(piled)


def pads_on_board(state, ref):
    """Does this part's PAD COPPER lie inside the outline bbox?"""
    p = state.parts[ref]
    fp = pcb.footprints[ref]
    dx, dy = p.x - fp.x, p.y - fp.y
    xs = [q.global_x + dx for q in fp.pads]
    ys = [q.global_y + dy for q in fp.pads]
    x0, y0, x1, y1 = state.board
    return (min(xs) >= x0 - 0.01 and max(xs) <= x1 + 0.01
            and min(ys) >= y0 - 0.01 and max(ys) <= y1 + 0.01)


# --------------------------------------------------------------------------
# C. the fraction clamp knows the part's own width
# --------------------------------------------------------------------------
st = fresh_state()
j2 = st.parts['J2']
r = j2.rect(0.0, 0.0, j2.rot)
width, span = r[2] - r[0], st.board[2] - st.board[0]
lo, hi = seeder._edge_frac_bounds(j2, st.board, 'south')
# centre may range over (span - width); as a fraction that is width/2/span in
# from each end. 41.16 on 50.80 -> 0.405 .. 0.595.
want_lo = (width / 2.0) / span
check("the fraction clamp is derived from the part's own half-extent",
      abs(lo - want_lo) < 1e-6 and abs(hi - (1.0 - want_lo)) < 1e-6,
      f"{lo:.3f}..{hi:.3f}, part {width:.2f}mm on a {span:.2f}mm edge")
check("and it is far tighter than the old bare [0.05, 0.95]",
      lo > 0.05 and hi < 0.95, f"{lo:.3f}..{hi:.3f}")

# a part wider than its edge has NO legal fraction, and says so
wide_lo, wide_hi = seeder._edge_frac_bounds(j2, (0.0, 0.0, 10.0, 10.0), 'south')
check("a part wider than the edge yields lo > hi (no legal fraction)",
      wide_lo > wide_hi, f"{wide_lo:.3f}..{wide_hi:.3f}")

# --------------------------------------------------------------------------
# B. the correction walk reports divergence instead of hiding it
# --------------------------------------------------------------------------
st = fresh_state()
# Start it where the agent measured divergence: far along the edge, so the
# along-edge overshoot is a constant the single-axis walk cannot cancel.
x_bad, y_bad = seeder._edge_pose(st.parts['J2'], st.board, 'south', 0.90, 0.0)
_x, _y, converged = seeder._edge_correct(st, 'J2', 'south', x_bad, y_bad, 0.0)
check("a diverging overhang walk reports converged=False",
      converged is False, f"landed at ({_x:.3f}, {_y:.3f})")
check("and it really did march off the board (the case is not hypothetical)",
      _y < st.board[1] or _y > st.board[3],
      f"y={_y:.3f} vs board y {st.board[1]:.2f}..{st.board[3]:.2f}")

# a centred start converges
x_ok, y_ok = seeder._edge_pose(st.parts['J2'], st.board, 'south', 0.5, 0.0)
_x2, _y2, conv2 = seeder._edge_correct(st, 'J2', 'south', x_ok, y_ok, 0.0)
check("a well-posed walk still converges (the flag is not always False)",
      conv2 is True, f"({_x2:.3f}, {_y2:.3f})")

# --------------------------------------------------------------------------
# A + the whole thing: seat against a pile, land on the board
# --------------------------------------------------------------------------
st = fresh_state()
piled = pile(st)
entry = {'edge': 'south', 'overhang_mm': {'min': 0.0, 'max': 1.0}}
notes = []
ok = seeder._seat_edge(st, 'J2', entry, set(), notes, exclude=piled)
check("it seats against a pile", ok, str(notes[-2:]))
check("and the seat is ON the board -- the whole point",
      pads_on_board(st, 'J2'),
      f"pose ({st.parts['J2'].x:.3f}, {st.parts['J2'].y:.3f}), "
      f"board {[round(v, 2) for v in st.board]}")
check("it lands on the SOUTH edge it was asked for",
      abs(st.parts['J2'].y - st.board[3]) < 3.0,
      f"y={st.parts['J2'].y:.3f} vs south edge {st.board[3]:.2f}")

# The measured pre-fix pose, asserted absent by name.
check("it is not the pre-fix pose (170.068, 88.767)",
      not (abs(st.parts['J2'].x - 170.068) < 0.01
           and abs(st.parts['J2'].y - 88.767) < 0.01),
      "that pose is 26.17mm away and entirely off the north edge")

# A NONSENSE BAND must not buy an off-board seat. This is what makes the
# containment gate load-bearing rather than decorative: with only the fraction
# clamp in place, `converged`/`on_board` are never reached on a well-posed
# call, so removing them changes nothing observable. Here the declared band is
# satisfiable ONLY by a part sitting entirely off the board -- a 20mm overhang
# on a 3.0mm-tall connector -- and the overlap invariant is the thing that
# refuses it. Fault-injecting the gate away turns this red.
# ALL FOUR EDGES. Testing only `south` is how the first version of this file
# passed while the same nonsense band was accepted on east and west with 8 of
# 16 pads off the board: on south, J2's 3.00mm dimension is normal to the edge
# so a bare overlap test collapses to zero and refuses by accident; on east and
# west its 41.16mm dimension keeps half the courtyard over the bbox and the
# "invariant" held. One edge is not a test of an edge predicate.
for _edge in ('south', 'north', 'east', 'west'):
    st3 = fresh_state()
    piled3 = pile(st3)
    silly = {'edge': _edge, 'overhang_mm': {'min': 20.0, 'max': 21.0}}
    ok3 = seeder._seat_edge(st3, 'J2', silly, set(), [], exclude=piled3)
    check(f"[{_edge}] a band only an off-board pose could satisfy is refused",
          (not ok3) or pads_on_board(st3, 'J2'),
          f"ok={ok3} pose ({st3.parts['J2'].x:.3f}, {st3.parts['J2'].y:.3f})")

# A merely WRONG band, not a nonsense one. The overlap test only refuses a part
# that is 100% off, so `{3,4}` on a 3.00mm-deep connector seated it with 1.7%
# of its courtyard on the board. The depth bound is what refuses this.
for _band in ({'min': 3.0, 'max': 4.0}, {'min': 2.0, 'max': 3.0}):
    st4 = fresh_state()
    piled4 = pile(st4)
    ok4 = seeder._seat_edge(st4, 'J2',
                            {'edge': 'south', 'overhang_mm': _band},
                            set(), [], exclude=piled4)
    check(f"a band of {_band['min']:g}-{_band['max']:g}mm on a 3.0mm-deep part "
          f"does not seat it off the board",
          (not ok4) or pads_on_board(st4, 'J2'),
          f"ok={ok4} pose ({st4.parts['J2'].x:.3f}, {st4.parts['J2'].y:.3f})")

# A LEGITIMATE band must still seat -- otherwise the bound above is just a
# refusal machine and the edge seat stops working.
st5 = fresh_state()
piled5 = pile(st5)
ok5 = seeder._seat_edge(st5, 'J2',
                        {'edge': 'south', 'overhang_mm': {'min': 0.0,
                                                          'max': 1.0}},
                        set(), [], exclude=piled5)
check("a sane band still seats (the bound is not a blanket refusal)",
      ok5 and pads_on_board(st5, 'J2'),
      f"ok={ok5} pose ({st5.parts['J2'].x:.3f}, {st5.parts['J2'].y:.3f})")

# WITHOUT the exclude set the pile is a full obstacle set -- that is defect A.
# The seat must then either refuse or still be on-board; what it must never do
# is slide off the end and report success.
st2 = fresh_state()
pile(st2)
notes2 = []
ok2 = seeder._seat_edge(st2, 'J2', entry, set(), notes2)     # no exclude
check("even with NO exclude set it never seats off the board",
      (not ok2) or pads_on_board(st2, 'J2'),
      f"ok={ok2} pose ({st2.parts['J2'].x:.3f}, {st2.parts['J2'].y:.3f})")

print(f"\n{passed} passed, {failed} failed")
sys.exit(1 if failed else 0)
