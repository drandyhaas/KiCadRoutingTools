#!/usr/bin/env python3
"""A part that owns a milled ring may sit ON it, not merely swallow it.

An Edge.Cuts contour enclosing >= 2 pad centres is reclassified by the parser
from board CUTOUT to inner milled edge (#291/#628) -- otherwise the enclosed
pads would be marked off-board and the run breaks. `rings_enclosing` /
`skip_rings` then exempt the OWNING part, so a connector over its own milled
relief is not judged board-violating at its own hand-placed pose.

That exemption covered only the SWALLOW probe. The EDGE-MARGIN test -- which
vetoes any rect within `max(clearance, edge)` of any ring edge -- had no
exemption at all, so a part sitting inside its own relief had every pose vetoed,
which for such a part is every pose there is.

Measured on run 20's SW2, which owns the strap slot its two NPTH posts sit
inside: 0 legal poses of 14884 at the board's own floors; at margin 0, 508 of
9604, all at rot 0/180 and none at SW2's own rot 270. `place_optimize` with SW2
free and 83 refs locked moved it 0 mm with the edge term as the entire
objective (86024 of 86024). `place_reconstruct --stages classify,legalize`
reported "no legal pose within any cap". The run declared SW2 an
`edge_actuator` and waived it permanently.

PER-PART, never global. The ring stays a hard edge for every other part, and
`rings_enclosing` returns MILLED ids only -- the outer outline and the genuine
cutouts can never be exempted through this path.

    python3 tests/test_run20_ring_ownership.py
"""
import contextlib
import io
import os
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (REPO, os.path.join(REPO, 'py_router'), os.path.join(REPO, 'py_tools'),
           os.path.join(REPO, 'py_placer')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from kicad_parser import parse_kicad_pcb                           # noqa: E402
from placement.legality import BoardOutlineGate                    # noqa: E402

passed = failed = 0


def check(label, cond, detail=''):
    global passed, failed
    if cond:
        passed += 1
        print(f'  OK   {label}')
    else:
        failed += 1
        print(f'  FAIL {label} -- {detail}')


# 40x30 board with an interior milled ring (a strap slot) at 18..22 x 12..18.
# SW2's two THROUGH-HOLE posts sit INSIDE it, which is what makes the contour a
# milled edge rather than a cutout. R9 sits outside, in the same band.
_BOARD = '''(kicad_pcb (version 20260206) (generator t)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))
 (gr_rect (start 0 0) (end 40 30) (layer "Edge.Cuts") (width 0.1))
 (gr_rect (start 18 12) (end 22 18) (layer "Edge.Cuts") (width 0.1))
 (net 0 "") (net 1 "N1")
 (footprint "t:sw" (layer "F.Cu") (at 20 15)
  (property "Reference" "SW2" (at 0 0) (layer "F.SilkS"))
  (pad "1" thru_hole circle (at -1 0) (size 1 1) (drill 0.8) (layers "*.Cu")
   (net 1 "N1"))
  (pad "2" thru_hole circle (at 1 0) (size 1 1) (drill 0.8) (layers "*.Cu")
   (net 1 "N1")))
 (footprint "t:r" (layer "F.Cu") (at 30 15)
  (property "Reference" "R9" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") (net 1 "N1")))
)'''

_D = tempfile.mkdtemp()
_B = os.path.join(_D, 'ring.kicad_pcb')
with open(_B, 'w', encoding='utf-8') as fh:
    fh.write(_BOARD)
with contextlib.redirect_stdout(io.StringIO()):
    pcb = parse_kicad_pcb(_B)

MARGIN = 0.254
gate = BoardOutlineGate(pcb.board_info, MARGIN)

print('--- the fixture is the shape under test ---')
check('the pad-enclosing contour was reclassified to a MILLED edge',
      len(pcb.board_info.board_edge_contours or []) == 1
      and not (pcb.board_info.board_cutouts or []),
      f'{len(pcb.board_info.board_edge_contours or [])} milled / '
      f'{len(pcb.board_info.board_cutouts or [])} cutouts -- if it stayed a '
      f'cutout the enclosed pads would be off-board and this tests nothing')
check('the outline gate is active (a rect board with one ring is exact '
      'without it)', gate.active and len(gate.milled) == 1,
      f'active={gate.active} milled={len(gate.milled)}')

_sw = pcb.footprints['SW2']
_own = gate.rings_enclosing([(p.global_x, p.global_y) for p in _sw.pads])
check('SW2 OWNS the ring its posts sit in', bool(_own), str(_own))
_r9 = pcb.footprints['R9']
_foreign = gate.rings_enclosing([(p.global_x, p.global_y) for p in _r9.pads])
check('R9 owns nothing', not _foreign, str(_foreign))

print('--- the pose that was vetoed: a body SPANNING its own slot ---')
# Edges cross the ring, so the margin test measures a distance of 0.
_span = (17.9, 13.5, 22.1, 16.5)
check('without an exemption the owner is BLOCKED at its own pose',
      gate.rect_blocked(_span),
      'if this is False the fixture is not exercising the margin test')
check('WITH ownership the owner is legal there',
      not gate.rect_blocked(_span, skip_rings=_own),
      'the fix: the exemption now covers the edge-margin test, not only the '
      'swallow probe')
check('and the cost function agrees -- 0, not a residual graze',
      gate.rect_outside_amount(_span, skip_rings=_own) == 0.0,
      f'{gate.rect_outside_amount(_span, skip_rings=_own)} -- the two must '
      f'agree or `violation() == 0` stops implying `not rect_blocked()`, and '
      f'every acceptance rule downstream reads the disagreement as a regression')
check('the unexempted cost is non-zero, so the pair really does discriminate',
      gate.rect_outside_amount(_span) > 0.0,
      str(gate.rect_outside_amount(_span)))

print('--- and it is PER-PART ---')
check('a FOREIGN part is still blocked at the same rect',
      gate.rect_blocked(_span, skip_rings=_foreign),
      'the ring is a hard edge for everyone who does not own it; a global '
      'exemption would let any part sit in the slot')
check('...and still charged for it',
      gate.rect_outside_amount(_span, skip_rings=_foreign) > 0.0,
      str(gate.rect_outside_amount(_span, skip_rings=_foreign)))

print('--- the outer outline is NOT exemptible through this path ---')
# A rect hanging off the real board edge must stay blocked no matter what is
# claimed as owned. `rings_enclosing` cannot return the outer ring, but a
# caller could pass any id, so the guard is tested rather than assumed.
_off = (-1.0, 13.0, 3.0, 17.0)
check('a rect off the real board edge is blocked with ownership claimed',
      gate.rect_blocked(_off, skip_rings=frozenset(range(8))),
      'the corner test reads the OUTER ring and the cutouts, which this '
      'exemption never touches')

print('--- a pose clear of the ring is unaffected either way ---')
_clear = (18.6, 13.6, 21.4, 16.4)      # >0.254 from every ring edge
check('an owner well inside its slot was never blocked',
      not gate.rect_blocked(_clear) and not gate.rect_blocked(
          _clear, skip_rings=_own), '')
check('and its cost is 0 both ways -- the exemption adds nothing here',
      gate.rect_outside_amount(_clear) == 0.0
      and gate.rect_outside_amount(_clear, skip_rings=_own) == 0.0, '')

print('--- rotation: the owner is legal at ITS OWN orientation ---')
# The run-20 tell was that the only legal poses were rot 0/180 -- never SW2's
# own 270. A slot-spanning body at 90 degrees is the same test rotated.
_span90 = (18.5, 11.9, 21.5, 18.1)
check('the owner spans its slot the OTHER way and is still legal',
      gate.rect_blocked(_span90)
      and not gate.rect_blocked(_span90, skip_rings=_own),
      f'plain={gate.rect_blocked(_span90)} '
      f'owned={gate.rect_blocked(_span90, skip_rings=_own)}')

print('--- caching does not leak between parts ---')
# The edge set is cached per skip-set; a cache keyed carelessly would hand one
# part another's exemption.
for _ in range(3):
    check_a = gate.rect_blocked(_span, skip_rings=_own)
    check_b = gate.rect_blocked(_span, skip_rings=_foreign)
check('repeated interleaved queries keep their own answers',
      not check_a and check_b, f'owner={check_a} foreign={check_b}')

print(f'\n{passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
