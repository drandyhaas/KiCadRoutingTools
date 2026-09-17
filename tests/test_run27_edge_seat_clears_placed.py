#!/usr/bin/env python3
"""Stage 1 seats a declared edge connector CLEAR of the parts already placed.

The seeder's stage 1 ("edge connectors: spec geometry, no legality gate") put
a connector at its declared along-edge position and asked three questions
about the seat -- is it on the board, does it hit a keep-out, does it hit an
exclusive zone -- and never asked whether another part was already there.
Measured on esp_prog seeded from a zone plan: CON2, declared on the south edge
with band 0.25-0.75, took the band's midpoint and drove pin 1 through the
FIXED USB socket's ground tab. All ten seeds did it; the seed gate passed them
(a pad conflict is not a budgeted channel) and `check_assembly` then called
every one of them NOT BUILDABLE.

`placed` at that point is exactly the parts whose pose is AUTHORITATIVE --
locked in the file, or outside an explicit `seed_refs` scope -- so the slide
ladder stage 1 already runs (scaled to the declared window) now also has to
clear them. A band with no clear seat anywhere keeps its declared edge and
names the blocker, because the stages that would otherwise take the connector
park it in the board interior.

Run:
    python3 tests/test_run27_edge_seat_clears_placed.py
"""

import os
import random
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))

from kicad_parser import parse_kicad_pcb  # noqa: E402
from placement import floorplan as fp  # noqa: E402
from placement import seeder  # noqa: E402
from placement.legality import grade_pad_legality  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402

RUN_ALL_FAST_OK = True

#: A 30 x 20 board WITH ITS `(layers ...)` SECTION -- without one
#: `copper_layers` is empty, `grade_pad_legality` resolves no shared layer for
#: any pair, and every pad-conflict assertion below passes on a grader that
#: cannot see a conflict at all. The first draft of this file had no layers
#: section and arm 1 passed vacuously; arm 3, which asserts a conflict IS
#: found, is what exposed it.
#:
#: FIX1 is `(locked yes)` astride the MIDPOINT of the south
#: edge, which is where the declared band 0.1-0.9 puts its connector; J1 is a
#: 3-pin 2.54mm header declared on that edge; U2 is an ordinary pile part so
#: the seed has something else to do.
BOARD = '''(kicad_pcb
 (version 20241229)
 (net 0 "")
 (net 1 "/A") (net 2 "/B") (net 3 "/C")
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal))
 (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (uuid "e1"))
 (footprint "test:FIX" (layer "F.Cu") (uuid "fp-fix") (at 15 18.4)%s
   (property "Reference" "FIX1" (at 0 0 0))
   (pad "1" thru_hole circle (at 0 0) (size %s 2.2) (drill 1.0) (layers "*.Cu") (net 3 "/C") (uuid "fixp1"))
 )
 (footprint "test:HDR" (layer "F.Cu") (uuid "fp-j1") (at 15 10)
   (property "Reference" "J1" (at 0 0 0))
   (pad "1" thru_hole circle (at -2.54 0) (size 1.7 1.7) (drill 1.0) (layers "*.Cu") (net 1 "/A") (uuid "j1p1"))
   (pad "2" thru_hole circle (at 0 0) (size 1.7 1.7) (drill 1.0) (layers "*.Cu") (net 2 "/B") (uuid "j1p2"))
   (pad "3" thru_hole circle (at 2.54 0) (size 1.7 1.7) (drill 1.0) (layers "*.Cu") (net 0 "") (uuid "j1p3"))
 )
 (footprint "test:C" (layer "F.Cu") (uuid "fp-u2") (at 15 10)
   (property "Reference" "U2" (at 0 0 0))
   (pad "1" smd rect (at -0.5 0) (size 0.6 0.6) (layers "F.Cu") (net 1 "/A") (uuid "u2p1"))
   (pad "2" smd rect (at 0.5 0) (size 0.6 0.6) (layers "F.Cu") (net 2 "/B") (uuid "u2p2"))
 )
)
'''

INTENT = {
    'schema': 1, 'kind': fp.KIND, 'units': 'mm',
    'edge_connectors': [{'ref': 'J1', 'edge': 'south',
                         'class': 'edge_receptacle',
                         'overhang_mm': {'min': 0.0, 'max': 0.0},
                         'along_edge_band': {'from': 0.1, 'to': 0.9}}],
}


def _board(tmp, name, locked=True, fix_w='2.2'):
    p = os.path.join(tmp, name)
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(BOARD % (' (locked yes)' if locked else '', fix_w))
    return p


def _seed(path, out, entry=None):
    pcb = parse_kicad_pcb(path)
    doc = dict(INTENT)
    if entry is not None:
        doc['edge_connectors'] = [entry]
    intent = fp.intent_from_dict(doc, path)
    res = seeder.seed_from_intent(pcb, path, intent, random.Random('27'),
                                  group_sources=(), clearance=0.2,
                                  board_edge_clearance=0.3, grid_step=0.1)
    write_placed_output(path, out, res['placements'])
    pose = {p['reference']: (round(p['new_x'], 3), round(p['new_y'], 3),
                             p['new_rotation']) for p in res['placements']}
    return res, pose, grade_pad_legality(parse_kicad_pcb(out), 0.2,
                                         pcb_file=out)


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    with tempfile.TemporaryDirectory(prefix='t_esc_') as tmp:
        # --- 0: ANTI-VACUITY. The band's midpoint, where the seat used to go,
        #        really does land on the locked part.
        path = _board(tmp, 'b.kicad_pcb')
        pcb = parse_kicad_pcb(path)
        import pose_score
        st = pose_score.make_state(pcb, path, clearance=0.2,
                                   board_edge_clearance=0.3, grid_step=0.1)
        ctx = st.legality_ctx
        # The south-edge seat at the band midpoint: x = 15 (the edge's centre),
        # pads flush inside the outline.
        sf = ctx.pair_shortfall('J1', 'FIX1', pose_a=(15.0, 19.15, 0.0))
        check('the fixture is a trap: the band midpoint shorts the locked part',
              sf.pad > 1e-6 or sf.hole > 1e-6, f'pad {sf.pad} hole {sf.hole}')
        check('...and FIX1 is locked in the file, so the seeder treats it as placed',
              getattr(pcb.footprints['FIX1'], 'locked', False))

        # --- 1: the seat slides clear, inside its own declared band ---------
        res, pose, pads = _seed(path, os.path.join(tmp, 'out.kicad_pcb'))
        jx, jy, _rot = pose['J1']
        check('J1 is seated with no pad or hole conflict',
              pads['pad_conflicts'] == 0 and pads['hole_conflicts'] == 0,
              f"pad {pads['pad_conflicts']} hole {pads['hole_conflicts']} "
              f"worst {pads['worst'][:2]}")
        check('...on its declared south edge', jy > 15.0, f'y={jy}')
        check('...inside the declared band 0.1-0.9 of a 30mm edge',
              3.0 - 1e-6 <= jx <= 27.0 + 1e-6, f'x={jx}')
        check('...and it MOVED off the midpoint to get there',
              abs(jx - 15.0) > 0.5, f'x={jx}')

        # --- 2: nothing placed -> the seat is where it always was -----------
        # The change is inert on a board with no authoritative pose: the
        # ladder's first rung is the declared position, exactly as before.
        free = _board(tmp, 'free.kicad_pcb', locked=False)
        _r2, pose2, _p2 = _seed(free, os.path.join(tmp, 'free_out.kicad_pcb'))
        check('with nothing locked, J1 still takes the declared midpoint',
              abs(pose2['J1'][0] - 15.0) < 1e-6, f"x={pose2['J1'][0]}")

        # --- 3: a band with NO clear seat keeps the declared edge, named ----
        # A locked pad wider than the band leaves the connector nowhere to go.
        # Parking it in the interior would trade a pad conflict for a lost
        # edge; it is seated where it was declared and the blocker is named.
        wide = _board(tmp, 'wide.kicad_pcb', fix_w='26')
        r3, pose3, p3 = _seed(wide, os.path.join(tmp, 'wide_out.kicad_pcb'))
        check('with no clear seat, J1 still gets its declared south edge',
              pose3['J1'][1] > 15.0, f"{pose3['J1']}")
        check('...and the blocker is NAMED on the record',
              any('FIX1' in n and 'J1' in n for n in r3['notes']),
              f"{[n for n in r3['notes'] if 'J1' in n][:2]}")
        check('...and the conflict is real, for the gate to refuse',
              p3['pad_conflicts'] > 0 or p3['hole_conflicts'] > 0,
              f"pad {p3['pad_conflicts']} hole {p3['hole_conflicts']} "
              f"J1 at {pose3['J1']} worst {p3['worst'][:2]}")

        # --- 4: NOTHING declared but the edge -- the ARMING condition -------
        # No band, no `center_on_edge`, no keep-out, no exclusive zone: the
        # three conditions that used to arm the slide ladder are all absent,
        # so stage 1 tried exactly one fraction -- the even distribution
        # (k+1)/(n+1), which for a single connector is the edge's midpoint,
        # which is where the locked part is. A declared edge with no declared
        # position is the ordinary case on a board whose brief names an edge
        # and nothing more. Without this arm the `or placed` condition is
        # never reached and a mutation removing it SURVIVES (measured).
        bare = {'ref': 'J1', 'edge': 'south', 'class': 'edge_receptacle',
                'overhang_mm': {'min': 0.0, 'max': 0.0}}
        _r4, pose4, p4 = _seed(path, os.path.join(tmp, 'bare_out.kicad_pcb'),
                               entry=bare)
        check('an undeclared position still slides clear of the locked part',
              p4['pad_conflicts'] == 0 and p4['hole_conflicts'] == 0,
              f"pad {p4['pad_conflicts']} hole {p4['hole_conflicts']} "
              f"J1 at {pose4['J1']} worst {p4['worst'][:2]}")
        check('...on its declared south edge, off the midpoint',
              pose4['J1'][1] > 15.0 and abs(pose4['J1'][0] - 15.0) > 0.5,
              f"{pose4['J1']}")

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
