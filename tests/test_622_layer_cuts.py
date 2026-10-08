#!/usr/bin/env python3
"""The whole route's LAYER cuts from the loop's own evidence (whole_route.layer_cut, audit_layer_cuts, snap_layer_cuts).

  python3 tests/test_622_layer_cuts.py

A lane the audit finds against an island on one layer, or the snap cannot lay with its search stuck beside one, is held
on the other layer across that island (whole_solve reads the cut files' `lcuts`) -- a wall the lanes cannot go round.
On a geometry's islands of its own -- WALL on F alone, BARREL on both layers, FAR on B -- this pins:

1. a cut only for an island on one layer, to the other layer, carrying the island's box; none for one on both layers
   (no change answers it) and none where the finding's layer is not the island's;
2. an audit's STATIC findings against a pad or a hole of an island give its cut; a finding against other copper, or a
   pad of no island, gives none; each lane and island once;
3. a snap's failure stuck within SNAP_ISLAND_REACH of an island on its layer gives a cut under the NEAREST such
   island; one stuck farther, or beside an island on the other layer only, gives none.
"""
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'awx'))
sys.argv = sys.argv[:1]
import whole_route as wr  # noqa: E402

GEO = {'islands': {'C1.1': 'WALL', 'C1.2': 'WALL', 'H1.': 'WALL', 'TP1.1': 'BARREL', 'C9.1': 'FAR'},
       'island_boxes': {'WALL': [5.0, -3.0, 6.0, 3.0, [0]], 'BARREL': [5.0, 4.0, 6.0, 5.0, [0, 1]],
                        'FAR': [5.0, 8.0, 6.0, 9.0, [1]]}}

AUDIT = """STATIC SYN05   F -0.257/0.181 pad C1.1 0 at (4.91,2.74)
STATIC SYN05   F -0.079/0.181 pad C1.2 0 at (4.69,2.20)
STATIC SYN06   F +0.143/0.181 hole H1.  at (4.50,1.12)
STATIC SYN07   F -0.050/0.181 pad TP1.1 GND at (4.50,4.50)
STATIC SYN08   F -0.050/0.181 copper SYN09 at (3.00,0.00)
STATIC SYN09   F -0.050/0.181 pad U7.3 0 at (3.00,0.00)
STATIC 6 lane/object pair(s) short of their bar (track/2 + clearance 0.168, + half a grid step off the grid): {}
"""

SNAP = """snap: 9/12 lanes laid, 0 vias, 26 s
    SYN00   FAILED: no path in its band (54080 states searched); the farthest it got: 8.39 of 21.85 mm along it, at (4.20, -4.10) on F.Cu
    SYN11   FAILED: no path in its band (61033 states searched); the farthest it got: 8.28 of 19.95 mm along it, at (5.50, 12.50) on F.Cu
    SYN03   FAILED: no path in its band (1000 states searched); the farthest it got: 2.00 of 19.95 mm along it, at (5.50, 7.50) on F.Cu
    SYN04   FAILED: no path in its band (1000 states searched); the farthest it got: 2.00 of 19.95 mm along it, at (5.50, 7.50) on B.Cu
SNAP FAILED: 4 lane(s) could not be laid: SYN00, SYN11, SYN03, SYN04
"""


def main():
    print('=' * 60)
    print('the whole route\'s layer cuts from the audit and the snap')
    print('=' * 60)
    fails = []
    bx = GEO['island_boxes']
    # 1
    c = wr.layer_cut('SYN05', 'WALL', bx['WALL'], 0)
    if c != {'lane': 'SYN05', 'island': 'WALL', 'layer': 1, 'box': [5.0, -3.0, 6.0, 3.0]}:
        fails.append(f'one layer: {c}')
    for what, args in (('both layers', ('SYN05', 'BARREL', bx['BARREL'], 0)),
                       ('another layer', ('SYN05', 'WALL', bx['WALL'], 1)), ('no box', ('SYN05', 'X', None, 0))):
        if wr.layer_cut(*args) is not None:
            fails.append(f'{what}: a cut')
    with tempfile.TemporaryDirectory() as td:
        # 2
        pa = os.path.join(td, 'p.audit')
        open(pa, 'w').write(AUDIT)
        got = sorted((c['lane'], c['island'], c['layer']) for c in wr.audit_layer_cuts(pa, GEO))
        if got != [('SYN05', 'WALL', 1), ('SYN06', 'WALL', 1)]:
            fails.append(f'audit: {got}, want SYN05 and SYN06 under WALL once each (a hole too; not the barrel, '
                         f'other copper, nor a pad of no island)')
        # 3
        ps = os.path.join(td, 'snap.log')
        open(ps, 'w').write(SNAP)
        got = sorted((c['lane'], c['island'], c['layer']) for c in wr.snap_layer_cuts(ps, GEO))
        # SYN00 stuck on F 1.4 mm off WALL (F alone): under it, to B; SYN04 stuck on B 0.5 mm off FAR (B alone):
        # under it, to F; SYN11 stuck 3.5 mm off FAR, past the reach; SYN03 stuck on F beside FAR, on B alone, with
        # WALL 4.5 mm off
        if got != [('SYN00', 'WALL', 1), ('SYN04', 'FAR', 0)]:
            fails.append(f'snap: {got}, want SYN00 under WALL (to B) and SYN04 under FAR (to F)')
        if not 1.4 < wr.SNAP_ISLAND_REACH < 3.5:
            raise SystemExit(f'BROKEN TEST: the reach {wr.SNAP_ISLAND_REACH} is outside the distances it was drawn for')
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: a cut only under an island on one layer, to the other; the audit\'s pad and hole findings against an '
          'island, each lane once; a snap failure under the nearest one-layer island on its layer within reach')
    return 0


if __name__ == '__main__':
    sys.exit(main())
