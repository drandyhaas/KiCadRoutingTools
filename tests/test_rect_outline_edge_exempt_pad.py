#!/usr/bin/env python3
"""check_drc accepts pad-covered edge copper on a RECTANGULAR outline too.

A track that runs into an edge-exempt pad sitting inside the board's edge rule
adds no new edge copper, so check_drc publishes it `accepted: edge-exempt-pad`
(#441: the router opens such a pad's own cells on purpose,
obstacle_map._edge_band_pad_layer_exemption). That acceptance lived only on
the polygon path. A plain 4-segment rectangle gets no ring from
kicad_parser.extract_board_contours, so it was graded on a bounding-box path
with no acceptance, and the same track counted as real DRC: one board's 5
tracks into its edge-launch SMA pins. The rectangle is now given to tracks
and vias AS A RING, so both outlines take one path.

Each arm grades the SAME three tracks on a 4-gr_line rectangle and on a
5-segment polygon (the rectangle with its far corner chamfered):
  * COVERED  -- into a pad whose copper is already nearer the edge: accepted;
  * BARE     -- in the edge band with no pad under it: counted (control);
  * OFF-BOARD -- past the outline over a pad that overhangs it: counted, the
    acceptance never applies off the board (control).
Preconditions: the parser returns no ring for the rectangle and one for the
polygon, and check_drc says which path it graded on.

    python3 tests/test_rect_outline_edge_exempt_pad.py
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check, evidence  # noqa: E402
from kicad_parser import extract_board_contours  # noqa: E402

CHECK_DRC = os.path.join(ROOT, 'py_router', 'check_drc.py')

fails = []


def ok(name, cond, detail=''):
    print(('PASS: ' if cond else 'FAIL: ') + name
          + (f'  {detail}' if detail and not cond else ''))
    if not cond:
        fails.append(name)


def _outline(pts):
    return "\n".join(
        f' (gr_line (start {pts[i][0]} {pts[i][1]}) '
        f'(end {pts[(i + 1) % len(pts)][0]} {pts[(i + 1) % len(pts)][1]}) '
        f'(layer "Edge.Cuts") (width 0.1))' for i in range(len(pts)))


RECT = [(0, 0), (20, 0), (20, 10), (0, 10)]
# The same board with its far corner chamfered: 5 segments, so the parser
# returns a ring. Nothing near the copper below moves.
POLY = [(0, 0), (20, 0), (20, 9), (19, 10), (0, 10)]

# J1.1: 1x1 mm pad, centre 0.6 mm from the x=0 edge (copper 0.1 mm from it),
# inside the 0.5 mm edge rule -- an edge-launch connector pin.
# J2.1: the same pad overhanging the outline (copper from x=-0.3).
PARTS = '''
 (footprint "t:edge_pin" (layer "F.Cu") (at 0.6 5)
  (property "Reference" "J1" (at 0 -2) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste" "F.Mask") (net 1 "SIG"))
 )
 (footprint "t:edge_pin" (layer "F.Cu") (at 0.2 8)
  (property "Reference" "J2" (at 0 -2) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste" "F.Mask") (net 2 "SIG2"))
 )'''

# Track ending inside J1.1, 0.4 mm from the edge: 0.2 mm into the band, all of
# it under pad copper that is already 0.1 mm from the edge.
COVERED = ' (segment (start 0.4 5) (end 3 5) (width 0.2) (layer "F.Cu") (net 1) (uuid "c"))'
# The same depth into the band with no pad anywhere near.
BARE = ' (segment (start 0.3 2) (end 3 2) (width 0.2) (layer "F.Cu") (net 1) (uuid "b"))'
# Over the overhanging J2.1 and on past the outline.
OFF = ' (segment (start 2 8) (end -0.2 8) (width 0.2) (layer "F.Cu") (net 2) (uuid "o"))'


def _board(outline, tracks):
    return (f'(kicad_pcb\n (version 20221018)\n (net 0 "")\n (net 1 "SIG")\n'
            f' (net 2 "SIG2")\n{_outline(outline)}\n{PARTS}\n'
            + "\n".join(tracks) + '\n)')


def _grade(tmp, tag, outline, tracks, counted):
    pcb = os.path.join(tmp, f'{tag}.kicad_pcb')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(_board(outline, tracks))
    out = os.path.join(tmp, f'{tag}.json')
    argv = [sys.executable, '-X', 'utf8', CHECK_DRC, pcb, '--clearance', '0.2',
            '--board-edge-clearance', '0.5', '--json', out]
    if counted:
        r = check(argv, refuse='SEGMENT-BOARD-EDGE', code=1)
    else:
        r = check(argv, accept=True)
    evidence(out, f'{tag} check_drc --json')
    with open(out, encoding='utf-8') as f:
        doc = json.load(f)
    edge = [i for i in doc['items'] if i['type'] == 'segment-board-edge']
    by = {}
    for i in edge:
        x0 = i['seg_loc'][0]
        key = {0.4: 'covered', 0.3: 'bare', 2: 'off'}.get(x0, str(i['seg_loc']))
        by[key] = i
    return r, by


def main():
    # -- preconditions: which path each outline takes --------------------------
    rect_outers, _ = extract_board_contours(_board(RECT, []))
    poly_outers, _ = extract_board_contours(_board(POLY, []))
    ok('precondition: the parser returns NO ring for the 4-segment rectangle',
       rect_outers == [], rect_outers)
    ok('precondition: the parser returns a ring for the chamfered polygon',
       len(poly_outers) == 1 and len(poly_outers[0]) >= 5, poly_outers)

    with tempfile.TemporaryDirectory() as tmp:
        for name, outline, path in (('rect', RECT, 'bounding box'),
                                    ('poly', POLY, 'real Edge.Cuts outline')):
            # Covered track alone: accepted, so check_drc exits 0.
            r, by = _grade(tmp, f'{name}_covered', outline, [COVERED], counted=False)
            ok(f'{name}: graded on the {path} path',
               f'Checking board edge clearances ({path})' in r.stdout,
               r.stdout[-400:])
            c = by.get('covered')
            ok(f'{name}: the covered track IS inside the edge rule (strict geometry)',
               c is not None and c.get('overlap_mm', 0) > 0.15, c)
            ok(f'{name}: ... and is published accepted: edge-exempt-pad',
               c is not None and c.get('accepted') == 'edge-exempt-pad', c)

            # All three: the two controls count, the covered one stays accepted.
            r, by = _grade(tmp, f'{name}_all', outline, [COVERED, BARE, OFF],
                           counted=True)
            ok(f'{name}: control -- a BARE track in the band still counts',
               'bare' in by and not by['bare'].get('accepted'), by.get('bare'))
            ok(f'{name}: control -- an OFF-BOARD track over an overhanging pad '
               f'still counts (off-board, never accepted)',
               'off' in by and not by['off'].get('accepted')
               and by['off'].get('edge') == 'off-board', by.get('off'))
            ok(f'{name}: the covered track stays accepted beside them',
               by.get('covered', {}).get('accepted') == 'edge-exempt-pad',
               by.get('covered'))
            ok(f'{name}: exactly 2 counted edge items',
               r.stdout.count('too close to board edge')
               + r.stdout.count('off the board / in a cutout') == 2,
               r.stdout[-600:])

    if fails:
        print(f'{len(fails)} FAILURE(S): {fails}')
        return 1
    print('all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
