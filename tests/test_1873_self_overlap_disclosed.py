#!/usr/bin/env python3
"""#1873 (the disclosure): a pair member whose copper runs back along itself
is named, with the length laid twice.

route_diff's hybrid escape can lay a member's terminal leg back along its
own coupled middle: on glasgow_revC `/IO_Banks/Z9`, the N member runs east
along y 93.20 and west again along y 93.15, 0.05 mm apart on 0.2 mm tracks,
for 0.85 mm. The intra-pair status is graded on all of a member's copper (as
KiCad's skew rule is), so that stretch counts twice and the pair is reported
`matched` at 0.0006 mm. Nothing named the shape.

`net_queries.net_self_overlap` measures it, and the pair step's copper
record (`pair_reports[].copper.self_overlap_mm`) and console carry it. This
is report-only: the leg itself is routed as before.

Rows (in memory, F.Cu, 0.2 mm tracks):
  1. a track that turns and comes back 0.05 mm beside itself for 0.85 mm:
     0.85 mm, one row, the two passes named;
  2. shapes that are not a self-overlap, each 0: a straight continuation, a
     45 degree bend, a T-junction, two passes a full width apart, the same
     two passes on different layers, a run shorter than a width;
  3. an exact duplicate segment is its whole length;
  4. a net over the segment limit is not searched (None);
  5. the pair record: `measure_pair_copper` adds `self_overlap_mm` for the
     member that has it, and no key at all for a clean pair.
"""
import os
import sys

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from synth import make_seg, make_net, make_pcb   # noqa: E402
from kicad_parser import BoardInfo               # noqa: E402
import net_queries                               # noqa: E402
from net_queries import net_self_overlap         # noqa: E402

FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def board(segs, nets=(1,)):
    return make_pcb(nets={n: make_net(n, f'N{n}') for n in nets},
                    segments=[make_seg(*s[:4], **(s[4] if len(s) > 4 else {})) for s in segs],
                    pads_by_net={n: [] for n in nets},
                    board_info=BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'},
                                         copper_layers=['F.Cu', 'B.Cu'],
                                         board_bounds=(-1.0, -1.0, 12.0, 5.0)),
                    zones=[])


def main():
    print("1. a track that comes back along itself")
    hairpin = [(0, 0.05, 2.55, 0.05), (2.55, 0.05, 2.6, 0.0), (2.6, 0.0, 1.7, 0.0),
               (1.7, 0.0, 0.95, -0.75)]
    total, rows = net_self_overlap(board(hairpin), 1)
    ok(total == 0.85 and len(rows) == 1,
       f"0.85 mm laid twice, one row: {total}, {len(rows)} row(s)")
    ok(rows and rows[0]['centre_gap_mm'] == 0.05 and rows[0]['along_mm'] == 0.85
       and rows[0]['layer'] == 'F.Cu',
       f"the passes 0.05 mm apart for 0.85 mm: {rows[:1]}")

    print("2. shapes that are not one")
    for tag, segs in (
            ('a straight continuation', [(0, 0, 2, 0), (2, 0, 5, 0)]),
            ('a 45 degree bend', [(0, 0, 2, 0), (2, 0, 4, 2)]),
            ('a T-junction', [(0, 0, 4, 0), (2, 0, 2, 3)]),
            ('two passes a width apart', [(0, 0, 4, 0), (4, 0, 4, 0.2), (4, 0.2, 0, 0.2)]),
            ('two passes on different layers',
             [(0, 0, 4, 0), (0, 0.05, 4, 0.05, {'layer': 'B.Cu'})]),
            ('a run shorter than a width', [(0, 0, 2, 0), (1.85, 0.05, 2.0, 0.05)])):
        total, rows = net_self_overlap(board(segs), 1)
        ok(total == 0.0 and rows == [], f"{tag}: {total}")

    print("3. an exact duplicate")
    total, rows = net_self_overlap(board([(0, 0, 3, 0), (0, 0, 3, 0)]), 1)
    ok(total == 3.0, f"its whole length: {total}")

    print("4. a net over the limit")
    keep = net_queries.SELF_OVERLAP_MAX_SEGMENTS
    net_queries.SELF_OVERLAP_MAX_SEGMENTS = 3
    try:
        total, rows = net_self_overlap(board(hairpin), 1)
    finally:
        net_queries.SELF_OVERLAP_MAX_SEGMENTS = keep
    ok(total is None and rows == [], "is not searched")

    print("5. the pair's copper record")
    from types import SimpleNamespace
    from diff_pair_custody import measure_pair_copper
    pair = SimpleNamespace(p_net_id=1, n_net_id=2, p_net_name='N1', n_net_name='N2')
    res = {1: {'is_diff_pair': True}, 2: {'is_diff_pair': True}}
    partner = [(0, 0.45, 2.6, 0.45, {'net_id': 2})]
    rec = measure_pair_copper(board(hairpin + partner, nets=(1, 2)), [('X', pair)], res,
                              lambda _p: (0.2, 0.2)).get('X') or {}
    ok(rec.get('self_overlap_mm') == {'p': 0.85, 'n': 0.0},
       f"names the member and the length: {rec.get('self_overlap_mm')}")
    clean = [(0, 0.05, 2.6, 0.05)]
    rec = measure_pair_copper(board(clean + partner, nets=(1, 2)), [('X', pair)], res,
                              lambda _p: (0.2, 0.2)).get('X') or {}
    ok(rec and 'self_overlap_mm' not in rec, "a clean pair carries no such key")
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
