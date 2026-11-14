#!/usr/bin/env python3
"""#1873 (the disclosure), three cases it missed and one place it did not
reach.

`net_queries.net_self_overlap` asked that two passes stay within a track
width over the WHOLE length they run side by side, and compared their centre
lines with the NARROWER track's width. So:

  * a pass that comes back at 2.9 degrees, its copper fused with the first
    for 4 mm before the two part, read 0;
  * a 0.1 mm pass lying inside a 0.4 mm track of the net read 0;
  * a member over the segment limit, or one the search raised on, left no
    key in the pair's record, which reads the same as "none found".

And the record that says `matched` (`intra_pair_matching[]`) did not carry
the figure; only the pair's copper record did.

Now the stretch counted is where the two pieces' copper overlaps (centre
lines closer than half the two widths together), a member not searched is
named (`self_overlap_not_searched`, `self_overlap_error`), and the
intra-pair record carries the same keys beside its status.

Rows (in memory, F.Cu):
  1. converging passes: counted over the stretch where their copper meets;
     past 5 degrees, not at all;
  2. unequal widths: a narrow pass inside a wide track is counted, and one
     that only overlaps its edge; beside it with copper apart, not;
  3. equal widths read what they read: the hairpin of
     test_1873_self_overlap_disclosed.py, and two passes a width apart;
  4. the pair's copper record: a member over the limit, and a search that
     raises, are said; beside a partner that has some, the member not
     searched reads None, not 0;
  5. the intra-pair record: `regrade_intra_pair_reports` copies the figure
     beside `status`, and the two "not searched" keys with it; a clean
     pair's record has none of them.

    python3 tests/test_1873_self_overlap_edges.py
"""
import contextlib
import io
import os
import sys
from types import SimpleNamespace

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


def near(got, want, tol=2e-3):
    return got is not None and abs(got - want) <= tol


def measure_rows():
    print("1. converging passes (0.2 mm tracks)")
    # out along y = 0 and back to (0, 0.5): 2.86 degrees apart, the centre
    # lines closer than 0.2 mm for the 4.005 mm nearest the turn
    total, rows = net_self_overlap(board([(0, 0, 10, 0), (10, 0, 0, 0.5)]), 1)
    ok(near(total, 4.005) and len(rows) == 1,
       f"the 4.005 mm where their copper meets, one row: {total}, {len(rows)} row(s)")
    ok(rows and rows[0]['centre_gap_mm'] == 0.0 and near(rows[0]['along_mm'], 4.005),
       f"narrowest at the turn, where the centre lines meet: {rows[:1]}")
    # 1.09 degrees: never 0.2 mm apart, so all 10 mm
    total, _rows = net_self_overlap(board([(0, 0, 10, 0), (10, 0, 0, 0.19)]), 1)
    ok(near(total, 10.0), f"a pass that stays inside a width: all of it, {total}")
    # 6.3 degrees is a bend, not a pass back
    total, rows = net_self_overlap(board([(0, 0, 10, 0), (10, 0, 0, 1.1)]), 1)
    ok(total == 0.0 and rows == [], f"past 5 degrees: {total}")
    # parallel for 3 mm, then away at 4.5 degrees in a second piece
    total, rows = net_self_overlap(board(
        [(0, 0, 10, 0), (10, 0, 10, 0.05), (10, 0.05, 7, 0.05), (7, 0.05, 0, 0.6)]), 1)
    ok(near(total, 3.0 + 1.917, 5e-3) and len(rows) == 2,
       f"3 mm side by side, then the 1.92 mm before the second piece is clear: "
       f"{total}, {len(rows)} row(s)")

    print("2. unequal widths (a 0.4 mm track and a 0.1 mm pass)")
    wide = {'width': 0.4}
    thin = {'width': 0.1}
    total, rows = net_self_overlap(board(
        [(0, 0, 10, 0, wide), (10, 0, 10, 0.15, thin), (10, 0.15, 5, 0.15, thin)]), 1)
    ok(near(total, 5.0) and rows and rows[0]['centre_gap_mm'] == 0.15,
       f"the pass inside the wide track's copper, 0.15 mm off its centre: {total}")
    # half the two widths together is 0.25 mm: 0.24 still overlaps the edge
    total, rows = net_self_overlap(board(
        [(0, 0, 10, 0, wide), (10, 0, 10, 0.24, thin), (10, 0.24, 5, 0.24, thin)]), 1)
    ok(near(total, 5.0) and rows and rows[0]['centre_gap_mm'] == 0.24,
       f"0.24 mm off its centre, overlapping the wide track's edge by 0.01 mm: {total}")
    total, rows = net_self_overlap(board(
        [(0, 0, 10, 0, wide), (10, 0, 10, 0.3, thin), (10, 0.3, 5, 0.3, thin)]), 1)
    ok(total == 0.0 and rows == [],
       f"0.3 mm off its centre, 0.05 mm of gap between the two: {total}")
    # a branch that leaves forward at 2.9 degrees, not a pass back
    total, rows = net_self_overlap(board([(0, 0, 10, 0), (5, 0, 15, 0.5)]), 1)
    ok(near(total, 4.005) and len(rows) == 1,
       f"a branch leaving forward at 2.9 degrees, while the two overlap: {total}")

    print("3. equal widths read what they read")
    hairpin = [(0, 0.05, 2.55, 0.05), (2.55, 0.05, 2.6, 0.0), (2.6, 0.0, 1.7, 0.0),
               (1.7, 0.0, 0.95, -0.75)]
    total, rows = net_self_overlap(board(hairpin), 1)
    ok(total == 0.85 and len(rows) == 1 and rows[0]['centre_gap_mm'] == 0.05
       and rows[0]['along_mm'] == 0.85, f"the hairpin: {total}, {rows[:1]}")
    total, rows = net_self_overlap(
        board([(0, 0, 4, 0), (4, 0, 4, 0.2), (4, 0.2, 0, 0.2)]), 1)
    ok(total == 0.0 and rows == [], f"two passes a width apart: {total}")
    return hairpin


def record_rows(hairpin):
    print("4. the pair's copper record says what it did not search")
    from diff_pair_custody import measure_pair_copper
    pair = SimpleNamespace(p_net_id=1, n_net_id=2, p_net_name='N1', n_net_name='N2')
    res = {1: {'is_diff_pair': True}, 2: {'is_diff_pair': True}}
    partner = [(0, 0.45, 2.6, 0.45, {'net_id': 2})]
    pcb = board(hairpin + partner, nets=(1, 2))

    def record():
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            rec = measure_pair_copper(pcb, [('X', pair)], res, lambda _p: (0.2, 0.2))
        return rec.get('X') or {}, out.getvalue()

    keep = net_queries.SELF_OVERLAP_MAX_SEGMENTS
    net_queries.SELF_OVERLAP_MAX_SEGMENTS = 3       # P has 4 pieces, N has 1
    try:
        rec, _said = record()
    finally:
        net_queries.SELF_OVERLAP_MAX_SEGMENTS = keep
    ok(rec.get('self_overlap_not_searched') == ['p'] and 'self_overlap_mm' not in rec,
       f"a member over the limit is named, and no length is claimed for the pair: "
       f"{rec.get('self_overlap_not_searched')}, {rec.get('self_overlap_mm')}")

    # the other way round: N (5 pieces, with a pass back) is searched, P
    # (6 pieces) is not. P must not read 0.0 beside N's length.
    long_p = [(0, 2 + 0.01 * i, 1, 2 + 0.01 * i) for i in range(0, 60, 10)]
    hairpin_n = [(s[0], s[1] + 1.0, s[2], s[3] + 1.0, {'net_id': 2}) for s in hairpin]
    mixed = board(long_p + hairpin_n + [(0, 1.45, 2.6, 1.45, {'net_id': 2})],
                  nets=(1, 2))
    net_queries.SELF_OVERLAP_MAX_SEGMENTS = 5
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            rec = (measure_pair_copper(mixed, [('X', pair)], res,
                                       lambda _p: (0.2, 0.2)).get('X') or {})
    finally:
        net_queries.SELF_OVERLAP_MAX_SEGMENTS = keep
    ok(rec.get('self_overlap_not_searched') == ['p']
       and rec.get('self_overlap_mm') == {'p': None, 'n': 0.85},
       f"beside a partner with 0.85 mm, the member not searched reads None: "
       f"{rec.get('self_overlap_mm')}")

    real = net_queries.net_self_overlap

    def boom(_pcb, _nid):
        raise RuntimeError('probe')
    net_queries.net_self_overlap = boom
    try:
        rec, said = record()
    finally:
        net_queries.net_self_overlap = real
    ok(rec.get('self_overlap_error') == 'RuntimeError: probe'
       and 'could not search X for copper laid along itself' in said
       and rec.get('coupled_mm') is not None and 'coupled' in rec,
       f"a search that raises is said, and the rest of the record stands: "
       f"{rec.get('self_overlap_error')}")
    rec, said = record()
    ok(rec.get('self_overlap_mm') == {'p': 0.85, 'n': 0.0}
       and 'self_overlap_not_searched' not in rec and 'self_overlap_error' not in rec
       and 'could not search' not in said, "the searched pair carries the length alone")
    return rec


def intra_rows(rec):
    print("5. the intra-pair record carries it beside the status")
    from length_matching import regrade_intra_pair_reports

    def regraded(copper):
        reports = [{'pair': 'X', 'status': 'matched', 'delta_mm': 0.0,
                    'tolerance_mm': 5.0}]
        # the in-memory pair has no pads, so its record has no skew of its own
        regrade_intra_pair_reports(reports, {'X': dict(copper, skew_mm=0.0006)},
                                   None, [], 5.0)
        return reports[0]

    x = regraded(rec)
    ok(x['status'] == 'matched' and x.get('self_overlap_mm') == {'p': 0.85, 'n': 0.0},
       f"matched, with the 0.85 mm its skew counts twice: {x.get('self_overlap_mm')}")
    x = regraded({'self_overlap_mm': {'p': None, 'n': 0.85},
                  'self_overlap_not_searched': ['p']})
    ok(x.get('self_overlap_not_searched') == ['p']
       and x.get('self_overlap_mm') == {'p': None, 'n': 0.85},
       f"a member not searched is named there too: {x.get('self_overlap_not_searched')}")
    x = regraded({'self_overlap_error': 'RuntimeError: probe'})
    ok(x.get('self_overlap_error') == 'RuntimeError: probe' and 'self_overlap_mm' not in x,
       f"and a search that failed: {x.get('self_overlap_error')}")
    y = regraded({})
    ok(y['status'] == 'matched'
       and not [k for k in y if k.startswith('self_overlap')],
       "a clean pair's record has none of the three keys")


def main():
    hairpin = measure_rows()
    rec = record_rows(hairpin)
    intra_rows(rec)
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
