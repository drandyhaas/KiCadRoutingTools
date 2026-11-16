#!/usr/bin/env python3
"""#1873: a hybrid terminal leg joins its coupled middle where the two meet.

route_diff's direct hybrid routes each terminal leg to the END of its net's
coupled middle. Where the terminal lies beside the middle short of that end,
the leg ran beside the middle, inside its copper, up to the end and the
signal came back down the middle: on glasgow_revC `/IO_Banks/Z9` the N member
laid 0.85 mm twice, 0.05 mm apart, and the pair read `matched` at 0.0006 mm
with that stretch counted twice in N's length.

`diff_pair_routing._fold_legs_onto_middle` cuts the middle and the leg where
the doubled stretch stops and joins them there. Checked:

  1. the issue's geometry: the middle loses its end stub and the doubled
     segment, the leg stops where the overlap began and jogs 0.05 mm onto
     the middle, one piece of copper, 0.85 mm folded;
  1b. a leg that leaves its own via beside where the overlap begins (Z0_P
     on the same route): the doubled stretch is folded and the via keeps its
     link to the leg;
  2. what is left alone: a leg that reaches the end without running along
     the middle, one on another layer, a middle whose tail carries the net's
     via, and an overlap that starts further up the middle than its end stub;
  3. the issue's route end to end: no Z* net has copper laid along itself,
     Z0's skew is read on copper counted once, and the other pairs' records
     are what they were. Since #1441 corrected the coupled middle's search,
     Z9 no longer routes as a pair here and its doubled leg is gone; Z0_P's,
     beside its via, is the fold this route exercises.
"""
import json
import os
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'rust_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def seg(a, b, layer='B.Cu', net=7, width=0.2):
    from kicad_parser import Segment
    return Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1], width=width,
                   layer=layer, net_id=net)


def middle():
    return [seg((80.8515, 92.5843), (80.5686, 92.5843)),
            seg((81.4172, 93.15), (80.8515, 92.5843)),
            seg((82.2672, 93.15), (81.4172, 93.15)),
            seg((82.3586, 93.2414), (82.2672, 93.15))]


def leg(layer='B.Cu'):
    return [seg((79.25, 92.7), (79.75, 93.2), layer), seg((79.75, 93.2), (82.3, 93.2), layer),
            seg((82.3, 93.2), (82.3586, 93.2414), layer)]


def pts(segs):
    return sorted(((round(s.start_x, 4), round(s.start_y, 4)),
                   (round(s.end_x, 4), round(s.end_y, 4))) for s in segs)


def the_fold():
    print("1. the issue's geometry")
    from diff_pair_routing import _fold_legs_onto_middle as fold
    from net_queries import net_self_overlap
    from types import SimpleNamespace
    m, l, got = fold(middle(), leg(), [], (7,))
    ok(abs(got.get(7, 0) - 0.85) < 1e-6, f"0.85 mm of doubled copper is folded: {got}")
    ok(pts(m) == pts(middle()[:2]),
       "the middle keeps its run up to the join and loses the doubled segment and end stub")
    ok(pts(l) == [((79.25, 92.7), (79.75, 93.2)), ((79.75, 93.2), (81.4172, 93.2)),
                  ((81.4172, 93.2), (81.4172, 93.15))],
       f"the leg stops where the overlap began and jogs onto the middle: {pts(l)}")
    before = SimpleNamespace(segments=middle() + leg())
    after = SimpleNamespace(segments=m + l)
    ok(net_self_overlap(before, 7)[0] > 0.8 and net_self_overlap(after, 7)[0] == 0.0,
       "net_self_overlap reads 0.85 mm before and none after")
    ends = {}
    for s in m + l:
        for k in ((round(s.start_x, 4), round(s.start_y, 4)),
                  (round(s.end_x, 4), round(s.end_y, 4))):
            ends[k] = ends.get(k, 0) + 1
    ok(sorted(k for k, n in ends.items() if n == 1) == [(79.25, 92.7), (80.5686, 92.5843)],
       "the copper is one chain from the terminal to the middle's far end")


def via_beside_the_join():
    print("1b. a leg from its own via beside where the overlap begins")
    from diff_pair_routing import _fold_legs_onto_middle as fold
    from kicad_parser import Via
    # glasgow_revC /IO_Banks/Z0_P: the leg leaves its fanout via 0.125 mm
    # from the middle and runs 0.025 mm beside it to the middle's end.
    mid = [seg((87.1, 91.9), (87.1, 90.0172))]
    lg = [seg((87.2, 91.175), (87.125, 91.25)), seg((87.125, 91.25), (87.125, 91.875)),
          seg((87.125, 91.875), (87.1, 91.9))]
    via = Via(x=87.2, y=91.175, size=0.4, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=7)
    m, l, got = fold(mid, lg, [via], (7,))
    ok(abs(got.get(7, 0) - 0.65) < 1e-6, f"the doubled 0.65 mm is folded: {got}")
    ok(pts(m) == [((87.1, 91.25), (87.1, 90.0172))], f"the middle starts at the join: {pts(m)}")
    ok(pts(l) == [((87.125, 91.25), (87.1, 91.25)), ((87.2, 91.175), (87.125, 91.25))],
       f"the via keeps its link and a jog joins it to the middle: {pts(l)}")


def left_alone():
    print("2. what is left alone")
    from diff_pair_routing import _fold_legs_onto_middle as fold
    from kicad_parser import Via
    direct = [seg((83.0, 94.0), (82.3586, 93.2414))]
    ok(fold(middle(), direct, [], (7,))[2] == {}, "a leg that arrives at the end from outside")
    ok(fold(middle(), leg('F.Cu'), [], (7,))[2] == {}, "a leg on another layer")
    via = Via(x=82.3586, y=93.2414, size=0.5, drill=0.25, layers=['F.Cu', 'B.Cu'], net_id=7)
    ok(fold(middle(), leg(), [via], (7,))[2] == {},
       "a middle whose end carries the net's via (the leg may use it)")
    other = Via(x=82.3586, y=93.2414, size=0.5, drill=0.25, layers=['F.Cu', 'B.Cu'], net_id=8)
    ok(fold(middle(), leg(), [other], (7,))[2] != {}, "another net's via does not hold it")
    far = middle() + [seg((82.3586, 93.2414), (83.4, 93.2414))]
    ok(fold(far, leg(), [], (7,))[2] == {},
       "an overlap that starts 1 mm up the middle from its end (not the end's turn back)")
    ok(fold(middle(), leg(), [], (9,))[2] == {}, "a net that is not asked for")


def the_route(td):
    print("3. glasgow_revC /IO_Banks/Z*, the issue's command")
    src = os.path.join(REPO, 'kicad_files', 'glasgow_revC.kicad_pcb')
    board, out = os.path.join(td, 'in.kicad_pcb'), os.path.join(td, 'out.kicad_pcb')
    summary = os.path.join(td, 'out.json')
    check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'),
                evidence(src, 'the board'), board], accept=True)
    r = check(PY + [os.path.join(REPO, 'py_router', 'route_diff.py'), board, out,
                    '--nets', '/IO_Banks/Z*', '--diff-pair-gap', '0.2', '--clearance', '0.1',
                    '--grid-step', '0.025', '--diff-pair-intra-match',
                    '--length-match-tolerance', '0.127', '--json-out', summary],
              accept=True, timeout=1800)
    log = r.stdout or ''
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    recs = {x['pair']: x for x in doc.get('intra_pair_matching') or []}
    ok('leg fold (#1873)' in log and 'net 114 joins its coupled middle 0.650 mm' in log,
       "the step says where it folded a leg (Z0_P, off its own via)")
    ok('Self-overlap:' not in log and not any(x.get('self_overlap_mm') for x in recs.values()),
       "no pair has copper laid along itself")
    z0 = recs.get('/IO_Banks/Z0') or {}
    ok(z0.get('status') == 'unmatched' and abs((z0.get('delta_mm') or 0) - 0.1431) < 1e-4,
       f"Z0's skew is read on copper counted once (1.31 mm with the stretch doubled): "
       f"{z0.get('status')} {z0.get('delta_mm')}")
    want = {'/IO_Banks/Z1': ('matched', 0.0006), '/IO_Banks/Z3': ('within-tolerance', 0.0155),
            '/IO_Banks/Z6': ('matched', 0.0006), '/IO_Banks/Z8': ('unmatched', 0.1362)}
    got = {k: (recs.get(k, {}).get('status'), recs.get(k, {}).get('delta_mm')) for k in want}
    ok(got == want, f"the other pairs' records are what they were: {got}")
    from kicad_parser import parse_kicad_pcb
    from net_queries import net_self_overlap
    pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
    laid = {n.name: net_self_overlap(pcb, n.net_id)[0] for n in pcb.nets.values()
            if n.name.startswith('/IO_Banks/Z')}
    ok(laid and not any(laid.values()),
       f"and no Z* net on the shipped board reads any: {[k for k, v in laid.items() if v]}")


def main():
    the_fold()
    via_beside_the_join()
    left_alone()
    with tempfile.TemporaryDirectory() as td:
        the_route(td)
    if FAILS:
        print(f"\nFAILED ({len(FAILS)}):")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nALL PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
