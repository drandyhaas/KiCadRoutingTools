#!/usr/bin/env python3
"""#1979: the micro-jog weld keeps the longer neighbour's bearing.

`add_route_to_pcb_data` drops a segment no longer than 0.01 mm and, when it
bridges two neighbours, welds them. It moved BOTH neighbours' ends to the
bridge's midpoint. On a diff pair routed by route_diff (grid 0.1,
heuristic weight 1.3) the bridge is a 6.8 um jog between a 0.79 mm connector
and a 12.2 mm 45-degree leg, so the leg turned by 3.4 um, was no longer
parallel to its partner in KiCad's integer test, and the pair read 1.8 mm
coupled of 15.5 mm (KiCad agreed). Now the longer neighbour keeps its own
end and the shorter neighbour's end moves onto it.

  1. the issue's three segments: the leg keeps its exact 45 degrees and its
     end points, the connector takes the 6.8 um, the chain stays joined;
  2. the pair built from them reads the facing span of the legs coupled
     (`net_queries.diff_pair_coupling`, KiCad's measure), not the tail alone;
  3. what keeps the old result: two equally long legs meet at the jog's
     midpoint, a 0.02 mm segment is kept untouched, a micro segment at a
     route's end is dropped with its neighbour left alone;
  3b. two micro segments in a row: in line, the second takes the first's
     move and is kept as a 0.012 mm piece (the midpoint weld left 9 um
     between the legs); bent, both are dropped and the legs stay 8.49 um
     apart, under the 0.010 mm soft-joint gap (the midpoint weld left
     6.71 um);
  4. a same-net via at the joint changes nothing: the leg keeps its bearing;
  5. the issue's route: Z3's and Z11's legs are exact and the pairs read
     their coupled length (Z3 1.8 mm and Z11 15.6 mm before).

    python3 tests/test_1979_microjog_weld.py
"""
import json
import math
import os
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'rust_router'))
from run_utils import check, evidence  # noqa: E402
from kicad_parser import Pad, PCBData, Segment, Via  # noqa: E402
from pcb_modification import add_route_to_pcb_data  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FAILS = []
P, N = 1, 2
# The issue's pair: the centre line's first segment is a 0.1 mm stub that
# turns 45 degrees; offset by the 0.225 mm half pitch, the inner member's
# copy of it is 0.1 - 0.225 * tan(22.5 deg) = 0.0068 mm long, the outer's
# 0.1932 mm (create_parallel_path_from_float).
T = 0.225 * math.tan(math.radians(22.5))


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def seg(a, b, net=P, width=0.2, layer='F.Cu'):
    return Segment(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1],
                   width=width, layer=layer, net_id=net)


def pad(xy, net):
    return Pad(component_ref='U1', pad_number=f'{xy}', global_x=xy[0], global_y=xy[1],
               local_x=0.0, local_y=0.0, size_x=0.3, size_y=0.3, shape='rect',
               layers=['F.Cu'], net_id=net, net_name=f'n{net}')


def commit(segs, vias=()):
    """add_route_to_pcb_data on one result, a pad at each end of the chain
    (else the per-commit prune takes the whole chain as a dead end);
    returns what it kept."""
    net = segs[0].net_id
    pts = [(s.start_x, s.start_y) for s in segs] + [(s.end_x, s.end_y) for s in segs]
    ends_ = [q for q in pts if pts.count(q) == 1]
    pcb = PCBData(board_info=None, nets={}, footprints={}, vias=list(vias), segments=[],
                  pads_by_net={net: [pad(ends_[0], net), pad(ends_[-1], net)]})
    result = {'new_segments': list(segs), 'new_vias': []}
    add_route_to_pcb_data(pcb, result)
    return result['new_segments']


def ends(s):
    return (s.start_x, s.start_y), (s.end_x, s.end_y)


def p_member():
    """connector, 6.8 um jog, 12.2 mm leg, tail: the Z3_P copper."""
    return [seg((84.5, 91.3688), (84.675, 90.6)),
            seg((84.675, 90.6), (84.675, 90.5 + T)),
            seg((84.675, 90.5 + T), (76.075, 81.9 + T)),
            seg((76.075, 81.9 + T), (76.075, 80.0))]


def n_member():
    """Z3_N: the outer copy of the stub is 0.1932 mm, kept."""
    return [seg((85.3, 91.3688), (85.125, 90.6), N),
            seg((85.125, 90.6), (85.125, 90.5 - T), N),
            seg((85.125, 90.5 - T), (76.525, 81.9 - T), N),
            seg((76.525, 81.9 - T), (76.525, 80.0), N)]


def the_issue_segments():
    print("1. the issue's three segments")
    conn, jog, leg, tail = p_member()
    leg0 = ends(leg)
    kept = commit([conn, jog, leg, tail])
    ok(not any(s is jog for s in kept) and len(kept) == 3, f"the 6.8 um jog is dropped: {len(kept)} segments")
    dx, dy = leg.end_x - leg.start_x, leg.end_y - leg.start_y
    ok(abs(dx - dy) < 1e-9, f"the 12.2 mm leg keeps dx == dy: dx {dx:.7f} dy {dy:.7f}")
    ok(ends(leg) == leg0, f"the leg's end points do not move: {ends(leg)}")
    ok((conn.end_x, conn.end_y) == (leg.start_x, leg.start_y),
       f"the connector ends where the leg starts: {(conn.end_x, conn.end_y)}")
    moved = math.hypot(conn.end_x - 84.675, conn.end_y - 90.6)
    ok(abs(moved - (0.1 - T)) < 1e-12, f"the connector's end took the whole jog: {moved:.7f} mm")


def the_pair():
    print("2. the pair's coupled length, KiCad's measure")
    from net_queries import diff_pair_coupling
    p = commit(p_member())
    n = commit(n_member())
    ok(len(n) == 4, f"the partner has no micro segment and keeps its 4: {len(n)}")
    got = diff_pair_coupling(None, P, N, p_tracks=p, n_tracks=n,
                             obstruction=False)['coupled_mm']
    # the legs face each other over their length less their stagger along
    # the 45-degree line; the tails over the shorter one (1.9 - T)
    legs = 8.6 * math.sqrt(2) - (0.45 - 2 * T) / math.sqrt(2)
    want = legs + 1.9 - T
    ok(abs(got - want) < 1e-5,
       f"coupled {got:.4f} mm = legs {legs:.4f} + tails {1.9 - T:.4f} "
       f"(the midpoint weld read the tails alone, 1.8068)")


def today_kept():
    print("3. the cases that keep the old result")
    a, jog, b = seg((0, 0), (1, 0)), seg((1, 0), (1, 0.006)), seg((1, 0.006), (2, 0.006))
    commit([a, jog, b])
    ok((a.end_x, a.end_y) == (1, 0.003) and (b.start_x, b.start_y) == (1, 0.003),
       f"equally long legs meet at the jog's midpoint: {(a.end_x, a.end_y)} {(b.start_x, b.start_y)}")
    a, mid, b = seg((0, 0), (1, 0)), seg((1, 0), (1, 0.02)), seg((1, 0.02), (13, 12.02))
    before = [ends(s) for s in (a, mid, b)]
    kept = commit([a, mid, b])
    ok(len(kept) == 3 and [ends(s) for s in (a, mid, b)] == before,
       f"a 0.02 mm segment is kept and nothing moves: {len(kept)} segments")
    jog, leg = seg((0, 0), (0, 0.006)), seg((0, 0.006), (5, 5.006))
    kept = commit([jog, leg])
    ok(kept == [leg] and ends(leg) == ((0, 0.006), (5, 5.006)),
       f"a micro segment at a route's end is dropped, its neighbour left alone: {ends(leg)}")


def two_in_a_row():
    print("3b. two micro segments in a row between two legs")
    a, m1, m2, b = (seg((0, 0), (1, 0)), seg((1, 0), (1, 0.006)), seg((1, 0.006), (1, 0.012)),
                    seg((1, 0.012), (3, 2.012)))
    kept = commit([a, m1, m2, b])
    ok(not any(s is m1 for s in kept) and any(s is m2 for s in kept) and ends(a) == ((0, 0), (1, 0))
       and ends(m2) == ((1, 0), (1, 0.012)) and ends(b) == ((1, 0.012), (3, 2.012)),
       f"in line: the second takes the first's 6 um and stays, 0.012 mm, the legs untouched "
       f"(the midpoint weld left the legs 9 um apart): {[ends(s) for s in kept]}")
    # bent: after the first's weld the second still sees the first, so it is
    # dropped unwelded and the legs stay apart, inside the soft-joint gap
    from routing_constants import SOFT_JOINT_MIN_GAP
    a, m1, m2, b = (seg((0, 0), (1, 0)), seg((1, 0), (1.006, 0)), seg((1.006, 0), (1.006, 0.006)),
                    seg((1.006, 0.006), (3.006, 2.006)))
    kept = commit([a, m1, m2, b])
    gap = math.hypot(b.start_x - a.end_x, b.start_y - a.end_y)
    ok(len(kept) == 2 and ends(a) == ((0, 0), (1, 0)) and ends(b) == ((1.006, 0.006), (3.006, 2.006))
       and abs(gap - 0.006 * math.sqrt(2)) < 1e-12 and gap < SOFT_JOINT_MIN_GAP,
       f"bent: both dropped, the legs untouched and {gap * 1000:.2f} um apart, under the "
       f"{SOFT_JOINT_MIN_GAP} mm soft-joint gap (the midpoint weld left 6.71 um)")


def via_end():
    print("4. a via at the joint changes nothing")
    conn, jog, leg, tail = p_member()
    leg0 = ends(leg)
    via = Via(x=84.675, y=90.6, size=0.45, drill=0.2, layers=['F.Cu', 'B.Cu'], net_id=P)
    commit([conn, jog, leg, tail], vias=[via])
    dx, dy = leg.end_x - leg.start_x, leg.end_y - leg.start_y
    ok(ends(leg) == leg0 and abs(dx - dy) < 1e-9,
       f"with a same-net via at the connector's end the leg still keeps dx == dy: "
       f"dx {dx:.7f} dy {dy:.7f}")


def the_route(td):
    print("5. the issue's route_diff command")
    src = os.path.join(REPO, 'kicad_files', 'glasgow_revC.kicad_pcb')
    board_in, out = os.path.join(td, 'in.kicad_pcb'), os.path.join(td, 'out.kicad_pcb')
    summary = os.path.join(td, 'out.json')
    check(PY + [os.path.join(REPO, 'py_router', 'copy_board.py'),
                evidence(src, 'the board'), board_in], accept=True)
    check(PY + [os.path.join(REPO, 'py_router', 'route_diff.py'), board_in,
                '--output', out, '--nets', '/IO_Banks/Z*', '--grid-step', '0.1',
                '--heuristic-weight', '1.3', '--json-out', summary],
          accept=True, timeout=900)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        reps = {r['pair']: r for r in json.load(fh)['pair_reports']}
    for pair, floor, before in (('/IO_Banks/Z3', 13.0, 1.8068), ('/IO_Banks/Z11', 20.0, 15.6)):
        c = (reps.get(pair) or {}).get('copper') or {}
        ok((c.get('coupled_mm') or 0) > floor,
           f"{pair} reads {c.get('coupled_mm')} mm coupled of "
           f"{max(c.get('length_p_mm', 0), c.get('length_n_mm', 0)):.1f} (before {before})")
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
    ids = {n.name: i for i, n in pcb.nets.items()}
    legs, skew = {}, []
    for name in ('/IO_Banks/Z3_P', '/IO_Banks/Z3_N', '/IO_Banks/Z11_P', '/IO_Banks/Z11_N'):
        for s in pcb.segments:
            dx, dy = abs(s.end_x - s.start_x), abs(s.end_y - s.start_y)
            if s.net_id == ids[name] and math.hypot(dx, dy) > 5 and min(dx, dy) > 1e-6:
                legs[name] = max(legs.get(name, 0), round(math.hypot(dx, dy), 2))
                if abs(dx - dy) > 1e-5:
                    skew.append(f"{name} {dx:.4f} x {dy:.4f}")
    ok(legs.get('/IO_Banks/Z3_P', 0) > 12 and legs.get('/IO_Banks/Z11_P', 0) > 7,
       f"the welded members' diagonals are there (12.2 and 7.7 mm): {legs}")
    ok(not skew, f"and every diagonal over 5 mm of Z3 and Z11 is at 45 degrees: {skew}")


def main():
    the_issue_segments()
    the_pair()
    today_kept()
    two_in_a_row()
    via_end()
    with tempfile.TemporaryDirectory(prefix='t1979_') as td:
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
