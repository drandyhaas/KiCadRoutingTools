#!/usr/bin/env python3
"""#1978: a loop of routed copper that its own via shorts is cut out.

A coupled pair turned back right after a layer change, and its inner member's
return leg passed over that member's own via, its copper on the via's ring:
the copper from the via out to the U-turn and back is a loop whose two ends
are one via. No signal travels it, but KiCad's all-copper length counts it.
The post-route cleanup kept it (the cycle prune joins a via only to segment
ENDS and removes only whole segments, and the return leg is load-bearing on
its far side), and the shipped re-match (#1622) then lengthened the OTHER
member to balance it: reported matched at 0.002 mm, the pad-to-pad paths
1.79 mm apart.

`pcb_modification.excise_via_shorted_loops` (in the cleanup pipeline after
the cycle prune) removes the loop's chain and shortens the passing track to
the foot of the via's centre.

Rows:
  1. a hand-built loop: the chain goes, the passing track ends at the foot,
     the net stays connected, the pad-to-pad path is unchanged and the
     all-copper length drops by the loop, no longer over the path;
  2. controls left exactly as they were: the same copper locked, the same
     copper not routed this run, a third track teeing into the loop, and a
     track passing its via with both sides going to pads (no loop);
  3. a coupled pair routed with intra-pair matching on an in-repo board,
     where the inner member turns back across its own via: the shipped
     member paths agree within the tolerance, the all-copper skew does too,
     the record is matched without a re-match, and no member track passes
     over its own via.

    python3 tests/test_1978_via_shorted_loop.py
"""
import contextlib
import io
import json
import math
import os
import re
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import BoardInfo, parse_kicad_pcb               # noqa: E402
from synth import make_net, make_pad, make_pcb, make_seg, make_via  # noqa: E402
from run_utils import check as run_check, evidence, tool          # noqa: E402
from check_connected import check_net_connectivity                # noqa: E402
from geometry_utils import via_copper_layers                      # noqa: E402
import net_queries as nq                                          # noqa: E402
import pcb_modification as pm                                     # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


W = 0.3


def fixture(lock=False, branch=False, no_loop=False):
    """Pad A (B.Cu) -> B.Cu track -> via V (5, 0) -> F.Cu: out 0.8 mm past V,
    a U-turn, and back west along y=0.25 (the centreline on V's 0.25 mm
    ring) to pad B. Returns (pcb, chain, passing track)."""
    bi = BoardInfo(layers={0: 'F.Cu', 31: 'B.Cu'}, copper_layers=['F.Cu', 'B.Cu'],
                   board_bounds=(-5.0, -5.0, 15.0, 5.0))
    pads = [make_pad(1, 10, -0.1, ref='U1', layers=('B.Cu',)),
            make_pad(1, 1, 0.25, ref='U2', layers=('F.Cu',))]
    segs = [make_seg(10, -0.1, 5, -0.1, layer='B.Cu', width=W, locked=lock)]
    if no_loop:
        pads.append(make_pad(1, 6, 0.25, ref='U3', layers=('F.Cu',)))
        chain = []
        passing = make_seg(6, 0.25, 1, 0.25, width=W, locked=lock)
        segs.append(make_seg(5, -0.1, 5, -0.6, width=W, locked=lock))
        pads.append(make_pad(1, 5, -0.6, ref='U4', layers=('F.Cu',)))
    else:
        chain = [make_seg(5, -0.1, 5.8, -0.1, width=W, locked=lock),
                 make_seg(5.8, -0.1, 5.95, 0.05, width=W, locked=lock),
                 make_seg(5.95, 0.05, 5.95, 0.1, width=W, locked=lock),
                 make_seg(5.95, 0.1, 5.8, 0.25, width=W, locked=lock)]
        passing = make_seg(5.8, 0.25, 1, 0.25, width=W, locked=lock)
    segs += chain + [passing]
    if branch:
        pads.append(make_pad(1, 8, 0.1, ref='U3', layers=('F.Cu',)))
        segs.append(make_seg(5.95, 0.1, 8, 0.1, width=W, locked=lock))
    pcb = make_pcb(nets={1: make_net(1, 'S')}, segments=segs,
                   vias=[make_via(5, 0, size=0.5)], pads_by_net={1: pads},
                   board_info=bi)
    return pcb, chain, passing


def sigs(pcb):
    return sorted((s.layer, round(s.start_x, 4), round(s.start_y, 4),
                   round(s.end_x, 4), round(s.end_y, 4)) for s in pcb.segments)


def excise(pcb, routed):
    results = [{'new_segments': list(pcb.segments) if routed else []}]
    with contextlib.redirect_stdout(io.StringIO()):
        n, removed = pm.excise_via_shorted_loops(results, pcb, {1})
    return n, removed, results


def unit_rows():
    print("hand-built loop:")
    pcb, chain, passing = fixture()
    a, b = pcb.pads_by_net[1]
    path0 = nq.pin_pair_path_length(pcb, 1, a, b)
    total0 = nq.net_match_length(pcb, 1, 'total')
    n, removed, results = excise(pcb, True)
    check('one loop cut out, its 4 chain segments removed', (n, removed) == (1, 4),
          f'{n}, {removed}')
    gone = {id(c) for c in chain}
    check('the chain is off the board and off the write list',
          not any(id(s) in gone for s in pcb.segments)
          and not any(id(s) in gone for s in results[0]['new_segments']))
    check('the passing track now starts at the foot of the via centre (5, 0.25)',
          math.hypot(passing.start_x - 5, passing.start_y - 0.25) < 1e-9
          and (passing.end_x, passing.end_y) == (1, 0.25),
          f'({passing.start_x}, {passing.start_y})')
    conn = check_net_connectivity(1, pcb.segments, pcb.vias, pcb.pads_by_net[1])
    check('the net stays connected', conn.get('connected') is True)
    path1 = nq.pin_pair_path_length(pcb, 1, a, b)
    check('the pad-to-pad path is unchanged',
          path0 is not None and path1 is not None and abs(path1 - path0) < 1e-9,
          f'{path0} -> {path1}')
    total1 = nq.net_match_length(pcb, 1, 'total')
    loop = 0.8 + 0.8 + 2 * math.hypot(0.15, 0.15) + 0.05
    check(f'the all-copper length drops by at least the loop ({loop:.3f} mm)',
          total0 is not None and total1 is not None
          and total0 - total1 >= loop - 1e-6, f'{total0} -> {total1}')
    check('the all-copper length was over the path by the loop, and is not now',
          total0 > path0 + 1.0 and total1 <= path1 + 1e-6,
          f'total {total0} -> {total1}, path {path1}')

    print("controls (left exactly as they were):")
    for label, kw, routed in (('the same copper locked', dict(lock=True), True),
                              ('the same copper not routed this run', {}, False),
                              ('a third track teeing into the loop',
                               dict(branch=True), True),
                              ('a track passing its via, both sides to pads',
                               dict(no_loop=True), True)):
        pcb, _chain, _passing = fixture(**kw)
        before = sigs(pcb)
        n, removed, _r = excise(pcb, routed)
        check(f'control: {label}', n == 0 and removed == 0 and sigs(pcb) == before,
              f'{n} loop(s), {removed} segment(s)')


def passes_own_via(pcb, nid):
    """Member tracks whose copper overlaps their own via away from its ends."""
    out = []
    for v in [v for v in pcb.vias if v.net_id == nid]:
        layers = via_copper_layers(v, pcb.board_info.copper_layers)
        for s in pcb.segments:
            if s.net_id != nid or s.layer not in layers:
                continue
            d = pm._pt_seg_dist(v.x, v.y, s.start_x, s.start_y, s.end_x, s.end_y)
            near_end = min(math.hypot(s.start_x - v.x, s.start_y - v.y),
                           math.hypot(s.end_x - v.x, s.end_y - v.y))
            if d < v.size / 2 + s.width / 2 - 1e-4 and near_end > v.size / 2 + 1e-3:
                out.append(s)
    return out


def route_row():
    print("coupled pair turning back across its own via:")
    board = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
    evidence(board, 'the in-repo board')
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'out.kicad_pcb')
        r = run_check([sys.executable, '-X', 'utf8', tool('route_diff.py'), board, out,
                       '--nets', 'USB_FTDI_D*', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu',
                       'B.Cu', '--grid-step', '0.1', '--heuristic-weight', '1.3',
                       '--diff-pair-intra-match'], accept=True)
        evidence(out, 'the routed board')
        m = re.search(r'^JSON_SUMMARY: (.*)$', r.stdout, re.M)
        check('the run printed its JSON summary', m is not None)
        if m is None:
            return
        rec = json.loads(m.group(1))['intra_pair_matching'][0]
        tol = rec['tolerance_mm']
        check('the cleanup cut out a via-shorted loop',
              'Via-shorted loops: cut out 1 loop(s)' in r.stdout)
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            pcb = parse_kicad_pcb(out)
        ids = {n.name: i for i, n in pcb.nets.items()}
        path, total = {}, {}
        for nm in ('USB_FTDI_D+', 'USB_FTDI_D-'):
            nid = ids[nm]
            pads = nq._copper_pads(pcb, nid)
            path[nm] = nq.pin_pair_path_length(pcb, nid, pads[0], pads[1])
            total[nm] = nq.net_match_length(pcb, nid, 'total')
            check(f'no {nm[-1]} member track passes over its own via',
                  not passes_own_via(pcb, nid), str(len(passes_own_via(pcb, nid))))
        ps = abs(path['USB_FTDI_D+'] - path['USB_FTDI_D-'])
        ts = abs(total['USB_FTDI_D+'] - total['USB_FTDI_D-'])
        check(f'the pad-to-pad paths agree within the {tol} mm tolerance',
              ps <= tol, f'{ps:.4f} mm')
        check(f'the all-copper skew is within the tolerance too', ts <= tol, f'{ts:.4f} mm')
        check('the record is matched, at the shipped all-copper skew, with no re-match',
              rec['status'] == 'matched' and rec.get('rematch') is None
              and abs(rec['delta_mm'] - ts) < 1e-3,
              f"{rec['status']} {rec['delta_mm']} rematch={rec.get('rematch')}")


def main():
    if hasattr(pm, 'excise_via_shorted_loops'):
        unit_rows()
    else:
        check('pcb_modification.excise_via_shorted_loops exists', False)
    route_row()
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
