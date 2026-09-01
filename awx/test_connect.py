#!/usr/bin/env python3
"""connect() on the case that broke every straight-line rule: a trunk
exit above the SW corner of the berth, a stub end on the south face,
and a decoupling cap (C5) sitting exactly where a straight descent
passes -- its top pad on the linear morph's line, its bottom pad on
the 45 degree line. The legal way is between the pads or below them,
which only a search finds.

The test knows nothing about the board beyond two nets it picks by
geometry: it adds a lone F.Cu segment as the "trunk exit" island, asks
connect() for the join, writes the result, and grades it with the real
checkers -- connected, and DRC-clean at the routed clearance. With a
BAND it also asserts the copper stayed inside it. The negative control
is the same call with the direct line blocked by a foreign segment
laid across it: connect() must go around, not through.

usage: test_connect.py FANOUT_BOARD.kicad_pcb NET  (e.g. ch6_fo_k15 SDQ0)
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'py_router'))
sys.path.insert(0, HERE)
from kicad_parser import parse_kicad_pcb, Segment  # noqa: E402
from kicad_writer import add_tracks_and_vias_to_pcb  # noqa: E402
from connectivity import find_connected_groups  # noqa: E402
import connect as cn  # noqa: E402
import braid as te  # noqa: E402

PY = sys.executable


def grade(board, nm):
    r = subprocess.run([PY, os.path.join(HERE, '..', 'py_router',
                                         'check_connected.py'), board],
                       capture_output=True, text=True)
    open_ = [ln for ln in (r.stdout + r.stderr).splitlines()
             if re.search(r'\(net \d+\):', ln)
             and ln.split('(')[0].strip().split('/')[-1] == nm]
    r = subprocess.run([PY, os.path.join(HERE, '..', 'py_router',
                                         'check_drc.py'), board,
                        '--clearance', '0.1', '--clearance-margin', '0.1'],
                       capture_output=True, text=True)
    m = re.search(r'FOUND (\d+) DRC VIOLATIONS', r.stdout + r.stderr)
    pairs = [ln.strip() for ln in (r.stdout + r.stderr).splitlines()
             if '<->' in ln]
    return len(open_), (int(m.group(1)) if m else 0), pairs


def run_case(board, nm, label, block=False, band=None):
    pcb = parse_kicad_pcb(board)
    byname = {n.name.split('/')[-1]: (i, n) for i, n in pcb.nets.items()}
    nid, net = byname[nm]
    ends = te.endpoints(pcb, [nm], byname, dest_ref=None)
    # the stub end is the free end nearest the destination pad; the
    # "trunk exit" is a synthetic lone segment 9 mm west, 3 mm up
    stub = ends[nm][0]
    src = next(p for p in net.pads
               if abs(p.global_x - stub[0]) + abs(p.global_y - stub[1])
               > 3.0)
    exit_pt = (stub[0] - 9.0, stub[1] - 3.0)
    trunk = Segment(exit_pt[0] - 0.5, exit_pt[1], exit_pt[0], exit_pt[1],
                    te.TRACK, 'F.Cu', nid)
    pcb.segments.append(trunk)
    if block:
        # negative control: a foreign wall across the straight line, on
        # the connection's own layer -- through it is not an answer
        foreign = next(i for i, n in pcb.nets.items()
                       if i != nid and i != 0 and n.pads)
        mx, my = (exit_pt[0] + stub[0]) / 2, (exit_pt[1] + stub[1]) / 2
        pcb.segments.append(Segment(mx, my - 1.2, mx, my + 1.2, 0.2,
                                    'F.Cu', foreign))
    cfg = cn.make_config(pcb, te.TRACK, te.CLEAR, te.VIA_SIZE, te.VIA_DRILL)
    res = cn.connect(pcb, nid, exit_pt, 'F.Cu', stub, 'F.Cu', cfg,
                     band=band, verbose=True)
    if res is None:
        print(f'{label}: connect() found NO route')
        return False
    segs, vias = res
    print(f'{label}: {len(segs)} segments, {len(vias)} vias, '
          f'{cn.seg_len(segs):.2f} mm')
    print('  corners: ' + ' '.join(f'({s.end_x:.2f},{s.end_y:.2f})'
                                   for s in segs))
    groups = find_connected_groups(
        [s for s in pcb.segments if s.net_id == nid] + segs,
        vias=[v for v in pcb.vias if v.net_id == nid] + vias)
    joined = any(trunk in g and any(
        abs(s.start_x - stub[0]) + abs(s.start_y - stub[1]) < 0.01
        or abs(s.end_x - stub[0]) + abs(s.end_y - stub[1]) < 0.01
        for s in g) for g in groups)
    ok = joined
    if band is not None:
        lo, hi = band
        for s in segs:
            for (x, y) in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
                if lo is not None and y < lo(x) - 0.01:
                    print(f'  BAND BREACH above at ({x:.2f},{y:.2f})')
                    ok = False
                if hi is not None and y > hi(x) + 0.01:
                    print(f'  BAND BREACH below at ({x:.2f},{y:.2f})')
                    ok = False
    out = os.path.join(HERE, f'test_connect_{label}.kicad_pcb')
    net_names = {i: n.name for i, n in pcb.nets.items()}
    tracks = [{'start': (s.start_x, s.start_y), 'end': (s.end_x, s.end_y),
               'width': s.width, 'layer': s.layer, 'net_id': s.net_id}
              for s in [trunk] + segs
              + ([pcb.segments[-1]] if block else [])]
    vs = [{'x': v.x, 'y': v.y, 'size': v.size, 'drill': v.drill,
           'layers': v.layers, 'net_id': v.net_id} for v in vias]
    add_tracks_and_vias_to_pcb(board, out, tracks, vs, [],
                               net_id_to_name=net_names)
    n_open, n_drc, pairs = grade(out, nm)
    own = [p for p in pairs if nm in p]
    print(f'  joined={joined} open={n_open} drc={n_drc} '
          f'(involving {nm}: {len(own)})')
    for p in own[:5]:
        print('   ', p)
    return ok and not own


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit('usage: test_connect.py FANOUT_BOARD.kicad_pcb [NET]')
    board = sys.argv[1]
    nm = sys.argv[2] if len(sys.argv) > 2 else 'SDQ0'
    results = []
    results.append(('open', run_case(board, nm, 'open')))
    # the band steps down 1 mm east of the exit: the free route descends
    # gently, this one must drop early -- a band that changes the answer
    results.append(('band', run_case(
        board, nm, 'band',
        band=(lambda x: 65.1 if x < 129.5 else 66.8, None))))
    results.append(('blocked', run_case(board, nm, 'blocked', block=True)))
    print()
    for k, v in results:
        print(f'{k:8} {"PASS" if v else "FAIL"}')
    sys.exit(0 if all(v for _, v in results) else 1)
