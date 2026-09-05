#!/usr/bin/env python3
"""#530 decision 4: each net's vias are drawn at ITS OWN class size.

Two nets on the harness's synthetic board, each with one pad on F.Cu and
one on B.Cu (so every route needs a via): net A in the Default class
(via 0.6 / 0.3) and net P1 in class 'power' (via 0.8 / 0.4, pattern 'P*').

  --via-size omitted   -> A's vias are 0.6/0.3, P1's are 0.8/0.4, the summary
                          names the per-net sizes, check_drc is clean
  --via-size 0.5 given -> every via is 0.5/0.3 (an explicit request applies
                          to every net, as before)

Needs a grid_router with the via-rung API (0.22.0+); SKIPS otherwise.
"""
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'rust_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))
from constraint_agreement import write_board  # noqa: E402

VIA_RE = re.compile(r'\(via\b.*?\(size ([0-9.]+)\).*?\(drill ([0-9.]+)\).*?\(net (\d+)\)', re.S)


def _route(board, out, extra):
    js = out + '.json'
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                        '--nets', 'A', 'P1', '--json-out', js] + extra,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    vias = [(float(s), float(d), int(n)) for s, d, n in VIA_RE.findall(open(out).read())] \
        if os.path.exists(out) else []
    return r, vias


def main():
    try:
        import grid_router
        if not hasattr(grid_router.GridObstacleMap, 'add_blocked_vias_rung_batch'):
            print("SKIP: grid_router without the via-rung API (needs 0.22.0+)")
            return 0
    except ImportError:
        print("SKIP: grid_router not importable")
        return 0
    fails = []
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'v.kicad_pcb')
        fps = [{'ref': 'U1', 'x': 10, 'y': 10, 'net_id': 1, 'net_name': 'A', 'layer': 'F.Cu'},
               {'ref': 'U2', 'x': 20, 'y': 10, 'net_id': 1, 'net_name': 'A', 'layer': 'B.Cu'},
               {'ref': 'U3', 'x': 10, 'y': 20, 'net_id': 3, 'net_name': 'P1', 'layer': 'F.Cu'},
               {'ref': 'U4', 'x': 20, 'y': 20, 'net_id': 3, 'net_name': 'P1', 'layer': 'B.Cu'}]
        # NETS in the harness are {1: A, 2: B, 3: C}; give net 3 the name P1
        import constraint_agreement as ca
        ca.NETS[3] = 'P1'
        try:
            write_board(board, footprints=fps,
                        classes=[{'name': 'power', 'clearance': 0.2, 'track_width': 0.4,
                                  'via_diameter': 0.8, 'via_drill': 0.4, 'priority': 0}],
                        patterns=[('P*', 'power')])
        finally:
            ca.NETS[3] = 'C'
        r, vias = _route(board, os.path.join(td, 'out.kicad_pcb'), [])
        if r.returncode != 0 or not vias:
            print(r.stdout[-3000:])
            fails.append("the class-via run produced no vias")
        by_net = {}
        for s, d, n in vias:
            by_net.setdefault(n, set()).add((s, d))
        # the harness's Default class draws 0.6 / 0.3
        if by_net.get(1) != {(0.6, 0.3)}:
            fails.append(f"net A (Default class) vias: {by_net.get(1)} != {{(0.6, 0.3)}}")
        if by_net.get(3) != {(0.8, 0.4)}:
            fails.append(f"net P1 (power class) vias: {by_net.get(3)} != {{(0.8, 0.4)}}")
        if 'Per-net via sizes for 1 net(s)' not in r.stdout:
            fails.append("the run did not announce the per-net via sizes")
        g = subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'check_drc.py'),
                            os.path.join(td, 'out.kicad_pcb'), '--quiet'],
                           capture_output=True, text=True, cwd=ROOT)
        if 'EXIT=0' not in g.stdout:
            fails.append("check_drc is not clean on the per-net-via board:\n" + g.stdout[-1500:])
        # an explicit --via-size applies to every net
        r2, vias2 = _route(board, os.path.join(td, 'out2.kicad_pcb'),
                           ['--via-size', '0.5', '--via-drill', '0.3'])
        if r2.returncode != 0 or not vias2:
            fails.append("the explicit-via run produced no vias")
        if {(s, d) for s, d, _ in vias2} != {(0.5, 0.3)}:
            fails.append(f"explicit --via-size did not apply to every net: {vias2}")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: nets are routed and emitted at their own class via size through per-net "
          "via-legality rungs; an explicit --via-size still applies to every net")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
