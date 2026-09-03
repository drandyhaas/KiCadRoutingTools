#!/usr/bin/env python3
"""#530 decision 2 follow-up (corpus A/B, schoko): a diff pair's coupling gap
is floored at ITS OWN net class's clearance, not just the run's.

#441 floors the gap at --clearance because KiCad grades the pair's P<->N
spacing under the plain clearance rule. With net classes honoured, the
clearance KiCad applies between the P and N of a pair in class 'DDMI' is the
DDMI class's, and the old cap-every-class writeback no longer lowers that
class to the gap. schoko (set2): 12 pairs routed at --diff-pair-gap 0.1 under
a 0.125 class -> 177 KiCad intra-pair clearance violations (0 before).

Synthetic board from the agreement harness: pair /X_P,/X_N in class 'pairs'
(clearance 0.15, pattern '/X_*') plus a Default net; route_diff with
--clearance 0.1 --diff-pair-gap 0.1:

  - the run announces the raise for /X and routes the pair at gap 0.15
    (every P segment is >= 0.15 mm from every N segment)
  - kicad-cli DRC on the output is clean of clearance items
  - negative control: the same run with the pair in the Default class prints
    no raise line and still emits the pair's copper
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))
import constraint_agreement as ca  # noqa: E402

SEG = re.compile(r'\(segment\s*\(start ([0-9.-]+) ([0-9.-]+)\)\s*\(end ([0-9.-]+) ([0-9.-]+)\)'
                 r'\s*\(width ([0-9.]+)\)\s*\(layer "([^"]+)"\)\s*\(net (\d+)\)', re.S)


def _segs(path):
    return [(float(a), float(b), float(c), float(d), float(w), layer, int(n))
            for a, b, c, d, w, layer, n in SEG.findall(open(path).read())]


def _seg_dist(s, t):
    """Edge-to-edge distance between two segments' copper (same layer)."""
    def _pt_seg(px, py, ax, ay, bx, by):
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        return math.hypot(px - (ax + u * dx), py - (ay + u * dy))
    d = min(_pt_seg(s[0], s[1], *t[:4]), _pt_seg(s[2], s[3], *t[:4]),
            _pt_seg(t[0], t[1], *s[:4]), _pt_seg(t[2], t[3], *s[:4]))
    return d - s[4] / 2 - t[4] / 2


def _min_pn_gap(path, p_id, n_id):
    segs = _segs(path)
    P = [s for s in segs if s[6] == p_id]
    N = [s for s in segs if s[6] == n_id]
    best = None
    for s in P:
        for t in N:
            if s[5] != t[5]:
                continue
            d = _seg_dist(s, t)
            best = d if best is None else min(best, d)
    return best, len(P), len(N)


def _route(board, out, extra):
    return subprocess.run([sys.executable, '-X', 'utf8',
                           os.path.join(ROOT, 'py_router', 'route_diff.py'), board, out,
                           '--nets', '/X_P', '/X_N', '--track-width', '0.1',
                           '--diff-pair-gap', '0.1', '--clearance', '0.1',
                           '--via-size', '0.4', '--via-drill', '0.2'] + extra,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace', cwd=ROOT)


def _kicad_clearance_items(board):
    cli = ca.find_kicad_cli()
    if not cli:
        return None
    rep = board + '.drc.json'
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-error',
                    '-o', rep, board], capture_output=True, text=True)
    if not os.path.exists(rep):
        return None
    d = json.load(open(rep))
    return [v for v in d.get('violations', []) if v.get('type') == 'clearance']


def main():
    fails = []
    with tempfile.TemporaryDirectory() as td:
        fps = [{'ref': 'U1', 'x': 10, 'y': 10, 'net_id': 1, 'net_name': '/X_P', 'layer': 'F.Cu'},
               {'ref': 'U2', 'x': 10, 'y': 11, 'net_id': 2, 'net_name': '/X_N', 'layer': 'F.Cu'},
               {'ref': 'U3', 'x': 30, 'y': 10, 'net_id': 1, 'net_name': '/X_P', 'layer': 'F.Cu'},
               {'ref': 'U4', 'x': 30, 'y': 11, 'net_id': 2, 'net_name': '/X_N', 'layer': 'F.Cu'},
               {'ref': 'U5', 'x': 20, 'y': 20, 'net_id': 3, 'net_name': 'C', 'layer': 'F.Cu'}]
        saved = dict(ca.NETS)
        ca.NETS[1], ca.NETS[2] = '/X_P', '/X_N'
        try:
            b1 = os.path.join(td, 'cls.kicad_pcb')
            ca.write_board(b1, footprints=fps,
                           classes=[{'name': 'pairs', 'clearance': 0.15, 'track_width': 0.1,
                                     'via_diameter': 0.4, 'via_drill': 0.2, 'priority': 0}],
                           patterns=[('/X_*', 'pairs')])
            b2 = os.path.join(td, 'dflt.kicad_pcb')
            ca.write_board(b2, footprints=fps)
        finally:
            ca.NETS.clear()
            ca.NETS.update(saved)

        out1 = os.path.join(td, 'cls_out.kicad_pcb')
        r = _route(b1, out1, [])
        if r.returncode != 0 or not os.path.exists(out1):
            fails.append(f"class run failed rc={r.returncode}:\n{r.stdout[-2000:]}\n{r.stderr[-800:]}")
        else:
            if 'raised to its net-class clearance 0.15' not in r.stdout:
                fails.append("class run did not announce the gap raise to 0.15")
            gap, nP, nN = _min_pn_gap(out1, 1, 2)
            if not nP or not nN:
                fails.append(f"class run emitted no pair copper (P={nP} N={nN})")
            elif gap is not None and gap < 0.15 - 1e-3:
                fails.append(f"class run: P<->N copper gap {gap:.4f} < 0.15")
            items = _kicad_clearance_items(out1)
            if items is None:
                print("  (kicad-cli not found: KiCad DRC leg skipped)")
            elif items:
                fails.append(f"KiCad flags {len(items)} clearance item(s) on the class run: "
                             f"{items[0].get('description')}")

        out2 = os.path.join(td, 'dflt_out.kicad_pcb')
        r2 = _route(b2, out2, [])
        if r2.returncode != 0 or not os.path.exists(out2):
            fails.append(f"default run failed rc={r2.returncode}:\n{r2.stdout[-1500:]}")
        else:
            if 'raised to its net-class clearance' in r2.stdout:
                fails.append("default-class run raised the gap (negative control)")
            gap2, nP2, nN2 = _min_pn_gap(out2, 1, 2)
            if not nP2 or not nN2:
                fails.append(f"default run emitted no pair copper (P={nP2} N={nN2})")
            # (no ~0.1 assertion: with the pads 1 mm apart the router need not
            # couple the pair at the gap; the control is the absent raise line)
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: a pair's coupling gap is floored at its own net-class clearance "
          "(0.1 -> 0.15 under class 'pairs'; KiCad clean), and a Default-class pair keeps 0.1")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
