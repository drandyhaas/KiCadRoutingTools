#!/usr/bin/env python3
"""#1729: a coupled pair on a layer wider than the track width keeps both
members at the layer's width past a neighbour.

A pair's centreline map stamps every obstacle with one extra clearance, the
member half-pitch. It was the call's, (track_width + gap) / 2, while a member
on a wider layer rides (layer width + gap) / 2 off the centreline: the map
let the outer member reach (layer width - track width) / 2 into a
neighbour's clearance, the graze neck then narrowed that member alone, and
the pair shipped one member narrow beside a full-width partner (an
intra-pair width and impedance mismatch). The map now stamps the widest
layer's half-pitch (`centreline_extra_clearance`).

  * the value: the call's half-pitch with no layer widths, else the widest
    layer's;
  * hole stamps: on the wide layer the centreline map reaches the member's
    far edge exactly as before (no double count), and a narrower layer
    reserves no less than it did;
  * routed: an in-repo board with a 2-layer stackup, --impedance 80 over a
    0.25 --track-width and a foreign track ending just above the pair's
    path. Every member segment ships at the layer width or, for the
    connectors, at the track width; --strict-sizes exits 0; DRC clean at
    the 0.25 clearance and connected (FAILS on 42f06d57: P ships 5 mm at
    0.29-0.36 mm beside N at 0.41, exit 3).

    python3 tests/test_1729_pair_members_keep_width.py
"""
import contextlib
import io
import json
import math
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import check, evidence, tool  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
TW, GAP, CLR = 0.25, 0.25, 0.25

STACKUP = ('\t\t(stackup\n'
           '\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t\t(layer "dielectric 1" (type "core") (thickness 0.2) (material "FR4") '
           '(epsilon_r 4.5) (loss_tangent 0.02))\n'
           '\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t)\n')
# another pair's member, a track on both layers ending just above DPA's path
WALL = ''.join('\t(segment (start 14 5.5) (end 14 15.45) (width 0.25) (layer "%s") '
               '(net "DPB_P") (uuid "00000000-0000-0000-0000-00000000000%d"))\n' % (lay, k)
               for k, lay in enumerate(('F.Cu', 'B.Cu')))
PRO = {"net_settings": {"classes": [{
    "name": "Default", "clearance": CLR, "track_width": TW, "diff_pair_gap": GAP,
    "diff_pair_width": TW, "via_diameter": 0.6, "via_drill": 0.3}]}}


def _cfg(layer_widths, reserve=True):
    from routing_config import GridRouteConfig
    return GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.1,
                           diff_pair_gap=0.1, layer_widths=layer_widths,
                           reserve_layer_widths=reserve)


def t_value(td):
    from diff_pair_routing import centreline_extra_clearance
    assert abs(centreline_extra_clearance(_cfg({})) - 0.1) < 1e-12
    v = centreline_extra_clearance(_cfg({'F.Cu': 0.3, 'B.Cu': 0.1}))
    assert abs(v - 0.2) < 1e-12, v
    v = centreline_extra_clearance(_cfg({'F.Cu': 0.3}))
    assert abs(v - 0.2) < 1e-12, v
    print("  ok  the call's half-pitch with no layer widths, else the widest layer's")


def t_hole_stamps(td):
    from diff_pair_routing import centreline_extra_clearance
    from obstacle_map import _hole_stamp_widths
    cfg = _cfg({'F.Cu': 0.3})
    call = (cfg.track_width + cfg.diff_pair_gap) / 2
    new = centreline_extra_clearance(cfg)

    def reach(extra):
        return {i: w / 2 + extra for w, idxs in _hole_stamp_widths(cfg, extra).items()
                for i in idxs}
    old, now = reach(call), reach(new)
    mitre = 1.0 / math.cos(math.radians(22.5))
    far = 0.3 / 2 + (0.3 + 0.1) / 2 * mitre
    assert abs(old[0] - far) < 1e-9 and abs(now[0] - far) < 1e-9, (old, now, far)
    assert now[1] >= old[1] - 1e-9, (old, now)
    se = _cfg({'F.Cu': 0.3}, reserve=False)
    assert _hole_stamp_widths(se, 0.05) == {0.1 + 2 * 0.05 * (mitre - 1): [0, 1]}, \
        _hole_stamp_widths(se, 0.05)
    print("  ok  hole stamps: the wide layer reaches the member's far edge as before, "
          "a narrower layer no less; a map with no baked width unchanged")


def _board(td):
    src = open(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'),
               encoding='utf-8').read()
    i = src.index('\n', src.index('(setup')) + 1
    txt = src[:i] + STACKUP + src[i:]
    k = txt.rstrip().rfind(')')
    txt = txt[:k] + WALL + txt[k:]
    dst = os.path.join(td, 'cc.kicad_pcb')
    with open(dst, 'w', encoding='utf-8', newline='') as fh:
        fh.write(txt)
    with open(os.path.join(td, 'cc.kicad_pro'), 'w', encoding='utf-8') as fh:
        json.dump(PRO, fh)
    return evidence(dst, 'the staged board')


def t_routed(td):
    from kicad_parser import parse_kicad_pcb
    from check_drc import run_drc
    from check_connected import run_connectivity_check
    from kicad_writer import EDGE_ROUNDING_MM
    src = _board(td)
    out = os.path.join(td, 'o.kicad_pcb')
    r = check(PY + [tool('route_diff.py'), src, out, '--nets', 'DPA_*',
                    '--impedance', '80', '--track-width', str(TW),
                    '--diff-pair-gap', str(GAP), '--clearance', str(CLR),
                    '--via-size', '0.6', '--via-drill', '0.3',
                    '--layers', 'F.Cu', 'B.Cu', '--strict-sizes'], accept=True)
    js = [l for l in r.stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]
    assert js, r.stdout[-1500:]
    s = json.loads(js[-1].split(': ', 1)[1])
    assert s['routed_diff_pairs'] == ['DPA'], s['routed_diff_pairs']
    pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
    names = {n.net_id: n.name for n in pcb.nets.values()}
    widths = {}
    for g in pcb.segments:
        n = names.get(g.net_id, '')
        if n.startswith('DPA'):
            widths.setdefault(n, set()).add(round(g.width, 4))
    lw = max(w for ws in widths.values() for w in ws)
    assert lw > 0.4, widths
    assert all(ws <= {TW, lw} for ws in widths.values()) and len(widths) == 2, widths
    with contextlib.redirect_stdout(io.StringIO()):
        drc = run_drc(out, clearance=CLR, quiet=True, print_summary=False,
                      check_sizes=False, clearance_margin=0.0)
        conn = run_connectivity_check(out, net_patterns=['DPA_*'], quiet=True)
    drc = [v for v in drc if v.get('overlap_mm', 1.0) > EDGE_ROUNDING_MM]
    assert list(drc) == [], str(drc)[:300]
    assert list(conn) == [], str(conn)[:200]
    print(f"  ok  routed: both members at {lw:g} with connectors at {TW:g}, "
          f"--strict-sizes exit 0, DRC clean, connected")


CASES = [t_value, t_hole_stamps, t_routed]


def main():
    want = set(sys.argv[1:])
    for fn in CASES:
        if want and fn.__name__ not in want:
            continue
        td = tempfile.mkdtemp(prefix='t1729_')
        try:
            print(fn.__name__)
            fn(td)
        finally:
            shutil.rmtree(td, ignore_errors=True)
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
