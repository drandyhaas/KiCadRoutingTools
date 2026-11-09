#!/usr/bin/env python3
"""#1747: a pair narrowed to fit narrows its gap toward the impedance target.

#1742 couples a pair that fits at no layer width by narrowing both members to
one width over the stretch that does not fit. Only the width narrowed, so that
stretch kept the declared gap and landed well above the differential impedance
the layer widths were solved for. Now each narrowed layer takes the gap that
holds the target at the narrowed width, no narrower than the clearance between
P and N allows, and the run says per layer what gap it used and what
impedance that reaches.

  * solver: the gap holding a target at a fixed width reproduces the target;
    where even the floor leaves the pair above it the floor is returned with
    its impedance; the gap never drops under the floor;
  * config: only a narrowed, routable layer takes a gap, and the pair's
    half-pitch on it is (narrowed width + that gap) / 2 while another layer
    keeps the pair's gap; no gap without a target;
  * routed: a 2-layer board, --impedance 80 over a 0.25 track width and a
    0.25 gap at a 0.15 clearance, with two foreign tracks leaving a gate the
    pair passes only narrowed. The pair couples narrowed, pair_reports names
    the gap of each narrowed layer (under 0.25, not under 0.15) and the
    impedance there, which is nearer 80 than at the 0.25 gap; the shipped
    narrowed copper sits at that gap; DRC clean at 0.15 and connected
    (FAILS on f7fa23c5: no narrowed_gap_mm, the narrowed copper at 0.25).

    python3 tests/test_1747_narrowed_pair_gap.py
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
TW, GAP, CLR, OHMS = 0.25, 0.25, 0.15, 80.0

STACKUP = ('\t\t(stackup\n'
           '\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t\t(layer "dielectric 1" (type "core") (thickness 0.2) (material "FR4") '
           '(epsilon_r 4.5) (loss_tangent 0.02))\n'
           '\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t)\n')
# another pair's member on both layers, from above and from below, leaving
# a gate the pair passes only narrowed
GATE = ((5.5, 15.45), (16.75, 24.5))
WALL = ''.join('\t(segment (start 14 %s) (end 14 %s) (width 0.25) (layer "%s") '
               '(net "DPB_P") (uuid "00000000-0000-0000-0000-0000000000%d%d"))\n'
               % (a, b, lay, j, k)
               for j, (a, b) in enumerate(GATE)
               for k, lay in enumerate(('F.Cu', 'B.Cu')))
PRO = {"net_settings": {"classes": [{
    "name": "Default", "clearance": CLR, "track_width": TW, "diff_pair_gap": GAP,
    "diff_pair_width": TW, "via_diameter": 0.6, "via_drill": 0.3}]}}


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


def _zdiff(pcb, layer, w, gap):
    from impedance import calculate_impedance_for_layer
    return calculate_impedance_for_layer(pcb, layer, w, gap)['zdiff']


def t_solver(td):
    from kicad_parser import parse_kicad_pcb
    from impedance import differential_gap_for_impedance
    pcb = parse_kicad_pcb(_board(td))
    w = 0.3
    target = _zdiff(pcb, 'F.Cu', w, 0.17)
    g, z = differential_gap_for_impedance(pcb, 'F.Cu', w, target, 0.1, 0.25)
    assert 0.17 - 1e-6 <= g <= 0.1701 + 1e-6 and abs(z - target) < 0.05, (g, z, target)
    # a target the floor cannot reach: the floor, with the impedance there
    g, z = differential_gap_for_impedance(pcb, 'F.Cu', 0.1, 50.0, 0.12, 0.25)
    assert g == 0.12 and z > 50.0 and abs(z - _zdiff(pcb, 'F.Cu', 0.1, 0.12)) < 1e-9, (g, z)
    # a target above the pair's own gap: the pair's gap
    g, z = differential_gap_for_impedance(pcb, 'F.Cu', 0.6, 150.0, 0.1, 0.25)
    assert g == 0.25 and z < 150.0, (g, z)
    print("  ok  the solved gap holds the target; the floor and the pair's gap "
          "bound it, each with its own impedance")


def t_config(td):
    from kicad_parser import parse_kicad_pcb
    from routing_config import GridRouteConfig
    from diff_pair_loop import narrowed_pair_gaps, narrowed_pair_config
    from diff_pair_routing import layer_pair_spacing
    pcb = parse_kicad_pcb(_board(td))
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=TW, grid_step=0.05,
                          diff_pair_gap=GAP, layer_widths={'F.Cu': 0.41, 'B.Cu': 0.3})
    gz = narrowed_pair_gaps(cfg, pcb, 0.35, (OHMS, 0.0), CLR)
    assert set(gz) == {'F.Cu'}, gz          # B.Cu is no wider than 0.35
    g, z = gz['F.Cu']
    assert CLR - 1e-9 <= g < GAP and z < _zdiff(pcb, 'F.Cu', 0.35, GAP), gz
    nc = narrowed_pair_config(cfg, 0.35, {'F.Cu': g})
    sp = layer_pair_spacing(nc)
    assert abs(sp('F.Cu') - (0.35 + g) / 2) < 1e-9, sp('F.Cu')
    assert abs(sp('B.Cu') - (0.3 + GAP) / 2) < 1e-9, sp('B.Cu')
    assert narrowed_pair_gaps(cfg, pcb, 0.35, None, CLR) == {}
    # a layer the run may not route on is not narrowed
    rc = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=TW, grid_step=0.05,
                         diff_pair_gap=GAP, layer_widths={'F.Cu': 0.41, 'B.Cu': 0.41},
                         layer_costs=[1.0, -1.0])
    assert set(narrowed_pair_gaps(rc, pcb, 0.35, (OHMS, 0.0), CLR)) == {'F.Cu'}
    print(f"  ok  a narrowed layer takes gap {g:.4f} ({z:.1f} ohm), half-pitch "
          f"(w + gap) / 2 there; other layers keep {GAP}; none without a target")


def _edge_gap(a, segs):
    """Edge-to-edge distance from the middle of `a` to the nearest of `segs`."""
    mx, my = (a.start_x + a.end_x) / 2, (a.start_y + a.end_y) / 2
    best = None
    for b in segs:
        dx, dy = b.end_x - b.start_x, b.end_y - b.start_y
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((mx - b.start_x) * dx
                                                   + (my - b.start_y) * dy) / l2))
        d = math.hypot(mx - b.start_x - t * dx, my - b.start_y - t * dy) \
            - a.width / 2 - b.width / 2
        best = d if best is None or d < best else best
    return best


def t_routed(td):
    from kicad_parser import parse_kicad_pcb
    from check_drc import run_drc
    from check_connected import run_connectivity_check
    from kicad_writer import EDGE_ROUNDING_MM
    src = _board(td)
    out = os.path.join(td, 'o.kicad_pcb')
    argv = PY + [tool('route_diff.py'), src, out, '--nets', 'DPA_*',
                 '--impedance', str(OHMS), '--track-width', str(TW),
                 '--diff-pair-gap', str(GAP), '--clearance', str(CLR),
                 '--via-size', '0.6', '--via-drill', '0.3', '--layers', 'F.Cu', 'B.Cu']
    r = check(argv, accept=True)
    js = [l for l in r.stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]
    assert js, r.stdout[-1500:]
    s = json.loads(js[-1].split(': ', 1)[1])
    assert s['routed_diff_pairs'] == ['DPA'], (s['routed_diff_pairs'],
                                              s.get('single_ended_diff_pairs'))
    rep = [p for p in s['pair_reports'] if p['pair'] == 'DPA'][0]
    nar = rep.get('narrowed_to_mm')
    assert nar is not None, rep
    gaps, zs = rep.get('narrowed_gap_mm'), rep.get('narrowed_zdiff_ohm')
    assert gaps and set(gaps) == set(zs), rep
    pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
    for layer, g in gaps.items():
        assert CLR - 1e-9 <= g < GAP - 1e-6, (layer, g)
        assert abs(zs[layer] - OHMS) < abs(_zdiff(pcb, layer, nar, GAP) - OHMS), (layer, zs)
    names = {n.net_id: n.name for n in pcb.nets.values()}
    segs = {'DPA_P': [], 'DPA_N': []}
    for g in pcb.segments:
        if names.get(g.net_id) in segs:
            segs[names[g.net_id]].append(g)
    # the narrowed copper sits at the narrowed gap
    seen = []
    for a in segs['DPA_P']:
        if abs(a.width - nar) < 1e-4 and a.layer in gaps and \
                math.hypot(a.end_x - a.start_x, a.end_y - a.start_y) > 0.3:
            eg = _edge_gap(a, [b for b in segs['DPA_N'] if b.layer == a.layer])
            if eg is not None and eg < GAP:
                seen.append((a.layer, round(eg, 4)))
    assert any(abs(eg - gaps[l]) < 0.01 for l, eg in seen), (seen, gaps)
    with contextlib.redirect_stdout(io.StringIO()):
        drc = run_drc(out, clearance=CLR, quiet=True, print_summary=False,
                      check_sizes=False, clearance_margin=0.0)
        conn = run_connectivity_check(out, net_patterns=['DPA_*'], quiet=True)
    drc = [v for v in drc if v.get('overlap_mm', 1.0) > EDGE_ROUNDING_MM]
    assert list(drc) == [], str(drc)[:300]
    assert list(conn) == [], str(conn)[:200]
    print(f"  ok  narrowed to {nar:g} at gap(s) {gaps} ({zs} ohm against {OHMS:g}); "
          f"the narrowed copper sits there; DRC clean at {CLR}, connected")


CASES = [t_solver, t_config, t_routed]


def main():
    want = set(sys.argv[1:])
    for fn in CASES:
        if want and fn.__name__ not in want:
            continue
        td = tempfile.mkdtemp(prefix='t1747_')
        try:
            print(fn.__name__)
            fn(td)
        finally:
            shutil.rmtree(td, ignore_errors=True)
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
