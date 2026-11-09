#!/usr/bin/env python3
"""#1742: a pair that fits at no layer width couples narrowed, both members
together, instead of shipping single-ended.

Since #1729 a pair's centreline map reserves each layer's own member
half-pitch, so on a run whose layers are all wide impedance layers a pair
can find no coupled route at a tight spot and was left to the single-ended
fallback. Now, before that fallback, the pair is tried narrowed -- both
members at one narrower width, on a centreline map built for that width,
widest first down to the fab floor -- and every stretch where the full pair
fits goes back to the layer width, both members at once.

  * rungs: below the widest layer width one grid step apart, down to the
    fab floor; none on a run without per-layer widths;
  * routed: an in-repo board with a 2-layer stackup, --impedance 80 over a
    0.25 --track-width, and two foreign tracks leaving a gate the pair must
    pass that is too narrow for it at its layer width. The pair ships
    coupled, back at the layer width away from the gate; walking each member
    from its pad, P and N change width in the same order (same runs); the
    narrowed copper is named in design_rules under its own site and
    --strict-sizes exits 3 on it; DRC clean at the 0.25 clearance and
    connected (FAILS on e46ae20f: the pair ships single-ended).

    python3 tests/test_1742_pair_narrowed_to_fit.py
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
SITE = 'coupled pair narrowed to fit, both members'

STACKUP = ('\t\t(stackup\n'
           '\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t\t(layer "dielectric 1" (type "core") (thickness 0.2) (material "FR4") '
           '(epsilon_r 4.5) (loss_tangent 0.02))\n'
           '\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))\n'
           '\t\t)\n')
# another pair's member on both layers, from above and from below, leaving
# a gate the pair passes only narrowed
GATE_LOW = 16.95
WALL = ''.join('\t(segment (start 14 %s) (end 14 %s) (width 0.25) (layer "%s") '
               '(net "DPB_P") (uuid "00000000-0000-0000-0000-0000000000%d%d"))\n'
               % (a, b, lay, j, k)
               for j, (a, b) in enumerate(((5.5, 15.45), (GATE_LOW, 24.5)))
               for k, lay in enumerate(('F.Cu', 'B.Cu')))
PRO = {"net_settings": {"classes": [{
    "name": "Default", "clearance": CLR, "track_width": TW, "diff_pair_gap": GAP,
    "diff_pair_width": TW, "via_diameter": 0.6, "via_drill": 0.3}]}}


def t_rungs(td):
    from routing_config import GridRouteConfig, DiffPairNet
    from diff_pair_loop import narrow_pair_rungs, narrowed_pair_config
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'))
    pair = DiffPairNet(base_name='DPA', p_net_id=1, n_net_id=2,
                       p_net_name='DPA_P', n_net_name='DPA_N')
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.25, grid_step=0.05,
                          diff_pair_gap=0.25, layer_widths={'F.Cu': 0.41, 'B.Cu': 0.41})
    rungs = narrow_pair_rungs(cfg, pcb, pair)
    assert rungs == sorted(rungs, reverse=True) and rungs[0] < 0.41, rungs
    assert all(abs(a - b - 0.05) < 1e-6 for a, b in zip(rungs[:-2], rungs[1:-1])), rungs
    assert 0.08 < rungs[-1] < 0.25 and all(abs(w - 0.25) > 1e-4 for w in rungs), rungs
    nc = narrowed_pair_config(cfg, 0.2)
    assert nc.layer_widths == {'F.Cu': 0.2, 'B.Cu': 0.2} and nc.track_width == 0.2, nc
    assert narrow_pair_rungs(GridRouteConfig(layers=['F.Cu', 'B.Cu'], track_width=0.25),
                             pcb, pair) == []
    print(f"  ok  rungs {rungs[0]:g} .. {rungs[-1]:g} one grid step apart; none "
          f"without per-layer widths")


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


def _width_runs(segs):
    """The member's widths in order along its copper from its first pad
    (the end nearest the board's left edge), equal neighbours merged."""
    def key(x, y):
        return (round(x, 4), round(y, 4))
    ends = {}
    for g in segs:
        for e in (key(g.start_x, g.start_y), key(g.end_x, g.end_y)):
            ends.setdefault(e, []).append(g)
    tips = [e for e, gs in ends.items() if len(gs) == 1]
    at = min(tips)
    left, runs = set(range(len(segs))), []
    while True:
        nxt = [k for k in left if at in (key(segs[k].start_x, segs[k].start_y),
                                          key(segs[k].end_x, segs[k].end_y))]
        if not nxt:
            break
        g = segs[nxt[0]]
        left.discard(nxt[0])
        w = round(g.width, 4)
        if not runs or runs[-1] != w:
            runs.append(w)
        a, b = key(g.start_x, g.start_y), key(g.end_x, g.end_y)
        at = b if a == at else a
    assert not left, f'{len(left)} segment(s) off the chain'
    return runs


def t_routed(td):
    from kicad_parser import parse_kicad_pcb
    from check_drc import run_drc
    from check_connected import run_connectivity_check
    from kicad_writer import EDGE_ROUNDING_MM
    src = _board(td)
    out = os.path.join(td, 'o.kicad_pcb')
    argv = PY + [tool('route_diff.py'), src, out, '--nets', 'DPA_*',
                 '--impedance', '80', '--track-width', str(TW),
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
    assert nar is not None and nar < 0.4, rep
    rows = [w for w in s['design_rules']['narrowed'] if w['site'] == SITE]
    assert sorted(w['net_name'] for w in rows) == ['DPA_N', 'DPA_P'], s['design_rules']
    check(argv[:5] + [os.path.join(td, 'o2.kicad_pcb')] + argv[6:] + ['--strict-sizes'],
          refuse='--strict-sizes: Design rules', code=3)
    pcb = parse_kicad_pcb(evidence(out, 'the routed board'))
    names = {n.net_id: n.name for n in pcb.nets.values()}
    segs = {'DPA_P': [], 'DPA_N': []}
    for g in pcb.segments:
        if names.get(g.net_id) in segs:
            segs[names[g.net_id]].append(g)
    runs = {n: _width_runs(v) for n, v in segs.items()}
    assert runs['DPA_P'] == runs['DPA_N'], runs
    assert min(runs['DPA_P']) <= nar + 1e-4 and len(runs['DPA_P']) >= 3, runs
    lw = max(g.width for v in segs.values() for g in v)
    assert lw > 0.4, lw
    with contextlib.redirect_stdout(io.StringIO()):
        drc = run_drc(out, clearance=CLR, quiet=True, print_summary=False,
                      check_sizes=False, clearance_margin=0.0)
        conn = run_connectivity_check(out, net_patterns=['DPA_*'], quiet=True)
    drc = [v for v in drc if v.get('overlap_mm', 1.0) > EDGE_ROUNDING_MM]
    assert list(drc) == [], str(drc)[:300]
    assert list(conn) == [], str(conn)[:200]
    print(f"  ok  routed coupled, narrowed to {nar:g} where the gate is and at {lw:g} "
          f"elsewhere, P and N matched, rows under '{SITE}', --strict-sizes exit 3, "
          f"DRC clean, connected")


CASES = [t_rungs, t_routed]


def main():
    want = set(sys.argv[1:])
    for fn in CASES:
        if want and fn.__name__ not in want:
            continue
        td = tempfile.mkdtemp(prefix='t1742_')
        try:
            print(fn.__name__)
            fn(td)
        finally:
            shutil.rmtree(td, ignore_errors=True)
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
