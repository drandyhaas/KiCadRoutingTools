#!/usr/bin/env python3
"""route_diff honours a layer-scoped .kicad_dru track_width opt (route.py's
#530 twin).

route.py fills per-layer widths from a rule such as
`(rule x (layer "F.Cu") (constraint track_width (opt 0.15mm)))`; route_diff
ignored it, so a layer with a narrower pair than the class routed at the class
width. With no --impedance and no --track-width, the pair members' resolved
per-layer opts become the call's layer widths when every member agrees on
every routed layer.

Fixture: cap_chain's two pairs (DPA_*, DPB_*, coupled on F.Cu), Default class
0.25 mm track / 0.25 mm gap / 0.25 mm clearance, rule F.Cu opt 0.15 mm. Rows:
  - with the rule: the widths line is printed, the coupled F.Cu run is
    0.15 mm, and the P/N centre pitch is 0.15 + 0.25 = 0.40 mm (the pitch
    follows the layer width, not the 0.25 class width's 0.50);
  - without the rule: no widths line, every leg at 0.25 mm;
  - both outputs: check_drc clean at the routed 0.25 mm clearance, and every
    pair net connected;
  - a class whose single-ended track_width (0.3) is not its diff_pair_width
    (0.25), F.Cu priced up so the pairs run on B.Cu: the layer no rule scopes
    keeps the 0.25 pair width (KiCad's inDiffPair implicit rule), not 0.3;
  - DPB in a class of its own (diff_pair_width 0.2): the pairs disagree on
    B.Cu, the engine carries one width per layer, so the rule is NOT applied
    and the run says which layer disagrees.

    python3 tests/test_diff_dru_layer_widths.py
"""
import contextlib
import io
import json
import math
import os
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _d))

from run_utils import evidence  # noqa: E402

RULE = '(version 1)\n(rule "f_narrow" (layer "F.Cu") (constraint track_width (opt 0.15mm)))\n'
PRO = {"net_settings": {"classes": [{
    "name": "Default", "clearance": 0.25, "track_width": 0.25, "diff_pair_gap": 0.25,
    "diff_pair_width": 0.25, "via_diameter": 0.6, "via_drill": 0.3}]}}
# Default's single-ended track_width differs from its diff_pair_width.
PRO_TW = {"net_settings": {"classes": [dict(PRO["net_settings"]["classes"][0], track_width=0.3)]}}
# DPB in a class of its own with a narrower pair width.
PRO_SPLIT = {"net_settings": {
    "classes": PRO["net_settings"]["classes"] + [{
        "name": "Narrow", "clearance": 0.25, "track_width": 0.2, "diff_pair_gap": 0.25,
        "diff_pair_width": 0.2, "via_diameter": 0.6, "via_drill": 0.3, "priority": 0}],
    "netclass_patterns": [{"netclass": "Narrow", "pattern": "DPB_*"}]}}
FAILS = []


def check(name, cond, detail=''):
    print(('  PASS  ' if cond else '  FAIL  ') + name + (f'  [{detail}]' if detail else ''))
    if not cond:
        FAILS.append(name)


def _seg_dist(a, b):
    """Shortest distance between two segments' centrelines."""
    def pt(p, s):
        (ax, ay), (bx, by) = (s.start_x, s.start_y), (s.end_x, s.end_y)
        L = (bx - ax) ** 2 + (by - ay) ** 2
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / L))
        return math.hypot(p[0] - ax - t * (bx - ax), p[1] - ay - t * (by - ay))
    return min(pt((a.start_x, a.start_y), b), pt((a.end_x, a.end_y), b),
               pt((b.start_x, b.start_y), a), pt((b.end_x, b.end_y), a))


def _route(td, dru, label=None, pro=None, layer_costs=None):
    import route_diff
    from kicad_parser import parse_kicad_pcb
    from check_drc import run_drc
    from check_connected import run_connectivity_check
    sub = os.path.join(td, label or ('rule' if dru else 'plain'))
    os.makedirs(sub)
    dst = os.path.join(sub, 'cc.kicad_pcb')
    shutil.copy(os.path.join(ROOT, 'kicad_files', 'cap_chain.kicad_pcb'), dst)
    for stem in ('cc', 'o'):
        if dru:
            with open(os.path.join(sub, stem + '.kicad_dru'), 'w', encoding='utf-8') as fh:
                fh.write(RULE)
        with open(os.path.join(sub, stem + '.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump(pro or PRO, fh)
    out = os.path.join(sub, 'o.kicad_pcb')
    log = io.StringIO()
    with contextlib.redirect_stdout(log):
        route_diff.batch_route_diff_pairs(evidence(dst), out, ['DPA_*', 'DPB_*'],
                                          diff_pair_width_from_class=True,
                                          layers=['F.Cu', 'B.Cu'], layer_costs=layer_costs)
    pcb = parse_kicad_pcb(evidence(out))
    with contextlib.redirect_stdout(io.StringIO()):
        drc = run_drc(out, clearance=0.25, quiet=True, print_summary=False,
                      check_sizes=False, clearance_margin=0.0)
        conn = run_connectivity_check(out, net_patterns=['DPA_*', 'DPB_*'], quiet=True)
    # Margin 0 past the writer's 1 nm coordinate rounding: a pair coupled
    # exactly at the rule reads up to sqrt(2) nm inside it in the file.
    from kicad_writer import EDGE_ROUNDING_MM
    drc = [v for v in drc if v.get('overlap_mm', 1.0) > EDGE_ROUNDING_MM]
    return log.getvalue(), pcb, drc, conn


def _coupled_pitches(pcb, width):
    """Centre distance from each N leg of `width` to the nearest P leg of the
    same pair and layer (P may be necked at the attach, so match on N)."""
    names = {n.net_id: n.name for n in pcb.nets.values()}
    out = []
    for s in pcb.segments:
        nm = names.get(s.net_id, '')
        if not nm.endswith('_N') or abs(s.width - width) > 1e-6:
            continue
        partner = [t for t in pcb.segments
                   if names.get(t.net_id) == nm[:-2] + '_P' and t.layer == s.layer]
        if partner:
            out.append(round(min(_seg_dist(s, t) for t in partner), 3))
    return out


def main():
    with tempfile.TemporaryDirectory() as td:
        log, pcb, drc, conn = _route(td, dru=True)
        check('rule: the widths line names F.Cu 0.15',
              'Design-rule layer widths: F.Cu=0.15mm, B.Cu=0.25mm' in log)
        fcu = sorted({round(s.width, 4) for s in pcb.segments if s.layer == 'F.Cu'})
        check('rule: coupled F.Cu legs are drawn at 0.15', 0.15 in fcu, str(fcu))
        pitches = _coupled_pitches(pcb, 0.15)
        check('rule: the 0.15 run sits at the 0.40 layer pitch (not 0.50)',
              pitches and all(abs(p - 0.40) < 1e-3 for p in pitches), str(pitches))
        check('rule: DRC clean at the routed 0.25 clearance', list(drc) == [], str(drc)[:200])
        check('rule: every pair net connected', list(conn) == [], str(conn)[:200])

        log0, pcb0, drc0, conn0 = _route(td, dru=False)
        check('plain: no widths line', 'Design-rule layer widths' not in log0)
        w0 = sorted({round(s.width, 4) for s in pcb0.segments})
        check('plain: no leg below the 0.25 class width', w0 and min(w0) >= 0.2495, str(w0))
        check('plain: DRC clean at 0.25', list(drc0) == [], str(drc0)[:200])
        check('plain: every pair net connected', list(conn0) == [], str(conn0)[:200])

        # F.Cu priced up so the coupled run drops to B.Cu, the layer no rule scopes
        log1, pcb1, drc1, conn1 = _route(td, dru=True, label='class_tw', pro=PRO_TW,
                                         layer_costs=[20, 1])
        check('class track_width 0.3: the unscoped layer keeps the 0.25 pair width',
              'Design-rule layer widths: F.Cu=0.15mm, B.Cu=0.25mm' in log1,
              next((l.strip() for l in log1.splitlines() if 'layer widths' in l), ''))
        w1 = sorted({round(s.width, 4) for s in pcb1.segments if s.layer == 'B.Cu'})
        check('class track_width 0.3: the B.Cu run is drawn at 0.25, not 0.3',
              w1 and max(w1) <= 0.2501, str(w1))
        check('class track_width 0.3: DRC clean at 0.25', list(drc1) == [], str(drc1)[:200])
        check('class track_width 0.3: every pair net connected', list(conn1) == [],
              str(conn1)[:200])

        log2, _pcb2, drc2, conn2 = _route(td, dru=True, label='split', pro=PRO_SPLIT)
        check('split classes: the rule is not applied', 'Design-rule layer widths' not in log2)
        check('split classes: the run names the layer the pairs disagree on',
              'NOT applied' in log2 and 'B.Cu (0.2/0.25mm)' in log2,
              next((l.strip() for l in log2.splitlines() if 'NOT applied' in l), log2[-300:]))
        check('split classes: DRC clean at 0.25', list(drc2) == [], str(drc2)[:200])
        check('split classes: every pair net connected', list(conn2) == [], str(conn2)[:200])
    if FAILS:
        print(f'\nFAIL: {len(FAILS)} check(s): ' + '; '.join(FAILS))
        return 1
    print('\nALL CHECKS PASSED')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
