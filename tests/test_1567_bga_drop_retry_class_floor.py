#!/usr/bin/env python3
"""#1567: bga_fanout's plane-drop fab-floor retry never descends a drop net
below its own net class.

A drop that fails at the run's via is retried once at the fab-floor rung. The
#1529 class passes floor that retry at a class ABOVE --clearance, but a drop
net in a class BELOW it rode the run-clearance pass, whose retry stopped at
0.0: it laid the net's vias at the 0.09 mm floor. On a 0.8 mm BGA whose GND
sits in a 0.30 class (Default 0.1), escaped at --clearance 0.35 with every
foreign part immovable (the post-route setting, so the decaps are obstacles
and the nominal drops fail), the retry laid 3 GND vias 0.266 mm from the
part's own balls -- below the class, which KiCad grades them at.

The retry now stops each drop net at its own class (and any scoped or track
.kicad_dru rule binding it, #1565), up to the run clearance, and the drops
that cannot meet it are left failed for the plane tap.

Checks (generate_bga_fanout in process, the drop vias measured with
check_drc's own pad gap):
  1. the retry runs at the class (0.30), not the fab floor;
  2. no GND drop via stands below the 0.30 class from foreign copper (was 3);
  3. the drops the class does not admit are reported failed, not laid.
  4. checks 1-3 again with GND's floor set by a .kicad_dru rule on its class
     (#1565) instead of the class value. The rule also holds every foreign
     net at it on the obstacle side, so check 2 held there before; check 1
     (the retry ran at the 0.09 floor) is the one that failed.

    python3 tests/test_1567_bga_drop_retry_class_floor.py
"""
import contextlib
import copy
import io
import json
import math
import os
import re
import shutil
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from copy_board import copy_board                                  # noqa: E402
from run_utils import evidence                                     # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
CLASS = 0.30          # GND's class: below the run clearance, above the fab floor
RUN = 0.35
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def stage(dst, rule=False):
    """GND in class PWR: at CLASS, or (`rule`) at the Default 0.1 with a
    .kicad_dru rule holding PWR copper at CLASS instead."""
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(BOARD, dst)
    pro = os.path.splitext(dst)[0] + '.kicad_pro'
    with open(pro, encoding='utf-8') as f:
        proj = json.load(f)
    ns = proj['net_settings']
    base = next(c for c in ns['classes'] if c['name'] == 'Default')
    base['clearance'] = 0.1
    c = copy.deepcopy(base)
    c.update(name='PWR', clearance=0.1 if rule else CLASS, priority=0)
    ns['classes'] = [base, c]
    ns['netclass_patterns'] = [{'netclass': 'PWR', 'pattern': 'GND'}]
    with open(pro, 'w', encoding='utf-8') as f:
        json.dump(proj, f, indent=2)
    if rule:
        with open(os.path.splitext(dst)[0] + '.kicad_dru', 'w', encoding='utf-8') as f:
            f.write('(version 1)\n(rule "pwr" (condition "A.NetClass == \'PWR\'")\n'
                    f'  (constraint clearance (min {CLASS}mm)))\n')
    return dst


def via_gap(v, pads, segs, vias):
    """The via's smallest edge gap to foreign copper, and what it is."""
    from check_drc import segment_pad_gap
    from geometry_utils import point_to_segment_distance
    r = v['size'] / 2.0
    best = (math.inf, '')
    for p in pads:
        if math.hypot(p.global_x - v['x'], p.global_y - v['y']) < 3.0:
            g = segment_pad_gap(v['x'], v['y'], v['x'], v['y'], p) - r
            if g < best[0]:
                best = (g, f"pad {p.component_ref}.{p.pad_number}")
    for (ax, ay, bx, by, w) in segs:
        g = point_to_segment_distance(v['x'], v['y'], ax, ay, bx, by) - r - w / 2.0
        if g < best[0]:
            best = (g, 'track')
    for (ox, oy, osz) in vias:
        g = math.hypot(ox - v['x'], oy - v['y']) - r - osz / 2.0
        if g < best[0]:
            best = (g, 'via')
    return best


def run(rule):
    """One fanout; checks 1-3 (`rule`: the floor comes from a .kicad_dru rule)."""
    tmp = tempfile.mkdtemp(prefix='t1567d_')
    try:
        src = evidence(stage(os.path.join(tmp, 'in.kicad_pcb'), rule))
        from kicad_parser import parse_kicad_pcb
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            pcb = parse_kicad_pcb(src)
        # the post-route setting: no later step moves a decap off a via
        pcb._fanout_all_foreign_immovable = True
        import bga_fanout
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            tracks, vias, _rm, _failed = bga_fanout.generate_bga_fanout(
                pcb.footprints['U30'], pcb, net_filter=['*', '!GND', '!+3V3', '!+1V2'],
                layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], track_width=0.12,
                clearance=RUN, via_size=0.4, via_drill=0.2)
        log = out.getvalue()
        retry = re.findall(r'Plane drops at the fab floor \(.*clearance ([0-9.]+)\)', log)
        if not retry:
            raise AssertionError("BROKEN TEST: the drop retry did not fire, so "
                                 f"nothing was laid below the run clearance\n{log[-1500:]}")

        gnd = next(i for i, n in pcb.nets.items() if n.name == 'GND')
        pads = [p for fp in pcb.footprints.values() for p in fp.pads
                if p.net_id != gnd and p.pad_type != 'np_thru_hole']
        segs = [(s.start_x, s.start_y, s.end_x, s.end_y, s.width) for s in pcb.segments
                if s.layer.endswith('.Cu') and s.net_id != gnd]
        segs += [(*t['start'], *t['end'], t['width']) for t in tracks if t['net_id'] != gnd]
        others = [(o.x, o.y, o.size) for o in pcb.vias if o.net_id != gnd]
        others += [(o['x'], o['y'], o['size']) for o in vias if o['net_id'] != gnd]
        drops = [v for v in vias if v['net_id'] == gnd]
        if not drops:
            raise AssertionError("BROKEN TEST: no GND drop via was laid")

        print("1. the retry stops at the drop net's class")
        check("the retry runs at the 0.30 floor, not the fab floor",
              all(abs(float(c) - CLASS) < 1e-6 for c in retry), f"retry clearance {retry}")

        print("2. no drop via below its class")
        short = [(v, via_gap(v, pads, segs, others)) for v in drops]
        short = [(v, g) for v, g in short if g[0] < CLASS - 0.005]
        check("no GND drop via stands below the 0.30 floor (was 3 with the class)", not short,
              '; '.join(f"({v['x']:.2f},{v['y']:.2f}) {g[0]:.3f} to {g[1]}"
                        for v, g in short))

        print("3. the drops the class does not admit are left failed")
        rep = bga_fanout.LAST_PLANE_DROP_REPORT
        check("they are reported failed (left for the plane tap)",
              (rep.get('failed') or 0) > 0, f"failed={rep.get('failed')}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)



def main():
    if not os.path.isfile(BOARD):
        print(f"SKIP: {os.path.relpath(BOARD, ROOT)} missing")
        return 77
    run(False)
    print()
    print("4. the same floor from a .kicad_dru rule (#1565) instead of the class")
    run(True)

    print()
    if failures:
        print(f"FAILED: {len(failures)} check(s): " + '; '.join(failures))
        return 1
    print("PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
