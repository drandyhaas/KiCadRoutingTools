#!/usr/bin/env python3
"""The plane's web through an array's via lattice (py_router/plane_lattice.py).

  python3 tests/test_plane_lattice.py

A pour reaches the drops under a ball-grid array only through the copper left
between two vias one pitch apart: pitch - via - 2 * clearance, which the fill
drops below the zone's minimum width. Two levers keep it: the planes step
lowers the pour's clearance (route_planes), and the fanout steps its via down
the fab ladder when the pour over the array stands too wide (bga_fanout) --
even below an explicit via size.

1. THE RULE: the numbers of plane_lattice's web, at zynq_ad9364's sizes (0.8 mm
   pitch, 0.45 mm vias, 0.1 mm min width): a 0.2 mm pour has no web, 0.12 has.
2. THE RUNG: plane_web_via picks the largest ladder via that threads, keeps
   the asked one when it threads, and says when none does.
3. THE FANOUT, on kicad_files/ulx3s.kicad_pcb U1 (0.8 mm, no zone of its own)
   with a GND pour laid under it in memory: at 0.2 mm the escape and drop vias
   come down to a via the pour passes between (recorded as a narrowing); at
   0.12 mm -- the CONTROL -- they stay at the 0.45 asked. With --escalation off
   nothing may narrow, and the vias stay as asked.

Uses kicad_files/ulx3s.kicad_pcb; part 3 skips cleanly if absent.
"""
import contextlib
import io
import os
import sys
from types import SimpleNamespace

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import plane_lattice as pl  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
LAYERS = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
LADDER = [{'via_diameter': 0.45, 'via_drill': 0.2}, {'via_diameter': 0.30, 'via_drill': 0.15},
          {'via_diameter': 0.25, 'via_drill': 0.15}]


def rule(fails):
    if not abs(pl.clearance_to_thread(0.8, 0.45, 0.1) - 0.12) < 1e-9:
        fails.append(f'clearance to thread 0.45 vias at 0.8: {pl.clearance_to_thread(0.8, 0.45, 0.1)}, want 0.12')
    if not abs(pl.via_to_thread(0.8, 0.2, 0.1) - 0.29) < 1e-9:
        fails.append(f'via to thread a 0.2 pour at 0.8: {pl.via_to_thread(0.8, 0.2, 0.1)}, want 0.29')
    if pl.threads(0.8, 0.45, 0.2, 0.1):
        fails.append('a 0.2 pour threads 0.45 vias at 0.8 pitch (it leaves no web)')
    if not pl.threads(0.8, 0.45, 0.12, 0.1):
        fails.append('a 0.12 pour does not thread 0.45 vias at 0.8 pitch')


def synthetic(clearance, net=7, other=8):
    """a 4x4 ball grid at 0.8 mm, half its balls on `net`, under a zone of `net` at `clearance`"""
    pads = [SimpleNamespace(global_x=i * 0.8, global_y=j * 0.8, net_id=net if (i + j) % 2 else other)
            for i in range(4) for j in range(4)]
    fp = SimpleNamespace(pads=pads, reference='U9')
    zone = SimpleNamespace(net_id=net, net_name='GND', layer='In1.Cu', clearance=clearance, min_thickness=0.1,
                           polygon=[(-5, -5), (10, -5), (10, 10), (-5, 10)], in_footprint=False)
    return fp, SimpleNamespace(zones=[zone])


def rung(fails):
    fp, pcb = synthetic(0.2)
    r = pl.plane_web_via(fp, pcb, 0.45, 0.2, 0.1, LADDER, pitch=0.8)
    if not r or not r['threads'] or abs(r['via'] - 0.25) > 1e-9 or abs(r['drill'] - 0.15) > 1e-9:
        fails.append(f'0.2 pour: want the 0.25/0.15 rung (0.30 leaves 0.09 of web past the margin), got {r}')
    fp, pcb = synthetic(0.12)
    r = pl.plane_web_via(fp, pcb, 0.45, 0.2, 0.1, LADDER, pitch=0.8)
    if r is not None:
        fails.append(f'0.12 pour: the 0.45 via threads and nothing should change, got {r}')
    fp, pcb = synthetic(0.2)
    r = pl.plane_web_via(fp, pcb, 0.45, 0.2, 0.1, [], pitch=0.8)
    if not r or r['threads'] or r['via'] != 0.45:
        fails.append(f'no ladder (escalation off): want the asked via kept and threads False, got {r}')
    fp, pcb = synthetic(0.2, net=7, other=8)
    pcb.zones[0].net_id = 99                   # a pour of a net with no ball on the array asks nothing
    if pl.plane_web_via(fp, pcb, 0.45, 0.2, 0.1, LADDER, pitch=0.8) is not None:
        fails.append('a pour of a net with no ball on the array changed the via')


def fanout(fails):
    if not os.path.exists(BOARD):
        print(f'  [SKIP] board not present: {BOARD}')
        return
    from kicad_parser import parse_kicad_pcb, Zone
    from bga_fanout import generate_bga_fanout
    import fab_tiers

    def run(clearance, policy=None):
        prev = fab_tiers.get_escalation_policy()
        try:
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                if policy:
                    fab_tiers.set_escalation_policy(policy)
                else:
                    fab_tiers.reset_ledger()
                pcb = parse_kicad_pcb(BOARD)
                fp = pcb.footprints['U1']
                gnd = next(p.net_id for p in fp.pads if p.net_name == 'GND')
                xs = [p.global_x for p in fp.pads]
                ys = [p.global_y for p in fp.pads]
                pcb.zones = list(pcb.zones or []) + [Zone(
                    net_id=gnd, net_name='GND', layer='In1.Cu', clearance=clearance, min_thickness=0.1,
                    polygon=[(min(xs) - 3, min(ys) - 3), (max(xs) + 3, min(ys) - 3), (max(xs) + 3, max(ys) + 3),
                             (min(xs) - 3, max(ys) + 3)])]
                sig = next(p for p in fp.pads if p.pad_number == 'F5').net_name
                _t, vias, _vr, _f = generate_bga_fanout(
                    fp, pcb, layers=LAYERS, track_width=0.12, clearance=0.1, via_size=0.45, via_drill=0.2,
                    net_filter=[sig, 'GND'], plane_drop='auto')
                rows = [r for r in fab_tiers.escalation_summary()['narrowed'] if 'plane web' in str(r.get('site'))]
            return vias, rows, out.getvalue()
        finally:
            fab_tiers.set_escalation_policy(*prev)

    vias, rows, log = run(0.2)
    sizes = sorted({round(v['size'], 3) for v in vias})
    print(f'  0.2 pour: {len(vias)} via(s), sizes {sizes}; narrowing rows {len(rows)}')
    if not vias:
        fails.append('0.2 pour: the fanout laid no via -- the arm tests nothing')
    elif max(sizes) > 0.29 + 1e-9:
        fails.append(f'0.2 pour: a via of {max(sizes)} under U1 leaves the pour no web (want <= 0.29)')
    if not rows or 'Plane web under U1' not in log:
        fails.append('0.2 pour: the step-down was not disclosed (no narrowing row, or no line in the log)')
    vias, rows, _log = run(0.12)
    sizes = sorted({round(v['size'], 3) for v in vias})
    print(f'  0.12 pour (control): {len(vias)} via(s), sizes {sizes}')
    if 0.45 not in sizes or rows:
        fails.append(f'0.12 pour: the 0.45 via threads, want it laid as asked and no narrowing (sizes {sizes}, '
                     f'rows {len(rows)})')
    vias, rows, log = run(0.2, policy='off')
    sizes = sorted({round(v['size'], 3) for v in vias})
    print(f'  0.2 pour, escalation off: sizes {sizes}')
    if 0.45 not in sizes or rows or 'no via on the fab ladder' not in log:
        fails.append(f'escalation off: want the asked 0.45 kept, no narrowing and the warning (sizes {sizes})')


def main():
    print('=' * 60)
    print("plane_lattice: the pour's web through an array's vias")
    print('=' * 60)
    fails = []
    rule(fails)
    rung(fails)
    fanout(fails)
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print('PASS: the web rule, the rung, and the fanout stepping its via down only where the pour needs it')
    return 0


if __name__ == '__main__':
    sys.exit(main())
