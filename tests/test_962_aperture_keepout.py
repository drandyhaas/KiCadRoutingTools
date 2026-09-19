#!/usr/bin/env python3
"""#962: `--same-net-pad-clearance` keeps vias out of the net's solder-paste OPENINGS.

`--same-net-pad-clearance 0.4` measured against the declared pad rectangle
only. On the run-25 esp_prog placement, U2's pad 2 is F.Cu-only inside a
4.5 x 1.6 mm F.Paste opening, and the router's `Net-(C1-Pad1)` via landed
0.55 mm off the pad and still inside the paste (P5 reproduction).

Invariants:
1. Cells:
   - at snpc 0.4 the keep-out covers the reproduction via's cell, and the
     aperture half of it is non-empty for `Net-(C1-Pad1)`;
   - negative controls: snpc -1 or 0 stamps nothing, and GND's keep-out does
     not include U2's opening.
2. The plane-step via map (`build_via_obstacle_map`) blocks the same cell for
   the target net under the flag, and not without it (positive control).
3. The sub-grid nudge treats the opening as an obstacle under the flag. A via
   0.01 mm short of the required clearance is moved clear; without the flag it
   is left alone.
4. The #907 seal hint NAMES the opening when the opening's cells are what seal
   the pad.
5. End to end, routing the run-25 fixture: without the flag a
   `Net-(C1-Pad1)` via lands in the opening (positive control, the P5
   reproduction). At 0.4 no via is in any opening that concerns its net.

Run:
    python3 tests/test_962_aperture_keepout.py
"""
import os
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

import numpy as np  # noqa: E402
from kicad_parser import parse_kicad_pcb, Via  # noqa: E402
from routing_config import GridRouteConfig  # noqa: E402
import obstacle_map as om  # noqa: E402
import paste_apertures as pa  # noqa: E402
import fab_notes  # noqa: E402
from copy_board import copy_board  # noqa: E402
from run_utils import check as run_check  # noqa: E402

RUN_ALL_TIMEOUT = 1500
FAILS = []
FIX = os.path.join(ROOT, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')
REPRO_VIA = (124.8, 94.7)       # the P5 reproduction's in-aperture via


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def net(p, name):
    return next(n for n, v in p.nets.items() if v.name == name)


def cfg(snpc):
    c = GridRouteConfig()
    c.same_net_pad_clearance = snpc
    return c


def main():
    p = parse_kicad_pcb(FIX)
    c1, gnd = net(p, 'Net-(C1-Pad1)'), net(p, 'GND')
    u2ap = [a for a in p.paste_apertures if a.owner_ref == 'U2' and a.source == 'graphic'][0]

    # 1
    c = cfg(0.4)
    coord = om.GridCoord(c.grid_step)
    cell = tuple(coord.to_grid(*REPRO_VIA))
    cells = set(map(tuple, om.same_net_pad_via_keepout_cells(p, c1, c).tolist()))
    check('1. snpc 0.4: the reproduction via cell is kept out', cell in cells)
    ap_cells = om.same_net_pad_via_keepout_cells(p, c1, c, pads=[], apertures=[u2ap])
    check('1. ... and the APERTURE half alone covers it',
          cell in set(map(tuple, ap_cells.tolist())), str(len(ap_cells)))
    pad_only = set(map(tuple, om.same_net_pad_via_keepout_cells(
        p, c1, c, pads=p.pads_by_net[c1]).tolist()))
    check('1. ... which the pad-rectangle half alone does NOT (the #962 gap)',
          cell not in pad_only)
    for s in (-1.0, 0.0):
        check('1. snpc %g stamps nothing (unchanged #581 semantics)' % s,
              len(om.same_net_pad_via_keepout_cells(p, c1, cfg(s))) == 0)
    check('1. GND\'s keep-out does not include U2\'s opening',
          u2ap not in om.paste_keepout_apertures(p, gnd))

    # 2 -- the plane-step via map. On the real board every cell around U2's
    # opening (and around GND's larger-than-pad USB1 openings) is blocked by
    # other copper anyway, so no control can discriminate there. A synthetic
    # board isolates it: ONE pad whose (solder_paste_margin 0.5) makes its
    # opening 0.5 mm wider than the pad, with nothing else near.
    from plane_obstacle_builder import build_via_obstacle_map
    work2 = tempfile.mkdtemp(prefix='krt962k2_')
    try:
        sp = os.path.join(work2, 'one_pad.kicad_pcb')
        with open(sp, 'w', encoding='utf-8') as fh:
            fh.write('(kicad_pcb (version 20240108) (generator "t")\n'
                     ' (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (35 "F.Paste" user) '
                     '(44 "Edge.Cuts" user))\n (setup)\n (net 0 "") (net 1 "/A")\n'
                     ' (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type solid)) '
                     '(fill no) (layer "Edge.Cuts"))\n'
                     ' (footprint "L:P" (layer "F.Cu") (at 20 15) (property "Reference" "U1")\n'
                     '  (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu" "F.Paste") '
                     '(net 1 "/A") (solder_paste_margin 0.5))))\n')
        sq = parse_kicad_pcb(sp)
        gaps = om.paste_keepout_apertures(sq, 1)
        band = om.paste_aperture_keepout_cells(sq, 1, cfg(0.4), 0.4, apertures=gaps)
        vm_on = build_via_obstacle_map(sq, cfg(0.4), exclude_net_id=1, same_net_pad_clearance=0.4)
        # the config must be OFF too: build_via_obstacle_map falls back to the
        # config's same_net_pad_clearance when the argument is -1
        vm_off = build_via_obstacle_map(sq, cfg(-1.0), exclude_net_id=1, same_net_pad_clearance=-1.0)
        check('2. plane via map under the flag: EVERY opening keep-out cell is blocked',
              len(band) > 0 and all(vm_on.is_via_blocked(int(a), int(b)) for a, b in band))
        freed = sum(1 for a, b in band if not vm_off.is_via_blocked(int(a), int(b)))
        check('2. ... and without it they are free (positive control)',
              freed == len(band) > 0, f'{freed} of {len(band)}')
        # 0 keeps its legacy meaning (the declared pad only), as on the
        # routing path: the opening's extra 0.5 mm is NOT kept out
        vm_zero = build_via_obstacle_map(sq, cfg(0.0), exclude_net_id=1, same_net_pad_clearance=0.0)
        free0 = sum(1 for a, b in band if not vm_zero.is_via_blocked(int(a), int(b)))
        check('2. at an explicit 0 the plane via map adds no opening keep-out '
              '(only the pad blocks)', 0 < free0 < len(band), f'{free0} of {len(band)} free')
    finally:
        shutil.rmtree(work2, ignore_errors=True)

    # 3 -- the nudge treats the opening as a CANDIDATE constraint (it moves
    # vias that graze foreign copper, #280). A via just clear of the opening's
    # keep-out, grazing a foreign track on the far side: the only fix pushes
    # it into the keep-out, so under the flag it must stay; without it, it moves.
    from pcb_modification import nudge_grazing_vias
    from kicad_parser import Segment
    r = 0.15
    yc = (u2ap.bounds[1] + u2ap.bounds[3]) / 2.0
    x = u2ap.bounds[2]
    while pa.aperture_distance(x, yc, u2ap) < r + 0.4 + 0.005:
        x += 0.0005
    for snpc, want_moved in ((0.4, False), (-1.0, True)):
        pp = parse_kicad_pcb(FIX)
        v = Via(x=x, y=yc, size=2 * r, drill=0.15, layers=['F.Cu', 'B.Cu'], net_id=c1)
        pp.vias.append(v)
        tx = x + r + 0.1 + 0.1 - 0.01          # 0.2 mm track grazing by 0.01 mm
        pp.segments.append(Segment(tx, yc - 1.0, tx, yc + 1.0, 0.2, 'F.Cu', gnd))
        moved, _nets, _mv = nudge_grazing_vias([{'new_vias': [v]}], pp, {c1},
                                               clearance=0.1, max_shift=0.025,
                                               same_net_pad_clearance=snpc)
        ap = [a for a in pp.paste_apertures if a.owner_ref == 'U2' and a.source == 'graphic'][0]
        d = pa.aperture_distance(v.x, v.y, ap)
        if want_moved:
            check('3. snpc -1: the opening is no constraint, the graze is nudged (control)',
                  moved == 1, f'moved={moved}')
        else:
            check('3. snpc 0.4: the nudge will NOT push the via into the opening\'s keep-out',
                  moved == 0 and d >= r + 0.4 - 1e-6, f'moved={moved} d={d:.4f}')

    # 4 -- the seal hint names the opening
    from routing_diagnostics import same_net_pad_seal_hint
    pad2 = [pd for pd in p.footprints['U2'].pads if pd.pad_number == '2'][0]
    vm = om.GridObstacleMap(4) if hasattr(om, 'GridObstacleMap') else None
    if vm is not None:
        c = cfg(0.4)
        keep = om.same_net_pad_via_keepout_cells(p, c1, c)
        keep_set = set(map(tuple, keep.tolist()))
        pgx, pgy = coord.to_grid(pad2.global_x, pad2.global_y)
        span = int(round((max(pad2.size_x, pad2.size_y) / 2 + 1.0) / c.grid_step)) + 1
        others = np.array([(gx, gy) for gx in range(pgx - span, pgx + span + 1)
                           for gy in range(pgy - span, pgy + span + 1)
                           if (gx, gy) not in keep_set], dtype=np.int32)
        if len(others):
            vm.add_blocked_vias_batch(others)
        vm.add_blocked_vias_batch(np.ascontiguousarray(keep, dtype=np.int32))
        hint, verdict = same_net_pad_seal_hint(p, c, c1, obstacles=vm, return_verdict=True)
        check('4. the seal hint fires and names U2\'s paste opening',
              bool(verdict) and 'U2 F.Paste (graphic)' in (verdict or {}).get('aperture', [])
              and 'paste opening' in hint, str(verdict))

    # 5 -- end to end
    work = tempfile.mkdtemp(prefix='krt962k_')
    try:
        src = os.path.join(work, 'in.kicad_pcb')
        copy_board(FIX, src)
        hits = {}
        for tag, extra in (('off', []), ('on', ['--same-net-pad-clearance', '0.4'])):
            out = os.path.join(work, tag + '.kicad_pcb')
            run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route.py'),
                       src, '--output', out] + extra, accept=True, timeout=1200)
            q = parse_kicad_pcb(out)
            hits[tag] = [(round(v.x, 3), round(v.y, 3), ap.label())
                         for v, ap, _pen in fab_notes.via_paste_sites(q.vias, q)
                         if ap.source != 'pad']
        check('5. positive control: without the flag a via lands in a graphic/pane opening',
              any('U2 F.Paste (graphic)' in h[2] for h in hits['off']), str(hits['off']))
        check('5. --same-net-pad-clearance 0.4: NO via in a graphic/pane opening',
              not hits['on'], str(hits['on']))
    finally:
        shutil.rmtree(work, ignore_errors=True)

    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
