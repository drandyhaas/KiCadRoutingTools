#!/usr/bin/env python3
"""#1217: four papercuts from fa10 board 06 (rp2350). The fifth, KiCad's
track_dangling on a joint stub lying on one other track, is row 23 of
tests/test_check_weird.py.

  1. "Copper-to-hole clearance 0.25mm" did not say the floor covers NPTH
     holes only, and FAB FLOOR RELAXED advised "re-route at that floor" for a
     key no router setting holds at a via drill or PTH barrel.
  2. The under-pad warning suggested "Try via <= 0.04mm" -- pitch minus
     track and clearance, clamped to nothing a fab makes.
  3. A dropped ball carried no reason, and the closing line offered only a
     smaller --clearance (which changed nothing on rp2350 seed 3).
  4. The extra-ball strap was a 0.05-grid A* only and missed a clean straight
     diagonal to a same-net ball (C4 -> D5, 23 um of slack).

Checks:
  1. resolve_hole_clearance announces "for NPTH holes ... plated holes and
     vias: copper clearance"; the relaxation line for min_hole_clearance says
     the router holds it at NPTH walls only.
  2./3. bga_fanout on tracked glasgow_revC U30 with a 0.6 mm via at 0.8 mm
     pitch and --escalation off: the warning names the fab ladder's smallest
     via instead of a sub-floor size, and the dropped balls get one line each
     naming what crowds them.
  4. A synthetic 3x3 BGA: an extra ball diagonal to a same-net ball is
     strapped by ONE exact straight segment; with a foreign via on that
     diagonal the straight segment is refused and nothing laid grazes it.

    python3 tests/test_1217_rp2350_papercuts.py
"""
import contextlib
import io
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from copy_board import copy_board                              # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet_parse(path):
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


BGA9 = """(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "N")
  (net 2 "X1") (net 3 "X2") (net 4 "X3") (net 5 "X4")
  (net 6 "X5") (net 7 "X6") (net 8 "X7")
  (footprint "t:BGA9" (layer "F.Cu") (at 10 10)
    (property "Reference" "U1" (at 0 0) (layer "F.SilkS"))
{pads}  )
  (gr_rect (start 0 0) (end 20 20) (stroke (width 0.1) (type default))
    (fill none) (layer "Edge.Cuts"))
)
"""


def bga9(path):
    nets = {('A', 1): (1, 'N'), ('B', 2): (1, 'N')}
    lines, k = [], 2
    for r, row in enumerate('ABC'):
        for c in (1, 2, 3):
            nid, nm = nets.get((row, c), (None, None))
            if nid is None:
                nid, nm = k, f'X{k - 1}'
                k += 1
            lines.append(f'    (pad "{row}{c}" smd circle (at {(c - 2) * 0.8:g} '
                         f'{(r - 1) * 0.8:g}) (size 0.4 0.4) (layers "F.Cu") '
                         f'(net {nid} "{nm}"))\n')
    open(path, 'w').write(BGA9.replace('{pads}', ''.join(lines)))


def main():
    tmp = tempfile.mkdtemp(prefix='t1217_')
    try:
        # 1. Copper-to-hole wording.
        src = os.path.join(ROOT, 'kicad_files', 'splitflap_driver.kicad_pcb')
        b = os.path.join(tmp, 'sf.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(src, b)
        json.dump({"board": {"design_settings": {"rules": {"min_hole_clearance": 0.3}}},
                   "meta": {"version": 1}},
                  open(os.path.splitext(b)[0] + '.kicad_pro', 'w'))
        import obstacle_map
        from types import SimpleNamespace
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            v = obstacle_map.resolve_hole_clearance(
                SimpleNamespace(source_path=b), SimpleNamespace(hole_clearance=0.0),
                pcb_file=b)
        line = buf.getvalue().strip()
        check('the copper-to-hole floor says NPTH only',
              abs(v - 0.3) < 1e-9 and 'for NPTH holes' in line
              and 'plated holes and vias: copper clearance' in line, line)
        from fix_kicad_drc_settings import _fab_floor_disclosure
        lines = _fab_floor_disclosure(
            '', {'min_hole_clearance': 0.25},
            {'board': {'design_settings': {'rules': {'min_hole_clearance': 0.1}}}},
            {}, objects={})
        rel = next((l for l in lines if 'copper-to-hole' in l), '')
        check('the relaxation line says re-routing cannot restore it at plated holes',
              'NPTH walls only' in rel, rel.strip())

        # 2./3. The under-pad warning and the dropped-ball report.
        g = os.path.join(tmp, 'g.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb'), g)
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', 'py_router/bga_fanout.py', g, '-o',
             os.path.join(tmp, 'g_out.kicad_pcb'), '--component', 'U30',
             '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu', '--track-width', '0.25',
             '--clearance', '0.25', '--via-size', '0.6', '--via-drill', '0.3',
             '--nets', '/IO_Banks/DA*', '--escape-method', 'underpad',
             '--escalation', 'off'],
            cwd=ROOT, capture_output=True, text=True, encoding='utf-8', errors='replace')
        log = r.stdout + r.stderr
        warn = next((l for l in log.splitlines() if 'Under-pad: WARNING' in l), '')
        check('the warning names the fab ladder, not a sub-floor via',
              'No via fits' in warn and 'smallest the fab ladder reaches' in warn
              and 'Try via <=' not in warn, warn.strip()[:200])
        summ = next((l for l in log.splitlines() if l.startswith('JSON_SUMMARY:')), '')
        failed = json.loads(summ.split(':', 1)[1]).get('failed', 0) if summ else 0
        balls = [l for l in log.splitlines() if l.startswith('    U30.')]
        check('precondition: the run dropped balls', failed >= 1, str(failed))
        check('each dropped ball gets a line naming what crowds it',
              'Dropped ball(s) on U30' in log and len(balls) == failed
              and all('nearest foreign copper' in l for l in balls),
              balls[0][:200] if balls else log[-300:])
        check('the closing advice points at the per-ball lines',
              "'Dropped ball(s)' lines above" in log)

        # 4. The straight strap.
        from bga_fanout import _strap_unescaped_extras
        bp = os.path.join(tmp, 'bga9.kicad_pcb')
        bga9(bp)
        pcb = quiet_parse(bp)
        fp = pcb.footprints['U1']
        b2 = next(p for p in fp.pads if p.pad_number == 'B2')
        a1 = next(p for p in fp.pads if p.pad_number == 'A1')
        tracks = []
        n, bare = _strap_unescaped_extras(fp, pcb, [b2], tracks, [], 0.1, 0.1,
                                          0.3, 0.15, 0.1)
        ends = {(round(a1.global_x, 3), round(a1.global_y, 3)),
                (round(b2.global_x, 3), round(b2.global_y, 3))}
        check('a clean diagonal is strapped by one exact straight segment',
              n == 1 and len(tracks) == 1
              and {tuple(round(c, 3) for c in tracks[0]['start']),
                   tuple(round(c, 3) for c in tracks[0]['end'])} == ends,
              json.dumps(tracks))
        from kicad_parser import Via
        mid = ((a1.global_x + b2.global_x) / 2, (a1.global_y + b2.global_y) / 2)
        pcb2 = quiet_parse(bp)
        pcb2.vias.append(Via(x=mid[0], y=mid[1], size=0.2, drill=0.1,
                             layers=['F.Cu', 'B.Cu'], net_id=2))
        fp2 = pcb2.footprints['U1']
        b2b = next(p for p in fp2.pads if p.pad_number == 'B2')
        t2 = []
        n2, bare2 = _strap_unescaped_extras(fp2, pcb2, [b2b], t2, [], 0.1, 0.1,
                                            0.3, 0.15, 0.1)
        from geometry_utils import point_to_segment_distance
        gap = min((point_to_segment_distance(mid[0], mid[1], *t['start'], *t['end'])
                   - 0.1 - t['width'] / 2 for t in t2), default=None)
        straight = any({tuple(round(c, 3) for c in t['start']),
                        tuple(round(c, 3) for c in t['end'])} == ends for t in t2)
        check('a foreign via on the diagonal refuses the straight segment',
              not straight and (gap is None or gap >= 0.1 - 1e-6),
              f'strapped {n2}, bare {bare2}, gap {gap}')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
