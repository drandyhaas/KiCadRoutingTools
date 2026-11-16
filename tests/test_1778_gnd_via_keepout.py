#!/usr/bin/env python3
"""GND return vias stay out of keep-outs that forbid vias (#1778).

`add_gnd_vias_to_existing_board` (route_planes --add-gnd-vias and the Planes
tab's "Add GND vias near signal vias", one engine call) judged a candidate
site by the obstacle map's TRACK cells. A rule area that forbids vias but
allows tracks is stamped into the via cells alone (obstacle_map's rule-area
stamp), so its interior read clear and a return via could land inside it
(KiCad: items_not_allowed). The site check now also asks
`check_drc.copper_keepout_gate`, and the search goes on to the next angle.

The board is written here from s-expressions (the suite image has no KiCad):
a 2-layer 30 x 20 mm board, a SIG via at (15, 10) with nothing near it, and
a B.Cu rule area x 15.3..25 (vias not allowed, tracks and pours allowed) to
its right, where the search's first angle (0 deg) looks.

  1. with the area: route_planes --add-gnd-vias places a GND via near the
     SIG via (non-vacuity) and none in the area (check_drc's census);
  2. control: the same area with vias ALLOWED -- the via lands inside the
     area's outline, so the area is where the search goes first and the
     gate is what moved it.

    python3 tests/test_1778_gnd_via_keepout.py
"""
import contextlib
import io
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402
from kicad_parser import parse_kicad_pcb                         # noqa: E402
from check_drc import keepout_census                             # noqa: E402

failures = []
SIG_VIA = (15.0, 10.0)
AREA_X0 = 15.3

HEAD = '''(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(13 "F.Paste" user)
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(25 "Edge.Cuts" user)
\t\t(35 "F.Fab" user)
\t)
\t(setup
\t\t(pad_to_mask_clearance 0)
\t)
'''


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def _pad_fp(ref, x, y, net):
    return ('\t(footprint ""\n\t\t(layer "F.Cu")\n\t\t(at %g %g)\n'
            '\t\t(property "Reference" "%s"\n\t\t\t(at 0 0 0)\n'
            '\t\t\t(layer "F.SilkS")\n\t\t\t(effects\n\t\t\t\t(font\n'
            '\t\t\t\t\t(size 1.27 1.27)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
            '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 1 1)\n'
            '\t\t\t(layers "F.Cu" "F.Mask" "F.Paste")\n\t\t\t(net "%s")\n'
            '\t\t)\n\t)\n' % (x, y, ref, net))


def _edge(x1, y1, x2, y2):
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n'
            '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
            '\t\t(layer "Edge.Cuts")\n\t)\n' % (x1, y1, x2, y2))


def _via(x, y, net):
    return ('\t(via\n\t\t(at %g %g)\n\t\t(size 0.6)\n\t\t(drill 0.3)\n'
            '\t\t(layers "F.Cu" "B.Cu")\n\t\t(net "%s")\n'
            '\t\t(uuid "17780000-0000-4000-8000-000000000001")\n\t)\n' % (x, y, net))


def _area(vias_allowed):
    return ('\t(zone\n\t\t(layers "B.Cu")\n\t\t(name "NOVIA")\n\t\t(hatch edge 0.5)\n'
            '\t\t(connect_pads\n\t\t\t(clearance 0)\n\t\t)\n'
            '\t\t(min_thickness 0.25)\n\t\t(keepout\n'
            '\t\t\t(tracks allowed)\n\t\t\t(vias %s)\n'
            '\t\t\t(pads allowed)\n\t\t\t(copperpour allowed)\n'
            '\t\t\t(footprints allowed)\n\t\t)\n'
            '\t\t(fill\n\t\t\t(thermal_gap 0.5)\n'
            '\t\t\t(thermal_bridge_width 0.5)\n\t\t)\n'
            '\t\t(polygon\n\t\t\t(pts\n\t\t\t\t(xy %g 5) (xy 25 5) '
            '(xy 25 15) (xy %g 15)\n\t\t\t)\n\t\t)\n\t)\n'
            % ('allowed' if vias_allowed else 'not_allowed', AREA_X0, AREA_X0))


def write_board(path, vias_allowed):
    parts = [HEAD, _pad_fp('J1', 3, 3, 'GND'), _pad_fp('J2', 3, 17, 'SIG'),
             _via(*SIG_VIA, 'SIG')]
    corners = [(0, 0), (30, 0), (30, 20), (0, 20)]
    for i in range(4):
        parts.append(_edge(*corners[i], *corners[(i + 1) % 4]))
    parts.append(_area(vias_allowed))
    parts.append(')\n')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(''.join(parts))
    return run_utils.evidence(path, 'gnd-via board')


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def gnd_vias(tmp, name, vias_allowed):
    src = write_board(os.path.join(tmp, name + '.kicad_pcb'), vias_allowed)
    out = os.path.join(tmp, name + '.out.kicad_pcb')
    r = subprocess.run([sys.executable, '-B', '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'route_planes.py'), src, out,
                        '--nets', 'GND', 'GND', '--plane-layers', 'F.Cu', 'B.Cu',
                        '--add-gnd-vias', '--gnd-via-distance', '3.0'],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    log = r.stdout + r.stderr
    if 'Traceback' in log or r.returncode != 0:
        raise AssertionError(f'BROKEN TEST: route_planes failed ({r.returncode}):\n{log[-1500:]}')
    pcb = parse(run_utils.evidence(out, f'{name} output'))
    gnd = {nid for nid, n in pcb.nets.items() if n.name == 'GND'}
    vias = [v for v in pcb.vias if v.net_id in gnd]
    rows = keepout_census(pcb, pcb.board_info.copper_layers, segments=[], vias=vias)
    return vias, rows


def end_to_end(tmp):
    print('1. vias not allowed in the area')
    vias, rows = gnd_vias(tmp, 'novia', False)
    ok('a GND via is placed near the SIG via', len(vias) == 1,
       repr([(round(v.x, 2), round(v.y, 2)) for v in vias]))
    ok('no GND via in the area (check_drc census)', rows == [], repr(rows))
    ok('the via sits left of the area',
       all(v.x + v.size / 2 <= AREA_X0 + 1e-6 for v in vias),
       repr([(round(v.x, 2), round(v.y, 2)) for v in vias]))
    print('2. control: vias allowed in the area')
    vias, rows = gnd_vias(tmp, 'allowed', True)
    ok('a GND via is placed', len(vias) == 1)
    ok('wholly inside the area outline (where the search looks first)',
       len(vias) == 1 and vias[0].x - vias[0].size / 2 > AREA_X0,
       repr([(round(v.x, 2), round(v.y, 2)) for v in vias]))


def main():
    tmp = tempfile.mkdtemp(prefix='t1778g_')
    try:
        end_to_end(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
