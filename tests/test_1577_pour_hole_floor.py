#!/usr/bin/env python3
"""#1577: route_planes writes the copper-to-hole floor it HELD, not the copper
clearance -- the pour step's half of #1369 (route_diff) and #1493 (route.py).

route_planes lowered `rules.min_hole_clearance` to its copper clearance
whenever it drew ANY copper (compute_targets' `clearance if copper_holes`
fallback), outside the design_rules ledger, so --strict-sizes exited 0. Even
the soft-joint bridges its finalize lays on copper the board already carried
counted: orangecrab_ext_pll placed no via and no track, closed 14 joints, and
went 0.2 -> 0.16 while KiCad found the same 31 hole errors at 0.2 on the input
and on the output. Now:

  1. orangecrab_ext_pll, whose pour draws only soft-joint bridges, is a bare
     pour: 0.2 kept (and no track floor), the bridges named, exit 0 under
     --strict-sizes.
  2. kit-dev-coldfire-xilinx_5213, whose pour places 110 vias that all clear
     every foreign hole by 0.2, keeps 0.2 (exit 0 under --strict-sizes).
  3. a floor written under the one held is a design_rules ledger row, so
     --strict-sizes exits 3 (the census patched to 0.18, in process).
  4. plane_floor_segments drops the bridges; drawn_copper_items(exclude=)
     leaves the listed segments out.

    python3 tests/test_1577_pour_hole_floor.py
"""
import contextlib
import io
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)

from copy_board import copy_board                              # noqa: E402
from run_utils import check as run_check, evidence             # noqa: E402

failures = []

#: A user's project: copper-to-hole 0.2 above the 0.16 Default class (the
#: issue's repro; neither board ships a project).
RULES = {"min_clearance": 0.16, "min_hole_clearance": 0.2,
         "min_via_annular_width": 0.1, "min_via_diameter": 0.25,
         "min_through_hole_diameter": 0.15, "min_track_width": 0.1}
CLASS = {"name": "Default", "clearance": 0.16, "track_width": 0.2,
         "via_diameter": 0.45, "via_drill": 0.2, "diff_pair_gap": 0.25,
         "diff_pair_width": 0.2, "microvia_diameter": 0.3, "microvia_drill": 0.1,
         "wire_width": 6, "bus_width": 12, "line_style": 0,
         "pcb_color": "rgba(0,0,0,0)", "schematic_color": "rgba(0,0,0,0)",
         "priority": 2147483647}


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def stage(td, name):
    src = os.path.join(td, f'{name}.kicad_pcb')
    copy_board(os.path.join(REPO, 'kicad_files', f'{name}.kicad_pcb'), src)
    evidence(src, f'the {name} copy')
    pro = os.path.splitext(src)[0] + '.kicad_pro'
    with open(pro, 'w') as fh:
        json.dump({"board": {"design_settings": {"rules": dict(RULES),
                                                 "rule_severities": {}}},
                   "meta": {"filename": os.path.basename(pro), "version": 1},
                   "net_settings": {"classes": [dict(CLASS)], "meta": {"version": 3}}},
                  fh, indent=2)
    return src


def rules(pcb):
    with open(os.path.splitext(pcb)[0] + '.kicad_pro') as fh:
        return json.load(fh)['board']['design_settings']['rules']


def pour(src, out, *extra):
    return [sys.executable, '-X', 'utf8', '-B', os.path.join('py_router', 'route_planes.py'),
            src, out, '--nets', 'GND', '--plane-layers', 'In1.Cu',
            '--zone-clearance', '0.2', '--strict-sizes', *extra]


def bridges_only(td):
    print('1. a pour that draws only soft-joint bridges is a bare pour')
    src = stage(td, 'orangecrab_ext_pll')
    out = os.path.join(td, 'oc_out.kicad_pcb')
    r = run_check(pour(src, out), accept=True)
    log = r.stdout + r.stderr
    evidence(out, 'the poured orangecrab')
    check('the finalize closed soft joints (the case under test)',
          'soft joint(s) in plane/restored copper' in log, log[-600:])
    line = next((ln for ln in log.splitlines() if 'Bare pour' in ln), '')
    check('it is a bare pour, and says the bridges do not count',
          'soft-joint bridge' in line, line or log[-600:])
    got = rules(out)
    check('min_hole_clearance kept at 0.2', got.get('min_hole_clearance') == 0.2,
          str(got.get('min_hole_clearance')))
    check('min_track_width kept at 0.1', got.get('min_track_width') == 0.1,
          str(got.get('min_track_width')))


def vias_clear(td):
    print('2. vias that clear every foreign hole by the held floor keep it')
    src = stage(td, 'kit-dev-coldfire-xilinx_5213')
    out = os.path.join(td, 'kd_out.kicad_pcb')
    r = run_check(pour(src, out), accept=True)
    log = r.stdout + r.stderr
    evidence(out, 'the poured kit-dev board')
    check('the pour placed vias (the case under test)',
          'New vias placed: 0' not in log and 'New vias placed' in log, log[-600:])
    check('the banner names the held floor',
          'Copper-to-hole floor written at 0.2 mm' in log, log[-600:])
    got = rules(out).get('min_hole_clearance')
    check('min_hole_clearance kept at 0.2 (it was 0.16)', got == 0.2, str(got))
    return src


def ledgered(td, src):
    print('3. a floor written under the one held is ledgered: --strict-sizes exits 3')
    import fab_tiers
    import fix_kicad_drc_settings as fk
    import route_planes
    out = os.path.join(td, 'kd_led.kicad_pcb')
    saved, argv = fk.step_copper_to_hole, sys.argv
    fab_tiers._LEDGER['narrowed'].clear()
    buf = io.StringIO()
    try:
        fk.step_copper_to_hole = lambda pcb, segs, vias, held, clr: 0.18
        sys.argv = pour(src, out)[4:]
        sys.argv[0] = 'route_planes.py'
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            rc = route_planes.main()
        rows = [r for r in fab_tiers._LEDGER['narrowed'] if r['kind'] == 'hole_clearance']
    finally:
        fk.step_copper_to_hole, sys.argv = saved, argv
        fab_tiers._LEDGER['narrowed'].clear()
    evidence(out, 'the ledgered pour')
    got = rules(out).get('min_hole_clearance')
    check('the project takes the census value, 0.18', got == 0.18, str(got))
    check('one hole_clearance row, 0.2 -> 0.18',
          len(rows) == 1 and (rows[0]['requested'], rows[0]['delivered']) == (0.2, 0.18),
          json.dumps(rows))
    check('--strict-sizes exits 3', rc == 3, f'rc {rc}: {buf.getvalue()[-600:]}')


def units(td):
    print('4. the bridges are not the step\'s copper')
    from fix_kicad_drc_settings import plane_floor_segments, drawn_copper_items
    from kicad_parser import Segment, parse_kicad_pcb
    from kicad_writer import add_tracks_and_vias_to_pcb
    a = {'start': (0, 0), 'end': (1, 0), 'width': 0.2, 'layer': 'F.Cu', 'net_id': 1}
    b = dict(a, soft_joint=True, from_restore=True)
    s = Segment(start_x=0, start_y=0, end_x=1, end_y=0, width=0.2, layer='F.Cu', net_id=1)
    check('plane_floor_segments keeps a track, drops a bridge',
          plane_floor_segments([a, b, s]) == [a, s])
    board = os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pcb')
    pcb = parse_kicad_pcb(board)
    gnd = next(nid for nid, n in pcb.nets.items() if n.name == 'GND')
    x0, y0, x1, y1 = pcb.board_info.board_bounds
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    t1 = {'start': (cx, cy), 'end': (cx + 1.0, cy), 'width': 0.2, 'layer': 'F.Cu',
          'net_id': gnd}
    t2 = {'start': (cx, cy + 1.0), 'end': (cx + 1.0, cy + 1.0), 'width': 0.2,
          'layer': 'F.Cu', 'net_id': gnd, 'soft_joint': True}
    drew = os.path.join(td, 'drew.kicad_pcb')
    add_tracks_and_vias_to_pcb(board, drew, tracks=[t1, t2], vias=[])
    evidence(drew, 'the board with two added tracks')
    _o, segs, _v = drawn_copper_items(board, drew)
    check('both tracks are drawn copper', len(segs) == 2, str(len(segs)))
    _o, segs, _v = drawn_copper_items(board, drew, exclude=[t2])
    check('the excluded one is not', len(segs) == 1 and abs(segs[0].start_y - cy) < 1e-6,
          str([(s.start_x, s.start_y) for s in segs]))


def main():
    td = tempfile.mkdtemp(prefix='krt1577_')
    try:
        bridges_only(td)
        src = vias_clear(td)
        ledgered(td, src)
        units(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
