#!/usr/bin/env python3
"""#1319: a pour step writes only the floors of copper it drew, and a floor
the board never declared is not called "ORIGINAL".

route_planes passed its --track-width / --via-size / --via-drill and its
clearance to the writeback whether or not it drew a track or via, so a bare
pour (0 vias, 0 tracks) wrote interf_u's min_via_drill None -> 0.6, daplink's
min_hole_clearance 0.25 -> 0.2 and multichannel_mixer's min_track_width
0.3 -> 0.2 / min_via_diameter 1.5 -> 0.6. The next route step then compared
its 0.5 via drill against that invented 0.6 and printed "FAB FLOOR RELAXED
... via drill: 0.6 -> 0.5", because the disclosure fell back to its immediate
input for a key the board's recorded origin did not carry.

`fix_kicad_drc_settings.plane_fix_kwargs` is now the one decision (the CLI
counts the copper with `drawn_copper`, the Planes tab from what it applied),
and the disclosure takes an origin's silence as "never declared".

Checks:
  1. a bare GND pour on flat_hierarchy at sizes below every declared floor
     writes no track, via, drill, hole-diameter or copper-to-hole floor and
     no min_via_drill, and still writes the copper clearance (the refill
     needs it).
  2. plane_fix_kwargs: sizes and hole floors follow the copper drawn.
  3. drawn_copper counts a via and a track added to a copy, and nothing on
     an identical copy.
  4. the disclosure: an origin without the key says nothing; an origin with
     it, and a chain with no origin yet, still disclose.

    python3 tests/test_1319_pour_writes_drawn_floors.py
"""
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from copy_board import copy_board                              # noqa: E402
from run_utils import check as run_check, evidence             # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'flat_hierarchy.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def project(board):
    with open(os.path.splitext(board)[0] + '.kicad_pro') as f:
        return json.load(f)


def bare_pour(tmp):
    print('1. a bare pour writes no size or copper-to-hole floor:')
    evidence(BOARD, 'the flat_hierarchy fixture')
    evidence(os.path.splitext(BOARD)[0] + '.kicad_pro', "the fixture's project")
    before = project(BOARD)['board']['design_settings']['rules']
    out = os.path.join(tmp, 'pour.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/route_planes.py',
                   BOARD, out, '--nets', 'GND', '--plane-layers', 'B.Cu',
                   '--clearance', '0.15', '--track-width', '0.1',
                   '--via-size', '0.3', '--via-drill', '0.2'], accept=True)
    log = r.stdout + r.stderr
    evidence(out, 'the poured board')
    doc = project(out)
    rules = doc['board']['design_settings']['rules']
    check('it says the pour drew no track or via', 'Bare pour' in log, log[-300:])
    for key in ('min_track_width', 'min_via_diameter', 'min_through_hole_diameter',
                'min_hole_clearance', 'min_hole_to_hole', 'min_via_annular_width'):
        check(f'{key} unchanged ({before.get(key)})',
              rules.get(key) == before.get(key), f"now {rules.get(key)}")
    repo = (doc.get('kicad_routing_tools') or {}).get('repo_rules') or {}
    check('no min_via_drill invented', 'min_via_drill' not in rules
          and 'min_via_drill' not in repo, f"rules {rules.get('min_via_drill')} "
          f"records {repo.get('min_via_drill')}")
    dflt = next(c for c in doc['net_settings']['classes'] if c['name'] == 'Default')
    check('the copper clearance is still written (min_clearance 0.15)',
          abs(rules.get('min_clearance', 0) - 0.15) < 1e-9, str(rules.get('min_clearance')))
    check('and the Default class clearance (0.15)',
          abs(dflt.get('clearance', 0) - 0.15) < 1e-9, str(dflt.get('clearance')))


def decision():
    print('2. plane_fix_kwargs follows the copper drawn:')
    from fix_kicad_drc_settings import plane_fix_kwargs
    kw = dict(clearance=0.15, hole_to_hole=0.2, track_width=0.1, via_size=0.3,
              via_drill=0.2)
    cases = {
        (0, 0): dict(clearance=0.15, hole_to_hole=None, track_width=None,
                     via_diameter=None, via_drill=None, copper_holes=False),
        (4, 0): dict(clearance=0.15, hole_to_hole=None, track_width=0.1,
                     via_diameter=None, via_drill=None, copper_holes=True),
        (0, 2): dict(clearance=0.15, hole_to_hole=0.2, track_width=None,
                     via_diameter=0.3, via_drill=0.2, copper_holes=True),
        (4, 2): dict(clearance=0.15, hole_to_hole=0.2, track_width=0.1,
                     via_diameter=0.3, via_drill=0.2, copper_holes=True),
    }
    for (t, v), want in cases.items():
        got = plane_fix_kwargs(t, v, **kw)
        check(f'{t} track(s), {v} via(s)', got == want, f"got {got}")
    from fix_kicad_drc_settings import compute_targets
    tg = compute_targets(clearance=0.15, copper_holes=False)
    check('compute_targets(copper_holes=False) writes no copper-to-hole floor',
          'min_hole_clearance' not in tg and tg.get('min_clearance') == 0.15, str(tg))
    tg = compute_targets(clearance=0.15)
    check('the default still follows the clearance',
          tg.get('min_hole_clearance') == 0.15, str(tg))


def counting(tmp):
    print('3. drawn_copper counts what a step added:')
    from fix_kicad_drc_settings import drawn_copper
    from kicad_parser import parse_kicad_pcb
    from kicad_writer import add_tracks_and_vias_to_pcb
    same = os.path.join(tmp, 'same.kicad_pcb')
    copy_board(BOARD, same)
    check('an identical copy drew nothing', drawn_copper(BOARD, same) == (0, 0),
          str(drawn_copper(BOARD, same)))
    pcb = parse_kicad_pcb(BOARD)
    gnd = next(nid for nid, n in pcb.nets.items() if n.name == 'GND')
    x0, y0, x1, y1 = pcb.board_info.board_bounds
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    drew = os.path.join(tmp, 'drew.kicad_pcb')
    add_tracks_and_vias_to_pcb(
        BOARD, drew,
        tracks=[{'start': (cx, cy), 'end': (cx + 1.0, cy), 'width': 0.2,
                 'layer': 'F.Cu', 'net_id': gnd}],
        vias=[{'x': cx + 1.0, 'y': cy, 'size': 0.6, 'drill': 0.3,
               'layers': ['F.Cu', 'B.Cu'], 'net_id': gnd}])
    evidence(drew, 'the board with added copper')
    check('one track and one via added', drawn_copper(BOARD, drew) == (1, 1),
          str(drawn_copper(BOARD, drew)))


def disclosure():
    print('4. the FAB FLOOR disclosure baselines on what the board declared:')
    from fix_kicad_drc_settings import _fab_floor_disclosure
    before = {'min_via_drill': 0.6, 'min_track_width': 0.2}
    proj = {'board': {'design_settings': {'rules': {'min_via_drill': 0.5,
                                                    'min_track_width': 0.2}}}}
    lines = _fab_floor_disclosure('', before, proj, {'min_track_width': 0.2},
                                  objects={})
    check('origin without the key: nothing "ORIGINAL"', lines == [], str(lines))
    lines = _fab_floor_disclosure('', before, proj, {'min_track_width': 0.2,
                                                     'min_via_drill': 0.6},
                                  objects={})
    check('origin with the key: disclosed',
          any('via drill: 0.6 -> 0.5' in l for l in lines), str(lines))
    lines = _fab_floor_disclosure('', before, proj, {}, objects={})
    check('no origin yet: the input is the baseline, as before',
          any('via drill: 0.6 -> 0.5' in l for l in lines), str(lines))


def main():
    with tempfile.TemporaryDirectory() as tmp:
        bare_pour(tmp)
        decision()
        counting(tmp)
    disclosure()
    if failures:
        print(f"\nFAILED ({len(failures)}): {failures}")
        return 1
    print('\nPASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
