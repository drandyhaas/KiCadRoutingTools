#!/usr/bin/env python3
"""#1843: route.py routes at the clearance a rule area sets.

A `.kicad_dru` clearance rule conditioned on `A.insideArea('ROOM')` was
graded by position (check_drc) but not routed to: the router priced every
keep-out at the class value, so a gap that is legal inside the room stayed
closed, and a room with a WIDER rule was routed through at the class value.

The fixtures (tests/fixtures/1843/room_*) are synthetic: Default class
clearance 0.2 mm, track 0.15 mm, two walls of pads across the board, each
with one gap, and a rule area `ROOM` on F.Cu around the first wall's gap.
`ROOM_NET` has a pad on either side of the wall in the room, `OPEN_NET` on
either side of the wall outside it; routed on F.Cu alone, a net connects
only through its wall's gap.

  * room_relax: gaps 0.45 mm, rule `clearance (min 0.1mm)` in ROOM. A
    0.15 mm track needs 0.35 mm at 0.1 and 0.55 mm at 0.2, so only the gap
    in the room is open. kicad-cli 10.0.8 grades the routed board at 0
    violations.
  * room_tight: gaps 0.6 mm, rule `clearance (min 0.3mm)` in ROOM. The gap
    in the room needs 0.75 mm, so only the gap outside it is open. Without
    the rule read, ROOM_NET was routed through and kicad-cli reported 2
    clearance violations.

Checks:
  1. the composition: the rows kept outside the areas and the rows kept
     inside them partition every row, cells and spans alike, and a via cell
     is inside only where the area covers every copper layer;
  2. room_relax: ROOM_NET routes, OPEN_NET does not, check_drc is clean at
     KiCad's tolerance, and the rule is reported as routed to;
  3. room_tight: OPEN_NET routes, ROOM_NET does not, check_drc is clean;
  4. the map's reference counts balance (KICAD_OBSTACLE_AUDIT);
  5. a rule whose condition is more than rule-area predicates is not
     routed to, and the run says so;
  6. the rule reads as KiCad orders it: an OR of area predicates names every
     area; a class- or net-scoped clearance rule LATER in the file still
     binds its own pairs inside the room (and leaves the others at the
     room's value), and one EARLIER in the file gives way to the room;
  7. a room setup that fails says so and names no rule as routed to: the
     report line and the routing cannot disagree. (The area index gained a
     field on main once; the setup failed with a warning while the report
     still said "routed to".)
"""
import json
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace

import numpy as np

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
FIX = os.path.join(TESTS, 'fixtures', '1843')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def composition():
    print("1. the composition partitions every row")
    from shapely.geometry import box
    from area_clearance import AreaClearance, Region
    cfg = SimpleNamespace(grid_step=0.1, layers=['F.Cu', 'B.Cu'])
    area = AreaClearance([Region('r', 'ROOM', 0.1, box(1.0, 1.0, 2.0, 2.0),
                                 frozenset(['F.Cu']), 0.0)], ['F.Cu', 'B.Cu'])
    cells = np.array([[gx, gy, l] for gx in range(0, 31, 3) for gy in range(0, 31, 3)
                      for l in (0, 1)], dtype=np.int32)
    out, ins = area.keep('cells', cells, cfg, None), area.keep('cells', cells, cfg, 0)
    ok(len(out) + len(ins) == len(cells) and len(ins) > 0,
       f"cells: {len(out)} outside + {len(ins)} inside = {len(cells)}")
    ok(set(ins[:, 2].tolist()) == {0}, "inside rows are on the area's layer only")
    spans = np.array([[gx, 0, 30, l] for gx in range(0, 31) for l in (0, 1)], dtype=np.int32)

    def n_cells(s):
        return int((s[:, 2] - s[:, 1] + 1).sum()) if len(s) else 0
    so, si = area.keep('cell_spans', spans, cfg, None), area.keep('cell_spans', spans, cfg, 0)
    ok(n_cells(so) + n_cells(si) == n_cells(spans) and n_cells(si) > 0,
       f"cell spans: {n_cells(so)} outside + {n_cells(si)} inside = {n_cells(spans)}")
    vias = np.array([[gx, gy] for gx in range(0, 31, 3) for gy in range(0, 31, 3)],
                    dtype=np.int32)
    ok(len(area.keep('vias', vias, cfg, 0)) == 0,
       "a via cell is not inside an area drawn on one of two copper layers")
    both = AreaClearance([Region('r', 'ROOM', 0.1, box(1.0, 1.0, 2.0, 2.0),
                                 frozenset(['F.Cu', 'B.Cu']), 0.0)], ['F.Cu', 'B.Cu'])
    vo, vi = both.keep('vias', vias, cfg, None), both.keep('vias', vias, cfg, 0)
    ok(len(vi) > 0 and len(vo) + len(vi) == len(vias),
       f"and is inside one drawn on both: {len(vo)} outside + {len(vi)} inside")


def failed_setup():
    print("7. a room setup that fails")
    import contextlib
    import io
    import area_clearance
    from design_rules import parse_dru
    from kicad_parser import parse_kicad_pcb
    pcb = parse_kicad_pcb(os.path.join(FIX, 'room_relax.kicad_pcb'))
    with open(os.path.join(FIX, 'room_relax.kicad_dru'), encoding='utf-8') as fh:
        rules, _notes = parse_dru(fh.read())

    class Broken:
        def __init__(self, rules):
            self.rules = rules

        def area_index(self, pcb_data):
            raise RuntimeError("index changed shape")

    class Working(Broken):
        copper_layers = list(pcb.board_info.copper_layers)

        def area_index(self, pcb_data):
            from design_rules import AreaIndex
            return AreaIndex(self, pcb_data)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        area_clearance.mark_honoured(Broken(rules), pcb)
    ok(not any(r.honoured for r in rules), "no rule is named as routed to")
    ok('could not be set up' in out.getvalue() and 'routes WITHOUT them' in out.getvalue(),
       "and the run says it routes without the rooms")
    with contextlib.redirect_stdout(io.StringIO()):
        area_clearance.mark_honoured(Working(rules), pcb)
    ok(any('routed to it there' in (r.honoured or '') for r in rules),
       "a setup that works names the rule")


def stage(td, name, dru=None):
    for ext in ('.kicad_pcb', '.kicad_pro', '.kicad_dru'):
        shutil.copy(evidence(os.path.join(FIX, name + ext), 'the fixture'),
                    os.path.join(td, name + ext))
    if dru is not None:
        with open(os.path.join(td, name + '.kicad_dru'), 'w', encoding='utf-8') as fh:
            fh.write(dru)
    return os.path.join(td, name + '.kicad_pcb')


def route(td, board, tag, env=None):
    out = os.path.join(td, tag + '.kicad_pcb')
    summary = os.path.join(td, tag + '.json')
    r = check(PY + [os.path.join(REPO, 'py_router', 'route.py'), board, out,
                    '--nets', 'ROOM_NET', 'OPEN_NET', '--layers', 'F.Cu',
                    '--escalation', 'off', '--json-out', summary],
              accept=True, timeout=900, env=env)
    with open(evidence(summary, 'the summary'), encoding='utf-8') as fh:
        doc = json.load(fh)
    return (out, sorted(doc.get('routed_single') or []),
            sorted(doc.get('failed_single') or []), (r.stdout or '') + (r.stderr or ''))


def drc_clean(board):
    r = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                    evidence(board, 'the routed board'),
                    '--clearance-margin-mm', '0.0005'], accept=True)
    return 'NO DRC VIOLATIONS FOUND' in (r.stdout or '')


def main():
    composition()
    failed_setup()
    with tempfile.TemporaryDirectory() as td:
        print("2. a room that relaxes the clearance")
        out, routed, failed, log = route(td, stage(td, 'room_relax'), 'relax')
        ok(routed == ['ROOM_NET'] and failed == ['OPEN_NET'],
           f"the net in the room routes, the other does not: {routed} / {failed}")
        ok(drc_clean(out), "check_drc is clean at KiCad's tolerance")
        ok("rule 'room': clearance min 0.1 mm in rule area 'ROOM': tracks and vias "
           "are routed to it there" in log, "the run names the rule as routed to")
        ok('NOT honoured' not in log, "and no longer as not honoured")

        print("3. a room that tightens it")
        out, routed, failed, _log = route(td, stage(td, 'room_tight'), 'tight')
        ok(routed == ['OPEN_NET'] and failed == ['ROOM_NET'],
           f"the net outside the room routes, the one in it does not: {routed} / {failed}")
        ok(drc_clean(out), "check_drc is clean")

        print("4. the map's reference counts balance")
        env = dict(os.environ, KICAD_OBSTACLE_AUDIT='1')
        _out, _r, _f, log = route(td, os.path.join(td, 'room_relax.kicad_pcb'),
                                  'audit', env=env)
        ok('[OBSTACLE AUDIT]' in log and 'BALANCED' in log,
           "the working map returns exactly to the base map")

        print("5. a condition with more than the area predicate")
        sub = os.path.join(td, 'more')
        os.mkdir(sub)
        board = stage(sub, 'room_relax', dru=(
            '(version 1)\n(rule "room"\n  (constraint clearance (min 0.1mm))\n'
            "  (condition \"A.insideArea('ROOM') && A.Type == 'track'\"))\n"))
        _out, routed, failed, log = route(sub, board, 'more')
        ok(routed == [] and failed == ['OPEN_NET', 'ROOM_NET'],
           f"is not routed to: {routed} / {failed}")
        ok("rule 'room': its area clearance is NOT routed to" in log,
           "and the run says so")
        print("6. rule order, and an OR of areas")
        room = ('(rule "room"\n  (constraint clearance (min 0.1mm))\n'
                "  (condition \"A.insideArea('ROOM') || A.intersectsArea('ROOM')\"))\n")

        def scoped(net):
            return (f'(rule "wide"\n  (constraint clearance (min 0.3mm))\n'
                    f"  (condition \"A.NetName == '{net}'\"))\n")
        for tag, dru, want, why in (
                ('or', room, ['ROOM_NET'], "an OR of two predicates is routed to"),
                ('later_other', room + scoped('OPEN_NET'), ['ROOM_NET'],
                 "a later rule on another net leaves the room open"),
                ('later_own', room + scoped('ROOM_NET'), [],
                 "a later rule on the net itself closes the gap again"),
                ('earlier_own', scoped('ROOM_NET') + room, ['ROOM_NET'],
                 "an earlier rule on the net gives way to the room")):
            sub = os.path.join(td, tag)
            os.mkdir(sub)
            board = stage(sub, 'room_relax', dru='(version 1)\n' + dru)
            out, routed, _failed, _log = route(sub, board, tag)
            ok(routed == want, f"{why}: routed {routed}")
            ok(drc_clean(out), "  and check_drc is clean")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
