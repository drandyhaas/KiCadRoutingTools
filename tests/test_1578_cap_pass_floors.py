#!/usr/bin/env python3
"""#1578: place_fanout_clearance writes the size floors of the copper it laid
or moved, and nothing for copper it did not touch; and it takes
--no-fix-drc-settings like every routing CLI.

A run that moved nothing ("No caps moved; passing the board through
unchanged", the board byte-identical) still wrote the project's via floors
down to the vias the input already carried -- routed_output with a project
declaring min_via_annular_width 0.13 / min_via_diameter 0.35 shipped 0.05 /
0.3, and KiCad's 199 annular_width / 199 via_diameter errors on that copper
read 0 -- because fix_project_for_output scanned the whole board's minima,
whoever laid them. Now:

  1. a zero-move run keeps every size floor and still does #769's custody
     (the project carried, fab_floor_origin recorded) and its ceiling.
  2. a run that nudged a via (#313) writes THAT via's floors (0.8 / ring
     0.25 / drill 0.3), not those of the via it left alone (0.6 / 0.15).
  3. a run that moved caps and no via writes no size floor.
  4. --no-fix-drc-settings writes nothing -- not the ceiling either -- and
     still carries the input's project through, on both exits.
  5. the flag converts to the GUI plan as the shared box (fix_drc_check
     False), and an omitted flag is not materialised; the Fanout tab's
     standalone cap step gates its class clamp on the same box.

    python3 tests/test_1578_cap_pass_floors.py
"""
import ast
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(TESTS, 'stress'))

from copy_board import copy_board                              # noqa: E402
from run_utils import check as run_check, evidence             # noqa: E402
from test_741_via_nudge_tenting import RIG_BOARD, CLEAR        # noqa: E402

CLI = os.path.join(REPO, 'py_placer', 'place_fanout_clearance.py')
SIZE_KEYS = ('min_via_diameter', 'min_via_annular_width', 'min_via_drill',
             'min_through_hole_diameter', 'min_track_width')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def project(pcb):
    with open(os.path.splitext(pcb)[0] + '.kicad_pro', encoding='utf-8') as fh:
        return json.load(fh)


def rules(pcb):
    return project(pcb)['board']['design_settings']['rules']


def classes(pcb):
    return {c['name']: c.get('clearance')
            for c in project(pcb)['net_settings']['classes']}


def cap_pass(src, dst, *extra):
    r = run_check([sys.executable, '-X', 'utf8', '-B', CLI, src, dst,
                   *[str(a) for a in extra]], accept=True)
    evidence(dst, 'the cap pass output')
    return r.stdout + r.stderr


def stage_routed(td, name):
    """routed_output with the issue's floors: its 0.3/0.2 vias sit below both."""
    src = os.path.join(td, name + '.kicad_pcb')
    copy_board(os.path.join(REPO, 'kicad_files', 'routed_output.kicad_pcb'), src)
    evidence(src, 'the routed_output copy')
    pro = os.path.splitext(src)[0] + '.kicad_pro'
    doc = json.load(open(pro, encoding='utf-8'))
    doc['board']['design_settings']['rules'].update(
        min_via_annular_width=0.13, min_via_diameter=0.35)
    doc['net_settings']['classes'].append(
        {'name': 'Wide', 'clearance': 0.4, 'track_width': 0.4,
         'via_diameter': 0.8, 'via_drill': 0.4, 'priority': 1})
    json.dump(doc, open(pro, 'w', encoding='utf-8'), indent=2)
    return src


def stage_rig(td, name):
    """test_741's rig (C1/C2 grazing the VF1/VF2 fanout vias), its VF3 control
    via shrunk to 0.6/0.3 so the board's smallest via is one the pass never
    touches, with a project declaring floors above every via on it."""
    src = os.path.join(td, name + '.kicad_pcb')
    i = RIG_BOARD.rfind('(size 0.8)')               # the last via: VF3
    text = RIG_BOARD[:i] + '(size 0.6)' + RIG_BOARD[i + len('(size 0.8)'):]
    with open(src, 'w', encoding='utf-8', newline='') as fh:
        fh.write(text)
    evidence(src, 'the staged rig')
    json.dump({"board": {"design_settings": {"rules": {
                  "min_clearance": 0.1, "min_via_diameter": 1.0,
                  "min_via_annular_width": 0.3, "min_through_hole_diameter": 0.35,
                  "min_track_width": 0.3}, "rule_severities": {}}},
               "meta": {"filename": name + ".kicad_pro", "version": 1},
               "net_settings": {"classes": [{"name": "Default", "clearance": 0.1,
                                             "track_width": 0.3, "via_diameter": 1.0,
                                             "via_drill": 0.4}],
                                "meta": {"version": 3}}},
              open(os.path.splitext(src)[0] + '.kicad_pro', 'w'), indent=2)
    return src


NUDGE = ('--clearance', CLEAR, '--max-displacement', 0, '--max-displacement-cap', 0,
         '--max-passes', 1, '--no-rotate')


def zero_move(td):
    print('1. a zero-move run writes no size floor, and keeps #769')
    src = stage_routed(td, 'z')
    before = rules(src)
    dst = os.path.join(td, 'z_out.kicad_pcb')
    out = cap_pass(src, dst, '--clearance', 0.3)
    check('the run moved nothing (the case under test)', 'unchanged copy' in out,
          out[-600:])
    got = rules(dst)
    for k in SIZE_KEYS:
        check(f'{k} kept ({before.get(k)})', got.get(k) == before.get(k),
              f'now {got.get(k)}')
    check('the ceiling still caps the classes (#769): Wide 0.4 -> 0.3',
          abs((classes(dst).get('Wide') or 0) - 0.3) < 1e-9, str(classes(dst)))
    check('fab_floor_origin recorded (custody)',
          'fab_floor_origin' in (project(dst).get('kicad_routing_tools') or {}))


def nudged(td):
    print("2. a nudged via writes that via's floors")
    src = stage_rig(td, 'n')
    dst = os.path.join(td, 'n_out.kicad_pcb')
    out = cap_pass(src, dst, *NUDGE)
    check('the pass nudged a via (the case under test)',
          'via-nudge: moved VF1 via' in out, out[-600:])
    got = rules(dst)
    for k, want in (('min_via_diameter', 0.8), ('min_via_annular_width', 0.25),
                    ('min_through_hole_diameter', 0.3), ('min_via_drill', 0.3)):
        check(f'{k} -> {want} (the nudged via), not the untouched 0.6 via',
              abs((got.get(k) or 0) - want) < 1e-9, f'now {got.get(k)}')
    check('min_track_width kept 0.3 (no connector was drawn)',
          got.get('min_track_width') == 0.3, f"now {got.get('min_track_width')}")


def caps_only(td):
    print('3. a run that moved caps and no via writes no size floor')
    src = stage_rig(td, 'c')
    before = rules(src)
    dst = os.path.join(td, 'c_out.kicad_pcb')
    out = cap_pass(src, dst, '--clearance', CLEAR)
    check('caps moved, no via (the case under test)',
          'Moved 2 cap(s)' in out and 'via-nudge: moved' not in out, out[-600:])
    got = rules(dst)
    for k in SIZE_KEYS:
        check(f'{k} kept ({before.get(k)})', got.get(k) == before.get(k),
              f'now {got.get(k)}')


def no_fix(td):
    print('4. --no-fix-drc-settings writes nothing and carries the project')
    src = stage_routed(td, 'f')
    dst = os.path.join(td, 'f_out.kicad_pcb')
    out = cap_pass(src, dst, '--clearance', 0.3, '--no-fix-drc-settings')
    check('it says so', 'not written (--no-fix-drc-settings)' in out, out[-600:])
    check('the zero-move project is the input\'s, rules and classes',
          rules(dst) == rules(src) and classes(dst) == classes(src),
          f'{rules(dst)} {classes(dst)}')
    src = stage_rig(td, 'fn')
    dst = os.path.join(td, 'fn_out.kicad_pcb')
    out = cap_pass(src, dst, *NUDGE, '--no-fix-drc-settings')
    check('the nudge path wrote the board', 'via-nudge: moved VF1 via' in out,
          out[-600:])
    evidence(os.path.splitext(dst)[0] + '.kicad_pro', "the nudge output's project")
    check('and carried the input project through unchanged', rules(dst) == rules(src),
          f'{rules(dst)}')


def converts():
    print('5. the flag reaches the GUI plan, and the tab gates on the same box')
    import manifest_to_plan as m2p
    step = m2p.cap_optimization_step(
        ['python3', 'py_placer/place_fanout_clearance.py', 'b.kicad_pcb',
         '--no-fix-drc-settings'])
    check('--no-fix-drc-settings -> fix_drc_check False',
          (step.get('params') or {}).get('fix_drc_check') is False, str(step))
    bare = m2p.cap_optimization_step(
        ['python3', 'py_placer/place_fanout_clearance.py', 'b.kicad_pcb'])
    check('an omitted flag is not materialised',
          'fix_drc_check' not in (bare.get('params') or {}), str(bare))
    src = open(os.path.join(REPO, 'kicad_routing_plugin', 'fanout_gui.py'),
               encoding='utf-8').read()
    fn = next(n for n in ast.walk(ast.parse(src))
              if isinstance(n, ast.FunctionDef) and n.name == 'run_cap_optimization')
    gate = [n for n in ast.walk(fn) if isinstance(n, ast.If)
            and 'clamp_nondefault_netclasses_on_board' in ast.get_source_segment(src, n)]
    check("the standalone cap step's class clamp is gated on fix_drc_settings",
          bool(gate) and "fix_drc_settings" in ast.get_source_segment(src, gate[-1].test),
          ast.get_source_segment(src, gate[-1].test) if gate else 'no gate')


def main():
    td = tempfile.mkdtemp(prefix='krt1578_')
    try:
        zero_move(td)
        nudged(td)
        caps_only(td)
        no_fix(td)
        converts()
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
