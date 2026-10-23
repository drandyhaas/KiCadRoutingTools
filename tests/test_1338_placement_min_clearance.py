#!/usr/bin/env python3
"""#1338: check_assembly, place_pose and the placers work at the pad clearance
KiCad grades -- the Default class floored at Board Setup `min_clearance` --
not at the class alone.

#1210 made check_drc and board_score grade at max(class, min_clearance) and
left the placement graders on the class. multichannel_mixer declares class
0.2 and min_clearance 0.3; on its placed board kicad-cli and check_drc report
two pad pairs at 0.24 / 0.25 mm while check_assembly ("grading at clearance
0.2mm [board netclass]") and place_pose ("legality at clearance 0.2") read
clean, and `check_assembly --clearance 0.3` called its own correct grade
STRICTER than the board asks for.

The fixture is esp_prog with a project declaring class 0.2 and
min_clearance 0.8: ten pad pairs sit between the two, none below 0.2.

Checks:
  1. The rule (`fix_kicad_drc_settings.grading_clearance`) and its board
     reader (`list_nets.board_grading_clearance`): the larger of the two, a
     non-positive value unset, an explicit value first.
  2. `board_floor('clearance')` keeps the class (what a routing prediction
     works at); `board_floor_knobs`, which every placement grade and placer
     reads, is the grade.
  3. check_assembly grades at 0.8 [board minimum clearance] and finds the
     pairs; `--clearance 0.8` is not called stricter, `--clearance 0.2` is
     called looser.
  4. place_pose's verdict runs at 0.8 and reads the same conflicts.
  5. Control: min_clearance 0.1 (below the class) grades at the class, 0.2,
     and finds nothing.

    python3 tests/test_1338_placement_min_clearance.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from copy_board import copy_board                                   # noqa: E402
from fix_kicad_drc_settings import grading_clearance                # noqa: E402
import list_nets                                                    # noqa: E402
from run_utils import check as run_check, evidence                  # noqa: E402

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
POSE = os.path.join(ROOT, 'py_placer', 'place_pose.py')
ASSEMBLY = os.path.join(ROOT, 'py_tools', 'check_assembly.py')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def board(work, name, min_clearance):
    """esp_prog with a project: Default class 0.2, this min_clearance."""
    dst = os.path.join(work, name + '.kicad_pcb')
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        copy_board(ESP, dst)
    with open(os.path.join(work, name + '.kicad_pro'), 'w') as fh:
        json.dump({'board': {'design_settings': {'rules': {
            'min_clearance': min_clearance}}},
            'net_settings': {'classes': [{'name': 'Default',
                                          'clearance': 0.2}]}}, fh)
    return evidence(dst, 'fixture board')


def assembly(b, work, *extra):
    out = os.path.join(work, 'asm.json')
    if os.path.exists(out):
        os.remove(out)
    r = run_check([sys.executable, '-X', 'utf8', ASSEMBLY, b, '--json', out]
                  + list(extra), accept=True)
    with open(evidence(out, 'check_assembly --json'), encoding='utf-8') as fh:
        return r, json.load(fh)


def pose(b, work):
    r = run_check([sys.executable, '-X', 'utf8', POSE, b,
                   os.path.join(work, 'pp.kicad_pcb'), '--dry-run',
                   '--allow-routed', 'lock', 'U1'], accept=True)
    m = re.search(r'JSON_SUMMARY: (\{.*\})', r.stdout or '')
    return json.loads(m.group(1)) if m else {}


def main():
    print("1. the rule and its reader")
    check('the larger of class and minimum',
          grading_clearance(0.2, 0.3) == (0.3, 'board minimum clearance'))
    check('a minimum below the class leaves the class',
          grading_clearance(0.2, 0.1) == (0.2, 'Default net class'))
    check('a 0 minimum is unset', grading_clearance(0.2, 0) ==
          (0.2, 'Default net class'))
    check('nothing declared', grading_clearance(None, None) == (None, None))
    work = tempfile.mkdtemp(prefix='krt1338_')
    try:
        hi = board(work, 'hi', 0.8)
        lo = board(work, 'lo', 0.1)
        check('the reader: 0.8 [board minimum clearance]',
              list_nets.board_grading_clearance(hi)
              == (0.8, 'board minimum clearance'),
              str(list_nets.board_grading_clearance(hi)))
        check('the reader: an explicit value first',
              list_nets.board_grading_clearance(hi, 0.25) == (0.25, 'cli'))
        check('the reader: the class when the minimum is below it',
              list_nets.board_grading_clearance(lo) == (0.2, 'board netclass'))

        print("2. the routing floor keeps the class; the placement knobs grade")
        check("board_floor('clearance') is the class",
              list_nets.board_floor(hi, 'clearance') == (0.2, 'board netclass'))
        g = list_nets.board_floor_knobs(hi)
        check('board_floor_knobs is the grade',
              g[0] == 0.8 and g[2]['clearance']['source']
              == 'board minimum clearance', str(g[2]['clearance']))
        g = list_nets.board_floor_knobs(lo)
        check('board_floor_knobs: the class when the minimum is below it',
              g[0] == 0.2 and g[2]['clearance']['source'] == 'board netclass',
              str(g[2]['clearance']))

        print("3. check_assembly")
        r, doc = assembly(hi, work)
        check('grades at 0.8 [board minimum clearance]',
              'grading at clearance 0.8mm  [board minimum clearance]' in r.stdout
              and doc['clearance'] == 0.8
              and doc['clearance_source'] == 'board minimum clearance',
              f"{doc['clearance']} {doc['clearance_source']}")
        n_pairs = doc['pad_conflicts']
        check('...and finds the pairs the class missed', n_pairs > 0,
              str(n_pairs))
        r, doc = assembly(hi, work, '--clearance', '0.8')
        check('--clearance 0.8 is not called stricter than the board',
              'NOTE: --clearance' not in r.stdout)
        r, doc = assembly(hi, work, '--clearance', '0.2')
        check('--clearance 0.2 is called looser',
              'LOOSER' in r.stdout and doc['pad_conflicts'] == 0)

        print("4. place_pose")
        s = pose(hi, work)
        check('the verdict runs at 0.8 [board minimum clearance]',
              s.get('clearance') == 0.8
              and s['knobs']['clearance']['source'] == 'board minimum clearance',
              str(s.get('knobs', {}).get('clearance')))
        check("...and reads check_assembly's conflicts; not legal",
              s.get('pad_conflicts_after') == n_pairs
              and s.get('legal') is False,
              f"{s.get('pad_conflicts_after')} vs {n_pairs}, {s.get('legal')}")

        print("5. control: a minimum below the class")
        _r, doc = assembly(lo, work)
        check('check_assembly at 0.2 [board netclass], no pair',
              doc['clearance'] == 0.2 and doc['pad_conflicts'] == 0,
              f"{doc['clearance']} {doc['pad_conflicts']}")
        s = pose(lo, work)
        check('place_pose at 0.2, no conflict',
              s.get('clearance') == 0.2 and s.get('pad_conflicts_after') == 0)
    finally:
        shutil.rmtree(work, ignore_errors=True)

    print(f"\n{'FAILED' if failures else 'PASSED'}: {len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
