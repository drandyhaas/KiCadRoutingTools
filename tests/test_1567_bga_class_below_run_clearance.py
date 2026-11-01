#!/usr/bin/env python3
"""#1567: bga_fanout grades a net class BELOW the run clearance wherever its
fab-floor rescue laid copper below that class.

The #1529 class check ran only when some class stood above --clearance, and
graded only those classes: copper laid at the run clearance meets every class
below it. The fab-floor rescue (`_underpad_shrink_rescue`) lays copper far
below the run clearance, so a ball of a class between the two shipped under
its class. On a 0.8 mm BGA whose IO nets sit in a 0.22 class (Default 0.1)
escaped at --clearance 0.25, the step reported every ball escaped and left 5
via-segment and 1 pad-segment rows check_drc grades only at the class (KiCad
flagged them too). A class at 0.30, above the run clearance, was checked: the
balls that could not meet it were reported unescaped.

The rescue now marks the copper it lays below the run clearance with the
clearance it was laid at, and the class check grades such copper at any class
above that, re-escaping a short ball at its class (its rescue stopped there)
and reporting the ones that cannot.

Checks (bga_fanout.py runs on the 0.8 mm BGA, graded by check_drc.py):
  1. class 0.22, --clearance 0.25, --no-fix-drc-settings: the step runs the
     class check, the output's grade at its classes has no row the class adds
     (the same rows as the grade with the class taken out; was 6), the short
     balls' re-escape stops its rescue at the class (0.22, not the 0.09 fab
     floor), and the balls that still fall short are reported unescaped, not
     shipped.
  2. ...and only the classed side of a short pair is taken off: the Default
     nets whose copper the short IO copper grazed keep their escape, and on
     one such pair `_drop_refs` names the IO side alone.

    python3 tests/test_1567_bga_class_below_run_clearance.py
"""
import contextlib
import copy
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from copy_board import copy_board                                  # noqa: E402
from run_utils import check as run_check, evidence                 # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
IO = ['/IO_Banks/D*', '/IO_Banks/Q*']
ARGS = ['--component', 'U30', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
        '--nets', '*', '!GND', '!+3V3', '!+1V2', '--track-width', '0.12',
        '--via-size', '0.3', '--via-drill', '0.15', '--clearance', '0.25',
        '--no-fix-drc-settings']
# the Default nets the rescued IO copper grazed on the old code
GRAZED = ('/VCCPLL1', '/IO_Banks/U4')
CLEARANCE_ROWS = ('SEGMENT-SEGMENT', 'VIA-SEGMENT', 'PAD-SEGMENT', 'PAD-VIA', 'VIA-VIA')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def pro_path(board):
    return os.path.splitext(board)[0] + '.kicad_pro'


def set_classes(board, default_mm, io_mm):
    """Default at `default_mm`, plus class IO at `io_mm` on the IO nets (none
    when `io_mm` is None)."""
    with open(pro_path(board), encoding='utf-8') as f:
        proj = json.load(f)
    ns = proj['net_settings']
    base = next(c for c in ns['classes'] if c['name'] == 'Default')
    base['clearance'] = default_mm
    ns['classes'], ns['netclass_patterns'] = [base], []
    if io_mm is not None:
        c = copy.deepcopy(base)
        c.update(name='IO', clearance=io_mm, priority=0)
        ns['classes'].append(c)
        ns['netclass_patterns'] = [{'netclass': 'IO', 'pattern': p} for p in IO]
    with open(pro_path(board), 'w', encoding='utf-8') as f:
        json.dump(proj, f, indent=2)


def stage(src, dst, io_mm):
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(src, dst)
    set_classes(dst, 0.1, io_mm)
    return dst


def drc(board):
    """check_drc.py's grade (it reads the board's net classes): the count of
    each clearance row kind."""
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py', board,
                        '--no-size-checks', '--max-print', '0'],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=600)
    out = r.stdout + r.stderr
    if 'Traceback' in out or not ('NO DRC VIOLATIONS' in out or 'DRC VIOLATION' in out):
        raise AssertionError(f"BROKEN TEST: no verdict from check_drc.py\n{out[-1500:]}")
    return {k: int(m.group(1)) if m else 0 for k in CLEARANCE_ROWS
            for m in [re.search(rf'^{k} violations \((\d+)\)', out, re.M)]}


def main():
    if not os.path.isfile(BOARD):
        print(f"SKIP: {os.path.relpath(BOARD, ROOT)} missing")
        return 77
    tmp = tempfile.mkdtemp(prefix='t1567_')
    try:
        src = evidence(stage(BOARD, os.path.join(tmp, 'in.kicad_pcb'), 0.22))
        out = os.path.join(tmp, 'fan.kicad_pcb')
        r = run_check([sys.executable, '-X', 'utf8', 'py_router/bga_fanout.py', src,
                       '--output', out] + ARGS, accept=True)
        evidence(out, 'bga_fanout output')
        m = re.search(r'^JSON_SUMMARY: (\{.*\})$', r.stdout, re.M)
        if not m:
            raise AssertionError(f"BROKEN TEST: no JSON_SUMMARY\n{r.stdout[-1500:]}")
        js = json.loads(m.group(1))
        if 'Under-pad shrink rescue' not in r.stdout:
            raise AssertionError("BROKEN TEST: the fab-floor rescue did not run, "
                                 "so nothing below the run clearance was laid")

        print("1. a class below the run clearance is graded where the rescue undercut it")
        check("the step ran the class check on the rescued copper",
              bool(re.search(r'Net classes \(#1529\): \d+ pair\(s\) of this escape fell '
                             r'short.*a fab-floor rescue laid copper at', r.stdout)))
        bare = stage(out, os.path.join(tmp, 'noclass.kicad_pcb'), None)
        a, b = drc(out), drc(bare)
        added = sum(a.values()) - sum(b.values())
        check("the class adds no row to the grade (was 6)", added == 0,
              f"{added} class row(s); with the class {a}, without {b}")
        check("the short balls' re-escape stops its rescue at their class, not the fab floor",
              bool(re.search(r'Under-pad shrink rescue @ .*clearance 0\.2200mm', r.stdout)))
        unesc = js.get('unescaped_nets') or []
        check("the balls that cannot meet the class are reported unescaped",
              js.get('failed', 0) > 0 and all(n.startswith('/IO_Banks/') for n in unesc),
              f"failed={js.get('failed')} unescaped={unesc}")

        print("2. only the classed side of a short pair comes off")
        check("the Default nets the IO copper grazed keep their escape",
              not set(GRAZED) & set(unesc), f"unescaped={unesc}")
        # the rule itself, on one pair: an IO track (class 0.22) laid below
        # it against a Default via, both this call's copper, run clearance
        # 0.25. Measured from the run clearance both sides stand at 0.25 >=
        # 0.22, which would take both escapes off.
        from bga_fanout.class_clearance import _drop_refs
        net_of = {('t', 0): 'IO', ('v', 0): 'Default'}
        drop = _drop_refs([(('t', 0), ('v', 0), 0.22, True)],
                          lambda r: 0.25, lambda r: {'IO': 0.22}.get(net_of[r], 0.0))
        check("a pair short of a class below the run clearance takes off only its classed side",
              drop == {('t', 0)}, f"{sorted(drop)}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print()
    if failures:
        print(f"FAILED: {len(failures)} check(s): " + '; '.join(failures))
        return 1
    print("PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
