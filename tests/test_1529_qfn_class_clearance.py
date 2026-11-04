#!/usr/bin/env python3
"""#1529: qfn_fanout spaces its stubs at KiCad's pairwise max(classA, classB),
grades its own `drc_grazes` at the written project's classes, and writes the
classes back as route.py does -- kept, or capped at --clearance-ceiling.

It used to price every foreign obstacle at the one clearance it escaped at
(the Default class), so on a 0.4 mm-pitch QFN whose nets sit in 0.2 mm classes
over a 0.1 Default it reported `failed: 0` and `drc_grazes 0` while check_drc
found 21 segment-segment violations at 0.2 (KiCad 26). The default writeback
then lowered EVERY non-Default class to 0.1 -- including a class whose nets
have no pad on the part -- which hid that copper from every later grade.

Checks (each a qfn_fanout.py run, graded by check_drc.py):
  1. classes 0.2 over Default 0.1, --no-fix-drc-settings: every pad still
     escapes and the output grades CLEAN at the classes (was 21 violations).
  2. the same with the default writeback: the classes are KEPT at 0.2 (were
     lowered to 0.1), and the output still grades clean.
  3. --clearance-ceiling 0.1: each class is written at min(class, ceiling) =
     0.1, as route.py's ceiling writes it, and the output grades clean.
  4. --clearance-ceiling 0.1 --no-fix-drc-settings: the copper is spaced at the
     ceiling while the project keeps its 0.2 classes, and the step's own
     drc_grazes counts the same segment-segment rows check_drc does (> 0) --
     the self-audit is no longer class-blind (it reported 0).
  5. a 0.4 class on nets with no pad on the fanned part: kept at 0.4 by the
     default writeback (was lowered to the 0.2 escape clearance).

    python3 tests/test_1529_qfn_class_clearance.py
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

FINE = os.path.join(ROOT, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
SPARSE = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def pro_path(board):
    return os.path.splitext(board)[0] + '.kicad_pro'


def stage(src, dst, default_mm, specs):
    """Copy `src` with its siblings and give the copy's project a Default class
    of `default_mm` plus one class per (name, mm, [patterns]) in `specs`."""
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(src, dst)
    if os.path.isfile(pro_path(dst)):
        with open(pro_path(dst), encoding='utf-8') as f:
            proj = json.load(f)
    else:                       # a project-less fixture: a minimal project
        proj = {'board': {'design_settings': {'rules': {
                    'min_clearance': 0.0, 'min_track_width': 0.1}}},
                'net_settings': {'meta': {'version': 4}, 'classes': [{
                    'name': 'Default', 'clearance': default_mm, 'track_width': 0.2,
                    'via_diameter': 0.6, 'via_drill': 0.3, 'diff_pair_gap': 0.25,
                    'diff_pair_width': 0.2, 'diff_pair_via_gap': 0.25,
                    'microvia_diameter': 0.3, 'microvia_drill': 0.1,
                    'priority': 2147483647}]},
                'meta': {'filename': os.path.basename(pro_path(dst)), 'version': 3}}
    ns = proj['net_settings']
    base = next(c for c in ns['classes'] if c['name'] == 'Default')
    base['clearance'] = default_mm
    ns['classes'], ns['netclass_patterns'] = [base], []
    for i, (name, mm, pats) in enumerate(specs):
        c = copy.deepcopy(base)
        c.update(name=name, clearance=mm, priority=i)
        ns['classes'].append(c)
        ns['netclass_patterns'] += [{'netclass': name, 'pattern': p} for p in pats]
    with open(pro_path(dst), 'w', encoding='utf-8') as f:
        json.dump(proj, f, indent=2)
    return dst


def classes(board):
    with open(pro_path(board), encoding='utf-8') as f:
        return {c['name']: c['clearance']
                for c in json.load(f)['net_settings']['classes']}


def fan(inp, out, component, *extra):
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/qfn_fanout.py', inp,
                   '--output', out, '--component', component] + list(extra),
                  accept=True)
    evidence(out, 'qfn_fanout output')
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', r.stdout, re.M)
    return json.loads(m.group(1)) if m else {}


def drc(board):
    """check_drc.py's own grade (it reads the board's net classes): the
    segment-segment count and the total."""
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py', board,
                        '--no-size-checks', '--max-print', '0'],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=600)
    out = r.stdout + r.stderr
    if 'Traceback' in out:
        raise AssertionError(f"BROKEN TEST: check_drc.py crashed\n{out[-1500:]}")
    total = 0 if 'NO DRC VIOLATIONS' in out else None
    m = re.search(r'FOUND (\d+) DRC VIOLATION', out)
    if m:
        total = int(m.group(1))
    ss = re.search(r'SEGMENT-SEGMENT violations \((\d+)\)', out)
    if total is None:
        raise AssertionError(f"BROKEN TEST: no verdict from check_drc.py\n{out[-1500:]}")
    return (int(ss.group(1)) if ss else 0), total


def main():
    for b in (FINE, SPARSE):
        if not os.path.isfile(b):
            print(f"SKIP: {os.path.relpath(b, ROOT)} missing")
            return 77
    tmp = tempfile.mkdtemp(prefix='t1529_')
    try:
        fine = stage(FINE, os.path.join(tmp, 'in.kicad_pcb'), 0.1,
                     [('DATA', 0.2, ['Net-(U2A-DATA_*']),
                      ('CTRL', 0.2, ['Net-(U2A-~*']),
                      ('GPIO', 0.2, ['Net-(U2A-GPIO*'])])
        evidence(fine)
        check("the staged input grades clean at its classes", drc(fine)[1] == 0,
              f"{drc(fine)}")
        args = ['--nets', '*', '!GND', '--width', '0.15', '--fab-tier', 'standard',
                '--escalation', 'board']

        print("1. classes honoured, no writeback")
        out = os.path.join(tmp, 'fan.kicad_pcb')
        js = fan(fine, out, 'U2', *args, '--no-fix-drc-settings')
        check("every pad escaped", js.get('failed') == 0 and js.get('escaped') == 47,
              f"escaped={js.get('escaped')} failed={js.get('failed')}")
        ss, total = drc(out)
        check("check_drc finds no violation at the classes (was 21 segment-segment)",
              total == 0, f"segment-segment {ss}, total {total}")
        check("...and drc_grazes agrees", (js.get('drc_grazes') or {}).get('total') == 0,
              f"{js.get('drc_grazes')}")

        print("2. the default writeback keeps the classes")
        out = os.path.join(tmp, 'fan_fix.kicad_pcb')
        fan(fine, out, 'U2', *args)
        cl = classes(out)
        check("DATA / CTRL / GPIO are still 0.2 (were lowered to 0.1)",
              all(abs(cl.get(n, 0) - 0.2) < 1e-9 for n in ('DATA', 'CTRL', 'GPIO')),
              f"{cl}")
        check("...and the output grades clean at them", drc(out)[1] == 0, f"{drc(out)}")

        print("3. --clearance-ceiling 0.1 writes min(class, ceiling)")
        out = os.path.join(tmp, 'fan_ceil.kicad_pcb')
        js = fan(fine, out, 'U2', *args, '--clearance-ceiling', '0.1')
        cl = classes(out)
        check("every class at 0.1", all(abs(v - 0.1) < 1e-9 for v in cl.values()), f"{cl}")
        check("every pad escaped", js.get('failed') == 0, f"failed={js.get('failed')}")
        check("...and the output grades clean", drc(out)[1] == 0, f"{drc(out)}")

        print("4. drc_grazes reads the written project's classes")
        out = os.path.join(tmp, 'fan_ceil_nofix.kicad_pcb')
        js = fan(fine, out, 'U2', *args, '--clearance-ceiling', '0.1',
                 '--no-fix-drc-settings')
        ss, total = drc(out)
        g = js.get('drc_grazes') or {}
        check("copper spaced at the ceiling violates the kept 0.2 classes",
              ss > 0, f"check_drc segment-segment {ss}")
        check("...and the step's own drc_grazes counts the same rows (was 0)",
              g.get('segment_segment') == ss and g.get('total') == total,
              f"drc_grazes {g} vs check_drc segment-segment {ss}, total {total}")

        print("5. a class the part never touches is not lowered")
        sparse = stage(SPARSE, os.path.join(tmp, 'sp.kicad_pcb'), 0.2,
                       [('Sense', 0.4, ['Net-(U12-*', 'Net-(U21-*'])])
        out = os.path.join(tmp, 'sp_fan.kicad_pcb')
        js = fan(sparse, out, 'U1', '--nets', '*', '!GND', '!+3V3')
        check("the fanout laid copper", (js.get('escaped') or 0) > 0, f"{js.get('escaped')}")
        cl = classes(out)
        check("Sense is still 0.4 (was lowered to 0.2)",
              abs(cl.get('Sense', 0) - 0.4) < 1e-9, f"{cl}")
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
