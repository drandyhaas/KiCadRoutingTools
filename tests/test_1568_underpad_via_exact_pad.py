#!/usr/bin/env python3
"""#1568: the QFN under-pad escape tests a candidate via against a pad with
check_drc's own pad geometry, at the pair clearance with no grading margin.

`via_clears` used to grow the pad's rectangle by via/2 + clearance with SQUARE
corners, which beside a pad corner overstates the keep-out by up to
margin * (sqrt(2) - 1) and ignores a roundrect's radius. On the 0.4 mm-pitch
QFN fixture with its DATA nets in a 0.2 mm class over the 0.1 Default, 16 of
47 pads were dropped as `no clear via offset` although each has a via offset
check_drc and KiCad grade clean. The same test ignored a pad clearance
override, so a footprint override above the pair clearance shipped a via
check_drc flags (1 PAD-VIA row).

Checks (each a fanout run, graded by check_drc.py at --clearance-margin 0):
  1. DATA nets in a 0.2 mm class: every pad escapes (was 31 of 47) and no
     clearance row.
  2. A 0.2 mm footprint clearance override on the QFN over a 0.1 Default:
     every pad escapes and no clearance row (was 1 PAD-VIA).

    python3 tests/test_1568_underpad_via_exact_pad.py
"""
import contextlib
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

QFN = os.path.join(ROOT, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
ARGS = ['--component', 'U2', '--nets', '*', '!GND', '--escape-method', 'underpad',
        '--allow-via-in-pad', '--width', '0.1', '--clearance', '0.1',
        '--via-size', '0.45', '--via-drill', '0.25', '--no-fix-drc-settings']
ROWS = ('SEGMENT-SEGMENT', 'VIA-SEGMENT', 'PAD-SEGMENT', 'PAD-VIA', 'VIA-VIA')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def stage(dst, class_mm=None, fp_override=None):
    """Copy the fixture with a project: Default 0.1, plus a class IO at
    `class_mm` holding the DATA nets; with `fp_override`, the QFN footprint
    carries that `(clearance ...)`."""
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(QFN, dst)
    default = {'name': 'Default', 'clearance': 0.1, 'track_width': 0.2,
               'via_diameter': 0.6, 'via_drill': 0.3, 'priority': 2147483647}
    ns = {'meta': {'version': 4}, 'classes': [default]}
    if class_mm is not None:
        ns['classes'].append(dict(default, name='IO', clearance=class_mm, priority=0))
        ns['netclass_patterns'] = [{'netclass': 'IO', 'pattern': 'Net-(U2A-DATA_*'}]
    proj = {'board': {'design_settings': {'rules': {
                'min_clearance': 0.0, 'min_track_width': 0.1,
                'min_via_diameter': 0.4, 'min_through_hole_diameter': 0.2}}},
            'net_settings': ns}
    with open(os.path.splitext(dst)[0] + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump(proj, f, indent=2)
    if fp_override is not None:
        with open(dst, encoding='utf-8', newline='') as f:
            text = f.read()
        ref = text.index('(property "Reference" "U2"')
        tag = '(tags "QFN NoLead")'
        at = text.rindex(tag, 0, ref) + len(tag)
        text = text[:at] + f'\n\t\t(clearance {fp_override})' + text[at:]
        with open(dst, 'w', encoding='utf-8', newline='') as f:
            f.write(text)
    return evidence(dst)


def fan(inp, out):
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/qfn_fanout.py', inp,
                   '--output', out] + ARGS, accept=True)
    evidence(out, 'qfn_fanout.py output')
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', r.stdout, re.M)
    if not m:
        raise AssertionError(f"BROKEN TEST: no JSON_SUMMARY\n{r.stdout[-1500:]}")
    return json.loads(m.group(1))


def drc(board):
    """check_drc.py's clearance rows at margin 0 (it reads the classes and the
    pad overrides): {row kind: count}."""
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py', board,
                        '--no-size-checks', '--clearance-margin', '0', '--max-print', '0'],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=600)
    out = r.stdout + r.stderr
    if 'Traceback' in out or not ('NO DRC VIOLATIONS' in out or 'DRC VIOLATION' in out):
        raise AssertionError(f"BROKEN TEST: no verdict from check_drc.py\n{out[-1500:]}")
    return {k: int(m.group(1)) for k in ROWS
            for m in [re.search(rf'^{k} violations \((\d+)\)', out, re.M)] if m}


def main():
    if not os.path.isfile(QFN):
        print(f"SKIP: {os.path.relpath(QFN, ROOT)} missing")
        return 77
    tmp = tempfile.mkdtemp(prefix='t1568_')
    try:
        print("1. DATA nets in a 0.2 mm class over the 0.1 Default")
        inp = stage(os.path.join(tmp, 'cls.kicad_pcb'), class_mm=0.2)
        out = os.path.join(tmp, 'cls_fan.kicad_pcb')
        js = fan(inp, out)
        check("every pad escaped (was 31 of 47)",
              js.get('escaped') == 47 and js.get('failed') == 0,
              f"escaped={js.get('escaped')} failed={js.get('failed')}")
        rows = drc(out)
        check("check_drc at margin 0 finds no clearance row", not rows, f"{rows}")

        print("2. A 0.2 mm footprint clearance override over the 0.1 Default")
        inp = stage(os.path.join(tmp, 'ovr.kicad_pcb'), fp_override=0.2)
        out = os.path.join(tmp, 'ovr_fan.kicad_pcb')
        js = fan(inp, out)
        check("every pad escaped", js.get('escaped') == 47 and js.get('failed') == 0,
              f"escaped={js.get('escaped')} failed={js.get('failed')}")
        rows = drc(out)
        check("check_drc at margin 0 finds no clearance row (was 1 PAD-VIA)",
              not rows, f"{rows}")
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
