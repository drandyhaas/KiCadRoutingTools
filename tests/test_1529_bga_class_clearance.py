#!/usr/bin/env python3
"""#1529 for bga_fanout: the escape is spaced at KiCad's pairwise
max(clearance, class A, class B), `drc_grazes` is graded at the written
project's classes, and the classes are written back as route.py writes them --
kept, or capped at --clearance-ceiling.

It used to escape every ball at the one run clearance, grade its own
`drc_grazes` without the classes, and then lower EVERY non-Default class to
the run clearance -- including a class none of whose nets the part touches --
which hid the copper laid below a class from every later grade. On a 0.8 mm
BGA whose data nets sit in a 0.16 class over a 0.1 Default, the escape left 159
rows check_drc grades only at the class (KiCad grades them too), with
`drc_grazes` blind to all of them.

Checks (bga_fanout.py runs on a 0.8 mm-pitch BGA, graded by check_drc.py):
  1. class 0.16 over Default 0.1, --no-fix-drc-settings: every ball still
     escapes, and the output's grade at its classes finds no row the classes
     add -- the same rows as the grade with the classes taken out (was 174 vs
     15: 159 class rows).
  2. the default writeback KEEPS the 0.16 class (was lowered to 0.1), and a
     0.4 class on nets with no pad on the part stays 0.4 (was lowered too).
  3. --clearance-ceiling 0.1: each class is written at min(class, ceiling) =
     0.1, as route.py's ceiling writes it, every ball escapes.
  4. --clearance-ceiling 0.1 --no-fix-drc-settings: the copper is spaced at
     the ceiling while the project keeps its 0.16 class, and the step's own
     drc_grazes counts the class rows check_drc finds (it counted none).
  5. the #1518 gap test at the pair clearance a ball's own class sets: a
     class whose track cannot fit the gap between the balls keeps ITS inner
     balls off the top layer, and only those.

    python3 tests/test_1529_bga_class_clearance.py
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
REF = 'U30'
IO = ['/IO_Banks/D*', '/IO_Banks/Q*']
FAR = ['Net-(U12-*', 'Net-(U21-*']          # nets with no pad on U30
ARGS = ['--component', REF, '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
        '--nets', '*', '!GND', '!+3V3', '!+1V2', '--track-width', '0.12',
        '--via-size', '0.3', '--via-drill', '0.15']
CLEARANCE_ROWS = ('SEGMENT-SEGMENT', 'VIA-SEGMENT', 'PAD-SEGMENT', 'PAD-VIA')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def pro_path(board):
    return os.path.splitext(board)[0] + '.kicad_pro'


def set_classes(board, default_mm, specs):
    """Give `board`'s project a Default class of `default_mm` plus one class
    per (name, mm, [patterns]) in `specs` (none: Default only)."""
    with open(pro_path(board), encoding='utf-8') as f:
        proj = json.load(f)
    ns = proj['net_settings']
    base = next(c for c in ns['classes'] if c['name'] == 'Default')
    base['clearance'] = default_mm
    ns['classes'], ns['netclass_patterns'] = [base], []
    for i, (name, mm, pats) in enumerate(specs):
        c = copy.deepcopy(base)
        c.update(name=name, clearance=mm, priority=i)
        ns['classes'].append(c)
        ns['netclass_patterns'] += [{'netclass': name, 'pattern': p} for p in pats]
    with open(pro_path(board), 'w', encoding='utf-8') as f:
        json.dump(proj, f, indent=2)


def stage(dst, specs):
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(BOARD, dst)
    set_classes(dst, 0.1, specs)
    return dst


def classes(board):
    with open(pro_path(board), encoding='utf-8') as f:
        return {c['name']: c['clearance']
                for c in json.load(f)['net_settings']['classes']}


def fan(inp, out, *extra):
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/bga_fanout.py', inp,
                   '--output', out] + ARGS + list(extra), accept=True)
    evidence(out, 'bga_fanout output')
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', r.stdout, re.M)
    if not m:
        raise AssertionError(f"BROKEN TEST: no JSON_SUMMARY\n{r.stdout[-1500:]}")
    return json.loads(m.group(1))


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


def class_rows(board, tmp):
    """The clearance rows the board's net classes add: its grade minus the
    grade of the same copper with the classes taken out."""
    bare = os.path.join(tmp, 'noclass_' + os.path.basename(board))
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(board, bare)
    set_classes(bare, classes(board)['Default'], [])
    a, b = drc(board), drc(bare)
    return sum(a.values()) - sum(b.values()), a


def gap_case(tmp):
    """5: one channel escape on a board whose IO class (0.17) cannot fit the
    gap between the balls: 0.12 + 2 x 0.17 = 0.46 mm against 0.45."""
    from kicad_parser import parse_kicad_pcb
    from net_queries import matches_net_filter
    src = stage(os.path.join(tmp, 'gap.kicad_pcb'), [('IO', 0.17, IO)])
    out = os.path.join(tmp, 'gap_fan.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', 'py_router/bga_fanout.py', src,
                   '--output', out] + ARGS + ['--clearance', '0.1', '--escape-method',
                                              'channel', '--no-fix-drc-settings'],
                  accept=True)
    evidence(out, 'bga_fanout output')
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(out)
    fp = pcb.footprints[REF]
    xs = sorted({round(p.global_x, 3) for p in fp.pads})
    ys = sorted({round(p.global_y, 3) for p in fp.pads})
    on_top = {(round(t.start_x, 3), round(t.start_y, 3)) for t in pcb.segments
              if t.layer == 'F.Cu'} | {(round(t.end_x, 3), round(t.end_y, 3))
                                       for t in pcb.segments if t.layer == 'F.Cu'}

    def inner_on_top(p):
        return (xs[0] < round(p.global_x, 3) < xs[-1]
                and ys[0] < round(p.global_y, 3) < ys[-1]
                and (round(p.global_x, 3), round(p.global_y, 3)) in on_top)
    is_io = [p for p in fp.pads if p.net_name
             and any(matches_net_filter(p.net_name, [g]) for g in IO)]
    top_io = [p.net_name for p in is_io if inner_on_top(p)]
    top_other = [p.net_name for p in fp.pads if p.net_id and p not in is_io
                 and inner_on_top(p)]
    check("the run says which balls left the top layer, and why",
          bool(re.search(r'Inner balls of \d+ net\(s\) in a wider net class stay off F\.Cu',
                         r.stdout)))
    check("no inner ball of the 0.17 class escapes on the top layer",
          not top_io, f"{sorted(top_io)}")
    check("...while inner balls at the run clearance still do", bool(top_other),
          f"{len(top_other)} on F.Cu")


def main():
    if not os.path.isfile(BOARD):
        print(f"SKIP: {os.path.relpath(BOARD, ROOT)} missing")
        return 77
    tmp = tempfile.mkdtemp(prefix='t1529bga_')
    try:
        src = stage(os.path.join(tmp, 'in.kicad_pcb'),
                    [('IO', 0.16, IO), ('Far', 0.4, FAR)])
        evidence(src)
        added, rows = class_rows(src, tmp)
        check("the staged input grades clean at its classes",
              sum(rows.values()) == 0, f"{rows}")

        print("1. classes honoured, no writeback")
        out = os.path.join(tmp, 'fan.kicad_pcb')
        js = fan(src, out, '--clearance', '0.1', '--no-fix-drc-settings')
        check("every ball escaped", js.get('failed') == 0 and (js.get('escaped') or 0) > 90,
              f"escaped={js.get('escaped')} failed={js.get('failed')}")
        added, rows = class_rows(out, tmp)
        check("the classes add no row to the grade (was 159)", added == 0,
              f"{added} class row(s); {rows}")

        print("2. the default writeback keeps the classes")
        out = os.path.join(tmp, 'fan_fix.kicad_pcb')
        js = fan(src, out, '--clearance', '0.1')
        cl = classes(out)
        check("IO is still 0.16 (was lowered to 0.1)",
              abs(cl.get('IO', 0) - 0.16) < 1e-9, f"{cl}")
        check("a class no net of the part is in is still 0.4 (was lowered to 0.1)",
              abs(cl.get('Far', 0) - 0.4) < 1e-9, f"{cl}")
        added, rows = class_rows(out, tmp)
        check("...and the output's grade has no class row", added == 0, f"{added}")

        print("3. --clearance-ceiling 0.1 writes min(class, ceiling)")
        out = os.path.join(tmp, 'fan_ceil.kicad_pcb')
        js = fan(src, out, '--clearance-ceiling', '0.1')
        cl = classes(out)
        check("every class at 0.1", all(abs(v - 0.1) < 1e-9 for v in cl.values()), f"{cl}")
        check("every ball escaped", js.get('failed') == 0, f"failed={js.get('failed')}")

        print("4. drc_grazes reads the written project's classes")
        out = os.path.join(tmp, 'fan_ceil_nofix.kicad_pcb')
        js = fan(src, out, '--clearance-ceiling', '0.1', '--no-fix-drc-settings')
        added, rows = class_rows(out, tmp)
        g = js.get('drc_grazes') or {}
        mine = sum(g.get(k, 0) for k in ('segment_segment', 'via_segment',
                                         'pad_segment', 'pad_via'))
        check("copper spaced at the ceiling violates the kept 0.16 class",
              added > 0, f"{added} class row(s)")
        check("...and the step's own drc_grazes counts check_drc's rows (was blind to them)",
              mine == sum(rows.values()),
              f"drc_grazes {g} vs check_drc {rows}")

        print("5. the top-layer gap test at a ball's own class")
        gap_case(tmp)
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
