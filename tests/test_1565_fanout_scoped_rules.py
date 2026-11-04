#!/usr/bin/env python3
"""#1565: the QFN and BGA fanouts hold a class/type-scoped (#1491) and a
track-scoped (#735) .kicad_dru clearance rule, as the route steps do.

They used to read only the per-LAYER rules, so a rule raising a pair above the
class value was priced at the class: on a 0.4 mm-pitch QFN whose DATA nets sit
in a class 'HV' at the 0.1 Default, with `A.NetClass == 'HV' && B.NetClass ==
'HV'` at 0.2 mm, qfn_fanout laid 17 segment-segment pairs check_drc (and
KiCad: 12) grade below the rule, reporting every pad escaped; the same rule
spelled as a track rule (`A.Type == 'track' && B.Type == 'track' &&
A.NetClass == 'HV'`) gave the same 17. On a 0.8 mm BGA with 0.16 mm between IO
nets, bga_fanout left 23 rows (10 via-segment, 13 pad-segment) the rule adds,
laid by its fab-floor rescue below the run clearance.

Checks (each a fanout run, graded by check_drc.py, which grades both rules):
  1. QFN, scoped rule: every pad escapes and the output grades clean.
  2. QFN, the same rule as a track rule: the same.
  3. QFN, a .kicad_dru with no scoped or track rule: copper identical to the
     run with no .kicad_dru at all (the change is inert there).
  4. BGA, scoped rule: no clearance row the rule adds (the output's grade
     equals the grade of the same copper with the .kicad_dru taken away),
     and the step names the rule it held.

    python3 tests/test_1565_fanout_scoped_rules.py
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

QFN = os.path.join(ROOT, 'kicad_files', 'haasoscope_pro_max_test.kicad_pcb')
BGA = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
QFN_ARGS = ['--component', 'U2', '--nets', '*', '!GND', '--width', '0.15',
            '--fab-tier', 'standard', '--escalation', 'board', '--no-fix-drc-settings']
BGA_ARGS = ['--component', 'U30', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
            '--nets', '*', '!GND', '!+3V3', '!+1V2', '--track-width', '0.12',
            '--via-size', '0.3', '--via-drill', '0.15', '--no-fix-drc-settings']
SCOPED = "A.NetClass == 'HV' && B.NetClass == 'HV'"
TRACK = "A.Type == 'track' && B.Type == 'track' && A.NetClass == 'HV'"
ROWS = ('SEGMENT-SEGMENT', 'SEGMENT-SEGMENT-TRACK-RULE', 'VIA-SEGMENT',
        'PAD-SEGMENT', 'PAD-VIA', 'VIA-VIA')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def stem(board):
    return os.path.splitext(board)[0]


def stage(src, dst, patterns, rule=None, mm=None):
    """Copy `src` with its siblings: Default 0.1 plus a class HV, also 0.1,
    holding the nets of `patterns`; and a .kicad_dru with `rule` at `mm` (a
    condition string), or with only a track-width rule when `rule` is ''."""
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(src, dst)
    pro = stem(dst) + '.kicad_pro'
    if os.path.isfile(pro):
        with open(pro, encoding='utf-8') as f:
            proj = json.load(f)
    else:
        proj = {'board': {'design_settings': {'rules': {
                    'min_clearance': 0.0, 'min_track_width': 0.1}}},
                'net_settings': {'meta': {'version': 4}, 'classes': [{
                    'name': 'Default', 'clearance': 0.1, 'track_width': 0.2,
                    'via_diameter': 0.6, 'via_drill': 0.3, 'diff_pair_gap': 0.25,
                    'diff_pair_width': 0.2, 'diff_pair_via_gap': 0.25,
                    'microvia_diameter': 0.3, 'microvia_drill': 0.1,
                    'priority': 2147483647}]},
                'meta': {'filename': os.path.basename(pro), 'version': 3}}
    ns = proj['net_settings']
    base = next(c for c in ns['classes'] if c['name'] == 'Default')
    base['clearance'] = 0.1
    hv = copy.deepcopy(base)
    hv.update(name='HV', clearance=0.1, priority=0)
    ns['classes'] = [base, hv]
    ns['netclass_patterns'] = [{'netclass': 'HV', 'pattern': p} for p in patterns]
    with open(pro, 'w', encoding='utf-8') as f:
        json.dump(proj, f, indent=2)
    dru = stem(dst) + '.kicad_dru'
    if rule is not None:
        body = (f'(rule "HV rule" (constraint clearance (min {mm}mm))\n'
                f'  (condition "{rule}"))\n' if rule else
                '(rule "width" (constraint track_width (min 0.1mm)))\n')
        with open(dru, 'w', encoding='utf-8') as f:
            f.write('(version 1)\n' + body)
    elif os.path.exists(dru):
        os.remove(dru)
    return evidence(dst)


def fan(script, inp, out, args):
    r = run_check([sys.executable, '-X', 'utf8', f'py_router/{script}', inp,
                   '--output', out] + args, accept=True)
    evidence(out, f'{script} output')
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', r.stdout, re.M)
    if not m:
        raise AssertionError(f"BROKEN TEST: no JSON_SUMMARY\n{r.stdout[-1500:]}")
    return json.loads(m.group(1)), r.stdout


def drc(board):
    """check_drc.py's grade (it reads the classes and the .kicad_dru): the
    count of each clearance row kind."""
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py', board,
                        '--no-size-checks', '--max-print', '0'],
                       cwd=ROOT, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', timeout=600)
    out = r.stdout + r.stderr
    if 'Traceback' in out or not ('NO DRC VIOLATIONS' in out or 'DRC VIOLATION' in out):
        raise AssertionError(f"BROKEN TEST: no verdict from check_drc.py\n{out[-1500:]}")
    return {k: int(m.group(1)) for k in ROWS
            for m in [re.search(rf'^{k} violations \((\d+)\)', out, re.M)] if m}


def copper(board):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        p = parse_kicad_pcb(board)
    return (sorted((round(s.start_x, 4), round(s.start_y, 4), round(s.end_x, 4),
                    round(s.end_y, 4), round(s.width, 4), s.layer, s.net_id)
                   for s in p.segments if not getattr(s, 'graphic', False)),
            sorted((round(v.x, 4), round(v.y, 4), round(v.size, 4), v.net_id)
                   for v in p.vias))


def main():
    for b in (QFN, BGA):
        if not os.path.isfile(b):
            print(f"SKIP: {os.path.relpath(b, ROOT)} missing")
            return 77
    tmp = tempfile.mkdtemp(prefix='t1565_')
    try:
        for n, (label, rule) in enumerate((('scoped', SCOPED), ('track', TRACK)), 1):
            print(f"{n}. QFN, the HV rule as a {label} rule (0.2 mm over a 0.1 class)")
            inp = stage(QFN, os.path.join(tmp, f'q_{label}.kicad_pcb'),
                        ['Net-(U2A-DATA_*'], rule, 0.2)
            out = os.path.join(tmp, f'q_{label}_fan.kicad_pcb')
            js, log = fan('qfn_fanout.py', inp, out, QFN_ARGS)
            check("every pad escaped", js.get('failed') == 0 and js.get('escaped') == 47,
                  f"escaped={js.get('escaped')} failed={js.get('failed')}")
            rows = drc(out)
            check("check_drc finds no clearance row (was 17 segment-segment)",
                  not rows, f"{rows}")
            check("...and the step says it held the rule", '(#1565)' in log)

        print("3. QFN, a .kicad_dru with no scoped or track rule: inert")
        a = stage(QFN, os.path.join(tmp, 'q_nodru.kicad_pcb'), ['Net-(U2A-DATA_*'])
        b = stage(QFN, os.path.join(tmp, 'q_widthdru.kicad_pcb'), ['Net-(U2A-DATA_*'], '')
        fan('qfn_fanout.py', a, os.path.join(tmp, 'q_nodru_fan.kicad_pcb'), QFN_ARGS)
        _js, log = fan('qfn_fanout.py', b, os.path.join(tmp, 'q_widthdru_fan.kicad_pcb'),
                       QFN_ARGS)
        check("the same copper as with no .kicad_dru",
              copper(os.path.join(tmp, 'q_nodru_fan.kicad_pcb'))
              == copper(os.path.join(tmp, 'q_widthdru_fan.kicad_pcb')))
        check("...and no rule is claimed held", '(#1565)' not in log)

        print("4. BGA, the scoped rule at 0.16 mm over a 0.1 class")
        inp = stage(BGA, os.path.join(tmp, 'b.kicad_pcb'),
                    ['/IO_Banks/D*', '/IO_Banks/Q*'], SCOPED, 0.16)
        out = os.path.join(tmp, 'b_fan.kicad_pcb')
        js, log = fan('bga_fanout.py', inp, out, BGA_ARGS)
        check("the fanout laid copper", (js.get('escaped') or 0) > 0,
              f"escaped={js.get('escaped')}")
        ruled = drc(out)
        bare = os.path.join(tmp, 'b_fan_bare.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(out, bare)
        os.remove(stem(bare) + '.kicad_dru')
        unruled = drc(bare)
        added = {k: ruled.get(k, 0) - unruled.get(k, 0) for k in ROWS
                 if ruled.get(k, 0) != unruled.get(k, 0)}
        check("no clearance row the rule adds (were 10 via-segment, 13 pad-segment)",
              not added, f"with the rule {ruled}, without {unruled}")
        check("...and the step says it held the rule", '(#1565)' in log)
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
