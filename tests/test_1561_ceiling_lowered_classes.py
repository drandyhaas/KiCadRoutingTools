"""#1561: `--clearance-ceiling` lowering a net class by more than 2x is named.

The ceiling caps every net class at min(class, ceiling) and the writeback
clamps the project to it (the #530 contract, kept). A class far above the
ceiling -- a 0.6 mm high-voltage class under a 0.1 ceiling -- used to be
lowered with one generic "capped at" line. Every class the cap lowers by more
than `fab_tiers.CEILING_LOWER_RATIO` is now named on the console and in
JSON_SUMMARY.design_rules.ceiling_lowered_classes (class, clearance, capped
value, nets), from `fab_tiers.cap_net_clearances`, the one function every
front caps the class map through. Disclosure only: the map is unchanged.

    python3 tests/test_1561_ceiling_lowered_classes.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
sys.path.insert(0, os.path.join(TESTS, 'oracle'))
import run_utils  # noqa: E402

from constraint_agreement import write_board  # noqa: E402

RUN_ALL_TIMEOUT = 600
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


# nets A (1) and B (2) in class HV (0.6); C (3) in class MID (0.2, exactly 2x
# a 0.1 ceiling, so not named)
CLASSES = [{'name': 'HV', 'clearance': 0.6, 'track_width': 0.2, 'priority': 0},
           {'name': 'MID', 'clearance': 0.2, 'track_width': 0.2, 'priority': 1}]
PATTERNS = [('A', 'HV'), ('B', 'HV'), ('C', 'MID')]


def unit(td):
    print('1. cap_net_clearances caps as before and names the classes')
    import fab_tiers
    b = os.path.join(td, 'u.kicad_pcb')
    write_board(b, classes=CLASSES, patterns=PATTERNS)
    names = {1: 'A', 2: 'B', 3: 'C'}
    fab_tiers.reset_ledger()
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        got = fab_tiers.cap_net_clearances({1: 0.6, 2: 0.6, 3: 0.2}, 0.1,
                                           net_names=names, board_path=b)
    check('the map is min(class, ceiling), unchanged by the disclosure',
          got == {1: 0.1, 2: 0.1, 3: 0.1}, f'{got}')
    rows = fab_tiers.escalation_summary().get('ceiling_lowered_classes')
    check('design_rules names HV (6x) and not MID (exactly 2x)',
          rows == [{'class': 'HV', 'clearance': 0.6, 'capped': 0.1, 'ratio': 6.0,
                    'nets': ['A', 'B'], 'net_count': 2}], json.dumps(rows))
    check('the console names it once', out.getvalue().count('LOWERS NET CLASS HV') == 1,
          out.getvalue())
    check('the non-route CLIs add design_rules for it',
          fab_tiers.narrowed_summary() is not None)
    with contextlib.redirect_stdout(io.StringIO()):
        fab_tiers.cap_net_clearances({1: 0.6}, 0.1, net_names=names, board_path=b)
    check('a second cap in the same run does not repeat the line', len(
        fab_tiers.escalation_summary()['ceiling_lowered_classes']) == 1)
    fab_tiers.reset_ledger()
    check('a new run starts with no disclosure',
          'ceiling_lowered_classes' not in fab_tiers.escalation_summary())
    with contextlib.redirect_stdout(io.StringIO()):
        fab_tiers.cap_net_clearances({1: 0.6, 2: 0.6}, 0.1, net_names=names,
                                     class_of={'A': 'HV', 'B': 'HV'})
    rows = fab_tiers.escalation_summary().get('ceiling_lowered_classes') or []
    check("a front that read the classes off the live board names them (class_of)",
          [r['class'] for r in rows] == ['HV'], json.dumps(rows))
    fab_tiers.reset_ledger()


def routed(td):
    print('2. route.py --clearance-ceiling: console and JSON_SUMMARY')
    b = os.path.join(td, 'r.kicad_pcb')
    write_board(b, classes=CLASSES, patterns=PATTERNS, rules={'min_clearance': 0.05},
                footprints=[{'ref': 'U1', 'x': 5, 'y': 10, 'net_id': 1, 'net_name': 'A'},
                            {'ref': 'U2', 'x': 15, 'y': 10, 'net_id': 1, 'net_name': 'A'}])
    run_utils.evidence(b)
    for label, extra, want in (('with --clearance-ceiling 0.1', ['--clearance-ceiling', '0.1'], True),
                               ('without a ceiling', [], False)):
        o = os.path.join(td, f'r_out{int(want)}.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8', os.path.join('py_router', 'route.py'),
                            b, o, '--nets', 'A'] + extra,
                           capture_output=True, text=True, cwd=REPO, timeout=300)
        line = next((ln for ln in r.stdout.splitlines() if ln.startswith('JSON_SUMMARY:')), None)
        check(f'{label}: the route ran', r.returncode == 0 and line is not None,
              (r.stdout + r.stderr)[-600:])
        if line is None:
            continue
        rows = (json.loads(line[len('JSON_SUMMARY:'):]).get('design_rules') or {}).get(
            'ceiling_lowered_classes')
        named = 'LOWERS NET CLASS HV 6.0x -- 0.6 -> 0.1 mm' in r.stdout
        if want:
            check(f'{label}: the console names HV 0.6 -> 0.1', named, r.stdout[-800:])
            check(f'{label}: JSON_SUMMARY.design_rules.ceiling_lowered_classes names HV',
                  bool(rows) and rows[0]['class'] == 'HV' and rows[0]['capped'] == 0.1
                  and 'A' in rows[0]['nets'], json.dumps(rows))
        else:
            check(f'{label}: nothing is named', not named and not rows, json.dumps(rows))


def main():
    td = tempfile.mkdtemp(prefix='krt1561c_')
    try:
        for t in (unit, routed):
            t(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
