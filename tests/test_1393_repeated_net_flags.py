#!/usr/bin/env python3
"""#1393: a repeated `--impedance-nets` / `--nets` kept only the LAST occurrence.

`nargs='+'` without `action='extend'` replaces the list on each flag, so
`--impedance-nets A --impedance-nets B` graded B alone and the verdict
depended on argument order. grade.py's repeatable `--spec impedance-nets=X`
emits exactly that form.

  1. board_score's parser accumulates repeated --impedance-nets (both forms).
  2. check_complete forwards BOTH values to board_score.
  3. Every `--nets` / `--impedance-nets` flag of the graders declared
     nargs='+' says action='extend' (a lint over the sources, so a new
     grader flag cannot reintroduce the order dependence).

    python3 tests/test_1393_repeated_net_flags.py
"""
import ast
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
for _p in ('py_router', 'py_tools'):
    sys.path.insert(0, os.path.join(ROOT, _p))

failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def main():
    import board_score
    p = board_score.build_parser()
    a = p.parse_args(['x.kicad_pcb', '--impedance-nets', 'A*',
                      '--impedance-nets', 'B*'])
    check('board_score: repeated flag accumulates',
          a.impedance_nets == ['A*', 'B*'], str(a.impedance_nets))
    a = p.parse_args(['x.kicad_pcb', '--impedance-nets', 'A*', 'C*',
                      '--impedance-nets', 'B*'])
    check('board_score: mixed forms accumulate',
          a.impedance_nets == ['A*', 'C*', 'B*'], str(a.impedance_nets))

    import check_complete as CC
    seen = []

    def fake_run(args, timeout=0):
        seen.append(list(args))
        return 0, ''
    CC._run = fake_run
    board = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()):
        CC.main([board, '--skip-slow', '--impedance-nets', 'A*',
                 '--impedance-nets', 'B*'])
    bs = [s for s in seen if s and s[0].endswith('board_score.py')]
    check('check_complete ran board_score once', len(bs) == 1, str(seen)[:200])
    if bs:
        i = bs[0].index('--impedance-nets')
        check('check_complete forwards both globs',
              bs[0][i + 1:i + 3] == ['A*', 'B*'], str(bs[0][i:]))

    targets = {
        'py_tools/board_score.py': ('--impedance-nets',),
        'check_complete.py': ('--impedance-nets',),
        'py_tools/check_impedance.py': ('--nets',),
        'py_router/check_drc.py': ('--nets',),
        'py_router/check_connected.py': ('--nets',),
    }
    for rel, flags in targets.items():
        with open(os.path.join(ROOT, rel), encoding='utf-8') as fh:
            tree = ast.parse(fh.read())
        found = set()
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'add_argument'):
                continue
            names = [x.value for x in node.args if isinstance(x, ast.Constant)]
            kw = {k.arg: k.value for k in node.keywords}
            for f in flags:
                if f in names and isinstance(kw.get('nargs'), ast.Constant) \
                        and kw['nargs'].value in ('+', '*'):
                    found.add(f)
                    act = kw.get('action')
                    check(f'{rel} {f} is action=extend',
                          isinstance(act, ast.Constant) and act.value == 'extend')
        check(f'{rel}: lint found its flag(s)', found == set(flags), str(found))
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
