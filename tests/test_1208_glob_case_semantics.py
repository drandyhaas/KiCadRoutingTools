#!/usr/bin/env python3
"""#1208: a net or reference glob selects the same things on every platform.

`fnmatch.fnmatch` and `fnmatch.filter` pass both sides through
`os.path.normcase`, which lower-cases on Windows and does nothing on POSIX. So
`--nets '/*PCIE*'` selected `/PCIe-M2/FB` on Windows only, while netclass
membership (`list_nets`, `fnmatchcase`) never did: one command routed and
graded different nets per host. Every name glob now uses `fnmatchcase`.

Checks:
  1. Emulated Windows (normcase lower-cases): net_pattern_matches,
     matches_net_filter, matches_diff_pair_patterns and identify_power_nets
     answer exactly as on POSIX.
  2. AST gate: no plain `fnmatch.fnmatch(` / `fnmatch.filter(` / bare
     `fnmatch(` call in the shipped trees, except the file-path globs listed
     in PATH_GLOBS (a filename's case rules ARE the platform's).

    python3 tests/test_1208_glob_case_semantics.py
"""
import ast
import fnmatch
import os
import sys
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import net_queries                                             # noqa: E402

TREES = ('py_router', 'py_tools', 'py_placer', 'kicad_routing_plugin')
# file -> why its plain fnmatch is a FILE glob, matched as the platform does.
PATH_GLOBS = {
    os.path.join('py_tools', 'make_film.py'): 'reject globs over board file paths',
}
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def answers():
    pcb = SimpleNamespace(nets={
        1: SimpleNamespace(name='/PCIE_A', pads=[]),
        2: SimpleNamespace(name='/PCIe-M2/FB', pads=[]),
        3: SimpleNamespace(name='/Power/vcc_io', pads=[])})
    return (
        net_queries.net_pattern_matches('/PCIe-M2/FB', '/*PCIE*'),
        net_queries.net_pattern_matches('/PCIE_A', '/*PCIE*'),
        net_queries.net_pattern_matches('/Power/vcc_io', 'VCC*'),
        net_queries.matches_net_filter('/PCIe-M2/FB', ['/*PCIE*']),
        net_queries.matches_diff_pair_patterns('/usb_p', '/usb', ['/USB*']),
        sorted(net_queries.identify_power_nets(pcb, ['*VCC*'], [0.5])),
    )


def main():
    # 1. Emulated Windows.
    posix = answers()
    check('POSIX: a differently-cased net is not selected',
          posix[:5] == (False, True, False, False, False) and posix[5] == [],
          str(posix))
    real = fnmatch.os.path.normcase
    fnmatch.os.path.normcase = lambda s: s.replace('/', '\\').lower()
    try:
        win = answers()
        # The emulation is live: plain fnmatch now folds case.
        check('precondition: the emulation folds plain fnmatch',
              fnmatch.fnmatch('/PCIe-M2/FB', '/*PCIE*'))
    finally:
        fnmatch.os.path.normcase = real
    check('emulated Windows answers exactly as POSIX', win == posix,
          f'windows {win} vs posix {posix}')

    # 2. AST gate.
    offenders = []
    for tree in TREES:
        for dirpath, _dirs, files in os.walk(os.path.join(ROOT, tree)):
            for fn in files:
                if not fn.endswith('.py'):
                    continue
                path = os.path.join(dirpath, fn)
                rel = os.path.relpath(path, ROOT)
                if rel in PATH_GLOBS:
                    continue
                try:
                    mod = ast.parse(open(path, encoding='utf-8').read())
                except SyntaxError:
                    continue
                for node in ast.walk(mod):
                    if not isinstance(node, ast.Call):
                        continue
                    f = node.func
                    plain = (isinstance(f, ast.Attribute)
                             and isinstance(f.value, ast.Name)
                             and f.value.id == 'fnmatch'
                             and f.attr in ('fnmatch', 'filter'))
                    bare = isinstance(f, ast.Name) and f.id == 'fnmatch'
                    if plain or bare:
                        offenders.append(f'{rel}:{node.lineno}')
    check('no plain fnmatch / fnmatch.filter in the shipped trees',
          not offenders, ', '.join(offenders[:12]))
    for rel in PATH_GLOBS:
        check(f'allowlisted path glob still exists: {rel}',
              os.path.isfile(os.path.join(ROOT, rel)))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
