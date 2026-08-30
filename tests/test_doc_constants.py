#!/usr/bin/env python3
"""A measured NUMBER in the routing docs is a claim about the code.

Sibling of `test_doc_flag_liveness.py`, which holds the docs' `--flags` and
script names to the engine's real surface. This holds their CONSTANTS.

The routing skill's silent-timeout table names two timeouts by value:

    | `EXACT_FILL_TIMEOUT` (`py_router/kicad_exact_fill.py`) | 300 s | ...
    | `ORACLE_DRC_TIMEOUT` (`py_router/kicad_oracle.py`)     | 240 s | ...

Both are load-bearing advice -- they tell a reader how long a phase may sit
before its silent fallback fires, and both degrade with no failure signal and
no effect on the exit code, so the number is the ONLY warning the reader gets.
Change `EXACT_FILL_TIMEOUT` from 300 to 900 and every test in the repo stays
green while the doc quietly lies. That is the same defect as documenting a flag
that does not exist, which is why it is gated the same way.

DELIBERATELY PATTERN-DRIVEN, not a hardcoded pair: the gate finds every row of
the form

    | `CONST_NAME` (`path/to/module.py`) | <int> s |

and re-derives each one from its module. A new row in that table is covered the
day it is written, with no edit here -- a gate that must be extended by hand to
cover new claims is a gate that silently stops covering them.

Run: python3 -X utf8 tests/test_doc_constants.py
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#: Same doc set as the flag gate. Keep the two in step.
DOCS = (
    os.path.join('.claude', 'skills', 'plan-pcb-routing', 'SKILL.md'),
    os.path.join('.claude', 'skills', 'plan-pcb-placement-and-routing',
                 'SKILL.md'),
    os.path.join('docs', 'api-routing-config.md'),
)

#: A doc row that quotes a module constant by value. The `s` unit is part of
#: the pattern on purpose: it keeps the gate to quantities whose unit the doc
#: states, rather than every integer that happens to sit in a table cell.
_ROW = re.compile(
    r'\|\s*`([A-Z_][A-Z0-9_]*)`\s*\(\s*`([^`]+\.py)`\s*\)\s*\|\s*(\d+)\s*s\s*\|')

#: The constant's definition in its own module, read from SOURCE rather than by
#: importing: importing py_router modules pulls in the compiled router and the
#: rest of the engine, which a doc check has no business needing.
def _defined_value(rel_module, const):
    path = os.path.join(ROOT, rel_module)
    if not os.path.isfile(path):
        return None, f'{rel_module} does not exist'
    with open(path, encoding='utf-8', errors='replace') as fh:
        src = fh.read()
    m = re.search(r'^%s\s*=\s*(\d+)' % re.escape(const), src, re.M)
    if not m:
        return None, f'{const} is not defined at module level in {rel_module}'
    return int(m.group(1)), ''


FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def rows_in(rel):
    path = os.path.join(ROOT, rel)
    if not os.path.isfile(path):
        return []
    with open(path, encoding='utf-8', errors='replace') as fh:
        return _ROW.findall(fh.read())


def main():
    total = 0
    for rel in DOCS:
        found = rows_in(rel)
        if not found:
            continue
        print(rel)
        for const, module, quoted in found:
            total += 1
            actual, why = _defined_value(module, const)
            if actual is None:
                check(f'{const} re-derives from {module}', False, why)
                continue
            check(f'{const} is {quoted} s in {module}',
                  actual == int(quoted),
                  f'the doc says {quoted} s; {module} defines {actual}. '
                  f'Update the doc row, or the constant -- one of them is '
                  f'lying to the next reader, and this number is the only '
                  f'warning a silent fallback gives.')

    # The gate's own vacuity guard. If the table is reworded so the pattern no
    # longer matches, every check above silently stops running and this file
    # keeps printing OK -- the exact failure mode test_doc_flag_liveness was
    # written after (a gate blind to the thing it was built for).
    check('the scan found the constants it is here to check',
          total >= 2,
          f'only {total} quoted constant row(s) matched across {len(DOCS)} '
          f'docs; the silent-timeout table alone should supply two. Either '
          f'the table was reworded (fix the pattern) or it was deleted (say '
          f'so deliberately) -- do not leave this passing on zero rows.')

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print(f'OK ({total} quoted constant(s) re-derived)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
