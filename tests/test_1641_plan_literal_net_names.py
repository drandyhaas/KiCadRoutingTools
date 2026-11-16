#!/usr/bin/env python3
"""A plan's route step selects a bracketed net the way route.py does (#1641).

route.py's expand_net_patterns hands net_pattern_matches the board's net
names, so a pattern with no '*' or '?' that spells a net ('D[3]') selects
only that net. The GUI plan executor's own matcher (ai_plan._match_net_names)
called it without the names, so 'D[3]' still globbed: it selected '/D3' as
well, and '!D[3]' dropped '/D3' too.

The board has /D[3], /D3 and /D4. For each pattern list, the plan's selection
must equal route.py's; and the old call (no names) must select differently on
this board, or the test proves nothing.

    python3 tests/test_1641_plan_literal_net_names.py
"""
import contextlib
import io
import os
import shutil
import sys
import tempfile
import types

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for p in (ROOT, os.path.join(ROOT, 'py_router')):
    sys.path.insert(0, p)
sys.path.insert(0, TESTS_DIR)

# ai_plan does `import wx` at module level and nothing else with it at import;
# the suite image has no wx, and the matcher under test uses none.
sys.modules.setdefault('wx', types.ModuleType('wx'))

import run_utils                                                 # noqa: E402
from kicad_parser import parse_kicad_pcb                         # noqa: E402
from net_queries import expand_net_patterns, net_pattern_matches  # noqa: E402
from kicad_routing_plugin.ai_plan import _match_net_names        # noqa: E402

NETS = ['/D[3]', '/D3', '/D4']
failures = []


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def write_board(path):
    nets = ''.join(f'  (net {i + 1} "{n}")\n' for i, n in enumerate(NETS))
    fps = ''
    for i, n in enumerate(NETS):
        for j in (0, 1):
            fps += (f'  (footprint "t:R" (layer "F.Cu") (at {5 + 5 * i} {5 + 5 * j})\n'
                    f'    (property "Reference" "R{2 * i + j + 1}" (at 0 0) (layer "F.SilkS"))\n'
                    f'    (pad "1" smd rect (at 0 0) (size 1 1) (layers "F.Cu") '
                    f'(net {i + 1} "{n}")))\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20240108) (generator "test")\n'
                '  (general (thickness 1.6))\n'
                '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
                '  (net 0 "")\n' + nets + fps +
                '  (gr_rect (start 0 0) (end 30 20) (stroke (width 0.1) (type default))\n'
                '    (fill none) (layer "Edge.Cuts"))\n)\n')
    return run_utils.evidence(path, 'bracket-net board')


def main():
    tmp = tempfile.mkdtemp(prefix='t1641p_')
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(write_board(os.path.join(tmp, 'b.kicad_pcb')))
        old = sorted(n for n in NETS if net_pattern_matches(n, 'D[3]'))
        ok("the board discriminates: without the names 'D[3]' globs to /D3 too",
           '/D3' in old, repr(old))
        for patterns in (['D[3]'], ['/D[3]'], ['*', '!D[3]'], ['D*', '!/D[3]']):
            with contextlib.redirect_stdout(io.StringIO()):
                cli = sorted(expand_net_patterns(pcb, patterns, quiet=True))
            plan = sorted(_match_net_names(pcb, patterns))
            ok(f'{patterns}: the plan selects what route.py selects',
               plan == cli, f'plan {plan} vs route.py {cli}')
        with contextlib.redirect_stdout(io.StringIO()):
            only = sorted(_match_net_names(pcb, ['D[3]']))
        ok("'D[3]' selects /D[3] alone", only == ['/D[3]'], repr(only))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if failures:
        print(f'FAILED: {len(failures)}: {failures}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
