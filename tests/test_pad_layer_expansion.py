#!/usr/bin/env python3
"""One expansion for "which copper layers does this pad occupy" (#722 follow-up).

KiCad writes TWO copper layer-set tokens in a pad's `layers` list: `*.Cu`
(every copper layer) and `F&B.Cu` (front and back only, never the inners).
`expand_pad_layers` handled the first and passed the second through VERBATIM,
because it only tested `endswith('.Cu')` -- and "F&B.Cu" matches no real layer
name, so every consumer scoping by it gave such a pad NO copper layer at all.

That is not a checker-only bug: `expand_pad_layers` is what `check_connected`
(the authority) and `connectivity.py` (the ROUTER) scope pads with, so an
`F&B.Cu` pad was unroutable and graded disconnected.

#697 had forked a second copy into `check_drc.pad_copper_layers` to cure the
clearance paths, which left the authority and the router wrong. Two spellings
of one expansion is the defect class #695/#722 are about, so the fix went into
`expand_pad_layers` and the fork now delegates to it.

    python3 tests/test_pad_layer_expansion.py
"""
import ast
import io
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522
os.environ.setdefault('KRT_NO_BANNER', '1')

from kicad_parser import Pad                                   # noqa: E402
from net_queries import expand_pad_layers                      # noqa: E402
import check_drc                                               # noqa: E402

FAILURES = []
R4 = ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def _pad(layers):
    return Pad(component_ref='U1', pad_number='1', global_x=0.0, global_y=0.0,
               local_x=0.0, local_y=0.0, size_x=1.0, size_y=1.0, shape='rect',
               layers=list(layers), net_id=1, net_name='/A', rotation=0.0,
               drill=0.0)


#: declaration -> the copper layers it occupies on a 4-layer board.
TABLE = [
    (['*.Cu'],            {'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'}),
    (['F&B.Cu'],          {'F.Cu', 'B.Cu'}),          # the token that was dropped
    (['F&B'],             {'F.Cu', 'B.Cu'}),          # bare spelling, as the
                                                      # zone paths already accept
    (['F.Cu'],            {'F.Cu'}),
    (['In1.Cu'],          {'In1.Cu'}),
    (['*.Mask'],          set()),                     # mask wildcard is NOT copper
    (['*.Paste'],         set()),
    (['F&B.Cu', '*.Mask'], {'F.Cu', 'B.Cu'}),
    (['F.Cu', 'B.Cu'],    {'F.Cu', 'B.Cu'}),
]


def main():
    print('the expansion table')
    for decl, want in TABLE:
        got = set(expand_pad_layers(list(decl), list(R4)))
        check(f'{decl} -> {sorted(want)}', got == want, f'got {sorted(got)}')

    print('F&B is front+back, and specifically NOT the inner layers')
    fb = set(expand_pad_layers(['F&B.Cu'], list(R4)))
    check('an inner layer is not credited to an F&B pad',
          not (fb & {'In1.Cu', 'In2.Cu'}), f'got {sorted(fb)}')
    check('...and it is not empty either (the old passthrough)',
          fb == {'F.Cu', 'B.Cu'}, f'got {sorted(fb)}')
    check('the raw token never survives expansion',
          not any('&' in L for L in fb), f'got {sorted(fb)}')

    print('the two entry points give the SAME answer')
    for decl, want in TABLE:
        a = set(expand_pad_layers(list(decl), list(R4)))
        b = check_drc.pad_copper_layers(_pad(decl), list(R4))
        check(f'expand_pad_layers == pad_copper_layers for {decl}', a == b,
              f'{sorted(a)} vs {sorted(b)}')

    print('...structurally, not just today')
    # pad_copper_layers must DELEGATE. Re-implementing it is how the two
    # answers drifted in the first place, and a value test cannot see a
    # re-implementation that happens to agree on this table.
    src = io.open(check_drc.__file__, encoding='utf-8').read()
    fn = next(n for n in ast.walk(ast.parse(src))
              if isinstance(n, ast.FunctionDef) and n.name == 'pad_copper_layers')
    calls = {c.func.id for c in ast.walk(fn)
             if isinstance(c, ast.Call) and isinstance(c.func, ast.Name)}
    check('pad_copper_layers calls expand_pad_layers',
          'expand_pad_layers' in calls, f'calls {sorted(calls)}')
    check('...and does not re-implement the wildcard branch',
          'F&B.Cu' not in ast.get_source_segment(src, fn).split('"""')[-1],
          'a literal layer-set token still appears in its body')

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
