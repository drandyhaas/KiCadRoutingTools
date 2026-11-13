#!/usr/bin/env python3
"""#1874: `A.isCoupledDiffPair()` is `AB.isCoupledDiffPair()`.

The condition parser read the coupled-pair predicate only when it was
spelled on `AB`. Spelled on one item -- `A.isCoupledDiffPair()`, as KiCad's
own examples write it -- the rule was "not modelled": route_diff.py did not
hold the pair gap it sets and check_drc.py graded the pair at the class
clearance.

The fixture (tests/fixtures/1874) is synthetic: 4 copper layers, Default
class clearance 0.2 mm (Board Setup minimum 0.05), and three pairs of
parallel 0.1 mm tracks with a 0.06 mm edge gap -- `/D0_P` + `/D0_N` on F.Cu,
`/D1_P` + `/D1_N` on In1.Cu, and `/X_P` + `/Y_N` on F.Cu, which is no pair.
Each row of `ROWS` is one rule `clearance (min 0.055mm)` with its condition
and layer clause, and the clearance violations kicad-cli 10.0.8 reports for
it (`--all-track-errors`): the three spellings are one predicate, which reads
the pair whichever item it is called on.

Checks (no routing):
  1. each row's check_drc violations are KiCad's;
  2. a pair-gap rule of the shape boards carry -- two net classes OR'ed, the
     predicate on `A`, Track/Arc type terms on both items, a layer clause --
     parses as a coupled-pair rule, with nothing unsupported;
  3. the predicate OR'ed with another term is still left unread.
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, TESTS)
from run_utils import check, evidence  # noqa: E402
from design_rules import parse_dru  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
BOARD = os.path.join(TESTS, 'fixtures', '1874', 'pairs')
TYPES = "(A.Type == 'Track' || A.Type == 'Arc') && (B.Type == 'Track' || B.Type == 'Arc')"
D0, D1, XY = ('/D0_N', '/D0_P'), ('/D1_N', '/D1_P'), ('/X_P', '/Y_N')
# (tag, layer clause, condition, the pairs KiCad reports)
ROWS = (
    ('none', None, None, [D0, D1, XY]),
    ('AB', None, "AB.isCoupledDiffPair()", [XY]),
    ('A', None, "A.isCoupledDiffPair()", [XY]),
    ('B', None, "B.isCoupledDiffPair()", [XY]),
    ('A_types', None, "A.isCoupledDiffPair() && " + TYPES, [XY]),
    ('A_outer', 'outer', "A.isCoupledDiffPair() && " + TYPES, [D1, XY]),
    ('A_inner', 'inner', "A.isCoupledDiffPair() && " + TYPES, [D0, XY]),
)
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def dru(layer, cond):
    return ('(version 1)\n(rule "gap"\n'
            + (f'  (layer {layer})\n' if layer else '')
            + '  (constraint clearance (min 0.055mm))\n'
            + f'  (condition "{cond}"))\n')


def main():
    print("1. check_drc against kicad-cli 10.0.8")
    with tempfile.TemporaryDirectory() as td:
        for tag, layer, cond, want in ROWS:
            sub = os.path.join(td, tag)
            os.mkdir(sub)
            for ext in ('.kicad_pcb', '.kicad_pro'):
                shutil.copy(evidence(BOARD + ext, 'the fixture'),
                            os.path.join(sub, 'b' + ext))
            if cond is not None:
                with open(os.path.join(sub, 'b.kicad_dru'), 'w', encoding='utf-8') as fh:
                    fh.write(dru(layer, cond))
            out = os.path.join(sub, 'drc.json')
            r = check(PY + [os.path.join(REPO, 'py_router', 'check_drc.py'),
                            os.path.join(sub, 'b.kicad_pcb'),
                            '--clearance-margin-mm', '0.0005', '--json', out],
                      code=1, refuse='violation')
            with open(evidence(out, 'the check_drc report'), encoding='utf-8') as fh:
                doc = json.load(fh)
            got = sorted(tuple(sorted((i['net1'], i['net2']))) for i in doc['items'])
            ok(got == sorted(want), f"{tag}: {got}")
            if cond is not None:
                ok('not modelled' not in (r.stdout or '') + (r.stderr or ''),
                   f"{tag}: the rule is read")

    print("2. the shape a board's pair-gap rule has")
    rules, _notes = parse_dru(dru(
        'outer', "(A.hasNetclass('HS') || A.hasNetclass('HS_CLK')) && "
                 "A.isCoupledDiffPair() && " + TYPES))
    r = rules[0]
    ok(r.unsupported is None and r.coupled and not r.keep_apart and r.layer_clause == 'outer',
       f"parses as a coupled-pair rule: unsupported={r.unsupported!r} coupled={r.coupled}")
    rules, _notes = parse_dru(dru(None, "!A.isCoupledDiffPair()"))
    ok(rules[0].unsupported is None and rules[0].keep_apart,
       "and its negation as a keep-pairs-apart rule")

    print("3. the predicate under an OR")
    rules, _notes = parse_dru(dru(None, "A.isCoupledDiffPair() || A.NetClass == 'HS'"))
    ok(rules[0].unsupported is not None, f"stays unread: {rules[0].unsupported}")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
