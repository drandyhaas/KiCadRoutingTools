#!/usr/bin/env python3
"""#1522: autoplan routes every step at the fab floor, not at the net class.

A finished board meets the fab clearance floor everywhere; a Default net class
declared above it is aspirational. So:

1. `analyze` on a board whose Default class (0.2) is above the fab floor
   returns the floor, `effective_floors`' drc_clearance, as THE clearance.
2. `run_plan` with a recording chain: every step's clearance argument -- the
   planes, each fanout rung, the decap pass, the diff pairs' gap and ceiling,
   and the route passes' ceiling -- is that one value, and no part's fanout
   ladder runs the same command twice (a rung that used to differ from the
   one before it only by the class-vs-floor clearance is gone).
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import autoplan as AP                                        # noqa: E402
from list_nets import (board_default_netclass_clearance,     # noqa: E402
                       effective_floors, read_design_rules)

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
FAILS = []


def expect(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILS.append(what)


def test_analyze():
    print("analyze: the clearance is the fab floor")
    declared = board_default_netclass_clearance(BOARD)
    a = AP.analyze(BOARD)
    floor = effective_floors(read_design_rules(BOARD).get('constraints') or {},
                             len(a['layers']))['drc_clearance']
    expect(declared and declared > floor + 1e-9,
           f"the board declares a Default class above the floor ({declared} > {floor})")
    expect(abs(a['clearance'] - floor) < 1e-9,
           f"clearance {a['clearance']} is the fab floor {floor}, not the class {declared}")


class RecordingChain:
    """Chain.run recorded; every fanout rung reports a dropped escape, so the
    whole ladder runs."""

    def __init__(self):
        self.cur = 'in'
        self.runs = []
        self.steps = []
        self.last_iterations = 100

    def run(self, kind, args, tag):
        self.runs.append((kind, tag, list(args)))
        self.steps.append({'tag': tag, 'ok': True})
        s ={'escaped': 1, 'failed': 1} if kind in ('bga', 'qfn') else {}
        return f'out{len(self.runs)}', s, {}

    def accept(self, out):
        if out:
            self.cur = out

    def grade(self, board):
        return {'open': 0, 'drc': 0, 'open_nets': []}

    def drc_nets(self, board):
        return set()

    def check_drc(self, board):
        return {'violations': 0, 'items': []}


def _values(args, flag):
    return [args[i + 1] for i, x in enumerate(args) if x == flag]


def test_run_plan():
    print("run_plan: one clearance for every step, no repeated fanout rung")
    a = dict(layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], clearance=0.09,
             track=0.09, via=0.45, drill=0.2, fine_via=0.25, fine_drill=0.15,
             planes=[('GND', 'In1.Cu')], plane_nets=['GND'], power_nets=['GND'],
             layer_costs=[1.0, -1.0, 1.0, 1.0], grid=0.05, congested=False,
             n_nets=10, pair_nets=['D_P', 'D_N'],
             globs={'GND': 'GND', 'D_P': 'D_P', 'D_N': 'D_N'},
             # parts: a coarse BGA (working via != fine via), a fine BGA
             # (they are the same via), a QFN and a mezzanine
             parts=[('U1', 'bga', 0.8), ('U2', 'bga', 0.5), ('U3', 'qfn', 0.5),
                    ('U4', 'mezz', 0.4)],
             # the key the pre-#1522 plan read; harmless now
             fab_clearance=0.09)
    ch = RecordingChain()
    orig = AP.Chain, AP.route_passes
    AP.Chain = lambda *args, **kw: ch
    AP.route_passes = lambda ch, a, L, costs, *r, **k: (ch, ch.cur, None)
    try:
        AP.run_plan('in', 'out', a, 'work')
    finally:
        AP.Chain, AP.route_passes = orig
    ch2 = RecordingChain()
    AP.route_passes(ch2, a, a['layers'], ['1.0', '-1.0', '1.0', '1.0'])
    runs = ch.runs + ch2.runs

    want = str(a['clearance'])
    for kind, tag, args in runs:
        got = (_values(args, '--clearance') + _values(args, '--clearance-ceiling')
               + _values(args, '--diff-pair-gap'))
        if kind == 'diff':
            want_here = {want, str(max(0.1, a['clearance']))}
        else:
            want_here = {want}
        expect(got and set(got) <= want_here,
               f"{tag}: clearance {got} is the floor {sorted(want_here)}")

    by_part = {}
    for kind, tag, args in runs:
        if kind in ('bga', 'qfn'):
            ref = tag.split('_')[1]
            by_part.setdefault(ref, []).append(args)
    for ref, ladder in sorted(by_part.items()):
        uniq = {tuple(x) for x in ladder}
        expect(len(uniq) == len(ladder),
               f"{ref}: {len(ladder)} fanout rungs, {len(uniq)} distinct")
    expect(len(by_part.get('U1', [])) == 3,
           f"coarse BGA: working via, fine via, dogbone ({len(by_part.get('U1', []))} rungs)")
    expect(len(by_part.get('U3', [])) == 2,
           f"QFN: the floor, then under-pad ({len(by_part.get('U3', []))} rungs)")
    expect(len(by_part.get('U4', [])) == 1,
           f"mezzanine: under-pad only ({len(by_part.get('U4', []))} rungs)")

    expect(len(by_part.get('U2', [])) == 2,
           f"fine BGA (its working via is the fine via): fine via, dogbone "
           f"({len(by_part.get('U2', []))} rungs)")


def main():
    test_analyze()
    test_run_plan()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS: #1522 autoplan at the fab floor")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
