#!/usr/bin/env python3
"""autoroute's size ladder: the standard tier then the fab floor (#2021), the
fab floor alone on demand (#1522), the board's own rules with --board-rules.

1. `analyze` on a board whose Default class (0.2) is above the fab floor:
   by default its first size rung is the standard tier, not the class (an
   aspirational class, as the plan skill reads one), then the floor, each
   strictly narrower; with `floor_first` the one rung is the floor,
   `effective_floors`' drc_clearance.
2. `run_plan` at one rung (--floor-first) with a recording chain: every
   step's clearance argument -- the planes, each fanout rung, the decap
   pass, the diff pairs' gap and ceiling, and the route passes' ceiling --
   is that one value, and no part's fanout ladder runs the same command
   twice.
3. The ladder, on three rungs: a part's first escape at the first rung,
   then every escape at the floor; while nets stay open the route step runs
   again at each narrower rung from the board it started on, its ceiling and
   track the rung's, judged on its own passes; a narrower rung that grades no
   better ships nothing, and no rung runs once nothing is open.
4. --board-rules: every routing step is told the board's own Default class
   as --clearance with --escalation board, never the ceiling.
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
import autoroute as AP                                        # noqa: E402
from list_nets import (board_default_netclass_clearance,     # noqa: E402
                       effective_floors, read_design_rules)

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
FAILS = []


def expect(cond, what):
    print(('  ok   ' if cond else '  FAIL ') + what)
    if not cond:
        FAILS.append(what)


def test_analyze():
    print("analyze: the standard tier first, then the fab floor")
    declared = board_default_netclass_clearance(BOARD)
    a = AP.analyze(BOARD)
    floor = effective_floors(read_design_rules(BOARD).get('constraints') or {},
                             len(a['layers']))['drc_clearance']
    expect(declared and declared > floor + 1e-9,
           f"the board declares a Default class above the floor ({declared} > {floor})")
    rungs = a['size_rungs']
    std = effective_floors(read_design_rules(BOARD).get('constraints') or {},
                           len(a['layers']))['fab']['clearance']
    expect(abs(a['clearance'] - std) < 1e-9 and rungs[0]['name'] == 'standard'
           and a['clearance'] < declared - 1e-9,
           f"clearance {a['clearance']} is the standard tier {std}, not the "
           f"class {declared}")
    expect(len(rungs) == 2, f"two rungs, standard and floor ({rungs})")
    expect(abs(rungs[-1]['clearance'] - floor) < 1e-9 and rungs[-1]['name'] == 'floor',
           f"the last rung {rungs[-1]} is the fab floor {floor}")
    expect(all((n['clearance'], n['track']) != (p['clearance'], p['track'])
               and n['clearance'] <= p['clearance'] and n['track'] <= p['track']
               for p, n in zip(rungs, rungs[1:])),
           f"each rung is strictly narrower than the one before ({rungs})")
    ff = AP.analyze(BOARD, floor_first=True)
    expect(abs(ff['clearance'] - floor) < 1e-9 and len(ff['size_rungs']) == 1,
           f"--floor-first: one rung, the fab floor ({ff['size_rungs']})")


class RecordingChain(AP.ChainEvents):
    """Chain.run recorded; every fanout rung reports a dropped escape, so the
    whole ladder runs."""

    def __init__(self):
        self.cur = 'in'
        self.runs = []
        self.steps = []
        self.last_iterations = 100

    def run(self, kind, args, tag, **kw):
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
    print("run_plan at one rung (--floor-first): one clearance for every step, "
          "no repeated fanout rung")
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


def test_board_rules_and_keep_placement():
    """--board-rules: every routing step is told the board's own Default
    class as --clearance with --escalation board, never the ceiling;
    --keep-placement: the decap pass does not run. Default mode is the
    test above."""
    print("run_plan --board-rules --keep-placement: the board's rules, no part moved")
    br = AP.analyze(BOARD, board_rules=True)
    expect(br.get('board_rules') is True, "analyze records board_rules")
    expect(br['clearance'] >= AP.analyze(BOARD)['clearance'] - 1e-9,
           f"board-rules clearance {br['clearance']} is not below the fab floor's")
    a = dict(layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], clearance=0.2,
             track=0.2, via=0.6, drill=0.3, fine_via=0.6, fine_drill=0.3,
             pair_gap=0.15, pair_width=0.18, board_rules=True,
             planes=[('GND', 'In1.Cu')], plane_nets=['GND'], power_nets=['GND'],
             layer_costs=[1.0, -1.0, 1.0, 1.0], grid=0.05, congested=False,
             n_nets=10, pair_nets=['D_P', 'D_N'],
             globs={'GND': 'GND', 'D_P': 'D_P', 'D_N': 'D_N'},
             parts=[('U1', 'bga', 0.8), ('U3', 'qfn', 0.5)])
    ch = RecordingChain()
    orig = AP.Chain, AP.route_passes
    AP.Chain = lambda *args, **kw: ch
    AP.route_passes = lambda ch, a, L, costs, *r, **k: (ch, ch.cur, None)
    try:
        AP.run_plan('in', 'out', a, 'work', keep_placement=True)
    finally:
        AP.Chain, AP.route_passes = orig
    ch2 = RecordingChain()
    AP.route_passes(ch2, a, a['layers'], ['1.0', '-1.0', '1.0', '1.0'])
    runs = ch.runs + ch2.runs
    expect(not [t for k, t, _ in runs if k == 'decap' or 'decap' in t],
           "--keep-placement: no decap pass ran")
    for kind, tag, args in runs:
        if kind == 'prune':
            continue
        expect('--clearance-ceiling' not in args,
               f"{tag}: no --clearance-ceiling")
        expect(_values(args, '--clearance') == ['0.2'],
               f"{tag}: --clearance {_values(args, '--clearance')} is the board's 0.2")
        expect(_values(args, '--escalation') == ['board'],
               f"{tag}: --escalation {_values(args, '--escalation')} is board")
        if kind == 'diff':
            expect(_values(args, '--diff-pair-gap') == ['0.15']
                   and _values(args, '--track-width') == ['0.18'],
                   f"{tag}: the board's pair gap / width")


class OpenChain(RecordingChain):
    """RecordingChain whose route passes grade OPEN[i] open nets in turn."""

    def __init__(self, open_counts):
        super().__init__()
        self.open_counts = list(open_counts)
        self.inputs = []

    def run(self, kind, args, tag, **kw):
        self.inputs.append((tag, self.cur))
        return super().run(kind, args, tag, **kw)

    def grade(self, board):
        n = self.open_counts.pop(0) if self.open_counts else 0
        return {'open': n, 'drc': 0, 'open_nets': [f'N{i}' for i in range(n)]}

    def end_step(self, result=None, **extra):
        pass

    def decide(self, tag, kept, why, **extra):
        pass


def test_default_ladder():
    print("the default ladder: class first, narrower rungs while nets stay open")
    rungs = [dict(name='class', clearance=0.2, track=0.2),
             dict(name='standard', clearance=0.1, track=0.1),
             dict(name='floor', clearance=0.09, track=0.1)]
    a = dict(layers=['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu'], clearance=0.2,
             track=0.2, via=0.45, drill=0.2, fine_via=0.25, fine_drill=0.15,
             size_rungs=rungs, planes=[], plane_nets=[], power_nets=[],
             layer_costs=[1.0, 1.0, 1.0, 1.0], grid=0.05, congested=False,
             n_nets=10, pair_nets=[], globs={},
             parts=[('U1', 'bga', 0.8), ('U3', 'qfn', 0.5)])
    ch = RecordingChain()
    orig = AP.Chain, AP.route_passes
    AP.Chain = lambda *args, **kw: ch
    AP.route_passes = lambda ch, a, L, costs, *r, **k: (ch, ch.cur, None)
    try:
        AP.run_plan('in', 'out', a, 'work', keep_placement=True)
    finally:
        AP.Chain, AP.route_passes = orig
    for ref, n in (('U1', 4), ('U3', 3)):
        lad = [args for kind, tag, args in ch.runs if tag.startswith(f'fanout_{ref}')]
        cl = [_values(x, '--clearance-ceiling') for x in lad]
        expect(len(lad) == n and cl[0] == ['0.2'] and all(c == ['0.09'] for c in cl[1:]),
               f"{ref}: first escape at the class, then {n - 1} at the floor ({cl})")
    # open after every rung's first pass. The floor's pass 1 (3 open) is worse
    # than the best so far (2) but is the first at its sizes, so the floor
    # still gets its later passes; none ranks above the standard's 2.
    ch = OpenChain([5, 5, 4, 3, 2, 3, 2, 2])
    ch.cur = 'start'
    AP.route_passes(ch, a, a['layers'], ['1.0'] * 4, effort_factor=100.0)
    tags = [t for t, _ in ch.inputs]
    expect(tags == ['route', 'route_p2', 'route_standard', 'route_standard_p2',
                    'route_standard_p3', 'route_floor', 'route_floor_p2',
                    'route_floor_p3'],
           f"passes per rung, each rung judged on its own passes ({tags})")
    starts = dict(ch.inputs)
    outs = dict(zip([t for t, _ in ch.inputs], [f'out{i + 1}' for i in range(len(ch.runs))]))
    expect(starts['route_standard'] == 'start' and starts['route_floor'] == 'start',
           f"each narrower rung starts on the route step's input ({starts})")
    expect(starts['route_floor_p2'] == outs['route_floor'],
           f"a rung's pass 2 starts on its own pass 1, not on the best of the "
           f"wider rungs ({starts['route_floor_p2']})")
    for tag, c in (('route', '0.2'), ('route_standard', '0.1'), ('route_floor', '0.09')):
        args = next(x for k, t, x in ch.runs if t == tag)
        expect(_values(args, '--clearance-ceiling') == [c],
               f"{tag}: ceiling {_values(args, '--clearance-ceiling')} is {c}")
    expect(ch.cur == outs['route_standard_p3'],
           f"the best board, route_standard_p3's (2 open, the floor's 2 is no "
           f"better), is the chain's board ({ch.cur})")
    # nothing open after the class pass: no narrower rung runs
    ch = OpenChain([0])
    ch.cur = 'start'
    AP.route_passes(ch, a, a['layers'], ['1.0'] * 4)
    expect([t for t, _ in ch.inputs] == ['route'],
           f"0 open at the class: one pass ({[t for t, _ in ch.inputs]})")


def main():
    test_analyze()
    test_run_plan()
    test_default_ladder()
    test_board_rules_and_keep_placement()
    if FAILS:
        print(f"FAIL: {len(FAILS)} check(s)")
        return 1
    print("PASS: autoroute's size ladder (#2021, #1522)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
