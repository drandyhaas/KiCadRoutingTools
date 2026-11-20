#!/usr/bin/env python3
"""#1842: via kinds and jog-only layers route the same through the GUI as
through route.py.

The route tab's Via Kinds, Jog Layers and Max Jog controls reach the engine
as `--via-kinds`, `--jog-layers` and `--jog-max-length` do. Both legs route
the mixed-kind fixture (tests/fixtures/1842/mixed_kinds.py) with one step on
`/K*`: the CLI with route.py and the three flags, the GUI with the same step
as a plan through the REAL headless dialog and plan executor
(`py_router/run_plan.py`). They must agree on: every net routed, the vias
(count, and each kind's type, layer pair, size and drill), the track count,
and the longest run of track on a jog-only layer, which must be within the
cap on both.

Needs KiCad's python (pcbnew + wx): launched under any other, it re-execs
into KiCad's (`headless_plan.reexec_into_kicad`), and exits 77 only when
there is none.

    python3 tests/gui_parity/test_1842_via_kinds_gui.py
"""
import collections
import contextlib
import io
import json
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'tests', 'fixtures', '1842'))
STACK = ['F.Cu', 'In1.Cu', 'In2.Cu', 'In3.Cu', 'In4.Cu', 'In5.Cu', 'In6.Cu', 'B.Cu']
KINDS = ['F.Cu', 'In1.Cu', 'In1.Cu', 'In6.Cu', 'In6.Cu', 'B.Cu']
JOG = ['In1.Cu', 'In6.Cu']
CAP = 1.5
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def run(argv, what):
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO, timeout=1800)
    text = (r.stdout or '') + (r.stderr or '')
    if r.returncode != 0 or 'Traceback' in text:
        raise AssertionError(f"BROKEN TEST: {what} did not run (exit {r.returncode})\n"
                             f"{text[-1500:]}")
    return text


def facts(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    kinds = collections.Counter((v.via_type, tuple(v.layers), round(v.size, 4),
                                 round(v.drill, 4)) for v in pcb.vias)
    longest = 0.0
    for layer in JOG:
        segs = [s for s in pcb.segments if s.layer == layer]
        parent = list(range(len(segs)))

        def find(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i
        ends = collections.defaultdict(list)
        for i, s in enumerate(segs):
            for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
                ends[(s.net_id, round(x, 4), round(y, 4))].append(i)
        for group in ends.values():
            for i in group[1:]:
                parent[find(i)] = find(group[0])
        total = collections.Counter()
        for i, s in enumerate(segs):
            total[find(i)] += math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
        longest = max([longest] + list(total.values()))
    return {'vias': len(pcb.vias), 'kinds': dict(kinds), 'segments': len(pcb.segments),
            'longest_jog': round(longest, 3)}


def main():
    try:
        import pcbnew  # noqa: F401
        import wx      # noqa: F401
    except Exception as e:                                     # noqa: BLE001
        from headless_plan import reexec_into_kicad
        reexec_into_kicad(__file__)
        print(f"SKIP: needs KiCad's python (pcbnew + wx), and none was found: {e}")
        return 77
    import mixed_kinds
    py = [sys.executable, '-X', 'utf8']
    with tempfile.TemporaryDirectory() as td:
        board = mixed_kinds.write(os.path.join(td, 'in.kicad_pcb'))
        cli_out = os.path.join(td, 'cli.kicad_pcb')
        cli_log = run(py + [os.path.join(REPO, 'py_router', 'route.py'), board, cli_out,
                            '--nets', '/K*', '--layers'] + STACK + ['--via-kinds'] + KINDS
                      + ['--jog-layers'] + JOG + ['--jog-max-length', str(CAP)], 'route.py')
        plan = os.path.join(td, 'plan.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump({'steps': [{'action': 'route',
                                  'params': {'layers': STACK, 'via_kinds': KINDS,
                                             'jog_layers': JOG, 'jog_max_length': CAP},
                                  'nets': ['/K*']}]}, fh)
        gui_out = os.path.join(td, 'gui.kicad_pcb')
        gui_log = run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board, plan,
                            '-o', gui_out, '--timeout', '1500'], 'run_plan.py')

        cli, gui = facts(cli_out), facts(gui_out)
        print("via kinds + jog-only layers, CLI vs the real dialog")
        ok('4/4 routed' in cli_log and '4/4 routed' in gui_log, "both fronts route every net")
        said = 'Via kinds (#1842): the search changes layers through micro F.Cu-In1.Cu 0.25/0.1'
        ok(said in cli_log and said in gui_log
           and f'a jog of at most {CAP:g} mm between two vias' in gui_log,
           "the GUI run declares the kinds and the cap, as the engine says")
        ok(cli['kinds'] == gui['kinds'] and len(cli['kinds']) == 3
           and {k[0] for k in cli['kinds']} == {'micro', 'buried'},
           f"the same vias of each kind: CLI {cli['kinds']} GUI {gui['kinds']}")
        ok(cli['vias'] == gui['vias'] and cli['segments'] == gui['segments'],
           f"the same copper: CLI {cli['vias']} vias / {cli['segments']} tracks, "
           f"GUI {gui['vias']} / {gui['segments']}")
        ok(0 < cli['longest_jog'] <= CAP + 0.02 and cli['longest_jog'] == gui['longest_jog'],
           f"the longest run on a jog-only layer: CLI {cli['longest_jog']} GUI "
           f"{gui['longest_jog']} mm (cap {CAP})")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
