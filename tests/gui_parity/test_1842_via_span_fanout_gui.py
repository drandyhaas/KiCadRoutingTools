#!/usr/bin/env python3
"""#1842: a via span fans a BGA out the same through the GUI as through
bga_fanout.py.

The fan-out tab reads the route tab's Via Span controls and hands them to the
same engine `bga_fanout.py --via-span` calls. Both legs fan out U1 of
tests/fixtures/1842/bga_under (see tests/test_1842_via_span_fanout.py) with
via span F.Cu-In1.Cu at the board's clearance and track width: the CLI with
bga_fanout.py, the GUI with the same step as a plan through the REAL headless
dialog and plan executor (`py_router/run_plan.py`). They must agree on: the
vias (count, type, layer pair, size, drill), the layers the tracks are on,
how many nets got copper, and the output project's via and drilled-hole
minimums, which must be the board's own.

Needs KiCad's python (pcbnew + wx): exit 77 without it.

    <KiCad python> tests/gui_parity/test_1842_via_span_fanout_gui.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
BOARD = os.path.join(REPO, 'tests', 'fixtures', '1842', 'bga_under.kicad_pcb')
KEYS = ('min_via_diameter', 'min_through_hole_diameter', 'min_microvia_diameter',
        'min_microvia_drill', 'min_via_annular_width')
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
    with open(os.path.splitext(path)[0] + '.kicad_pro', encoding='utf-8') as fh:
        rules = json.load(fh)['board']['design_settings']['rules']
    vias = sorted({(v.via_type, tuple(v.layers), round(v.size, 4), round(v.drill, 4))
                   for v in pcb.vias})
    return {'vias': len(pcb.vias), 'kinds': vias,
            'layers': sorted({s.layer for s in pcb.segments}),
            'nets': len({s.net_id for s in pcb.segments}),
            'rules': {k: rules.get(k) for k in KEYS}}


def main():
    try:
        import pcbnew  # noqa: F401
        import wx      # noqa: F401
    except Exception as e:                                     # noqa: BLE001
        print(f"SKIP: needs KiCad's python (pcbnew + wx): {e}")
        return 77
    py = [sys.executable, '-X', 'utf8']
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'in.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, board],
            'copy_board')
        with open(os.path.splitext(BOARD)[0] + '.kicad_pro', encoding='utf-8') as fh:
            own = json.load(fh)['board']['design_settings']['rules']

        cli_out = os.path.join(td, 'cli.kicad_pcb')
        cli_log = run(py + [os.path.join(REPO, 'py_router', 'bga_fanout.py'), board,
                            '-o', cli_out, '--component', 'U1', '--clearance', '0.1',
                            '--track-width', '0.1', '--via-span', 'F.Cu', 'In1.Cu'],
                      'bga_fanout.py')
        plan = os.path.join(td, 'plan.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump({'steps': [{'action': 'fanout', 'component': 'U1',
                                  'params': {'via_span': ['F.Cu', 'In1.Cu'],
                                             'clearance': 0.1, 'track_width': 0.1}}]}, fh)
        gui_out = os.path.join(td, 'gui.kicad_pcb')
        gui_log = run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board, plan,
                            '-o', gui_out, '--timeout', '1500'], 'run_plan.py')

        cli, gui = facts(cli_out), facts(gui_out)
        print("fan-out with via span F.Cu-In1.Cu, CLI vs the real dialog")
        ok('Via span (#1842): micro vias F.Cu-In1.Cu' in cli_log
           and 'Via span (#1842): micro vias F.Cu-In1.Cu' in gui_log,
           "both runs resolve the span")
        ok(cli['kinds'] == gui['kinds'] == [('micro', ('F.Cu', 'In1.Cu'), 0.25, 0.1)],
           f"the vias are the same kind: CLI {cli['kinds']} GUI {gui['kinds']}")
        ok(cli['vias'] == gui['vias'] == 12,
           f"the same number of vias: CLI {cli['vias']} GUI {gui['vias']}")
        ok(cli['layers'] == gui['layers'] == ['F.Cu', 'In1.Cu'],
           f"copper on the span only: CLI {cli['layers']} GUI {gui['layers']}")
        ok(cli['nets'] == gui['nets'] == 36,
           f"every ball escaped on both: CLI {cli['nets']} GUI {gui['nets']} nets with copper")
        ok(cli['rules'] == gui['rules'], f"the same project minimums: {gui['rules']}")
        kept = ('min_via_diameter', 'min_through_hole_diameter', 'min_microvia_diameter',
                'min_microvia_drill')
        ok(all(gui['rules'][k] == own.get(k) for k in kept),
           "and the board's own via and drilled-hole minimums are kept")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
