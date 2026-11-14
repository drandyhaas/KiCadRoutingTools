#!/usr/bin/env python3
"""#1842: a via span routes the same through the GUI as through route.py.

The route tab's Via Span control reaches the engine as `--via-span` does, but
three things a span changes used to live on the CLI's side of the engine call
and differed in the GUI: the micro via a span draws when no via size is asked
for (the GUI told the engine to draw each net's CLASS via, a through via's
size), the project writeback (the live board's scan lowered
`min_via_diameter` to the micro vias' size), and an explicit via on a micro
span, which route.py leaves off the mechanical fab floor and the GUI's via
controls pinned to it (both now read `via_span.laser_drilled`).

Both legs route tests/fixtures/1842 with one step, via span F.Cu-In1.Cu on
`S*`: the CLI with `route.py --via-span F.Cu In1.Cu`, the GUI with the same
step as a plan through the REAL headless dialog and plan executor
(`py_router/run_plan.py`). They must agree on: every net routed, the vias
(count, type, layer pair, size, drill), the track count, and the output
project's via and drilled-hole minimums, which must be the board's own. A
second pair gives both the via 0.25/0.1 explicitly and must lay it as asked.

Needs KiCad's python (pcbnew + wx): exit 77 without it.

    <KiCad python> tests/gui_parity/test_1842_via_span_gui.py
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
BOARD = os.path.join(REPO, 'tests', 'fixtures', '1842', 'hdi_repro.kicad_pcb')
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
    return {'vias': len(pcb.vias), 'kinds': vias, 'segments': len(pcb.segments),
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
        cli_log = run(py + [os.path.join(REPO, 'py_router', 'route.py'), board, cli_out,
                            '--nets', 'S*', '--via-span', 'F.Cu', 'In1.Cu'], 'route.py')
        plan = os.path.join(td, 'plan.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump({'steps': [{'action': 'route',
                                  'params': {'via_span': ['F.Cu', 'In1.Cu']},
                                  'nets': ['S*']}]}, fh)
        gui_out = os.path.join(td, 'gui.kicad_pcb')
        gui_log = run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board, plan,
                            '-o', gui_out, '--timeout', '1500'], 'run_plan.py')

        cli, gui = facts(cli_out), facts(gui_out)
        print("via span F.Cu-In1.Cu, CLI vs the real dialog")
        ok('36/36 routed' in cli_log and '36/36 routed' in gui_log,
           "both fronts route every net")
        ok('micro via span: drawing the board Default net-class micro via 0.25/0.1'
           in gui_log, "the GUI run draws the class micro via, as the engine says")
        ok(cli['kinds'] == gui['kinds'] == [('micro', ('F.Cu', 'In1.Cu'), 0.25, 0.1)],
           f"the vias are the same kind: CLI {cli['kinds']} GUI {gui['kinds']}")
        ok(cli['vias'] == gui['vias'] and cli['segments'] == gui['segments'],
           f"the same copper: CLI {cli['vias']} vias / {cli['segments']} tracks, "
           f"GUI {gui['vias']} / {gui['segments']}")
        ok(cli['rules'] == gui['rules'], f"the same project minimums: {gui['rules']}")
        kept = ('min_via_diameter', 'min_through_hole_diameter', 'min_microvia_diameter',
                'min_microvia_drill')
        ok(all(gui['rules'][k] == own.get(k) for k in kept),
           "and the board's own via and drilled-hole minimums are kept")

        # An explicit via on a micro span: route.py passes the fab floor no via
        # for a laser-drilled span, and the GUI's via controls read the same
        # via_span.laser_drilled, so neither pins the 0.1 drill up to 0.15.
        print("explicit via 0.25/0.1 on the span, CLI vs the real dialog")
        cli_out2 = os.path.join(td, 'cli2.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'route.py'), board, cli_out2,
                  '--nets', 'S*', '--via-span', 'F.Cu', 'In1.Cu',
                  '--via-size', '0.25', '--via-drill', '0.1'], 'route.py (explicit via)')
        plan2 = os.path.join(td, 'plan2.json')
        with open(plan2, 'w', encoding='utf-8') as fh:
            json.dump({'steps': [{'action': 'route',
                                  'params': {'via_span': ['F.Cu', 'In1.Cu'],
                                             'via_size': 0.25, 'via_drill': 0.1},
                                  'nets': ['S*']}]}, fh)
        gui_out2 = os.path.join(td, 'gui2.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board, plan2,
                  '-o', gui_out2, '--timeout', '1500'], 'run_plan.py (explicit via)')
        cli2, gui2 = facts(cli_out2), facts(gui_out2)
        ok(cli2['kinds'] == gui2['kinds'] == [('micro', ('F.Cu', 'In1.Cu'), 0.25, 0.1)],
           f"both lay the asked micro via: CLI {cli2['kinds']} GUI {gui2['kinds']}")
        ok(cli2['vias'] == gui2['vias'] and cli2['segments'] == gui2['segments'],
           f"the same copper: CLI {cli2['vias']} vias / {cli2['segments']} tracks, "
           f"GUI {gui2['vias']} / {gui2['segments']}")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
