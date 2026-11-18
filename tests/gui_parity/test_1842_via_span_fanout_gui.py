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
minimums, which must be the board's own. A third leg runs the plan in process
and reads those minimums off the LIVE board: the saved project cannot show
them, since the headless save skips the settings and the project is written
from the file's own copper, while KiCad writes the live settings at the
user's next save.

Needs KiCad's python (pcbnew + wx): launched under any other, it re-execs
into KiCad's (`headless_plan.reexec_into_kicad`), and exits 77 only when
there is none.

    python3 tests/gui_parity/test_1842_via_span_fanout_gui.py
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
        'min_microvia_drill', 'min_via_annular_width', 'min_hole_clearance')
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


def _read(path):
    try:
        with open(path, encoding='utf-8') as fh:
            return fh.read()
    except OSError:
        return None


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
            'rules': {k: rules.get(k) for k in KEYS},
            # #1842: the micro vias' own rule (fix_kicad_drc_settings.MICRO_VIA_RULE)
            'dru': _read(os.path.splitext(path)[0] + '.kicad_dru')}


def main():
    try:
        import pcbnew  # noqa: F401
        import wx      # noqa: F401
    except Exception as e:                                     # noqa: BLE001
        from headless_plan import reexec_into_kicad
        reexec_into_kicad(__file__)
        print(f"SKIP: needs KiCad's python (pcbnew + wx), and none was found: {e}")
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
        ok(cli['dru'] is not None and cli['dru'] == gui['dru']
           and "(condition \"A.Via_Type == 'Micro'\")" in cli['dru'],
           f"the same micro via rule in the rules file (#1842): CLI {cli['dru']!r} "
           f"GUI {gui['dru']!r}")
        kept = ('min_via_diameter', 'min_through_hole_diameter', 'min_microvia_diameter',
                'min_microvia_drill', 'min_via_annular_width', 'min_hole_clearance')
        ok(all(gui['rules'][k] == own.get(k) for k in kept),
           "and the board's own via and drilled-hole minimums are kept")

        # The live board: what KiCad saves into the project at the user's save.
        # A micro span's via is no floor for the board's through vias and holes.
        from copy_board import copy_board
        from headless_plan import run_plan
        import pcbnew
        live_path = os.path.join(td, 'live.kicad_pcb')
        copy_board(board, live_path)
        with contextlib.redirect_stdout(io.StringIO()):
            res = run_plan(live_path, json.load(open(plan, encoding='utf-8'))['steps'],
                           timeout=1500, save=False)
        if [st.get('status') for st in res.get('steps') or ()] != ['done']:
            raise AssertionError(f"BROKEN TEST: the in-process plan did not run: "
                                 f"{res.get('steps')} {res.get('log')}")
        ds = pcbnew.GetBoard().GetDesignSettings()
        live = {'min_via_diameter': round(ds.m_ViasMinSize / 1e6, 4),
                'min_through_hole_diameter': round(ds.m_MinThroughDrill / 1e6, 4)}
        ok(all(live[k] == own.get(k) for k in live),
           f"the live board keeps them too: live {live}, board's own "
           f"{ {k: own.get(k) for k in live} }")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
