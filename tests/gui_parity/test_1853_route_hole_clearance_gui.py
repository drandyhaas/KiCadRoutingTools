#!/usr/bin/env python3
"""#1853: a `.kicad_dru` hole_clearance rule is routed to through the GUI as through route.py.

The rule is read engine-side (`design_rules.install_design_rules`), with no
flag and no dialog control, so both fronts must reach the same copper. Both
legs route tests/fixtures/1853/track_fits (see
tests/test_1853_route_hole_clearance.py) on F.Cu at a 0.01 mm grid with
escalation off: the CLI with route.py, the GUI with the same step as a plan
through the REAL headless dialog and plan executor (`py_router/run_plan.py`).
On both, SIG must pass the slot the rule's distance from the via's hole (the
row 0.45 mm from its centre is legal at the ring's clearance alone), with the
same copper, and check_drc must be clean.

Needs KiCad's python (pcbnew + wx): launched under any other, it re-execs
into KiCad's (`headless_plan.reexec_into_kicad`), and exits 77 only when
there is none.

    python3 tests/gui_parity/test_1853_route_hole_clearance_gui.py
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
BOARD = os.path.join(REPO, 'tests', 'fixtures', '1853', 'track_fits.kicad_pcb')
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
    return sorted((pcb.nets[s.net_id].name, s.layer, round(s.start_x, 3), round(s.start_y, 3),
                   round(s.end_x, 3), round(s.end_y, 3), round(s.width, 4))
                  for s in pcb.segments)


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
        cli_out = os.path.join(td, 'cli.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'route.py'), board, cli_out,
                  '--nets', 'SIG', '--layers', 'F.Cu', '--grid-step', '0.01',
                  '--escalation', 'off'], 'route.py')
        plan = os.path.join(td, 'plan.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump({'steps': [{'action': 'route',
                                  'params': {'layers': ['F.Cu'], 'grid_step': 0.01,
                                             'escalation': 'off'},
                                  'nets': ['SIG']}]}, fh)
        gui_out = os.path.join(td, 'gui.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board, plan,
                  '-o', gui_out, '--timeout', '1500'], 'run_plan.py')
        cli, gui = facts(cli_out), facts(gui_out)
        print("a 0.254 mm hole_clearance rule, CLI vs the real dialog")

        def reach(rows):
            from geometry_utils import point_to_segment_distance
            return min(point_to_segment_distance(15.0, 6.0, s[2], s[3], s[4], s[5])
                       for s in rows if s[0] == 'SIG')
        need = 0.15 + 0.254 + 0.05
        ok(any(s[0] == 'SIG' for s in cli) and reach(cli) >= need - 1e-6,
           f"the CLI routes SIG {reach(cli):.3f} mm from the hole's centre (needs {need:.3f})")
        ok(any(s[0] == 'SIG' for s in gui) and reach(gui) >= need - 1e-6,
           f"and so does the GUI: {reach(gui):.3f} mm")
        ok(cli == gui, f"with the same copper: CLI {cli} GUI {gui}")
        for name, out in (('CLI', cli_out), ('GUI', gui_out)):
            text = run(py + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                             '--clearance-margin-mm', '0.0005'], 'check_drc')
            ok('NO DRC VIOLATIONS FOUND' in text, f"check_drc is clean on the {name} board")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
