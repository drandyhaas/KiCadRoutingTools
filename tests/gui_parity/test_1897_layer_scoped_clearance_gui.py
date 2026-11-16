#!/usr/bin/env python3
"""#1897: a class-scoped clearance rule with a `(layer ...)` clause is routed to
through the GUI as through route.py.

The rule is read engine-side (the scoped channel, `kicad_dru`), with no flag
and no dialog control, so both fronts must reach the same copper. Both legs
route tests/fixtures/1897/wall_relax (see
tests/test_1897_layer_scoped_clearance.py) on In1.Cu at a 0.01 mm grid with
escalation off: the CLI with route.py, the GUI with the same step as a plan
through the REAL headless dialog and plan executor (`py_router/run_plan.py`).
On both, A1 must pass the slot, which is open only at the 0.1 mm the rule
gives In1.Cu (the classes ask 0.3 mm), with the same copper, and check_drc
must be clean.

ipc-migration: the dialog runs over the fake IPC board
(`tests/gui_parity/fake_ipc_board.py`, which `headless_plan.run_plan`
installs), as an UNSAVED document (`fake_ipc_board.UNSAVED_ENV`): the copper
the steps commit lives in a copy, and the board file the writebacks read
keeps the copper from before the run until the run ends, as in a real KiCad.

Needs KiCad's python (wx + kipy on this branch); re-execs into it, and exits
77 only when no interpreter has both.

    <KiCad python> tests/gui_parity/test_1897_layer_scoped_clearance_gui.py
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
BOARD = os.path.join(REPO, 'tests', 'fixtures', '1897', 'wall_relax.kicad_pcb')
FAILS = []


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


# ipc-migration: `run_plan.py` run as a child installs the fake IPC board, as
# an UNSAVED document (`fake_ipc_board.UNSAVED_ENV`), as in a real KiCad: the
# file the writebacks read holds the copper from before the step, which the
# default harness, flushing every commit to it, hides.
PLAN_ENV = dict(os.environ, KRT_FAKE_IPC_UNSAVED='1')


def run(argv, what, verdict=None):
    """Run a step; BROKEN TEST unless it ran. ``verdict`` ({exit code: the
    line that exit prints}) admits a non-zero exit that IS an answer:
    check_drc exits 1 when it finds violations, which the caller's own check
    grades as a FAIL, not as a step that did not run."""
    r = subprocess.run(argv, capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=REPO, timeout=1800,
                       env=PLAN_ENV if any(a.endswith('run_plan.py') for a in argv)
                       else None)
    text = (r.stdout or '') + (r.stderr or '')
    answered = r.returncode in (verdict or {}) and verdict[r.returncode] in text
    if (r.returncode != 0 and not answered) or 'Traceback' in text:
        raise AssertionError(f"BROKEN TEST: {what} did not run (exit {r.returncode})\n"
                             f"{text[-1500:]}")
    return text


# check_drc's two answers: clean (exit 0) and violations found (exit 1)
DRC_VERDICT = {0: 'NO DRC VIOLATIONS FOUND', 1: 'DRC VIOLATIONS:'}


def facts(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    return sorted((pcb.nets[s.net_id].name, s.layer, round(s.start_x, 3), round(s.start_y, 3),
                   round(s.end_x, 3), round(s.end_y, 3), round(s.width, 4))
                  for s in pcb.segments)


def main():
    try:
        import wx      # noqa: F401
        import kipy    # noqa: F401
    except Exception as e:                                     # noqa: BLE001
        # ipc-migration: the gate used to SKIP here, never re-execing, so it
        # tested nothing under plain python3; it re-execs as the siblings do.
        from headless_plan import reexec_into_kicad
        reexec_into_kicad(__file__)
        print(f"SKIP: needs KiCad's python (wx + kipy), and none was found: {e}")
        return 77
    py = [sys.executable, '-X', 'utf8']
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'in.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, board],
            'copy_board')
        cli_out = os.path.join(td, 'cli.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'route.py'), board, cli_out,
                  '--nets', 'A1', '--layers', 'In1.Cu', '--grid-step', '0.01',
                  '--escalation', 'off'], 'route.py')
        plan = os.path.join(td, 'plan.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump({'steps': [{'action': 'route',
                                  'params': {'layers': ['In1.Cu'], 'grid_step': 0.01,
                                             'escalation': 'off'},
                                  'nets': ['A1']}]}, fh)
        gui_out = os.path.join(td, 'gui.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board, plan,
                  '-o', gui_out, '--timeout', '1500'], 'run_plan.py')
        cli, gui = facts(cli_out), facts(gui_out)
        print("a 0.1 mm rule on In1.Cu in a 0.3 mm board, CLI vs the real dialog")
        ok(any(s[0] == 'A1' and s[1] == 'In1.Cu' for s in cli),
           f"the CLI routes A1 through the slot on In1.Cu ({len(cli)} segment(s))")
        ok(any(s[0] == 'A1' and s[1] == 'In1.Cu' for s in gui),
           f"and so does the GUI ({len(gui)} segment(s))")
        ok(cli == gui, f"with the same copper: CLI {cli} GUI {gui}")
        for name, out in (('CLI', cli_out), ('GUI', gui_out)):
            text = run(py + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                             '--clearance-margin-mm', '0.0005'], 'check_drc', DRC_VERDICT)
            ok('NO DRC VIOLATIONS FOUND' in text, f"check_drc is clean on the {name} board")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
