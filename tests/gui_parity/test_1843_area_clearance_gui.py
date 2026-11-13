#!/usr/bin/env python3
"""#1843: a rule area's clearance is routed to through the GUI as through route.py.

The rule is read engine-side (`kicad_dru.install_layer_clearances`), with no
flag and no dialog control, so both fronts must reach the same copper. Both
legs route tests/fixtures/1843/room_relax (see
tests/test_1843_area_clearance_routing.py) on F.Cu with escalation off: the
CLI with route.py, the GUI with the same step as a plan through the REAL
headless dialog and plan executor (`py_router/run_plan.py`). On both,
`ROOM_NET` must pass the gap in the room (legal at the room's 0.1 mm, closed
at the class 0.2 mm), `OPEN_NET` must stay open, and check_drc must be clean.

A second leg routes tests/fixtures/1843/room_tight with the room moved onto
OPEN_NET's first pad (x 15-18): its straight route leaves the tightened room
and passes the wall's gap outside it, which is legal only for a piece that is
not in the room. Both fronts must split it at the same point, half a width
plus the split margin past the edge, and ship the same clean copper.

Needs KiCad's python (pcbnew + wx); re-execs into it automatically, and
exits 77 when no interpreter on this machine has both.

    python3 tests/gui_parity/test_1843_area_clearance_gui.py
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
BOARD = os.path.join(REPO, 'tests', 'fixtures', '1843', 'room_relax.kicad_pcb')
TIGHT = os.path.join(REPO, 'tests', 'fixtures', '1843', 'room_tight.kicad_pcb')
FAILS = []


def _reexec_into_kicad():
    """Run this gate under KiCad's own python (pcbnew + wx), found the way
    the other gates find it (`kicad_exact_fill.kicad_python_candidates`);
    exit 77 when there is none."""
    from kicad_exact_fill import kicad_python_candidates
    for cand in kicad_python_candidates():
        if cand == sys.executable or not os.path.exists(cand):
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            argv = [cand, os.path.abspath(__file__)] + sys.argv[1:]
            if os.name == 'nt':
                # os.execv re-splits argv on spaces on Windows ("Program Files")
                sys.exit(subprocess.run(argv).returncode)
            os.execv(cand, argv)
    print("SKIP: needs KiCad's python (pcbnew + wx); none found")
    sys.exit(77)


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
    except ImportError:
        _reexec_into_kicad()
    py = [sys.executable, '-X', 'utf8']
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'in.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, board],
            'copy_board')
        cli_out = os.path.join(td, 'cli.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'route.py'), board, cli_out,
                  '--nets', 'ROOM_NET', 'OPEN_NET', '--layers', 'F.Cu',
                  '--escalation', 'off'], 'route.py')
        plan = os.path.join(td, 'plan.json')
        with open(plan, 'w', encoding='utf-8') as fh:
            json.dump({'steps': [{'action': 'route',
                                  'params': {'layers': ['F.Cu'], 'escalation': 'off'},
                                  'nets': ['ROOM_NET', 'OPEN_NET']}]}, fh)
        gui_out = os.path.join(td, 'gui.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board, plan,
                  '-o', gui_out, '--timeout', '1500'], 'run_plan.py')
        cli, gui = facts(cli_out), facts(gui_out)
        print("a room at 0.1 mm in a 0.2 mm board, CLI vs the real dialog")
        ok({s[0] for s in cli} == {'ROOM_NET'},
           f"the CLI routes the net in the room only: {sorted({s[0] for s in cli})}")
        ok({s[0] for s in gui} == {'ROOM_NET'},
           f"and so does the GUI: {sorted({s[0] for s in gui})}")
        ok(cli == gui, f"with the same copper: CLI {cli} GUI {gui}")
        for name, out in (('CLI', cli_out), ('GUI', gui_out)):
            text = run(py + [os.path.join(REPO, 'py_router', 'check_drc.py'), out,
                             '--clearance-margin-mm', '0.0005'], 'check_drc')
            ok('NO DRC VIOLATIONS FOUND' in text, f"check_drc is clean on the {name} board")
        print("a track leaving a tightened room, CLI vs the real dialog")
        board = os.path.join(td, 'leave.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'copy_board.py'), TIGHT, board],
            'copy_board')
        with open(board, encoding='utf-8') as fh:
            text = fh.read()
        moved = text.replace('(xy 8 4) (xy 12 4) (xy 12 8) (xy 8 8)',
                             '(xy 15 4) (xy 18 4) (xy 18 8) (xy 15 8)')
        ok(moved != text, "the room is moved onto OPEN_NET's first pad")
        with open(board, 'w', encoding='utf-8') as fh:
            fh.write(moved)
        cli_out = os.path.join(td, 'leave_cli.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'route.py'), board, cli_out,
                  '--nets', 'ROOM_NET', 'OPEN_NET', '--layers', 'F.Cu',
                  '--escalation', 'off'], 'route.py')
        gui_out = os.path.join(td, 'leave_gui.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board, plan,
                  '-o', gui_out, '--timeout', '1500'], 'run_plan.py')
        cli, gui = facts(cli_out), facts(gui_out)
        spans = sorted((s[2], s[4]) for s in cli if s[0] == 'OPEN_NET')
        ok(spans == [(17.0, 18.085), (18.085, 23.0)],
           f"the CLI splits OPEN_NET 0.085 mm past the edge: {spans}")
        ok(cli == gui, f"and the GUI ships the same copper: CLI {cli} GUI {gui}")
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
