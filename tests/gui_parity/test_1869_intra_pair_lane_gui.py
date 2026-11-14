#!/usr/bin/env python3
"""#1869: the intra-pair lane reaches the engine through the GUI as through
route_diff.py.

`--diff-pair-intra-lane` is a new engine parameter, `diff_pair_intra_lane`,
so it must be threaded through both fronts. Both legs run the pair step on
the bundled lvds_converter_dualclk for `/DATA+` / `/DATA-` with intra-pair
matching and `auto` lanes: the CLI with route_diff.py, the GUI with the same
step as a plan through the REAL headless dialog and plan executor
(`py_router/run_plan.py`), the plan step carrying the parameter by its own
name. The CLI's output must record the lane's pads with the pair's
tolerance (IC4.10 / J1.5 and IC4.9 / J1.4). The GUI leg is read off the
engine's own console line, `Intra-pair lane (#1869): N pair(s) ...`, printed
by the shared engine only when the parameter reached it: present with the
parameter, absent without. (`run_plan.py -o` writes a board that carries no
`kicad_routing_tools` record at all, lane or not, so the GUI leg cannot be
read off the output's records.)

Needs KiCad's python (pcbnew + wx): exit 77 without it.

    <KiCad python> tests/gui_parity/test_1869_intra_pair_lane_gui.py
"""
import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, 'py_router'))
BOARD = os.path.join(REPO, 'kicad_files', 'lvds_converter_dualclk.kicad_pcb')
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


def lane(board):
    """The recorded lane of /DATA+ and /DATA-, read as the graders read it
    (`kicad_dru.recorded_intra_pair_lanes`: the project or the board's own
    record, whichever is newer -- the GUI writes the board's)."""
    from kicad_dru import recorded_intra_pair_lanes
    got = recorded_intra_pair_lanes(board)
    if '/DATA+' not in got:
        return None
    return {'p': sorted(got['/DATA+']), 'n': sorted(got.get('/DATA-') or [])}


def main():
    try:
        import pcbnew  # noqa: F401
        import wx      # noqa: F401
    except Exception as e:                                     # noqa: BLE001
        print(f"SKIP: needs KiCad's python (pcbnew + wx): {e}")
        return 77
    py = [sys.executable, '-X', 'utf8']
    want = {'p': ['IC4.10', 'J1.5'], 'n': ['IC4.9', 'J1.4']}
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'in.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'copy_board.py'), BOARD, board],
            'copy_board')
        cli_out = os.path.join(td, 'cli.kicad_pcb')
        run(py + [os.path.join(REPO, 'py_router', 'route_diff.py'), board, cli_out,
                  '--nets', '/DATA+', '/DATA-', '--diff-pair-intra-match',
                  '--length-match-tolerance', '0.15', '--force-reroute',
                  '--diff-pair-intra-lane', 'auto'], 'route_diff.py')
        outs = {}
        for tag, extra in (('gui', {'diff_pair_intra_lane': ['auto']}), ('gui_off', {})):
            plan = os.path.join(td, tag + '_plan.json')
            with open(plan, 'w', encoding='utf-8') as fh:
                json.dump({'steps': [{'action': 'route_diff',
                                      'params': dict({'diff_pair_intra_match': True,
                                                      'length_match_tolerance': 0.15,
                                                      'force_reroute': True}, **extra),
                                      'nets': ['/DATA+', '/DATA-']}]}, fh)
            outs[tag] = run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board,
                                  plan, '-o', os.path.join(td, tag + '.kicad_pcb'),
                                  '--timeout', '1500'], 'run_plan.py')
        print("an auto intra-pair lane, CLI vs the real dialog")
        ok(lane(cli_out) == want, f"the CLI records the lane's pads: {lane(cli_out)}")
        line = 'Intra-pair lane (#1869):'
        ok("set diff_pair_intra_lane=['auto']" in outs['gui'] and line in outs['gui']
           and 'auto: the two terminals the pair step routes' in outs['gui'],
           "the plan sets the control by the parameter's name and the engine takes the lane")
        ok(line not in outs['gui_off'],
           "with the parameter left out the engine takes none")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
