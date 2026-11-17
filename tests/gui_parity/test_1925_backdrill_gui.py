#!/usr/bin/env python3
"""#1925: back-drills are defined the same through the GUI as through route.py.

The route tab's Back-drill Nets field and the three numbers beside it reach
`batch_route` as `--backdrill-nets` and its three flags do. Both legs route
the two-via fixture of tests/test_1925_backdrill_spacing.py (two F.Cu pads of
/S with a wall between them, routed on F.Cu and In1.Cu) with one step: the
CLI with route.py, the GUI with the same step as a plan through the REAL
headless dialog and plan executor (`py_router/run_plan.py`). They must agree
on the vias and their back-drills, and the vias the plugin lays on the live
board must carry them through pcbnew's own save. A second pair runs the step
with the fields empty and must write no back-drill on either front.

Needs KiCad's python (pcbnew + wx): launched under any other, it re-execs
into KiCad's (`headless_plan.reexec_into_kicad`), and exits 77 only when
there is none.

    python3 tests/gui_parity/test_1925_backdrill_gui.py
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
sys.path.insert(0, os.path.join(REPO, 'tests'))
FAILS = []
RUN = {'layers': ['F.Cu', 'In1.Cu'], 'clearance': 0.1, 'track_width': 0.2,
       'via_size': 0.6, 'via_drill': 0.3, 'grid_step': 0.05}
ASK = {'backdrill_nets': ['/S'], 'backdrill_oversize': 0.3, 'backdrill_margin': 0.2,
       'backdrill_min_stub': 0.25}


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


def flags(params):
    out = []
    for k, v in params.items():
        out.append('--' + k.replace('_', '-'))
        out += [str(x) for x in (v if isinstance(v, list) else [v])]
    return out


def facts(path):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    return sorted((round(v.size, 4), round(v.drill, 4),
                   tuple(sorted((t, tuple(bd)) for t, bd in v.backdrills.items())))
                  for v in pcb.vias)


def main():
    try:
        import pcbnew  # noqa: F401
        import wx      # noqa: F401
    except Exception as e:                                     # noqa: BLE001
        from headless_plan import reexec_into_kicad
        reexec_into_kicad(__file__)
        print(f"SKIP: needs KiCad's python (pcbnew + wx), and none was found: {e}")
        return 77
    from backdrill_fixture import write_board
    py = [sys.executable, '-X', 'utf8']
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'in.kicad_pcb')
        write_board(board, pads=[('P1', '/S', 3, 5, 'F.Cu', 1.0),
                                 ('P2', '/S', 17, 5, 'F.Cu', 1.0)],
                    tracks=[('/W', 10, 0, 10, 10, 'F.Cu', 0.3)])
        # a project of its own: a board without one makes the fronts differ
        with open(os.path.join(REPO, 'kicad_files', 'flat_hierarchy.kicad_pro'),
                  encoding='utf-8') as fh:
            pro = json.load(fh)
        pro['board']['design_settings']['rules'].update(
            min_clearance=0.1, min_track_width=0.1, min_via_diameter=0.4,
            min_through_hole_diameter=0.2, min_copper_edge_clearance=0.2)
        pro['net_settings']['classes'][0].update(clearance=0.1, track_width=0.2,
                                                 via_diameter=0.6, via_drill=0.3)
        with open(os.path.join(td, 'in.kicad_pro'), 'w', encoding='utf-8') as fh:
            json.dump(pro, fh)

        want = [(0.6, 0.3, (('backdrill', (0.6, 'B.Cu', 'In2.Cu')),))] * 2
        for label, ask, expect in (('asked', ASK, want),
                                   ('not asked', {}, [(0.6, 0.3, ())] * 2)):
            print(f"back-drills {label}, CLI vs the real dialog")
            tag = label.replace(' ', '_')
            cli_out = os.path.join(td, f'cli_{tag}.kicad_pcb')
            run(py + [os.path.join(REPO, 'py_router', 'route.py'), board, cli_out,
                      '--nets', '/S'] + flags({**RUN, **ask}), 'route.py')
            plan = os.path.join(td, f'plan_{tag}.json')
            with open(plan, 'w', encoding='utf-8') as fh:
                json.dump({'steps': [{'action': 'route', 'params': {**RUN, **ask},
                                      'nets': ['/S']}]}, fh)
            gui_out = os.path.join(td, f'gui_{tag}.kicad_pcb')
            gui_log = run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board, plan,
                                '-o', gui_out, '--timeout', '1500'], 'run_plan.py')
            cli, gui = facts(cli_out), facts(gui_out)
            ok(cli == expect, f"route.py: {cli}")
            ok(gui == expect, f"the dialog, saved by pcbnew: {gui}")
            ok(('BACK-DRILL (route): 2 of 2' in gui_log) == bool(ask),
               "the GUI run reports the back-drills it decided"
               if ask else "the GUI run decides none")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
