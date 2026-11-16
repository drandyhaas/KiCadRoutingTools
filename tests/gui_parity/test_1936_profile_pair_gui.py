#!/usr/bin/env python3
"""#1936: the Differential tab couples a pair at its tuning profile's width
and gap as route_diff.py does.

The profile geometry is resolved inside the shared engine
(`batch_route_diff_pairs`) from the project next to the board, with no flag
and no control, so the GUI gets it when the tab leaves its Width and Gap
overrides unchecked. Both legs run the pair step on cap_chain staged with a
class that names a differential profile (F.Cu 0.15 mm / gap 0.3 mm, B.Cu
0.2 mm / gap 0.35 mm), F.Cu priced up so the pairs run on both layers: the
CLI with route_diff.py, the GUI with the same step as a plan through the REAL
headless dialog and plan executor (`py_router/run_plan.py`).

  - both fronts print the engine's `Tuning-profile pair geometry:` line with
    the same layers, widths and gaps;
  - both boards carry the same track widths on each layer, the profile's
    among them;
  - with the plan setting an explicit pair width the engine takes no profile
    (the tab's override reaches it).

Needs KiCad's python (pcbnew + wx). Run under any python, it re-execs into
the first KiCad interpreter `kicad_locate.kicad_python_candidates` finds
that imports both; exit 77 only when there is none.

    python3 tests/gui_parity/test_1936_profile_pair_gui.py
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
LINE = 'Tuning-profile pair geometry:'
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


def widths(board):
    """{layer: sorted track widths} of the routed pair nets."""
    from kicad_parser import parse_kicad_pcb
    from run_utils import evidence
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(evidence(board))
    out = {}
    for s in pcb.segments:
        if s.net_id:
            out.setdefault(s.layer, set()).add(round(s.width, 3))
    return {k: sorted(v) for k, v in out.items()}


def line(text):
    return next((ln.strip() for ln in text.splitlines() if LINE in ln), '')


def _reexec_into_kicad(why):
    """Run this gate under KiCad's python, as the other gates do; 77 when no
    interpreter on this machine has pcbnew and wx."""
    from kicad_locate import kicad_python_candidates
    here = os.path.realpath(sys.executable)
    for cand in kicad_python_candidates():
        if not os.path.exists(cand) or os.path.realpath(cand) == here:
            continue
        if subprocess.run([cand, '-c', 'import pcbnew, wx'],
                          capture_output=True).returncode == 0:
            print(f"(re-running under KiCad's python: {cand})", flush=True)
            return subprocess.run([cand, os.path.abspath(__file__)]
                                  + sys.argv[1:]).returncode
    print(f"SKIP: needs KiCad's python (pcbnew + wx), and none was found: {why}")
    return 77


def main():
    try:
        import pcbnew  # noqa: F401
        import wx      # noqa: F401
    except Exception as e:                                     # noqa: BLE001
        return _reexec_into_kicad(e)
    import test_1936_tuning_profile_pair_widths as t
    py = [sys.executable, '-X', 'utf8']
    nets = ['DPA_P', 'DPA_N', 'DPB_P', 'DPB_N']
    with tempfile.TemporaryDirectory() as td:
        board, cli_out = t.stage(td, 'g', t.project(
            [t.entry('F.Cu', 0.15, 0.3), t.entry('B.Cu', 0.2, 0.35)]))
        sub = os.path.dirname(board)
        cli_log = run(py + [os.path.join(REPO, 'py_router', 'route_diff.py'), board,
                            cli_out, '--nets'] + nets
                      + ['--layers', 'F.Cu', 'B.Cu', '--layer-costs', '20', '1'],
                      'route_diff.py')
        logs = {}
        for tag, extra in (('gui', {}), ('gui_width', {'diff_pair_width': 0.22})):
            plan = os.path.join(sub, tag + '_plan.json')
            with open(plan, 'w', encoding='utf-8') as fh:
                json.dump({'steps': [{'action': 'route_diff',
                                      'params': dict({'layers': ['F.Cu', 'B.Cu'],
                                                      'layer_costs': [20, 1]}, **extra),
                                      'nets': nets}]}, fh)
            logs[tag] = run(py + [os.path.join(REPO, 'py_router', 'run_plan.py'), board,
                                  plan, '-o', os.path.join(sub, tag + '.kicad_pcb'),
                                  '--timeout', '1500'], 'run_plan.py')
        print("a differential tuning profile, CLI vs the real dialog")
        ok('F.Cu=0.15mm/gap 0.3mm, B.Cu=0.2mm/gap 0.35mm' in line(cli_log),
           f"the CLI takes the profile: {line(cli_log)}")
        ok(line(logs['gui']) == line(cli_log) != '',
           f"the tab's engine call takes the same geometry: {line(logs['gui'])}")
        w_cli, w_gui = widths(cli_out), widths(os.path.join(sub, 'gui.kicad_pcb'))
        ok(w_cli == w_gui and 0.2 in w_cli.get('B.Cu', []) and 0.15 in w_cli.get('F.Cu', []),
           f"both boards carry the same widths per layer: CLI {w_cli}, GUI {w_gui}")
        w_set = widths(os.path.join(sub, 'gui_width.kicad_pcb'))
        ok(LINE not in logs['gui_width'] and {w for v in w_set.values() for w in v} == {0.22},
           f"with the tab's width override set the engine takes no profile: {w_set}")
    if FAILS:
        print(f"\nFAILED: {len(FAILS)}")
        return 1
    print("\nALL PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
