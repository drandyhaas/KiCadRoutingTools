#!/usr/bin/env python3
"""A .kicad_dru track_width rule reaches the router (#530 / #770).

Two pads of one net 10 mm apart on the harness's synthetic board, a Default
class drawing 0.2 mm, and a rule

    (rule "w" (constraint track_width (min 0.3mm) (opt 0.35mm)))

With --track-width omitted, route.py draws the net at the rule's opt (0.35,
what KiCad's own router draws under "use netclass values") and nothing on
the board is narrower than the rule's min (0.3): the rescue and neck ladders
floor at the rule, not at the fab tier. Before #530 the router read only
clearance rules and this board came out at 0.2 (#770: the modelled subset
never bound on a real board).
"""
import json
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'tests', 'oracle'))
from constraint_agreement import write_board  # noqa: E402


def main():
    fails = []
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'w.kicad_pcb')
        fps = [{'ref': 'U1', 'x': 10, 'y': 10, 'net_id': 1, 'net_name': 'A', 'size': 1.0},
               {'ref': 'U2', 'x': 20, 'y': 10, 'net_id': 1, 'net_name': 'A', 'size': 1.0}]
        write_board(board, footprints=fps,
                    dru='(rule "w" (constraint track_width (min 0.3mm) (opt 0.35mm)))')
        out = os.path.join(td, 'out.kicad_pcb')
        js = os.path.join(td, 'out.json')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                            '--nets', 'A', '--json-out', js],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=ROOT)
        if r.returncode != 0 or not os.path.exists(out):
            print(r.stdout[-3000:])
            print("FAIL: route.py did not produce a board")
            return 1
        widths = [float(m.group(1)) for m in
                  re.finditer(r'\(segment\b.*?\(width ([0-9.]+)\)', open(out).read(), re.S)]
        if not widths:
            fails.append("no segment was routed")
        if any(w < 0.3 - 1e-9 for w in widths):
            fails.append(f"a segment is narrower than the rule's 0.3 min: {sorted(set(widths))}")
        if not any(abs(w - 0.35) < 1e-9 for w in widths):
            fails.append(f"the rule's 0.35 opt was not drawn: {sorted(set(widths))}")
        data = json.load(open(js)) if os.path.exists(js) else {}
        dr = data.get('design_rules') or {}
        if dr.get('unsupported_rules'):
            fails.append(f"the rule was reported unsupported: {dr}")
        # the grader agrees: nothing under the rule
        g = subprocess.run([sys.executable, os.path.join(ROOT, 'py_router', 'check_drc.py'),
                            out, '--quiet'], capture_output=True, text=True, cwd=ROOT)
        if 'track-width' in g.stdout or 'too thin' in g.stdout:
            fails.append("check_drc flagged a track-width violation on the routed board")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print("PASS: a .kicad_dru track_width rule is drawn at its opt and floors every descent at its min")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
