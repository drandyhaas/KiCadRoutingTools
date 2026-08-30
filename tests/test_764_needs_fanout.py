#!/usr/bin/env python3
"""Issue #764: tell "no room" apart from "needs a fanout I do not build".

  python3 tests/test_764_needs_fanout.py [-v]

route_diff reported `no-escape-path` for two different situations: a terminal
with no room for any copper, and a terminal with room for a TRACK but not for
the coupled PAIR. Only the second is fixable by building an escape, and the
reporter's board (0.5mm-pitch CSP, interior MIPI balls) was the second -- their
own control was route.py routing all six nets single-ended on the board where
every pair reported no-escape-path.

The discriminator re-probes the identical setback ladder with the pair collapsed
to a single track, on the un-inflated obstacle map. Both directions are graded
here, because the interesting failure is the FALSE POSITIVE: own-net copper is
excluded from the obstacle map, so a launch point sitting inside the terminal's
own pad reads "free" on any board however dense. Before the own-pad guard, the
tight arm below (0.42mm pads on 0.5mm pitch, where nothing escapes at all)
reported needs-fanout -- the probe was "succeeding" at the pad centre.

  loose arm  0.20mm pads / 0.5mm pitch -> a track fits, the pair does not
                                       -> needs-fanout + the fanout advice
  tight arm  0.42mm pads / 0.5mm pitch -> nothing fits -> no-escape-path, silent
"""
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from run_utils import tool  # noqa: E402

N = 9            # 9x9 keeps every ladder rung (max 1.31mm) inside the array
PITCH = 0.5


def board_text(pad_size):
    """A CSP-like array with two adjacent INTERIOR pads carrying a diff pair,
    and a 2-pad connector out in open space as the far end."""
    xs = [100.0 + (i - (N - 1) / 2) * PITCH for i in range(N)]
    pads, k = [], 0
    for i, x in enumerate(xs):
        for j, y in enumerate(xs):
            k += 1
            net, name = 0, ""
            if (i, j) == (N // 2, N // 2 - 1):
                net, name = 1, "/PAIR_P"
            if (i, j) == (N // 2, N // 2):
                net, name = 2, "/PAIR_N"
            pads.append(f'    (pad "{k}" smd circle (at {x-100:.3f} {y-100:.3f}) '
                        f'(size {pad_size} {pad_size}) (layers "F.Cu") (net {net} "{name}"))')
    return f'''(kicad_pcb (version 20240108) (generator test)
  (layers
    (0 "F.Cu" signal)
    (31 "B.Cu" signal)
    (44 "Edge.Cuts" user)
  )
  (net 0 "")
  (net 1 "/PAIR_P")
  (net 2 "/PAIR_N")
  (gr_line (start 94 94) (end 116 94) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 116 94) (end 116 106) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 116 106) (end 94 106) (layer "Edge.Cuts") (width 0.1))
  (gr_line (start 94 106) (end 94 94) (layer "Edge.Cuts") (width 0.1))
  (footprint "test:CSP81" (at 100 100) (layer "F.Cu")
    (attr smd)
    (fp_text reference "U1" (at 0 -3) (layer "F.SilkS") (uuid "aaaaaaaa-0000-0000-0000-000000000001"))
{chr(10).join(pads)}
  )
  (footprint "test:CONN" (at 111 100) (layer "F.Cu")
    (attr smd)
    (fp_text reference "J1" (at 0 -3) (layer "F.SilkS") (uuid "aaaaaaaa-0000-0000-0000-000000000002"))
    (pad "1" smd rect (at 0 -0.25) (size 0.6 0.2) (layers "F.Cu") (net 1 "/PAIR_P"))
    (pad "2" smd rect (at 0 0.25) (size 0.6 0.2) (layers "F.Cu") (net 2 "/PAIR_N"))
  )
)
'''


def run_arm(workdir, name, pad_size, verbose):
    src = os.path.join(workdir, f"{name}.kicad_pcb")
    with open(src, 'w') as fh:
        fh.write(board_text(pad_size))
    out = os.path.join(workdir, f"{name}_out.kicad_pcb")
    # F.Cu only: no via escape, so the launch is the whole story.
    # --diff-pair-gap 0.25 reproduces the reporter's inflated gap (#441 floors
    # the gap at --clearance) and with it their exact ladder 0.66/0.49/0.36/...
    r = subprocess.run(
        [sys.executable, tool('route_diff.py'), src, out,
         '--nets', '*PAIR*', '--layers', 'F.Cu',
         '--track-width', '0.115', '--clearance', '0.1',
         '--diff-pair-gap', '0.25', '--grid-step', '0.05'],
        cwd=ROOT, capture_output=True, text=True)
    text = r.stdout + r.stderr
    if verbose:
        print(text)
    reasons = []
    for line in text.splitlines():
        if line.startswith('JSON_SUMMARY: '):
            d = json.loads(line.split('JSON_SUMMARY: ', 1)[1])
            reasons = [p.get('failure_reason') for p in d.get('pair_reports', [])]
    return text, reasons


def main():
    verbose = '-v' in sys.argv or '--verbose' in sys.argv
    failures = []
    with tempfile.TemporaryDirectory() as wd:
        # --- loose: a track fits between 0.2mm pads, the 0.68mm pair does not
        text, reasons = run_arm(wd, 'loose', 0.2, verbose)
        if reasons != ['needs-fanout']:
            failures.append(f"loose arm: expected ['needs-fanout'], got {reasons}")
        else:
            print("PASS: pair-too-wide terminal classified needs-fanout")
        if 'needs a FANOUT' not in text:
            failures.append("loose arm: no fanout advice printed")
        elif 'bga_fanout.py' not in text or '--escape-method underpad' not in text:
            failures.append("loose arm: advice printed without a runnable fanout command")
        else:
            print("PASS: advice names a runnable coupled-fanout command")
        if 'min pad spacing 0.500mm' not in text:
            failures.append("loose arm: advice did not report the measured pitch")
        else:
            print("PASS: advice reports the measured pitch")
        # The whole point is that ripping neighbours cannot help.
        if 'Skipping rip-up' not in text:
            failures.append("loose arm: rip-up ladder was not skipped")
        else:
            print("PASS: rip ladder skipped for a geometrically impossible pair")

        # --- tight: nothing escapes; must NOT be sold a fanout
        text, reasons = run_arm(wd, 'tight', 0.42, verbose)
        if reasons != ['no-escape-path']:
            failures.append(f"tight arm: expected ['no-escape-path'], got {reasons} "
                            f"(a terminal with no room for ANY copper is not a fanout case)")
        else:
            print("PASS: no-room terminal still classified no-escape-path")
        if 'needs a FANOUT' in text:
            failures.append("tight arm: advised a fanout where nothing escapes at all")
        else:
            print("PASS: no fanout advice on the no-room board")

    if failures:
        print("\nFAIL:")
        for f in failures:
            print("  - " + f)
        return 1
    print("\nALL CHECKS PASSED")
    return 0


if __name__ == '__main__':
    sys.exit(main())
