#!/usr/bin/env python3
"""A neck-down never narrows a track below its .kicad_dru track_width minimum.

A synthetic two-layer board: a wall of plated holes (another net) across the
whole board, pitched so the gaps pass a 0.1016 mm track but not a 0.18 mm one,
and one net that must cross it. A rule fixes that net at 0.18 mm on the outer
layers,

    (rule "fixed_outer" (layer outer) (condition "A.NetName == 'SIG*'")
        (constraint track_width (min 0.18mm) (opt 0.18mm) (max 0.18mm)))

and the run passes --track-width 0.1016 (the board minimum, as a floor-width
retry does) under --escalation board, which promises that no descent goes
below a rule minimum. The explicit width is clamped into the rule window, so
the net routes at 0.18; when that is blocked, the neck-down used to retry at
the run's nominal 0.1016 and ship it, plus a taper of 0.117..0.164 pieces:
every one of them a track_width violation (72 on a board where measured).
Now the neck stops at the rule minimum: here that is the net's own width, so
the crossing fails honestly instead of shipping copper DRC rejects.

A second net under a rule with a lower minimum (0.1016) and a 0.18 opt
still necks through the wall, at no less than its minimum: the neck-down
itself is not switched off.

A third net is a 0.18 mm POWER net on a short edge (the step-down ladder)
whose rule minimum binds on F.Cu only. The ladder's bottom rung is the
lowest per-layer floor, so it still necks through the wall on B.Cu while
F.Cu holds 0.18; a ladder floored at the first layer's minimum had no rung
below 0.18 at all, and the edge failed.
"""
import os
import re
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))
from synth import RoutingBoard                                  # noqa: E402

DRU = """(version 1)
(rule "fixed_outer" (layer outer) (condition "A.NetName == 'SIG*'")
    (constraint track_width (min 0.18mm) (opt 0.18mm) (max 0.18mm)))
(rule "soft_outer" (layer outer) (condition "A.NetName == 'SOFT*'")
    (constraint track_width (min 0.1016mm) (opt 0.18mm) (max 0.18mm)))
(rule "split_front" (layer "F.Cu") (condition "A.NetName == 'SPLIT*'")
    (constraint track_width (min 0.18mm)))
"""
PITCH = 1.94   # 1.6 mm holes: 0.34 mm gaps; 0.1016 + 2 x 0.1 fits, 0.18 + 2 x 0.1 does not


def build(td, net):
    b = RoutingBoard(w=20, h=10)
    for k in range(6):
        b.pad(f'H{k}', 10, round(0.15 + PITCH * k, 4), 'GND', sx=1.6, sy=1.6, tht=True)
    b.pad('A1', 2, 5.0, net, sx=0.6, sy=0.6)
    b.pad('A2', 18, 5.0, net, sx=0.6, sy=0.6)
    path = os.path.join(td, f'{net}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(b.text())
    with open(path[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as f:
        f.write(DRU)
    return path


def build_short(td, net):
    """A 12 x 10 board: THT pads 8 mm apart (a short edge) with the wall
    between them."""
    b = RoutingBoard(w=12, h=10)
    for k in range(6):
        b.pad(f'H{k}', 6, round(0.15 + PITCH * k, 4), 'GND', sx=1.6, sy=1.6, tht=True)
    b.pad('A1', 2, 5.0, net, sx=0.8, sy=0.8, tht=True)
    b.pad('A2', 10, 5.0, net, sx=0.8, sy=0.8, tht=True)
    path = os.path.join(td, f'{net}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(b.text())
    with open(path[:-len('.kicad_pcb')] + '.kicad_dru', 'w', encoding='utf-8') as f:
        f.write(DRU)
    return path


def segments(path, net):
    txt = open(path, encoding='utf-8').read()
    nid = re.search(r'\(net (\d+) "%s"\)' % re.escape(net), txt).group(1)
    out = []
    for m in re.finditer(r'\(segment\s.*?\(uuid', txt, re.S):
        s = m.group(0)
        if re.search(r'\(net %s\)' % nid, s) or ('"%s"' % net) in s:
            w = float(re.search(r'\(width ([0-9.]+)\)', s).group(1))
            l = re.search(r'\(layer "([^"]+)"\)', s).group(1)
            out.append((l, w))
    return out


def route(td, net, short=False):
    board = (build_short if short else build)(td, net)
    out = os.path.join(td, f'{net}.out.kicad_pcb')
    extra = ['--power-nets', net, '--power-nets-widths', '0.18'] if short else []
    r = subprocess.run([sys.executable, '-X', 'utf8',
                        os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                        '--nets', net, '--track-width', '0.1016', '--clearance', '0.1',
                        '--grid-step', '0.02', '--layers', 'F.Cu', 'B.Cu',
                        '--escalation', 'board'] + extra,
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT)
    if not os.path.exists(out):
        print(r.stdout[-3000:], r.stderr[-2000:])
        return None, r.stdout
    return segments(out, net), r.stdout


def main():
    fails = []
    with tempfile.TemporaryDirectory() as td:
        segs, log = route(td, 'SIG')
        if segs is None:
            fails.append("route.py produced no board for SIG")
        else:
            if 'neck-down' not in log and 'Neck-down' not in log and segs:
                print("note: no neck-down line in the SIG log")
            narrow = sorted({w for _, w in segs if w < 0.18 - 1e-6})
            if narrow:
                fails.append(f"SIG: {sum(w < 0.18 - 1e-6 for _, w in segs)}/{len(segs)} "
                             f"segments narrower than the rule's 0.18 min: {narrow}")
        soft, log2 = route(td, 'SOFT')
        if soft is None:
            fails.append("route.py produced no board for SOFT")
        else:
            if not soft:
                fails.append("SOFT did not route: the neck-down must still cross the wall")
            low = sorted({w for _, w in soft if w < 0.1016 - 1e-6})
            if low:
                fails.append(f"SOFT: segments below the rule's 0.1016 min: {low}")
            if soft and not any(w < 0.18 - 1e-6 for _, w in soft):
                fails.append("SOFT crossed the wall without necking: the wall does not bite")
        split, _ = route(td, 'SPLIT', short=True)
        if split is None:
            fails.append("route.py produced no board for SPLIT")
        elif not split:
            fails.append("SPLIT did not route: the short-edge ladder must neck "
                         "on B.Cu, where no rule minimum binds")
        else:
            front = sorted({w for l, w in split if l == 'F.Cu' and w < 0.18 - 1e-6})
            if front:
                fails.append(f"SPLIT: F.Cu segments below the rule's 0.18 min: {front}")
            if not any(l == 'B.Cu' and w < 0.18 - 1e-6 for l, w in split):
                fails.append("SPLIT crossed without necking on B.Cu: the wall does not bite")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print(f"PASS: SIG shipped {len(segs)} segments, none below 0.18; "
          f"SOFT necked through the wall at >= 0.1016 ({len(soft)} segments); "
          f"SPLIT necked on B.Cu only ({len(split)} segments)")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
