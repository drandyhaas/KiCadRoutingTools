#!/usr/bin/env python3
"""An explicit --track-width still meets a layer-scoped .kicad_dru width rule.

An in-repo four-signal-layer board with a rule file in the shape of a
55-ohm class: the SDRAM data nets fixed at 0.18 mm on outer layers and
0.102 mm on inner layers,

    (rule "se_outer" (layer outer) (condition "A.NetName == 'SDRAM_D*'")
        (constraint track_width (min 0.18mm) (opt 0.18mm) (max 0.18mm)))
    (rule "se_inner" (layer inner) (condition "A.NetName == 'SDRAM_D*'")
        (constraint track_width (min 0.102mm) (opt 0.102mm) (max 0.102mm)))

routed with --track-width 0.18 (the width a bench passes for the Default
class) under --escalation board (descents stop at the rule minimums; the
`fab` policy may go below them by design). Before the fix an explicit --track-width dropped the per-layer widths
whole: every inner segment came out at 0.18, a track_width violation on each
one (199 where measured). The operator's width still wins where the rule allows it;
where a layer's rule window excludes it, the net draws the width clamped into
that window, as KiCad's router does for a custom size against a rule.
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'ulx3s.kicad_pcb')
NETS = [f'SDRAM_D{i}' for i in range(8)]
DRU = """(version 1)
(rule "se_outer" (layer outer) (condition "A.NetName == 'SDRAM_D*'")
    (constraint track_width (min 0.18mm) (opt 0.18mm) (max 0.18mm)))
(rule "se_inner" (layer inner) (condition "A.NetName == 'SDRAM_D*'")
    (constraint track_width (min 0.102mm) (opt 0.102mm) (max 0.102mm)))
"""


def segments(path, names):
    """(layer, width) of every segment on the named nets, line-based (a regex over
    the whole board backtracks for minutes on a large file)."""
    out, cur = [], None
    for line in open(path, encoding='utf-8'):
        s = line.strip()
        if s.startswith('(segment'):
            cur = {}
        if cur is None:
            continue
        m = re.match(r'\(width ([0-9.]+)\)', s)
        if m:
            cur['w'] = float(m.group(1))
        m = re.match(r'\(layer "([^"]+)"\)', s)
        if m:
            cur['l'] = m.group(1)
        m = re.match(r'\(net (?:\d+ )?"([^"]*)"\)', s) or re.match(r'\(net (\d+)\)', s)
        if m:
            cur['n'] = m.group(1)
        if 'w' in cur and 'l' in cur and 'n' in cur:
            if cur['n'] in names:
                out.append((cur['l'], cur['w']))
            cur = None
    return out


def main():
    fails = []
    with tempfile.TemporaryDirectory() as td:
        board = os.path.join(td, 'ulx3s.kicad_pcb')
        shutil.copyfile(BOARD, board)
        with open(os.path.join(td, 'ulx3s.kicad_dru'), 'w', encoding='utf-8') as f:
            f.write(DRU)
        out = os.path.join(td, 'out.kicad_pcb')
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'route.py'), board, out,
                            '--nets', *NETS, '--track-width', '0.18',
                            '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
                            '--escalation', 'board'],
                           capture_output=True, text=True, encoding='utf-8',
                           errors='replace', cwd=ROOT)
        if r.returncode != 0 or not os.path.exists(out):
            print(r.stdout[-3000:], r.stderr[-2000:])
            print("FAIL: route.py did not produce a board")
            return 1
        segs = segments(out, set(NETS))
        inner = [w for l, w in segs if l.startswith('In')]
        outer = [w for l, w in segs if not l.startswith('In')]
        if not inner:
            fails.append(f"no inner-layer segment was routed, the rule is not exercised: {len(segs)} segments")
        bad_in = sorted({w for w in inner if abs(w - 0.102) > 1e-6})
        if bad_in:
            fails.append(f"{sum(abs(w - 0.102) > 1e-6 for w in inner)}/{len(inner)} inner segments off "
                         f"the inner rule's fixed 0.102: {bad_in}")
        bad_out = sorted({w for w in outer if w < 0.18 - 1e-6})
        if bad_out:
            fails.append(f"{sum(w < 0.18 - 1e-6 for w in outer)}/{len(outer)} outer segments narrower than "
                         f"the outer rule's 0.18 min: {bad_out}")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        return 1
    print(f"PASS: explicit --track-width 0.18 drew {len(inner)} inner segments at the inner rule's 0.102 "
          f"and {len(outer)} outer segments at 0.18")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
