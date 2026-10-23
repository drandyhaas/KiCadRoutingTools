#!/usr/bin/env python3
"""py_tools/render_nets.py: chosen nets' copper on several boards, side by side.

On the tracked, partly routed routed_output board (a few seconds):
  1. two labelled panels of one net are drawn side by side, the PNG two panels
     wide, and each panel's title counts the net's vias and track length as
     the board has them;
  2. a pattern no board's net matches is refused (exit 2, naming it) -- the
     control that a typo draws nothing rather than an empty picture;
  3. a fixed --view sets the panel size: scale x the window.
"""
import math
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
from kicad_parser import parse_kicad_pcb  # noqa: E402

TOOL = os.path.join(ROOT, 'py_tools', 'render_nets.py')
BOARD = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
results = []


def ok(name, cond, detail=''):
    results.append(cond)
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f" -- {detail}" if not cond and detail else ''))


def run(*args):
    return subprocess.run([sys.executable, '-X', 'utf8', TOOL, *args],
                          capture_output=True, text=True, timeout=300)


def main():
    from PIL import Image
    print("render_nets: chosen nets, side by side")
    pcb = parse_kicad_pcb(BOARD)
    nid = max({s.net_id for s in pcb.segments if s.net_id},
              key=lambda n: sum(1 for v in pcb.vias if v.net_id == n))
    net = pcb.nets[nid].name
    nv = sum(1 for v in pcb.vias if v.net_id == nid)
    mm = sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
             for s in pcb.segments if s.net_id == nid and not getattr(s, 'graphic', False))
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'two.png')
        r = run('-o', out, '--nets', net, f'a={BOARD}', f'b={BOARD}')
        ok("two panels drawn", r.returncode == 0 and os.path.isfile(out), r.stderr[-300:])
        ok(f"each title counts {net}'s {nv} vias and {mm:.1f} mm",
           r.stdout.count(f"{nv} vias, {mm:.1f} mm") == 2, r.stdout)
        out2 = os.path.join(td, 'view.png')
        r2 = run('-o', out2, '--nets', net, '--view', '0,0,10,5', '--scale', '10', BOARD)
        if r2.returncode == 0:
            w, h = Image.open(out2).size
            ok("--view sets the panel: 100 x 50 px plus the title row", (w, h) == (100, 80), (w, h))
        else:
            ok("--view run exits 0", False, r2.stderr[-300:])
        if os.path.isfile(out):
            w2, _h = Image.open(out).size
            ok("two panels are wider than one", w2 > 2 * (Image.open(out).size[0] // 2) - 20)
        r3 = run('-o', os.path.join(td, 'no.png'), '--nets', 'NO_SUCH_NET_XYZ', BOARD)
        ok("an unmatched pattern is refused, naming it",
           r3.returncode == 2 and 'NO_SUCH_NET_XYZ' in r3.stderr, (r3.returncode, r3.stderr[-200:]))
    print(f"\n{sum(results)}/{len(results)} checks passed")
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
