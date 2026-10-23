#!/usr/bin/env python3
"""awx/plane_render.py draws a board with its planes.

  python3 tests/test_622_plane_render.py

On kicad_files/lvds_converter_dualclk_gnd.kicad_pcb (a GND pour on B.Cu): the render exits 0, writes a PNG of a
board's size with a legend beside it, and names the plane it drew -- KiCad's fill where KiCad's python is to be had,
the zone's outline where it is not (either is a plane drawn, and the line says which).
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BOARD = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk_gnd.kicad_pcb')


def main():
    print('=' * 60)
    print('plane_render: a board with its planes')
    assert os.path.isfile(BOARD) and os.path.getsize(BOARD) > 0, f'no board {BOARD}'
    with tempfile.TemporaryDirectory() as td:
        out = os.path.join(td, 'planes.png')
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'awx', 'plane_render.py'), BOARD, out, '--all-nets',
                            '--quarter', '1'], capture_output=True, text=True, timeout=600)
        line = [ln for ln in r.stdout.splitlines() if ln.startswith(out)]
        fails = []
        if r.returncode != 0:
            fails.append(f'exit {r.returncode}: {r.stderr[-400:]}')
        if not os.path.isfile(out) or os.path.getsize(out) == 0:
            fails.append('no PNG written')
        else:
            from PIL import Image
            w, h = Image.open(out).size
            if not (w > 1000 and h > 500):
                fails.append(f'a PNG of {w} x {h}')
        if not line or 'GND@B.Cu' not in line[-1] or not ('KiCad fill' in line[-1] or 'zone outlines' in line[-1]):
            fails.append(f'the GND plane not named as drawn: {line or r.stdout[-300:]}')
    for f in fails:
        print(f'  FAIL: {f}')
    if fails:
        return 1
    print(f'PASS: the board drawn with its plane -- {line[-1].split(": ", 1)[1]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
