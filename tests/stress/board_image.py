#!/usr/bin/env python3
"""
Render a .kicad_pcb to a single at-a-glance PNG (F.Cu + B.Cu + Edge.Cuts by
default) via kicad-cli SVG export. Used to drop a `<board>.png` next to every
stress-run final board and after each redo replay (#296) so a run's routing
can be eyeballed without opening KiCad.

Usage:
    python3 tests/stress/board_image.py BOARD.kicad_pcb [-o OUT.png] [--layers L1,L2]

PNG rasterization uses macOS qlmanage when available; otherwise the SVG is
kept next to the requested output (still viewable in any browser).
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

KICAD_CLI_CANDIDATES = [
    shutil.which('kicad-cli'),
    '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli',
    '/usr/lib/kicad/bin/kicad-cli',
]
DEFAULT_LAYERS = 'auto'  # all copper layers defined in the file + Edge.Cuts


def find_kicad_cli():
    for c in KICAD_CLI_CANDIDATES:
        if c and os.path.exists(c):
            return c
    return None


def board_copper_layers(board_path):
    """Copper layer names from the file's (layers ...) header, in stack order.
    Cheap regex scan -- no parser dependency."""
    import re
    layers = []
    with open(board_path, encoding='utf-8', errors='replace') as f:
        head = f.read(60000)
    m = re.search(r'\(layers\s*\n(.*?)\n\s*\)', head, re.S)
    if m:
        for ln in m.group(1).splitlines():
            lm = re.match(r'\s*\(\d+\s+"([^"]+\.Cu)"\s+signal', ln)
            if lm:
                layers.append(lm.group(1))
    return layers


def render_board_png(board_path, out_png=None, layers=DEFAULT_LAYERS, size=2000,
                     quiet=False):
    """Render board -> PNG (or SVG fallback). Returns the path written, or None."""
    kc = find_kicad_cli()
    if kc is None:
        if not quiet:
            print("board_image: kicad-cli not found, skipping render")
        return None
    board_path = os.path.abspath(board_path)
    if layers == 'auto':
        cu = board_copper_layers(board_path)
        layers = ','.join((cu or ['F.Cu', 'B.Cu']) + ['Edge.Cuts'])
    if out_png is None:
        out_png = os.path.splitext(board_path)[0] + '.png'
    out_svg = os.path.splitext(out_png)[0] + '.svg'
    r = subprocess.run(
        [kc, 'pcb', 'export', 'svg', '--mode-single', '--page-size-mode', '2',
         '-l', layers, '-o', out_svg, board_path],
        capture_output=True, text=True)
    if r.returncode != 0 or not os.path.exists(out_svg):
        if not quiet:
            print(f"board_image: svg export failed: {(r.stderr or r.stdout).strip()[:200]}")
        return None
    # Rasterize (macOS qlmanage). qlmanage names the output <basename>.svg.png
    # in the -o dir, so run it in a temp dir and move the result into place.
    if shutil.which('qlmanage'):
        with tempfile.TemporaryDirectory() as td:
            q = subprocess.run(['qlmanage', '-t', '-s', str(size), '-o', td, out_svg],
                               capture_output=True, text=True)
            thumb = os.path.join(td, os.path.basename(out_svg) + '.png')
            if q.returncode == 0 and os.path.exists(thumb):
                shutil.move(thumb, out_png)
                os.remove(out_svg)
                if not quiet:
                    print(f"board_image: wrote {out_png}")
                return out_png
    if not quiet:
        print(f"board_image: wrote {out_svg} (no PNG rasterizer; kept SVG)")
    return out_svg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('-o', '--output', default=None, help='output PNG path (default: alongside board)')
    ap.add_argument('--layers', default=DEFAULT_LAYERS)
    ap.add_argument('--size', type=int, default=2000, help='max thumbnail dimension px')
    args = ap.parse_args()
    out = render_board_png(args.board, args.output, args.layers, args.size)
    return 0 if out else 1


if __name__ == '__main__':
    sys.exit(main())
