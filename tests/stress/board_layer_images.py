#!/usr/bin/env python3
"""
Render a .kicad_pcb to per-layer PNGs (one image per copper layer, each with
Edge.Cuts for the outline) plus the combined all-copper PNG. Per-layer images
make layer roles readable at a glance -- plane vs signal layers, bus rivers,
pour islands -- which the combined render hides behind the top copper (#424
ottercast study). Drops `<board>_<layer>.png` for every copper layer and the
existing `<board>.png` next to the board, so a stress-run final board can be
eyeballed layer by layer without opening KiCad.

This supersedes calling board_image.py directly for stress-run renders; the
single-image machinery (kicad-cli SVG export + headless-Chrome rasterize)
still lives there and is reused here.

Usage:
    python3 tests/stress/board_layer_images.py BOARD.kicad_pcb [-o OUT.png]
        [--size PX] [--no-combined]

`-o OUT.png` names the COMBINED image; per-layer files derive from it
(`OUT_F.Cu.png`, `OUT_In1.Cu.png`, ...). Default: alongside the board.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from board_image import render_board_png, board_copper_layers  # noqa: E402


def render_board_layer_pngs(board_path, out_png=None, size=2000, quiet=False,
                            combined=True):
    """Render one PNG per copper layer (+ Edge.Cuts) and optionally the
    combined all-copper PNG. Returns the list of paths written ([] on total
    failure -- e.g. kicad-cli missing)."""
    board_path = os.path.abspath(board_path)
    if out_png is None:
        out_png = os.path.splitext(board_path)[0] + '.png'
    base, ext = os.path.splitext(out_png)
    written = []
    for layer in board_copper_layers(board_path) or ['F.Cu', 'B.Cu']:
        out = render_board_png(board_path, f'{base}_{layer}{ext}',
                               layers=f'{layer},Edge.Cuts', size=size,
                               quiet=quiet)
        if out:
            written.append(out)
    if combined:
        out = render_board_png(board_path, out_png, size=size, quiet=quiet)
        if out:
            written.append(out)
    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('-o', '--output', default=None,
                    help='combined PNG path; per-layer files derive from it '
                         '(default: alongside board)')
    ap.add_argument('--size', type=int, default=2000,
                    help='max image dimension px')
    ap.add_argument('--no-combined', action='store_true',
                    help='per-layer images only, skip the combined render')
    args = ap.parse_args()
    written = render_board_layer_pngs(args.board, args.output, args.size,
                                      combined=not args.no_combined)
    return 0 if written else 1


if __name__ == '__main__':
    sys.exit(main())
