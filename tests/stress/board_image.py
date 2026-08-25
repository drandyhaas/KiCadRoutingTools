#!/usr/bin/env python3
"""
Render a .kicad_pcb to a single at-a-glance PNG (F.Cu + B.Cu + Edge.Cuts by
default) via kicad-cli SVG export. Used to drop a `<board>.png` next to every
stress-run final board and after each redo replay (#296) so a run's routing
can be eyeballed without opening KiCad.

Usage:
    python3 tests/stress/board_image.py BOARD.kicad_pcb [-o OUT.png] [--layers L1,L2]

PNG rasterization uses headless Chrome when available (the only rasterizer
tested that honors the SVG aspect ratio -- macOS qlmanage crops non-square
content to a square thumbnail); otherwise the SVG is kept next to the
requested output (still viewable in any browser).
"""
import argparse
import os
import re
import shutil
import subprocess
import sys

KICAD_CLI_CANDIDATES = [
    shutil.which('kicad-cli'),
    '/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli',
    '/usr/lib/kicad/bin/kicad-cli',
]
CHROME_CANDIDATES = [
    shutil.which('google-chrome'),
    shutil.which('chromium'),
    '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome',
    '/Applications/Chromium.app/Contents/MacOS/Chromium',
]
DEFAULT_LAYERS = 'auto'  # all copper layers defined in the file + Edge.Cuts


def find_kicad_cli():
    for c in KICAD_CLI_CANDIDATES:
        if c and os.path.exists(c):
            return c
    return None


def _find_chrome():
    for c in CHROME_CANDIDATES:
        if c and os.path.exists(c):
            return c
    return None


def _rasterize_svg(svg_path, png_path, max_px):
    """SVG -> PNG at the SVG's true aspect ratio via headless Chrome.
    Rewrites the width/height attributes to pixels first (KiCad emits mm,
    which browsers scale at 96dpi -- too small for dense boards). Returns
    True on success."""
    chrome = _find_chrome()
    if chrome is None:
        return False
    s = open(svg_path, encoding='utf-8', errors='replace').read()
    m = re.search(r'viewBox="([\d.\-]+) ([\d.\-]+) ([\d.]+) ([\d.]+)"', s)
    if not m:
        return False
    w_mm, h_mm = float(m.group(3)), float(m.group(4))
    if w_mm <= 0 or h_mm <= 0:
        return False
    if w_mm >= h_mm:
        W, H = max_px, max(1, int(round(max_px * h_mm / w_mm)))
    else:
        W, H = max(1, int(round(max_px * w_mm / h_mm))), max_px
    s = re.sub(r'width="[^"]*"', f'width="{W}px"', s, count=1)
    s = re.sub(r'height="[^"]*"', f'height="{H}px"', s, count=1)
    tmp_svg = svg_path + '.px.svg'
    with open(tmp_svg, 'w') as f:
        f.write(s)
    try:
        r = subprocess.run(
            [chrome, '--headless', '--disable-gpu', '--no-sandbox',
             f'--screenshot={png_path}', f'--window-size={W},{H}',
             '--default-background-color=FFFFFFFF', 'file://' + os.path.abspath(tmp_svg)],
            capture_output=True, text=True, timeout=120)
        return r.returncode == 0 and os.path.exists(png_path)
    except Exception:
        return False
    finally:
        if os.path.exists(tmp_svg):
            os.remove(tmp_svg)


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
                     quiet=False, svg_post=None):
    """Render board -> PNG (or SVG fallback). Returns the path written, or None.
    svg_post: optional callable(svg_text) -> svg_text, applied to the exported
    SVG before rasterization (e.g. zone-outline overlays for boards whose zone
    fills are not stored in the file)."""
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
         '--exclude-drawing-sheet', '--drill-shape-opt', '2',
         '-l', layers, '-o', out_svg, board_path],
        capture_output=True, text=True)
    if r.returncode != 0 or not os.path.exists(out_svg):
        if not quiet:
            print(f"board_image: svg export failed: {(r.stderr or r.stdout).strip()[:200]}")
        return None
    # KiCad plots Edge.Cuts in a near-invisible light gray (#D0D2CD) -- recolor
    # to black and thicken so the outline and internal cutouts actually read.
    s = open(out_svg, encoding='utf-8', errors='replace').read()
    s = s.replace('#D0D2CD', '#000000').replace('#d0d2cd', '#000000')
    if svg_post is not None:
        try:
            s = svg_post(s)
        except Exception as e:
            if not quiet:
                print(f"board_image: svg_post skipped ({e})")
    with open(out_svg, 'w') as f:
        f.write(s)
    if _rasterize_svg(out_svg, out_png, size):
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
