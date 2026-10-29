#!/usr/bin/env python3
"""
Render a Mermaid flowchart source to the PNG the docs show.

Usage:
    python3 docs/diagrams/render_mermaid.py SRC.mmd OUT.png

The docs carry PNGs, not Mermaid blocks; each picture's source is a `.mmd`
file here, redrawn with this script:

    python3 docs/diagrams/render_mermaid.py docs/diagrams/bus_rounds_detail.mmd awx/img/bus_rounds.png
    python3 docs/diagrams/render_mermaid.py docs/diagrams/bus_rounds.mmd docs/bus-rounds.png

The source is drawn by Mermaid itself (a pinned release, loaded from
jsdelivr) in headless Chrome -- found as tests/stress/board_image.py finds it --
at its natural size, twice over for a sharp image, and cropped to the drawing.
"""
import html
import os
import subprocess
import sys
import tempfile

from PIL import Image, ImageChops

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(ROOT, 'tests', 'stress'))
from board_image import _find_chrome  # noqa: E402

MERMAID = 'https://cdn.jsdelivr.net/npm/mermaid@10.9.1/dist/mermaid.min.js'
SCALE = 2                   # device pixels per CSS pixel
WINDOW = (2000, 3200)       # CSS pixels: room for the largest drawing here
PAD = 40                    # device pixels of white kept round the drawing

PAGE = '''<!doctype html><html><head><meta charset="utf-8">
<style>body{{margin:0;background:#fff}} #d{{padding:24px;width:max-content}}</style>
<script src="{mermaid}"></script></head>
<body><div id="d"><pre class="mermaid">{src}</pre></div>
<script>mermaid.initialize({{startOnLoad:true, theme:'default', fontFamily:'Helvetica, Arial, sans-serif',
  flowchart:{{useMaxWidth:false, htmlLabels:true, curve:'basis', nodeSpacing:45, rankSpacing:50, padding:12}},
  themeVariables:{{fontSize:'15px'}}}});</script></body></html>'''


def render(src_path, out_path):
    chrome = _find_chrome()
    if chrome is None:
        sys.exit('render_mermaid: no Chrome or Chromium found')
    with open(src_path, encoding='utf-8') as f:
        src = f.read()
    with tempfile.TemporaryDirectory() as td:
        page = os.path.join(td, 'diagram.html')
        with open(page, 'w', encoding='utf-8') as f:
            f.write(PAGE.format(mermaid=MERMAID, src=html.escape(src)))
        raw = os.path.join(td, 'raw.png')
        subprocess.run([chrome, '--headless=new', '--disable-gpu', '--hide-scrollbars',
                        f'--force-device-scale-factor={SCALE}', f'--window-size={WINDOW[0]},{WINDOW[1]}',
                        '--virtual-time-budget=15000', f'--screenshot={raw}', 'file://' + page],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        im = Image.open(raw).convert('RGB')
    box = ImageChops.difference(im, Image.new('RGB', im.size, (255, 255, 255))).getbbox()
    if box is None:
        sys.exit(f'render_mermaid: {src_path} drew nothing (Mermaid could not be loaded, or the source is wrong)')
    if box[2] >= im.size[0] - 1 or box[3] >= im.size[1] - 1:
        sys.exit(f'render_mermaid: {src_path} runs past the {WINDOW[0]}x{WINDOW[1]} window -- enlarge WINDOW')
    im = im.crop((max(0, box[0] - PAD), max(0, box[1] - PAD), min(im.size[0], box[2] + PAD),
                  min(im.size[1], box[3] + PAD)))
    im.save(out_path, optimize=True)
    print(f'{out_path}: {im.size[0]}x{im.size[1]}')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    render(sys.argv[1], sys.argv[2])
