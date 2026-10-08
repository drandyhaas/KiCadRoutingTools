#!/usr/bin/env python3
"""Build tests/fixtures/1067/u30_crop.kicad_pcb from run 34's `r/a2.kicad_pcb`.

Run 34 (free-agent, glasgow revC) ran route_planes -> bga_fanout U30 ->
place_fanout_clearance --clearance 0.1, and the cap pass moved C63 from 2.00
to 2.96 mm (pad edge) from its +3V3 ball U30.C10 while its log said
"0 unresolved" (#1067). `a2` is the board that pass read. It lives in a
gitignored run directory and is 2.1 MB, so this keeps only what the cap pass
and the decap grade look at around U30: every footprint whose origin lies in
WINDOW, every via and track with a point in it, and everything that is not a
footprint, via, track or zone (the outline, the nets, the setup). Zones are
dropped (the cap pass reads none). glasgow revC is a tracked corpus board
(kicad_files/glasgow_revC.kicad_pcb), so the crop carries no new licence.

    python3 tests/fixtures/1067/make_fixture.py <run34>/r/a2.kicad_pcb

Writes u30_crop.kicad_pcb and copies a2's .kicad_pro beside it.
"""
import os
import re
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'u30_crop.kicad_pcb')
#: (x0, y0, x1, y1) mm around U30 (origin (100, 97), balls 96..104 x 93..101)
WINDOW = (90.0, 87.0, 110.0, 107.0)
_NUM = r'(-?[\d.]+)'


def _children(text):
    """[(start, end)] of every top-level child of the (kicad_pcb ...) form."""
    out, depth, start, i, n = [], 0, None, 0, len(text)
    in_str = False
    while i < n:
        c = text[i]
        if in_str:
            if c == '\\':
                i += 2
                continue
            if c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == '(':
            depth += 1
            if depth == 2:
                start = i
        elif c == ')':
            if depth == 2:
                out.append((start, i + 1))
            depth -= 1
        i += 1
    return out


def _inside(x, y):
    return WINDOW[0] <= x <= WINDOW[2] and WINDOW[1] <= y <= WINDOW[3]


def _keep(block):
    head = re.match(r'\(\s*([a-z_]+)', block).group(1)
    if head == 'zone':
        return False
    if head == 'footprint':
        m = re.search(r'\(at\s+' + _NUM + r'\s+' + _NUM, block)
        return bool(m) and _inside(float(m.group(1)), float(m.group(2)))
    if head == 'via':
        m = re.search(r'\(at\s+' + _NUM + r'\s+' + _NUM, block)
        return bool(m) and _inside(float(m.group(1)), float(m.group(2)))
    if head in ('segment', 'arc'):
        pts = re.findall(r'\((?:start|mid|end)\s+' + _NUM + r'\s+' + _NUM,
                         block)
        return any(_inside(float(x), float(y)) for x, y in pts)
    return True


def main(src):
    text = open(src, encoding='utf-8', newline='').read()
    kids = _children(text)
    pieces, last, kept, dropped = [], 0, 0, 0
    for s, e in kids:
        block = text[s:e]
        pieces.append(text[last:s])
        if _keep(block):
            pieces.append(block)
            kept += 1
        else:
            dropped += 1
            # swallow the indentation/newline before a dropped child
            pieces[-1] = pieces[-1].rstrip(' \t')
        last = e
    pieces.append(text[last:])
    out = ''.join(pieces)
    out = re.sub(r'\n(\s*\n)+', '\n', out)
    with open(OUT, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(out)
    pro = os.path.splitext(src)[0] + '.kicad_pro'
    if os.path.isfile(pro):
        shutil.copyfile(pro, os.path.splitext(OUT)[0] + '.kicad_pro')
    print(f"kept {kept} top-level item(s), dropped {dropped}; wrote {OUT} "
          f"({os.path.getsize(OUT)} bytes)")


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
