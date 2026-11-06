#!/usr/bin/env python3
"""Render each named edge part close up, from outside its edge (#1719).

plan-pcb-placement's edge check judges which way an edge connector opens from
a 3D render taken from outside the board edge it sits on. A whole-board
`kicad-cli pcb render` frames the whole BOARD, so the scale falls as the
board grows: a ~155 mm board lands at ~5.6 px/mm in a 2400 x 900 render,
where a 1.00 mm side-entry header is a 35 px block whose contacts are 1 px
specks and its mouth-out and mouth-in renders look the same. This renders
each part on its own, centred and at a STATED scale (default 40 px/mm; such
a header's contacts already read at about 19), and prints the scale each
image was taken at.

How the camera is set, measured on kicad-cli 10 (orthographic view):
  * `--pivot X,Y,Z` (cm, relative to the board's outline centre, KiCad's 3D
    axes: +Y is the board's north) is the point the view centres on, so
    the part's position becomes the pivot;
  * the scale is linear in `--zoom`, and the scale at zoom 1 is MEASURED per
    side, not assumed: two zoom-1 renders whose pivots differ by 1 cm along
    the image's horizontal, the shift of the opaque columns between them
    being the pixels per cm. (The board's extent is no measure: an
    overhanging part widens it.)

    python3 py_tools/edge_closeup.py BOARD J7 J3:north --out-dir DIR

Each REF is rendered from outside the outline edge its pads are nearest
(`REF:north|south|east|west` names it instead): `back` stands outside the
north edge, `front` the south, `left` the west, `right` the east. Prints one
line per part and, with `--json PATH`, the same as JSON. A part whose
3D model is not declared or does not resolve is listed as such: the render
shows its pads only and says nothing about its mouth.
"""

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement', 'combined'], 'kind': 'utility'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

#: The scale a part is rendered at when none is asked for: a 1.00 mm pitch
#: header's 0.2 mm contact marks are 8 px here and about 1 px at the
#: whole-board framing of a ~155 mm board.
DEFAULT_PX_PER_MM = 40.0
#: Which `--side` stands outside which outline edge.
SIDES = {'north': 'back', 'south': 'front', 'west': 'left', 'east': 'right'}


def _cli():
    from kicad_oracle import find_kicad_cli
    return find_kicad_cli(warn=False)


def _render(cli, board, side, out, width, height, zoom=1.0, pivot=(0, 0, 0)):
    cmd = [cli, 'pcb', 'render', board, '--side', side,
           '-w', str(width), '-h', str(height), '--zoom', '%.6g' % zoom,
           '--pivot', ','.join('%.6g' % v for v in pivot), '-o', out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or not os.path.isfile(out):
        raise RuntimeError('kicad-cli pcb render failed (exit %d): %s'
                           % (r.returncode, (r.stderr or r.stdout)[-400:]))


def _opaque_columns(path):
    import numpy as np
    from PIL import Image
    a = np.asarray(Image.open(path).convert('RGBA'))[:, :, 3] > 0
    cols = np.where(a.any(0))[0]
    if not len(cols):
        raise RuntimeError(f'{path}: the render is empty')
    return int(cols[0]), int(cols[-1]), a.shape[1]


def px_per_mm_at_zoom1(cli, board, side, width, height, tmp):
    """Pixels per mm at zoom 1 on this side: the opaque columns' shift when
    the pivot moves 1 cm along the image's horizontal."""
    axis = 'x' if side in ('front', 'back') else 'y'
    a = os.path.join(tmp, f'cal_{side}_0.png')
    b = os.path.join(tmp, f'cal_{side}_1.png')
    _render(cli, board, side, a, width, height)
    _render(cli, board, side, b, width, height,
            pivot=(1, 0, 0) if axis == 'x' else (0, 1, 0))
    (a0, a1, w), (b0, b1, _w) = _opaque_columns(a), _opaque_columns(b)
    if min(a0, b0) <= 0 or max(a1, b1) >= w - 1:
        raise RuntimeError('the zoom-1 board touches the image edge, so its '
                           'shift is not measurable: give a wider --width')
    shift = ((a0 - b0) + (a1 - b1)) / 2.0
    if abs(shift) < 1:
        raise RuntimeError('a 1 cm pivot moved the render by under a pixel')
    return abs(shift) / 10.0


def _pads_rect(fp):
    xs, ys = [], []
    for p in fp.pads:
        hx, hy = (p.size_x or 0) / 2.0, (p.size_y or 0) / 2.0
        xs += [p.global_x - hx, p.global_x + hx]
        ys += [p.global_y - hy, p.global_y + hy]
    if not xs:
        return (fp.x, fp.y, fp.x, fp.y)
    return (min(xs), min(ys), max(xs), max(ys))


def nearest_edge(rect, bounds):
    """The outline edge (north/south/east/west) the rect is nearest."""
    d = {'west': rect[0] - bounds[0], 'north': rect[1] - bounds[1],
         'east': bounds[2] - rect[2], 'south': bounds[3] - rect[3]}
    return min(d, key=d.get)


def _model_states(board, refs):
    """{ref: 'resolved' | 'unresolved' | 'none'}: whether each footprint names
    a 3D model and whether one resolves, as the height measurement stages
    them (`cover_facts` reads the same)."""
    import re
    from kicad_parser import iter_footprint_blocks
    from model_export import (_locate, board_embedded_names, embedded_names,
                              model_dirs, resolve_cli)
    with open(board, encoding='utf-8') as fh:
        text = fh.read()
    cli, _ = resolve_cli()
    board_dir = os.path.dirname(os.path.abspath(board))
    dirs = model_dirs(cli, board_dir) if cli else {'KIPRJMOD': board_dir}
    board_embedded = board_embedded_names(text)
    out = {}
    for _s, _e, fp, _raw, k in iter_footprint_blocks(text):
        if k not in refs:
            continue
        embedded = board_embedded | embedded_names(fp)
        paths = re.findall(r'\(model\s+"([^"]+)"', fp)
        out[k] = ('none' if not paths else
                  'resolved' if any(_locate(m, dirs, board_dir, embedded)[0]
                                    for m in paths) else 'unresolved')
    return out


def closeups(board, specs, out_dir, px_per_mm=DEFAULT_PX_PER_MM,
             width=1600, height=900, cli=None):
    """Render each `REF` or `REF:edge` in `specs`; returns one row per part."""
    from kicad_parser import parse_kicad_pcb
    cli = cli or _cli()
    if not cli:
        raise RuntimeError('kicad-cli not found (set KICAD_CLI)')
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    bounds = pcb.board_info.board_bounds
    if not bounds:
        raise RuntimeError(f'{board}: no board outline, so no edge to stand at')
    cx, cy = (bounds[0] + bounds[2]) / 2.0, (bounds[1] + bounds[3]) / 2.0
    os.makedirs(out_dir, exist_ok=True)
    models = _model_states(board, {sp.partition(':')[0] for sp in specs})
    scale1, rows = {}, []
    with tempfile.TemporaryDirectory() as tmp:
        for spec in specs:
            ref, _, edge = spec.partition(':')
            fp = pcb.footprints.get(ref)
            if fp is None:
                raise RuntimeError(f'{ref}: no such part on {board}')
            rect = _pads_rect(fp)
            basis = 'named' if edge else 'nearest'
            edge = (edge or nearest_edge(rect, bounds)).lower()
            if edge not in SIDES:
                raise RuntimeError(f'{spec}: the edge is one of '
                                   f'{", ".join(SIDES)}')
            side = SIDES[edge]
            if side not in scale1:
                scale1[side] = px_per_mm_at_zoom1(cli, board, side, width,
                                                  height, tmp)
            zoom = px_per_mm / scale1[side]
            px, py = (rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0
            # KiCad's 3D +Y is the board's north (board y runs south).
            pivot = ((px - cx) / 10.0, -(py - cy) / 10.0, 0.0)
            out = os.path.join(out_dir, f'{ref}_{edge}.png')
            _render(cli, board, side, out, width, height, zoom, pivot)
            span = (rect[2] - rect[0]) if side in ('front', 'back') \
                else (rect[3] - rect[1])
            rows.append({'ref': ref, 'edge': edge, 'edge_basis': basis,
                         'side': side, 'png': out,
                         'px_per_mm': round(px_per_mm, 3),
                         'px_per_mm_at_zoom1': round(scale1[side], 3),
                         'zoom': round(zoom, 4),
                         'pivot_cm': [round(v, 4) for v in pivot],
                         'pads_span_px': round(span * px_per_mm),
                         'model': models.get(ref, 'none')})
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='Render each named edge part close up from outside its '
                    'edge, at a stated scale (#1719).')
    ap.add_argument('board')
    ap.add_argument('refs', nargs='+',
                    help='REF, or REF:north|south|east|west to name the edge')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--px-per-mm', type=float, default=DEFAULT_PX_PER_MM,
                    help='the scale to render at (default %(default)g)')
    ap.add_argument('--width', type=int, default=1600)
    ap.add_argument('--height', type=int, default=900)
    ap.add_argument('--json', help='write the rows as JSON here too')
    args = ap.parse_args(argv)
    try:
        rows = closeups(args.board, args.refs, args.out_dir, args.px_per_mm,
                        args.width, args.height)
    except RuntimeError as exc:
        print(f'edge_closeup: {exc}', file=sys.stderr)
        return 2
    for r in rows:
        print(f"{r['ref']:8} {r['edge']} edge ({r['edge_basis']}), side "
              f"{r['side']}: {r['png']} at {r['px_per_mm']:g} px/mm "
              f"(zoom {r['zoom']:g}; zoom 1 frames the board at "
              f"{r['px_per_mm_at_zoom1']:g} px/mm); pads span "
              f"{r['pads_span_px']} px"
              + ('' if r['model'] == 'resolved' else
                 ' -- its 3D model %s: the render shows its pads only, and '
                 'nothing about its mouth'
                 % ('is not declared' if r['model'] == 'none'
                    else 'does not resolve')))
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            json.dump(rows, fh, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
