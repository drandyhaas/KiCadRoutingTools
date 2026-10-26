#!/usr/bin/env python3
"""What a board says about the bodies its connectors carry: the facts a
`covers[]` row is declared from (#1443). Facts only -- no decision.

    cover_facts.py BOARD.kicad_pcb [--refs J3,J8] [--json OUT.json]

A card in a socket (an M.2 card), a module on stacking connectors or a shield
on headers stands over the board, and parts may go under it only where they
fit. How low it reaches depends on the socket's height and on the card
fitted (single-sided, or double-sided with its bottom-side parts), which the
board does not say: the intent declares it (`covers[]`), from the datasheets.
This prints what the board does say, per connector, in the connector's own
frame (the footprint's local axes; axis 0 = local +x, 90 = local +y, as a
`covers[]` row's `axis_deg`):

  * its footprint and its own height (its 3D model's top, when one resolves);
  * each 3D model the footprint names that does NOT resolve (a card model in
    the designer's own library is the usual one);
  * what its resolved models raise over the board (a card or module model
    attached to it): the raised extent and its measured underside;
  * the parts that could be its standoffs: standoffs and mounting holes (a
    hole 2 mm or wider, at most one numbered pad) on its face or through the
    board, within 15 mm of one of its four axes, each with its distance
    along that axis and its offset to the axis's left (a row's `offset_mm`);
  * the parts that stand inside its raised extent now, with their heights
    where known (from a model, an IPC-7351 name, or nowhere).

The candidates are the connectors (a J, P, X or CN designator, 10 pads or
more) and every part whose models raise a body past its own outline;
`--refs` names them instead.
A model's underside is the model's, not the card's: a generic card model
draws one card type. Weigh it against the datasheets.

Exit 0, 2 on an unreadable board.
"""
#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['placement'], 'kind': 'instrument'}

import argparse
import contextlib
import io
import json

import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ('py_router', 'py_placer'):
    sys.path.insert(0, os.path.join(ROOT, _d))

#: how far (mm) along an axis a standoff is looked for, and how far off it
REACH_MM = 120.0
OFF_AXIS_MM = 15.0
#: a standoff or mounting hole: a hole at least this wide (mm), and no more
#: than one numbered pad
STANDOFF_HOLE_MM = 2.0
#: a connector with fewer pads than this carries no card or module
CONNECTOR_PADS = 10


def _raster(entry):
    import numpy as np
    x0, y0, nx, ny, hx = entry
    cm = np.frombuffer(bytes.fromhex(hx), '<u2').reshape(nx, ny)
    return x0, y0, np.where(cm == 65535, np.inf, cm / 100.0)


def _raised(entry):
    """(extent, underside min, median) of a part's raised cells, or None."""
    import numpy as np
    from placement.heights import CELL_MM, LIFT_MIN_MM
    x0, y0, lo = _raster(entry)
    up = np.isfinite(lo) & (lo >= LIFT_MIN_MM)
    if not up.any():
        return None
    ii, jj = np.nonzero(up)
    ext = (x0 + ii.min() * CELL_MM, y0 + jj.min() * CELL_MM,
           x0 + (ii.max() + 1) * CELL_MM, y0 + (jj.max() + 1) * CELL_MM)
    return ext, float(lo[up].min()), float(np.median(lo[up]))


def _models(text, board, refs):
    """The model references each footprint in `refs` names, and whether
    each resolves (as the height measurement stages them)."""
    from kicad_parser import iter_footprint_blocks
    from model_export import _locate, model_dirs, resolve_cli
    cli, _ = resolve_cli()
    board_dir = os.path.dirname(os.path.abspath(board))
    dirs = model_dirs(cli, board_dir) if cli else {'KIPRJMOD': board_dir}
    out = {}
    for _s, _e, fp, _raw, k in iter_footprint_blocks(text):
        if k not in refs:
            continue
        rows = []
        for m in re.finditer(r'\(model\s+"([^"]+)"', fp):
            real, _how = _locate(m.group(1), dirs, board_dir)
            rows.append({'path': m.group(1), 'resolved': bool(real)})
        out[k] = rows
    return out


def facts(pcb, board, refs=None):
    from placement.heights import for_board, to_local, to_board
    from placement.legality import footprint_side
    from placement.part_class import CONN_PREFIX_MED, _prefix
    from placement.body import board_bodies
    fps = pcb.footprints or {}
    h = for_board(pcb, board)
    raw = getattr(h, 'raw', None) or {}
    rasters = raw.get('covers') or {}
    geo = board_bodies(pcb, board)
    if refs:
        cands = [r for r in refs if r in fps]
    else:
        cands = sorted(r for r, f in fps.items()
                       if f.pads and (_prefix(r) in CONN_PREFIX_MED
                                      and len(f.pads) >= CONNECTOR_PADS or r in rasters
                                      and _raised(rasters[r]) is not None
                                      and _beyond(r, rasters, geo)))
    with open(board, encoding='utf-8') as f:
        text = f.read()
    models = _models(text, board, set(cands))
    out = []
    for r in cands:
        f = fps[r]
        side = footprint_side(f)
        pose = (f.x, f.y, f.rotation or 0.0)
        row = {'ref': r, 'footprint': f.footprint_name, 'side': side,
               'pose': [round(v, 4) for v in pose], 'height_mm': h.height(r),
               'height_from': (h.top.get(r) or (None, None))[1],
               'models': models.get(r, [])}
        rz = _raised(rasters[r]) if r in rasters else None
        if rz:
            ext, lo_min, lo_med = rz
            row['raised'] = {'extent_local': [round(v, 3) for v in ext],
                             'underside_min_mm': round(lo_min, 3),
                             'underside_median_mm': round(lo_med, 3)}
        stand = []
        for o, g in fps.items():
            if o == r or not _mounting(g):
                continue
            gs = footprint_side(g)
            tht = any((p.drill or 0) > 0 for p in g.pads or ())
            if gs != side and not tht:
                continue
            lx, ly = to_local((g.x, g.y, g.x, g.y), pose)[:2]
            for deg, t, w in ((0, lx, ly), (90, ly, -lx), (180, -lx, -ly), (270, -ly, lx)):
                if 0 < t <= REACH_MM and abs(w) <= OFF_AXIS_MM:
                    stand.append({'ref': o, 'footprint': g.footprint_name,
                                  'axis_deg': deg, 'along_mm': round(t, 3),
                                  'off_axis_mm': round(w, 3),
                                  'side': gs, 'through_hole': tht})
        row['standoff_candidates'] = sorted(stand, key=lambda s: (s['axis_deg'], s['along_mm']))
        if rz:
            ext = rz[0]
            under = []
            for o, g in geo.items():
                if o == r or o not in fps or g.occupancy_local is None:
                    continue
                if footprint_side(fps[o]) != side:
                    continue
                go = fps[o]
                b = to_local(to_board(g.occupancy_local, (go.x, go.y, go.rotation or 0.0)), pose)
                if b[0] < ext[2] and ext[0] < b[2] and b[1] < ext[3] and ext[1] < b[3]:
                    under.append({'ref': o, 'height_mm': h.height(o),
                                  'height_from': (h.top.get(o) or (None, None))[1]})
            row['inside_raised'] = sorted(under, key=lambda u: u['ref'])
        out.append(row)
    return {'board': board, 'heights': h.note, 'connectors': out}


def _mounting(fp):
    """A standoff or mounting hole: a hole `STANDOFF_HOLE_MM` or wider, and
    at most one numbered pad."""
    pads = fp.pads or ()
    named = {str(p.pad_number) for p in pads if str(p.pad_number or '').strip()}
    return len(named) <= 1 and any((p.drill or 0) >= STANDOFF_HOLE_MM for p in pads)


def _beyond(r, rasters, geo):
    """Does part r's raised model reach past its own occupancy (a card or a
    module it carries, rather than its own body's shoulders)?"""
    rz = _raised(rasters[r])
    occ = getattr(geo.get(r), 'occupancy_local', None)
    if rz is None or occ is None:
        return rz is not None
    e = rz[0]
    return e[0] < occ[0] - 1 or e[1] < occ[1] - 1 or e[2] > occ[2] + 1 or e[3] > occ[3] + 1


def _print(doc):
    print(f"cover facts: {doc['board']}")
    print(f"  heights: {doc['heights']}")
    for c in doc['connectors']:
        ht = (f"{c['height_mm']:g}mm ({c['height_from']})" if c['height_mm'] is not None
              else 'unknown')
        print(f"\n{c['ref']}  {c['footprint']}  {c['side']}.Cu  at {c['pose']}  height {ht}")
        for m in c['models']:
            if not m['resolved']:
                print(f"  model NOT resolved: {m['path']}")
        rz = c.get('raised')
        if rz:
            e = rz['extent_local']
            print(f"  raised by its models over local x {e[0]:.2f}..{e[2]:.2f}, "
                  f"y {e[1]:.2f}..{e[3]:.2f}; underside min {rz['underside_min_mm']:g}mm, "
                  f"median {rz['underside_median_mm']:g}mm")
        else:
            print("  no resolved model raises anything over the board")
        for s in c['standoff_candidates']:
            print(f"  standoff? {s['ref']} ({s['footprint']}, {s['side']}"
                  f"{', through' if s['through_hole'] else ''}): axis {s['axis_deg']}, "
                  f"{s['along_mm']:.2f}mm along, {s['off_axis_mm']:+.2f}mm off it")
        if 'inside_raised' in c:
            ins = c['inside_raised']
            print(f"  {len(ins)} part(s) inside the raised extent now"
                  + (': ' + ', '.join(
                      f"{u['ref']} {u['height_mm']:g}mm" if u['height_mm'] is not None
                      else f"{u['ref']} unknown" for u in ins) if ins else ''))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('board')
    ap.add_argument('--refs', help='comma-separated connectors to report (default: every '
                                   'connector and every part whose models raise a body)')
    ap.add_argument('--json', metavar='OUT', help='write the facts as JSON here too')
    a = ap.parse_args(argv)
    from kicad_parser import parse_kicad_pcb
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(a.board)
    except Exception as ex:                                  # noqa: BLE001
        print(f"cover_facts: cannot read {a.board}: {ex}", file=sys.stderr)
        return 2
    refs = [r.strip() for r in a.refs.split(',') if r.strip()] if a.refs else None
    doc = facts(pcb, a.board, refs)
    _print(doc)
    if a.json:
        with open(a.json, 'w', encoding='utf-8') as f:
            json.dump(doc, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main())
