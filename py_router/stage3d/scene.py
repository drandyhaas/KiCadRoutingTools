#!/usr/bin/env python3
"""The 3D board's static scene (#1081): outline, parts, pads, and -- when
kicad-cli can supply them -- the parts' real 3D models.

**Coordinates.** Millimetres, KiCad's own board frame: x right, y DOWN. The
page maps board (x, y) to three.js (x, z) with y UP out of the board's top
face, so a KiCad rotation `rot` is exactly `rotation.y = rad(rot)` -- measured
on the GLB kicad-cli writes (every F.Cu part node is `Ry(rot)`, every B.Cu one
`Ry(rot) Rx(180)`; the spike's 254 matched nodes over 3 boards).

**Parts are ALWAYS drawn** as a body box over their pads plus the pads, from
the board itself, so the 3D board needs nothing but the parsed boards and the
determinism test can run anywhere Chromium can. Each part's geometry is kept in
its OWN frame (`local`), recovered from the parsed pad positions by inverting
`kicad_parser.local_to_global`, so a glide that turns a part (#1086) turns it
in 3D exactly as in 2D.

**Real models are optional** (`export_glb`). kicad-cli exports the FINAL
board's parts, each as a node named by its bare refdes; the page re-poses a
node by `F(now) * F(final)^-1`, which needs no knowledge of the model's own
offset. Two measured traps are handled here rather than left to the viewer:
KiCad 10 ships only `.step` models while old boards reference `.wrl`, and an
unresolved model is dropped SILENTLY (55 of 58 missing on splitflap) -- so the
board is staged with each missing `.wrl` pointed at an existing `.step` twin,
and `matched` says how many parts got a model.
"""
from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from typing import Dict, List, Optional, Tuple

#: Board thickness when the stackup does not say.
DEFAULT_THICKNESS = 1.6
#: A part body's height, from its footprint's own size: a quarter of its
#: shorter side, clamped. A stand-in for the model the file does not carry.
BODY_H_FRAC = 0.25
BODY_H_MIN, BODY_H_MAX = 0.35, 3.0
#: A part whose pad field covers more than this share of the board is not a
#: package sitting on it -- it is a module, a connector field or a castellated
#: carrier whose pads ARE the board's edge (rp2350_fpga_eensy's footprint
#: spans the whole outline) -- so it gets no body box, which would otherwise
#: lid the board and hide everything under it.
BODY_MAX_BOARD_FRAC = 0.2
#: kicad-cli's GLB export, measured at 3-6 s on a 60-part board.
GLB_TIMEOUT_S = 300


def board_thickness(pcb) -> float:
    st = getattr(pcb.board_info, 'stackup', None) or []
    t = sum(float(getattr(s, 'thickness', 0) or 0) for s in st)
    return t if 0.2 <= t <= 10.0 else DEFAULT_THICKNESS


def _to_local(fp, x, y):
    """A board point in `fp`'s own frame: global = origin + R(-rot) local,
    so local = R(rot) (global - origin)."""
    rot = math.radians(fp.rotation or 0.0)
    dx, dy = x - fp.x, y - fp.y
    return (dx * math.cos(rot) - dy * math.sin(rot),
            dx * math.sin(rot) + dy * math.cos(rot))


def _local_pad(fp, p) -> list:
    """A pad in its footprint's own frame: `[lx, ly, sx, sy, angle, shape,
    face, drill, hole, polys]`. `face` is 'F', 'B' or 'T' (through-hole);
    `hole` is the drill centre `[hx, hy]` when the drill is OFFSET from the
    copper (castellated paddles), else None; `polys` is a custom pad's real
    copper outline(s), else None -- the parser's `size_x/size_y` for a custom
    pad is only its board-space bbox, which is neither its shape nor
    rotation-invariant (#1090: 0.52 mm off on rp2350 U4/U5 at a 37 degree
    turn)."""
    lx, ly = _to_local(fp, p.global_x, p.global_y)
    ang = (p.rect_rotation or 0.0) + (fp.rotation or 0.0)
    layers = [str(x) for x in (p.layers or ())]
    drill = float(getattr(p, 'drill', 0.0) or 0.0)
    if drill > 0 and getattr(p, 'pad_type', '') != 'np_thru_hole':
        face = 'T'
    elif any(x.startswith('B.') for x in layers) and not any(
            x.startswith('F.') or x.startswith('*') for x in layers):
        face = 'B'
    else:
        face = 'F'
    hole = None
    if (drill > 0 and getattr(p, 'hole_x', None) is not None
            and getattr(p, 'hole_y', None) is not None):
        hx, hy = _to_local(fp, p.hole_x, p.hole_y)
        hole = [round(hx, 5), round(hy, 5)]
    polys = None
    if getattr(p, 'polygons', None):
        polys = [[[round(c, 5) for c in _to_local(fp, x, y)] for x, y in poly]
                 for poly in p.polygons if len(poly) >= 3] or None
    return [round(lx, 5), round(ly, 5), round(float(p.size_x), 5),
            round(float(p.size_y), 5), round(ang, 4),
            (p.shape or 'rect').lower(), face, round(drill, 4), hole, polys]


def part_geometry(fp, board_area=None) -> dict:
    """`{'pads': [...], 'body': [x0, y0, x1, y1, h] | None}` in the part's
    frame. No body for a part with no pads (a hole, a fiducial, a logo) or
    one whose pad field is a large share of the board."""
    pads = [_local_pad(fp, p) for p in fp.pads
            if getattr(p, 'pad_type', '') != 'np_thru_hole']
    if pads:
        x0 = min(p[0] - p[2] / 2 for p in pads)
        x1 = max(p[0] + p[2] / 2 for p in pads)
        y0 = min(p[1] - p[3] / 2 for p in pads)
        y1 = max(p[1] + p[3] / 2 for p in pads)
    else:
        return {'pads': pads, 'body': None}
    # the body sits INSIDE the pad field, the way a package does
    w, h = x1 - x0, y1 - y0
    if board_area and w * h > BODY_MAX_BOARD_FRAC * board_area:
        return {'pads': pads, 'body': None}
    ix, iy = min(0.15 * w, 0.4), min(0.15 * h, 0.4)
    h3 = min(BODY_H_MAX, max(BODY_H_MIN, BODY_H_FRAC * min(w, h)))
    return {'pads': pads,
            'body': [round(x0 + ix, 4), round(y0 + iy, 4),
                     round(x1 - ix, 4), round(y1 - iy, 4), round(h3, 3)]}


def _not_assembled(pcb) -> set:
    """Parts that have no body to draw: they declare no 3D model AND KiCad
    says nothing is placed there (`board_only`, or `exclude_from_pos_files`
    -- no pick-and-place). StickHub's J1 is a USB-A plug MADE of board
    copper, H1 a plain hole, JP1 a solder jumper; a box drawn on the plug's
    tongue read as a part that is not there (run 36). A part with no model
    that IS assembled keeps its box, as the user chose. Read off the
    parser's `Footprint.attrs` / `has_model` (#1098), the same facts the
    placement tools use to keep that tongue clear."""
    return {ref for ref, fp in (getattr(pcb, 'footprints', None) or {}).items()
            if not getattr(fp, 'has_model', False)
            and set(getattr(fp, 'attrs', ()) or ())
            & {'board_only', 'exclude_from_pos_files'}}


def build_scene(pcb) -> dict:
    """The static scene from the film's FINAL board."""
    bi = pcb.board_info
    bb = bi.board_bounds or (0.0, 0.0, 100.0, 100.0)
    outline = [list(map(float, pt)) for pt in (bi.board_outline or [])]
    if len(outline) < 3:
        x0, y0, x1, y1 = bb
        outline = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
    cutouts = [[list(map(float, pt)) for pt in ring]
               for ring in (getattr(bi, 'board_cutouts', None) or [])
               if len(ring) >= 3]
    parts = {}
    area = max(1e-6, (bb[2] - bb[0]) * (bb[3] - bb[1]))
    bodiless = _not_assembled(pcb)
    for ref, fp in pcb.footprints.items():
        if ref.startswith('#'):
            continue
        g = part_geometry(fp, area)
        if ref in bodiless:
            g['body'] = None
        g['side'] = 'B' if (fp.layer or '').startswith('B') else 'F'
        parts[ref] = g
    # #1090: the plane pours, each shown from the frame its net's fill is
    # revealed (the film's `revealed_zones`, per frame in the timeline).
    # Like the 2D film, the zone OUTLINE -- not the computed fill.
    li = {n: i for i, n in enumerate(bi.copper_layers)}
    pours = [{'net': int(z.net_id), 'layer': li[z.layer],
              'poly': [[float(x), float(y)] for x, y in z.polygon]}
             for z in (getattr(pcb, 'zones', None) or [])
             if z.layer in li and len(z.polygon) >= 3]
    return {'bounds': [float(v) for v in bb], 'outline': outline,
            'cutouts': cutouts, 'thickness': board_thickness(pcb),
            'layers': list(bi.copper_layers), 'parts': parts, 'glb': None,
            'pours': pours}


# ---------------------------------------------------------------------------
# the optional real models
# ---------------------------------------------------------------------------
_MODEL_RE = re.compile(r'(\(model\s+")([^"]+)(")')


def _resolve(path, dirs) -> str:
    def sub(m):
        return dirs.get(m.group(1) or m.group(2), m.group(0))
    return re.sub(r'\$\{(\w+)\}|\$\((\w+)\)', sub, path)


#: A missing `.wrl`'s twins, in the order tried: KiCad's own libraries
#: ship `.step`, a vendor's may spell it `.STEP` or `.stp`.
TWIN_EXTS = ('.step', '.STEP', '.stp', '.STP')


#: Every model file under KiCad's library dirs, by lower-case stem, built
#: once per process: {stem: [paths]}.
_INDEX = {}


def _library_index(dirs) -> Dict[str, List[str]]:
    """The model files under the LIBRARY dirs in `dirs` (not KIPRJMOD, the
    board's own folder), by lower-case stem, each list sorted so a lookup is
    deterministic."""
    roots = sorted({v for k, v in dirs.items()
                    if k != 'KIPRJMOD' and v and os.path.isdir(v)})
    key = tuple(roots)
    if key not in _INDEX:
        idx = {}
        for root in roots:
            for d, _sub, files in os.walk(root):
                for fn in files:
                    stem, ext = os.path.splitext(fn)
                    if ext.lower() in ('.step', '.stp', '.wrl', '.vrml'):
                        idx.setdefault(stem.lower(), []).append(
                            os.path.join(d, fn))
        for v in idx.values():
            v.sort()
        _INDEX[key] = idx
    return _INDEX[key]


def _by_name(path, dirs):
    """A model the reference names but whose path does not exist, found by
    its file NAME in the library dirs, `.step` preferred. Old boards (and
    Olimex's) spell `${KISYS3DMOD}/R_0402_1005Metric.wrl` FLAT, where KiCad
    10 keeps `3dmodels/Resistor_SMD.3dshapes/R_0402_1005Metric.step`:
    esp_prog's 16 references found 0 models that way (run 35)."""
    stem = os.path.splitext(os.path.basename(path.replace('\\', '/')))[0]
    hits = _library_index(dirs).get(stem.lower()) or []
    for ext in ('.step', '.stp', '.wrl', '.vrml'):
        for h in hits:
            if h.lower().endswith(ext):
                return h
    return None


def stage_models(board_text, dirs, board_dir=None) -> Tuple[str, int, int]:
    """Make every model reference survive the board being re-written into a
    temp dir, and point a missing `.wrl` at an existing `.step` twin.

    The staged copy lives in a temp dir, so `${KIPRJMOD}` and a RELATIVE
    path -- both resolved against the board's OWN directory (`board_dir`) --
    would point at nothing there (the phase-5 verifier: ulx3s, orangecrab,
    watchy and tigard use KIPRJMOD, glasgow `../../packages3D/`). Every
    reference that resolves is therefore rewritten to its ABSOLUTE path.
    A reference that still resolves to nothing is looked up by its file name
    in the library dirs (`_by_name`), last.
    Returns `(text, n_twins, n_named)`: how many `.wrl` were pointed at a
    twin, and how many references were found by name."""
    n = [0, 0]

    def fix(m):
        path = m.group(2)
        real = _resolve(path, dirs)
        if board_dir and not os.path.isabs(real) and '${' not in real:
            real = os.path.normpath(os.path.join(board_dir, real))
        if os.path.isfile(real):
            return m.group(1) + real.replace('\\', '/') + m.group(3)
        if path.lower().endswith(('.wrl', '.vrml')):
            stem = os.path.splitext(real)[0]
            for ext in TWIN_EXTS:
                if os.path.isfile(stem + ext):
                    n[0] += 1
                    return (m.group(1) + (stem + ext).replace('\\', '/')
                            + m.group(3))
        found = _by_name(path, dirs)
        if found:
            n[1] += 1
            return m.group(1) + found.replace('\\', '/') + m.group(3)
        return m.group(0)
    return _MODEL_RE.sub(fix, board_text), n[0], n[1]


def _glb_node_names(glb_path):
    import json
    import struct
    with open(glb_path, 'rb') as f:
        magic, _ver, _len = struct.unpack('<III', f.read(12))
        if magic != 0x46546C67:
            return []
        clen, _ctype = struct.unpack('<II', f.read(8))
        doc = json.loads(f.read(clen))
    return [n.get('name', '') for n in doc.get('nodes', [])]


def export_glb(board_path, pcb, out_dir, cli=None,
               timeout=GLB_TIMEOUT_S) -> Tuple[Optional[dict], str]:
    """`(glb_info, why)`: the parts' real models as one GLB, or None and why.

    `glb_info` = `{'path', 'matched': [refs], 'poses': {ref: [x, y, rot,
    side]}}` -- the poses are the FINAL board's, which is what kicad-cli
    exported the nodes at, so the page can re-pose them."""
    try:
        import kicad_iso_render as kir
    except Exception as exc:                                   # noqa: BLE001
        return None, 'no kicad-cli resolver (%s)' % exc
    cli, why = kir.resolve_cli(cli) if cli is None else (cli, '')
    if not cli:
        return None, why or 'no kicad-cli'
    tmp = tempfile.mkdtemp(prefix='stage3d_glb_')
    try:
        src = open(board_path, encoding='utf-8').read()
        text, nfix, nnamed = stage_models(
            src, kir.model_dirs(cli, board_path),
            os.path.dirname(os.path.abspath(board_path)))
        staged = os.path.join(tmp, os.path.basename(board_path))
        with open(staged, 'w', encoding='utf-8') as f:
            f.write(text)
        pro = os.path.splitext(board_path)[0] + '.kicad_pro'
        if os.path.isfile(pro):
            shutil.copy(pro, os.path.splitext(staged)[0] + '.kicad_pro')
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, 'parts.glb')
        argv = [cli, 'pcb', 'export', 'glb', '--no-dnp', '--subst-models',
                '--no-board-body', '-f', '-o', out, staged]
        try:
            r = subprocess.run(
                argv, capture_output=True, text=True, encoding='utf-8',
                errors='replace', timeout=timeout,
                **({'creationflags': subprocess.CREATE_NO_WINDOW}
                   if sys.platform.startswith('win') else {}))
        except subprocess.TimeoutExpired:
            return None, 'kicad-cli pcb export glb timed out after %gs' % (
                timeout)
        except OSError as exc:
            return None, 'could not run kicad-cli (%s)' % exc
        if r.returncode != 0 or not os.path.isfile(out):
            blob = (r.stderr or r.stdout or '').strip()
            head = blob.splitlines()[0][:160] if blob else ''
            return None, 'kicad-cli pcb export glb exited %s%s' % (
                r.returncode, (': ' + head) if head else '')
        names = set(_glb_node_names(out))
        # a duplicate reference's later blocks are keyed `REF~2`... by the
        # parser, while kicad-cli names every node by the bare reference
        refs = [ref for ref in pcb.footprints
                if ref.split('~')[0] in names]
        poses = {ref: [fp.x, fp.y, fp.rotation or 0.0,
                       'B' if (fp.layer or '').startswith('B') else 'F']
                 for ref, fp in pcb.footprints.items() if ref in refs}
        n = sum(1 for ref in pcb.footprints if not ref.startswith('#'))
        return ({'path': out,
                 'matched': sorted({r.split('~')[0] for r in refs}),
                 'poses': poses},
                'GLB: %d of %d parts have a model%s%s'
                % (len(refs), n, (' (%d .wrl -> .step)' % nfix)
                   if nfix else '',
                   (' (%d found by file name in the library)' % nnamed)
                   if nnamed else ''))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def pose_matrix(x, y, rot, side, thickness) -> Dict[str, list]:
    """The part frame `F` the page uses, for tests: translation and the
    rotation the GLB nodes carry (`Ry(rot)`, then `Rx(180)` on the back)."""
    return {'t': [x, thickness if side == 'F' else 0.0, y],
            'ry': math.radians(rot), 'rx': math.pi if side == 'B' else 0.0}
