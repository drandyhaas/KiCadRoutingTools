#!/usr/bin/env python3
"""#1462: a declared `heights[].underside_mm` wins over the "reached from
above" rule, a model's room that rule keeps solid is named, and a `heights[]`
row that names no part is named rather than dropped.

The board is written here: a 20 x 20 mm shield-can fence (12 SMD pads, a .Fab
outline, no courtyard, no model) with two 0603s and a QFN inside it, once with
the can designated like a connector (J7) and once not (SH1). No kicad-cli:
the measurement is replaced by nothing (`heights.EMPTY`), or by a synthetic
raster where a model's room is wanted.

  1. a declared underside gives J7 room under it, as it gives SH1, and the
     notice says it was applied over the reached rule (SH1 has none);
  2. a model's room under J7 is kept solid, named, and J7 is no
     "room under" cover; a declaration then replaces the notice;
  3. a row naming a part not on the board, a glob matching nothing: named;
     a glob applies to the parts it matches, a literal row outranks it;
  4. the readers print it: the placement state (place_simple, place_pose,
     check_floorplan) and check_assembly's grade;
  5. the design brief does not call a glob that matches parts "not on this
     board".

    python3 tests/test_1462_declared_underside.py
"""
import contextlib
import io
import math
import os
import sys
import tempfile
import uuid

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))

import numpy as np  # noqa: E402

from placement import heights  # noqa: E402

RUN_ALL_TIMEOUT = 300
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*a, **k)


def _fp(name, ref, x, y, pads, crt=None, fab=None):
    s = (f'\t(footprint "{name}"\n\t\t(layer "F.Cu")\n\t\t(uuid "{uuid.uuid4()}")\n'
         f'\t\t(at {x} {y} 0)\n\t\t(property "Reference" "{ref}"\n\t\t\t(at 0 0 0)\n'
         f'\t\t\t(layer "F.SilkS")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t\t(effects\n'
         '\t\t\t\t(font\n\t\t\t\t\t(size 1 1)\n\t\t\t\t\t(thickness 0.15)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n')
    for lay, h in (('F.CrtYd', crt), ('F.Fab', fab)):
        if h:
            s += (f'\t\t(fp_rect\n\t\t\t(start {-h[0]} {-h[1]})\n\t\t\t(end {h[0]} {h[1]})\n'
                  '\t\t\t(stroke\n\t\t\t\t(width 0.05)\n\t\t\t\t(type solid)\n\t\t\t)\n'
                  f'\t\t\t(fill no)\n\t\t\t(layer "{lay}")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n')
    for n, px, py, sx, sy in pads:
        s += (f'\t\t(pad "{n}" smd rect\n\t\t\t(at {px} {py})\n\t\t\t(size {sx} {sy})\n'
              f'\t\t\t(layers "F.Cu" "F.Paste" "F.Mask")\n\t\t\t(uuid "{uuid.uuid4()}")\n\t\t)\n')
    return s + '\t)\n'


def write_can(tmp, ref):
    """A 40 x 40 mm board with a can `ref` at (20, 20) holding C1, C2, U1."""
    fence = [('1', a * 6, b * 9.6, 3, 0.8) for a in (-1, 0, 1) for b in (-1, 1)]
    fence += [('1', b * 9.6, a * 6, 0.8, 3) for a in (-1, 0, 1) for b in (-1, 1)]
    cap = [('1', -0.8, 0, 0.9, 0.95), ('2', 0.8, 0, 0.9, 0.95)]
    qfn = [(str(i + 1), -1.5 + i, 1.4, 0.4, 0.8) for i in range(4)]
    parts = (_fp('Test:ShieldCan_20x20mm', ref, 20, 20, fence, fab=(10, 10))
             + _fp('Test:C_0603', 'C1', 15, 15, cap, crt=(1.5, 0.75))
             + _fp('Test:C_0603', 'C2', 25, 15, cap, crt=(1.5, 0.75))
             + _fp('Test:QFN_4x4', 'U1', 20, 24, qfn, crt=(2.2, 2.2)))
    path = os.path.join(tmp, f'{ref}.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n'
                '\t(generator_version "10.0")\n\t(general\n\t\t(thickness 1.6)\n\t)\n'
                '\t(paper "A4")\n\t(layers\n\t\t(0 "F.Cu" signal)\n\t\t(2 "B.Cu" signal)\n'
                '\t\t(13 "F.Paste" user)\n\t\t(1 "F.Mask" user)\n\t\t(3 "B.Mask" user)\n'
                '\t\t(5 "F.SilkS" user "F.Silkscreen")\n\t\t(25 "Edge.Cuts" user)\n'
                '\t\t(31 "F.CrtYd" user "F.Courtyard")\n\t\t(35 "F.Fab" user)\n\t)\n'
                '\t(setup\n\t\t(pad_to_mask_clearance 0)\n\t)\n\t(net 0 "")\n' + parts
                + '\t(gr_rect\n\t\t(start 0 0)\n\t\t(end 40 40)\n\t\t(stroke\n\t\t\t(width 0.1)\n'
                  f'\t\t\t(type default)\n\t\t)\n\t\t(fill no)\n\t\t(layer "Edge.Cuts")\n'
                  f'\t\t(uuid "{uuid.uuid4()}")\n\t)\n)\n')
    return path


def lines(out):
    """The `heights:` lines of a reader's output."""
    return ' | '.join(ln.strip() for ln in out.splitlines() if 'heights' in ln)


def declared(ref, extra=()):
    rows = [{'ref': ref, 'underside_mm': 3.0, 'source': 'test'}]
    rows += [{'ref': r, 'height_mm': 0.9, 'source': 'test'} for r in ('C1', 'C2', 'U1')]
    return {'covers': [], 'heights': rows + list(extra)}


def modelled(pcb, path, ref, under=3.0):
    """A measurement with `ref`'s model `under` mm over its whole occupancy
    and C1, C2, U1 0.9 mm tall."""
    from placement.body import board_bodies
    occ = board_bodies(pcb, path)[ref].occupancy_local
    c = heights.CELL_MM
    x0, y0 = math.floor(occ[0] / c) * c, math.floor(occ[1] / c) * c
    nx = int(math.ceil((occ[2] - x0) / c)) + 1
    ny = int(math.ceil((occ[3] - y0) / c)) + 1
    cm = np.full((nx, ny), round(under * 100), '<u2')
    raw = {'top': {ref: 4.0, 'C1': 0.9, 'C2': 0.9, 'U1': 0.9},
           'covers': {ref: [x0, y0, nx, ny, cm.tobytes().hex()]}, 'bodies': {}}
    return heights._load(raw, pcb, path, {})


def pair(h, pcb, a, b):
    from placement.body import board_bodies
    geo = board_bodies(pcb, pcb.source_path)
    fa, fb = pcb.footprints[a], pcb.footprints[b]
    return h.pair(a, (fa.x, fa.y, fa.rotation or 0.0), geo[a].occupancy_local, 'F',
                  b, (fb.x, fb.y, fb.rotation or 0.0), geo[b].occupancy_local, 'F', 0.1)


def main():
    from kicad_parser import parse_kicad_pcb
    from placement.legality import grade_body_overlap
    from placement.quench import QuenchState
    tmp = tempfile.mkdtemp(prefix='t1462_')
    boards = {}
    for r in ('J7', 'SH1'):
        path = write_can(tmp, r)
        boards[r] = quiet(parse_kicad_pcb, path)
        boards[r].source_path = path

    print('1. a declared underside wins over the reached rule')
    for ref in ('J7', 'SH1'):
        pcb = boards[ref]
        h = heights.with_declared(heights.EMPTY, declared(ref), pcb, pcb.source_path)
        cov = h.covers.get(ref)
        check(f'{ref}: a cover, not solid, offered as room', cov is not None and not cov.solid
              and h.room_refs() == [ref], f'{h.room_refs()}')
        check(f'{ref}: C1 and U1 fit under it', pair(h, pcb, ref, 'C1') == 0.0
              and pair(h, pcb, ref, 'U1') == 0.0)
        want = (['J7: declared underside_mm 3 applied, though it is reached from above '
                 '(reference prefix J)'] if ref == 'J7' else [])
        check(f'{ref}: the notice', list(h.notices) == want, f'{h.notices}')

    print('2. a model\'s room on a reached part is kept solid, and named')
    pcb = boards['J7']
    h = modelled(pcb, pcb.source_path, 'J7')
    check('J7 stays solid and is offered as no room',
          'J7' not in h.room_refs() and (h.covers.get('J7') is None or h.covers['J7'].solid),
          f'{h.room_refs()}')
    check('...C1 under it is refused', (pair(h, pcb, 'J7', 'C1') or 0) > 0.0)
    check('...and the reader is told why and how to override',
          list(h.notices) == ['J7: model room under it not applied: reached from above '
                              '(reference prefix J) -- declare heights[].underside_mm to '
                              'override'], f'{h.notices}')
    check('...and the note counts no room', '0 part(s) with room under them' in h.note, h.note)
    hs = modelled(boards['SH1'], boards['SH1'].source_path, 'SH1')
    check('control: SH1 with the same model has room and no notice',
          hs.room_refs() == ['SH1'] and not hs.notices, f'{hs.room_refs()} {hs.notices}')
    hd = heights.with_declared(h, declared('J7'), pcb, pcb.source_path)
    check('a declaration replaces the set-aside notice with the applied one',
          hd.room_refs() == ['J7'] and len(hd.notices) == 1 and 'applied' in hd.notices[0]
          and pair(hd, pcb, 'J7', 'C1') == 0.0, f'{hd.notices}')

    print('3. rows that name no part, and globs')
    pcb = boards['SH1']
    rows = {'covers': [], 'heights': [
        {'ref': 'SH1', 'underside_mm': 3.0, 'source': 't'},
        {'ref': 'C*', 'height_mm': 0.9, 'source': 't'},
        {'ref': 'C2', 'height_mm': 1.2, 'source': 't'},
        {'ref': 'C9', 'height_mm': 0.9, 'source': 't'},
        {'ref': 'Q?', 'height_mm': 0.9, 'source': 't'}]}
    h = heights.with_declared(heights.EMPTY, rows, pcb, pcb.source_path)
    check('a glob applies to the parts it matches', h.height('C1') == 0.9, f'{h.top}')
    check('...and a literal row outranks it, wherever it is listed', h.height('C2') == 1.2)
    check('a reference not on the board and a glob matching nothing are named',
          list(h.notices) == [
              'heights[3] names C9, which is not on this board -- the row is not applied',
              "heights[4] 'Q?' matches no part on this board -- the row is not applied"],
          f'{h.notices}')

    print('4. the readers print it')
    real = heights._for_board
    try:
        heights._for_board = lambda *_a, **_k: heights.EMPTY
        for ref in ('J7', 'SH1'):
            pcb = boards[ref]
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                QuenchState(pcb, pcb.source_path, 0.1, 0.25, declared_heights=declared(ref))
            out = lines(buf.getvalue())
            check(f'the placement state: room under {ref}', f'; room under {ref}' in out, out)
            check('...and the J7 notice only for J7',
                  ('heights: J7: declared underside_mm 3 applied' in out) == (ref == 'J7'))
        pcb = boards['J7']
        heights._for_board = lambda *_a, **_k: modelled(pcb, pcb.source_path, 'J7')
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            QuenchState(pcb, pcb.source_path, 0.1, 0.25)
        out = lines(buf.getvalue())
        check('a solid cover gets no "room under" line, and is named',
              '; room under' not in out
              and 'heights: J7: model room under it not applied' in out, out)
        g = quiet(grade_body_overlap, pcb, 0.1, pcb_file=pcb.source_path)
        check("check_assembly's grade carries the notice",
              any('J7: model room under it not applied' in n for n in g.get('height_notices', ())),
              f"{g.get('height_notices')}")
    finally:
        heights._for_board = real
        heights._DECL_MEMO.clear()

    print('5. the design brief and globs')
    from placement.design_brief import brief_from_dict, compile_brief
    b = brief_from_dict({'schema': 1, 'kind': 'design-brief', 'units': 'mm',
                         'heights': [r for r in rows['heights']]})
    _frag, rep = compile_brief(b, board_refs=['SH1', 'C1', 'C2', 'U1'])
    check('the brief names C9 and Q?, not C*', rep['unmatched'] == ['C9', 'Q?'],
          f"{rep['unmatched']}")

    print('6. a declared underside is room however low; a measured one is floored (#1475)')
    pcb = boards['SH1']
    short = [{'ref': r, 'height_mm': 0.5, 'source': 't'} for r in ('C1', 'C2', 'U1')]
    for under, fits in ((1.6, True), (1.45, True), (1.2, True), (0.9, False)):
        heights._DECL_MEMO.clear()
        h = heights.with_declared(heights.EMPTY, {'covers': [], 'heights': short + [
            {'ref': 'SH1', 'underside_mm': under, 'source': 't'}]}, pcb, pcb.source_path)
        got = pair(h, pcb, 'SH1', 'C1')
        check(f'declared underside {under:g} over a 0.5 mm part: '
              f"{'fits' if fits else 'refused (the 0.5 mm margin)'}",
              (got == 0.0) == fits, f'blocked {got}')
    heights._DECL_MEMO.clear()
    h = heights.with_declared(modelled(pcb, pcb.source_path, 'SH1', under=1.2),
                              {'covers': [], 'heights': short}, pcb, pcb.source_path)
    got = pair(h, pcb, 'SH1', 'C1')
    check('a MEASURED 1.2 mm underside keeps the floor: refused', got > 0.0,
          f'blocked {got}')

    print('7. a height-limited keep-out over a zone binds the members too tall for it (#1475)')
    from placement import floorplan as fp
    path = pcb.source_path
    for h_mm, flagged in ((0.9, False), (2.5, True), (None, True)):
        doc = {'schema': fp.SCHEMA_VERSION, 'kind': fp.KIND, 'units': 'mm',
               'min_reader': fp.COVERS_READER,
               'blocks': [{'name': 'b', 'refs': ['C1', 'C2', 'U1'], 'zone': [11, 11, 29, 29],
                           'side': 'F'}],
               'keepouts': [{'name': 'lid', 'rect': [11, 11, 29, 29], 'sides': ['F'],
                             'max_height_mm': 2.0, 'allow': ['SH1']}],
               'heights': ([{'ref': r, 'height_mm': h_mm, 'source': 't'}
                            for r in ('C1', 'C2', 'U1')] if h_mm else [])}
        it = fp.intent_from_dict(doc, 't.json')
        heights._DECL_MEMO.clear()
        r = quiet(fp.grade, it, pcb, path)
        vs = r.errors + r.warnings
        zone = sorted(v.ref or v.block for v in vs if v.rule == 'intent_zone_in_keepout')
        keep = sorted(v.ref for v in vs if v.rule == 'keepout')
        label = 'unknown' if h_mm is None else f'{h_mm:g} mm'
        check(f'members {label} under a 2.0 mm lid: the zone check agrees with the '
              f"keepout rule ({'flagged' if flagged else 'admitted'})",
              bool(zone) == flagged and bool(keep) == flagged,
              f'intent_zone_in_keepout {zone}, keepout {keep}')

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
