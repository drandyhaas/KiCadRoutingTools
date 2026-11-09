#!/usr/bin/env python3
"""#1603: a part whose own 3D model has body at board depth inside the outline
is not buildable, and the emitted intent keeps an edge-mount part's body off
the board.

The fixture is kicad_files/cap_chain.kicad_pcb (outline x 5..31, y 5..25)
with KiCad's stock Connector_Coaxial:SMA_Amphenol_132289_EdgeMount as J3, at
the pose its footprint draws for an edge-mount jack: its pads end on the east
edge and its flange and barrel, which the board slides into, hang past it.
Placed by place_simple from an intent emitted off that board (`min 0.0`), J3
landed wholly on the board, ~12 mm of its body through the laminate, and
check_floorplan and check_assembly both passed it.

Here, without KiCad, the measurement is synthetic -- J3's body as a box at
the depth its model has one -- and goes through the real pipeline:

  1. `heights._depth_raster`: a body through the board reads its AREA at
     board depth, not only its walls; a body seated on the face, a tab
     touching the bottom face and an open mesh's unpaired row read nothing;
  2. `_load` leaves out what lies on a part's own pads and drilled holes (a
     header's pins), and reads nothing when the export's frame is a guess;
  3. `body_in_board`: the designer's pose reads nothing, the pose
     place_simple chose gates, a cutout under the body reads nothing, and a
     body under `DEPTH_MIN_MM2` is listed and never gates;
  4. check_assembly: the placed pose is NOT BUILDABLE on the new conjunct
     alone, and the designer's pose is buildable;
  5. the emitted intent's `overhang_mm.min` is J3's own reading less how far
     it can move in before the gate reads it, labelled `derived_default`,
     and the designer's pose grades it clean.

The real kicad-cli measurement is tests/test_1603_body_in_board_kicad.py.

    python3 tests/test_1603_body_in_board.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, os.path.join(REPO, 'py_tools'))
sys.path.insert(0, TESTS)

import numpy as np  # noqa: E402

import run_utils  # noqa: E402
from placement import heights  # noqa: E402

RUN_ALL_TIMEOUT = 600
FIXTURE = os.path.join(TESTS, 'fixtures', '1603', 'edge_sma.kicad_pcb')
#: J3 as the designer seats it, and as place_simple seated it from `min 0`
DESIGNED = (28.46, 15.0, 0.0)
PLACED = (16.53, 10.2634, 0.0)
T = 1.6                       # the fixture's (general (thickness 1.6))
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def quiet(fn, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return fn(*a, **k)


def box(x0, x1, y0, y1, z0, z1, drop=()):
    """A closed box as (n, 3, 3) triangles (x, y, height over the face),
    less the faces named in `drop` ('x1' its +x wall, ...)."""
    v = {(i, j, k): (x, y, z) for i, x in enumerate((x0, x1))
         for j, y in enumerate((y0, y1)) for k, z in enumerate((z0, z1))}
    faces = {'x0': [(0, 0, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1)],
             'x1': [(1, 0, 0), (1, 0, 1), (1, 1, 1), (1, 1, 0)],
             'y0': [(0, 0, 0), (0, 0, 1), (1, 0, 1), (1, 0, 0)],
             'y1': [(0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1)],
             'z0': [(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)],
             'z1': [(0, 0, 1), (0, 1, 1), (1, 1, 1), (1, 0, 1)]}
    tris = []
    for name, q in faces.items():
        if name in drop:
            continue
        a, b, c, d = (v[k] for k in q)
        tris += [(a, b, c), (a, c, d)]
    return np.array(tris, float)


def low(q):
    """What `_measure` hands `_depth_raster`: the geometry reaching below the face."""
    return q[q[:, :, 2].min(1) < -heights.DEPTH_FACE_MM]


def area(r):
    if r is None:
        return 0.0
    m = np.frombuffer(bytes.fromhex(r[4]), 'u1')
    return float(m.sum()) * heights.CELL_MM ** 2


#: J3's body at board depth, as its model has it: own x 2.54..15.24 (its pads
#: end at 2.54), y -4.76..4.76, from 3.58 mm under its face to 4.34 over it
J3_BODY = box(2.54, 15.24, -4.76, 4.76, -3.58, 4.34)


def synthetic(pcb, frame_fits=1, extra=None):
    raw = {'top': {'J3': 4.34, 'J1': 8.54, 'J2': 8.54, 'C1': 0.5, 'C2': 0.5},
           'covers': {}, 'bodies': {},
           'depth': {'J3': heights._depth_raster(low(J3_BODY), T)},
           'frame_fits': frame_fits}
    raw['depth'].update(extra or {})
    return heights._load(raw, pcb, FIXTURE, {})


def unit(pcb):
    print('the board-depth raster:')
    a = area(heights._depth_raster(low(J3_BODY), T))
    want = 12.7 * 9.52
    check('a body through the board reads its area at board depth, not only its walls',
          want - 2 <= a <= want + 12, f'{a:.2f} mm^2 for a {want:.2f} mm^2 section')
    check('a body seated on the face reads nothing',
          heights._depth_raster(low(box(0, 5, 0, 5, 0.0, 3.0)), T) is None)
    check('a tab touching the bottom face reads nothing',
          heights._depth_raster(low(box(0, 5, 0, 5, -3.58, -T)), T) is None)
    a = area(heights._depth_raster(low(box(2.54, 15.24, -4.76, 4.76, -3.58, 4.34,
                                            drop=('x1',))), T))
    check('an open mesh does not paint the rows its crossings leave unpaired',
          0 < a < 0.25 * 12.7 * 9.52, f'{a:.2f} mm^2')

    print('the part\'s own pads and holes, and the frame:')
    j1 = pcb.footprints['J1']
    pin = np.concatenate([box(p.local_x - 0.32, p.local_x + 0.32, p.local_y - 0.32,
                              p.local_y + 0.32, -3.0, 8.5) for p in j1.pads])
    pin_r = heights._depth_raster(low(pin), T)
    h = synthetic(pcb, extra={'J1': pin_r})
    check("a header's pins through their own holes read nothing",
          area(pin_r) > 0 and 'J1' not in h.depth, f'{area(pin_r):.2f} mm^2 raw')
    check("...and J3's body, off its pads, reads its section",
          'J3' in h.depth and len(h.depth['J3']) * heights.CELL_MM ** 2 > 100,
          f"{len(h.depth.get('J3', ())) * heights.CELL_MM ** 2:.2f} mm^2")
    g = synthetic(pcb, frame_fits=4)
    c = heights.body_in_board(g, pcb, {'J3': PLACED})
    check('an export whose frame is a guess reads nothing, and says so',
          not g.depth and not c['rows'] and 'guess' in (c['unmeasured'] or ''),
          str(c['unmeasured']))

    print('the export\'s faces (`_faces`, world metres, up = axis 2):')
    def node(z):
        m = np.eye(4)
        m[2, 3] = z
        return m

    def part(z0, z1):
        return box(0, 0.001, 0, 0.001, z0, z1)
    # a front face at 1.6 mm, the back at 0; bodies stand out of their faces
    front = {'F1': part(0.0016, 0.0026), 'F2': part(0.0016, 0.0030), 'F3': part(0.0016, 0.0020)}
    back = {'B1': part(-0.0085, 0.0), 'B2': part(-0.0030, 0.0)}
    for name, tris, roots, side, want in (
            ('a board whose every model is on the back reads its front outward',
             back, {'B1': node(0.0), 'B2': node(0.0)}, {'B1': 'B', 'B2': 'B'},
             (1.0, None, 0.0)),
            ('a front model drawn through its board, first in order, does not turn it over',
             dict(front, A0=part(-0.0085, 0.0016)),
             dict({k: node(0.0016) for k in front}, A0=node(0.0016)),
             dict({k: 'F' for k in front}, A0='F'), (1.0, 0.0016, None)),
            ('a lone back part whose node carries its model offset keeps the true back face',
             dict(front, B9=part(-0.0085, 0.0015)),
             dict({k: node(0.0016) for k in front}, B9=node(0.0015)),
             dict({k: 'F' for k in front}, B9='B'), (1.0, 0.0016, 0.0))):
        got = heights._faces(sorted(roots), side, tris, roots, 2, 1000.0, T)
        ok = (got is not None and got[0] == want[0]
              and (want[1] is None or abs(got[1] - want[1]) < 1e-9)
              and (want[2] is None or abs(got[2] - want[2]) < 1e-9))
        check(name, ok, str(got))

    print('body_in_board:')
    c = heights.body_in_board(h, pcb, {'J3': DESIGNED})
    check("the designer's pose, the body starting on the edge, reads nothing",
          c['unmeasured'] is None and not c['rows'] and c['count'] == 0, str(c['rows']))
    c = heights.body_in_board(h, pcb, {'J3': PLACED})
    row = (c['rows'] or [{}])[0]
    check('the pose place_simple chose gates J3, about 12 mm of its body inside',
          c['gating_refs'] == ['J3'] and row.get('area_mm2', 0) > 100
          and 18.5 < row['box'][0] < 19.5 and 30.5 < row['box'][2] <= 31.0, str(row))
    c = heights.body_in_board(h, pcb, {'J3': (DESIGNED[0] - 0.05, DESIGNED[1], 0.0)})
    check('0.05 mm in from the designer\'s pose, inside the tolerance, does not gate',
          not c['gating_refs'], str(c['rows']))
    c = heights.body_in_board(h, pcb, {'J3': (DESIGNED[0] - 0.15, DESIGNED[1], 0.0)})
    check('...and 0.15 mm in does (#1659)', c['gating_refs'] == ['J3'], str(c['rows']))
    bi = pcb.board_info
    was = list(bi.board_cutouts or [])
    try:
        bi.board_cutouts = was + [[(18.8, 5.1), (30.95, 5.1), (30.95, 15.5), (18.8, 15.5)]]
        c = heights.body_in_board(h, pcb, {'J3': PLACED})
        check('a cutout under the body (a mid-mount part over its slot) reads nothing',
              not c['gating_refs'], str(c['rows']))
    finally:
        bi.board_cutouts = was
    small = heights._depth_raster(low(box(1.0, 2.0, 1.0, 2.0, -1.2, 0.5)), T)
    h2 = synthetic(pcb, extra={'C1': small})
    c = heights.body_in_board(h2, pcb)
    rows = {r['ref']: r for r in c['rows']}
    check('a model drawn low, under DEPTH_MIN_MM2, is listed and never gates',
          'C1' in rows and not rows['C1']['gates'] and 'C1' not in c['gating_refs'],
          str(rows.get('C1')))

    print('depth_past_edge:')
    d = heights.depth_past_edge(h, pcb, 'J3', 'east', DESIGNED)
    check("at the designer's pose J3 can move in by the tolerance before the gate reads it",
          d is not None and abs(d - heights.DEPTH_EDGE_TOL_MM) < 0.002, f'{d}')
    check('...and none of a body too small to gate',
          heights.depth_past_edge(h2, pcb, 'C1', 'east') is None)
    return h


def at(path, pose):
    """The fixture with J3 at `pose`, written to `path` (its siblings: none)."""
    with open(FIXTURE, encoding='utf-8') as f:
        text = f.read()
    assert text.count('(at 28.46 15)') == 1
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text.replace('(at 28.46 15)', f'(at {pose[0]:g} {pose[1]:g})'))
    return path


def assembly(path, h, tmp):
    """check_assembly in process, the synthetic measurement in for kicad-cli's
    (its --json written to `tmp`, never beside a committed fixture)."""
    import check_assembly
    out = os.path.join(tmp, os.path.basename(path) + '.json')
    if os.path.exists(out):
        os.remove(out)
    argv = sys.argv
    real = heights.for_board
    heights.for_board = lambda *_a, **_k: h
    try:
        sys.argv = ['check_assembly.py', path, '--json', out]
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(io.StringIO()):
            rc = check_assembly.main()
    finally:
        sys.argv = argv
        heights.for_board = real
    run_utils.evidence(out, 'check_assembly --json')
    with open(out, encoding='utf-8') as f:
        return rc, json.load(f), buf.getvalue()


def wiring(h, tmp):
    from kicad_parser import parse_kicad_pcb
    print('check_assembly:')
    placed = at(os.path.join(tmp, 'placed.kicad_pcb'), PLACED)
    hp = synthetic(quiet(parse_kicad_pcb, placed))
    rc, doc, text = assembly(placed, hp, tmp)
    check('the placed pose is NOT BUILDABLE, on the body-through-the-board conjunct',
          rc == 4 and doc['buildable'] is False and doc['body_in_board_count'] == 1
          and doc['blocking'] == 0 and 'BODY THROUGH THE BOARD' in text,
          f"rc {rc}, count {doc.get('body_in_board_count')}, blocking {doc.get('blocking')}")
    import board_score
    comp = board_score.assembly_component(doc, rc)
    check("...and board_score names it as the conjunct that fired",
          'body_in_board_count' in (comp.get('conjuncts_fired') or ()), str(comp.get('conjuncts_fired')))
    rc, doc, text = assembly(FIXTURE, h, tmp)
    check("the designer's pose is buildable, the conjunct measured at 0",
          rc == 0 and doc['buildable'] is True and doc['body_in_board_count'] == 0,
          f"rc {rc}, count {doc.get('body_in_board_count')}")
    rc, doc, text = assembly(placed, heights.Heights(note='no heights',
                                                     depth_note='no kicad-cli: no model was read'),
                             tmp)
    check('with no models read the conjunct is unmeasured (None), never 0, and does not gate',
          rc == 0 and doc['body_in_board_count'] is None
          and 'no kicad-cli' in (doc['body_in_board_unmeasured'] or ''),
          f"rc {rc}, {doc.get('body_in_board_count')!r}, {doc.get('body_in_board_unmeasured')!r}")

    print('the emitted intent:')
    from kicad_parser import parse_kicad_pcb as parse
    from placement.floorplan import emit_intent
    pcb = quiet(parse, FIXTURE)
    real = heights.for_board
    heights.for_board = lambda *_a, **_k: h
    try:
        doc = quiet(emit_intent, pcb, FIXTURE)
    finally:
        heights.for_board = real
    j3 = next((c for c in doc.get('edge_connectors') or () if c.get('ref') == 'J3'), {})
    band = j3.get('overhang_mm') or {}
    basis = (doc.get('context') or {}).get('basis') or {}
    d = heights.depth_past_edge(h, pcb, 'J3', 'east')
    check("J3's band floor is its reading less how far it can move in before the gate "
          "reads its body (about 11.7 mm, not 0)",
          j3.get('edge') == 'east' and 11.5 < band.get('min', 0) < 11.93
          and abs(band['min'] - (11.93 - d)) < 0.002 and band.get('max', 0) >= 11.93,
          f'{band}, past {d}')
    check('...labelled derived_default, the band itself observed',
          basis.get('edge_connectors[J3].overhang_mm.min') == 'derived_default'
          and basis.get('edge_connectors[J3].overhang_mm') == 'observed_baseline',
          str({k: v for k, v in basis.items() if 'J3' in k}))
    check('...and the note says where the floor came from',
          '3D model' in (j3.get('note') or ''), j3.get('note'))
    path = os.path.join(tmp, 'intent.json')
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(doc, f)
    run_utils.evidence(path, 'the emitted intent')
    import check_floorplan
    heights.for_board = lambda *_a, **_k: h
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(io.StringIO()):
            rc = check_floorplan.main([FIXTURE, '--intent', path])
        line = next((ln for ln in buf.getvalue().splitlines() if 'J3 east: overhang' in ln), '')
        check("the designer's pose grades the emitted band clean",
              rc == 0 and ' pass;' in line and 'overhang 11.93' in line, f'rc {rc}: {line.strip()}')
        hp = synthetic(quiet(parse, os.path.join(tmp, 'placed.kicad_pcb')))
        heights.for_board = lambda *_a, **_k: hp
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(io.StringIO()):
            rc = check_floorplan.main([os.path.join(tmp, 'placed.kicad_pcb'), '--intent', path])
        line = next((ln for ln in buf.getvalue().splitlines() if 'J3 east: overhang' in ln), '')
        check('...and the pose place_simple chose from `min 0` fails it',
              rc != 0 and 'overhang 0.0000mm' in line and ' fail;' in line,
              f'rc {rc}: {line.strip()}')
    finally:
        heights.for_board = real


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    from kicad_parser import parse_kicad_pcb
    pcb = quiet(parse_kicad_pcb, FIXTURE)
    tmp = tempfile.mkdtemp(prefix='t1603_')
    try:
        h = unit(pcb)
        wiring(h, tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
