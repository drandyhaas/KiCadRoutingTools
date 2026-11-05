#!/usr/bin/env python3
"""#1662: on a pile, `--emit-intent` gives an unlocked edge-mount part the
overhang its own 3D model needs, read in the part's own frame.

The emit's #1603 floor stood behind the pile's `continue`, so a plain emit wrote
no row and `--declare-classes` a band of `min 0`, and the edge written by hand
seated the part wholly on the board on every seed. check_assembly's row then
printed the distance to the nearest outline, which written as `overhang_mm.min`
still gated. With #1603's synthetic body measurement (no KiCad needed):

  1. off the pile the part gets the floor the placed board's emit gives it, at
     any staging pose, with no edge (#1103), with or without --declare-classes,
     labelled `derived_default`;
  2. written with its edge, that floor keeps the body off the board, and the
     depth the row prints would not;
  3. the OWED line says it, until an edge and a band reaching it, or a `why`,
     or a declared pose answers it; nothing on a placed board, or with no
     model read;
  4. a part with no mating face gets the floors of its sides, and no row; a
     part carrying its own Edge.Cuts is not asked; a brief's declared pose or
     mechanical.json's outranks the row, so the loader does not refuse the
     intent; a brief's edge keeps the floor;
  5. check_assembly prints the overhang under a gating row.

    python3 tests/test_1662_pile_body_floor.py
"""
import contextlib
import copy
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(REPO, _p))
sys.path.insert(0, TESTS)

import run_utils  # noqa: E402
from placement import heights  # noqa: E402
from test_1603_body_in_board import (DESIGNED, FIXTURE, PLACED, T, assembly, at,  # noqa: E402
                                     box, low, quiet, synthetic)

RUN_ALL_TIMEOUT = 600
READING = 11.93     # J3's band reading at the pose it is drawn at
TOL = 0.1           # how far inside the edge its body may stand (#1659)
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def row_of(doc, ref='J3'):
    return next((c for c in doc.get('edge_connectors') or () if c.get('ref') == ref), None)


def emit(path, make_h, **kw):
    from kicad_parser import parse_kicad_pcb
    from placement.floorplan import emit_intent
    pcb = quiet(parse_kicad_pcb, path)
    real = heights.for_board
    heights.for_board = lambda *_a, **_k: make_h(pcb)
    try:
        return pcb, quiet(emit_intent, pcb, path, **kw)
    finally:
        heights.for_board = real


def restage(pile, tmp, name, x, y, rot):
    """The pile with J3 staged at another pose."""
    with open(pile, encoding='utf-8', newline='') as f:
        t = f.read()
    m = re.search(r'\(footprint "Connector_Coaxial:SMA_Amphenol[^\n]*\n(?:[^\n]*\n)*?\s*\(at [^)]*\)', t)
    assert m, 'J3 block'
    head = re.sub(r'\(at [^)]*\)$', f'(at {x} {y} {rot})', m.group(0))
    path = os.path.join(tmp, name)
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(t[:m.start()] + head + t[m.end():])
    return path


def owed(pcb, path, h, doc):
    from placement import floorplan
    return floorplan.body_floor_owed(floorplan.intent_from_dict(doc, 'test'), pcb, path, h)


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    tmp = tempfile.mkdtemp(prefix='t1662_')
    try:
        pile = os.path.join(tmp, 'pile.kicad_pcb')
        run_utils.check([sys.executable, '-X', 'utf8', '-B',
                         os.path.join(REPO, '.claude/skills/plan-pcb-placement/scripts/make_unplaced.py'),
                         FIXTURE, pile], accept=True)
        run_utils.evidence(pile, 'the pile')

        print('the floor off a pile:')
        _pcb, placed = emit(FIXTURE, synthetic)
        want = row_of(placed)['overhang_mm']
        check('the placed board\'s emit gives J3 its floor, as #1603 reads it',
              abs(want['min'] - (READING - TOL)) < 0.002, str(want))
        pcb, plain = emit(pile, synthetic)
        row = row_of(plain) or {}
        check('a plain emit off the pile writes the same band, with no edge',
              row.get('overhang_mm') == want and 'edge' not in row, str(row))
        check('...and the note says what it is and that no edge is declared',
              '3D model' in (row.get('note') or '') and 'No edge is read' in row['note'],
              row.get('note'))
        basis = plain['context']['basis']
        check('...derived_default, the band as well as its floor: nothing was observed',
              basis.get('edge_connectors[J3].overhang_mm') == 'derived_default'
              and basis.get('edge_connectors[J3].overhang_mm.min') == 'derived_default',
              str({k: v for k, v in basis.items() if 'J3' in k}))
        _p, cls = emit(pile, synthetic, declare_classes=True)
        row = row_of(cls) or {}
        check('--declare-classes writes that floor with the class, and no edge',
              row.get('overhang_mm') == want and row.get('class') == 'edge_receptacle'
              and 'edge' not in row, str(row))
        for name, x, y, rot in (('far', 120.0, -40.0, 0), ('turned', 18.0, 15.0, 90),
                                ('back', 60.0, 60.0, 180)):
            p = restage(pile, tmp, name + '.kicad_pcb', x, y, rot)
            _p, d = emit(p, synthetic)
            check(f'staged {name}: the same floor',
                  (row_of(d) or {}).get('overhang_mm') == want and 'edge' not in row_of(d),
                  str(row_of(d)))

        print('what the floor buys, and what the printed depth does not:')
        h = synthetic(quiet(parse_kicad_pcb, FIXTURE))
        pcb0 = quiet(parse_kicad_pcb, FIXTURE)

        def seated(band):
            return (DESIGNED[0] + band - READING, DESIGNED[1], 0.0)
        c = heights.body_in_board(h, pcb0, {'J3': seated(want['min'])})
        check('seated at that minimum the body does not gate', not c['gating_refs'], str(c['rows']))
        c = heights.body_in_board(h, pcb0, {'J3': PLACED})
        printed = next(r for r in c['rows'] if r['ref'] == 'J3')['inside_mm']
        c = heights.body_in_board(h, pcb0, {'J3': seated(printed)})
        check(f'...and the {printed} mm the gate prints off the placed pose, seated as a '
              f'minimum, still does', c['gating_refs'] == ['J3'], str(c['rows']))

        print('the OWED line:')
        h_pile = synthetic(pcb)
        lines = owed(pcb, pile, h_pile, plain)
        one = next((ln for ln in lines if ln.startswith('J3 ')), '')
        check('it names the part, its body and the overhang it needs',
              'board depth' in one and f"{want['min']:.3f}mm" in one and '+x' in one, one)
        ans = copy.deepcopy(plain)
        row_of(ans)['edge'] = 'east'
        check('an edge with that band answers it', not owed(pcb, pile, h_pile, ans), '')
        low_band = copy.deepcopy(ans)
        row_of(low_band)['overhang_mm'] = {'min': 0.0, 'max': 0.5}
        one = next((ln for ln in owed(pcb, pile, h_pile, low_band) if ln.startswith('J3 ')), '')
        check('a band under it is asked about, and says so',
              'declares a minimum of 0.000mm on the east edge' in one, one)
        why = copy.deepcopy(plain)
        row_of(why)['why'] = 'over a slot the outline will carry'
        check('a `why` answers it', not owed(pcb, pile, h_pile, why))
        pcb_placed = quiet(parse_kicad_pcb, FIXTURE)
        check('a placed board owes nothing',
              not owed(pcb_placed, FIXTURE, synthetic(pcb_placed), placed))
        check('no model read owes nothing, and writes no floor',
              not owed(pcb, pile, heights.Heights(note='no heights'), plain)
              and row_of(emit(pile, lambda _p: heights.Heights(note='no heights'))[1]) is None)

        print('the band\'s shape:')
        class Stub:
            @staticmethod
            def rect(_ref, _pose):
                return (-1.0, -1.0, 13.0, 1.0), 'F.Fab', ''
        pc = quiet(parse_kicad_pcb, pile)
        got = floorplan.body_clearing_overhang(synthetic(pc), pc, pile, 'J3', geometry=Stub)
        check('it reads the drawn body where one can be read',
              got and abs(got['min'] - (13.0 - 2.54 - TOL)) < 0.002, str(got))

        print('who is not asked for an edge:')
        lip = heights._depth_raster(low(box(-3.0, 3.0, 1.0, 1.5, -1.2, 0.5)), T)
        extra = lambda p: synthetic(p, extra={'C1': lip})   # noqa: E731
        pc, doc = emit(pile, extra)
        check('a part with no mating face gets no row', row_of(doc, 'C1') is None)
        one = next((ln for ln in owed(pc, pile, extra(pc), doc) if ln.startswith('C1 ')), '')
        check('...and its OWED line gives each side', all(f in one for f in ('+x', '-x', '+y', '-y')),
              one)
        pc = quiet(parse_kicad_pcb, pile)
        pc.footprints['J3'].owns_edge_cuts = True
        check('a part carrying its own Edge.Cuts is not asked',
              not [ln for ln in owed(pc, pile, synthetic(pc), plain) if ln.startswith('J3 ')])
        frag = {'fixed_poses': [{'ref': 'J3', 'x': 28.4, 'y': 15.0, 'rot': 0, 'basis': 'declared'}]}
        _p, doc = emit(pile, synthetic, brief_fragment=frag)
        check("a brief's declared pose writes no row", row_of(doc) is None)
        from placement import design_brief
        frag = {'edge_connectors': [{'ref': 'J3', 'edge': 'east'}]}
        report = dict.fromkeys(('path', 'declared', 'unknown', 'absent', 'not_graded', 'unmatched',
                                'unmatched_checked', 'counts', 'fixed', 'product'))
        report['contradictions'] = []
        merged = design_brief.merge_into_intent(copy.deepcopy(plain), frag, report)
        row = row_of(merged) or {}
        check("a brief's edge keeps the floor", row.get('edge') == 'east'
              and row.get('overhang_mm') == want, str(row))

        mech = os.path.join(tmp, 'mech.json')
        with open(mech, 'w', encoding='utf-8') as f:
            json.dump({'schema': 1, 'fixed': [{'ref': 'J3', 'x': 28.36, 'y': 15.0, 'rot': 0,
                                               'reason': 'enclosure'}]}, f)
        out = os.path.join(tmp, 'mech_intent.json')
        import check_floorplan
        real = heights.for_board
        heights.for_board = lambda *_a, **_k: synthetic(quiet(parse_kicad_pcb, pile))
        try:
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                check_floorplan.main([pile, '--emit-intent', out, '--allow-unplaced',
                                      '--mechanical', mech])
        finally:
            heights.for_board = real
        run_utils.evidence(out, 'the intent with a mechanical pose')
        with open(out, encoding='utf-8') as f:
            doc = json.load(f)
        poses = [f.get('ref') for f in doc.get('fixed_poses') or ()]
        check("mechanical.json's pose outranks the floor row, and the intent loads",
              row_of(doc) is None and 'J3' in poses
              and floorplan.intent_from_dict(doc, 'test') is not None,
              str({'row': row_of(doc), 'poses': poses,
                   'skipped': doc['context'].get('mechanical', {}).get('fixed_skipped')}))

        print('check_assembly:')
        path = at(os.path.join(tmp, 'on_board.kicad_pcb'), PLACED)
        rc, doc, text = assembly(path, synthetic(quiet(parse_kicad_pcb, path)), tmp)
        lines = text.splitlines()
        i = next((k for k, ln in enumerate(lines) if ln.strip().startswith('J3 ')), None)
        under = lines[i + 1].strip() if i is not None and i + 1 < len(lines) else ''
        check('under a gating row: the depth is not the overhang, and what is',
              rc == 4 and 'not the overhang it needs' in under
              and f"+x {want['min']:g}mm" in under, under)
        row = next((r for r in doc['body_in_board'] if r['ref'] == 'J3'), {})
        check('...and in the JSON', row.get('clearing_overhang_mm') == {'+x': want['min']},
              str(row.get('clearing_overhang_mm')))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
