#!/usr/bin/env python3
"""#1714: the #1603 floor row a pile's emit writes with no edge is graded as
the DISTANCE its band was read as, past the edge the side it was read along
points at, and the placed board's grade says the row still owes its edge.

The row's band is read off the part's 3D model along its mating face, in its
own frame (#1662), and is written with no `edge` (#1103). An edge-less row is
graded on #1634's summed reading, which on a non-rectangular outline adds
each off-board corner: with a 2 mm chamfer at the corner farthest from the
jack, the jack seated where the fixture draws it (11.93 mm past its edge)
read 35.79 mm and failed its own row. With #1603's synthetic body
measurement (no KiCad needed), on #1603's fixture and on that chamfered
outline:

  1. the pile's floor row carries `overhang_face`, the side its band was read
     along, and still no edge;
  2. graded against it, the part where the fixture draws it reads ~11.93 mm
     past the east edge on both outlines and passes, and the line says the
     row declares no edge and place_simple seats only a row with one;
  3. the same row without `overhang_face` keeps the summed reading (35.79 on
     the chamfer, the discriminator); with an `edge` the edge is read, the
     face aside; the face is read at the part's own turn;
  4. the loader refuses an `overhang_face` that is not a face, and a brief
     that replaces the band drops the face its band was read along.

    python3 tests/test_1714_edgeless_floor_distance.py
"""
import copy
import os
import shutil
import sys
import tempfile
from types import SimpleNamespace

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for _p in ('py_router', 'py_placer', 'py_tools'):
    sys.path.insert(0, os.path.join(REPO, _p))
sys.path.insert(0, TESTS)

import run_utils  # noqa: E402
from placement import heights  # noqa: E402
from test_1603_body_in_board import FIXTURE, quiet, synthetic  # noqa: E402
from test_1634_emit_overhang_distance import variants  # noqa: E402

RUN_ALL_TIMEOUT = 600
READING = 11.93     # J3's distance past the east edge where the fixture draws it
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def row_of(doc, ref='J3'):
    return next((c for c in doc.get('edge_connectors') or () if c.get('ref') == ref), None)


def with_heights(fn, pcb, *a, **k):
    real = heights.for_board
    heights.for_board = lambda *_a, **_k: synthetic(pcb)
    try:
        return quiet(fn, pcb, *a, **k)
    finally:
        heights.for_board = real


def pile_of(src, path):
    run_utils.check([sys.executable, '-X', 'utf8', '-B',
                     os.path.join(REPO, '.claude/skills/plan-pcb-placement/scripts/make_unplaced.py'),
                     src, path], accept=True)
    run_utils.evidence(path, 'the pile')
    return path


def graded(board, row):
    """`(J3's evidence row, its printed line, the ERRORs naming J3)` graded
    against an intent holding `row` alone."""
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    pcb = quiet(parse_kicad_pcb, board)
    it = floorplan.intent_from_dict({'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
                                     'units': 'mm', 'edge_connectors': [row]}, 'test')
    res = with_heights(lambda p, *a: floorplan.grade(it, p, board), pcb)
    ev = next((e for e in res.edge_connector_evidence if e['ref'] == 'J3'), {})
    line = next((ln.strip() for ln in floorplan.format_text(res).splitlines()
                 if ln.strip().startswith('J3 ') and 'overhang' in ln), '')
    errs = [v.message for v in res.violations if v.ref == 'J3' and v.severity == 'error'
            and 'overhangs' in v.message]
    return ev, line, errs


def main():
    run_utils.evidence(FIXTURE, 'the #1603 fixture')
    from kicad_parser import parse_kicad_pcb
    from placement import floorplan
    from placement.design_brief import merge_into_intent
    tmp = tempfile.mkdtemp(prefix='t1714_')
    try:
        chamfer, _name = variants(tmp)
        run_utils.evidence(chamfer, 'the chamfered outline')
        rows = {}
        for arm, src in (('rectangle', FIXTURE), ('chamfer', chamfer)):
            print(f'the {arm}:')
            pile = pile_of(src, os.path.join(tmp, f'{arm}.pile.kicad_pcb'))
            doc = with_heights(floorplan.emit_intent, quiet(parse_kicad_pcb, pile), pile)
            row = row_of(doc) or {}
            rows[arm] = row
            check("the pile's floor row carries the side its band was read along, no edge",
                  row.get('overhang_face') == '+x' and 'edge' not in row
                  and (row.get('overhang_mm') or {}).get('min', 0) > 11.5, str(row))
            check('...and the document says which reader acts on it',
                  # (#1820: the fixture's pair junction stamps its own reader)
                  doc.get('min_reader') == max(
                      floorplan.OVERHANG_FACE_READER,
                      floorplan.PAIR_SERIES_READER if doc.get('pair_series')
                      else 0) <= floorplan.READER_VERSION,
                  str(doc.get('min_reader')))
            ev, line, errs = graded(src, row)
            check(f'graded at the fixture\'s pose: ~{READING} mm past the east edge, pass',
                  abs(ev.get('overhang_mm', 0) - READING) < 0.02
                  and ev.get('overhang_disposition') == 'pass' and not errs,
                  f"{ev.get('overhang_mm')} {ev.get('overhang_disposition')} {errs}")
            check('...and the line says what it read past, and that the row owes its edge',
                  'past the east edge its overhang_face points at' in line
                  and 'its row declares no edge, and place_simple seats only a row with one'
                  in line and ev.get('band_edge') == 'east', line)
            bare = {k: v for k, v in row.items() if k != 'overhang_face'}
            ev, line, errs = graded(src, bare)
            if arm == 'chamfer':
                check('control: without overhang_face the edge-less row keeps the sum '
                      '(~3x the distance) and fails',
                      ev.get('overhang_mm', 0) > 3 * READING - 0.5 and errs,
                      f"{ev.get('overhang_mm')} {errs}")
            check('...and says nothing of an owed edge', 'band_edge' not in ev
                  and 'declares no edge' not in line, line)
            ev, line, errs = graded(src, dict(row, edge='east'))
            check('with its edge written in: read past it, passing, nothing owed',
                  abs(ev.get('overhang_mm', 0) - READING) < 0.02 and not errs
                  and 'band_edge' not in ev, f"{ev.get('overhang_mm')} {line}")

        print('the face, read at the part\'s turn:')
        for rot, edge in ((0, 'east'), (90, 'north'), (180, 'west'), (270, 'south')):
            ctx = SimpleNamespace(state=SimpleNamespace(parts={'J3': SimpleNamespace(rot=rot)}),
                                  pcb=SimpleNamespace(footprints={}))
            got = floorplan.band_edge(ctx, {'ref': 'J3', 'overhang_face': '+x'})
            check(f'+x at turn {rot}: {edge}', got == edge, str(got))
        check('a declared edge outranks the face',
              floorplan.band_edge(ctx, {'ref': 'J3', 'edge': 'east', 'overhang_face': '+x'})
              == 'east')

        print('the loader, and a brief:')
        try:
            floorplan.intent_from_dict({'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND,
                                        'units': 'mm', 'edge_connectors': [
                                            dict(rows['rectangle'], overhang_face='x')]}, 'test')
            refused = ''
        except floorplan.IntentError as exc:
            refused = str(exc)
        check('an overhang_face that is not a face is refused, by name',
              'overhang_face' in refused and "'x'" in refused, refused or 'accepted')
        emitted = {'edge_connectors': [copy.deepcopy(rows['rectangle'])]}
        report = dict({k: [] for k in ('declared', 'unknown', 'absent', 'not_graded',
                                       'unmatched', 'contradictions', 'fixed')},
                      path='t.design-brief.json', unmatched_checked=True, counts={},
                      product=None)
        kept = row_of(merge_into_intent(copy.deepcopy(emitted), {'edge_connectors': [
            {'ref': 'J3', 'edge': 'east', 'source': 'brief'}]}, report))
        check('a brief giving only the edge keeps the face beside the floor',
              kept.get('overhang_face') == '+x' and kept.get('edge') == 'east', str(kept))
        gone = row_of(merge_into_intent(copy.deepcopy(emitted), {'edge_connectors': [
            {'ref': 'J3', 'overhang_mm': {'min': 0.0, 'max': 3.0}, 'source': 'brief'}]},
            report))
        check('a brief replacing the band drops the face it was read along',
              'overhang_face' not in gone and gone['overhang_mm'] == {'min': 0.0, 'max': 3.0},
              str(gone))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
