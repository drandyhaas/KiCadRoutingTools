#!/usr/bin/env python3
"""#1769: an edge part can be declared to open INTO the board.

An M.2 or SO-DIMM socket at an edge has its back on the edge and its opening
facing the board, its card lying over it. `mating_face` names the side the
opening is on, and the seat and the grade pointed that side OUT of the edge,
so the only declaration that seated such a socket back-to-edge was its back as
`mating_face`, and the grade then said the opening faced the edge. Now
`opens: "inward"` on the row keeps the face as the opening (read or declared)
and puts the opposite side at the edge:

  1. the vocabulary: `opens` takes `outward` / `inward`, a reader-25 key, and
     the brief compiles it, stamping that reader;
  2. KiCad 10's stock SODIMM-260 socket (its body reads +x strong, the module
     side): the seat turns its back (-x) to the declared edge, the grade
     passes that pose and words it "back to <edge> (-x), opens into the
     board (+x)", and fails the turn that points the opening out. Skipped,
     said, when this machine has no KiCad 10 library;
  3. a drawn strong-faced socket on a synthetic board, the same, always;
  4. #1769's own repro, esp_prog's CON2 on the north edge (body abstains):
     its declared opening -y with `opens: inward` passes as "back to north
     (+y), opens into the board (-y)", and +y with it fails.

    python3 tests/test_1769_opens_inward.py
"""
import contextlib
import io
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

RUN_ALL_TIMEOUT = 600
ESP = os.path.join(REPO, 'tests', 'fixtures', 'run25', 'esp_prog_placed.kicad_pcb')
SODIMM = ('C:/Program Files/KiCad/10.0/share/kicad/footprints/Connector_PCBEdge.pretty/'
          'SODIMM-260_DDR4_H4.0-5.2_OrientationStd_Socket.kicad_mod')
CUTS = ''.join(f'  (gr_line (start {a} {b}) (end {c} {e}) (stroke (width 0.05) '
               f'(type default)) (layer "Edge.Cuts"))\n'
               for a, b, c, e in ((20, 20, 180, 20), (180, 20, 180, 180),
                                  (180, 180, 20, 180), (20, 180, 20, 20)))
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def quiet(f, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return f(*a, **k)


def one_part(tmp, name, module):
    """A 160 x 160 board holding `module` (a footprint block) at (100, 100)."""
    path = os.path.join(tmp, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20241229) (generator "t1769")\n  (net 0 "")\n'
                + CUTS + module + '\n)\n')
    return run_utils.evidence(path, name)


def socket_module():
    """A socket drawn the way a card socket is: a contact row at x 0, its
    body (fab and courtyard) running 6 mm in +x, where its card lies."""
    pads = ''.join(f'    (pad "{n + 1}" smd rect (at 0 {-4.5 + n}) (size 1.2 0.5) '
                   f'(layers "F.Cu" "F.Mask"))\n' for n in range(10))
    return ('  (footprint "t:CardSocket" (layer "F.Cu") (at 100 100)\n'
            '    (property "Reference" "J1" (at 0 -7) (layer "F.SilkS"))\n'
            '    (fp_rect (start -1 -5.5) (end 12 5.5) (stroke (width 0.1) (type default)) '
            '(layer "F.Fab"))\n'
            '    (fp_rect (start -1.25 -5.75) (end 12.25 5.75) (stroke (width 0.05) '
            '(type default)) (layer "F.CrtYd"))\n' + pads + '  )')


def sodimm_module():
    with open(SODIMM, encoding='utf-8') as f:
        mod = f.read().strip()
    return re.sub(r'\(layer "F\.Cu"\)', '(layer "F.Cu") (at 100 100)', mod, count=1)


def seat_and_grade(path, edge, opens, label):
    """Seat the board's one part on `edge` with `opens`, then grade it there
    and at the opposite turn."""
    from kicad_parser import parse_kicad_pcb
    from placement import edge_seat, floorplan, heights
    from placement.body import board_bodies
    from placement.mating import facing_edge, mating_face_local
    pcb = quiet(parse_kicad_pcb, path)
    ref = next(iter(pcb.footprints))
    lf = mating_face_local(pcb.footprints[ref], board_bodies(pcb, path)[ref])
    check(f'{label}: the body reads a strong face', lf.face is not None and lf.strong,
          f'{lf}')
    row = {'ref': ref, 'edge': edge, 'class': 'edge_receptacle',
           'overhang_mm': {'min': 0.0, 'max': 1.0}}
    if opens:
        row['opens'] = opens
    doc = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
           'edge_connectors': [row]}
    if opens:
        doc['min_reader'] = floorplan.OPENS_READER
    intent = floorplan.intent_from_dict(doc, source_path='t')
    real = heights.for_board
    heights.for_board = lambda *_a, **_k: heights.Heights(note='no heights')
    try:
        st = quiet(edge_seat.seat_state, pcb, path, intent, {}, *floorplan.resolved_floors(path))
        got, notes, why = quiet(edge_seat.seats, st, pcb, path, intent, {ref})
    finally:
        heights.for_board = real
    if ref not in got:
        check(f'{label}: seated', False, str(why))
        return None
    seat = got[ref]
    rots = sorted(seat.lines)
    back = '-x' if lf.face == '+x' else None
    side = back if opens == 'inward' else lf.face
    check(f'{label}: every turn it seats at points {side} at the {edge} edge',
          rots and all(facing_edge(side, r) == edge for r in rots), f'{rots}')
    note = ' '.join(n for n in notes if ref in n)
    if opens == 'inward':
        check(f'{label}: the seat note says its back is at the edge',
              f'its back to it ({side}), opening into the board' in note, note)
    rot = rots[0] if rots else 0.0
    ev = floorplan.edge_facing(row, pcb.footprints[ref], board_bodies(pcb, path)[ref],
                               rotation=rot)
    flipped = floorplan.edge_facing(row, pcb.footprints[ref], board_bodies(pcb, path)[ref],
                                    rotation=(rot + 180.0) % 360.0)
    return ev, flipped, lf


def vocabulary():
    from placement import design_brief, floorplan
    from placement.mating import edge_side
    print('the vocabulary:')
    check('edge_side flips only an inward part', edge_side('+x', 'inward') == '-x'
          and edge_side('-y', 'inward') == '+y' and edge_side('+x', 'outward') == '+x'
          and edge_side('+x', None) == '+x' and edge_side(None, 'inward') is None)
    check('reader 25 reads opens', floorplan.READER_VERSION >= floorplan.OPENS_READER == 25)
    base = {'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm'}
    for v in ('inward', 'outward'):
        it = floorplan.intent_from_dict(dict(base, edge_connectors=[
            {'ref': 'J1', 'edge': 'north', 'opens': v}]), source_path='t')
        check(f'opens {v!r} loads', it.edge_connectors[0].get('opens') == v)
    try:
        floorplan.intent_from_dict(dict(base, edge_connectors=[
            {'ref': 'J1', 'edge': 'north', 'opens': 'sideways'}]), source_path='t')
        check('opens "sideways" is refused', False, 'loaded')
    except floorplan.IntentError as exc:
        check('opens "sideways" is refused, naming the key', 'opens' in str(exc), str(exc))
    brief = design_brief.brief_from_dict({
        'schema': design_brief.SCHEMA_VERSION, 'kind': design_brief.KIND,
        'interfaces': [{'ref': 'J1', 'edge': 'north', 'mating_face': '-y',
                        'opens': 'inward'}]}, 'b')
    frag, _report = design_brief.compile_brief(brief, board_refs=['J1'])
    rows = frag.get('edge_connectors') or []
    check('the brief compiles opens onto the edge row, at reader 25',
          rows and rows[0].get('opens') == 'inward'
          and frag.get('min_reader', 0) >= floorplan.OPENS_READER, f'{frag}')


def main():
    vocabulary()
    from placement import floorplan
    tmp = tempfile.mkdtemp(prefix='t1769_')
    try:
        boards = [('a drawn card socket', one_part(tmp, 'socket', socket_module()))]
        if os.path.isfile(SODIMM):
            boards.append(('the stock SODIMM-260 socket', one_part(tmp, 'sodimm', sodimm_module())))
        else:
            print(f'  [skip] no KiCad 10 library here ({SODIMM}): the stock SODIMM arm is not run')
        for label, path in boards:
            print(f'{label}:')
            got = seat_and_grade(path, 'west', None, label + ', no opens')
            if got:
                ev, _f, lf = got
                check('...without opens its opening goes to the edge (as before)',
                      ev['facing'] == 'west' and ev['local_face'] == lf.face
                      and not ev['fails'], f'{ev}')
            got = seat_and_grade(path, 'west', 'inward', label + ', opens inward')
            if got:
                ev, flipped, lf = got
                tail = floorplan._facing_tail(dict(ev, disposition='pass'))
                check('...the grade passes it, its back at the edge',
                      ev['facing'] == 'west' and ev['edge_face'] == '-x'
                      and ev['local_face'] == '+x' and ev['opens'] == 'inward'
                      and not ev['fails'] and ev['policy'] == 'graded', f'{ev}')
                check('...and the clause says so',
                      tail.startswith('; back to west (-x), opens into the board (+x) [')
                      and tail.endswith('pass'), tail)
                check('...the opening turned out of the edge fails', flipped['fails']
                      and flipped['facing'] == 'east', f'{flipped}')

        print("#1769's repro: esp_prog's CON2 on the north edge:")
        run_utils.evidence(ESP, 'esp_prog_placed')
        work = os.path.join(tmp, 'esp')
        os.makedirs(work)
        board = os.path.join(work, 'b.kicad_pcb')
        run_utils.check([sys.executable, '-B', '-X', 'utf8',
                         os.path.join(REPO, 'py_router', 'copy_board.py'), ESP, board],
                        accept=True)
        from kicad_parser import parse_kicad_pcb
        pcb = quiet(parse_kicad_pcb, board)
        doc = quiet(floorplan.emit_intent, pcb, board, declare_classes=True)
        con2 = [c for c in doc['edge_connectors'] if c['ref'] == 'CON2']
        check('the emit declares CON2 on the north edge', con2 and con2[0].get('edge') == 'north',
              str(con2))
        for face, verdict, words in (('-y', 'pass', '; back to north (+y), opens into the board (-y) ['),
                                     ('+y', 'fail', '; back to south (-y), opens into the board (+y) [')):
            for c in doc['edge_connectors']:
                if c['ref'] == 'CON2':
                    c.update(mating_face=face, mating_face_source='datasheet p.2, pads matched',
                             opens='inward')
            doc['min_reader'] = max(int(doc.get('min_reader') or 0), floorplan.OPENS_READER)
            r = quiet(floorplan.grade, floorplan.intent_from_dict(doc, source_path='e'), pcb, board)
            ev = [e for e in r.edge_connector_evidence if e['ref'] == 'CON2']
            f = ev[0]['facing'] if ev else {}
            tail = floorplan._facing_tail(f)
            check(f'declared {face} with opens inward: {verdict}, worded as its back',
                  f.get('disposition') == verdict and tail.startswith(words)
                  and tail.endswith(verdict.upper() if verdict == 'fail' else verdict), tail)
            errs = [v.message for v in r.errors if v.rule == 'edge_connector' and v.ref == 'CON2']
            check(f'...and {"an ERROR names its back" if verdict == "fail" else "no ERROR"}',
                  (len(errs) == 1 and 'opens into the board' in errs[0]
                   and 'its back (-y)' in errs[0]) if verdict == 'fail' else not errs, str(errs))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
