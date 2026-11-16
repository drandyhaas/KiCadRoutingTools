#!/usr/bin/env python3
"""#1770: a "PCB Edge" text on its edge line's extension reads that line, and a
line only the text's stored angle chose may confirm any face but veto only a
weak one.

#1715's reader compared the two axes on the text's gap off each line PLUS how
far past the line's end it sits, so a text ON the edge line's extension, 1.2 mm
past its end, scored 1.2 mm against a body line 2.25 mm across it -- a "tie"
the stored angle broke. Stored at 0 to the part, it read the body line, vetoed
the body's correct face and warned a correct, sourced `mating_face` "against
its PCB-edge marker", which nothing could clear. Now (no KiCad needed):

  1. #1770's repro, esp_prog's USB1 with its edge line lengthened and the text
     on its extension: the edge line and +x, strong, at every stored angle,
     and check_floorplan says nothing against the marker;
  2. a text at a corner, the axes a true tie: the angle breaks it, and the
     reading says so (`by_angle`);
  3. ...such a reading that agrees with the body confirms it (strong);
  4. ...one that disagrees vetoes a WEAK body face (an abstention), never a
     STRONG one;
  5. ...and against a declared face on a body that names none, the WARN says
     it rests on the angle, and a recorded `mating_face_source` silences it.

    python3 tests/test_1770_marker_extension_and_angle.py
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
ESP = os.path.join(REPO, 'kicad_files', 'esp_prog.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def quiet(f, *a, **k):
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        return f(*a, **k)


def read(path, ref):
    from kicad_parser import parse_kicad_pcb
    from placement.body import board_bodies
    from placement.mating import marker_face_local, mating_face_local
    pcb = quiet(parse_kicad_pcb, path)
    fp, geom = pcb.footprints[ref], board_bodies(pcb, path)[ref]
    return pcb, fp, geom, marker_face_local(fp, geom), mating_face_local(fp, geom)


def usb1_on_its_extension(tmp):
    """1. #1770's repro: the edge line x = 2.4 lengthened to y = 4.75, the
    text at (2.4, 5.95), 1.2 mm past its end and 2.25 mm off the body line
    y = 3.7, stored at each angle."""
    from placement import floorplan
    print("USB1's text on its edge line's extension:")
    with open(ESP, encoding='utf-8') as f:
        s = f.read()
    a = s.index('"OLIMEX_Connectors-FP:USB-MICRO_MISB-SWMM-5B_LF"')
    b = s.index('\n\t(footprint', a)
    blk, n1 = re.subn(r'\(start 2\.4 -3\.7\)(\s*)\(end 2\.4 3\.7\)',
                      r'(start 2.4 -3.7)\1(end 2.4 4.75)', s[a:b])
    check('fixture: the edge line is where #1770 says', n1 == 1, str(n1))
    intent = floorplan.intent_from_dict({
        'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
        'edge_connectors': [{'ref': 'USB1', 'edge': 'west', 'mating_face': '+x',
                             'mating_face_source': 'datasheet: the receptacle opens '
                                                   'past the PCB-edge line'}]},
        source_path='t')
    for ang in (90, 180, 0, 270):
        out, n2 = re.subn(r'(fp_text user "pcb edge"\s*)\(at 2\.1 0 90\)',
                          rf'\1(at 2.4 5.95 {ang})', blk)
        if n2 != 1:
            check(f'fixture: the marker text is where #1770 says ({ang})', False, str(n2))
            continue
        path = os.path.join(tmp, f'usb1_{ang}.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as f:
            f.write(s[:a] + out + s[b:])
        pcb, fp, _g, mk, lf = read(path, 'USB1')
        check(f'stored at {ang}: the edge line x = 2.4 and +x, not by the angle',
              mk.face == '+x' and mk.line[:2] == ('x', 2.4) and not mk.by_angle, f'{mk}')
        check(f'...the body face +x, strong', lf.face == '+x' and lf.strong, f'{lf}')
        r = quiet(floorplan.grade, intent, pcb, path)
        said = [v.message for v in r.violations
                if v.ref == 'USB1' and 'PCB-edge marker' in v.message]
        check('...and no WARN against the marker', not said, str(said))


def corner_board(tmp, name, fab, text_angle):
    """A one-part board: J1's pads in a row about its origin, its F.Fab body
    `fab` (x0, y0, x1, y1), a silk edge line x = 3 (y -2..2) and a silk line
    y = 2 (x -3..3), and a "PCB Edge" text at their corner (3.3, 2.3): 0.3 mm
    off each line and 0.3 mm past each one's end, a tie."""
    pads = ''.join(f'    (pad "{n + 1}" smd rect (at {-1.5 + n} 0) (size 0.6 1.2) '
                   f'(layers "F.Cu" "F.Mask"))\n' for n in range(4))
    x0, y0, x1, y1 = fab
    fp = ('  (footprint "t:Marked" (layer "F.Cu") (at 50 50)\n'
          '    (property "Reference" "J1" (at 0 -4) (layer "F.SilkS"))\n'
          f'    (fp_text user "PCB Edge" (at 3.3 2.3 {text_angle}) (layer "Dwgs.User"))\n'
          '    (fp_line (start 3 -2) (end 3 2) (stroke (width 0.1) (type solid)) (layer "F.SilkS"))\n'
          '    (fp_line (start -3 2) (end 3 2) (stroke (width 0.1) (type solid)) (layer "F.SilkS"))\n'
          f'    (fp_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.1) (type default)) '
          f'(layer "F.Fab"))\n' + pads + '  )\n')
    cuts = ''.join(f'  (gr_line (start {a} {b}) (end {c} {e}) (stroke (width 0.05) '
                   f'(type default)) (layer "Edge.Cuts"))\n'
                   for a, b, c, e in ((0, 0, 100, 0), (100, 0, 100, 100), (100, 100, 0, 100),
                                      (0, 100, 0, 0)))
    path = os.path.join(tmp, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('(kicad_pcb (version 20241229) (generator "t1770")\n  (net 0 "")\n'
                + cuts + fp + ')\n')
    return run_utils.evidence(path, name)


def angle_ties(tmp):
    from placement import floorplan
    print('a text at a corner, the axes a tie:')
    # stored at 90 (along the x = 3 line): +x; at 0 (along y = 2): +y
    _p, _fp, _g, mk, _lf = read(corner_board(tmp, 'tie90', (-2, -1, 2.5, 1), 90), 'J1')
    check('the angle breaks it, and says so', mk.face == '+x' and mk.by_angle
          and 'angle' in mk.reason, f'{mk}')
    # 3. a weak +x body (fab 1.0 mm off the pads) the tie confirms
    _p, _fp, _g, mk, lf = read(corner_board(tmp, 'confirm', (-1, -1, 3, 1), 90), 'J1')
    check('...a tie reading that agrees confirms the body (strong)',
          mk.face == '+x' and mk.by_angle and lf.face == '+x' and lf.strong
          and 'marker' in lf.basis, f'{lf}')
    # 4. the same weak body, the text stored at 0: the tie reads +y and vetoes
    _p, _fp, _g, mk, lf = read(corner_board(tmp, 'weakveto', (-1, -1, 3, 1), 0), 'J1')
    check('...one that disagrees vetoes a weak body face',
          mk.face == '+y' and mk.by_angle and lf.face is None, f'{mk} / {lf}')
    # ...a strong +x body (fab 2.0 mm off the pads), the same tie: no veto
    _p, _fp, _g, mk, lf = read(corner_board(tmp, 'strongkept', (-1, -1, 5, 1), 0), 'J1')
    check('...and never a strong one',
          mk.face == '+y' and mk.by_angle and lf.face == '+x' and lf.strong, f'{mk} / {lf}')
    # 5. a body that names no face, a declared +x the tie reads as +y
    path = corner_board(tmp, 'declared', (-2, -1, 2, 1), 0)
    pcb, _fp, _g, mk, lf = read(path, 'J1')
    check('fixture: the body abstains, the tie reads +y', lf.face is None and mk.face == '+y'
          and mk.by_angle, f'{mk} / {lf}')
    for source in (None, 'datasheet p.3, pads matched'):
        row = {'ref': 'J1', 'edge': 'east', 'mating_face': '+x', 'class': 'edge_receptacle'}
        if source:
            row['mating_face_source'] = source
        intent = floorplan.intent_from_dict({
            'schema': floorplan.SCHEMA_VERSION, 'kind': floorplan.KIND, 'units': 'mm',
            'edge_connectors': [row]}, source_path='t')
        r = quiet(floorplan.grade, intent, pcb, path)
        said = [v.message for v in r.violations
                if v.ref == 'J1' and 'PCB-edge marker' in v.message]
        if source:
            check('...a recorded source silences the WARN against an angle-read marker',
                  not said, str(said))
        else:
            check('...without one the WARN says the marker rests on the angle',
                  len(said) == 1 and "as its text's stored angle reads it" in said[0], str(said))


def main():
    run_utils.evidence(ESP, 'esp_prog')
    tmp = tempfile.mkdtemp(prefix='t1770_')
    try:
        usb1_on_its_extension(tmp)
        angle_ties(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
