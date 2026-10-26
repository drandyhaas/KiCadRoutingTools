#!/usr/bin/env python3
"""The project's schematic, read for placement grouping (placement.schematic
and board_context's `## Schematic` section).

A synthetic project is written to a temp dir: a root sheet with two blocks
drawn apart and one part drawn on its own, a sub-sheet in a subdirectory, a
note beside one block, and layout notes -- one a pin claim, one STALE (its
part on another net), one a DNP note that is not a layout note. The board's references deliberately
DIFFER from the schematic's, because a part is matched by the uuids of its
`(path ...)`, never by its reference.

Run: python3 -X utf8 tests/test_schematic_context.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path[:0] = [os.path.join(ROOT, 'py_router'),
                os.path.join(ROOT, 'py_placer'), HERE]

import numpy as np  # noqa: E402

from synth import board_text, footprint_text  # noqa: E402

FAILURES = []
TOOL = os.path.join(ROOT, 'py_tools', 'board_context.py')


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f' -- {detail}' if detail and not cond else ''))
    if not cond:
        FAILURES.append(name)


LIBS = '''  (lib_symbols
    (symbol "Device:R" (in_bom yes) (on_board yes)
      (symbol "R_0_1" (rectangle (start -1.016 -2.54) (end 1.016 2.54)))
      (symbol "R_1_1"
        (pin passive line (at 0 3.81 270) (length 1.27) (name "~") (number "1"))
        (pin passive line (at 0 -3.81 90) (length 1.27) (name "~") (number "2"))))
    (symbol "Device:IC" (in_bom yes) (on_board yes)
      (symbol "IC_0_1" (rectangle (start -5.08 -5.08) (end 5.08 5.08)))
      (symbol "IC_1_1"
        (pin passive line (at -7.62 2.54 0) (length 2.54) (number "1"))
        (pin passive line (at -7.62 0 0) (length 2.54) (number "2"))
        (pin passive line (at 7.62 2.54 180) (length 2.54) (number "3"))
        (pin passive line (at 7.62 0 180) (length 2.54) (number "4"))))
    (symbol "power:GND" (power) (in_bom yes) (on_board yes)
      (symbol "GND_1_1" (pin power_in line (at 0 0 270) (length 0) (number "1")))))
'''


def sym(lib, ref, x, y, uuid, value='v', rot=0):
    return (f'  (symbol (lib_id "{lib}") (at {x} {y} {rot}) (unit 1)\n'
            f'    (uuid "{uuid}")\n'
            f'    (property "Reference" "{ref}" (at {x} {y} 0))\n'
            f'    (property "Value" "{value}" (at {x} {y} 0)))\n')


def text(t, x, y):
    return (f'  (text "{t}" (at {x} {y} 0)\n'
            f'    (effects (font (size 1.27 1.27)) (justify left bottom)))\n')


def sch(body, uuid='root-uuid'):
    return (f'(kicad_sch (version 20231120) (generator "eeschema")\n'
            f'  (uuid "{uuid}")\n{LIBS}{body})\n')


def write_project(d, *, with_sub=True):
    """The project in `d`; returns the board path."""
    root_body = ''.join([
        # block A: the IC and two caps drawn round it
        sym('Device:IC', 'U1', 50, 50, 'sym-u1', value='RP2040'),
        sym('Device:R', 'C1', 62, 50, 'sym-c1'),
        sym('Device:R', 'C2', 38, 50, 'sym-c2'),
        # block B: an LED string drawn 80 mm to the right
        sym('Device:R', 'R1', 150, 50, 'sym-r1'),
        sym('Device:R', 'D1', 156, 50, 'sym-d1'),
        # drawn on its own, far below everything
        sym('Device:R', 'J1', 50, 180, 'sym-j1'),
        # a drilled mounting hole: on the schematic, no copper
        sym('Device:R', 'H1', 250, 180, 'sym-h1'),
        # a power symbol is never a part
        sym('power:GND', '#PWR01', 62, 60, 'sym-gnd'),
        text('Status LED', 148, 40),
        text('C2 close to pin 3 of RP2040', 30, 40),
        # stale: R1 is on the LED net, not on U1 pin 1's
        text('R1 close to pin 1 of RP2040', 30, 70),
        text('Do not place R1', 150, 70),
        ('  (sheet (at 200 20) (size 20 10) (uuid "sheet-pwr")\n'
         '    (property "Sheetname" "Power" (at 200 19 0))\n'
         '    (property "Sheetfile" "sub/power.kicad_sch" (at 200 31 0)))\n'
         if with_sub else ''),
        ('  (sheet (at 200 60) (size 20 10) (uuid "sheet-gone")\n'
         '    (property "Sheetname" "Gone" (at 200 59 0))\n'
         '    (property "Sheetfile" "gone.kicad_sch" (at 200 71 0)))\n'),
    ])
    with open(os.path.join(d, 'proj.kicad_sch'), 'w') as fh:
        fh.write(sch(root_body))
    if with_sub:
        os.makedirs(os.path.join(d, 'sub'), exist_ok=True)
        sub = ''.join([sym('Device:IC', 'U2', 50, 50, 'sym-u2'),
                       sym('Device:R', 'C3', 62, 50, 'sym-c3'),
                       text('3V3 LDO', 45, 40)])
        with open(os.path.join(d, 'sub', 'power.kicad_sch'), 'w') as fh:
            fh.write(sch(sub, 'sub-uuid'))
    # The board's refs are NOT the schematic's: matching is by uuid path.
    fps = [('BU1', 'sym-u1', 4, 'RP2040', 1), ('BC1', 'sym-c1', 2, None, 1),
           ('BC2', 'sym-c2', 2, None, 1), ('BR1', 'sym-r1', 2, None, 2),
           ('BD1', 'sym-d1', 2, None, 2), ('BJ1', 'sym-j1', 2, None, 1),
           ('BU2', 'sheet-pwr/sym-u2', 4, None, 1),
           ('BC3', 'sheet-pwr/sym-c3', 2, None, 1),
           ('BX9', 'no-such-symbol', 2, None, 1)]
    npth = ('\t\t(path "/sym-h1")\n\t\t(pad "" np_thru_hole circle (at 0 0) '
            '(size 3 3) (drill 3) (layers "*.Cu" "*.Mask"))\n')
    blocks = [footprint_text('BH1', 150, 150, pads=0, extra=npth)]
    for i, (ref, path, pads, value, net) in enumerate(fps):
        extra = f'\t\t(path "/{path}")\n'
        if value:
            extra += f'\t\t(property "Value" "{value}" (at 0 0))\n'
        blocks.append(footprint_text(ref, 10 + 10 * i, 20, pads=pads,
                                     pad_net=net, extra=extra))
    board = os.path.join(d, 'proj.kicad_pcb')
    with open(board, 'w') as fh:
        fh.write(board_text(''.join(blocks), nets=(0, 1, 2)))
    return board


def _parse(board):
    import contextlib
    import io
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(board)


def test_average_link():
    from placement.schematic import average_link
    D = np.array([[0, 1, 50, 52], [1, 0, 51, 50], [50, 51, 0, 2],
                  [52, 50, 2, 0]], float)
    check('average link: two pairs at a 10 mm cut',
          average_link(D, 10) == [[0, 1], [2, 3]], average_link(D, 10))
    check('average link: one cluster past the average gap',
          average_link(D, 60) == [[0, 1, 2, 3]], average_link(D, 60))
    check('average link: nothing merges under the smallest gap',
          average_link(D, 0.5) == [[0], [1], [2], [3]])
    check('average link: empty', average_link(np.zeros((0, 0)), 5) == [])


def test_context(d):
    from placement import schematic as S
    board = write_project(d)
    root, looked = S.discover(board)
    check('discover finds <board>.kicad_sch',
          root == os.path.join(d, 'proj.kicad_sch'), (root, looked))
    sec = S.section(_parse(board), board)
    check('found, no error', sec.get('found') and not sec.get('error'), sec)
    check('the missing sheet file is named, not dropped',
          sec.get('missing_files') == ['gone.kicad_sch'],
          sec.get('missing_files'))
    check('matched by uuid path: 8 of the 9 board parts with copper (the '
          'drilled hole is on the schematic but counted in neither)',
          sec.get('matched') == 8 and sec.get('board_parts') == 9
          and 'BH1' in sec['parts'],
          (sec.get('matched'), sec.get('board_parts')))
    check('a footprint whose path names no symbol is listed',
          sec.get('not_on_schematic') == ['BX9'], sec.get('not_on_schematic'))
    parts = sec['parts']
    check('parts keyed by the BOARD reference', 'BU1' in parts and
          'U1' not in parts, sorted(parts))
    check('the IC and its caps are one block',
          parts['BU1']['block'] and parts['BU1']['block']
          == parts['BC1']['block'] == parts['BC2']['block'], parts)
    check('the LED string is another block',
          parts['BR1']['block'] == parts['BD1']['block']
          and parts['BR1']['block'] != parts['BU1']['block'], parts)
    check('a part drawn on its own has no block',
          parts['BJ1'] == {'sheet': '', 'block': None}, parts.get('BJ1'))
    check('the sub-sheet in a subdirectory resolves, by its sheet name',
          parts['BU2']['sheet'] == 'Power'
          and parts['BU2']['block'] == parts['BC3']['block'], parts.get('BU2'))
    by_ref = {r: b for s in sec['sheets'] for b in s['blocks']
              for r in b['refs']}
    check('a note beside a block is that block\'s',
          'Status LED' in by_ref['BR1']['notes'], by_ref['BR1'])
    check('a note on a sub-sheet reaches its block',
          '3V3 LDO' in by_ref['BU2']['notes'], by_ref['BU2'])
    check('a power symbol is never a part',
          not any('#PWR' in r for r in parts), sorted(parts))
    notes = {n['text']: n for n in sec['layout_notes']}
    check('two layout notes: "Do not place" is not one',
          sorted(notes) == ['C2 close to pin 3 of RP2040',
                            'R1 close to pin 1 of RP2040'], sorted(notes))
    rows = notes.get('C2 close to pin 3 of RP2040', {}).get('rows', [])
    check('a pin row: the pin owner is the subject, named by its VALUE, and '
          'the cap by its schematic reference, mapped to the board\'s',
          [{k: v for k, v in r.items() if k != 'why'} for r in rows]
          == [{'ref': 'BU1', 'near': 'BC2', 'pads': {'BU1': ['3']},
               'max_mm': None}], rows)
    stale = notes.get('R1 close to pin 1 of RP2040', {})
    check('a note whose part is not on the pin\'s net is STALE, never a row, '
          'and names the two-pad parts that are',
          not stale.get('rows') and len(stale.get('stale') or []) == 1
          and 'BR1 is not on BU1 pin 1' in stale['stale'][0]['stale']
          and 'BC1' in stale['stale'][0]['stale'], stale)


def test_layout_note_clauses():
    from placement import schematic as S
    refs = {r: r for r in ('C10', 'C11', 'U4', 'R4', 'R5', 'U3')}
    values = {'RP2040': ['U3']}
    c = S._clauses('C10 close to pin 11 C11 close to pin 10', refs, values)
    check('two clauses in one sentence, each with its own subject and pins',
          [(x['subjects'], x['pins']) for x in c]
          == [(['C10'], ['11']), (['C11'], ['10'])], c)
    c = S._clauses('Place R4/R5 close to RP2040', refs, values)
    check('"R4/R5 close to RP2040": two subjects, the value names the part',
          c == [{'subjects': ['R4', 'R5'], 'target': 'U3', 'pins': []}], c)
    check('"do not place" is no layout note',
          S._keywords('Do not place R4') == [])
    check('a wrapped note is one sentence',
          S._sentences('C26 close to\nMCU pin 44') == ['C26 close to MCU pin 44'])


def test_tree_files(d):
    from placement.schematic import tree_files
    inside, outside = tree_files(os.path.join(d, 'proj.kicad_sch'))
    rels = sorted(r.replace('\\', '/') for _p, r in inside)
    check('tree_files: the root and the sub-sheet, by relative path',
          rels == ['proj.kicad_sch', 'sub/power.kicad_sch'], rels)
    check('tree_files: nothing outside', outside == [], outside)


def test_discover_cases(d):
    from placement import schematic as S
    os.makedirs(os.path.join(d, 'pro'))
    b = os.path.join(d, 'pro', 'renamed.kicad_pcb')
    open(b, 'w').write('x')
    open(os.path.join(d, 'pro', 'proj.kicad_pro'), 'w').write('{}')
    open(os.path.join(d, 'pro', 'proj.kicad_sch'), 'w').write('(kicad_sch)')
    root, looked = S.discover(b)
    check('discover falls back to the one .kicad_pro\'s schematic',
          root == os.path.join(d, 'pro', 'proj.kicad_sch'), (root, looked))
    os.makedirs(os.path.join(d, 'legacy'))
    lb = os.path.join(d, 'legacy', 'old.kicad_pcb')
    open(lb, 'w').write('x')
    open(os.path.join(d, 'legacy', 'old.sch'), 'w').write('EESchema')
    sec = S.section(None, lb)
    check('a KiCad 5 .sch is reported, not read',
          not sec['found'] and 'KiCad 5' in sec.get('error', ''), sec)
    sec = S.section(None, lb, schematic=os.path.join(d, 'nope.kicad_sch'))
    check('a named schematic that does not exist is an error, not "none"',
          not sec['found'] and 'does not exist' in sec.get('error', ''), sec)


def test_board_context(d):
    import run_utils
    board = os.path.join(d, 'proj.kicad_pcb')
    r = subprocess.run([sys.executable, '-X', 'utf8', TOOL, board, '--json'],
                       capture_output=True, text=True, env=run_utils.tool_env(),
                       timeout=600)
    check('board_context --json exits 0', r.returncode == 0, r.stderr[-500:])
    try:
        doc = json.loads(r.stdout)
    except ValueError as exc:
        check('board_context --json parses', False, str(exc))
        return
    sch_sec = doc.get('schematic') or {}
    check('the document carries the schematic section',
          sch_sec.get('found') and sch_sec.get('matched') == 8, sch_sec)
    check('the per-part map is on the parts, not repeated in the section',
          'parts' not in sch_sec)
    by = {p['ref']: p for p in doc['parts']}
    check('parts[].schematic names the sheet and block',
          by['BU2']['schematic'] == {'sheet': 'Power',
                                     'block': by['BC3']['schematic']['block']}
          and by['BX9']['schematic'] is None,
          (by['BU2'].get('schematic'), by['BX9'].get('schematic')))
    r = subprocess.run([sys.executable, '-X', 'utf8', TOOL, board, '--md'],
                       capture_output=True, text=True, env=run_utils.tool_env(),
                       timeout=600)
    md = r.stdout
    check('--md prints the section before the pin-order one',
          0 <= md.find('## Schematic') < md.find('## Pin-order agreement'),
          md[:400])
    check('--md names the sheet, the note, the layout row and the stale one',
          "### Sheet `Power`" in md and '"Status LED"' in md
          and '"pads": {"BU1": ["3"]}' in md and 'STALE: BR1' in md,
          md[:3000])
    r = subprocess.run([sys.executable, '-X', 'utf8', TOOL, board, '--json',
                        '--no-schematic'], capture_output=True, text=True,
                       env=run_utils.tool_env(), timeout=600)
    doc = json.loads(r.stdout)
    check('--no-schematic reads none, and says so',
          doc['schematic'] == {'found': False,
                               'skipped': 'by --no-schematic'},
          doc.get('schematic'))


def main():
    d = tempfile.mkdtemp(prefix='krt_schematic_')
    try:
        print('average_link')
        test_average_link()
        print('context')
        test_context(d)
        print('layout note clauses')
        test_layout_note_clauses()
        print('tree_files')
        test_tree_files(d)
        print('discover')
        test_discover_cases(d)
        print('board_context')
        test_board_context(d)
    finally:
        shutil.rmtree(d, ignore_errors=True)
    if FAILURES:
        print(f'\n{len(FAILURES)} FAILED: ' + '; '.join(FAILURES))
        return 1
    print('\nALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
