"""#1823: a cover's hole site past the board outline is not clear.

`mount_report` recorded how far each `holes[]` site reached past the outline
and judged nothing about it, so a module whose mounting hole had no board
under it graded `mountable YES`, and check_assembly printed `site clear`
beside the distance. Now a site whose drill circle reaches past the outline
does not fit, appends a reason naming the hole and the distance, and the
cover is NOT mountable; the distance is read on the drill circle, not its
bounding box.

A synthetic board (60 x 60 mm): J1 a socket whose module (8 x 20 mm) has one
hole 18 mm along its axis.

  1. J1 turned 180 degrees, the site 9.35 mm past the outline: NO, the
     reason names hole 1 and the distance, `fits` False;
  2. control: J1 at 0 degrees, the site on the board and its ring empty:
     YES, the site fits with no standoff;
  3. control: R1 in the ring of an on-board site stays NO for R1;
  4. a round board where the drill circle is inside but its bounding box's
     corner is not: the site fits (the circle is measured, not the bbox);
  5. check_assembly prints NO and `past the outline`, not `site clear`.

    python3 tests/test_1823_cover_hole_off_board.py
"""
import contextlib
import io
import json
import math
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

from placement import floorplan as fp  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []
HOLE = {'length_mm': 18, 'diameter_mm': 2.7}


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def _fp(ref, x, y, rot, crt, pads, name='t:PART'):
    out = [f'  (footprint "{name}" (layer "F.Cu") (at {x} {y} {rot})',
           f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS") (hide yes))',
           f'    (fp_rect (start {crt[0]} {crt[1]}) (end {crt[2]} {crt[3]}) (stroke (width 0.05)'
           f' (type default)) (layer "F.CrtYd"))']
    for num, px, py, w, h in pads:
        out.append(f'    (pad "{num}" smd rect (at {px} {py} {rot}) (size {w} {h}) '
                   f'(layers "F.Cu") (net 0 ""))')
    out.append('  )')
    return '\n'.join(out) + '\n'


def board(td, name, j1=(10.0, 30.0, 0), r1=None, outline=None):
    sock = [(str(k + 1), 0.0, -4.5 + k, 0.6, 0.3) for k in range(10)]
    parts = _fp('J1', *j1, (-1.0, -5.0, 1.0, 5.0), sock, 't:SOCKET')
    if r1:
        parts += _fp('R1', r1[0], r1[1], 0, (-1.0, -0.6, 1.0, 0.6),
                     [('1', -0.5, 0, 0.6, 0.6), ('2', 0.5, 0, 0.6, 0.6)])
    edge = outline or ('  (gr_rect (start 0 0) (end 60 60) (stroke (width 0.1) '
                       '(type default)) (layer "Edge.Cuts"))\n')
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (net 0 "")\n' + edge + parts + ')\n')
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w') as fh:
        fh.write(text)
    return path


def intent(holes=(HOLE,)):
    row = {'connector': 'J1', 'side': 'F', 'axis_deg': 0, 'width_mm': 8, 'length_mm': 20,
           'underside_mm': 5.0, 'removable': True, 'source': 'synthetic: a module',
           'holes': list(holes)}
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'min_reader': fp.HOLES_READER, 'covers': [row],
            'heights': [{'ref': 'R1', 'height_mm': 0.5, 'source': 'synthetic'}]}


def report(path, doc):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    return fp.mount_report(fp.intent_from_dict(doc, ''), pcb, path)[0]


def off_board(td):
    print('1. J1 turned so its site lies past the outline')
    m = report(board(td, 'b1', j1=(10.0, 30.0, 180)), intent())
    s = m['holes'][0]
    # the site at x = 10 - 18 = -8, its drill reaching -8 + 1.35 = -6.65:
    # its nearest point 6.65 mm out, its farthest 9.35 mm
    check('the site is where the row puts it', s['at'] == [-8.0, 30.0], str(s['at']))
    check('past_outline_mm is the drill circle\'s farthest reach, 9.35',
          abs(s['past_outline_mm'] - 9.35) < 0.01, str(s['past_outline_mm']))
    check('the site does not fit', s['fits'] is False, json.dumps(s))
    check('NO', m['mountable'] is False, json.dumps(m['why']))
    check('the reason names hole 1 and the distance',
          any(w.startswith('hole 1 ') and 'past the outline' in w
              and f"{s['past_outline_mm']:g}mm" in w for w in m['why']),
          json.dumps(m['why']))


def on_board(td):
    print('2. control: the site on the board, its ring empty')
    m = report(board(td, 'b2'), intent())
    s = m['holes'][0]
    check('YES, the site fits, no standoff',
          m['mountable'] is True and s['fits'] and s['standoff'] is None
          and s['past_outline_mm'] == 0, json.dumps(m))


def ring(td):
    print('3. control: R1 in an on-board ring')
    m = report(board(td, 'b3', r1=(28.0, 30.0)), intent())
    s = m['holes'][0]
    check('NO for R1, nothing about the outline',
          m['mountable'] is False and [p['ref'] for p in s['parts']] == ['R1']
          and not any('past the outline' in w for w in m['why']), json.dumps(m['why']))


def round_board(td):
    print('4. a round board: the circle is measured, not its bbox')
    # board: a circle of radius 30 about (30, 30). The site sits on the 45
    # degree diagonal with its drill circle 0.2 mm inside the edge, so the
    # drill's bbox corner (1.35 * sqrt 2 = 1.91 mm from the centre) is past it.
    d = 30.0 - 1.35 - 0.2
    sx = sy = 30.0 + d / math.sqrt(2)
    edge = ('  (gr_circle (center 30 30) (end 60 30) (stroke (width 0.1) '
            '(type default)) (layer "Edge.Cuts"))\n')
    m = report(board(td, 'b4', j1=(sx - 18.0, sy, 0), outline=edge), intent())
    s = m['holes'][0]
    check('the drill circle inside a round outline fits',
          s['fits'] and s['past_outline_mm'] == 0 and m['mountable'] is True,
          json.dumps({k: s[k] for k in ('at', 'past_outline_mm', 'fits')}))
    m = report(board(td, 'b4b', j1=(sx - 18.0 + 0.5, sy + 0.5, 0), outline=edge), intent())
    s = m['holes'][0]
    check('...and one moved out across it does not',
          not s['fits'] and s['past_outline_mm'] > 0 and m['mountable'] is False,
          json.dumps({k: s[k] for k in ('at', 'past_outline_mm', 'fits')}))


def cli(td):
    print('5. check_assembly prints the site past the outline')
    path = run_utils.evidence(board(td, 'b5', j1=(10.0, 30.0, 180)), 'the board')
    ip = os.path.join(td, 'i5.json')
    with open(ip, 'w') as fh:
        json.dump(intent(), fh)
    run_utils.evidence(ip, 'the intent')
    r = run_utils.check(PY + [os.path.join(REPO, 'py_tools', 'check_assembly.py'), path,
                              '--intent', ip], accept=True)
    out = r.stdout
    row = next((ln for ln in out.splitlines() if 'hole 1 at (' in ln), '')
    check('J1 NO', 'mountable (#1504): J1 NO (removable)' in out, out[-800:])
    check('the site row says past the outline, not site clear',
          'past the outline' in row and 'site clear' not in row, repr(row))


def _state(path, doc):
    from kicad_parser import parse_kicad_pcb
    from placement.edge_seat import seat_state
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        return seat_state(pcb, path, fp.intent_from_dict(doc, ''), {}, 0.2, 0.2)


def seat(td):
    print('6. the seat predicate holds the site on the board')
    from placement.seeder import _fixed_pose_check, pose_ok
    path = board(td, 'b6')
    st = _state(path, intent())
    check('the state carries J1\'s site in its own frame',
          getattr(st, 'cover_hole_sites', {}).get('J1') == [(18.0, 0.0, 2.7)],
          str(getattr(st, 'cover_hole_sites', None)))
    check('pose_ok refuses J1 turned so the site is off the board',
          not pose_ok(st, 'J1', 10.0, 30.0, 180.0, set()))
    check('...and admits J1 where the site is on it', pose_ok(st, 'J1', 10.0, 30.0, 0.0, set()))
    # 41 + 18 + 1.35 = 60.35: the drill 0.35 mm past the right edge, the
    # courtyard well inside it
    check('pose_ok refuses a drill that only just leaves the board',
          not pose_ok(st, 'J1', 41.0, 30.0, 0.0, set()))
    how, why, _c = _fixed_pose_check(st, 'J1', (10.0, 30.0, 180.0), {})
    check('a declared/seated pose there is refused, naming the hole',
          how is None and any("cover's hole 1 is 9.350mm past the outline" in w for w in why),
          f"{how} {why}")
    doc = intent()
    del doc['covers'][0]['holes']
    st = _state(path, doc)
    check('with no holes[] the same poses are admitted',
          not getattr(st, 'cover_hole_sites', None) and pose_ok(st, 'J1', 10.0, 30.0, 180.0, set())
          and pose_ok(st, 'J1', 41.0, 30.0, 0.0, set()))


def no_seat(td):
    print('7. place_simple: a cover whose hole fits nowhere names why J1 is illegal')
    # a 60 mm board, the hole 70 mm along: no pose of J1 puts it on the board
    path = run_utils.evidence(board(td, 'b7'), 'the board')
    doc = intent(holes=({'length_mm': 70, 'diameter_mm': 2.7},))
    doc['covers'][0]['length_mm'] = 72
    ip, gp = os.path.join(td, 'i7.json'), os.path.join(td, 'g7.json')
    with open(ip, 'w') as fh:
        json.dump(doc, fh)
    with open(gp, 'w') as fh:
        json.dump({'schema': 1, 'groups': [{'name': 'all', 'refs': ['J1']}]}, fh)
    run_utils.evidence(ip, 'the intent')
    run_utils.check(PY + [os.path.join(REPO, 'py_placer', 'place_simple.py'), path,
                          os.path.join(td, 's{seed}.kicad_pcb'), '--intent', ip,
                          '--groups', gp, '--seeds', '0'],
                    refuse="illegal: J1 (its cover's hole 1 is", code=4)
    check('refused with exit 4, naming the hole', True)


def main():
    td = tempfile.mkdtemp(prefix='krt1823_')
    try:
        for t in (off_board, on_board, ring, round_board, cli, seat, no_seat):
            try:
                t(td)
            except Exception as ex:                            # noqa: BLE001
                check(f'{t.__name__} ran', False, f'{type(ex).__name__}: {ex}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
