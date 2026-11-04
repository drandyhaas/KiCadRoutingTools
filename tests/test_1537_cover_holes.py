"""#1537: a cover's own mounting holes (`covers[].holes[]`) and what stands at
each hole site.

`mountable` read standoffs only as parts already on the board, so a module
whose holes landed on parts read YES with 0 standoffs and never looked; a
`holes` key was refused, and a standoff with no ref was refused. Now a row
declares each hole from the body's drawing (`length_mm` along the axis,
`off_mm` off the centre line, `diameter_mm`), and `mount_report` lists per
site the standoff there, the parts, pads and other-net copper within the
screw-head ring (`mounting_holes.reach`), NO when a site is blocked.

A synthetic board: J1 a socket whose module (8 x 20 mm) has two holes 18 mm
along its axis, 2.5 mm either side of its centre line; R1 a resistor on the
first site.

  1. the schema: holes accepted (reader 22); an unknown key, a missing or
     zero diameter refused; a standoff with no ref points at `holes[]`;
  2. no holes: the row carries no `holes` and the verdict is unchanged;
  3. R1 on site 1, site 2 free: NO, naming R1 (part and pads) at hole 1;
     site 2 clear, no standoff, not in `why`;
  4. a mounting hole fitted at site 2: it is the site's standoff, its own
     pads are not obstructions;
  5. another net's track through site 2: named as copper there, NO;
  6. the brief compiles holes 1:1 and stamps min_reader 22;
  7. check_assembly prints every site.

    python3 tests/test_1537_cover_holes.py
"""
import contextlib
import io
import json
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

from placement import design_brief as db  # noqa: E402
from placement import floorplan as fp  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []

#: J1 at (10, 30), axis +x: the sites are at (28, 27.5) and (28, 32.5)
SITE1, SITE2 = (28.0, 27.5), (28.0, 32.5)


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def _fp(ref, x, y, crt, pads, name='t:PART', net=None):
    out = [f'  (footprint "{name}" (layer "F.Cu") (at {x} {y})',
           f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS") (hide yes))',
           f'    (fp_rect (start {crt[0]} {crt[1]}) (end {crt[2]} {crt[3]}) (stroke (width 0.05)'
           f' (type default)) (layer "F.CrtYd"))']
    for num, px, py, w, h, drill in pads:
        n = '(net 1 "GND")' if net else '(net 0 "")'
        if drill:
            out.append(f'    (pad "{num}" thru_hole circle (at {px} {py}) (size {w} {h}) '
                       f'(drill {drill}) (layers "*.Cu") {n})')
        else:
            out.append(f'    (pad "{num}" smd rect (at {px} {py}) (size {w} {h}) '
                       f'(layers "F.Cu") {n})')
    out.append('  )')
    return '\n'.join(out) + '\n'


def board(td, name, hole_at_site2=False, track=None):
    """`track`: the net id (1 GND, 2 SIG) of a B.Cu track through site 2."""
    sock = [(str(k + 1), 0.0, -4.5 + k, 0.6, 0.3, 0) for k in range(10)]
    parts = (_fp('J1', 10.0, 30.0, (-1.0, -5.0, 1.0, 5.0), sock, 't:SOCKET')
             + _fp('R1', SITE1[0], SITE1[1], (-1.0, -0.6, 1.0, 0.6),
                   [('1', -0.5, 0, 0.6, 0.6, 0), ('2', 0.5, 0, 0.6, 0.6, 0)]))
    if hole_at_site2:
        parts += _fp('H1', SITE2[0], SITE2[1], (-3, -3, 3, 3), [('1', 0, 0, 5, 5, 2.7)],
                     'MountingHole:M2.5_Pad', net=True)
    copper = (f'  (segment (start 26 32.5) (end 31 32.5) (width 0.2) (layer "B.Cu") '
              f'(net {track}))\n' if track else '')
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (net 0 "")\n  (net 1 "GND")\n  (net 2 "SIG")\n'
            '  (gr_rect (start 0 0) (end 60 60) (stroke (width 0.1) (type default))'
            ' (layer "Edge.Cuts"))\n' + parts + copper + ')\n')
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w') as fh:
        fh.write(text)
    return path


HOLES = [{'length_mm': 18, 'off_mm': -2.5, 'diameter_mm': 2.7},
         {'length_mm': 18, 'off_mm': 2.5, 'diameter_mm': 2.7}]


def cover(**extra):
    row = {'connector': 'J1', 'side': 'F', 'axis_deg': 0, 'width_mm': 8, 'length_mm': 20,
           'underside_mm': 5.0, 'removable': True, 'source': 'synthetic: a module'}
    row.update(extra)
    return row


def intent(row, reader=None):
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'min_reader': reader or fp.HOLES_READER, 'covers': [row],
            'heights': [{'ref': 'R1', 'height_mm': 0.5, 'source': 'synthetic'}]}


def report(path, doc):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    return fp.mount_report(fp.intent_from_dict(doc, ''), pcb, path)[0]


def refused(doc, needle):
    try:
        fp.intent_from_dict(doc, '')
    except fp.IntentError as ex:
        return needle in str(ex), str(ex)
    return False, 'accepted'


def schema():
    print('1. the schema')
    check('READER_VERSION reads holes', fp.READER_VERSION >= fp.HOLES_READER == 22)
    it = fp.intent_from_dict(intent(cover(holes=HOLES)), '')
    check('holes accepted, normalised', it.covers[0]['holes'][0] == {
        'length_mm': 18.0, 'off_mm': -2.5, 'diameter_mm': 2.7}, json.dumps(it.covers[0]))
    for label, h, needle in (
            ('an unknown key in a hole', {'length_mm': 1, 'diameter_mm': 2, 'ref': 'H1'},
             'unknown key(s) ref'),
            ('a hole with no diameter', {'length_mm': 1}, 'needs `diameter_mm`'),
            ('a hole with no length', {'diameter_mm': 2}, 'needs `length_mm`'),
            ('a zero diameter', {'length_mm': 1, 'diameter_mm': 0}, 'diameter_mm: expected > 0')):
        ok, msg = refused(intent(cover(holes=[h])), needle)
        check(f'refused: {label}', ok, msg)
    ok, msg = refused(intent(cover(standoffs=[{'length_mm': 18}])), 'is `holes[]`')
    check('a standoff with no ref points at holes[]', ok, msg)


def no_holes(td):
    print('2. no holes declared: nothing new')
    m = report(board(td, 'b2'), intent(cover(), reader=fp.MOUNT_READER))
    check('no `holes` key, YES as before', 'holes' not in m and m['mountable'] is True,
          json.dumps({k: m.get(k) for k in ('mountable', 'why')}))


def blocked(td):
    print('3. R1 on site 1, site 2 free')
    m = report(board(td, 'b3'), intent(cover(holes=HOLES)))
    s1, s2 = m['holes']
    check('NO', m['mountable'] is False, json.dumps(m['why']))
    check('site 1 at its declared place', s1['at'] == list(SITE1), f"{s1['at']}")
    check('site 1: R1 the part, both its pads named',
          [p['ref'] for p in s1['parts']] == ['R1']
          and sorted(p['pad'] for p in s1['pads'] if p['ref'] == 'R1') == ['1', '2']
          and not s1['fits'], json.dumps(s1))
    check('the reason names hole 1 and R1, and not hole 2',
          any(w.startswith('hole 1 ') and 'R1' in w for w in m['why'])
          and not any(w.startswith('hole 2 ') for w in m['why']), json.dumps(m['why']))
    check('site 2: clear, no standoff', s2['fits'] and s2['standoff'] is None,
          json.dumps(s2))
    check('the ring is the screw-head ring: 2.7/2 + 0.4 x 2.7', abs(s1['reach_mm'] - 2.43) < 1e-9,
          f"{s1['reach_mm']}")


def standoff(td):
    print('4. a mounting hole fitted at site 2')
    m = report(board(td, 'b4', hole_at_site2=True), intent(cover(holes=HOLES)))
    s2 = m['holes'][1]
    check("H1 is site 2's standoff, its own pad no obstruction",
          s2['standoff'] == 'H1' and s2['fits'] and not any(p['ref'] == 'H1' for p in s2['pads']),
          json.dumps(s2))


def copper(td):
    print("5. another net's track through site 2")
    m = report(board(td, 'b5', track=2), intent(cover(holes=HOLES)))
    s2 = m['holes'][1]
    check('the track is named there, and the site does not fit',
          [(h['kind'], h['net']) for h in s2['copper']] == [('track', 'SIG')] and not s2['fits'],
          json.dumps(s2))
    m = report(board(td, 'b5b', hole_at_site2=True, track=1), intent(cover(holes=HOLES)))
    check("a GND track there is the standoff's own net (its pad is GND): no obstruction",
          not m['holes'][1]['copper'] and m['holes'][1]['fits'], json.dumps(m['holes'][1]))
    m = report(board(td, 'b5c', hole_at_site2=True, track=2), intent(cover(holes=HOLES)))
    check('...but another net past the standoff still is',
          [h['net'] for h in m['holes'][1]['copper']] == ['SIG'], json.dumps(m['holes'][1]))


def brief():
    print('6. the brief compiles holes, reader 22')
    b = db.brief_from_dict({'schema': 1, 'kind': 'design-brief', 'covers': [cover(holes=HOLES)]})
    frag, _rep = db.compile_brief(b, refs_known=False)
    check('the row keeps its holes', frag['covers'][0].get('holes') == [
        {'length_mm': 18.0, 'off_mm': -2.5, 'diameter_mm': 2.7},
        {'length_mm': 18.0, 'off_mm': 2.5, 'diameter_mm': 2.7}], json.dumps(frag['covers'][0]))
    check('min_reader 22', frag.get('min_reader') == fp.HOLES_READER, str(frag.get('min_reader')))


def cli(td):
    print('7. check_assembly prints every site')
    path = board(td, 'b7')
    ip = os.path.join(td, 'i7.json')
    with open(ip, 'w') as fh:
        json.dump(intent(cover(holes=HOLES)), fh)
    r = run_utils.check(PY + [os.path.join(REPO, 'py_tools', 'check_assembly.py'), path,
                              '--intent', ip], accept=True)
    out = r.stdout
    check('NO with 2 hole sites', 'J1 NO (removable): 0 standoff(s), 2 hole site(s)' in out,
          out[-800:])
    check('site 1 blocked, site 2 clear', 'hole 1 at (28.00, 27.50)' in out
          and 'no standoff, site clear' in out, out[-800:])


def main():
    td = tempfile.mkdtemp(prefix='krt1537_')
    try:
        for t in (schema, lambda: no_holes(td), lambda: blocked(td), lambda: standoff(td),
                  lambda: copper(td), brief, lambda: cli(td)):
            try:
                t()
            except Exception as ex:                            # noqa: BLE001
                check(f'{getattr(t, "__name__", "step")} ran', False,
                      f'{type(ex).__name__}: {ex}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
