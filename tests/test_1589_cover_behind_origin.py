"""#1589: a cover's hole sites, standoffs and body may sit behind its
connector's origin.

The loader bounded `covers[].holes[].length_mm` and `.standoffs[].length_mm`
at 0, though every site is placed by the signed `heights.standoff_at`: a
module with holes on both sides of its connector could declare only one
side (`expected >= 0.0, got -1.75`), and `mountable` then said YES with
parts at the other two sites. The body had the same limit by design --
`cover_rect` started at the origin -- so the strip of a module behind its
connector was never judged. And with a site 1.75 mm from the connector, the
screw-head ring reached the connector itself, which was named at its own
site.

A synthetic board: J1 a socket at (30, 30), its module (8 x 20 mm, axis +x)
with two holes 18 mm along and two 3 mm BEHIND the origin, 2.5 mm either
side of its centre line.

  1. the schema: a signed hole and standoff length, `start_mm` accepted and
     normalised; a body that ends where it starts refused; reader 23;
  2. sites behind the origin are where `standoff_at` puts them and judged:
     R1 on one is named, NO;
  3. the cover's own connector reaches a near site's ring: reported on the
     site (`connector`), not judged; the site fits;
  4. a standoff behind the origin, fitted there, is at its place;
  5. `start_mm` behind the origin: the body covers the strip, and a part too
     tall for it there is refused, while the default body leaves it alone;
  6. the brief stamps min_reader 23 for either;
  7. check_assembly prints the near sites and the connector note.

    python3 tests/test_1589_cover_behind_origin.py
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
from placement.heights import cover_rect  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []

#: J1 at (30, 30), axis +x: the near sites at (27, 27.5) and (27, 32.5)
NEAR1, NEAR2 = (27.0, 27.5), (27.0, 32.5)
HOLES = [{'length_mm': 18, 'off_mm': -2.5, 'diameter_mm': 2.7},
         {'length_mm': 18, 'off_mm': 2.5, 'diameter_mm': 2.7},
         {'length_mm': -3, 'off_mm': -2.5, 'diameter_mm': 2.7},
         {'length_mm': -3, 'off_mm': 2.5, 'diameter_mm': 2.7}]


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


def _r(ref, x, y):
    return _fp(ref, x, y, (-1.0, -0.6, 1.0, 0.6),
               [('1', -0.5, 0, 0.6, 0.6, 0), ('2', 0.5, 0, 0.6, 0.6, 0)])


def board(td, name, r1=False, h1=False, r2=False):
    """`r1`: a resistor on near site 1; `h1`: a mounting hole fitted at near
    site 2; `r2`: a resistor in the strip behind the origin, between the
    near sites' rings."""
    sock = [(str(k + 1), 0.0, -4.5 + k, 0.6, 0.3, 0) for k in range(10)]
    parts = _fp('J1', 30.0, 30.0, (-1.0, -5.0, 1.0, 5.0), sock, 't:SOCKET')
    if r1:
        parts += _r('R1', *NEAR1)
    if h1:
        parts += _fp('H1', NEAR2[0], NEAR2[1], (-2.1, -2.1, 2.1, 2.1), [('1', 0, 0, 4, 4, 2.2)],
                     'MountingHole:M2_Pad', net=True)
    if r2:
        parts += _r('R2', 26.0, 30.0)
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (net 0 "")\n  (net 1 "GND")\n'
            '  (gr_rect (start 0 0) (end 60 60) (stroke (width 0.1) (type default))'
            ' (layer "Edge.Cuts"))\n' + parts + ')\n')
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w') as fh:
        fh.write(text)
    return path


def cover(**extra):
    row = {'connector': 'J1', 'side': 'F', 'axis_deg': 0, 'width_mm': 8, 'length_mm': 20,
           'underside_mm': 5.0, 'removable': True, 'source': 'synthetic: a module'}
    row.update(extra)
    return row


def intent(row):
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'min_reader': fp.BEHIND_READER, 'covers': [row],
            'heights': [{'ref': 'R*', 'height_mm': 6.0, 'source': 'synthetic: too tall'}]}


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
    check('READER_VERSION 23 reads it', fp.READER_VERSION >= fp.BEHIND_READER == 23)
    it = fp.intent_from_dict(intent(cover(holes=HOLES, start_mm=-4,
                                          standoffs=[{'ref': 'H1', 'length_mm': -3,
                                                      'off_mm': 2.5}])), '')
    c = it.covers[0]
    check('a hole behind the origin kept signed',
          c['holes'][2] == {'length_mm': -3.0, 'off_mm': -2.5, 'diameter_mm': 2.7},
          json.dumps(c['holes']))
    check('a standoff behind the origin kept signed',
          c['standoffs'][0]['length_mm'] == -3.0, json.dumps(c['standoffs']))
    check('start_mm kept', c['start_mm'] == -4.0, json.dumps(c))
    check('the body runs from start_mm to length_mm',
          cover_rect(c) == (-4.0, -4.0, 20.0, 4.0), str(cover_rect(c)))
    check('without start_mm it starts at the origin, as before',
          cover_rect(fp.intent_from_dict(intent(cover()), '').covers[0])
          == (0.0, -4.0, 20.0, 4.0))
    for label, row, needle in (
            ('a start past the far end', cover(start_mm=20), 'reaches nowhere'),
            ('only standoffs behind the origin, no length',
             dict(cover(standoffs=[{'ref': 'H1', 'length_mm': -3}]), length_mm=None),
             'reaches nowhere'),
            ('a start that is not a number', cover(start_mm='back'), 'start_mm')):
        ok, msg = refused(intent(row), needle)
        check(f'refused: {label}', ok, msg)
    ok = fp.intent_from_dict(intent(dict(cover(start_mm=-8, standoffs=[
        {'ref': 'H1', 'length_mm': -3}]), length_mm=None)), '')
    check('a body ending at a standoff behind the origin, with start_mm before it',
          cover_rect(ok.covers[0]) == (-8.0, -4.0, -3.0, 4.0), str(cover_rect(ok.covers[0])))


def sites(td):
    print('2-3. sites behind the origin, judged; the own connector reported')
    m = report(board(td, 'b2', r1=True), intent(cover(holes=HOLES)))
    s = m['holes']
    check('four sites, the near ones where standoff_at puts them',
          len(s) == 4 and s[2]['at'] == list(NEAR1) and s[3]['at'] == list(NEAR2),
          json.dumps([x['at'] for x in s]))
    check('R1 on near site 1 is named and the cover is NO',
          [p['ref'] for p in s[2]['parts']] == ['R1'] and m['mountable'] is False
          and any(w.startswith('hole 3 ') and 'R1' in w for w in m['why']),
          json.dumps(m['why']))
    m = report(board(td, 'b3'), intent(cover(holes=HOLES)))
    s = m['holes']
    check('with the sites clear it is YES', m['mountable'] is True, json.dumps(m['why']))
    check("J1's courtyard reaches each near ring, reported on the site and not judged",
          all(x['connector'] and x['connector']['ref'] == 'J1' and x['fits']
              and not any(p['ref'] == 'J1' for p in x['parts'] + x['pads'])
              for x in s[2:]), json.dumps(s[2:]))
    check('...and the far sites carry no connector', all(x['connector'] is None for x in s[:2]),
          json.dumps(s[:2]))


def standoff(td):
    print('4. a standoff behind the origin, fitted')
    doc = intent(cover(holes=HOLES, standoffs=[{'ref': 'H1', 'length_mm': -3, 'off_mm': 2.5}]))
    m = report(board(td, 'b4', h1=True), doc)
    check('H1 at its place (no miss) and the site\'s standoff',
          m['standoffs'][0]['miss_mm'] < 1e-6 and m['holes'][3]['standoff'] == 'H1'
          and m['mountable'] is True, json.dumps({k: m[k] for k in ('standoffs', 'why')}))


def body(td):
    print('5. start_mm: the strip behind the origin is under the body')
    path = board(td, 'b5', r2=True)
    m = report(path, intent(cover()))
    check('the default body leaves R2 alone (YES)',
          m['mountable'] is True and not m['blocked'], json.dumps(m['why']))
    m = report(path, intent(cover(start_mm=-6)))
    check('with start_mm -6, R2 (6 mm) is refused under the 5 mm underside',
          m['mountable'] is False and [b['ref'] for b in m['blocked']] == ['R2'],
          json.dumps(m['why']))


def brief():
    print('6. the brief stamps reader 23')
    for label, row in (('a hole behind the origin', cover(holes=HOLES)),
                       ('start_mm', cover(start_mm=-2))):
        b = db.brief_from_dict({'schema': 1, 'kind': 'design-brief', 'covers': [row]})
        frag, _rep = db.compile_brief(b, refs_known=False)
        check(f'{label}: min_reader 23', frag.get('min_reader') == fp.BEHIND_READER,
              str(frag.get('min_reader')))
    b = db.brief_from_dict({'schema': 1, 'kind': 'design-brief',
                            'covers': [cover(holes=HOLES[:2])]})
    frag, _rep = db.compile_brief(b, refs_known=False)
    check('far holes only: min_reader 22 as before', frag.get('min_reader') == fp.HOLES_READER,
          str(frag.get('min_reader')))


def cli(td):
    print('7. check_assembly')
    path = board(td, 'b7')
    ip = os.path.join(td, 'i7.json')
    with open(ip, 'w') as fh:
        json.dump(intent(cover(holes=HOLES)), fh)
    r = run_utils.check(PY + [os.path.join(REPO, 'py_tools', 'check_assembly.py'), path,
                              '--intent', ip], accept=True)
    out = r.stdout
    check('YES with 4 hole sites', 'J1 YES (removable): 0 standoff(s), 4 hole site(s)' in out,
          out[-800:])
    line = next((l for l in out.splitlines() if 'hole 3 at (27.00, 27.50)' in l), '')
    check('near site 3 printed clear, the connector named and not judged',
          'site clear' in line and 'its connector J1 reaches the ring' in line, line or out[-800:])


def main():
    td = tempfile.mkdtemp(prefix='krt1589_')
    try:
        for t in (schema, lambda: sites(td), lambda: standoff(td), lambda: body(td), brief,
                  lambda: cli(td)):
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
