"""#1504: can every removable body be mounted -- corner standoffs, a body's
own top, the path a card goes in by, and a `mountable` report.

A synthetic carrier board (written here, 80 x 60 mm): J1 a module socket
whose module (40 x 45 mm, underside 4.0, top 5.5) is screwed down at its two
far corners, H1 and H2, 16 mm either side of its centre line; J2 an M.2
socket under the module, its card (22 x 30 mm, underside 1.2, top 2.6) held
by H3; C1 and R1 small parts under the module, clear of the card; C9 a
tall capacitor.

  1. `standoffs[].off_mm`: the corner screws graded at their offsets.
  2. `covers[].top_mm`: the card under the module fits; with the module's
     underside lowered to 2.0 the two bodies are a height pair.
  3. `mountable`: YES with everything in place; NO naming a standoff off
     its place, a part too tall under the module.
  4. `insertion`: the card's path passes under the removable module, so it
     goes in first; with the module fixed it is blocked; a tall part just
     past the card's far end, inside its path, is refused, and past the
     path's reach it is not.
  5. place_pose refuses a move that puts the card's body into the module's.

    python3 tests/test_1504_mountable.py
"""
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
sys.path.insert(0, os.path.join(REPO, 'py_router'))
sys.path.insert(0, os.path.join(REPO, 'py_placer'))
sys.path.insert(0, TESTS)
import run_utils  # noqa: E402

from placement import floorplan as fp  # noqa: E402
from placement import heights as hm  # noqa: E402

PY = [sys.executable, '-X', 'utf8']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def _fp(ref, x, y, crt, pads, name='t:PART'):
    """A footprint with a courtyard `crt` (local) and `pads` [(num, x, y, w, h, drill)]."""
    out = [f'  (footprint "{name}" (layer "F.Cu") (at {x} {y})',
           f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS") (hide yes))',
           f'    (fp_rect (start {crt[0]} {crt[1]}) (end {crt[2]} {crt[3]}) (stroke (width 0.05)'
           f' (type default)) (layer "F.CrtYd"))']
    for num, px, py, w, h, drill in pads:
        if drill:
            out.append(f'    (pad "{num}" thru_hole circle (at {px} {py}) (size {w} {h}) '
                       f'(drill {drill}) (layers "*.Cu") (net 0 ""))')
        else:
            out.append(f'    (pad "{num}" smd rect (at {px} {py}) (size {w} {h}) '
                       f'(layers "F.Cu") (net 0 ""))')
    out.append('  )')
    return '\n'.join(out) + '\n'


def _two(ref, x, y):
    return _fp(ref, x, y, (-1.0, -0.6, 1.0, 0.6), [('1', -0.5, 0, 0.6, 0.6, 0),
                                                   ('2', 0.5, 0, 0.6, 0.6, 0)])


def _hole(ref, x, y):
    return _fp(ref, x, y, (-3, -3, 3, 3), [('1', 0, 0, 5, 5, 2.7)], 'MountingHole:M2.5')


def _socket(ref, x, y, n, pitch, half):
    pads = [(str(k + 1), -0.6 if k % 2 else 0.6, -half + 1 + (k // 2) * pitch, 0.4, 0.3, 0)
            for k in range(n)]
    return _fp(ref, x, y, (-1.5, -half, 1.5, half), pads, 't:SOCKET')


def board(td, h1=(52.0, 46.0), c9=None, j2=(20.0, 30.0)):
    parts = (_socket('J1', 10.0, 30.0, 40, 1.0, 11.0)
             + _socket('J2', j2[0], j2[1], 20, 1.0, 6.0)
             + _hole('H1', *h1) + _hole('H2', 52.0, 14.0) + _hole('H3', j2[0] + 30.0, j2[1])
             + _two('C1', 40.0, 15.0) + _two('R1', 30.0, 44.0)
             + (_two('C9', *c9) if c9 else ''))
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (net 0 "")\n'
            '  (gr_rect (start 0 0) (end 80 60) (stroke (width 0.1) (type default))'
            ' (layer "Edge.Cuts"))\n' + parts + ')\n')
    path = os.path.join(td, f'b{len(os.listdir(td))}.kicad_pcb')
    with open(path, 'w') as fh:
        fh.write(text)
    return path


def intent(module=None, card=None, heights_extra=(), offs=(16.0, -16.0)):
    m = {'connector': 'J1', 'side': 'F', 'axis_deg': 0, 'width_mm': 40, 'length_mm': 45,
         'underside_mm': 4.0, 'top_mm': 5.5, 'removable': True,
         'standoffs': [{'ref': 'H1', 'length_mm': 42, 'off_mm': offs[0]},
                       {'ref': 'H2', 'length_mm': 42, 'off_mm': offs[1]}],
         'source': 'synthetic: a compute module'}
    c = {'connector': 'J2', 'side': 'F', 'axis_deg': 0, 'width_mm': 22, 'length_mm': 30,
         'underside_mm': 1.2, 'top_mm': 2.6, 'removable': True,
         'standoffs': [{'ref': 'H3', 'length_mm': 30}], 'source': 'synthetic: an M.2 2230 card'}
    m.update(module or {})
    c.update(card or {})
    for row in (m, c):
        if row.get('standoffs') is None:
            row.pop('standoffs')
    hs = [{'ref': 'J2', 'height_mm': 2.3, 'under_cover': True, 'source': 'socket datasheet'},
          {'ref': 'H3', 'height_mm': 2.5, 'under_cover': True, 'source': 'standoff datasheet'},
          {'ref': 'C*', 'height_mm': 0.9, 'source': 's'}, {'ref': 'R*', 'height_mm': 0.5, 'source': 's'}]
    hs = [dict(h) for h in heights_extra] + hs
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'min_reader': fp.MOUNT_READER, 'covers': [m, c], 'heights': hs}


def report(td, pcb_path, doc):
    from kicad_parser import parse_kicad_pcb
    it = fp.intent_from_dict(doc, '')
    rows = fp.mount_report(it, parse_kicad_pcb(pcb_path), pcb_path)
    return {r['connector']: r for r in rows}


def save(td, doc, name):
    p = os.path.join(td, name)
    with open(p, 'w') as fh:
        json.dump(doc, fh)
    return p


def run(argv):
    return subprocess.run(PY + argv, capture_output=True, text=True, cwd=REPO)


def corners(td):
    print('1. corner standoffs (`off_mm`)')
    b = board(td)
    g = run([os.path.join('py_tools', 'check_floorplan.py'), b, '--intent',
             save(td, intent(), 'i1.json')])
    check('declared at +-16 mm, no cover warning', '[warn ] cover:' not in g.stdout, g.stdout[-500:])
    g = run([os.path.join('py_tools', 'check_floorplan.py'), b, '--intent',
             save(td, intent(offs=(0.0, 0.0)), 'i1b.json')])
    check('declared on the line, both read off it', '2 of 2 standoff(s) off the place' in g.stdout,
          g.stdout[-500:])


def thickness(td):
    print('2. a body under another body (`top_mm`)')
    b = board(td)
    r = report(td, b, intent())
    check('the card (top 2.6) fits under the module (underside 4.0)',
          not [x for x in r['J1']['blocked'] if x['ref'] == 'J2'], json.dumps(r['J1']['blocked']))
    r = report(td, b, intent(module={'underside_mm': 2.0, 'top_mm': 3.5}))
    check('with the module at 2.0 the two bodies are refused together',
          [x for x in r['J1']['blocked'] if x['ref'] == 'J2'], json.dumps(r['J1']['blocked']))
    r = report(td, b, intent(card={'top_mm': None}))
    check('a card of unknown top is never cleared under it',
          [x for x in r['J1']['blocked'] if x['ref'] == 'J2'], json.dumps(r['J1']['blocked']))


def mountable(td):
    print('3. the mountable report')
    b = board(td)
    r = report(td, b, intent())
    check('everything in place: J1 and J2 mountable',
          r['J1']['mountable'] is True and r['J2']['mountable'] is True,
          json.dumps({k: r[k]['why'] for k in r}))
    check("...the least room under the module is read (H3, the card's standoff: 1.5 mm)",
          (r['J1']['least_room'] or {}).get('ref') == 'H3'
          and (r['J1']['least_room'] or {}).get('room_mm') == 1.5, json.dumps(r['J1']['least_room']))
    r = report(td, board(td, h1=(52.0, 49.0)), intent())
    check('H1 3 mm off its corner: not mountable, H1 named',
          r['J1']['mountable'] is False and any('H1' in w for w in r['J1']['why']),
          json.dumps(r['J1']['why']))
    r = report(td, board(td, c9=(30.0, 25.0)), intent(heights_extra=[
        {'ref': 'C9', 'height_mm': 8.0, 'source': 'an electrolytic'}]))
    check('an 8 mm capacitor under the module: not mountable, C9 named',
          r['J1']['mountable'] is False and any('C9' in w for w in r['J1']['why']),
          json.dumps(r['J1']['why']))
    g = run([os.path.join('py_tools', 'check_floorplan.py'), board(td, c9=(30.0, 25.0)),
             '--intent', save(td, intent(heights_extra=[
                 {'ref': 'C9', 'height_mm': 8.0, 'source': 'an electrolytic'}]), 'i3.json')])
    check('check_floorplan warns `mountable`', '[warn ] mountable: covers[0] (J1)' in g.stdout,
          g.stdout[-500:])


def insertion(td):
    print('4. the way in (`insertion`)')
    sw = hm._sweep({'axis_deg': 0, 'width_mm': 22, 'length_mm': 30, 'underside_mm': 1.2,
                    'top_mm': 2.6, 'insertion': {'angle_deg': 20, 'travel_mm': 4}})
    lo, hi = hm.sweep_at(sw, 0.0, 1.0)
    check('at the pivot the path is as low as the body', abs(lo - 1.2) < 1e-6, f'{lo}')
    want = 1.2 + 30 * math.tan(math.radians(20)) + 1.4 / math.cos(math.radians(20))
    lo, hi = hm.sweep_at(sw, 29.5, 30.0)
    check('at its far end it rises to the tilted top', abs(hi - want) < 0.05, f'{hi} vs {want}')
    check("its reach is the slide's: (30 + 4) cos 20", abs(sw['reach'] - 34 * math.cos(
        math.radians(20))) < 1e-6, f"{sw['reach']}")
    b = board(td)
    card = {'insertion': {'angle_deg': 20, 'travel_mm': 4}}
    r = report(td, b, intent(card=card))
    check('the card goes in before the removable module',
          r['J2']['before'] == ['J1'] and r['J1']['after'] == ['J2'],
          json.dumps({k: (r[k]['before'], r[k]['after']) for k in r}))
    r = report(td, b, intent(card=card, module={'removable': False}))
    check('with the module fixed, the card cannot go in',
          r['J2']['mountable'] is False and any('J1' in w for w in r['J2']['why']),
          json.dumps(r['J2']['why']))
    # past the card's far end (x 50), inside its reach (x 51.9): the path's
    # underside there is about 1.2 + 31 tan 20 = 12.5 mm, so a 14 mm part is
    # in the way and a 0.9 mm one is not; past the reach nothing is
    tall = [{'ref': 'C9', 'height_mm': 14.0, 'source': 's'}]
    nomod = {'width_mm': 4, 'length_mm': 5, 'standoffs': None}
    for label, c9, hx, want in (('a 14 mm part just past the card', (51.0, 30.0), tall, True),
                                ('a 0.9 mm part there', (51.0, 30.0), (), False),
                                ('a 14 mm part past the reach', (58.0, 30.0), tall, False)):
        r = report(td, board(td, c9=c9), intent(card=card, module=nomod, heights_extra=hx))
        got = any(x['ref'] == 'C9' for x in r['J2']['blocked'])
        check(f'{label}: {"in the way" if want else "clear"}', got == want,
              json.dumps(r['J2']['blocked']))


def pose(td):
    print('5. place_pose refuses a move that meets the two bodies')
    b = board(td)
    doc = intent(module={'underside_mm': 2.0, 'top_mm': 3.5, 'standoffs': None,
                         'width_mm': 20})
    doc['covers'][1].pop('standoffs')
    p = save(td, doc, 'i5.json')
    # J2 starts beside the module (a 20 mm one, y 20-40: the card, y 41-63,
    # clear of it), and is moved under it
    b = board(td, j2=(20.0, 52.0))
    r = run([os.path.join('py_placer', 'place_pose.py'), b, os.path.join(td, 'o.kicad_pcb'),
             'set', 'J2', '20', '30', '--intent', p])
    check('refused (exit 4) naming the J1/J2 height pair',
          r.returncode == 4 and 'height_pairs new: J1/J2' in r.stdout + r.stderr,
          f'rc {r.returncode}: {(r.stdout + r.stderr)[-400:]}')


def main():
    td = tempfile.mkdtemp(prefix='krt1504_')
    try:
        for t in (corners, thickness, mountable, insertion, pose):
            t(td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
