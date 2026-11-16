"""#1833: a cover's mounting hole may screw into the enclosure, past the board edge.

Since #1823 every `covers[].holes[]` site had to be on the board: the grade
read a site past the outline NOT mountable, and place_simple refused every
connector pose that put a site off the board (exit 4 when none kept them
all on). A module whose hole is meant for an enclosure boss past the board
edge could not say so: the row took no key for it. Now a hole row takes
`mount`, `board` (the default, #1823's rule) or `enclosure`: such a site is
reported where it lands and how far past the outline, and neither the grade
nor the seat holds it on the board. Its screw-head ring is judged as any
site's.

A synthetic board (60 x 60 mm): J1 a socket whose module (8 x 72 mm) has
hole 1 4 mm along its axis and hole 2 70 mm along.

  1. the schema: `mount` read, `board` the default, a bad value refused;
  2. hole 2 `enclosure`, 21.35 mm past the outline: YES, the site reported
     with its distance and `mount`; control: the same hole as `board` is NO;
  3. an `enclosure` site on the board with R1 in its ring is NO for R1;
  4. the seat state leaves the enclosure hole out, so pose_ok admits J1
     where hole 2 is off the board and still refuses hole 1 off it;
  5. place_simple seats J1 (exit 0, where #1823 left it illegal, exit 4),
     and check_assembly reads the placed board YES;
  6. check_assembly prints the enclosure site as such;
  7. the design brief stamps min_reader 28 for a hole carrying `mount`.

    python3 tests/test_1833_cover_hole_enclosure.py
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
NEAR = {'length_mm': 4, 'off_mm': 3, 'diameter_mm': 2.7}
FAR = {'length_mm': 70, 'diameter_mm': 2.7, 'why': 'an enclosure boss, off the board'}


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


def board(td, name, j1=(10.0, 30.0, 0), r1=None):
    sock = [(str(k + 1), 0.0, -4.5 + k, 0.6, 0.3) for k in range(10)]
    parts = _fp('J1', *j1, (-1.0, -5.0, 1.0, 5.0), sock, 't:SOCKET')
    if r1:
        parts += _fp('R1', r1[0], r1[1], 0, (-1.0, -0.6, 1.0, 0.6),
                     [('1', -0.5, 0, 0.6, 0.6), ('2', 0.5, 0, 0.6, 0.6)])
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (net 0 "")\n'
            '  (gr_rect (start 0 0) (end 60 60) (stroke (width 0.1) '
            '(type default)) (layer "Edge.Cuts"))\n' + parts + ')\n')
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w') as fh:
        fh.write(text)
    return path


def cover(holes, length=72):
    return {'connector': 'J1', 'side': 'F', 'axis_deg': 0, 'width_mm': 8, 'length_mm': length,
            'underside_mm': 5.0, 'removable': True, 'source': 'synthetic: an overhanging module',
            'holes': [dict(h) for h in holes]}


def intent(holes, length=72):
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'min_reader': fp.HOLE_MOUNT_READER, 'covers': [cover(holes, length)],
            'heights': [{'ref': 'R1', 'height_mm': 0.5, 'source': 'synthetic'}]}


def encl(h):
    return dict(h, mount='enclosure')


def report(path, doc):
    from kicad_parser import parse_kicad_pcb
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    return fp.mount_report(fp.intent_from_dict(doc, ''), pcb, path)[0]


def schema(td):
    print('1. the schema')
    check('READER_VERSION reads mount', fp.READER_VERSION >= fp.HOLE_MOUNT_READER == 28)
    it = fp.intent_from_dict(intent([dict(NEAR, mount='board'), encl(FAR)]), '')
    h1, h2 = it.covers[0]['holes']
    check('`board` is the default and is not written back', 'mount' not in h1, json.dumps(h1))
    check('`enclosure` is kept', h2.get('mount') == 'enclosure', json.dumps(h2))
    try:
        fp.intent_from_dict(intent([dict(NEAR, mount='chassis')]), '')
        ok, msg = False, 'accepted'
    except fp.IntentError as ex:
        msg = str(ex)
        ok = 'holes[0].mount' in msg and 'board, enclosure' in msg
    check('a value that is neither is refused, naming both', ok, msg)


def graded(td):
    print('2. hole 2 into the enclosure, past the outline')
    path = board(td, 'b2')
    m = report(path, intent([NEAR, encl(FAR)]))
    s1, s2 = m['holes']
    check('YES', m['mountable'] is True, json.dumps(m['why']))
    check('hole 2 reported where it lands, 21.35 mm past, as the enclosure\'s',
          s2['at'] == [80.0, 30.0] and abs(s2['past_outline_mm'] - 21.35) < 0.01
          and s2['mount'] == 'enclosure' and s2['fits'] is True, json.dumps(s2))
    check('hole 1 is the board\'s', s1['mount'] == 'board' and s1['fits'], json.dumps(s1))
    m = report(path, intent([NEAR, FAR]))
    check('control: the same hole on the board is NO, as #1823 grades it',
          m['mountable'] is False and any(w.startswith('hole 2 ') and 'past the outline' in w
                                          for w in m['why']), json.dumps(m['why']))


def ring(td):
    print('3. an enclosure site\'s ring is judged as any site\'s')
    m = report(board(td, 'b3', r1=(14.0, 33.0)), intent([encl(NEAR)]))
    s = m['holes'][0]
    check('NO for R1', m['mountable'] is False and [p['ref'] for p in s['parts']] == ['R1']
          and not s['fits'], json.dumps(m['why']))


def _state(path, doc):
    from kicad_parser import parse_kicad_pcb
    from placement.edge_seat import seat_state
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
        return seat_state(pcb, path, fp.intent_from_dict(doc, ''), {}, 0.2, 0.2)


def seat(td):
    print('4. the seat leaves the enclosure hole out')
    from placement.seeder import _fixed_pose_check, cover_holes_past, pose_ok
    path = board(td, 'b4')
    st = _state(path, intent([NEAR, encl(FAR)]))
    check('only hole 1 is a seat site, with its own index',
          getattr(st, 'cover_hole_sites', {}).get('J1') == [(4.0, 3.0, 2.7, 0)],
          str(getattr(st, 'cover_hole_sites', None)))
    check('pose_ok admits J1 with hole 2 off the board', pose_ok(st, 'J1', 10.0, 30.0, 0.0, set())
          and not cover_holes_past(st, 'J1', 10.0, 30.0, 0.0))
    how, why, _c = _fixed_pose_check(st, 'J1', (10.0, 30.0, 0.0), {})
    check('...and so does a declared pose there', how is not None, f'{how} {why}')
    # J1 turned: hole 1 at x = 10 - 4 = 6, still on; at x = 3: 3 - 4 = -1, off
    check('pose_ok still refuses hole 1 off the board',
          not pose_ok(st, 'J1', 3.0, 30.0, 180.0, set())
          and cover_holes_past(st, 'J1', 3.0, 30.0, 180.0)
          and cover_holes_past(st, 'J1', 3.0, 30.0, 180.0)[0][0] == 0)
    st = _state(path, intent([NEAR, FAR]))
    check('control: with hole 2 the board\'s, the same pose is refused naming hole 2',
          not pose_ok(st, 'J1', 10.0, 30.0, 0.0, set())
          and [k for k, _mm in cover_holes_past(st, 'J1', 10.0, 30.0, 0.0)] == [1])


def placed(td):
    print('5. place_simple seats J1')
    path = run_utils.evidence(board(td, 'b5'), 'the board')
    ip, gp = os.path.join(td, 'i5.json'), os.path.join(td, 'g5.json')
    with open(ip, 'w') as fh:
        json.dump(intent([NEAR, encl(FAR)]), fh)
    with open(gp, 'w') as fh:
        json.dump({'schema': 1, 'groups': [{'name': 'all', 'refs': ['J1']}]}, fh)
    run_utils.evidence(ip, 'the intent')
    run_utils.evidence(gp, 'the groups')
    out = os.path.join(td, 's0.kicad_pcb')
    r = run_utils.check(PY + [os.path.join(REPO, 'py_placer', 'place_simple.py'), path,
                              os.path.join(td, 's{seed}.kicad_pcb'), '--intent', ip,
                              '--groups', gp, '--seeds', '0'], accept=True)
    check('exit 0, J1 not illegal', r.returncode == 0 and 'illegal' not in r.stdout,
          (r.stdout + r.stderr)[-800:])
    run_utils.evidence(out, 'the placed board')
    r = run_utils.check(PY + [os.path.join(REPO, 'py_tools', 'check_assembly.py'), out,
                              '--intent', ip], accept=True)
    check('the placed board: J1 mountable YES', 'mountable (#1504): J1 YES' in r.stdout,
          r.stdout[-800:])


def cli(td):
    print('6. check_assembly prints the enclosure site')
    path = run_utils.evidence(board(td, 'b6'), 'the board')
    ip = os.path.join(td, 'i6.json')
    with open(ip, 'w') as fh:
        json.dump(intent([NEAR, encl(FAR)]), fh)
    run_utils.evidence(ip, 'the intent')
    r = run_utils.check(PY + [os.path.join(REPO, 'py_tools', 'check_assembly.py'), path,
                              '--intent', ip], accept=True)
    row = next((ln for ln in r.stdout.splitlines() if 'hole 2 at (' in ln), '')
    check('J1 YES', 'mountable (#1504): J1 YES (removable)' in r.stdout, r.stdout[-800:])
    check('hole 2 printed as the enclosure\'s, past the outline, not judged',
          'into the enclosure' in row and 'site clear' in row
          and 'past the outline (reported, not judged)' in row
          and 'no board under the hole' not in row, repr(row))


def brief(td):
    print('7. the brief stamps reader 28')
    b = db.brief_from_dict({'schema': 1, 'kind': 'design-brief',
                            'covers': [cover([NEAR, encl(FAR)])]})
    frag, _rep = db.compile_brief(b, refs_known=False)
    check('a hole with `mount`: min_reader 28', frag.get('min_reader') == fp.HOLE_MOUNT_READER,
          str(frag.get('min_reader')))
    b = db.brief_from_dict({'schema': 1, 'kind': 'design-brief',
                            'covers': [cover([NEAR, FAR])]})
    frag, _rep = db.compile_brief(b, refs_known=False)
    check('control: no `mount`, min_reader 22 as before', frag.get('min_reader') == fp.HOLES_READER,
          str(frag.get('min_reader')))


def main():
    td = tempfile.mkdtemp(prefix='krt1833_')
    try:
        for t in (schema, graded, ring, seat, placed, cli, brief):
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
