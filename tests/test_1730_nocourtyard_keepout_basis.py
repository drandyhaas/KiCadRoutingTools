"""#1730: a part that draws no courtyard is read on ONE set of shapes by every
keep-out reader -- the estimated courtyard place_simple seats it on (#1300).

place_simple's seat reads such a part on its estimate (its drawn body united
with its pads), and so does a cover hole site's ring (#1712). check_floorplan's
`keepout` rule and place_pose's keep-out verdict (`keepout_findings`) read it
on its pad box alone, so a part whose body stands well past its pads was
refused by the placer and admitted by the pose tool and the grade.

A synthetic board: C1 has no courtyard, a 5 x 5 mm F.Fab body and two pads
whose box is 4 x 1 mm; J1 a socket whose module has one 2.7 mm hole
(ring 2.43 mm) at (28, 27.5). Each case asks four readers whether C1 is in the
intent's keep-out:

  A. a keep-out rect over the body's lower band, clear of the pads: the seat,
     the grade and place_pose all hold C1 (the grade and place_pose did not on
     the base code);
  B. C1 0.5 mm from the hole site by its body, 2.5 mm by its pads, and a
     keep-out circle of the ring's radius on the site: the seat, the grade,
     place_pose and the ring all hold C1;
  C. control: a keep-out clear of the body: nobody holds C1;
  D. a part WITH a drawn courtyard whose pad reaches past it into a keep-out
     (the courtyard clear): place_pose refuses it, as the grade does (#1588;
     `keepout_findings` read no pad copper on the base code).

    python3 tests/test_1730_nocourtyard_keepout_basis.py
"""
import contextlib
import io
import json
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

PY = [sys.executable, '-X', 'utf8']
failures = []
SITE, RING = (28.0, 27.5), 2.43


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def board(path, ref, at, courtyard=False):
    """J1 and one part `ref` at `at`: with `courtyard`, a 3 x 3 mm drawn
    courtyard and pads reaching 0.7 mm past its x ends; without, a 5 x 5 mm
    F.Fab body and no courtyard."""
    sock = ''.join(f'    (pad "{k + 1}" smd rect (at 0 {-4.5 + k}) (size 0.6 0.3) (layers "F.Cu")'
                   f' (net 0 ""))\n' for k in range(10))
    if courtyard:
        body = ('    (fp_rect (start -1.5 -1.5) (end 1.5 1.5) (stroke (width 0.05) (type default))'
                ' (layer "F.CrtYd"))\n'
                + ''.join(f'    (pad "{n}" smd rect (at {x} {y}) (size 0.6 0.4) (layers "F.Cu")'
                          f' (net {1 + n % 2} "{("GND", "SIG")[n % 2]}"))\n'
                          for n, x, y in ((1, -1.9, -0.5), (2, -1.9, 0.5), (3, 1.9, 0.5),
                                          (4, 1.9, -0.5))))
    else:
        body = ('    (fp_rect (start -2.5 -2.5) (end 2.5 2.5) (stroke (width 0.1) (type default))'
                ' (layer "F.Fab"))\n'
                '    (pad "1" smd rect (at -1.5 0) (size 1 1) (layers "F.Cu") (net 1 "GND"))\n'
                '    (pad "2" smd rect (at 1.5 0) (size 1 1) (layers "F.Cu") (net 2 "SIG"))\n')
    text = ('(kicad_pcb (version 20240108) (generator pcbnew)\n'
            '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            '  (net 0 "")\n  (net 1 "GND")\n  (net 2 "SIG")\n'
            '  (gr_rect (start 0 0) (end 60 60) (stroke (width 0.1) (type default))'
            ' (layer "Edge.Cuts"))\n'
            '  (footprint "t:SOCKET" (layer "F.Cu") (at 10 30)\n'
            '    (property "Reference" "J1" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            '    (fp_rect (start -1 -5) (end 1 5) (stroke (width 0.05) (type default))'
            ' (layer "F.CrtYd"))\n' + sock + '  )\n'
            f'  (footprint "t:PART" (layer "F.Cu") (at {at[0]} {at[1]})\n'
            f'    (property "Reference" "{ref}" (at 0 0) (layer "F.SilkS") (hide yes))\n'
            + body + '  )\n)\n')
    with open(path, 'w') as fh:
        fh.write(text)
    return run_utils.evidence(path, 'the board')


def intent(ref, keepout):
    return {'schema': 1, 'kind': 'floorplan-intent', 'units': 'mm',
            'min_reader': fp.HOLES_READER,
            'covers': [{'connector': 'J1', 'side': 'F', 'axis_deg': 0, 'width_mm': 8,
                        'length_mm': 20, 'underside_mm': 5.0, 'removable': True,
                        'source': 'synthetic: a module',
                        'holes': [{'length_mm': 18, 'off_mm': -2.5, 'diameter_mm': 2.7}]}],
            'heights': [{'ref': ref, 'height_mm': 0.5, 'source': 'synthetic'}],
            'keepouts': [dict(keepout, name='k', why='synthetic')]}


def readers(td, tag, ref, at, keepout, courtyard=False):
    """{seat, grade, place_pose, ring}: does each hold `ref` at `at`?"""
    import pose_score
    from kicad_parser import parse_kicad_pcb
    from placement.seeder import pose_ok
    b = board(os.path.join(td, f'{tag}.kicad_pcb'), ref, at, courtyard)
    doc = intent(ref, keepout)
    ip = os.path.join(td, f'{tag}.json')
    with open(ip, 'w') as fh:
        json.dump(doc, fh)
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(b)
        it = fp.intent_from_dict(doc, '')
        st = pose_score.make_state(pcb, b, keepouts=it.keepouts, estimate_courtyards=True)
        seat = not pose_ok(st, ref, at[0], at[1], 0.0, exclude=set())
        ring = [p['ref'] for p in fp.mount_report(it, pcb, b)[0]['holes'][0]['parts']]
    out = os.path.join(td, f'{tag}_fp.json')
    r = subprocess.run(PY + [os.path.join(REPO, 'py_tools', 'check_floorplan.py'), b,
                             '--intent', ip, '--json', out], capture_output=True, text=True)
    check(f'{tag}: check_floorplan graded the board', r.returncode in (0, 4)
          and 'Traceback' not in r.stdout + r.stderr, (r.stdout + r.stderr)[-600:])
    run_utils.evidence(out, 'the floorplan grade')
    grade = ref in [v['ref'] for v in json.load(open(out))['violations'] if v['rule'] == 'keepout']
    # place_pose: the part off to the side, then `set` to `at` under the intent
    b0 = board(os.path.join(td, f'{tag}_off.kicad_pcb'), ref, (45.0, 45.0), courtyard)
    r = subprocess.run(PY + [os.path.join(REPO, 'py_placer', 'place_pose.py'), b0,
                             os.path.join(td, f'{tag}_pp.kicad_pcb'), '--intent', ip,
                             'set', ref, repr(at[0]), repr(at[1]), '--rot', '0'],
                       capture_output=True, text=True)
    text = r.stdout + r.stderr
    check(f'{tag}: place_pose answered', r.returncode in (0, 4) and 'Traceback' not in text,
          text[-600:])
    pose = r.returncode == 4 and f'intent_keepout_pairs new: {ref}/k' in text
    return {'seat': seat, 'grade': grade, 'place_pose': pose, 'ring': ref in ring}


def main():
    td = tempfile.mkdtemp(prefix='krt1730_')
    try:
        print('A. a keep-out over the body, clear of the pads')
        got = readers(td, 'A', 'C1', (20.0, 20.0), {'rect': [18.0, 21.5, 22.0, 23.0]})
        check('A: the seat, the grade and place_pose all hold C1',
              got['seat'] and got['grade'] and got['place_pose'], str(got))
        print('B. the body 0.5 mm from a hole site, a keep-out of the ring\'s radius on it')
        got = readers(td, 'B', 'C1', (SITE[0], SITE[1] - 3.0),
                      {'circle': [SITE[0], SITE[1], RING]})
        check('B: the seat, the grade, place_pose and the ring all hold C1',
              all(got.values()), str(got))
        print('C. control: a keep-out clear of the body')
        got = readers(td, 'C', 'C1', (20.0, 20.0), {'rect': [18.0, 23.0, 22.0, 24.0]})
        check('C: nobody holds C1', not any(got.values()), str(got))
        print('D. a drawn courtyard, its pad past it into a keep-out')
        got = readers(td, 'D', 'U1', (SITE[0] - 4.2, SITE[1] - 0.5),
                      {'circle': [SITE[0], SITE[1], RING]}, courtyard=True)
        check('D: the grade and place_pose both hold U1', got['grade'] and got['place_pose'],
              str(got))
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
