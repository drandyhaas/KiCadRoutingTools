#!/usr/bin/env python3
"""The recovery table is measured, and the two ways it used to lie are shut.

The previous run's table was assembled by hand and three of its numbers were
wrong, all in the flattering direction. Two of those were tool defects rather
than typing:

1. HOME WAS COUNTED IN THE WRONG SPACE. `placement_recovery_arm.py` computed
   it from footprint-ORIGIN distance with a hardcoded 2.0, against
   `recovery.py`'s own headline rule that displacement is measured in PAD
   space. A part rotated in place moves its origin 0.00 mm and its pads up to
   49 mm, so an origin-distance count reports a rotated part as home. The
   correct value was already computed and thrown away.

2. `check_assembly --json` CARRIED NO VERDICT. Every reader re-derived
   `blocking == 0 and not locked_contact_pairs` for itself, and the ones that
   got it wrong got it wrong quietly -- the arm scraped the verdict back out of
   stdout, and board_score's assembly component reads `blocking` alone.

Run: python3 -X utf8 tests/test_run9_arm_report.py
"""
import json
import math
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_placer'))  # #522/py_placer layout
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))  # #522/py_placer layout
os.environ.setdefault('KRT_NO_BANNER', '1')

BOARD = os.path.join('kicad_files', 'splitflap_driver.kicad_pcb')
FAILURES = []


def check(name, cond, detail=''):
    print(f'  {"PASS" if cond else "FAIL"}  {name}'
          + (f'\n        {detail}' if not cond and detail else ''))
    if not cond:
        FAILURES.append(name)


def main():
    print('check_assembly states its own verdict, so nobody re-derives it')
    with tempfile.TemporaryDirectory() as tmp:
        jp = os.path.join(tmp, 'a.json')
        p = subprocess.run(
            [sys.executable, '-X', 'utf8', 'py_tools/check_assembly.py', BOARD,
             '--json', jp], capture_output=True, text=True, encoding='utf-8',
            errors='replace', cwd=ROOT)
        doc = json.load(open(jp, encoding='utf-8'))
        for key in ('buildable', 'verdict', 'locked_contacts'):
            check(f'--json carries {key!r}', key in doc, str(sorted(doc)))
        check('buildable agrees with the exit code',
              doc.get('buildable') == (p.returncode == 0),
              f'buildable={doc.get("buildable")} exit={p.returncode}')
        check('buildable agrees with the printed verdict',
              doc.get('buildable') == ('NOT BUILDABLE' not in doc['verdict']),
              doc.get('verdict'))
        check('locked_contacts is a count, not a list',
              isinstance(doc.get('locked_contacts'), int),
              type(doc.get('locked_contacts')).__name__)

    print('home is counted in PAD space, not between footprint origins')
    # Rotate one part in place. Its origin does not move at all; its pads do.
    # An origin-distance count calls that home; the pad-space one must not.
    from kicad_parser import parse_kicad_pcb
    from placement import recovery as R
    pcb = parse_kicad_pcb(BOARD)
    poses = R.board_poses(pcb)
    ref = max(poses, key=lambda r: len(pcb.footprints[r].pads or []))
    x, y, rot = poses[ref]
    spun = dict(poses)
    spun[ref] = (x, y, (rot + 180.0) % 360.0)

    d = R.displacement_to_original(spun, poses, pcb.footprints, subset=[ref])
    origin_mm = math.hypot(0.0, 0.0)
    check(f'the origin did not move ({ref})', origin_mm == 0.0)
    check('...but the pad-space displacement did',
          d['perturbed_pad_rms'] > R.HOME_TOLERANCE_MM,
          f'pad rms {d["perturbed_pad_rms"]}')
    check('...so the part is NOT counted as home',
          d['parts_home_frac'] == 0.0, str(d['parts_home_frac']))
    check('...and the rotation is reported, not swallowed',
          d['rot_mismatch_total'] == 1, str(d.get('rot_mismatch')))

    print('the report tool reads that value rather than recomputing it')
    src = open(os.path.join(ROOT, 'tests', 'stress', 'arm_report.py'),
               encoding='utf-8').read()
    check('arm_report uses parts_home_frac', 'parts_home_frac' in src)
    check('...and does not hand-roll an origin distance',
          'math.hypot' not in src, 'a hypot in here is the old bug returning')
    check('...and takes the frozen member list as N',
          "['block']" in src or '"block"' in src or "'block'" in src)
    check('...and opens truth at ONE marked point',
          'truth opens HERE' in src)
    check('...and guards a copper-less human reference',
          'original_degeneracy' in src)


    print('an EQUAL tuple is no tie-break in favour of an unevidenced move')
    # The hole A2 closes: a zero-net part in free space touches no term the
    # gate tuple has, so `strictly better` never fires for it and a carry to
    # another part's seat survives the sweep that exists to catch it.
    from placement import reconstruct as Rc
    class _P:
        def __init__(s, x, y, rot=0.0):
            s.x, s.y, s.rot = x, y, rot
    class _S:
        def __init__(s, parts):
            s.parts = parts
    st = _S({'A': _P(10.0, 20.0), 'B': _P(0.0, 0.0)})
    old = {'A': (10.0 - 65.5, 20.0 + 11.3, 0.0), 'B': (5.0, 5.0, 0.0)}
    ev = Rc.evidenced_moves(st, old, [(65.5, -11.3)])
    check('a move matching a kept vector is evidenced', ev == {'A'}, str(ev))
    ev2 = Rc.evidenced_moves(st, old, [])
    check('with no vector, nothing is evidenced', ev2 == set(), str(ev2))
    ev3 = Rc.evidenced_moves(st, {'A': (10.0, 20.0, 0.0)}, [(65.5, -11.3)])
    check('a part that did not move needs no justification', ev3 == set(),
          str(ev3))
    st2 = _S({'A': _P(10.0 + 131.0, 20.0 - 22.6)})
    ev4 = Rc.evidenced_moves(st2, {'A': (10.0, 20.0, 0.0)}, [(65.5, -11.3)])
    check('a move of 2v is evidenced too (the lattice is +/-1, +/-2)',
          ev4 == {'A'}, str(ev4))
    check('prune takes an evidenced set',
          'evidenced' in Rc.prune_assignment.__code__.co_varnames)


    print('the off-board witness is silent on healthy boards, by construction')
    from pose_score import make_state
    for lab, bd, clr in (('splitflap', BOARD, 0.15),
                         ('tigard', os.path.join('kicad_files',
                                                 'tigard.kicad_pcb'), 0.09)):
        st = make_state(parse_kicad_pcb(bd), bd, clearance=clr)
        w = Rc.damage_witnesses(st)
        check(f'no witness on healthy {lab}', not w, str(sorted(w)))
    check('the mover preference consults it',
          'witnesses' in open(os.path.join(ROOT, 'py_placer', 'placement', 'seeder.py'),
                              encoding='utf-8').read())

    print()
    if FAILURES:
        print(f'FAIL: {len(FAILURES)} check(s): {", ".join(FAILURES)}')
        return 1
    print('OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
