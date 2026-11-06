#!/usr/bin/env python3
"""#1715: a pile keeps a "PCB Edge" marker text's angle to its part, so the
marker reader still finds the line the library drew where the board edge goes.

`make_unplaced` writes every text of a moved part at absolute 0 (#1163), and
the marker reader (#1245) picks the line by the marker text's angle to the
part, admitting only a line parallel to it. A USB receptacle at rotation 180
whose marker runs along a vertical line lost it on the pile: the text read
horizontal, two silk lines across it named different faces, the reader
abstained, the body's face went weak and a correct declared `mating_face`
read `unverified`. Every seed composes its rotation onto the pile's text, so
every seed read the same. On a tracked board (no KiCad needed):

  1. NEGATIVE CONTROL: the marker text written at the pile's 0 (the old
     write) is read against the wrong lines, and the face is weak;
  2. on make_unplaced's pile the marker text keeps its angle to the part,
     mod 180, and the reader names the source's face on the source's line,
     strong;
  3. put back at its own pose, and at another quarter turn, by place_pose
     (which composes each rotation onto the pile's text), the same;
  4. `writer.marker_pile_angle` folds an angle into the half-turn above the
     part's rotation.

    python3 tests/test_1715_pile_marker_angle.py
"""
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, '.claude', 'skills', 'plan-pcb-placement', 'scripts')
for _d in (os.path.join(ROOT, 'py_router'), os.path.join(ROOT, 'py_placer'),
           os.path.join(ROOT, 'tests')):
    sys.path.insert(0, _d)

import run_utils                                              # noqa: E402
from kicad_parser import parse_kicad_pcb, iter_footprint_blocks  # noqa: E402
from placement.body import board_bodies                       # noqa: E402
from placement.mating import marker_face_local, mating_face_local  # noqa: E402
from placement.writer import marker_pile_angle                # noqa: E402
from test_1603_body_in_board import quiet                     # noqa: E402

SRC = run_utils.evidence(os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb'))
REF = 'USB1'
#: its pose on the source board
POSE = (117.5, 100.0, 180.0)
PY = [sys.executable, '-X', 'utf8', '-B']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}" + (f" ({detail})" if detail and not ok else ''))
    if not ok:
        failures.append(label)


def read(path):
    """`(rotation, marker text angle, the marker's reading, the body's face)`."""
    pcb = quiet(parse_kicad_pcb, path)
    geom = board_bodies(pcb, path)[REF]
    fp = pcb.footprints[REF]
    (_tx, _ty, angle, _layer, _text), = geom.edge_marker.texts
    return fp.rotation, angle, marker_face_local(fp, geom), mating_face_local(fp, geom)


def main():
    with tempfile.TemporaryDirectory() as td:
        rot, angle, mk, lf = read(SRC)
        check('fixture: the source reads its marker strong, at its own pose',
              (rot, angle) == (POSE[2], 90.0) and mk.face == '+x' and lf.strong
              and 'marker' in lf.basis, (rot, angle, mk, lf))
        src_line, src_face = mk.line, mk.face

        pile = os.path.join(td, 'pile.kicad_pcb')
        r = run_utils.check(PY + [os.path.join(SCRIPTS, 'make_unplaced.py'), SRC, pile],
                            accept=True)
        run_utils.evidence(pile, 'the pile')
        check('make_unplaced reports no text leak', 'FAIL' not in r.stdout + r.stderr,
              (r.stdout + r.stderr)[-400:])

        # 1. the marker at the pile's text angle, as the old write left it
        with open(pile, encoding='utf-8', newline='') as fh:
            text = fh.read()
        block = next(b for _s, _e, b, _r, k in iter_footprint_blocks(text) if k == REF)
        old = re.sub(r'(\(fp_text user "pcb edge"[^\n]*\n\s*\(at \S+ \S+) \S+\)', r'\1 0)',
                     block, count=1)
        if old == block:
            raise AssertionError('BROKEN FIXTURE: no PCB-edge marker text in the pile block')
        zero = os.path.join(td, 'zero.kicad_pcb')
        with open(zero, 'w', encoding='utf-8', newline='') as fh:
            fh.write(text.replace(block, old))
        back0 = os.path.join(td, 'back0.kicad_pcb')
        run_utils.check(PY + [os.path.join(ROOT, 'py_placer', 'place_pose.py'), zero, back0,
                              'set', REF, str(POSE[0]), str(POSE[1]), '--rot', str(POSE[2])],
                        accept=True)
        _r, angle, mk, lf = read(run_utils.evidence(back0, 'the old pile, put back'))
        check('control: written at 0 and put back, the marker labels another line and '
              'the face is weak', mk.line != src_line and mk.face != src_face
              and not lf.strong, (angle, mk, lf))

        # 2. the pile itself
        rot, angle, mk, lf = read(pile)
        check("the pile keeps the marker's angle to the part, mod 180",
              (angle - rot) % 180 == (90.0 - POSE[2]) % 180, (rot, angle))
        check("...and reads the source's face on the source's line, strong",
              mk.line == src_line and mk.face == src_face and lf.strong, (mk, lf))

        # 3. put back by place_pose, at its own pose and turned
        for turn in (POSE[2], 90.0):
            out = os.path.join(td, f'back{turn:g}.kicad_pcb')
            run_utils.check(PY + [os.path.join(ROOT, 'py_placer', 'place_pose.py'), pile, out,
                                  'set', REF, str(POSE[0]), str(POSE[1]), '--rot', str(turn)],
                            accept=True)
            rot, angle, mk, lf = read(run_utils.evidence(out, f'the pile at turn {turn:g}'))
            check(f"placed at turn {turn:g}: the source's face on the source's line, strong",
                  rot == turn and mk.line == src_line and mk.face == src_face and lf.strong,
                  (rot, angle, mk, lf))

        # 4. the fold
        for a, part, want in ((90.0, 0.0, 90.0), (270.0, 0.0, 90.0), (0.0, 180.0, 180.0),
                              (90.0, 180.0, 270.0), (None, 90.0, 180.0), (45.0, 0.0, 45.0)):
            got = marker_pile_angle(a, part)
            check(f'marker_pile_angle({a}, {part:g}) = {want:g}', got == want, str(got))

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
