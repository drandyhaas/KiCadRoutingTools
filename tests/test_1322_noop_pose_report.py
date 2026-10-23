#!/usr/bin/env python3
"""#1322: a pose op that changes nothing says so -- the writer counts only
changed poses, and place_pose names why an op left its part where it was.

On ecc83-pp_v2's placed board `place_pose rotate R2 180`, with R2 already at
180, wrote a byte-identical board, exited 0 and printed "Modified 1 footprint
positions" next to its own "rotate R2 unmoved". `rotate` sets an ABSOLUTE
angle, so the call a reader meant as a half turn did nothing, and the one
line that could have said so claimed a change.

The fixture is esp_prog: R1 is stored at -90 (KiCad's spelling of 270), R3 at
(141.0, 101.5) rot 180.

Checks:
  1. The writer: a placement at the block's own pose (270 against a stored
     -90 included) is "Modified 0 footprint positions (1 placement(s) already
     at that pose, unchanged)"; a real move counts 1 with no suffix; both in
     one call count 1 and 1.
  2. place_pose `rotate R1 270` (absolute, already there): the op is unmoved,
     `unmoved_why` says the angle is ABSOLUTE and names `--relative`, and the
     writer line says 0.
  3. place_pose `rotate R1 180 --relative`: moved, the writer line says 1.
  4. place_pose `set R3 141.0 101.5`: unmoved, 'already at this pose'.

    python3 tests/test_1322_noop_pose_report.py
"""
import contextlib
import io
import json
import os
import re
import shutil
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
for _p in ('py_router', 'py_placer', 'tests'):
    sys.path.insert(0, os.path.join(ROOT, _p))

from copy_board import copy_board                                   # noqa: E402
from placement.writer import write_placed_output                    # noqa: E402
from run_utils import check as run_check, evidence                  # noqa: E402

ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
POSE = os.path.join(ROOT, 'py_placer', 'place_pose.py')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def write(src, dst, placements):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        write_placed_output(src, dst, placements)
    return [ln for ln in buf.getvalue().splitlines()
            if ln.startswith('Modified ') and 'footprint positions' in ln]


def pose(src, work, *ops):
    r = run_check([sys.executable, '-X', 'utf8', POSE, src,
                   os.path.join(work, 'pp.kicad_pcb'), '--dry-run',
                   '--allow-routed'] + list(ops), accept=True)
    m = re.search(r'^JSON_SUMMARY: (\{.*\})$', r.stdout or '', re.M)
    lines = [ln for ln in r.stdout.splitlines()
             if ln.startswith('Modified ') and 'footprint positions' in ln]
    return (json.loads(m.group(1)) if m else {}), lines, r.stdout


def main():
    work = tempfile.mkdtemp(prefix='krt1322_')
    try:
        esp = os.path.join(work, 'esp.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            copy_board(ESP, esp)
        evidence(esp, 'esp_prog copy')
        out = os.path.join(work, 'out.kicad_pcb')

        print("1. the writer")
        same = {'reference': 'R1', 'new_x': 136.4, 'new_y': 98.8,
                'new_rotation': 270.0}
        moved = {'reference': 'R3', 'new_x': 142.0, 'new_y': 101.5,
                 'new_rotation': 180.0}
        got = write(esp, out, [same])
        check('a placement at its own pose (270 vs a stored -90) counts 0',
              got == ['Modified 0 footprint positions (1 placement(s) '
                      'already at that pose, unchanged)'], str(got))
        got = write(esp, out, [moved])
        check('a real move counts 1, no suffix',
              got == ['Modified 1 footprint positions'], str(got))
        got = write(esp, out, [same, moved])
        check('both in one call: 1 modified, 1 unchanged',
              got == ['Modified 1 footprint positions (1 placement(s) '
                      'already at that pose, unchanged)'], str(got))

        print("2. place_pose rotate R1 270 (absolute, already there)")
        s, lines, text = pose(esp, work, 'rotate', 'R1', '270')
        op = (s.get('ops') or [{}])[0]
        check('the op is unmoved', op.get('moved') is False, str(op))
        why = op.get('unmoved_why') or ''
        check('...and says the angle is ABSOLUTE, naming --relative',
              'ABSOLUTE' in why and 'rotate R1 270 --relative' in why, why)
        check('...on the console too',
              'unmoved -- already at 270 degrees' in text)
        check('the writer line says 0', lines and lines[0].startswith(
            'Modified 0 footprint positions'), str(lines))

        print("3. place_pose rotate R1 180 --relative")
        s, lines, _t = pose(esp, work, 'rotate', 'R1', '180', '--relative')
        op = (s.get('ops') or [{}])[0]
        check('the op moved', op.get('moved') is True
              and 'unmoved_why' not in op, str(op))
        check('the writer line says 1',
              lines == ['Modified 1 footprint positions'], str(lines))

        print("4. place_pose set R3 at its own pose")
        s, lines, _t = pose(esp, work, 'set', 'R3', '141.0', '101.5')
        op = (s.get('ops') or [{}])[0]
        check("unmoved, 'already at this pose'",
              op.get('moved') is False
              and op.get('unmoved_why') == 'already at this pose', str(op))
    finally:
        shutil.rmtree(work, ignore_errors=True)

    print(f"\n{'FAILED' if failures else 'PASSED'}: {len(failures)} failure(s)")
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
