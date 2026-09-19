#!/usr/bin/env python3
"""#962: place_pose grades footprint GRAPHIC copper, and says what `legal` covers.

`place_pose set U2 115.34 93.6 --rot 90` on esp_prog used to exit 0 with
`legal: true` while U2's F.Cu tab sat 1.11 mm past the outline.
`oob_graphic_copper_count` is now a LEGALITY_KEY and `_amount` a MAGNITUDE_KEY.

Invariants (the follow-up comment's placement acceptance, run through the real CLI):
1. Without --force, 115.34 is REFUSED: exit 4, `output` null, the named
   finding, and the source board unchanged.
2. The controls are accepted, `legal` true: the original pose, and 116.70
   (0.25 mm inside).
3. `--force` at 115.34 writes the board and reports `legal` false.
4. Deepening an existing overrun (0.3 -> 1.11 mm, count 1 -> 1) is refused
   on the AMOUNT arm.
5. `legal_scope` / `legal_unmeasured` are published, and `legal_basis` still
   names `no_worse` (test_892) and graphic copper.
6. `--snap` from 115.34 never writes a pose whose graphic copper is off the
   outline.

Run:
    python3 tests/test_962_place_pose_graphic.py
"""
import hashlib
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

from copy_board import copy_board  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from check_drc import footprint_graphic_outline_census  # noqa: E402
from placement.writer import write_placed_output  # noqa: E402
from run_utils import check as run_check  # noqa: E402

FAILS = []
ESP = os.path.join(ROOT, 'kicad_files', 'esp_prog.kicad_pcb')
POSE = os.path.join(ROOT, 'py_placer', 'place_pose.py')


def check(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}"
          + (f"   [{detail}]" if detail and not cond else ''))
    if not cond:
        FAILS.append(name)


def digest(p):
    with open(p, 'rb') as fh:
        return hashlib.sha256(fh.read()).hexdigest()


def summary(r):
    m = re.search(r'JSON_SUMMARY: (\{.*\})', r.stdout or '')
    return json.loads(m.group(1)) if m else {}


def pose(board, out, x, *extra, **kw):
    argv = [sys.executable, '-X', 'utf8', POSE, board, out, 'set', 'U2', str(x), '93.6',
            '--rot', '90', *extra]
    return summary(run_check(argv, **kw))


def u2_overrun(board):
    rows = [r['overrun_mm'] for r in footprint_graphic_outline_census(
        parse_kicad_pcb(board))['rows'] if r['owner_ref'] == 'U2']
    return max(rows) if rows else None


def main():
    work = tempfile.mkdtemp(prefix='krt962p_')
    try:
        src = os.path.join(work, 'esp.kicad_pcb')
        copy_board(ESP, src)
        h0 = digest(src)

        # 1
        s = pose(src, os.path.join(work, 'bad.kicad_pcb'), 115.34, code=4,
                 refuse='oob_graphic_copper_count 0 -> 1')
        check('1. 115.34 refused: exit 4, output null, legal false',
              s.get('output') is None and s.get('legal') is False and s.get('no_worse') is False,
              str({k: s.get(k) for k in ('output', 'legal', 'no_worse')}))
        check('1. ... the finding names the graphic copper amount 0.0 -> 1.11',
              'oob_graphic_copper_amount 0.0 -> 1.11' in (s.get('refused') or ''),
              s.get('refused'))
        check('1. ... and the source board is unchanged', digest(src) == h0)
        check('1. ... nothing was written', not os.path.exists(os.path.join(work, 'bad.kicad_pcb')))

        # 2
        fp = parse_kicad_pcb(src).footprints['U2']
        s = summary(run_check([sys.executable, '-X', 'utf8', POSE, src,
                               os.path.join(work, 'same.kicad_pcb'), 'set', 'U2',
                               str(fp.x), str(fp.y), '--rot', str(fp.rotation)], accept=True))
        check('2. the original pose is legal', s.get('legal') is True, str(s.get('refused')))
        s = pose(src, os.path.join(work, 'ok.kicad_pcb'), 116.70, accept=True)
        check('2. 116.70 (0.25 mm inside) is legal', s.get('legal') is True
              and s.get('oob_graphic_copper_count_after') == 0, str(s.get('refused')))

        # 3
        s = pose(src, os.path.join(work, 'forced.kicad_pcb'), 115.34, '--force', accept=True)
        check('3. --force writes the board and reports legal false, forced true',
              s.get('forced') is True and s.get('legal') is False
              and os.path.exists(os.path.join(work, 'forced.kicad_pcb')))

        # 4 -- deepen an existing overrun: count stays 1, the amount grows
        shallow = os.path.join(work, 'shallow.kicad_pcb')
        copy_board(ESP, shallow)
        write_placed_output(shallow, shallow, [dict(reference='U2', new_x=116.15,
                                                    new_y=93.6, new_rotation=90)])
        ov = u2_overrun(shallow)
        check('4. staged input: U2 already 0.3 mm off', ov is not None and abs(ov - 0.30) <= 0.01,
              str(ov))
        s = pose(shallow, os.path.join(work, 'deeper.kicad_pcb'), 115.34, code=4,
                 refuse='oob_graphic_copper_amount')
        check('4. 0.3 -> 1.11 mm is refused on the AMOUNT, the count unchanged',
              s.get('oob_graphic_copper_count_before') == 1
              and s.get('oob_graphic_copper_count_after') == 1
              and 'oob_graphic_copper_count' not in (s.get('refused') or ''),
              str(s.get('refused')))

        # 5
        s = pose(src, os.path.join(work, 'ok2.kicad_pcb'), 116.70, accept=True)
        check('5. legal_scope names graphic copper; legal_unmeasured names paste/mask',
              any('graphic' in x for x in s.get('legal_scope') or [])
              and any('paste' in x for x in s.get('legal_unmeasured') or []))
        check('5. legal_basis still says no_worse (test_892) and names graphic copper',
              'no_worse' in (s.get('legal_basis') or '')
              and 'graphic' in (s.get('legal_basis') or ''))

        # 6 -- --snap never writes an overrun
        out = os.path.join(work, 'snap.kicad_pcb')
        r = run_check([sys.executable, '-X', 'utf8', POSE, src, out, 'set', 'U2', '115.34',
                       '93.6', '--rot', '90', '--snap'], accept=True, allow=())
        s = summary(r)
        if os.path.exists(out):
            ov = u2_overrun(out)
            check('6. --snap from 115.34 writes a pose with NO graphic overrun',
                  ov is not None and ov <= 1e-6 and s.get('legal') is True, f'{ov} {s.get("snapped")}')
        else:
            check('6. --snap from 115.34 refused rather than writing an overrun',
                  s.get('output') is None)

        # 7 -- a swap cannot launder an overrun onto a clean part by keeping
        # the totals level
        from placement.pose_ops import worsened
        b = {'oob_graphic_copper_count': 1, 'oob_graphic_copper_amount': 1.0,
             'oob_graphic_copper_refs': [['A', 1.0]]}
        a = {'oob_graphic_copper_count': 1, 'oob_graphic_copper_amount': 1.0,
             'oob_graphic_copper_refs': [['B', 1.0]]}
        check('7. overrun moved from A to B at level totals: worsened names the refs',
              worsened(b, a) == ['oob_graphic_copper_refs'], str(worsened(b, a)))
        check('7. control: the same part, the same overrun: not worsened',
              worsened(b, dict(b)) == [], str(worsened(b, dict(b))))
    finally:
        shutil.rmtree(work, ignore_errors=True)
    print(f"\n{'ALL PASS' if not FAILS else f'{len(FAILS)} FAILED'}")
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
