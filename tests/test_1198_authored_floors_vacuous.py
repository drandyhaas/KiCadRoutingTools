#!/usr/bin/env python3
"""#1198: check_complete --authored-from can say DONE on a board that honours
its declared floors, including vacuously.

Two inputs always landed in `unmeasured`, and any `unmeasured` entry made
the verdict INCOMPLETE:
  * `min_hole_clearance` is declaration-only (scan_board_minima measures no
    pairwise geometry), so every board whose authored project carried it --
    13 of the 14 tracked projects -- read INCOMPLETE, a declared 0.0 included,
    and a board compared with ITSELF too;
  * via floors on a board with no vias, since scan_board_minima emits via
    keys only when vias exist.

Then edgehero's follow-up: a copper-to-hole floor the project LOWERED was
still unmeasured (INCOMPLETE on every such board), and a floor the human
reference breaks too read as a blocker. Copper-to-hole is now graded by KiCad
at the authored value (check_drc's hole arms are NPTH-only), and the original
board is the REFERENCE: a floor its own copper breaks at least as badly is
`reference_too`, reported and not counted against the board.

Checks (fab_floor_integrity, on staged copies of tracked boards):
  1. routed_output compared with itself: nothing unmeasured; copper-to-hole
     is `declared_kept`.
  2. A 0-via board (sonde_u) whose authored project declares via floors and
     a copper-to-hole 0.0: every one is `vacuous`, none unmeasured.
  3. A LOWERED copper-to-hole, with the grade stubbed: unmeasurable stays
     unmeasured (both values and the reason named); 0 violations is
     `measured_kept`; violations the reference matches or exceeds are
     `reference_too`; worse than the reference, or a clean reference, is
     `relaxed`.
  4. A measured floor: relaxed when the reference holds it, `reference_too`
     when the reference is the same copper.
  5. The same with REAL kicad-cli, when it is installed: routed_output at an
     authored 0.4 against itself is `reference_too`, against its own
     unrouted copy `relaxed`.
  6. check_complete's verdict: no fab-floor reason on the self-comparison,
     and a `reference_too` floor named without blocking.

    python3 tests/test_1198_authored_floors_vacuous.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from check_complete import fab_floor_integrity                 # noqa: E402
from copy_board import copy_board                              # noqa: E402

ROUTED = os.path.join(ROOT, 'kicad_files', 'routed_output.kicad_pcb')
NOVIA = os.path.join(ROOT, 'kicad_files', 'sonde_u.kicad_pcb')
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def stage(src, tmp, name, rules=None):
    dst = os.path.join(tmp, name)
    with contextlib.redirect_stdout(io.StringIO()):
        copy_board(src, dst)
    if rules is not None:
        pro = dst[:-len('.kicad_pcb')] + '.kicad_pro'
        doc = {}
        if os.path.isfile(pro):
            with open(pro) as f:
                doc = json.load(f)
        doc.setdefault('board', {}).setdefault('design_settings', {})['rules'] = rules
        with open(pro, 'w') as f:
            json.dump(doc, f)
    return dst


def strip_routing(src, dst, rules):
    """`src` staged at `dst` with every track and via removed (a paren-
    balanced cut of each `(segment`/`(arc`/`(via` block), its project
    declaring `rules`."""
    tmp = os.path.dirname(dst)
    stage(src, tmp, os.path.basename(dst), rules)
    with open(dst, encoding='utf-8') as f:
        text = f.read()
    out, i = [], 0
    while True:
        j = min((k for k in (text.find('(segment', i), text.find('(arc', i),
                             text.find('(via', i)) if k >= 0), default=-1)
        if j < 0:
            out.append(text[i:])
            break
        out.append(text[i:j])
        depth, k = 0, j
        while True:
            depth += {'(': 1, ')': -1}.get(text[k], 0)
            k += 1
            if depth == 0:
                break
        i = k
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(''.join(out))
    return dst


def keys(rows):
    return sorted(r['key'] for r in rows or ())


def main():
    tmp = tempfile.mkdtemp(prefix='t1198_')
    try:
        # 1. A board compared with itself.
        r = fab_floor_integrity(ROUTED, ROUTED)
        check('self-comparison: nothing unmeasured, nothing relaxed',
              r['ran'] and not r['unmeasured'] and not r['relaxed'], json.dumps(r)[:300])
        check('self-comparison: copper-to-hole is declared_kept',
              keys(r['declared_kept']) == ['min_hole_clearance'])

        # 2. A 0-via board under declared via floors and a 0.0 copper-to-hole.
        authored = {'min_via_diameter': 0.4, 'min_via_annular_width': 0.05,
                    'min_via_drill': 0.2, 'min_hole_clearance': 0.0}
        a = stage(NOVIA, tmp, 'authored.kicad_pcb', authored)
        b = stage(NOVIA, tmp, 'final.kicad_pcb', dict(authored))
        r = fab_floor_integrity(b, a)
        check('0-via board: via floors and a declared 0 are vacuous',
              keys(r['vacuous']) == sorted(authored) and not r['unmeasured']
              and not r['relaxed'], json.dumps(r)[:400])
        why = {x['key']: x['why'] for x in r['vacuous']}
        check('...each saying why',
              why.get('min_via_diameter') == 'no via on the board'
              and 'declared 0' in why.get('min_hole_clearance', ''), str(why))

        # 3. Copper-to-hole lowered in the project, the grade stubbed.
        a3 = stage(NOVIA, tmp, 'a3.kicad_pcb', {'min_hole_clearance': 0.25})
        b3 = stage(NOVIA, tmp, 'b3.kicad_pcb', {'min_hole_clearance': 0.1})

        def grader(board_grade, ref_grade):
            return lambda b, v: ref_grade if b == a3 else board_grade
        r = fab_floor_integrity(b3, a3, hole_grader=grader(
            {'error': 'kicad-cli not found'}, None))
        um = r['unmeasured'][0] if r['unmeasured'] else {}
        check('unmeasurable: a lowered copper-to-hole stays unmeasured, '
              'both values and the reason named',
              um.get('key') == 'min_hole_clearance' and um.get('authored') == 0.25
              and um.get('declared') == 0.1 and 'kicad-cli' in um.get('why', ''),
              json.dumps(r)[:300])
        r = fab_floor_integrity(b3, a3, hole_grader=grader(
            {'count': 0, 'min_actual': None}, None))
        check('KiCad finds nothing at the authored value: measured_kept',
              keys(r['measured_kept']) == ['min_hole_clearance']
              and not r['unmeasured'] and not r['relaxed'], json.dumps(r)[:300])
        r = fab_floor_integrity(b3, a3, hole_grader=grader(
            {'count': 5, 'min_actual': 0.154}, {'count': 199, 'min_actual': 0.145}))
        rt = r['reference_too'][0] if r['reference_too'] else {}
        check('the reference breaks it worse (0.145 vs 0.154): reference_too',
              rt.get('key') == 'min_hole_clearance' and rt.get('on_board') == 0.154
              and rt.get('reference') == 0.145 and not r['relaxed'],
              json.dumps(r)[:400])
        r = fab_floor_integrity(b3, a3, hole_grader=grader(
            {'count': 5, 'min_actual': 0.10}, {'count': 199, 'min_actual': 0.145}))
        check('worse than the reference (0.10 vs 0.145): relaxed',
              keys(r['relaxed']) == ['min_hole_clearance'] and not r['reference_too'],
              json.dumps(r)[:300])
        r = fab_floor_integrity(b3, a3, hole_grader=grader(
            {'count': 5, 'min_actual': 0.154}, {'count': 0, 'min_actual': None}))
        check('a reference that holds the floor: relaxed',
              keys(r['relaxed']) == ['min_hole_clearance'], json.dumps(r)[:300])

        # 4. A measured floor: the reference decides relaxed vs reference_too.
        bare = strip_routing(ROUTED, os.path.join(tmp, 'bare.kicad_pcb'),
                             {'min_track_width': 5.0})
        r = fab_floor_integrity(ROUTED, bare)
        check('copper below an authored track floor the reference holds is relaxed',
              keys(r['relaxed']) == ['min_track_width'], json.dumps(r['relaxed']))
        a4 = stage(ROUTED, tmp, 'a4.kicad_pcb', {'min_track_width': 5.0})
        r = fab_floor_integrity(ROUTED, a4)
        check('...and reference_too when the reference is the same copper',
              keys(r['reference_too']) == ['min_track_width'] and not r['relaxed'],
              json.dumps(r['reference_too']))

        # 5. Real KiCad, when it is here (the suite's cloud image has none).
        from kicad_oracle import find_kicad_cli
        if find_kicad_cli(warn=False):
            a5 = stage(ROUTED, tmp, 'a5.kicad_pcb', {'min_hole_clearance': 0.4})
            b5 = stage(ROUTED, tmp, 'b5.kicad_pcb', {'min_hole_clearance': 0.1})
            r = fab_floor_integrity(b5, a5)
            rt = r['reference_too'][0] if r['reference_too'] else {}
            check('kicad-cli: routed_output at 0.4 against itself is reference_too',
                  rt.get('violations', 0) > 0
                  and rt.get('violations') == rt.get('reference_violations')
                  and not r['relaxed'] and not r['unmeasured'], json.dumps(r)[:400])
            bare5 = strip_routing(ROUTED, os.path.join(tmp, 'bare5.kicad_pcb'),
                                  {'min_hole_clearance': 0.4})
            r = fab_floor_integrity(b5, bare5)
            rl = r['relaxed'][0] if r['relaxed'] else {}
            check('kicad-cli: ...and relaxed against its own unrouted copy',
                  rl.get('key') == 'min_hole_clearance' and rl.get('violations', 0) > 0,
                  json.dumps(r)[:400])
        else:
            print('  [skip] kicad-cli not installed: the real-KiCad grade (5)')

        # 5. The verdict on the self-comparison.
        js = os.path.join(tmp, 'cc.json')
        subprocess.run([sys.executable, '-X', 'utf8', 'check_complete.py', ROUTED,
                        '--authored-from', ROUTED, '--json', js], cwd=ROOT,
                       capture_output=True, text=True)
        doc = json.load(open(js)) if os.path.isfile(js) else {}
        why = doc.get('reason') or ''
        check('check_complete names no fab-floor reason on the self-comparison',
              bool(why) and 'fab floor' not in why, why[:300])
        js = os.path.join(tmp, 'cc4.json')
        subprocess.run([sys.executable, '-X', 'utf8', 'check_complete.py', ROUTED,
                        '--authored-from', a4, '--json', js], cwd=ROOT,
                       capture_output=True, text=True)
        doc = json.load(open(js)) if os.path.isfile(js) else {}
        why = doc.get('reason') or ''
        check('a reference_too floor is named in the verdict, not UNSOUND',
              doc.get('verdict') != 'UNSOUND' and 'BROKEN BY THE REFERENCE TOO' in why
              and 'track width 5.0' in why, f"{doc.get('verdict')}: {why[-300:]}")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
