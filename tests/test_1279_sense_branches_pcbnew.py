#!/usr/bin/env python3
"""Both parse paths resolve `sense:<REF>.<PAD>` groups identically (#1279).

The GUI builds its PCBData from the live board (build_pcb_data_from_board,
`board.Groups()`, pcbnew uuids); the CLI parses the file (parse_kicad_pcb).
A branch one front protects and the other does not would route differently
on the two fronts. For three boards -- the fixture (one member track), the
fixture with a member via added, and the fixture with a load already tapping
the trace (refused) -- this loads the board with pcbnew in KiCad's bundled
python and asserts:

- the branch signatures (group, join, member uuids, refusal reason, sense
  pads, member track/via counts) are equal;
- `compare_pcb_data` reports no "Sense branches" difference;
- the member copper resolved on the pcbnew path (its tracks and vias carry
  the file's uuids).

Exits 77 (SKIP) only when KiCad's bundled python cannot be found.

Run:
    python3 tests/test_1279_sense_branches_pcbnew.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from run_utils import evidence  # noqa: E402
from kicad_exact_fill import find_kicad_python  # noqa: E402

RUN_ALL_TIMEOUT = 300
SKIP_EXIT = 77

BOARD = os.path.join(TESTS_DIR, 'fixtures', 'sense_branch', 'sense_star.kicad_pcb')
MEMBER = '11111111-0000-0000-0000-000000000001'
VIA = '66666666-0000-0000-0000-000000000001'

#: Runs in KiCad's python: one JSON line per board.
PROBE = r'''
import json, sys
sys.path.insert(0, sys.argv[1])
import pcbnew
from kicad_parser import parse_kicad_pcb, build_pcb_data_from_board, compare_pcb_data
from sense_branch import branch_signature
for path in sys.argv[2:]:
    pb = build_pcb_data_from_board(pcbnew.LoadBoard(path))
    pf = parse_kicad_pcb(path)
    print('RESULT ' + json.dumps({
        'board': path,
        'pcbnew': sorted(map(list, (branch_signature(b) for b in pb.sense_branches))),
        'file': sorted(map(list, (branch_signature(b) for b in pf.sense_branches))),
        'diff': [d for d in compare_pcb_data(pb, pf) if d.startswith('Sense branches')],
    }))
'''


def _variants(d):
    with open(BOARD, encoding='utf-8') as f:
        src = f.read()
    anchor = '  (segment (start 3 5)'
    assert src.count(anchor) == 1 and src.count(f'(members "{MEMBER}")') == 1, 'fixture moved'
    via = (f'  (via (at 20 5) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 1) '
           f'(uuid "{VIA}"))\n')
    tap = ('  (segment (start 15 5) (end 15 8) (width 0.3) (layer "F.Cu") (net 1) '
           '(uuid "44444444-0000-0000-0000-000000000001"))\n')
    texts = {
        'plain': src,
        'via': src.replace(anchor, via + anchor).replace(
            f'(members "{MEMBER}")', f'(members "{MEMBER}" "{VIA}")'),
        'tapped': src.replace(anchor, tap + anchor),
    }
    paths = {}
    for name, text in texts.items():
        p = os.path.join(d, f'{name}.kicad_pcb')
        with open(p, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
        paths[name] = p
    return paths


def main():
    evidence(BOARD, 'the sense_star fixture')
    kpy = find_kicad_python()
    if kpy is None:
        print("SKIP: KiCad's bundled python not found")
        return SKIP_EXIT
    d = tempfile.mkdtemp(prefix='t1279p_')
    try:
        paths = _variants(d)
        r = subprocess.run([kpy, '-c', PROBE, os.path.join(ROOT, 'py_router')]
                           + list(paths.values()), capture_output=True, text=True,
                           timeout=240)
        rows = {}
        for line in r.stdout.splitlines():
            if line.startswith('RESULT '):
                row = json.loads(line[len('RESULT '):])
                rows[os.path.basename(row['board'])[:-len('.kicad_pcb')]] = row
        if r.returncode != 0 or set(rows) != set(paths):
            print(f'BROKEN TEST: the pcbnew probe ran {sorted(rows)} of {sorted(paths)} '
                  f'(exit {r.returncode}):\n{r.stdout[-2000:]}\n{r.stderr[-3000:]}')
            return 2
        fails = []

        def ck(name, cond, detail=''):
            print(f"  {'PASS' if cond else 'FAIL'}  {name}" + ('' if cond else f'  [{detail}]'))
            if not cond:
                fails.append(name)

        for name, row in sorted(rows.items()):
            ck(f'{name}: both paths resolve the same branches',
               row['pcbnew'] == row['file'] and not row['diff'], json.dumps(row))
            ck(f'{name}: one branch declared', len(row['file']) == 1, json.dumps(row['file']))
        plain, via, tapped = (rows[k]['pcbnew'][0] for k in ('plain', 'via', 'tapped'))
        # signature: (name, join, uuids, reason, sense pads, #tracks, #vias)
        ck('plain: protected, its track resolved on the pcbnew path',
           plain[3] == '' and plain[5:] == [1, 0] and plain[4] == ['U1.1'], plain)
        ck('via: protected, its via resolved on the pcbnew path',
           via[3] == '' and via[5:] == [1, 1], via)
        ck('tapped: refused on the pcbnew path, with the reason',
           tapped[3].startswith('input copper already taps it'), tapped)
        print(f'{len(rows) * 2 + 3 - len(fails)}/{len(rows) * 2 + 3} checks passed')
        return 1 if fails else 0
    finally:
        shutil.rmtree(d, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
