"""check_drc grades two vias' copper only where they share a layer.

`check_via_via_overlap` assumed every via is a through via, so on an HDI
board a F.Cu-In1.Cu micro via over an In6.Cu-B.Cu micro via of another net
(laser holes from the two faces, no copper on a common layer) was reported
as a via-via short. kicad-cli reports nothing between them.

The board is tests/fixtures/1842/hdi_repro (8 copper layers) with vias of
nets S01 and S02 added clear of every part:

  1. a top micro via and a bottom micro via at one point: no via-via
     violation (no shared layer);
  2. a top micro via 0.05 mm (copper to copper) from a buried
     In1.Cu-In6.Cu via, so they share In1.Cu: still a via-via violation
     (non-vacuity: the check still runs);
  3. a through via as close to a bottom micro via: still a violation;
  4. kicad-cli (when installed) agrees: a clearance or short between the
     pair of row 2, none for the pair of row 1.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'py_router'))

import run_utils  # noqa: E402
from kicad_oracle import find_kicad_cli  # noqa: E402

FIX = os.path.join(REPO, 'tests', 'fixtures', '1842', 'hdi_repro.kicad_pcb')
FAILS = []

MICRO_TOP = ('micro', 0.25, 0.1, 'F.Cu', 'In1.Cu')
MICRO_BOT = ('micro', 0.25, 0.1, 'In6.Cu', 'B.Cu')
BURIED = ('buried', 0.4572, 0.2032, 'In1.Cu', 'In6.Cu')
THROUGH = (None, 0.4572, 0.2032, 'F.Cu', 'B.Cu')

# (x, y, kind, net): row 1 at (2, 5), row 2 at (2, 10), row 3 at (2, 15).
# Rows 2 and 3 sit 0.4 mm apart, 0.05 mm copper to copper, under the 0.1 mm
# clearance but not touching (KiCad gives touching copper one net on load).
VIAS = [
    (2.0, 5.0, MICRO_TOP, 'S01'), (2.0, 5.0, MICRO_BOT, 'S02'),
    (2.0, 10.0, MICRO_TOP, 'S01'), (2.4, 10.0, BURIED, 'S02'),
    (2.0, 15.0, THROUGH, 'S01'), (2.4, 15.0, MICRO_BOT, 'S02'),
]


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def via_sexpr(x, y, kind, net):
    typ, size, drill, a, b = kind
    word = f'via {typ}' if typ else 'via'
    return (f'\t({word} (at {x} {y}) (size {size}) (drill {drill}) '
            f'(layers "{a}" "{b}") (net "{net}") (uuid "{uuid.uuid4()}"))\n')


def stage(td):
    dst = os.path.join(td, 'b.kicad_pcb')
    subprocess.run([sys.executable, os.path.join(REPO, 'py_router', 'copy_board.py'),
                    FIX, dst], capture_output=True, cwd=REPO)
    with open(dst, encoding='utf-8') as fh:
        text = fh.read()
    assert re.search(r'\(net "S01"\)', text) and re.search(r'\(net "S02"\)', text), \
        "BROKEN TEST: the fixture has nets S01 and S02"
    cut = text.rstrip().rfind(')')
    text = text[:cut] + ''.join(via_sexpr(*v) for v in VIAS) + ')\n'
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return run_utils.evidence(dst)


def via_via_rows(board, td):
    out = os.path.join(td, 'drc.json')
    r = subprocess.run([sys.executable, os.path.join(REPO, 'py_router', 'check_drc.py'),
                        board, '--json', out], capture_output=True, text=True, cwd=REPO)
    assert os.path.isfile(out), f"BROKEN TEST: check_drc wrote no report\n{r.stderr[-800:]}"
    with open(out, encoding='utf-8') as fh:
        doc = json.load(fh)
    rows = [v for v in doc.get('items', []) if v.get('type') == 'via-via']
    return rows


def at(rows, y):
    """via-via rows whose two vias sit near the line y."""
    hits = []
    for r in rows:
        pts = [p for k in ('loc1', 'loc2') for p in [r.get(k)] if p]
        if pts and all(abs(p[1] - y) < 0.5 for p in pts):
            hits.append(r)
    return hits


def kicad(board, td):
    cli = find_kicad_cli(warn=False)
    if not cli:
        print("  (kicad-cli not installed: not run)")
        return
    out = os.path.join(td, 'kc.json')
    subprocess.run([cli, 'pcb', 'drc', '--format', 'json', '--severity-all',
                    '-o', out, board], capture_output=True, text=True)
    assert os.path.isfile(out), "BROKEN TEST: kicad-cli wrote no report"
    with open(out, encoding='utf-8') as fh:
        doc = json.load(fh)

    def near(y):
        found = []
        for v in doc.get('violations', []):
            items = v.get('items', [])
            if (len(items) == 2 and v.get('type') in ('clearance', 'shorting_items')
                    and all(abs(i['pos']['y'] - y) < 0.5 and abs(i['pos']['x'] - 2.2) < 0.5
                            for i in items)):
                found.append(v['type'])
        return found
    ok(near(10.0), f"kicad-cli reports the pair that shares In1.Cu: {near(10.0)}")
    ok(not near(5.0), f"and nothing for the pair that shares no layer: {near(5.0)}")


def main():
    td = tempfile.mkdtemp(prefix='krt_vvl_')
    try:
        board = stage(td)
        rows = via_via_rows(board, td)
        print('1. no shared layer')
        ok(not at(rows, 5.0), f"a top and a bottom micro via at one point: {len(at(rows, 5.0))} rows")
        print('2-3. a shared layer')
        ok(len(at(rows, 10.0)) == 1, f"a top micro via beside a buried via: {len(at(rows, 10.0))} row")
        ok(len(at(rows, 15.0)) == 1, f"a through via beside a bottom micro via: {len(at(rows, 15.0))} row")
        print('4. kicad-cli')
        kicad(board, td)
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
