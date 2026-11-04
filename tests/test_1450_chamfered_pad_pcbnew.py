#!/usr/bin/env python3
"""The text parser's chamfered-rect outline is pcbnew's (#1450).

tests/test_1450_chamfered_pad_copper.py holds the text parser to the
chamfered outline on one pad; this holds it to KiCad's own copper over the
poses a board can give a chamfered pad. In KiCad's bundled python it builds
one board of copies of the one-pad board's octagon footprint, each posed and
edited through pcbnew -- footprint at 0/90/37/-90 degrees, on the front and
flipped both ways, the pad itself at 0/90/30 degrees relative to it, every
non-empty set of chamfered corners, with and without rounding on the others,
a 0.5 chamfer, a copper offset -- saves it, and for every pad compares:

  - the text parser's `polygons` with pcbnew's effective polygon, as the
    union each describes, both ways, within pcbnew's own 5 um arc error;
  - build_pcb_data_from_board against parse_kicad_pcb: compare_pcb_data
    finds no difference on any of these pads (shape, ratio, polygons).

A hand-written `rect` naming a chamfered corner, which pcbnew loads as its
chamfered rect rounded at 0.25, is compared the same way.

Exits 77 (SKIP) only when KiCad's bundled python cannot be found.

    python3 tests/test_1450_chamfered_pad_pcbnew.py
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
import test_1450_chamfered_pad_copper as one  # noqa: E402

RUN_ALL_TIMEOUT = 300
SKIP_EXIT = 77
#: pcbnew inscribes an arc within the board's max error (5 um by default);
#: the text parser within 1 um.
TOL = 0.006

#: Runs in KiCad's python: builds the posed board, then one RESULT line.
PROBE = r'''
import itertools, json, math, sys
sys.path.insert(0, sys.argv[1])
import pcbnew
from kicad_parser import (parse_kicad_pcb, build_pcb_data_from_board, compare_pcb_data,
                          _polygon_union_excess)

src, posed, rect_board = sys.argv[2], sys.argv[3], sys.argv[4]
MM = pcbnew.FromMM
TL, TR, BL, BR = 1, 2, 4, 8
board = pcbnew.LoadBoard(src)
base = [f for f in board.GetFootprints() if f.GetReference() == 'J1'][0]

def deg(a):
    return pcbnew.EDA_ANGLE(a, pcbnew.DEGREES_T)

variants = []
for bits in range(1, 16):
    variants.append(dict(bits=bits, fp_rot=0, flip=None, pad_rot=0, rr=0.25, ratio=0.2928932188,
                         size=(2.4, 1.6), offset=None))
for fp_rot, flip, pad_rot, bits, rr in itertools.product(
        (0, 90, 37, -90), (None, 'TB', 'LR'), (0, 90, 30), (TL, TL | BR, TR | BL | BR, 15),
        (0.0, 0.25)):
    variants.append(dict(bits=bits, fp_rot=fp_rot, flip=flip, pad_rot=pad_rot, rr=rr,
                         ratio=0.2928932188, size=(2.4, 1.6), offset=None))
for fp_rot, flip in itertools.product((0, 37), (None, 'TB')):
    variants.append(dict(bits=TL | TR, fp_rot=fp_rot, flip=flip, pad_rot=0, rr=0.25, ratio=0.5,
                         size=(2.4, 2.4), offset=None))
    variants.append(dict(bits=TL | BR, fp_rot=fp_rot, flip=flip, pad_rot=30, rr=0.25,
                         ratio=0.2928932188, size=(2.4, 1.6), offset=(0.3, 0.1)))

refs = {}
for n, v in enumerate(variants):
    try:
        fp = base.Duplicate(False)
    except TypeError:
        fp = base.Duplicate()
    fp = pcbnew.Cast_to_FOOTPRINT(fp)
    ref = f'V{n}'
    fp.SetReference(ref)
    board.Add(fp)
    pad = fp.Pads()[0]
    pad.SetSize(pcbnew.VECTOR2I(MM(v['size'][0]), MM(v['size'][1])))
    pad.SetChamferPositions(v['bits'])
    pad.SetChamferRectRatio(v['ratio'])
    pad.SetRoundRectRadiusRatio(v['rr'])
    if v['offset']:
        pad.SetOffset(pcbnew.PADSTACK.ALL_LAYERS,
                      pcbnew.VECTOR2I(MM(v['offset'][0]), MM(v['offset'][1])))
    pad.SetFPRelativeOrientation(deg(v['pad_rot']))
    fp.SetPosition(pcbnew.VECTOR2I(MM(20 + 6 * (n % 25)), MM(20 + 6 * (n // 25))))
    fp.SetOrientation(deg(v['fp_rot']))
    if v['flip']:
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_TOP_BOTTOM if v['flip'] == 'TB'
                else pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    refs[ref] = v
refs['J1'] = dict(flip=None)  # the octagon itself, kept as it is
pcbnew.SaveBoard(posed, board)

rows = []
for path in (posed, rect_board):
    pb = build_pcb_data_from_board(pcbnew.LoadBoard(path))
    pf = parse_kicad_pcb(path)
    diffs = compare_pcb_data(pb, pf)
    for ref, fp in pf.footprints.items():
        if ref == 'P1':
            continue
        fpad, bpad = fp.pads[0], pb.footprints[ref].pads[0]
        row = dict(ref=ref, board=path, variant=refs.get(ref), layer=fp.layer,
                   file_shape=fpad.shape, board_shape=bpad.shape,
                   file_rr=fpad.roundrect_rratio, board_rr=bpad.roundrect_rratio,
                   file_rings=len(fpad.polygons or []), board_rings=len(bpad.polygons or []),
                   diffs=[d for d in diffs if d.startswith(f'Pad {ref}:')])
        if fpad.polygons and bpad.polygons:
            row['file_past_pcbnew'] = _polygon_union_excess(fpad.polygons, bpad.polygons)[0]
            row['pcbnew_past_file'] = _polygon_union_excess(bpad.polygons, fpad.polygons)[0]
        rows.append(row)
print('RESULT ' + json.dumps(rows))
'''


def main():
    kpy = find_kicad_python()
    if kpy is None:
        print("SKIP: KiCad's bundled python not found")
        return SKIP_EXIT
    d = tempfile.mkdtemp(prefix='t1450gp_')
    try:
        src = os.path.join(d, 'octagon_pad.kicad_pcb')
        with open(src, 'w', encoding='utf-8', newline='\n') as f:
            f.write(one.BOARD)
        rect = os.path.join(d, 'rect_chamfer.kicad_pcb')
        with open(rect, 'w', encoding='utf-8', newline='\n') as f:
            f.write(one.BOARD.replace('thru_hole roundrect', 'thru_hole rect')
                    .replace('\t\t\t(roundrect_rratio 0.25)\n', '')
                    .replace('(size 2.4 2.4)', '(size 2.4 1.6)')
                    .replace('top_left top_right bottom_left bottom_right', 'top_left bottom_right'))
        evidence(src, 'the one-pad board')
        evidence(rect, 'the chamfered-rect board')
        posed = os.path.join(d, 'posed.kicad_pcb')
        r = subprocess.run([kpy, '-c', PROBE, os.path.join(ROOT, 'py_router'), src, posed, rect],
                           capture_output=True, text=True, timeout=240)
        rows = None
        for line in r.stdout.splitlines():
            if line.startswith('RESULT '):
                rows = json.loads(line[len('RESULT '):])
        if r.returncode != 0 or not rows:
            print(f'BROKEN TEST: the pcbnew probe gave no result (exit {r.returncode}):\n'
                  f'{r.stdout[-2000:]}\n{r.stderr[-3000:]}')
            return 2
        posed_rows = [w for w in rows if w['board'] == posed]
        rect_rows = [w for w in rows if w['board'] == rect]
        fails = []

        def ck(name, cond, detail=''):
            print(f"  {'PASS' if cond else 'FAIL'}  {name}" + ('' if cond else f'  [{detail}]'))
            if not cond:
                fails.append(name)

        ck('the probe posed every variant and read back the rect board',
           len(posed_rows) > 300 and len(rect_rows) == 1, f'{len(posed_rows)} {len(rect_rows)}')
        ck('the flipped variants are on B.Cu',
           all((w['layer'] == 'B.Cu') == bool(w['variant']['flip']) for w in posed_rows))
        for label, group in (('posed', posed_rows), ('rect', rect_rows)):
            missing = [w['ref'] for w in group if w['file_rings'] != 1 or w['board_rings'] != 1]
            ck(f'{label}: both paths carry one outline for every pad', not missing,
               f'{missing[:5]}')
            worst = max(group, key=lambda w: max(w.get('file_past_pcbnew', 1.0),
                                                 w.get('pcbnew_past_file', 1.0)))
            gap = max(worst.get('file_past_pcbnew', 1.0), worst.get('pcbnew_past_file', 1.0))
            ck(f'{label}: the text outline is pcbnew\'s to within {TOL} mm '
               f'(worst {gap * 1000:.1f} um, {worst["ref"]})', gap < TOL, json.dumps(worst))
            bad = [w for w in group if w['diffs']]
            ck(f'{label}: compare_pcb_data finds no difference on these pads', not bad,
               json.dumps(bad[:2]))
            shape = [w for w in group if (w['file_shape'], w['file_rr'])
                     != (w['board_shape'], w['board_rr'])]
            ck(f'{label}: shape and ratio agree', not shape, json.dumps(shape[:2]))
        ck('a chamfered rect reads as pcbnew\'s roundrect at 0.25',
           rect_rows and (rect_rows[0]['file_shape'], rect_rows[0]['file_rr']) == ('roundrect', 0.25),
           json.dumps(rect_rows))
        n = 2 + 4 * 2 + 1
        print(f'{n - len(fails)}/{n} checks passed over {len(rows)} pads')
        return 1 if fails else 0
    finally:
        shutil.rmtree(d, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
