#!/usr/bin/env python3
"""#1315: a foreign track that ENTERS the BGA escape window is an obstacle,
whichever end KiCad wrote first.

The under-pad / dog-bone engine filtered the board's segments into its
occupancy grid by the segment's START point alone. A foreign track running
into the ball field from outside the window was therefore stamped or ignored
depending on its direction: on imx6ulz U2 a /DRAM/DRAM_DQ5 F.Cu track
starting at x=79.9 (window max ~75.0) was invisible, VMEM escapes and a
DQS0_N via were laid on it -- a short -- and the fanout reported failed 0,
while the same track written end-first was an obstacle and VMEM honestly
failed. The window test is now the segment's extent grown by its keep-out
(`underpad.segment_reaches_window`).

Reproduced on the tracked board: glasgow_revC U30.E8 alone, under-pad, with
a foreign /IO_Banks/DB1 F.Cu track across the ball's natural exit (the
channel between the last two ball rows), starting outside the window.

Checks:
  1. The predicate: a segment entering from outside, the same segment
     reversed, and one crossing the whole window with both ends outside all
     reach it; a segment wholly outside does not, and its keep-out reach is
     what decides a segment that stops just short of the edge.
  2. Precondition: without the track the escape runs down x=84.175 across
     the track's line (y=99.5) -- so the track is genuinely in its way.
  3. With the track written start-OUTSIDE (the defect's case) and written
     start-inside, the ball still escapes, every escape segment clears the
     track by half-widths + clearance, check_drc finds no violation between
     the two nets, and both directions give the SAME escape.

    python3 tests/test_1315_underpad_entering_track.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from copy_board import copy_board                              # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402
from geometry_utils import segment_to_segment_distance         # noqa: E402
from bga_fanout.underpad import segment_reaches_window         # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'glasgow_revC.kicad_pcb')
NET = '/IO_Banks/U5'                     # U30.E8
FOREIGN = '/IO_Banks/DB1'
OUTSIDE, INSIDE = (90.0, 99.5), (84.0, 99.5)   # U30 array ends at x=88.1
TRACK_W, HALF_W, CLR = 0.0889, 0.0889 / 2, 0.0889
ARGS = ['--component', 'U30', '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
        '--layer-costs', '1.0', '4.0', '2.5', '1.0', '--track-width', '0.0762',
        '--clearance-ceiling', '0.0889', '--clearance', '0.0889',
        '--via-size', '0.25', '--via-drill', '0.15', '--nets', NET,
        '--escape-method', 'underpad']
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def parse(path):
    with contextlib.redirect_stdout(io.StringIO()), \
            contextlib.redirect_stderr(io.StringIO()):
        return parse_kicad_pcb(path)


def seg(a, b):
    return SimpleNamespace(start_x=a[0], start_y=a[1], end_x=b[0], end_y=b[1])


def fan(inp, out):
    r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/bga_fanout.py',
                        inp, '-o', out] + ARGS, cwd=ROOT, capture_output=True,
                       text=True, encoding='utf-8', errors='replace')
    line = next((l for l in r.stdout.splitlines() if 'JSON_SUMMARY' in l), '')
    js = json.loads(line.split('JSON_SUMMARY:', 1)[1]) if line else {}
    before = {(s.start_x, s.start_y, s.end_x, s.end_y) for s in parse(inp).segments}
    after = parse(out)
    nid = next(i for i, n in after.nets.items() if n.name == NET)
    esc = [s for s in after.segments if s.net_id == nid
           and (s.start_x, s.start_y, s.end_x, s.end_y) not in before]
    return r.returncode, js, esc


def with_track(inp, path, a, b):
    """`inp` plus one foreign F.Cu track written a -> b (project carried)."""
    text = open(inp, encoding='utf-8').read()
    trk = (f'\t(segment\n\t\t(start {a[0]} {a[1]})\n\t\t(end {b[0]} {b[1]})\n'
           f'\t\t(width {TRACK_W})\n\t\t(layer "F.Cu")\n\t\t(net "{FOREIGN}")\n'
           f'\t\t(uuid "13151315-0000-4000-8000-000000001315")\n\t)\n')
    i = text.rstrip().rfind(')')
    open(path, 'w', encoding='utf-8').write(text[:i] + trk + text[i:])
    shutil.copyfile(inp[:-len('.kicad_pcb')] + '.kicad_pro',
                    path[:-len('.kicad_pcb')] + '.kicad_pro')
    return path


def main():
    # 1. The predicate.
    win = (78.3, 90.1, 89.1, 100.9)
    check('a segment entering from outside reaches the window',
          segment_reaches_window(seg(OUTSIDE, INSIDE), win, 0.2))
    check('the same segment reversed reaches it',
          segment_reaches_window(seg(INSIDE, OUTSIDE), win, 0.2))
    check('a segment crossing the window with both ends outside reaches it',
          segment_reaches_window(seg((70.0, 95.0), (95.0, 95.0)), win, 0.2))
    check('a segment wholly outside does not',
          not segment_reaches_window(seg((92.0, 95.0), (95.0, 95.0)), win, 0.2))
    check('a segment stopping short of the edge is decided by its reach',
          segment_reaches_window(seg((89.25, 95.0), (95.0, 95.0)), win, 0.2)
          and not segment_reaches_window(seg((89.35, 95.0), (95.0, 95.0)), win, 0.2))

    tmp = tempfile.mkdtemp(prefix='t1315_')
    try:
        inp = os.path.join(tmp, 'in.kicad_pcb')
        with contextlib.redirect_stdout(io.StringIO()):
            copy_board(BOARD, inp)

        # 2. Precondition: the natural escape crosses the track's line.
        rc, js, esc = fan(inp, os.path.join(tmp, 'plain.kicad_pcb'))
        crosses = any(min(s.start_y, s.end_y) < OUTSIDE[1] < max(s.start_y, s.end_y)
                      and INSIDE[0] < s.start_x < OUTSIDE[0] for s in esc)
        check('precondition: without the track the escape crosses y=99.5',
              rc == 0 and js.get('escaped') == 1 and crosses,
              f'rc={rc} escaped={js.get("escaped")} crosses={crosses}')

        # 3. Both directions of the foreign track.
        shapes = {}
        for label, a, b in (('start-outside', OUTSIDE, INSIDE),
                            ('start-inside', INSIDE, OUTSIDE)):
            tin = with_track(inp, os.path.join(tmp, f'{label}.kicad_pcb'), a, b)
            tout = os.path.join(tmp, f'{label}_out.kicad_pcb')
            rc, js, esc = fan(tin, tout)
            check(f'{label}: the ball still escapes',
                  rc == 0 and js.get('escaped') == 1 and esc,
                  f'rc={rc} escaped={js.get("escaped")}')
            gap = min((segment_to_segment_distance(
                s.start_x, s.start_y, s.end_x, s.end_y, *OUTSIDE, *INSIDE)
                - (s.width or 0.0) / 2 - HALF_W
                for s in esc if s.layer == 'F.Cu'), default=float('inf'))
            check(f'{label}: every F.Cu escape clears the foreign track',
                  gap >= CLR - 1e-6, f'edge gap {gap:.4f} >= {CLR}')
            r = subprocess.run([sys.executable, '-X', 'utf8', 'py_router/check_drc.py',
                                tout, '--clearance-margin', '0.1', '--max-print', '0'],
                               cwd=ROOT, capture_output=True, text=True,
                               encoding='utf-8', errors='replace')
            vs = [l.strip() for l in r.stdout.splitlines()
                  if FOREIGN in l and NET in l]
            graded = ('DRC VIOLATIONS:' in r.stdout
                      or 'NO DRC VIOLATIONS FOUND' in r.stdout)
            check(f'{label}: check_drc finds nothing between the two nets',
                  r.returncode in (0, 1) and graded and not vs,
                  '; '.join(vs[:2]) or ('' if graded else 'check_drc did not grade'))
            shapes[label] = sorted((round(s.start_x, 4), round(s.start_y, 4),
                                    round(s.end_x, 4), round(s.end_y, 4), s.layer)
                                   for s in esc)
        check('both directions of the track give the same escape',
              shapes['start-outside'] == shapes['start-inside'],
              f"{len(shapes['start-outside'])} vs {len(shapes['start-inside'])} segments")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print('FAILED: ' + ', '.join(failures) if failures else 'PASS')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
