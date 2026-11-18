#!/usr/bin/env python3
"""check_connected names a net whose only missing pads lie OFF the board in a
bucket of its own (#291), and counts it as no open net.

Every routing pass drops a pad off the Edge.Cuts outline from its endpoints
(`connectivity.drop_off_board_pads`): no copper can reach it. A board that
parks parts outside its outline used to grade every such net open, so an
instrument read "incomplete" for links no router can make. The net is now
graded on its ON-board pads: complete there, it is named in the
"complete on the board (only pads OFF the board outline missing)" line and is
neither an unrouted net nor a connectivity issue. A net broken ON the board
still counts, off-board pad or not.

The board is written here from s-expressions: a 30 x 20 mm outline and

  * ONB   -- two on-board pads joined by a track, a third pad off the board:
             bucketed;
  * LONE  -- one on-board pad and one off-board pad, no copper: bucketed (an
             unrouted net needs two on-board pads to be open);
  * BARE  -- two on-board pads, no copper: unrouted, counted (the control);
  * SPLIT -- three on-board pads with a track joining two of them, plus an
             off-board pad: a connectivity issue, counted.

`ab_replay_grade` reads the bucket into `nets_offboard` and leaves it out of
`nets_incomplete`.

    python3 tests/test_291_offboard_open_bucket.py
"""
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(TESTS_DIR, 'stress'))
sys.path.insert(0, TESTS_DIR)

import run_utils                                                 # noqa: E402
import ab_replay_grade                                           # noqa: E402

failures = []

HEAD = '''(kicad_pcb
\t(version 20260206)
\t(generator "pcbnew")
\t(generator_version "10.0")
\t(general
\t\t(thickness 1.6)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(2 "B.Cu" signal)
\t\t(1 "F.Mask" user)
\t\t(3 "B.Mask" user)
\t\t(5 "F.SilkS" user "F.Silkscreen")
\t\t(25 "Edge.Cuts" user)
\t)
\t(setup
\t\t(pad_to_mask_clearance 0)
\t)
'''


def ok(label, cond, detail=''):
    print(f"  [{'ok' if cond else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not cond:
        failures.append(label)


def _pad(ref, x, y, net):
    return ('\t(footprint "Test:Pad"\n\t\t(layer "F.Cu")\n\t\t(at %g %g)\n'
            '\t\t(property "Reference" "%s"\n\t\t\t(at 0 0 0)\n'
            '\t\t\t(layer "F.SilkS")\n\t\t\t(effects\n\t\t\t\t(font\n'
            '\t\t\t\t\t(size 1 1)\n\t\t\t\t)\n\t\t\t)\n\t\t)\n'
            '\t\t(pad "1" smd rect\n\t\t\t(at 0 0)\n\t\t\t(size 1 1)\n'
            '\t\t\t(layers "F.Cu" "F.Mask")\n\t\t\t(net "%s")\n\t\t)\n\t)\n'
            % (x, y, ref, net))


def _track(x1, y1, x2, y2, net):
    return ('\t(segment\n\t\t(start %g %g)\n\t\t(end %g %g)\n\t\t(width 0.25)\n'
            '\t\t(layer "F.Cu")\n\t\t(net "%s")\n\t)\n' % (x1, y1, x2, y2, net))


def _edge(x1, y1, x2, y2):
    return ('\t(gr_line\n\t\t(start %g %g)\n\t\t(end %g %g)\n'
            '\t\t(stroke\n\t\t\t(width 0.1)\n\t\t\t(type default)\n\t\t)\n'
            '\t\t(layer "Edge.Cuts")\n\t)\n' % (x1, y1, x2, y2))


def write_board(path):
    parts = [HEAD,
             # ONB: complete on the board, a third pad 10 mm past the edge
             _pad('R1', 5, 4, 'ONB'), _pad('R2', 20, 4, 'ONB'), _pad('R3', 40, 4, 'ONB'),
             _track(5, 4, 20, 4, 'ONB'),
             # LONE: one pad on, one pad off, no copper
             _pad('R4', 5, 8, 'LONE'), _pad('R5', 40, 8, 'LONE'),
             # BARE: two on-board pads, no copper (control: unrouted)
             _pad('R6', 5, 12, 'BARE'), _pad('R7', 20, 12, 'BARE'),
             # SPLIT: broken on the board, plus an off-board pad
             _pad('R8', 5, 16, 'SPLIT'), _pad('R9', 15, 16, 'SPLIT'),
             _pad('R10', 25, 16, 'SPLIT'), _pad('R11', 40, 16, 'SPLIT'),
             _track(5, 16, 15, 16, 'SPLIT')]
    corners = [(0, 0), (30, 0), (30, 20), (0, 20)]
    for i in range(4):
        parts.append(_edge(*corners[i], *corners[(i + 1) % 4]))
    parts.append(')\n')
    with open(path, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(''.join(parts))
    return run_utils.evidence(path, 'off-board pad board')


def main():
    with tempfile.TemporaryDirectory() as td:
        board = write_board(os.path.join(td, 'offboard.kicad_pcb'))
        r = subprocess.run([sys.executable, '-X', 'utf8',
                            os.path.join(ROOT, 'py_router', 'check_connected.py'), board],
                           capture_output=True, text=True, cwd=ROOT)
        text = r.stdout + r.stderr
        ok('check_connected ran (no traceback)', 'Traceback' not in text, text[-400:])
        bucket = next((ln for ln in text.splitlines()
                       if 'only pads OFF the board outline missing' in ln), '')
        ok('ONB and LONE are named in the off-board bucket',
           bucket.strip().startswith('2 net(s)') and 'ONB' in bucket and 'LONE' in bucket,
           bucket.strip())
        ok('BARE stays an unrouted net', 'Unrouted nets (1)' in text
           and '    BARE (2 pads)' in text)
        ok('SPLIT stays a connectivity issue', 'Connectivity issues (1)' in text
           and 'SPLIT (net' in text)
        ok('neither bucketed net is reported open',
           'ONB (net' not in text and 'LONE (' not in text.split('Unrouted nets')[-1])
        ok('exit is non-zero (BARE and SPLIT are open)', r.returncode != 0,
           f'rc={r.returncode}')
        ok('ab_replay_grade reads the bucket', ab_replay_grade._offboard_count(text) == 2)
        total, incomplete, _ = ab_replay_grade._completion(text)
        ok('nets_incomplete counts only BARE and SPLIT', incomplete == 2,
           f'incomplete={incomplete} total={total}')
    print('PASS' if not failures else f'FAIL ({len(failures)}): {failures}')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
