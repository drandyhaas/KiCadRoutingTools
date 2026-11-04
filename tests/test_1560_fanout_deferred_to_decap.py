#!/usr/bin/env python3
"""#1560: bga_fanout's `drc_grazes` tells the grazes it leaves for the
clearance step apart from the rest.

bga_fanout lays vias and escapes on unlocked 2-pad C/R/FB parts on purpose:
place_fanout_clearance moves them afterwards. A standalone fanout therefore
reports grazes the next step clears, and its `drc_grazes` read them as real.
It now carries `deferred_to_decap` (of `total`, the rows on a pad of a part
the clearance step moves, by that step's own rule) and `deferred_parts`.

On a synthetic 6x6 0.5 mm BGA with a back-side 0402-size part C1 whose pads
sit under two inner balls (the case of tests/test_1521_refan_held_parts.py):
1. The under-pad fanout lays copper on C1: `total` > 0, every row of it is
   `deferred_to_decap`, and `deferred_parts` is ['C1'].
2. `total` and the per-class counts are unchanged by the split: the same
   written board graded with nothing deferred has the same counts and
   `deferred_to_decap` 0; graded without `deferred_refs` (qfn_fanout's call)
   it carries no such key.
3. C1 held with --immovable-refs (the clearance step could not clear it):
   C1 is not deferred.

    python3 tests/test_1560_fanout_deferred_to_decap.py
"""
import contextlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, HERE)

from run_utils import tool, evidence  # noqa: E402

RUN_ALL_TIMEOUT = 300
N, PITCH, BALL = 6, 0.5, 0.3
FAILS = []


def check(label, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ') + label + (f'  [{detail}]' if not ok else ''))
    if not ok:
        FAILS.append(label)


def board_text():
    cx = cy = 10.0
    nets = ['\t(net 0 "")\n']
    balls, far = [], []
    k = 0
    for r in range(N):
        for c in range(N):
            k += 1
            lx, ly = (c - (N - 1) / 2) * PITCH, (r - (N - 1) / 2) * PITCH
            nets.append('\t(net %d "N%d")\n' % (k, k))
            balls.append('\t\t(pad "%s%d" smd circle (at %g %g) (size %g %g) '
                         '(layers "F.Cu") (net %d "N%d"))\n'
                         % (chr(65 + r), c + 1, lx, ly, BALL, BALL, k, k))
            far.append('\t\t(pad "%d" smd rect (at 0 %g) (size 0.3 0.3) '
                       '(layers "F.Cu") (net %d "N%d"))\n'
                       % (k, -16.0 + 0.4 * (k - 1), k, k))
    nets.append('\t(net 101 "P1V")\n\t(net 102 "GNDX")\n')
    # C1 on B.Cu, row C, its pads under balls C3 and C5 (symmetric about its
    # origin, so the side's mirror does not move them)
    y = cy + (2 - (N - 1) / 2) * PITCH
    c1x = cx + (3 - (N - 1) / 2) * PITCH
    c1 = ('\t(footprint "test:C0402" (layer "B.Cu") (uuid "c1") (at %g %g)\n'
          '\t\t(property "Reference" "C1" (at 0 0) (layer "B.SilkS"))\n'
          '\t\t(pad "1" smd rect (at -0.5 0) (size 0.3 0.3) (layers "B.Cu") (net 101 "P1V"))\n'
          '\t\t(pad "2" smd rect (at 0.5 0) (size 0.3 0.3) (layers "B.Cu") (net 102 "GNDX"))\n'
          '\t)\n' % (c1x, y))
    u1 = ('\t(footprint "test:BGA36" (layer "F.Cu") (uuid "u1") (at %g %g)\n'
          '\t\t(property "Reference" "U1" (at 0 0) (layer "F.SilkS"))\n%s\t)\n'
          % (cx, cy, ''.join(balls)))
    j1 = ('\t(footprint "test:J" (layer "F.Cu") (uuid "j1") (at 26 %g)\n'
          '\t\t(property "Reference" "J1" (at 0 0) (layer "F.SilkS"))\n%s\t)\n'
          % (cy, ''.join(far)))
    return ('(kicad_pcb\n\t(version 20241229)\n\t(generator "synth")\n'
            '\t(general (thickness 1.6))\n'
            '\t(layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal) '
            '(31 "B.Cu" signal) (44 "Edge.Cuts" user))\n'
            + ''.join(nets)
            + '\t(gr_rect (start 0 -12) (end 30 32) (stroke (width 0.1) (type solid)) '
              '(fill none) (layer "Edge.Cuts") (uuid "edge"))\n'
            + u1 + j1 + c1 + ')\n')


def fan(src, out, *extra):
    r = subprocess.run([sys.executable, '-X', 'utf8', tool('bga_fanout.py'), src,
                        '--component', 'U1', '--output', out,
                        '--layers', 'F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu',
                        '--escape-method', 'underpad', '--track-width', '0.1',
                        '--clearance', '0.1', '--via-size', '0.25',
                        '--via-drill', '0.15', *extra],
                       capture_output=True, text=True, encoding='utf-8',
                       errors='replace', cwd=ROOT, timeout=600)
    rows = [json.loads(l.split('JSON_SUMMARY: ', 1)[1])
            for l in r.stdout.splitlines() if l.startswith('JSON_SUMMARY: ')]
    if r.returncode != 0 or not rows:
        raise AssertionError(f"BROKEN TEST: bga_fanout exited {r.returncode} with "
                             f"{len(rows)} JSON_SUMMARY line(s)\n{r.stdout[-800:]}"
                             f"\n{r.stderr[-800:]}")
    evidence(out)
    return rows[-1].get('drc_grazes') or {}


def main():
    work = tempfile.mkdtemp(prefix='krt1560_')
    try:
        src = os.path.join(work, 'in.kicad_pcb')
        with open(src, 'w', encoding='utf-8') as fh:
            fh.write(board_text())
        out = os.path.join(work, 'out.kicad_pcb')
        g = fan(src, out)
        print("1. unheld: the fanout's copper on C1 is deferred to the clearance step")
        check("the fanout grazes C1 (the fixture exercises the case)",
              g.get('pad_via', 0) + g.get('pad_segment', 0) > 0, g)
        check("every graze is deferred_to_decap, on C1",
              g.get('deferred_to_decap') == g.get('total')
              and g.get('deferred_parts') == ['C1'], g)

        print("2. the split leaves the counts as they were")
        from kicad_parser import parse_kicad_pcb
        from check_drc import copper_keys, fanout_drc_grazes
        with contextlib.redirect_stdout(io.StringIO()):
            base = copper_keys(parse_kicad_pcb(src))
        keys = ('pad_via', 'via_segment', 'pad_segment', 'segment_segment', 'total')
        g0 = fanout_drc_grazes(out, 0.1, base, net_classes=True, deferred_refs=set())
        check("nothing deferred: same counts, deferred_to_decap 0",
              [g0.get(k) for k in keys] == [g.get(k) for k in keys]
              and g0.get('deferred_to_decap') == 0, (g0, g))
        g1 = fanout_drc_grazes(out, 0.1, base, net_classes=True)
        check("no deferred_refs (qfn_fanout's call): no deferred key",
              'deferred_to_decap' not in g1 and g1.get('total') == g.get('total'), g1)

        print("3. a held or locked part is not deferred")
        from bga_fanout.geometry import decap_deferred_refs
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(out)
        check("the clearance step's part (C1) is deferred; the BGA and the "
              "connector are not", decap_deferred_refs(pcb, out) == {'C1'},
              decap_deferred_refs(pcb, out))
        check("held with --immovable-refs: not deferred",
              decap_deferred_refs(pcb, out, ['C1']) == set(),
              decap_deferred_refs(pcb, out, ['C1']))
        locked = os.path.join(work, 'locked.kicad_pcb')
        with open(out, encoding='utf-8') as fh:
            text = fh.read()
        with open(locked, 'w', encoding='utf-8') as fh:
            fh.write(text.replace('(footprint "test:C0402"',
                                  '(footprint "test:C0402" (locked yes)', 1))
        with contextlib.redirect_stdout(io.StringIO()):
            pl = parse_kicad_pcb(locked)
        check("locked: not deferred", decap_deferred_refs(pl, locked) == set(),
              decap_deferred_refs(pl, locked))
        gh = fan(src, os.path.join(work, 'held.kicad_pcb'), '--immovable-refs', 'C1')
        check("the CLI passes its held parts: a held run defers nothing to C1",
              'C1' not in (gh.get('deferred_parts') or ())
              and gh.get('deferred_to_decap') == 0, gh)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    if FAILS:
        print(f"\nFAILED: {len(FAILS)} check(s)")
        for f in FAILS:
            print(f"  - {f}")
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
