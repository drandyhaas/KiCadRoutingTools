#!/usr/bin/env python3
"""A net whose pad is shorted to another net AS DRAWN is named, not counted.

A measured board stacks net ties NT1/NT2 at one pose, so
the pads tying /Connectors/ETH_PORT4 and ETH_PORT5 to the bridge coincide, and
overlaps NT3/NT4 rotated 180 degrees against each other, so ETH_PORT7 and
ETH_PORT8 each sit on the other tie's bridge pad. KiCad 10 grades seven
`shorting_items` on the UNROUTED board. No copper can reach those pads without
joining the other net, so five nets came back open from every route step,
one more line in the unrouted count. This pins:

  1. check_pads.pads_shorted_as_drawn names every pad shorted as drawn, by
     another net's pad and by another part's copper, for exactly the nets
     involved -- not the bridge net whose two tie pads coincide on ONE net;
  2. the board with the ties posed as intended (NT2 turned about the shared
     bridge pad, NT3 moved one pitch along its axis) has none;
  3. route.py names the open nets under `unroutable_as_designed` in
     JSON_SUMMARY and prints them; on the corrected board every net routes
     and the key is absent;
  4. merge_summaries carries the key from pass 1 for nets still failing,
     and JSON_SUMMARY_MIN names them.

    python3 tests/test_unroutable_as_designed.py
"""
import contextlib
import io
import json
import os
import subprocess
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence  # noqa: E402

fails = []


def check(label, ok, detail=''):
    print(f"  {'ok  ' if ok else 'FAIL'} {label}{(': ' + str(detail)) if detail and not ok else ''}")
    if not ok:
        fails.append(label)


NETS = {1: '/Connectors/ETH_PORT4', 2: '/Connectors/ETH_PORT5',
        3: '/Connectors/ETH_PORT7', 4: '/Connectors/ETH_PORT8',
        5: 'Net-(D1-Pad3)', 6: 'Net-(D1-Pad4)'}
SHORTED = sorted(NETS[n] for n in (1, 2, 3, 4, 5))


def tie(ref, x, y, rot, net1, net2):
    """KiCad's NetTie-2_SMD_Pad0.5mm, as the measured board places it."""
    return (f''' (footprint "NetTie:NetTie-2_SMD_Pad0.5mm" (layer "F.Cu") (at {x} {y} {rot})
  (property "Reference" "{ref}" (at 0 -1.2) (layer "F.SilkS"))
  (attr exclude_from_pos_files exclude_from_bom allow_missing_courtyard)
  (net_tie_pad_groups "1, 2")
  (fp_poly (pts (xy -0.5 -0.25) (xy 0.5 -0.25) (xy 0.5 0.25) (xy -0.5 0.25))
   (stroke (width 0) (type solid)) (fill solid) (layer "F.Cu"))
  (pad "1" smd circle (at -0.5 0 {rot}) (size 0.5 0.5) (layers "F.Cu") (net {net1} "{NETS[net1]}"))
  (pad "2" smd circle (at 0.5 0 {rot}) (size 0.5 0.5) (layers "F.Cu") (net {net2} "{NETS[net2]}")))
''')


def board(path, corrected):
    # As drawn: NT1/NT2 identical, NT3/NT4 at one centre turned 180 apart.
    # Corrected: NT2 turned 180 about the bridge pad it shares with NT1, and
    # NT3 moved 1 mm along its axis so its bridge pad lands on NT4's.
    nt2 = (113.054, 187.24, 0) if corrected else (114.054, 187.24, 180)
    nt3 = (112.784, 184.335, -90) if corrected else (112.784, 185.335, -90)
    nets = ''.join(f' (net {i} "{n}")\n' for i, n in NETS.items())
    jpads = ''.join(
        f'  (pad "{i}" smd rect (at {x} {y}) (size 0.8 0.8) (layers "F.Cu") '
        f'(net {n} "{NETS[n]}"))\n'
        for i, (x, y, n) in enumerate([(4.372, -1.148, 1), (3.356, 0.632, 2),
                                       (1.324, 0.632, 3), (0.308, -1.148, 4)],
                                      start=1))
    txt = f'''(kicad_pcb
 (version 20221018)
 (generator "test_unroutable_as_designed")
 (general (thickness 1.6))
 (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
 (net 0 "")
{nets} (gr_rect (start 98 172) (end 122 192) (layer "Edge.Cuts") (width 0.1))
 (footprint "t:J" (layer "F.Cu") (at 111 180)
  (property "Reference" "J3" (at 0 -3) (layer "F.SilkS"))
{jpads} )
 (footprint "t:D" (layer "F.Cu") (at 102 184)
  (property "Reference" "D1" (at 0 -4) (layer "F.SilkS"))
  (pad "3" smd rect (at 0 -2) (size 0.8 0.8) (layers "F.Cu") (net 5 "{NETS[5]}"))
  (pad "4" smd rect (at 0 2) (size 0.8 0.8) (layers "F.Cu") (net 6 "{NETS[6]}")))
{tie('NT1', 114.054, 187.24, 180, 1, 6)}{tie('NT2', *nt2, 2, 6)}{tie('NT3', *nt3, 3, 5)}{tie('NT4', 112.784, 185.335, 90, 4, 5)})
'''
    with open(path, 'w', encoding='utf-8') as f:
        f.write(txt)
    return evidence(path, 'generated board')


def measure(path):
    from kicad_parser import parse_kicad_pcb
    from check_pads import pads_shorted_as_drawn
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(path)
    return pads_shorted_as_drawn(pcb, list(pcb.nets))


def route(src, out, js):
    r = run_check([sys.executable, '-X', 'utf8',
                   os.path.join(ROOT, 'py_router', 'route.py'), src, out,
                   '--nets', *NETS.values(), '--layers', 'F.Cu', 'B.Cu',
                   '--track-width', '0.15', '--clearance', '0.1',
                   '--grid-step', '0.05', '--json-out', js],
                  accept=True, timeout=900,
                  env=dict(os.environ, MSYS2_ARG_CONV_EXCL='*'))
    with open(evidence(js, '--json-out summary'), encoding='utf-8') as f:
        return json.load(f), r.stdout


with tempfile.TemporaryDirectory() as d:
    drawn = board(os.path.join(d, 'drawn.kicad_pcb'), corrected=False)
    fixed = board(os.path.join(d, 'fixed.kicad_pcb'), corrected=True)

    print("1. as drawn: every shorted pad, by pad and by copper")
    rows = measure(drawn)
    got = {(r['net'], r['pad'], r['by'], r['foreign']) for r in rows}
    want = {(NETS[1], 'NT1.1', 'pad', 'NT2.1'), (NETS[2], 'NT2.1', 'pad', 'NT1.1'),
            (NETS[3], 'NT3.1', 'pad', 'NT4.2'), (NETS[4], 'NT4.1', 'pad', 'NT3.2'),
            (NETS[5], 'NT3.2', 'pad', 'NT4.1'), (NETS[5], 'NT4.2', 'pad', 'NT3.1'),
            (NETS[1], 'NT1.1', 'footprint copper', 'NT2'),
            (NETS[2], 'NT2.1', 'footprint copper', 'NT1'),
            (NETS[3], 'NT3.1', 'footprint copper', 'NT4'),
            (NETS[4], 'NT4.1', 'footprint copper', 'NT3')}
    check("KiCad's seven shorting_items, one row per shorted pad, no others",
          got == want, sorted(got ^ want))
    check("the bridge net whose two tie pads coincide on ONE net is not named",
          NETS[6] not in {r['net'] for r in rows})

    print("2. posed as intended: nothing shorted")
    check("no row on the corrected board", measure(fixed) == [], measure(fixed))

    print("3. route.py says so by name")
    doc, out = route(drawn, os.path.join(d, 'drawn_out.kicad_pcb'),
                     os.path.join(d, 'drawn.json'))
    named = sorted({r['net'] for r in doc.get('unroutable_as_designed') or []})
    check("JSON_SUMMARY unroutable_as_designed names the five open nets",
          named == SHORTED, named)
    check("...and the console says it",
          'Unroutable as designed' in out and NETS[1] in out.split(
              'Unroutable as designed', 1)[-1])
    doc, out = route(fixed, os.path.join(d, 'fixed_out.kicad_pcb'),
                     os.path.join(d, 'fixed.json'))
    check("corrected board: every net routes",
          not doc.get('failed_single') and not doc.get('failed_multipoint')
          and not doc.get('open_single'),
          (doc.get('failed_single'), doc.get('failed_multipoint')))
    check("...and nothing is disclosed",
          'unroutable_as_designed' not in doc, doc.get('unroutable_as_designed'))

    print("4. the merge keeps it for nets still failing")
    from route_summary import merge_summaries
    row = lambda n: {'net': n, 'pad': 'X.1', 'by': 'pad', 'foreign': 'Y.1',
                     'foreign_nets': ['Z']}
    first = {'failed_single': ['A', 'B'], 'open_single': [], 'failed_multipoint': [],
             'unroutable_as_designed': [row('A'), row('B')]}
    last = {'failed_single': ['A'], 'open_single': [], 'failed_multipoint': []}
    m = merge_summaries([first, last])
    check("pass 1's rows survive a reconcile summary without the key, for "
          "the nets still failing only",
          [r['net'] for r in m.get('unroutable_as_designed', [])] == ['A'],
          m.get('unroutable_as_designed'))
    from route_summary import summary_min
    check("JSON_SUMMARY_MIN names them",
          summary_min(m).get('unroutable_as_designed') == ['A'],
          summary_min(m).get('unroutable_as_designed'))
    check("...and has no such key when nothing is shorted",
          'unroutable_as_designed' not in summary_min(merge_summaries([last])))

print('-' * 60)
if fails:
    print(f"{len(fails)} FAILED: " + '; '.join(fails))
    sys.exit(1)
print('ALL PASS')
