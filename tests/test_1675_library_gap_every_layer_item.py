#!/usr/bin/env python3
"""#1675: every KiCad item on a published library pad pair goes to the channel.

check_drc publishes two of one footprint's pads closer than their clearance as
ONE `own-footprint-gap` row per pair. KiCad lists the pair once per copper layer
the two pads share: one item for an SMD pair, two identical ones for a
through-hole pair on 2 layers, four on 4. kicad_drc_compare spent the row on the
first item, so the second became a KICAD-ONLY "check_drc false negative" on a
pair check_drc had disclosed, and the tool exited 1.

The pure half needs no KiCad and pins what may and may not be dropped; the
through-hole boards are graded by the real kicad-cli when one is installed.

    python3 tests/test_1675_library_gap_every_layer_item.py
"""
import json
import os
import shutil
import sys
import tempfile

TESTS = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(TESTS)
for p in (TESTS, os.path.join(TESTS, 'stress'), os.path.join(REPO, 'py_router')):
    if p not in sys.path:
        sys.path.insert(0, p)
import run_utils  # noqa: E402
import kicad_drc_compare as kdc  # noqa: E402

FAILS = []


def ok(name, cond, detail=''):
    print(('  [ok] ' if cond else '  [FAIL] ') + name
          + ('' if cond else f': {detail}'))
    if not cond:
        FAILS.append(name)


def kicad_item(a, b, a_pos=(10.0, 10.0), b_pos=(11.0, 10.0)):
    return {'type': 'clearance',
            'description': "Clearance violation (netclass 'W' clearance 0.4 mm; actual 0.28 mm)",
            'severity': 'error',
            'items': [{'description': a, 'pos': {'x': a_pos[0], 'y': a_pos[1]}},
                      {'description': b, 'pos': {'x': b_pos[0], 'y': b_pos[1]}}]}


def published(ref_a='Q3.1', ref_b='Q3.2'):
    return {'type': 'pad-pad', 'nets': frozenset(('/A', '/B')),
            'pos': (10.0, 10.0), 'pos2': (11.0, 10.0),
            'accepted': 'own-footprint-gap',
            'pad_pair': frozenset((ref_a, ref_b))}


def drop(items, rows):
    parsed = kdc.parse_kicad_report({'violations': items})
    return kdc._drop_kicad_library_gaps(parsed, rows)


PTH_A, PTH_B = 'PTH pad 1 [/A] of Q3', 'PTH pad 2 [/B] of Q3'


def t_pure():
    keep, n = drop([kicad_item(PTH_A, PTH_B)] * 2, [published()])
    ok('both of a through-hole pair\'s items go to the channel',
       (len(keep), n) == (0, 2), (keep, n))
    keep, n = drop([kicad_item(PTH_A, PTH_B)] * 4, [published()])
    ok('all four on a 4-layer board go too', (len(keep), n) == (0, 4), (keep, n))
    keep, n = drop([kicad_item(PTH_B, PTH_A)], [published()])
    ok('either order of the two pads names the pair', (len(keep), n) == (0, 1), (keep, n))
    keep, n = drop([kicad_item('Pad 1 [/A] of Q3 on F.Cu', 'Pad 2 [/B] of Q3 on F.Cu')],
                   [published()])
    ok('an SMD pair\'s one item goes', (len(keep), n) == (0, 1), (keep, n))

    # What the channel must NOT take now that a row is never consumed.
    track = kicad_item('Track [/A] on F.Cu, length 3.0000 mm', PTH_B)
    keep, n = drop([kicad_item(PTH_A, PTH_B), track], [published()])
    ok('a track beside the pad, same net pair, stays a finding',
       n == 1 and len(keep) == 1 and 'track' in keep[0]['kinds'], (keep, n))
    other = kicad_item(PTH_A, 'PTH pad 2 [/B] of Q9')
    keep, n = drop([kicad_item(PTH_A, PTH_B), other], [published()])
    ok('another part\'s pad beside it, same net pair, stays a finding',
       n == 1 and len(keep) == 1, (keep, n))
    far = kicad_item('PTH pad 1 [/A] of Q7', 'PTH pad 2 [/B] of Q7',
                     a_pos=(80.0, 80.0), b_pos=(81.0, 80.0))
    keep, n = drop([kicad_item(PTH_A, PTH_B), far], [published()])
    ok('an item far from the published pads, same net pair, stays a finding',
       n == 1 and len(keep) == 1 and keep[0]['pos'] == (80.0, 80.0), (keep, n))
    same_name_far = kicad_item(PTH_A, PTH_B, a_pos=(80.0, 80.0), b_pos=(81.0, 80.0))
    keep, n = drop([same_name_far], [published()])
    ok('the same pad names far from the row (a reference used twice) stay a finding',
       n == 0 and len(keep) == 1, (keep, n))
    keep, n = drop([kicad_item(PTH_A, PTH_B)], [])
    ok('no published row, nothing dropped', (len(keep), n) == (1, 0), (keep, n))

    parsed = kdc.parse_kicad_report({'violations': [
        kicad_item('Pad [<no net>] of U4 on F.Cu', 'Pad 3 [/B] of U4 on F.Cu')]})
    ok('an unnumbered pad is still named', parsed[0]['pads'] == ('U4.', 'U4.3'),
       parsed[0]['pads'])


def build(d, n_layers, with_track=False):
    """One part, two through-hole pads 0.15 mm apart against a 0.3 class."""
    os.makedirs(d, exist_ok=True)
    pcb = os.path.join(d, 'b.kicad_pcb')

    def pad(n, x, net, name):
        return (f'    (pad "{n}" thru_hole circle (at {x} 0) (size 1 1) (drill 0.5) '
                f'(layers "*.Cu") (net {net} "{name}"))\n')

    def head(ref, x, y):
        return (f'  (footprint "T:{ref}" (layer "F.Cu") (at {x} {y})\n'
                f'    (property "Reference" "{ref}" (at 0 -2 0) (layer "F.SilkS") '
                f'(hide yes) (effects (font (size 1 1) (thickness 0.15))))\n'
                f'    (attr through_hole)\n')
    layers = {2: ((0, 'F.Cu'), (2, 'B.Cu')),
              4: ((0, 'F.Cu'), (4, 'In1.Cu'), (6, 'In2.Cu'), (2, 'B.Cu'))}[n_layers]
    b = ('(kicad_pcb (version 20241229) (generator "pcbnew") (generator_version "9.0")\n'
         ' (general (thickness 1.6))\n (paper "A4")\n (layers '
         + ' '.join(f'({i} "{n}" signal)' for i, n in layers)
         + ' (25 "Edge.Cuts" user))\n (setup (pad_to_mask_clearance 0))\n'
         ' (net 0 "") (net 1 "/A") (net 2 "/B")\n')
    b += head('Q3', 120, 110) + pad(1, 0, 1, '/A') + pad(2, 1.15, 2, '/B') + '  )\n'
    if with_track:
        # net /A, running over the far side of Q3's pad 2 inside its clearance
        b += ('  (segment (start 121.15 110.7) (end 124 110.7) (width 0.2) '
              '(layer "F.Cu") (net 1))\n')
    for x1, y1, x2, y2 in ((100, 100, 150, 100), (150, 100, 150, 125),
                           (150, 125, 100, 125), (100, 125, 100, 100)):
        b += (f'  (gr_line (start {x1} {y1}) (end {x2} {y2}) (layer "Edge.Cuts") '
              f'(stroke (width 0.05) (type solid)))\n')
    with open(pcb, 'w', encoding='utf-8') as f:
        f.write(b + ')\n')
    cl = dict(name='Default', clearance=0.3, track_width=0.15, via_diameter=0.6,
              via_drill=0.3, wire_width=6, bus_width=12, line_style=0,
              priority=2147483647)
    with open(pcb[:-len('.kicad_pcb')] + '.kicad_pro', 'w', encoding='utf-8') as f:
        json.dump({'board': {'design_settings': {
                       'rules': {'min_clearance': 0.1, 'min_track_width': 0.1}}},
                   'meta': {'filename': 'b.kicad_pro', 'version': 3},
                   'net_settings': {'classes': [cl], 'meta': {'version': 4}}},
                  f, indent=1)
    return pcb


def t_kicad(tmp):
    from kicad_oracle import find_kicad_cli
    if not find_kicad_cli(warn=False):
        print('  (kicad-cli not found: the through-hole boards are not graded)')
        return
    cmp_py = os.path.join(TESTS, 'stress', 'kicad_drc_compare.py')
    py = [sys.executable, '-X', 'utf8', cmp_py]
    for n_layers in (2, 4):
        pcb = build(os.path.join(tmp, f'pth{n_layers}'), n_layers)
        r = run_utils.check(py + [run_utils.evidence(pcb, 'fixture board')], accept=True)
        ok(f'{n_layers} layers: the printed channel counts KiCad items and pairs apart',
           f'[#1628: -{n_layers} kicad item(s) / -1 check_drc pair(s)' in r.stdout,
           r.stdout[-700:])
        ok(f'{n_layers} layers: no KICAD-ONLY row', 'KICAD-ONLY' not in r.stdout,
           r.stdout[-700:])

    pcb = build(os.path.join(tmp, 'track'), 2, with_track=True)
    data = kdc.compare_board_data(pcb)
    ok('a real track beside the published pair is still KiCad\'s finding',
       data and data.get('kicad_library_pad_gaps') == 2 and data.get('kicad', 0) >= 1,
       {k: (data or {}).get(k) for k in ('kicad', 'kicad_library_pad_gaps', 'kicad_only')})


def main():
    t_pure()
    tmp = tempfile.mkdtemp(prefix='t1675_')
    try:
        t_kicad(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    if FAILS:
        print(f'FAILED: {len(FAILS)}: {FAILS}')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
