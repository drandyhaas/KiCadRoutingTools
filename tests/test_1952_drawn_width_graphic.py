"""#1952: check_drc measures a stroke-0 filled copper graphic as drawn.

A filled shape drawn at stroke 0 (a net tie's bridge, a tab) is parsed with
its perimeter at the track width so the obstacle map has something to stamp;
`Segment.drawn_width` keeps the real 0. Graded at the modelled stroke, the
shape reached 0.15 mm past its own edge, and:

  - a hole that clears the bridge drew phantom `track-hole` rows, one per
    perimeter segment;
  - a foreign via 0.04 mm short of the bridge "touched" it, granted the
    net-tie copper its own net, and its real clearance violation was waived.

The board is #908's net tie (tests/test_908_net_tie_own_copper.py): the
stroke-0 bridge on F.Cu from y 9.9 to 10.1 between the tie pads NT1.1
(TIED_A, x 9.6) and NT1.2 (TIED_B, x 10.4); Default class 0.1 mm,
`min_hole_clearance` 0.1 mm, vias 0.45 / 0.2. One via per board. The
expected rows are kicad-cli 10.0.6's (`pcb drc --all-track-errors`), one row
per pair of items:

  via                          KiCad
  TIED_A, hole 0.15 off        clean
  FOREIGN, copper 0.12 off     clean
  FOREIGN, copper 0.04 off     clearance, Polygon of NT1
  FOREIGN, copper 0.06 off     clearance, Polygon of NT1; clearance, NT1.1
  TIED_A on its tie pad        clean (KiCad's net-tie exclusion)
  TIED_A, hole 0.05 off        hole_clearance, Polygon; clearance, NT1.2
  TIED_A inside the bridge     hole_clearance, Polygon; clearance, NT1.2
  FOREIGN inside the bridge    short, Polygon; hole_clearance, Polygon;
                               clearance, NT1.1 and NT1.2
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import run_utils  # noqa: E402
import test_908_net_tie_own_copper as t908  # noqa: E402

FAILS = []
VIA = ('(via (at {x} {y}) (size 0.45) (drill 0.2) (layers "F.Cu" "B.Cu") '
       '(net {net}) (uuid "bbbbbbbb-0000-4000-8000-000000000009"))')
TIED_A, FOREIGN = 1, 3
POLY = 'Polygon(NT1)'

# tag: (x, y, net, the rows as sorted (type, the other item))
CASES = {
    'tied hole 0.15 off': (9.8, 10.35, TIED_A, []),
    'foreign copper 0.12 off': (10.0, 10.445, FOREIGN, []),
    'foreign copper 0.04 off': (10.0, 10.365, FOREIGN, [('via-segment', POLY)]),
    'foreign copper 0.06 off': (9.8, 10.385, FOREIGN,
                                [('pad-via', 'NT1.1'), ('via-segment', POLY)]),
    'tied on its tie pad': (9.6, 10.0, TIED_A, []),
    'tied hole 0.05 off': (10.0, 10.25, TIED_A,
                           [('pad-via', 'NT1.2'), ('track-hole', POLY)]),
    'tied inside the bridge': (10.0, 10.0, TIED_A,
                               [('pad-via', 'NT1.2'), ('track-hole', POLY)]),
    'foreign inside the bridge': (10.0, 10.0, FOREIGN,
                                  [('pad-via', 'NT1.1'), ('pad-via', 'NT1.2'),
                                   ('track-hole', POLY), ('via-segment', POLY)]),
}


def ok(cond, msg):
    print(f"  [{'ok' if cond else 'FAIL'}] {msg}")
    if not cond:
        FAILS.append(msg)


def stage(td, tag, x, y, net):
    base = os.path.join(td, tag.replace(' ', '_').replace('.', ''))
    board = t908.BOARD % t908._TIE_POLY.format(layer='F.Cu')
    board = board.rstrip()[:-1] + '  ' + VIA.format(x=x, y=y, net=net) + '\n)\n'
    with open(base + '.kicad_pcb', 'w', encoding='utf-8') as fh:
        fh.write(board)
    with open(base + '.kicad_pro', 'w', encoding='utf-8') as fh:
        json.dump({'board': {'design_settings': {'rules': {
            'min_clearance': 0.0, 'min_hole_clearance': 0.1, 'min_hole_to_hole': 0.2}}},
            'net_settings': {'meta': {'version': 4}, 'classes': [
                {'name': 'Default', 'clearance': 0.1, 'track_width': 0.1,
                 'via_diameter': 0.45, 'via_drill': 0.2, 'priority': 2147483647}]},
            'meta': {'version': 3}}, fh)
    return run_utils.evidence(base + '.kicad_pcb')


def rows(board):
    out = board[:-len('.kicad_pcb')] + '.drc.json'
    r = subprocess.run([sys.executable, os.path.join(REPO, 'py_router', 'check_drc.py'),
                        board, '--json', out], capture_output=True, text=True)
    if r.returncode not in (0, 1) or not os.path.isfile(out):
        raise SystemExit(f'BROKEN TEST: check_drc exit {r.returncode}\n{r.stderr[-2000:]}')
    got = []
    for v in json.load(open(out, encoding='utf-8')).get('items', []):
        other = (v.get('item2') or v.get('pad_ref') or
                 (POLY if v.get('net2') == 'net_0' else v.get('net2')))
        got.append((v['type'], other))
    return sorted(got)


def main():
    td = tempfile.mkdtemp(prefix='krt1952_')
    try:
        for tag, (x, y, net, want) in CASES.items():
            got = rows(stage(td, tag, x, y, net))
            ok(got == sorted(want), f'{tag}: {got} == {sorted(want)}')
    finally:
        shutil.rmtree(td, ignore_errors=True)
    if FAILS:
        print(f'FAIL: {len(FAILS)}: ' + '; '.join(FAILS))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
