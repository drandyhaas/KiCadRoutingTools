#!/usr/bin/env python3
"""
Regression tests for three parser/writer text-handling bugs:

  #1436: save_extracted_data (the kicad_parser.py CLI) raised
         "Object of type set is not JSON serializable" on any board with a
         keep-out rule area, whose `layers` is a set.
  #1438: add_tracks_and_vias_to_pcb(remove_vias=...) left `(via blind`,
         `(via micro` and `(via buried` blocks in the output.
  #1440: the segment/via block removers took the quote in an escaped `\\"`
         as a string delimiter, so a net name holding `\\"` and `(`
         swallowed the next block, which was then never removed.

Run:
    python3 tests/test_1436_1438_1440_writer_parser.py
"""

import json
import os
import sys
import tempfile
from types import SimpleNamespace as N

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))

from kicad_parser import parse_kicad_pcb, save_extracted_data
from kicad_writer import (add_tracks_and_vias_to_pcb,
                          remove_segments_from_content,
                          remove_vias_from_content)

KEEPOUT_BOARD = '''(kicad_pcb
\t(version 20240108)
\t(generator "pcbnew")
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(31 "B.Cu" signal)
\t\t(44 "Edge.Cuts" user)
\t)
\t(net 0 "")
\t(gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))
\t(zone
\t\t(net 0)
\t\t(net_name "")
\t\t(layers "F.Cu" "B.Cu")
\t\t(uuid "k1")
\t\t(name "rf")
\t\t(hatch edge 0.5)
\t\t(connect_pads (clearance 0))
\t\t(min_thickness 0.25)
\t\t(keepout (tracks not_allowed) (vias not_allowed) (pads allowed)
\t\t\t(copperpour not_allowed) (footprints allowed))
\t\t(fill (thermal_gap 0.5) (thermal_bridge_width 0.5))
\t\t(polygon
\t\t\t(pts (xy 2 2) (xy 8 2) (xy 8 8) (xy 2 8))
\t\t)
\t)
)
'''


def _seg(net, x):
    return (f'\t(segment\n\t\t(start {x} 0)\n\t\t(end {x + 1} 0)\n'
            f'\t\t(width 0.2)\n\t\t(layer "F.Cu")\n\t\t(net "{net}")\n'
            f'\t\t(uuid "s{x}")\n\t)\n')


def _via(kind, x, net='(net 5)'):
    return (f'\t(via{kind}\n\t\t(at {x} 5)\n\t\t(size 0.6)\n\t\t(drill 0.3)\n'
            f'\t\t(layers "F.Cu" "B.Cu")\n\t\t{net}\n\t\t(uuid "u{x}")\n\t)\n')


def main():
    fails = []

    def check(name, ok):
        print(("  PASS " if ok else "  FAIL ") + name)
        if not ok:
            fails.append(name)

    tmp = tempfile.mkdtemp()

    # --- #1436 -------------------------------------------------------------
    board = os.path.join(tmp, 'keepout.kicad_pcb')
    with open(board, 'w') as f:
        f.write(KEEPOUT_BOARD)
    pcb = parse_kicad_pcb(board)
    check("fixture parses one keep-out with a set of layers",
          len(pcb.board_info.keepouts) == 1
          and isinstance(pcb.board_info.keepouts[0]['layers'], set))
    out_json = os.path.join(tmp, 'keepout.json')
    try:
        save_extracted_data(pcb, out_json)
        data = json.load(open(out_json))
        ko = data['board_info']['keepouts']
        check("#1436 JSON written; keep-out layers a sorted list",
              len(ko) == 1 and ko[0]['layers'] == ['B.Cu', 'F.Cu'])
    except TypeError as e:
        check(f"#1436 JSON written (raised {e})", False)

    # --- #1440 -------------------------------------------------------------
    target = N(start_x=5, start_y=0, end_x=6, end_y=0, layer='F.Cu',
               net_id=2, width=0.2)
    for first in ('A\\"B(', 'A'):
        content = '(kicad_pcb\n' + _seg(first, 0) + _seg('C', 5) + ')\n'
        new, n = remove_segments_from_content(content, [target],
                                              net_id_to_name={2: 'C'})
        check(f"#1440 segment after net {first!r} removed",
              n == 1 and '"C"' not in new and first in new)

    content = ('(kicad_pcb\n' + _via('', 1, '(net "A\\"B(")')
               + _via('', 2, '(net "C")') + ')\n')
    new, n = remove_vias_from_content(
        content, [N(x=2, y=5, net_id=2)], net_id_to_name={2: 'C'})
    check("#1440 via after an escaped-quote net removed",
          n == 1 and '(net "C")' not in new and 'A\\"B(' in new)

    # --- #1438 -------------------------------------------------------------
    inp = os.path.join(tmp, 'vias.kicad_pcb')
    outp = os.path.join(tmp, 'vias_out.kicad_pcb')
    with open(inp, 'w') as f:
        f.write('(kicad_pcb\n\t(version 20240108)\n\t(generator "pcbnew")\n'
                '\t(net 0 "")\n\t(net 5 "A")\n'
                + _via('', 1) + _via(' blind', 2) + _via(' micro', 3)
                + _via(' buried', 4) + _via(' blind', 9) + ')\n')
    add_tracks_and_vias_to_pcb(
        inp, outp, [], [],
        remove_vias=[{'x': x, 'y': 5, 'net_id': 5} for x in (1, 2, 3, 4)])
    out = open(outp).read()
    check("#1438 through/blind/micro/buried vias at listed positions removed",
          all(f'(at {x} 5)' not in out for x in (1, 2, 3, 4)))
    check("#1438 unlisted blind via survives", '(at 9 5)' in out)

    if fails:
        print(f"\nFAIL ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\nAll checks passed")
    return 0


if __name__ == '__main__':
    sys.exit(main())
