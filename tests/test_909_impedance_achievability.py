#!/usr/bin/env python3
"""#909: a board with no stackup should STATE the impedance numbers, not skip.

`route.py` printed one line on a stackup-less board -- "No stackup found in PCB
file. Using fixed track width." -- which is true and tells the reader nothing
about whether authoring a stackup would have helped. The repo's own solvers
answer that in one call against a NOMINAL FR4 stack.

Invariants gated here:
  1. `nominal_stackup` is a usable stackup: copper/dielectric alternating,
     summing to the declared board thickness, and it scales to N layers.
  2. `achievability_note` reproduces the issue's own measured figures on
     esp_prog: 90 ohm differential at gap 0.15 on 2-layer 1.6mm FR4 needs a
     1.133mm leg / 2.42mm channel.
  3. It is SILENT on a board that HAS a stackup -- there the real solver runs
     and this has nothing to add.
  4. It names its assumption; a caller cannot mistake it for a measurement.
  5. `tightest_pin_gap` measures the gap to the nearest OTHER pad of the SAME
     footprint, and returns 0.0 when it cannot answer.
  6. `_impedance_scope_net_ids` resolves globs, and an empty selection means
     every net (what the CLIs mean by it).
  7. NOTHING here writes a stackup: the probe leaves `pcb.board_info.stackup`
     empty. #909 point 2 is explicit about that.

Run:
    python3 tests/test_909_impedance_achievability.py
"""

import contextlib
import io
import os
import sys

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(TESTS_DIR)
sys.path.insert(0, ROOT_DIR)
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_router'))  # #522
sys.path.insert(0, os.path.join(ROOT_DIR, 'py_tools'))  # #522

from kicad_parser import parse_kicad_pcb
from impedance import (achievability_note, nominal_stackup, tightest_pin_gap,
                       _impedance_scope_net_ids, NOMINAL_EPSILON_R)

RUN_ALL_FAST_OK = True


def main():
    fails = []

    def check(name, cond, detail=''):
        print(f"  {'PASS' if cond else 'FAIL'}: {name}"
              + (f"   [{detail}]" if detail and not cond else ''))
        if not cond:
            fails.append(name)

    # --- 1: the synthetic stackup -----------------------------------------
    for n, thick in ((2, 1.6), (4, 1.6), (6, 0.8)):
        st = nominal_stackup(n, thick)
        cu = [l for l in st if l.layer_type == 'copper']
        total = sum(l.thickness for l in st)
        check(f'{n}-layer nominal stack has {n} copper layers',
              len(cu) == n, f'{len(cu)}')
        check(f'{n}-layer nominal stack sums to {thick}mm',
              abs(total - thick) < 1e-6, f'{total}')
        check(f'{n}-layer nominal dielectrics carry er {NOMINAL_EPSILON_R}',
              all(l.epsilon_r == NOMINAL_EPSILON_R for l in st
                  if l.layer_type in ('core', 'prepreg')))

    # --- 2/3/4/7: the note on a real stackup-less board --------------------
    path = os.path.join(ROOT_DIR, 'kicad_files', 'esp_prog.kicad_pcb')
    if not os.path.isfile(path):
        check('esp_prog present', False, path)
    else:
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(path)
        check('esp_prog really declares no stackup (the premise)',
              not pcb.board_info.stackup)
        text, detail = achievability_note(pcb, 'F.Cu', 90.0,
                                          is_differential=True, spacing=0.15,
                                          min_pitch_gap=0.325)
        # The issue measured 1.133mm at zdiff 89.7 and 2.42mm for the pair.
        check('90 ohm differential leg = 1.133mm (the issue\'s own figure)',
              detail and abs(detail['width_mm'] - 1.133) < 0.002,
              f'{detail and detail.get("width_mm")}')
        check('the pair channel = 2.42mm',
              detail and abs(detail['channel_mm'] - 2.42) < 0.005,
              f'{detail and detail.get("channel_mm")}')
        check('the pin-gap ratio is reported',
              detail and abs(detail['channel_over_pin_gap'] - 7.4) < 0.1,
              f'{detail and detail.get("channel_over_pin_gap")}')
        check('the text names the assumption as NOMINAL',
              'NOMINAL' in text and 'er 4.5' in text, text[:80])
        check('the text says the target is not achievable here',
              'Not achievable' in text)
        check('the probe wrote NO stackup onto the board (#909 point 2)',
              not pcb.board_info.stackup)

        # 5: the pin gap, measured on the same board
        nid = next((i for i in pcb.nets if i and pcb.pads_by_net.get(i)), None)
        gap = tightest_pin_gap(pcb, [nid])
        check('tightest_pin_gap returns a positive gap on a real board',
              gap > 0, f'{gap}')
        check('tightest_pin_gap on nothing is 0.0',
              tightest_pin_gap(pcb, []) == 0.0
              and tightest_pin_gap(pcb, None) == 0.0)

        # 6: scope resolution
        allnets = _impedance_scope_net_ids(pcb, None)
        check('an empty selection means every net',
              len(allnets) == len([i for i in pcb.nets if i]),
              f'{len(allnets)}')
        star = _impedance_scope_net_ids(pcb, ['*'])
        check('a "*" glob selects every named net', len(star) > 0)
        none = _impedance_scope_net_ids(pcb, ['/definitely-not-a-net'])
        check('a non-matching pattern selects nothing', none == [])

    # --- 3: a board WITH a stackup gets nothing from this ------------------
    flat = os.path.join(ROOT_DIR, 'kicad_files', 'flat_hierarchy.kicad_pcb')
    if not os.path.isfile(flat):
        check('flat_hierarchy present', False, flat)
    else:
        with contextlib.redirect_stdout(io.StringIO()):
            pcb2 = parse_kicad_pcb(flat)
        check('flat_hierarchy really HAS a stackup (the premise)',
              bool(pcb2.board_info.stackup))
        t2, d2 = achievability_note(pcb2, 'F.Cu', 50.0)
        check('a board with a stackup gets no note (the real solver runs)',
              t2 == '' and d2 is None, f'{t2[:60]}')

    print(f"\n{'FAILED: ' + ', '.join(fails) if fails else 'ALL PASS'}")
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
