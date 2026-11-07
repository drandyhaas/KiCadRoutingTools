#!/usr/bin/env python3
"""#1683: a stub layer switch's pad via stays out of a foreign filled graphic.

`via_barrel_clear_of_foreign_copper` is the layer switch's foreign-copper test
and it measured to a graphic's perimeter strokes, so a through via at a pad over
the middle of another net's filled rect touched none of them. route.py shipped
the short KiCad reports as `shorting_items`.

The fixture: an F.Cu pad of SIG over a filled B.Cu rect of GND, with a stub on
F.Cu running out of the rect.

Checks:
  1. The barrel test, and through it `fitting_pad_via`, refuse the via for SIG
     at every size on the ladder, naming the graphic.
  2. GND's own via stands there: own-net copper stays allowed.
  3. `(fill no)`: the same via is laid (the control).
  4. A swap partner's shape refuses too: a graphic does not move with the swap.
  5. The swap-via funnel (`_bare_pad_pair_vias_fit`) refuses it too.
  6. A real switch: `validate_single_swap` turns it down, and the switch is not
     made, where with `(fill no)` it is.

    python3 tests/test_1683_stub_switch_filled_graphic.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb, Via             # noqa: E402
from routing_config import GridRouteConfig                # noqa: E402
from stub_layer_switching import (fitting_pad_via, get_stub_info,  # noqa: E402
                                  validate_single_swap, foreign_stub_registry,
                                  via_barrel_clear_of_foreign_copper)
from layer_swap_optimization import _bare_pad_pair_vias_fit  # noqa: E402

failures = []


def ok(name, cond, detail=''):
    print(f"  {'PASS' if cond else 'FAIL'}: {name}" + (f' -- {detail}' if detail else ''))
    if not cond:
        failures.append(name)


def board(fill):
    return f'''(kicad_pcb (version 20260206) (generator test)
 (layers (0 "F.Cu" signal) (4 "In1.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
 (net 0 "")
 (net 1 "GND")
 (net 2 "SIG")
 (gr_rect (start 0 0) (end 40 40) (stroke (width 0.1) (type default))
  (fill no) (layer "Edge.Cuts"))
 (gr_rect (start 6 6) (end 16 16) (stroke (width 0.2) (type default))
  (fill {fill}) (layer "B.Cu") (net "GND"))
 (footprint "t:r" (layer "F.Cu") (at 11 11)
  (property "Reference" "R1" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1.2 1.2) (layers "F.Cu" "F.Mask") (net 2 "SIG"))
  (pad "2" smd rect (at 3 0) (size 1.2 1.2) (layers "F.Cu" "F.Mask") (net 1 "GND")))
 (footprint "t:r" (layer "F.Cu") (at 30 30)
  (property "Reference" "R2" (at 0 0) (layer "F.SilkS"))
  (pad "1" smd rect (at 0 0) (size 1.2 1.2) (layers "F.Cu" "F.Mask") (net 2 "SIG")))
 (segment (start 11 11) (end 11 22) (width 0.2) (layer "F.Cu") (net 2))
)
'''


def parsed(td, name, fill):
    path = os.path.join(td, name + '.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(board(fill))
    with contextlib.redirect_stdout(io.StringIO()):
        return parse_kicad_pcb(path)


CFG = GridRouteConfig(layers=['F.Cu', 'In1.Cu', 'B.Cu'], grid_step=0.05, clearance=0.1,
                      track_width=0.2, via_size=0.45, via_drill=0.3)
SIG, GND = 2, 1
X, Y = 11.0, 11.0


def main():
    print('#1683: a stub layer switch stays out of a foreign filled graphic')
    with tempfile.TemporaryDirectory() as td:
        pcb = parsed(td, 'yes', 'yes')
        clear, why = via_barrel_clear_of_foreign_copper(X, Y, SIG, pcb, CFG, set())
        ok('the barrel test refuses SIG at the pad', not clear and 'filled' in why, why)
        with contextlib.redirect_stdout(io.StringIO()):
            fit = fitting_pad_via(X, Y, SIG, pcb, CFG, set())
        ok('no size on the ladder fits', fit is None, f'{fit}')
        clear, why = via_barrel_clear_of_foreign_copper(X + 3, Y, GND, pcb, CFG, set())
        ok("GND's own via stands there", clear, why)
        clear, why = via_barrel_clear_of_foreign_copper(X, Y, SIG, pcb, CFG, {GND})
        ok('a swap partner does not take a graphic with it', not clear, why)

        v = Via(x=X, y=Y, size=0.45, drill=0.3, layers=['F.Cu', 'B.Cu'], net_id=SIG)
        fit_ok, why = _bare_pad_pair_vias_fit(pcb, [v], CFG)
        ok('the swap-via funnel refuses it', not fit_ok and 'filled' in why, why)

        stub = get_stub_info(pcb, SIG, 11.0, 22.0, 'F.Cu')
        ok('the fixture has its stub', stub is not None)
        swap_ok, why = validate_single_swap(stub, 'In1.Cu', foreign_stub_registry(pcb, SIG), pcb, CFG)
        ok('the switch is not made', not swap_ok and 'filled' in why, why)

        ctl = parsed(td, 'no', 'no')
        clear, why = via_barrel_clear_of_foreign_copper(X, Y, SIG, ctl, CFG, set())
        ok('control: (fill no) leaves the via clear', clear, why)
        stub = get_stub_info(ctl, SIG, 11.0, 22.0, 'F.Cu')
        swap_ok, why = validate_single_swap(stub, 'In1.Cu', foreign_stub_registry(ctl, SIG), ctl, CFG)
        ok('control: (fill no) makes the switch', swap_ok, why)
    print('FAILED: ' + '; '.join(failures) if failures else 'All checks passed')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
