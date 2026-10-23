#!/usr/bin/env python3
"""A phase-3 tap edge never draws its end connector on a layer the target
pad does not have.

_path_to_segments_vias draws the connector from the path's last cell to the
target's exact point on the path's end layer only when that IS the
original's layer, or when _terminal_copper_on_layer finds the net's copper
under the target on it (#505). Every caller passes the endpoint's OWN layer
-- except _route_multipoint_taps_impl, which passed the layer the path ended
on, so the test was vacuous for tap edges. A tap reaching an SMD pad through
a #189/#535 unblock via (in the pad off its centre, or off the pad with its
escape stub) ends on the via's other layer, and the connector was drawn
there to the centre of a pad with no copper on it: dangling, obstacle-exempt
copper.

Found on glasgow_revC (set1, step5_route, phase-3 taps): GND In1.Cu
(86.70,91.90)-(86.785,92.50) ending at C24.2, a B.Cu-only pad. On main the
same step ships GND B.Cu (95.20,105.40)-(95.15,105.30) ending at U25.2, an
F.Cu pad whose unblock via sits 0.15 mm off its centre.

Cases (an F.Cu pad at (10,10), a tap path ending on B.Cu at (10.0,10.2)):
  * the tap caller judges the connector on the target's layer (AST);
  * unblock via off the centre -> no B.Cu connector to the pad centre;
  * a same-net via covering the centre -> the connector stays (it lands on
    the barrel);
  * a plated through-hole target -> the connector stays.

    python3 tests/test_tap_end_connector_layer.py
"""
import ast
import contextlib
import io
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

import single_ended_routing as ser  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402
from routing_config import GridRouteConfig, GridCoord  # noqa: E402

fails = []


def check(name, cond, detail=''):
    print(("  ok  " if cond else " FAIL ") + name + (f"  {detail}" if detail else ''))
    if not cond:
        fails.append(name)


# 1. The tap caller builds its end_original through _tap_target_layer.
src = open(os.path.join(ROOT, 'py_router', 'single_ended_routing.py'),
           encoding='utf-8').read()
fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef)
          and n.name == '_route_multipoint_taps_impl')
pad_end = [n for n in ast.walk(fn) if isinstance(n, ast.Assign)
           and any(isinstance(t, ast.Name) and t.id == 'end_original' for t in n.targets)
           and isinstance(n.value, ast.Tuple)
           and ast.unparse(n.value.elts[0]) == 'tgt_x']
check("the tap edge's pad-target connector is judged on the target's layer",
      len(pad_end) == 1 and isinstance(pad_end[0].value.elts[2], ast.Call)
      and ast.unparse(pad_end[0].value.elts[2].func) == '_tap_target_layer',
      ast.unparse(pad_end[0].value) if pad_end else 'assignment not found')

# What the tap caller passes as the original's layer. On main there is no
# helper: the caller passes the path's own layer, which is what runs here.
layer_fn = getattr(ser, '_tap_target_layer', None) or \
    (lambda tgt_pad, names, path_layer: path_layer)


def board(pad, via=None):
    v = (f' (via (at {via[0]} {via[1]}) (size {via[2]}) (drill {via[3]}) '
         f'(layers "F.Cu" "B.Cu") (net 1) (uuid "v"))') if via else ''
    return f'''(kicad_pcb (version 20240108) (generator test)
 (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (44 "Edge.Cuts" user))
 (net 0 "")
 (net 1 "GND")
 (gr_rect (start 0 0) (end 20 20) (layer "Edge.Cuts") (width 0.1))
 (footprint "t:c" (layer "F.Cu") (at 10 10)
  (property "Reference" "C1" (at 0 0) (layer "F.SilkS"))
  {pad})
{v}
)'''


SMD = '(pad "2" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu" "F.Mask") (net 1 "GND"))'
PTH = ('(pad "2" thru_hole circle (at 0 0) (size 0.9 0.9) (drill 0.5) '
       '(layers "*.Cu" "*.Mask") (net 1 "GND"))')


def connectors(pad_text, via):
    """Segments the tap emission draws for a path ending on B.Cu at
    (10.0, 10.2), the target being the pad at (10, 10)."""
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, 'b.kicad_pcb')
        with open(p, 'w', encoding='utf-8') as f:
            f.write(board(pad_text, via))
        pcb = parse_kicad_pcb(p)
    cfg = GridRouteConfig(layers=['F.Cu', 'B.Cu'], grid_step=0.05, clearance=0.1,
                          track_width=0.2, via_size=0.25, via_drill=0.15)
    coord = GridCoord(cfg.grid_step)
    pad = pcb.footprints['C1'].pads[0]
    gx, gy = coord.to_grid(10.0, 10.0)
    tgt_pad = (gx, gy, 0, 10.0, 10.0, pad)       # pad_info: layer 0 = F.Cu
    path = [coord.to_grid(9.0, 10.2) + (1,), coord.to_grid(10.0, 10.2) + (1,)]
    end_original = (10.0, 10.0, layer_fn(tgt_pad, cfg.layers, 'B.Cu'))
    with contextlib.redirect_stdout(io.StringIO()):
        segs, _v = ser._path_to_segments_vias(path, coord, cfg.layers, 1, cfg,
                                              None, end_original, set(), pcb)
    return [s for s in segs if s.layer == 'B.Cu'
            and any(math.hypot(x - 10.0, y - 10.0) < 1e-6
                    for x, y in ((s.start_x, s.start_y), (s.end_x, s.end_y)))]


# 2. unblock via 0.15 mm off the pad centre (radius 0.125: not over it)
c = connectors(SMD, (10.0, 10.15, 0.25, 0.15))
check("off-centre unblock via: no B.Cu connector to the F.Cu pad's centre",
      not c, f"{len(c)} drawn")
# 3. a via over the centre: the connector lands on its barrel
check("via over the pad centre: the connector stays",
      len(connectors(SMD, (10.0, 10.0, 0.25, 0.15))) == 1)
# 4. plated through-hole target: copper on every layer
check("through-hole target: the connector stays",
      len(connectors(PTH, None)) == 1)

print("-" * 60)
if fails:
    print(f"{len(fails)} FAILED: " + "; ".join(fails))
    sys.exit(1)
print("ALL PASS")
