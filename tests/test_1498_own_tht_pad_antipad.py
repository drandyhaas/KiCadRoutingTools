"""#1498: a trace ending on its net's own through-hole pin crosses that pin's
antipad on every plane, and that is a layer change, not a plane slot.

#1203 exempted the antipad of the net's own via (and its P/N partner's); a
plated through-hole PAD of the net carves the same antipad on every layer its
barrel passes, and was counted as a crossing -- one run per pin, which no
routing can remove, each a `blocking` count in board_score.

Synthetic 4-layer board, GND planes on In1 and In2:
  /PIN    F.Cu into its own THT pin J1.1 (drill 1.0, pad 1.7): exempt;
  /OTHER  F.Cu passing 0.5 mm from J1.2, another net's pin: still a crossing.

    python3 tests/test_1498_own_tht_pad_antipad.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

import check_impedance                                         # noqa: E402
from kicad_parser import parse_kicad_pcb                       # noqa: E402


def zone(net, layer):
    return f"""  (zone (net "{net}") (layer "{layer}") (uuid "z-{layer}")
    (hatch edge 0.5) (connect_pads yes (clearance 0.2)) (min_thickness 0.1)
    (fill yes (thermal_gap 0.5) (thermal_bridge_width 0.5) (island_removal_mode 0))
    (polygon (pts (xy 0.5 0.5) (xy 39.5 0.5) (xy 39.5 29.5) (xy 0.5 29.5))))
"""


def seg(net, x0, y0, x1, y1):
    return (f'  (segment (start {x0} {y0}) (end {x1} {y1}) (width 0.2) '
            f'(layer "F.Cu") (net "{net}"))\n')


BOARD = ("""(kicad_pcb (version 20240108) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (1 "In1.Cu" signal) (2 "In2.Cu" signal)
          (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "GND")
  (net 2 "/PIN")
  (net 3 "/OTHER")
  (net 4 "/J2")
  (gr_rect (start 0 0) (end 40 30) (stroke (width 0.1) (type default))
    (fill none) (layer "Edge.Cuts"))
  (footprint "t:HDR" (layer "F.Cu") (at 20 10)
    (property "Reference" "J1" (at 0 0) (layer "F.SilkS") (hide yes))
    (pad "1" thru_hole circle (at 0 0) (size 1.7 1.7) (drill 1.0) (layers "*.Cu") (net 2 "/PIN"))
    (pad "2" thru_hole circle (at 0 6) (size 1.7 1.7) (drill 1.0) (layers "*.Cu") (net 4 "/J2"))
  )
"""
    + zone('GND', 'In1.Cu') + zone('GND', 'In2.Cu')
    # /PIN: along F.Cu into its own pin at (20, 10)
    + seg('/PIN', 10, 10, 20, 10)
    # /OTHER: past /J2's pin at (20, 16), 0.5 mm off its centre
    + seg('/OTHER', 10, 16.5, 30, 16.5)
    + seg('/J2', 20, 16, 20, 25)
    + ")\n")
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail and not ok else ''}")
    if not ok:
        failures.append(label)


def main():
    with tempfile.TemporaryDirectory(prefix='t1498_') as d:
        path = os.path.join(d, 'b.kicad_pcb')
        open(path, 'w').write(BOARD)
        with contextlib.redirect_stdout(io.StringIO()):
            pcb = parse_kicad_pcb(path)
        rep = check_impedance.analyze_impedance(pcb, ['/PIN', '/OTHER'], board_name='t1498')
    nets = {n.net_name: n for n in rep.nets}
    check('precondition: both nets analysed', {'/PIN', '/OTHER'} <= set(nets), str(sorted(nets)))
    p = nets.get('/PIN')
    check("/PIN: its own through-hole pin's antipad is not a crossing",
          p is not None and not p.crossings and p.own_via_antipad_runs > 0,
          f'crossings {[c.describe() for c in p.crossings]}, runs {p.own_via_antipad_runs}'
          if p else 'missing')
    o = nets.get('/OTHER')
    check("/OTHER: another net's pin antipad keeps its crossing",
          o is not None and o.crossings and not o.own_via_antipad_runs,
          f'crossings {[c.describe() for c in o.crossings]}' if o else 'missing')
    if failures:
        print(f'FAIL: {len(failures)}: ' + '; '.join(failures))
        return 1
    print('ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
