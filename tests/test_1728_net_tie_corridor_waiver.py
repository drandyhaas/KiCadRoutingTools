#!/usr/bin/env python3
"""#1728: the net-tie corridor lifts only what KiCad's net-tie waiver covers.

A Kelvin-shunt sense net may touch its net-tie PARTNER pad only where the
contact lies on the net's OWN pad (DRC_ENGINE::IsNetTieExclusion; check_drc's
_net_tie_span_waived). obstacle_map._compute_net_tie_corridors used to make a
cell legal for the tied net whenever its copper reached no partner copper off
the own pad -- all the way round the partner, at track half-width + 0.01 mm --
so the search rode the partner pad's side sub-clearance and KiCad graded
every such segment a clearance violation.

On a fixture (shunt R1: VBUS pad 1 with the SENSE pad 2 on its top edge,
tied by net_tie_pad_groups; SENSE's far pad R2.1 to the south-west, so the
shortest path runs down the partner's side):

  1. The SENSE map: a cell beside the partner's side, sub-clearance and off
     the own pad, is BLOCKED; the own-pad sliver and a cell just past the
     partner's top edge inside the own pad's span (the exit KiCad waives)
     stay open; the partner's centre stays blocked.
  2. route.py routes SENSE and the board grades clean with check_drc at the
     route's clearance (pad-segment against VBUS included), and connected.

    python3 tests/test_1728_net_tie_corridor_waiver.py
"""
import contextlib
import io
import os
import sys
import tempfile

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(TESTS_DIR)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, TESTS_DIR)

from run_utils import check as run_check, evidence             # noqa: E402

# R1: VBUS pad 1 spans x 9.525..10.475, y 9.925..10.725; SENSE pad 2 spans
# x 9.75..10.25, y 9.925..10.075, on pad 1's top edge.
BOARD = """(kicad_pcb (version 20241229) (generator "pcbnew")
  (general (thickness 1.6))
  (layers
    (0 "F.Cu" signal)
    (2 "B.Cu" signal)
    (25 "Edge.Cuts" user)
  )
  (setup (pad_to_mask_clearance 0))
  (net 0 "")
  (net 1 "VBUS")
  (net 2 "SENSE")
  (footprint "test:kelvin_shunt" (layer "F.Cu")
    (at 10 10)
    (property "Reference" "R1" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (net_tie_pad_groups "1, 2")
    (pad "1" smd rect (at 0 0.325) (size 0.95 0.8) (layers "F.Cu") (net 1 "VBUS"))
    (pad "2" smd rect (at 0 0) (size 0.5 0.15) (layers "F.Cu") (net 2 "SENSE"))
  )
  (footprint "test:res" (layer "F.Cu")
    (at 9.0 13)
    (property "Reference" "R2" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))
    (pad "1" smd rect (at 0 0) (size 0.5 0.5) (layers "F.Cu") (net 2 "SENSE"))
  )
  (gr_rect (start 0 0) (end 30 20) (layer "Edge.Cuts") (stroke (width 0.1) (type solid)))
)
"""
WIDTH, CLEARANCE, GRID = 0.1, 0.1, 0.05
failures = []


def check(label, ok, detail=''):
    print(f"  [{'ok' if ok else 'FAIL'}] {label}{(': ' + detail) if detail else ''}")
    if not ok:
        failures.append(label)


def write_board(tmp):
    path = os.path.join(tmp, 'tie.kicad_pcb')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(BOARD)
    return evidence(path)


def map_cells(board):
    from kicad_parser import parse_kicad_pcb
    from obstacle_map import build_base_obstacle_map
    from routing_config import GridRouteConfig, GridCoord
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(board)
    sense = next(i for i, n in pcb.nets.items() if n.name == 'SENSE')
    cfg = GridRouteConfig(grid_step=GRID, clearance=CLEARANCE, track_width=WIDTH,
                          via_size=0.45, via_drill=0.2, layers=['F.Cu', 'B.Cu'])
    coord = GridCoord(GRID)
    obs = build_base_obstacle_map(pcb, cfg, [sense])

    def open_(x, y):
        g = coord.to_grid(x, y)
        return not obs.is_blocked(g[0], g[1], 0)
    print("1. the SENSE obstacle map")
    # 0.05 + 0.025 off the partner's left edge (x 9.525): the copper would sit
    # 0.025 mm from VBUS, on its side, nowhere near the own pad.
    check("beside the partner's side, sub-clearance: blocked", not open_(9.45, 10.4))
    check("the own-pad sliver: open", open_(10.0, 10.0))
    check("just past the partner's top edge, inside the own pad's span: open",
          open_(10.0, 9.85))
    check("the partner's centre: blocked", not open_(10.0, 10.4))


def routed(tmp, board):
    print("2. route.py, then the grade")
    out = os.path.join(tmp, 'out.kicad_pcb')
    r = run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route.py'),
                   board, out, '--nets', 'SENSE', '--track-width', str(WIDTH),
                   '--clearance', str(CLEARANCE), '--grid-step', str(GRID),
                   '--via-size', '0.45', '--via-drill', '0.2'], accept=True)
    check("SENSE routed", 'Single-ended:  1/1 routed' in r.stdout,
          next((l for l in r.stdout.splitlines() if 'Single-ended:' in l), r.stdout[-300:]))
    if not os.path.isfile(out):
        check("an output board", False)
        return
    d = run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_drc.py'),
                   out, '--clearance', str(CLEARANCE)], accept=True)
    check("check_drc: clean", 'NO DRC VIOLATIONS FOUND' in d.stdout,
          ' | '.join(l.strip() for l in d.stdout.splitlines() if 'VBUS' in l)[:300])
    c = run_check([sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'check_connected.py'),
                   out, '--nets', 'SENSE'], accept=True)
    check("check_connected: SENSE connected", c.returncode == 0, c.stdout[-200:])


def main():
    with tempfile.TemporaryDirectory(prefix='t1728_') as tmp:
        board = write_board(tmp)
        map_cells(board)
        routed(tmp, board)
    if failures:
        print(f"\nFAIL: {len(failures)} check(s): {failures}")
        return 1
    print("\nPASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
