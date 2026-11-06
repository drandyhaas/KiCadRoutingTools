#!/usr/bin/env python3
"""#1689: ripping a pre-existing net must leave its net-tagged copper art on the board.

route.py registers each net it may rip as a result carrying the net's segments,
and a rip removes exactly those from the board model. They included the net's
copper GRAPHICS, so after the rip the art was gone for the rest of the run: no
obstacle stamped it, no clearance gate measured it. Nothing lays art again and
the writer leaves it in the file, so a later route crossed copper that ships.

  1. end to end: a net whose only way across is through a pre-existing net rips
     it (asserted from the run's own line), and its track does not cross the
     art that net carried;
  2. the partial rip: the branch chosen to rip never contains art.

    python3 tests/test_1689_rip_keeps_art.py
"""
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'tests'))

from kicad_parser import parse_kicad_pcb      # noqa: E402
from check_drc import run_drc                 # noqa: E402
from leg_rip import select_blocking_branch    # noqa: E402
from routing_config import GridRouteConfig    # noqa: E402
from run_utils import evidence                # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print(("  ok   " if cond else "  FAIL ") + name
          + (f"  {detail}" if detail else ''))
    if not cond:
        FAILS.append(name)


def board(path):
    """Net WALL runs top to bottom across a 20x20 board; net B has a pad on
    each side of it. WALL carries a stroke of copper art across B's straight
    line, standing clear of WALL's track and touching nothing."""
    pads = [('R1', 15.0, 20.5, 'WALL'), ('R2', 15.0, 39.5, 'WALL'),
            ('R3', 8.0, 30.0, 'B'), ('R4', 22.0, 30.0, 'B')]
    fps = ''.join(f'''
  (footprint "T:P" (layer "F.Cu") (at {x} {y})
    (property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.6 0.6) (layers "F.Cu") (net "{n}")))'''
                  for ref, x, y, n in pads)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 5 20) (end 25 40) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts"))
  (gr_line (start 19 28.5) (end 19 31.5) (stroke (width 0.2) (type default)) (layer "F.Cu") (net "WALL")){fps}
  (segment (start 15 20.5) (end 15 39.5) (width 0.2) (layer "F.Cu") (net "WALL"))
)
''')


def main():
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, 'in.kicad_pcb')
        out = os.path.join(tmp, 'out.kicad_pcb')
        board(src)

        print("1. the wall is ripped; the art it carried stays an obstacle")
        r = subprocess.run(
            [sys.executable, '-X', 'utf8', os.path.join(ROOT, 'py_router', 'route.py'),
             src, out, '--nets', 'B', '--rip-existing-nets', 'WALL', '--layers', 'F.Cu',
             '--track-width', '0.2', '--clearance', '0.2', '--grid-step', '0.1'],
            capture_output=True, text=True, errors='replace')
        evidence(out)
        check("the run ripped the pre-existing net",
              "PRE-EXISTING rip: 'WALL'" in r.stdout,
              '' if "PRE-EXISTING rip" in r.stdout else r.stdout[-600:])
        v = run_drc(out, clearance=0.2, quiet=True, print_summary=False)
        crossed = [x for x in v if x.get('type') == 'segment-crossing'
                   and 'WALL' in (x.get('net1'), x.get('net2'))
                   and 'B' in (x.get('net1'), x.get('net2'))]
        check("net B does not cross the art", not crossed,
              f"{[x.get('cross_point') for x in crossed]}")
        pcb = parse_kicad_pcb(out)
        wall = next(n for n, x in pcb.nets.items() if x.name == 'WALL')
        art = [s for s in pcb.segments if s.net_id == wall and getattr(s, 'graphic', False)]
        check("the art is still in the file", len(art) == 1, f"{len(art)} stroke(s)")

        print("2. a partial rip's branch is never art")
        pcb = parse_kicad_pcb(src)
        wall = next(n for n, x in pcb.nets.items() if x.name == 'WALL')
        cfg = GridRouteConfig(clearance=0.2, track_width=0.2, via_size=0.45,
                              via_drill=0.3, layers=['F.Cu'], grid_step=0.1)
        # Cells over the art alone: with art counted it is a branch of its own,
        # and it would be handed to the rip.
        cells = [(int(19.0 / 0.1), int(30.0 / 0.1), 0)]
        branch = select_blocking_branch(pcb, wall, cells, cfg)
        check("no art in the branch",
              not any(getattr(s, 'graphic', False) for s in (branch or [])),
              f"{[(s.start_x, s.start_y) for s in (branch or [])]}")

    if FAILS:
        print(f"FAILED: {len(FAILS)}")
        return 1
    print("PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
