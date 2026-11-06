#!/usr/bin/env python3
"""#1688: a graze the micro-shift had no room to clear is retried once smoothing
has straightened the neighbour that left no room.

A 45-degree track on the routing grid runs a few micrometres inside a pad's
clearance. The micro-shift, which runs early in the cleanup, measures that
shortfall correctly but every candidate moves the track into a neighbouring
net's jog, also on the grid, so it refuses them all. Octolinear smoothing runs
later and straightens the jog. Nothing looked at the graze again, so the board
shipped it with the room to clear it standing empty. check_drc and kicad-cli
both report it.

  1. the pipeline with smoothing on: the graze is gone and the jog straightened;
  2. the same with smoothing off: the jog stays, the graze stays, and the shift
     refused it (the mechanism, not just the outcome);
  3. a custom-polygon pad's distance reads the same in the shift and the checker.

    python3 tests/test_1688_microshift_after_smoothing.py
"""
import contextlib
import io
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'py_router'))

from kicad_parser import parse_kicad_pcb               # noqa: E402
from check_drc import run_drc, point_to_pad_distance   # noqa: E402
from cleanup_pipeline import run_post_route_cleanup    # noqa: E402
from pcb_modification import _seg_worst_offender       # noqa: E402
from routing_config import GridRouteConfig             # noqa: E402

FAILS = []
CLR = 0.105
W = 0.105
# The pad's corner a corridor away from the jog, as measured on the board that
# reported this: required 0.1575 from the track's centreline, 0.1515 found.
PAD_CORNER = (21.05495, 28.659325)


def check(name, cond, detail=''):
    print(("  ok   " if cond else "  FAIL ") + name
          + (f"  {detail}" if detail else ''))
    if not cond:
        FAILS.append(name)


def board(path, custom=False):
    px, py = PAD_CORNER
    if custom:
        # A polygon pad whose anchor sits far left of its copper, so the box
        # it declares is not where its copper is.
        pad = f'''(pad "1" smd custom (at {px - 4} {py + 1}) (size 0.5 0.5) (layers "F.Cu") (net "VICTIMPAD")
      (primitives (gr_poly (pts (xy 4 -1) (xy 6 -1) (xy 6 1) (xy 4 1)) (width 0) (fill yes))))'''
        at = ''
    else:
        pad = f'(pad "1" smd rect (at {px + 1} {py + 1}) (size 2 2) (layers "F.Cu") (net "VICTIMPAD"))'
        at = ''
    fp = f'''
  (footprint "T:P" (layer "F.Cu") (at 0 0)
    (property "Reference" "E1" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd){at}
    {pad})'''
    pts = [('V', 20.8, 33.2, 20.8, 28.7), ('V', 20.8, 28.7, 21.8, 27.7),
           ('V', 21.8, 27.7, 22.8, 27.7),
           # the neighbour's jog: one grid cell toward the track, out and back
           ('N', 20.5, 34.0, 20.5, 28.7), ('N', 20.5, 28.7, 20.8, 28.4),
           ('N', 20.8, 28.4, 20.8, 27.0), ('N', 20.8, 27.0, 20.5, 26.7),
           ('N', 20.5, 26.7, 20.5, 22.0)]
    segs = ''.join(f'\n  (segment (start {a} {b}) (end {c} {d}) (width {W}) '
                   f'(layer "F.Cu") (net "{n}"))' for n, a, b, c, d in pts)
    ends = [('R1', 20.8, 33.2, 'V'), ('R2', 22.8, 27.7, 'V'),
            ('R3', 20.5, 34.0, 'N'), ('R4', 20.5, 22.0, 'N')]
    fps = ''.join(f'''
  (footprint "T:Q" (layer "F.Cu") (at {x} {y})
    (property "Reference" "{ref}" (at 0 -1.5 0) (layer "F.SilkS"))
    (attr smd)
    (pad "1" smd rect (at 0 0) (size 0.2 0.2) (layers "F.Cu") (net "{n}")))'''
                  for ref, x, y, n in ends)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'''(kicad_pcb (version 20250114) (generator "pcbnew") (generator_version "10.0")
  (general (thickness 1.6))
  (paper "A4")
  (layers (0 "F.Cu" signal) (2 "B.Cu" signal) (25 "Edge.Cuts" user))
  (setup (pad_to_mask_clearance 0))
  (gr_rect (start 10 15) (end 40 45) (stroke (width 0.1) (type default)) (fill none) (layer "Edge.Cuts")){fp}{fps}{segs}
)
''')


def diag_of(pcb, nid):
    """V's diagonal, wherever the shift left its ends."""
    return [s for s in pcb.segments if s.net_id == nid
            and abs(s.end_x - s.start_x) > 0.5 and abs(s.end_y - s.start_y) > 0.5]


def run(tmp, tag, smooth):
    src = os.path.join(tmp, f'{tag}.kicad_pcb')
    board(src)
    pcb = parse_kicad_pcb(src)
    nets = {n.name: nid for nid, n in pcb.nets.items()}
    cfg = GridRouteConfig(clearance=CLR, track_width=W, via_size=0.45, via_drill=0.3,
                          layers=['F.Cu', 'B.Cu'], grid_step=0.1)
    grazed = [_seg_worst_offender(pcb, nets['V'], s, CLR) for s in diag_of(pcb, nets['V'])]
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        run_post_route_cleanup([], pcb, {nets['V'], nets['N']}, cfg, phantom=False,
                               smooth=smooth, strict_collapse=False)
    left = [o for s in diag_of(pcb, nets['V'])
            for o in [_seg_worst_offender(pcb, nets['V'], s, CLR)] if o]
    jog = any(s.net_id == nets['N'] and abs(s.start_x - 20.8) < 1e-6
              and abs(s.end_x - 20.8) < 1e-6 for s in pcb.segments)
    return grazed, left, jog


def main():
    with tempfile.TemporaryDirectory() as tmp:
        print("1. smoothing on")
        grazed, left, jog = run(tmp, 'on', True)
        check("the input grazes", any(grazed), f"{grazed}")
        check("smoothing straightened the jog", not jog)
        check("the graze is gone", not left, f"{left}")

        print("2. smoothing off: the jog stays and the shift has no room")
        grazed, left, jog = run(tmp, 'off', False)
        check("the jog stays", jog)
        check("the graze stays", bool(left), f"{left}")

        print("3. a custom pad reads the same in the shift and the checker")
        src = os.path.join(tmp, 'custom.kicad_pcb')
        board(src, custom=True)
        pcb = parse_kicad_pcb(src)
        nets = {n.name: nid for nid, n in pcb.nets.items()}
        pad = pcb.footprints['E1'].pads[0]
        s = next(g for g in diag_of(pcb, nets['V']))
        found = _seg_worst_offender(pcb, nets['V'], s, CLR)
        n = 4000
        best = min(point_to_pad_distance(s.start_x + (s.end_x - s.start_x) * i / n,
                                         s.start_y + (s.end_y - s.start_y) * i / n, pad)
                   for i in range(n + 1))
        check("the shift's shortfall is the checker's", found is not None
              and abs((CLR + W / 2 - best) - found[0]) < 5e-4,
              f"shift {found and round(found[0], 5)} checker {round(CLR + W / 2 - best, 5)}")
        v = run_drc(src, clearance=CLR, quiet=True, print_summary=False)
        hit = [x for x in v if x.get('type') == 'pad-segment']
        check("check_drc reports that track", len(hit) == 1, f"{len(hit)}")

    if FAILS:
        print(f"FAILED: {len(FAILS)}")
        return 1
    print("PASS")
    return 0


if __name__ == '__main__':
    sys.exit(main())
