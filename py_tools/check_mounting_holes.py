#!/usr/bin/env python3
"""Report other-net copper inside the screw-head ring of each mounting hole (#1442).

KiCad checks a mounting hole against the ordinary hole and pad clearance only.
The screw head, washer, nut or standoff that sits on the hole is not in the
board file, so a track laid right up to the hole passes every DRC while the
head lands on it. A mounting hole is a footprint that is ONE round hole: at
most 9 pads, the widest a round drill of 2.0 mm or wider, every other pad that
hole drawn again (front and back pads) or a small plated stitching via inside
its pad, none on a signal net (see ``mounting_holes``; a milled slot or a part
of several holes, like a keyboard stabilizer, is not one); the
ring reaches 0.4 x the drill past the hole's edge -- about an ISO 7380 head,
for every metric size -- or ``--ring MM``. Copper of the hole's own net (a
ground ring under a plated hole) is the designer's choice and is not reported.

Exit 0 when every ring is clear, 1 when any holds another net's copper. To
keep it clear while routing, write the ring as a keep-out first:

    python3 py_router/add_rule_area.py in.kicad_pcb out.kicad_pcb \\
        --name MOUNT --mounting-holes

Usage:
    python3 py_tools/check_mounting_holes.py board.kicad_pcb [--ring MM]
"""
from __future__ import annotations

#: #937 registry: which door(s) show this tool, and whether it changes
#: the board. Read by krt_registry.py -- by AST, never imported.
KRT_TOOL = {'scope': ['routing', 'placement', 'combined'], 'kind': 'instrument'}

import _path  # noqa: F401  (#522: makes ../py_router importable)

import argparse
import contextlib
import io
import sys


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split('\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument('input_file', help='the .kicad_pcb to read')
    ap.add_argument('--ring', type=float, default=None, metavar='MM',
                    help='how far past the hole edge the head reaches '
                         '(default: 0.4 x the drill)')
    args = ap.parse_args(argv)

    from kicad_parser import parse_kicad_pcb
    from mounting_holes import hole_copper
    with contextlib.redirect_stdout(io.StringIO()):
        pcb = parse_kicad_pcb(args.input_file)
    rows = hole_copper(pcb, args.ring)
    if not rows:
        print("No mounting holes on this board.")
        return 0
    bad = 0
    for hole, hits in rows:
        ring = hole.ring(args.ring)
        head = (f"{hole.ref}: drill {hole.drill:g}, head ring {ring:.2f} mm past "
                f"the edge")
        if not hits:
            print(f"  ok    {head}")
            continue
        bad += 1
        nets = sorted({h['net'] for h in hits})
        print(f"  FAIL  {head}: {len(hits)} item(s) of {', '.join(nets[:5])}"
              f"{', ...' if len(nets) > 5 else ''}")
        for h in hits[:5]:
            print(f"          {h['kind']} {h['net']} on {h['layer']}: "
                  f"{h['gap']:.2f} mm from the hole edge")
    if bad:
        print(f"MOUNTING HOLE COPPER: {bad} of {len(rows)} hole(s) have another "
              f"net's copper under the screw head")
        return 1
    print(f"All {len(rows)} mounting hole ring(s) clear.")
    return 0


if __name__ == '__main__':
    import cli_banner
    cli_banner.install()
    sys.exit(main())
