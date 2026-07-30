#!/usr/bin/env python3
"""gh#74: post-route smoothing collapses staircase jogs without breaking
clearance, connectivity anchors, or width groups.

    python3 tests/test_smooth_route.py
"""
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "rust_router"))

from kicad_parser import parse_kicad_pcb

HEAD = """(kicad_pcb (version 20241229) (generator "test")
  (general (thickness 1.6))
  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (44 "Edge.Cuts" user))
  (net 0 "")
  (net 1 "STAIR")
  (net 2 "NEIGHBOR")
  (net 3 "TAPPED")
"""


def staircase(x0, y0, steps, net, width=0.1):
    """0.05-grid staircase from (x0,y0) heading +x/+y."""
    out = []
    x, y = x0, y0
    for _ in range(steps):
        out.append((x, y, x + 0.05, y, width, net))
        x += 0.05
        out.append((x, y, x, y + 0.05, width, net))
        y += 0.05
    return out


def seg_line(x1, y1, x2, y2, w, net):
    return ('  (segment (start %g %g) (end %g %g) (width %g) '
            '(layer "F.Cu") (net %d))\n' % (x1, y1, x2, y2, w, net))


def total_len(pcb, net_id):
    return sum(math.hypot(s.end_x - s.start_x, s.end_y - s.start_y)
               for s in pcb.segments if s.net_id == net_id)


def main():
    body = HEAD
    # net 1: 20-step staircase (should collapse to ~one diagonal)
    for seg in staircase(10, 10, 20, 1):
        body += seg_line(*seg)
    # net 2: straight neighbor parallel to the staircase envelope, 0.25mm
    # below its lowest point -- collapse must keep clearance to it.
    body += seg_line(10.0, 9.75, 11.0, 9.75, 0.1, 2)
    # net 3: trunk with a mid-span tap (soft joint) -- tap point must survive
    body += seg_line(20, 20, 20.4, 20.4, 0.1, 3)
    body += seg_line(20.4, 20.4, 20.8, 20.4, 0.1, 3)
    body += seg_line(20.8, 20.4, 21.2, 20.8, 0.1, 3)
    body += seg_line(20.8, 20.4, 20.8, 21.5, 0.1, 3)  # tap at junction
    # anchor every free end with a via so --trim-dangles keeps the nets
    def via_line(x, y, net):
        return ('  (via (at %g %g) (size 0.3) (drill 0.2) '
                '(layers "F.Cu" "B.Cu") (net %d))\n' % (x, y, net))
    body += via_line(10, 10, 1)
    body += via_line(11, 11, 1)          # staircase end (20 steps of 0.05)
    body += via_line(10.0, 9.75, 2)
    body += via_line(11.0, 9.75, 2)
    body += via_line(20, 20, 3)
    body += via_line(21.2, 20.8, 3)
    body += via_line(20.8, 21.5, 3)

    # B.Cu counter-copper: a via touching copper on ONE layer only is itself
    # dangling and correctly trimmed -- anchor vias must connect two layers.
    def bseg(x, y, net):
        # short enough that BOTH ends stay inside the via copper (r=0.15)
        return ('  (segment (start %g %g) (end %g %g) (width 0.1) '
                '(layer "B.Cu") (net %d))\n' % (x, y, x + 0.1, y, net))
    for (bx, by, bnet) in ((10, 10, 1), (11, 11, 1), (10.0, 9.75, 2),
                           (11.0, 9.75, 2), (20, 20, 3), (21.2, 20.8, 3),
                           (20.8, 21.5, 3)):
        body += bseg(bx, by, bnet)
    body += ")\n"

    with tempfile.NamedTemporaryFile("w", suffix=".kicad_pcb",
                                     delete=False) as f:
        f.write(body)
        src = f.name
    out = src.replace(".kicad_pcb", "_s.kicad_pcb")
    try:
        r = subprocess.run(
            [sys.executable, os.path.join(HERE, "smooth_route.py"), src,
             "--output", out, "--trim-dangles"],
            capture_output=True, text=True)
        assert r.returncode == 0, r.stderr[-500:]
        pcb = parse_kicad_pcb(out)

        # staircase collapsed: strictly shorter, way fewer segments
        n1 = [s for s in pcb.segments if s.net_id == 1]
        l1 = total_len(pcb, 1)
        assert len(n1) < 40, f"staircase not collapsed: {len(n1)} segs"
        assert l1 < 2.0 * 0.99, f"no length gain: {l1}"

        # clearance to the neighbor honored: nearest approach >= 0.1 + halves
        from geometry_utils import segment_to_segment_distance
        n2 = [s for s in pcb.segments if s.net_id == 2][0]
        dmin = min(segment_to_segment_distance(
            s.start_x, s.start_y, s.end_x, s.end_y,
            n2.start_x, n2.start_y, n2.end_x, n2.end_y) for s in n1)
        assert dmin >= 0.1 + 0.05 + 0.05 - 1e-6, f"clearance violated: {dmin}"

        # tap junction survived: some net-3 segment still ends at the tap
        n3_pts = set()
        for s in pcb.segments:
            if s.net_id == 3:
                n3_pts.add((round(s.start_x, 3), round(s.start_y, 3)))
                n3_pts.add((round(s.end_x, 3), round(s.end_y, 3)))
        assert (20.8, 20.4) in n3_pts, "soft-joint tap vertex removed"
        assert (20.8, 21.5) in n3_pts, "tap stub lost"

        print("OK: gh#74 smoothing collapses jogs, honors clearance, "
              "keeps taps")
    finally:
        os.unlink(src)
        if os.path.exists(out):
            os.unlink(out)


if __name__ == "__main__":
    main()
