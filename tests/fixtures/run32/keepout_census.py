"""READ-ONLY census (run32; repro for #1031): pads whose copper reaches the board's
rule-area keepout band on a copper layer the pad lives on.

A keepout with tracks_allowed/vias_allowed False forbids every track and via
inside it on its layers, so an SMD pad sitting there cannot be reached on that
layer at all. A plated through-hole pad is still reachable on the layers the
keepout does not cover, and is listed separately as 'th'.

Pad copper is modelled as its rectangle (size_x/size_y, rect_rotation), which
over-covers round and oval pads -- conservative for this question.

Usage: python3 -X utf8 tests/fixtures/run32/keepout_census.py BOARD [--margin MM]
  margin: required distance between pad copper and the band (default 0.2 +
  0.0635, the board clearance plus half the 0.127 track). Writes nothing.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'py_router'))

from shapely.geometry import Polygon  # noqa: E402
from kicad_parser import parse_kicad_pcb  # noqa: E402


def pad_poly(pad):
    hx, hy = pad.size_x / 2.0, pad.size_y / 2.0
    a = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
    ca, sa = math.cos(a), math.sin(a)
    pts = []
    for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
        pts.append((pad.global_x + dx * ca - dy * sa,
                    pad.global_y + dx * sa + dy * ca))
    return Polygon(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('--margin', type=float, default=0.2 + 0.0635)
    a = ap.parse_args()
    pcb = parse_kicad_pcb(a.board)
    rows = []
    for ko in pcb.board_info.keepouts:
        if ko.get('tracks_allowed', True) and ko.get('vias_allowed', True):
            continue
        interior = Polygon(ko['polygon'], ko.get('holes') or [])
        # the band is the keepout area itself; allowed region = its holes
        allowed = None
        for h in ko.get('holes') or []:
            hp = Polygon(h)
            allowed = hp if allowed is None else allowed.union(hp)
        layers = set(ko.get('layers') or [])
        for ref, fp in pcb.footprints.items():
            for pad in fp.pads:
                if pad.pad_type == 'np_thru_hole':
                    continue
                pl = set(pad.layers)
                if '*.Cu' in pl:
                    pl |= {'F.Cu', 'B.Cu'}
                hit = layers & pl
                if not hit:
                    continue
                poly = pad_poly(pad)
                if not poly.intersects(Polygon(ko['polygon'])):
                    continue  # pad not over this keepout's outer extent
                inside = allowed is not None and allowed.contains(poly)
                gap = allowed.exterior.distance(poly) if inside else -1.0
                if inside and gap >= a.margin:
                    continue
                kind = 'th' if (pad.drill or 0) > 0 else 'smd'
                rows.append((ref, pad.pad_number, pad.net_name or '', kind,
                             round(pad.global_x, 3), round(pad.global_y, 3),
                             'inside-band' if not inside else 'near-band',
                             round(gap, 3), sorted(hit)))
    rows.sort()
    sig = [r for r in rows if r[3] == 'smd' and r[2] and r[2] != 'GND']
    print(f'board {a.board}  margin {a.margin:.4f} mm')
    print(f'pads reaching or within margin of the keepout band: {len(rows)}; '
          f'SMD signal (non-GND, netted): {len(sig)}')
    for r in rows:
        print('  ', r)
    return 0


if __name__ == '__main__':
    sys.exit(main())
