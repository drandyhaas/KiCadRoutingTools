#!/usr/bin/env python3
"""A routed via whose own net's track ENDS ON ITS RIM keeps its net on the
text path, inside a stale fill of another net.

propagate_via_cluster_nets re-nets a via that has "no same-net track end on
it" and sits in exactly one other net's fill, as pcbnew's connectivity pass
does on load. The track-end test was `(ex - x)**2 + (ey - y)**2 <= r*r` in
float. A router writes a track end exactly on a via's rim, at the via radius,
and in float that distance squared lands a few ulps OVER r*r
((39.6 - 39.3)**2 = 0.09000000000000256 > 0.09), so the via was taken as
bare and joined the fill's net. pcbnew holds coordinates in integer nm, where
the end lies on the rim (300000 nm <= 300000 nm) and the track joins the via:
the cluster then holds the track's pad as a second origin and the via keeps
its net.

Seen on a 6-layer board: a router's stage output still carried the GND fill
it was given, an ADC_DIN via at (60.6, 39.3) with its F.Cu track ending at
(60.6, 39.6) parsed as GND, and the grader reported a fully routed ADC_DIN
open. KiCad DRC on the same file: connected.

The fixture is kicad_files/lvds_converter_dualclk_gnd.kicad_pcb (KiCad 10,
B.Cu /GND fill) with one via of a signal net added inside the fill and one
F.Cu track from the via's rim to that net's pad.

    python3 tests/test_via_rim_track_end_renet.py
"""
import os
import re
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'py_router'))
sys.path.insert(0, os.path.join(ROOT, 'py_tools'))

from kicad_parser import parse_kicad_pcb, _pt_in_ring  # noqa: E402

BOARD = os.path.join(ROOT, 'kicad_files', 'lvds_converter_dualclk_gnd.kicad_pcb')
R = 0.3          # via radius (size 0.6)


def _fill_ring(text):
    """The B.Cu /GND zone's first filled_polygon ring."""
    z = text.index('(net "/GND")\n\t\t(layer "B.Cu")')
    f = text.index('(filled_polygon', z)
    body = text[f:text.index('\n\t\t)', f)]
    return [(float(x), float(y)) for x, y in re.findall(r'\(xy ([-\d.]+) ([-\d.]+)\)', body)]


def _pick(pcb, ring):
    """An F.Cu SMD pad of a signal net and a via spot 1.5 mm beside it, in
    the fill, clear of every pad, whose rim point straight up is a few ulps
    outside R in float. Deterministic: first match in pad order."""
    pads = [p for fp in pcb.footprints.values() for p in fp.pads]
    for p in pads:
        if not p.net_id or 'GND' in pcb.nets[p.net_id].name or 'F.Cu' not in p.layers:
            continue
        for dx, dy in ((0, 1.5), (0, -1.5), (1.5, 0), (-1.5, 0)):
            vx, vy = round(p.global_x + dx, 1), round(p.global_y + dy, 1)
            ey = round(vy + R, 1)
            if (ey - vy) ** 2 <= R * R:
                continue            # float happens to round down here
            if not _pt_in_ring(vx, vy, ring):
                continue
            if any(abs(q.global_x - vx) < 1.0 and abs(q.global_y - vy) < 1.0 for q in pads):
                continue
            return p, vx, vy, ey
    raise AssertionError('fixture: no pad/via spot found')


def main():
    text = open(BOARD, encoding='utf-8').read()
    ring = _fill_ring(text)
    base = parse_kicad_pcb(BOARD)
    pad, vx, vy, ey = _pick(base, ring)
    net = pcb_name = base.nets[pad.net_id].name
    raw = net.replace('\\', '\\\\').replace('"', '\\"')
    add = (f'\t(via (at {vx} {vy}) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") '
           f'(net "{raw}") (uuid "00000000-0000-0000-0000-00000000c0de"))\n'
           f'\t(segment (start {vx} {ey}) (end {pad.global_x} {pad.global_y}) (width 0.12) '
           f'(layer "F.Cu") (net "{raw}") (uuid "00000000-0000-0000-0000-00000000c0df"))\n')
    cut = text.rindex(')')
    with tempfile.TemporaryDirectory() as td:
        path = os.path.join(td, 'rim.kicad_pcb')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(text[:cut] + add + text[cut:])
        pcb = parse_kicad_pcb(path)
    via = next(v for v in pcb.vias if v.uuid.endswith('c0de'))
    got = pcb.nets[via.net_id].name if via.net_id in pcb.nets else via.net_id
    print(f'via ({vx}, {vy}) net {pcb_name!r}, track end ({vx}, {ey}) on the rim; parsed as {got!r}')
    if via.net_id != pad.net_id:
        print('FAIL: a via whose own track ends on its rim was re-netted to the fill net')
        return 1
    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
